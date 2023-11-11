// mod render;
mod render_search_state;
pub mod sampler;
mod search;

use std::collections::{HashMap, HashSet};

use cgmath::Vector2;

// pub(crate) use self::render::AvoidanceRenderParams;
use self::{
    sampler::{ForwardKinematicSampler, RrtStarSampler, SpaceSampler, StateSampler},
    search::{can_connect_goal, insert_to_grid_map, search, to_cell},
};
use super::{Bicycle, PathParams, STEERING_SPEED, WHEEL_BASE};
use crate::{
    interpolation::{lerp, AsPoint, LerpPoint},
    vec2::Vec2, // collision::{CollisionShape, Obb},
    // entity::Entity,
    xor128::Xor128,
};

const SEARCH_WIDTH: f64 = 100.;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AvoidanceMode {
    Kinematic,
    Rrt,
    RrtStar,
}

#[derive(Clone, Copy, Debug)]
pub struct AgentState {
    pub x: f64,
    pub y: f64,
    pub heading: f64,
    pub steering: f64,
}

impl AgentState {
    pub fn new(x: f64, y: f64, heading: f64) -> Self {
        Self {
            x,
            y,
            heading,
            steering: 0.,
        }
    }

    // pub fn collision_shape(&self, class: AgentClass) -> CollisionShape {
    //     let shape = class.shape();
    //     CollisionShape::BBox(Obb {
    //         center: Vector2::new(self.x, self.y),
    //         xs: shape.0,
    //         ys: shape.1,
    //         orient: self.heading,
    //     })
    // }

    // pub(crate) fn with_orient(&self, orient: f64) -> Self {
    //     let mut copy = *self;
    //     copy.heading = orient;
    //     copy
    // }

    pub(crate) fn simulate_step(
        &mut self,
        h_thrust: f64,
        v_thrust: f64,
        delta_time: f64,
        wheel_base: f64,
    ) {
        let max_steering = delta_time * STEERING_SPEED;
        self.steering = if (self.steering - h_thrust).abs() < max_steering {
            h_thrust
        } else if self.steering < h_thrust {
            self.steering + max_steering
        } else {
            self.steering - max_steering
        };
        let direction = Vec2::new(self.heading.cos(), self.heading.sin());
        let newpos = Vec2::new(self.x, self.y) + direction * v_thrust * delta_time;
        self.x = newpos.x;
        self.y = newpos.y;

        let theta_dot = v_thrust * self.steering.tan() / wheel_base;
        self.heading += theta_dot * delta_time;
    }
}

impl From<AgentState> for [f64; 2] {
    fn from(s: AgentState) -> Self {
        [s.x, s.y]
    }
}

impl From<AgentState> for Vector2<f64> {
    fn from(s: AgentState) -> Self {
        Self::new(s.x, s.y)
    }
}

impl From<AgentState> for Vec2<f64> {
    fn from(s: AgentState) -> Self {
        Self::new(s.x, s.y)
    }
}

impl From<&Bicycle> for AgentState {
    fn from(value: &Bicycle) -> Self {
        Self::new(value.pos.x, value.pos.y, value.heading)
    }
}

impl AsPoint for AgentState {
    fn as_point(&self) -> [f64; 2] {
        [self.x, self.y]
    }
}

impl LerpPoint for AgentState {
    fn lerp(&self, other: &Self, f: f64) -> Self {
        let p = lerp(&self.as_point(), &other.as_point(), f);
        Self {
            x: p[0],
            y: p[1],
            heading: self.heading,
            steering: self.steering * (1. - f) + other.steering * f,
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct PathNode {
    pub x: f64,
    pub y: f64,
    pub _backward: bool,
}

impl From<[f64; 2]> for PathNode {
    fn from(a: [f64; 2]) -> Self {
        Self {
            x: a[0],
            y: a[1],
            _backward: false,
        }
    }
}

impl From<PathNode> for [f64; 2] {
    fn from(a: PathNode) -> Self {
        [a.x, a.y]
    }
}

impl From<&SearchNode> for PathNode {
    fn from(node: &SearchNode) -> Self {
        PathNode {
            x: node.state.x,
            y: node.state.y,
            _backward: node.speed < 0.,
        }
    }
}

impl From<PathNode> for Vector2<f64> {
    fn from(node: PathNode) -> Self {
        Self::new(node.x, node.y)
    }
}

#[derive(Clone, Debug)]
pub struct SearchNode {
    pub state: AgentState,
    pub cost: f64,
    pub speed: f64,
    pub id: usize,
    /// The maximum recursion level to determine collision. Used for debugging
    pub max_level: usize,
    pub from: Option<usize>,
    pub cycle: bool,
    to: Vec<usize>,
    pub pruned: bool,
    pub blocked: bool,
}

impl SearchNode {
    pub(crate) fn new(state: AgentState, cost: f64, speed: f64) -> Self {
        Self {
            state,
            cost,
            speed,
            id: 0,
            max_level: 0,
            from: None,
            to: vec![],
            pruned: false,
            blocked: false,
            cycle: false,
        }
    }

    fn is_passable(&self) -> bool {
        !self.blocked && !self.pruned
    }
}

pub const DIST_RADIUS: f64 = 0.5 * WHEEL_BASE;
const DIST_THRESHOLD: f64 = DIST_RADIUS * DIST_RADIUS;

fn compare_state(s1: &AgentState, s2: &AgentState) -> bool {
    let delta_angle = wrap_angle(s1.heading - s2.heading);
    // println!("compareState deltaAngle: {}", deltaAngle);
    compare_distance(s1, s2, DIST_THRESHOLD) && delta_angle.abs() < std::f64::consts::PI / 6.
}

fn compare_distance(s1: &AgentState, s2: &AgentState, threshold: f64) -> bool {
    let delta_x = s1.x - s2.x;
    let delta_y = s1.y - s2.y;
    delta_x * delta_x + delta_y * delta_y < threshold
}

pub const CELL_SIZE: f64 = WHEEL_BASE;
const MAX_CELL_COUNT: usize = 10;

/// We use a grid of cells with fixed sizes to query nodes in a search tree.
/// The benefit of grid over RTree is that RTree requires O(n log n) to build
/// the index, while grid is just O(n). We need to insert as many times as
/// query, so the insertion time needs to be small.
type GridMap = HashMap<[i32; 2], HashSet<usize>>;

#[derive(Debug)]
pub struct SearchState {
    search_tree: Vec<SearchNode>,
    start_set: HashSet<usize>,
    goal: AgentState,
    last_solution: Option<usize>,
    pub(super) found_path: Option<Vec<usize>>,
    pub(super) grid_map: GridMap,
}

impl SearchState {
    /// Return the path if it was found, resampled in a given interval
    pub fn get_path(&self, interval: f64) -> Option<Vec<Vec2<f64>>> {
        self.found_path.as_ref().map(|path| {
            let mut ret = vec![];
            for (&next, &prev) in path.iter().zip(path.iter().skip(1)).rev() {
                let prev: Vec2<f64> = self.search_tree[prev].state.into();
                let next: Vec2<f64> = self.search_tree[next].state.into();
                let segment_delta = next - prev;
                let segment_len = segment_delta.length();
                let segment_splits = (segment_len / interval) as i32;
                for i in 0..segment_splits {
                    let fr = i as f64 / segment_splits as f64;
                    ret.push(prev + segment_delta * fr);
                }
            }
            ret
        })
    }
}

pub(super) struct AvoidanceParams<'a> {
    pub avoidance_mode: AvoidanceMode,
    pub search_state: &'a mut Option<SearchState>,
    pub env: &'a mut SearchEnv,
    pub agent: &'a AgentState,
    pub goal: &'a AgentState,
    pub path_params: &'a PathParams,
}

/// RRT* search
///
/// Returns true if the path is found
pub(super) fn avoidance_search(
    params: &mut AvoidanceParams,
    collision_callback: &impl Fn(AgentState) -> bool,
) -> bool {
    match params.avoidance_mode {
        AvoidanceMode::Kinematic => {
            avoidance_search_gen::<ForwardKinematicSampler>(params, collision_callback)
        }
        AvoidanceMode::Rrt => avoidance_search_gen::<SpaceSampler>(params, collision_callback),
        AvoidanceMode::RrtStar => {
            avoidance_search_gen::<RrtStarSampler>(params, collision_callback)
        }
    }
}

/// Templatized logic for searching avoidance path. The type argument speicfy how to
/// sample a new node.
pub(super) fn avoidance_search_gen<Sampler: StateSampler>(
    params: &mut AvoidanceParams,
    collision_callback: &impl Fn(AgentState) -> bool,
) -> bool {
    // println!(
    //     "search invoked: state: {} goal: {:?}",
    //     self.search_state.is_some(),
    //     self.goal
    // );
    let agent = *params.agent;
    let goal = *params.goal;
    let env = &mut params.env;

    // Restart search if the target has diverged
    if let Some(ss) = params.search_state {
        if !compare_state(&ss.goal, &goal) {
            *params.search_state = None;
        }
    }

    let mut searched_path = false;
    if let Some(mut sstate) = params.search_state.take() {
        if let Some(goal) = sstate.last_solution {
            if let Some(path) = can_connect_goal(&sstate.start_set, &sstate.search_tree, goal) {
                // Restore previous solution
                sstate.found_path = Some(path);
            }
        }
        if compare_distance(&goal, &sstate.goal, DIST_THRESHOLD) {
            // for root in &search_state.searchTree {
            //     enumTree(root, &mut nodes);
            // }

            let nodes = &mut sstate.search_tree;

            // println!(
            //     "Using existing tree with {} nodes start from {:?}",
            //     nodes.len(),
            //     search_state.start
            // );

            if 0 < nodes.len() && nodes.len() < 10000 {
                // Descending the tree is not a good way to sample a random node in a tree, since
                // the chances are much higher on shallow nodes. We want to give chances uniformly
                // among all nodes in the tree, so we randomly pick one from a linear list of all nodes.
                let path = search::<Sampler>(
                    &sstate.start_set,
                    &goal,
                    env,
                    nodes,
                    &mut sstate.grid_map,
                    collision_callback,
                    params.path_params.expand_states,
                );

                env.tree_size += 1;

                if let Some(path) = path {
                    // println!("Materialized found path: {:?}", self.path);
                    sstate.last_solution = path.last().copied();
                    sstate.found_path = Some(path);
                    *params.search_state = Some(sstate);
                    return true;
                }
            }

            // let treeSize = env.tree_size;
            sstate.goal = goal;
            *params.search_state = Some(sstate);
            searched_path = true;
        }
    }

    if !searched_path {
        // println!("Rebuilding tree with {} nodes should be 0", nodes.len());
        let mut nodes: Vec<SearchNode> = Sampler::initial_search(&agent, params.path_params);

        if !nodes.is_empty() {
            let root_set = (0..nodes.len()).collect();
            let mut grid_map = HashMap::new();
            for (i, node) in nodes.iter().enumerate() {
                insert_to_grid_map(&mut grid_map, to_cell(node.state), i);
            }
            println!(
                "avoidance {} nodes {} grid cells",
                nodes.len(),
                grid_map.len()
            );
            let found_path = search::<Sampler>(
                &root_set,
                &goal,
                env,
                &mut nodes,
                &mut grid_map,
                collision_callback,
                params.path_params.expand_states,
            );

            *params.search_state = Some(SearchState {
                search_tree: nodes,
                start_set: root_set,
                goal,
                last_solution: None,
                found_path,
                grid_map,
            });
            // else{
            //     *search_state = SearchState{
            //         searchTree: roots,
            //         treeSize: 0,
            //         start: State{x: this.x, y: this.y, heading: this.angle},
            //         goal: this.goal,
            //     };
            // }
            // println!("Search state: {search_state:?}");
            // self.search_state = Some(search_state);
        }

        // We add path nodes after connections are built, because path nodes may come from non-tree nodes and
        // path nodes do not need to be connected.
        // for node in &path {
        //     if *node < nodes.len() {
        //         let mut new_node = StateWithCost::new(
        //             State {
        //                 x: node[0],
        //                 y: node[1],
        //                 heading: node[2],
        //             },
        //             0.,
        //             0.,
        //             1.,
        //         );
        //         new_node.id = nodes.len();
        //         nodes.push(new_node);
        //     }
        // }
    }

    params
        .search_state
        .as_ref()
        .map(|ss| ss.found_path.is_some())
        .unwrap_or(false)
}

pub(super) struct SearchEnv {
    rng: Xor128,
    skipped_nodes: usize,
    wheel_base: f64,
    tree_size: usize,
    search_bounds: [f64; 4],
}

impl SearchEnv {
    pub(super) fn new(wheel_base: f64) -> Self {
        Self {
            rng: Xor128::new(233221),
            skipped_nodes: 0,
            wheel_base,
            tree_size: 0,
            search_bounds: [-SEARCH_WIDTH, -SEARCH_WIDTH, SEARCH_WIDTH, SEARCH_WIDTH],
        }
    }
}

pub(crate) fn wrap_angle(x: f64) -> f64 {
    use std::f64::consts::PI;
    const TWOPI: f64 = PI * 2.;
    // ((x + PI) - ((x + PI) / TWOPI).floor() * TWOPI) - PI
    x - (x + PI).div_euclid(TWOPI) * TWOPI
}
