//! Search logic in path finding

use std::collections::HashSet;

use cgmath::{MetricSpace, Vector2};

use crate::{interpolation::interpolate, measure_time};

use super::{
    compare_distance, sampler::StateSampler, AgentState, GridMap, SearchEnv, SearchNode, CELL_SIZE,
    DIST_RADIUS, MAX_CELL_COUNT,
};

pub(super) fn to_cell(state: AgentState) -> [i32; 2] {
    [
        state.x.div_euclid(CELL_SIZE) as i32,
        state.y.div_euclid(CELL_SIZE) as i32,
    ]
}

pub(super) fn insert_to_grid_map(grid_map: &mut GridMap, idx: [i32; 2], value: usize) {
    let cell: &mut HashSet<usize> = grid_map.entry(idx).or_insert_with(|| HashSet::new());
    cell.insert(value);
}

pub(super) fn count_from_grid_map(grid_map: &mut GridMap, idx: [i32; 2]) -> usize {
    grid_map.get(&idx).map(|cell| cell.len()).unwrap_or(0)
}

pub(super) fn for_each_neighbor(
    grid_map: &GridMap,
    idx: [i32; 2],
    mut f: impl FnMut(usize) -> bool,
) {
    let [ix, iy] = idx;
    for jy in (iy - 1)..=(iy + 1) {
        for jx in (ix - 1)..=(ix + 1) {
            let Some(cell_nodes) = grid_map.get(&[jx, jy]) else {
                continue;
            };
            for &i in cell_nodes {
                if f(i) {
                    return;
                }
            }
        }
    }
}

pub(super) fn can_connect_goal(
    start_set: &HashSet<usize>,
    nodes: &[SearchNode],
    mut node: usize,
) -> Option<Vec<usize>> {
    let mut path = vec![];
    while let Some(next_node) = nodes[node].from {
        if !nodes[next_node].is_passable() {
            return None;
        }
        path.push(next_node);
        if start_set.contains(&next_node) {
            return Some(path);
        }
        node = next_node;
    }
    None
}

/// Check if the goal is close enough to the added node, and if it was, return a built path
fn check_goal(
    start_set: &HashSet<usize>,
    start: usize,
    goal: &AgentState,
    nodes: &mut [SearchNode],
) -> Option<Vec<usize>> {
    if !compare_distance(&nodes[start].state, &goal, DIST_RADIUS.powf(2.)) {
        return None;
    }
    let mut node = start;
    let mut path = vec![];
    let mut cycle_check = HashSet::new();
    while let Some(next_node) = nodes[node].from {
        if cycle_check.contains(&next_node) {
            println!("Cycle detected in check_goal! {cycle_check:?}");
            mark_cycles(nodes, start);
            return None;
        }
        cycle_check.insert(next_node);
        if !nodes[next_node].is_passable() {
            return None;
        }
        path.push(next_node);
        if start_set.contains(&next_node) {
            break;
        }
        node = next_node;
    }
    println!(
        "Found path to {goal:?}: {:?}",
        path.iter()
            .map(|node| nodes[*node].speed)
            .collect::<Vec<_>>()
    );
    return Some(path);
}

pub(super) fn mark_cycles(nodes: &mut [SearchNode], start: usize) {
    let mut node = start;
    let mut cycle_check = HashSet::new();
    while let Some(next_node) = nodes[node].from {
        if cycle_check.contains(&next_node) {
            return;
        }
        cycle_check.insert(next_node);
        nodes[node].cycle = true;
        node = next_node;
    }
}

/// Perform the search using sampling strategy given by `S`.
/// Returns path node ids or None if the path is not found yet.
pub(super) fn search<S: StateSampler>(
    start_set: &HashSet<usize>,
    goal: &AgentState,
    env: &mut SearchEnv,
    nodes: &mut Vec<SearchNode>,
    grid_map: &mut GridMap,
    collision_callback: &impl Fn(AgentState) -> bool,
    expand_states: usize,
) -> Option<Vec<usize>> {
    let mut sample_time = 0.;
    let mut merge_time = 0.;
    let mut rewire_time = 0.;
    'skip: for _i in 0..expand_states {
        let mut sampler = S::new(env);

        // println!(
        //     "Searching {} states from {start}/{}",
        //     env.expandStates,
        //     nodes.len()
        // );

        // let start_state = nodes[start].state;
        // let this_shape = this.get_shape();

        let collision_check = |start_state: AgentState,
                               next_state: AgentState,
                               _next_direction: f64,
                               _distance: f64,
                               _steer: f64|
         -> (bool, usize) {
            // const USE_SEPAX: bool = true;
            // const USE_STEER: bool = false;
            // let collision_checker = |state: AgentState| {
            // if Agent::collision_check(Some(this.id), state, this.class, env.entities, true) {
            //     return false;
            // }
            // !env.game.check_hit(
            //     &start_state
            //         .collision_shape(this.class)
            //         .with_position(state.as_point().into()),
            // )
            //     collision_callback(state)
            // };
            // if USE_SEPAX {
            //     let start_shape = start_state.collision_shape();
            //         (
            //             interpolate(start_state, next_state, DIST_RADIUS * 0.5, collision_callback),
            //             level,
            //         )
            //     }
            // } else if USE_STEER {
            //     (
            //         interpolate_steer(
            //             &start_state,
            //             steer,
            //             next_direction * distance,
            //             DIST_RADIUS,
            //             &collision_checker,
            //         ),
            //         0,
            //     )
            // } else {
            (
                interpolate(start_state, next_state, DIST_RADIUS, collision_callback),
                0,
            )
            // }
        };

        let found = 'found: {
            for _ in 0..10 {
                let (res, time) =
                    measure_time(|| sampler.sample(nodes, env, grid_map, collision_check));
                sample_time += time;
                let Some((start, node)) = res else { continue };
                let cell_count = count_from_grid_map(grid_map, to_cell(node.state));
                if MAX_CELL_COUNT < cell_count {
                    continue;
                }
                break 'found Some((start, node));
            }
            None
        };

        let Some((start, mut node)) = found else {
            continue;
        };

        let next_direction = node.speed.signum();
        let start_state = nodes[start].state;

        // let AgentState { x, y, heading } = start_state;
        let (merged, time) =
            measure_time(|| sampler.merge_same_nodes(&node, start, nodes, env, grid_map));
        merge_time += time;
        if merged {
            continue 'skip;
        }
        // println!("stepMove: {:?} -> {:?}", nodes[start], next);

        let distance = Vector2::from(node.state).distance(start_state.into());

        let (hit, level) = collision_check(
            start_state,
            node.state,
            next_direction,
            distance,
            node.state.steering,
        );

        if hit {
            // println!("Search hit something!, {nextDirection} * {distance}");
            continue;
        }

        node.from = Some(start);
        let new_node_id = nodes.len();
        nodes[start].to.push(new_node_id);
        node.id = new_node_id;
        node.max_level = level;

        insert_to_grid_map(grid_map, to_cell(node.state), new_node_id);

        let mut hist = vec![];
        for cell in grid_map.values() {
            let bin = cell.len();
            if hist.len() <= bin {
                hist.resize(bin + 1, 0);
            }
            hist[bin] += 1;
        }

        nodes.push(node);

        let (_, time) = measure_time(|| {
            sampler.rewire(nodes, new_node_id, start, collision_check, grid_map);
        });
        rewire_time += time;

        if let Some(path) = check_goal(start_set, new_node_id, &goal, nodes) {
            return Some(path);
        }

        // callback(start, node);
    }
    println!(
        "rewire: {:.6}, merge: {:.6}, sample: {:.6}",
        rewire_time, merge_time, sample_time
    );
    None
}
