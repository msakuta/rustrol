use crate::vec2::Vec2;

use super::{
    avoidance::{
        avoidance_search, AgentState, AvoidanceMode, AvoidanceParams, SearchEnv, SearchState,
        SEARCH_WIDTH,
    },
    Bicycle, BicycleParams, Obstacle, WHEEL_BASE,
};

const TARGET_SPEED: f64 = 1.;
const CIRCLE_RADIUS: f64 = 50.;
const SINE_PERIOD: f64 = 80.;
const SINE_AMPLITUDE: f64 = 10.;
const CRANK_PERIOD4: f64 = 20.;
const CRANK_PERIOD: f64 = CRANK_PERIOD4 * 4.;

/// Vehicle state regarding navigation.
pub(crate) struct BicycleNavigation {
    pub path: Vec<Vec2<f64>>,
    pub prev_path_node: f64,
    pub search_state: Option<SearchState>,
    pub obstacles: Vec<Obstacle>,
    env: SearchEnv,
}

impl BicycleNavigation {
    pub fn reset_path(&mut self, params: &BicycleParams) {
        self.search_state = None;
        match params.path_shape {
            BicyclePath::ClickedPoint => {
                let wps = &params.path_params.path_waypoints;
                self.path.clear();
                for (&prev, &next) in wps.iter().zip(wps.iter().skip(1)) {
                    let delta = next - prev;
                    let length = delta.length();
                    let len = length as usize;
                    self.path.extend((0..len).map(|i| {
                        let f = i as f64 / len as f64;
                        next * f + prev * (1. - f)
                    }));
                }
                return;
            }
            BicyclePath::PathSearch => {
                self.path.clear();
                let Some(goal) = params.path_params.search_goal else {
                    return;
                };
                let search_size = params.path_params.search_size;
                self.env.search_bounds = [-search_size, -search_size, search_size, search_size];
                let agent = AgentState::new(0., 0., 0.);
                let goal = AgentState::new(goal.x, goal.y, 0.);
                let mut a_params = AvoidanceParams {
                    avoidance_mode: params.path_params.avoidance,
                    search_state: &mut self.search_state,
                    env: &mut self.env,
                    goal: &goal,
                    agent: &agent,
                    path_params: &params.path_params,
                };
                avoidance_search(&mut a_params, &|s| {
                    Self::collision_check(&self.obstacles, s)
                });
                self.prev_path_node = 0.;
                return;
            }
            _ => {}
        }

        self.path = gen_path(
            params.path_shape,
            &params.path_params,
            params.max_iter + params.prediction_states,
        );
        self.prev_path_node = 0.;
    }

    pub(crate) fn update_path(&mut self, bicycle: &Bicycle, params: &BicycleParams) {
        if matches!(params.path_shape, BicyclePath::PathSearch) {
            let Some(goal) = params.path_params.search_goal else {
                return;
            };
            let goal = AgentState::new(goal.x, goal.y, 0.);
            let mut a_params = AvoidanceParams {
                avoidance_mode: params.path_params.avoidance,
                search_state: &mut self.search_state,
                env: &mut self.env,
                goal: &goal,
                agent: &bicycle.into(),
                path_params: &params.path_params,
            };
            let found_path = avoidance_search(&mut a_params, &|s| {
                Self::collision_check(&self.obstacles, s)
            });

            if found_path {
                if let Some(sstate) = &self.search_state {
                    if let Some(path) = sstate.get_path(params.path_params.target_speed) {
                        self.path = path;
                        self.prev_path_node = 0.;
                    }
                }
            }
        }
    }

    fn collision_check(obstacles: &[Obstacle], state: AgentState) -> bool {
        obstacles
            .iter()
            .find(|obs| {
                obs.min.x < state.x
                    && state.x < obs.max.x
                    && obs.min.y < state.y
                    && state.y < obs.max.y
            })
            .is_some()
    }
}

impl Default for BicycleNavigation {
    fn default() -> Self {
        let path_shape = BicyclePath::Crank;
        let path_params = PathParams::default();
        Self {
            path: gen_path(path_shape, &path_params, 250),
            prev_path_node: 0.,
            search_state: None,
            obstacles: vec![
                Obstacle {
                    min: Vec2::new(10., 10.),
                    max: Vec2::new(30., 30.),
                },
                Obstacle {
                    min: Vec2::new(10., -30.),
                    max: Vec2::new(30., -10.),
                },
            ],
            env: SearchEnv::new(WHEEL_BASE),
        }
    }
}

fn gen_path(path: BicyclePath, path_params: &PathParams, len: usize) -> Vec<Vec2<f64>> {
    use std::f64::consts::PI;
    if matches!(path, BicyclePath::DirectControl) {
        return vec![];
    }
    (0..len)
        .map(|t| match path {
            BicyclePath::Circle => {
                let period = 2. * PI * path_params.circle_radius / path_params.target_speed;
                let phase = t as f64 * 2. * PI / period;
                Vec2::new(
                    path_params.circle_radius * phase.sin(),
                    path_params.circle_radius * (1. - phase.cos()),
                )
            }
            BicyclePath::Sine => {
                let phase = t as f64 / path_params.sine_period * PI * 2.;
                Vec2::new(
                    t as f64 * path_params.target_speed,
                    path_params.sine_amplitude * phase.sin(),
                )
            }
            BicyclePath::Crank => {
                let period = path_params.crank_period;
                let speed = path_params.target_speed;
                let ft = t as f64 % period;
                let period4 = period / 4.;
                let x_ofs = (t as f64).div_euclid(period) * period4 * 2. * speed;
                if ft < period4 {
                    Vec2::new(x_ofs + ft * speed, 0.)
                } else if ft < period4 * 2. {
                    Vec2::new(x_ofs + period4 * speed, (ft - period4) * speed)
                } else if ft < period4 * 3. {
                    Vec2::new(
                        x_ofs + period4 * speed + (ft - period4 * 2.) * speed,
                        period4 * speed,
                    )
                } else {
                    Vec2::new(
                        x_ofs + period4 * speed * 2.,
                        period4 * speed - (ft - period4 * 3.) * speed,
                    )
                }
            }
            _ => {
                unreachable!()
            }
        })
        .collect()
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum BicyclePath {
    DirectControl,
    ClickedPoint,
    Circle,
    Sine,
    Crank,
    PathSearch,
}

pub struct PathParams {
    pub target_speed: f64,
    pub circle_radius: f64,
    pub sine_period: f64,
    pub sine_amplitude: f64,
    pub crank_period: f64,
    pub path_waypoints: Vec<Vec2<f64>>,
    pub avoidance: AvoidanceMode,
    pub expand_states: usize,
    pub search_size: f64,
    pub search_goal: Option<Vec2<f64>>,
}

impl Default for PathParams {
    fn default() -> Self {
        Self {
            target_speed: TARGET_SPEED,
            circle_radius: CIRCLE_RADIUS,
            sine_period: SINE_PERIOD,
            sine_amplitude: SINE_AMPLITUDE,
            crank_period: CRANK_PERIOD,
            path_waypoints: vec![],
            avoidance: AvoidanceMode::Kinematic,
            expand_states: 10,
            search_size: SEARCH_WIDTH,
            search_goal: None,
        }
    }
}
