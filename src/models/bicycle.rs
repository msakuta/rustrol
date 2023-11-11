mod avoidance;
mod path_utils;

use std::{collections::VecDeque, error::Error};

use rustograd::{Tape, TapeTerm};

use crate::{
    error::GradDoesNotExist,
    ops::{ClampOp, CosOp, SinOp, TanOp},
    vec2::Vec2,
};

pub(crate) use self::{avoidance::AvoidanceMode, path_utils::interpolate_path};
use self::{
    avoidance::{avoidance_search, AgentState, SearchEnv, SearchState},
    path_utils::find_closest_node,
};

pub(crate) const MAX_THRUST: f64 = 0.5;
pub(crate) const MAX_STEERING: f64 = std::f64::consts::PI / 4.;
pub(crate) const STEERING_SPEED: f64 = std::f64::consts::PI * 0.01;
const WHEEL_BASE: f64 = 4.;
const RATE: f64 = 1e-4;
const TARGET_SPEED: f64 = 1.;
const CIRCLE_RADIUS: f64 = 50.;
const SINE_PERIOD: f64 = 80.;
const SINE_AMPLITUDE: f64 = 10.;
const CRANK_PERIOD4: f64 = 20.;
const CRANK_PERIOD: f64 = CRANK_PERIOD4 * 4.;

pub struct BicycleParams {
    pub rate: f64,
    pub optim_iter: usize,
    pub max_iter: usize,
    pub prediction_states: usize,
    pub path_shape: BicyclePath,
    pub path_params: PathParams,
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

impl BicycleParams {
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
}

impl Default for BicycleParams {
    fn default() -> Self {
        let path_shape = BicyclePath::Crank;
        let path_params = PathParams::default();
        Self {
            rate: RATE,
            optim_iter: 50,
            max_iter: 200,
            prediction_states: 15,
            path_shape,
            path_params,
        }
    }
}

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
                let agent = AgentState::new(0., 0., 0.);
                let goal = AgentState::new(40., 40., 0.);
                avoidance_search(
                    params.path_params.avoidance,
                    &mut self.search_state,
                    &mut self.env,
                    &agent,
                    &goal,
                    params,
                    &|s| Self::collision_check(&self.obstacles, s),
                );
                self.prev_path_node = 0.;
                return;
            }
            _ => {}
        }

        self.path = BicycleParams::gen_path(
            params.path_shape,
            &params.path_params,
            params.max_iter + params.prediction_states,
        );
        self.prev_path_node = 0.;
    }

    pub(crate) fn update_path(&mut self, bicycle: &Bicycle, params: &BicycleParams) {
        if matches!(params.path_shape, BicyclePath::PathSearch) {
            let goal = AgentState::new(40., 40., 0.);
            let found_path = avoidance_search(
                params.path_params.avoidance,
                &mut self.search_state,
                &mut self.env,
                &bicycle.into(),
                &goal,
                params,
                &|s| Self::collision_check(&self.obstacles, s),
            );

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
            path: BicycleParams::gen_path(path_shape, &path_params, 250),
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

pub struct Obstacle {
    pub min: Vec2<f64>,
    pub max: Vec2<f64>,
}

pub struct PathParams {
    pub target_speed: f64,
    pub circle_radius: f64,
    pub sine_period: f64,
    pub sine_amplitude: f64,
    pub crank_period: f64,
    pub path_waypoints: Vec<Vec2<f64>>,
    pub avoidance: AvoidanceMode,
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
        }
    }
}

pub(crate) struct Bicycle {
    pub pos: Vec2<f64>,
    pub heading: f64,
    pub steering: f64,
    pub wheel_base: f64,
    pub pos_history: VecDeque<Vec2<f64>>,
    pub predictions: Vec<Vec2<f64>>,
}

impl Bicycle {
    const MAX_HISTORY: usize = 1000;
    pub fn new() -> Self {
        Self {
            pos: Vec2::zero(),
            heading: 0.,
            steering: 0.,
            wheel_base: WHEEL_BASE,
            pos_history: VecDeque::new(),
            predictions: vec![],
        }
    }

    pub fn append_history(&mut self) {
        if self
            .pos_history
            .back()
            .map(|hist| *hist != self.pos)
            .unwrap_or(true)
        {
            self.pos_history.push_back(self.pos);
        }
        if Self::MAX_HISTORY < self.pos_history.len() {
            self.pos_history.pop_front();
        }
    }
}

pub(crate) fn bicycle_simulate_step(
    bicycle: &mut Bicycle,
    h_thrust: f64,
    v_thrust: f64,
    playback_speed: f64,
    obstacles: &[Obstacle],
) {
    let steering_speed = playback_speed * STEERING_SPEED;
    bicycle.steering = if (bicycle.steering - h_thrust).abs() < steering_speed {
        h_thrust
    } else if bicycle.steering < h_thrust {
        bicycle.steering + steering_speed
    } else {
        bicycle.steering - steering_speed
    };
    let direction = Vec2::new(bicycle.heading.cos(), bicycle.heading.sin());
    let newpos = bicycle.pos + direction * v_thrust * playback_speed;
    let hit_obs = obstacles.iter().find(|obs| {
        obs.min.x < newpos.x && newpos.x < obs.max.x && obs.min.y < newpos.y && newpos.y < obs.max.y
    });
    if hit_obs.is_none() {
        bicycle.pos = newpos;
    }

    let theta_dot = v_thrust * bicycle.steering.tan() / bicycle.wheel_base;
    bicycle.heading += theta_dot * playback_speed;
}

pub struct BicycleResultState {
    pub pos: Vec2<f64>,
    pub heading: f64,
    pub steering: f64,
    pub predictions: Vec<Vec2<f64>>,
    pub closest_path_node: f64,
}

/// A whole simulation result, including predictions and actual responses
#[derive(Default)]
pub struct BicycleResult {
    pub bicycle_states: Vec<BicycleResultState>,
}

pub(crate) fn simulate_bicycle(
    pos: Vec2<f64>,
    params: &BicycleParams,
    path: &[Vec2<f64>],
    obstacles: &[Obstacle],
) -> Result<BicycleResult, Box<dyn Error>> {
    let tape = Tape::new();
    let model = get_model(&tape, pos, params, path);

    if let Ok(f) = std::fs::File::create("bicycle_graph.dot") {
        model.loss.eval();
        model.loss.backprop().unwrap(); // To color
        let x2 = model.predictions.first().unwrap().pos;
        x2.x.dot_builder()
            .vertical(true)
            .output_term(model.loss, "loss")
            .dot(&mut std::io::BufWriter::new(f))
            .unwrap();
    }

    let mut prev_path_node = 0.;
    let bicycle_states = (0..params.max_iter)
        .map(|t| -> Result<BicycleResultState, Box<dyn Error>> {
            let (h_thrust, v_thrust, closest_path_node) =
                optimize(&model, prev_path_node, params, path)?;
            prev_path_node = closest_path_node;
            // tape.dump_nodes();
            let (pos, heading) = simulate_step(&model, t, h_thrust, v_thrust, 1., obstacles);
            let first = model.predictions.first().unwrap();
            Ok(BicycleResultState {
                pos,
                heading,
                steering: first.steering.eval_noclear(),
                predictions: model
                    .predictions
                    .iter()
                    .map(|b1| b1.pos.map(|x| x.eval_noclear()))
                    .collect(),
                closest_path_node,
            })
        })
        .collect::<Result<Vec<_>, _>>()?;

    Ok(BicycleResult { bicycle_states })
}

pub(crate) fn control_bicycle(
    bicycle: &Bicycle,
    nav: &BicycleNavigation,
    params: &BicycleParams,
    playback_speed: f64,
    obstacles: &[Obstacle],
) -> Result<BicycleResultState, Box<dyn Error>> {
    let tape = Tape::new();
    let model = get_model(&tape, bicycle.pos, params, &nav.path);

    let Some(first) = model.predictions.first() else {
        return Err("Model does not have any predictions".into());
    };

    first.heading.set(bicycle.heading).unwrap();
    first.steering.set(bicycle.steering).unwrap();

    let (h_thrust, v_thrust, closest_path_node) =
        optimize(&model, nav.prev_path_node, params, &nav.path)?;
    let (_pos, heading) = simulate_step(&model, 0, h_thrust, v_thrust, playback_speed, obstacles);
    tape.clear();
    let state = BicycleResultState {
        pos: first.pos.map(|v| v.eval_noclear()),
        heading,
        steering: first.steering.eval_noclear(),
        predictions: model
            .predictions
            .iter()
            .map(|b1| b1.pos.map(|x| x.eval_noclear()))
            .collect(),
        closest_path_node,
    };

    Ok(state)
}

fn optimize(
    model: &Model,
    prev_path_node: f64,
    params: &BicycleParams,
    path: &[Vec2<f64>],
) -> Result<(f64, f64, f64), Box<dyn Error>> {
    if model.predictions.len() <= 2 {
        return Err("Predictions need to be more than 2".into());
    }
    let prev_path_node = prev_path_node as usize;
    let closest_path_s = if prev_path_node < path.len() {
        let pos = model
            .predictions
            .first()
            .unwrap()
            .pos
            .map(|x| x.eval_noclear());
        find_closest_node(&path[prev_path_node..], pos) + prev_path_node as f64
    } else {
        (path.len() - 1) as f64
    };

    for (i, state) in model.predictions.iter().enumerate() {
        if let Some(target) = interpolate_path(path, closest_path_s + i as f64) {
            state.target_pos.x.set(target.x)?;
            state.target_pos.y.set(target.y)?;
        }
    }

    for _ in 0..params.optim_iter {
        model.loss.eval();
        model.loss.backprop().unwrap();
        for hist in model.predictions.iter().take(model.predictions.len() - 2) {
            let d_h_thrust = try_grad!(hist.h_thrust);
            let d_v_thrust = try_grad!(hist.v_thrust);
            let h_thrust = (hist.h_thrust.data().unwrap() - d_h_thrust * params.rate)
                .clamp(-STEERING_SPEED, STEERING_SPEED);
            let v_thrust = (hist.v_thrust.data().unwrap() - d_v_thrust * params.rate)
                .clamp(0., params.path_params.target_speed);
            hist.h_thrust.set(h_thrust).unwrap();
            hist.v_thrust.set(v_thrust).unwrap();
        }
    }

    let h_thrust = model.predictions.first().unwrap().h_thrust.eval_noclear();
    let v_thrust = model.predictions.first().unwrap().v_thrust.eval_noclear();

    Ok((h_thrust, v_thrust, closest_path_s))
}

fn simulate_step(
    model: &Model,
    _t: usize,
    h_thrust: f64,
    v_thrust: f64,
    delta_time: f64,
    obstacles: &[Obstacle],
) -> (Vec2<f64>, f64) {
    let bicycle = model.predictions.first().unwrap();
    let heading = bicycle.heading.data().unwrap();

    let steering = (bicycle.steering.data().unwrap() + h_thrust * delta_time)
        .clamp(-MAX_STEERING, MAX_STEERING);
    let theta_dot = v_thrust * steering.tan() / bicycle.wheel_base.data().unwrap();
    let next_heading = heading + theta_dot * delta_time;
    let direction = Vec2::new(heading.cos(), heading.sin());
    let oldpos = bicycle.pos.map(|x| x.data().unwrap());
    let newpos = oldpos + direction * v_thrust * delta_time;
    let hit_obs = obstacles.iter().find(|obs| {
        obs.min.x < newpos.x && newpos.x < obs.max.x && obs.min.y < newpos.y && newpos.y < obs.max.y
    });
    if hit_obs.is_none() {
        bicycle.pos.x.set(newpos.x).unwrap();
        bicycle.pos.y.set(newpos.y).unwrap();
    }
    bicycle.heading.set(next_heading).unwrap();
    bicycle.steering.set(steering).unwrap();

    (oldpos, next_heading)
}

#[derive(Clone, Copy)]
struct BicycleTape<'a> {
    h_thrust: TapeTerm<'a>,
    v_thrust: TapeTerm<'a>,
    pos: Vec2<TapeTerm<'a>>,
    // velo: Vec2<TapeTerm<'a>>,
    heading: TapeTerm<'a>,
    steering: TapeTerm<'a>,
    target_pos: Vec2<TapeTerm<'a>>,
    wheel_base: TapeTerm<'a>,
    // accel: Vec2<TapeTerm<'a>>,
}

impl<'a> BicycleTape<'a> {
    fn new(tape: &'a Tape, initial_pos: Vec2<f64>) -> Self {
        Self {
            h_thrust: tape.term("h_thrust", 0.0),
            v_thrust: tape.term("v_thrust", MAX_THRUST),
            pos: Vec2 {
                x: tape.term("x1", initial_pos.x),
                y: tape.term("y1", initial_pos.y),
            },
            // velo: Vec2 {
            //     x: tape.term("vx1", 0.),
            //     y: tape.term("vy1", -0.01),
            // },
            heading: tape.term("heading", 0.),
            steering: tape.term("steering", 0.1),
            wheel_base: tape.term("wheel_base", WHEEL_BASE),
            target_pos: Vec2 {
                x: tape.term("tx", 0.),
                y: tape.term("ty", -0.01),
            },
            // accel: Vec2 {
            //     x: tape.term("ax1", 0.),
            //     y: tape.term("ay1", 0.),
            // },
        }
    }

    fn simulate_model(
        &mut self,
        tape: &'a Tape,
        // c: &Constants<'a>,
        hist: &mut Vec<Self>,
        target: Vec2<f64>,
    ) {
        // let thrust_vec = Vec2 {
        //     x: -self.heading.apply("sin", |x| x.sin(), |x| x.cos()) * self.v_thrust,
        //     y: self.heading.apply("cos", |x| x.cos(), |x| -x.sin()) * self.v_thrust,
        // };
        // let steer_diff = self.steering - self.h_thrust;
        // let steer_diff2 = steer_diff * steer_diff;

        self.target_pos = Vec2 {
            x: tape.term(format!("target_x{}", hist.len()), target.x),
            y: tape.term(format!("target_y{}", hist.len()), target.y),
        };

        self.steering = (self.steering + self.h_thrust).apply_t(Box::new(ClampOp {
            min: -MAX_STEERING,
            max: MAX_STEERING,
        }));

        let heading = Vec2::new(
            self.heading.apply_t(Box::new(CosOp)),
            self.heading.apply_t(Box::new(SinOp)),
        );
        self.pos = self.pos + heading * self.v_thrust;

        let theta_dot = self.v_thrust * self.steering.apply_t(Box::new(TanOp)) / self.wheel_base;
        self.heading = self.heading + theta_dot;

        // self.velo = self.velo + self.accel;
        self.h_thrust = tape.term(format!("h_thrust{}", hist.len()), 0.);
        self.v_thrust = tape.term(format!("v_thrust{}", hist.len()), MAX_THRUST);
        hist.push(*self);
    }
}

/// A model for the simulation state
struct Model<'a> {
    predictions: Vec<BicycleTape<'a>>,
    loss: TapeTerm<'a>,
}

fn get_model<'a>(
    tape: &'a Tape<f64>,
    initial_pos: Vec2<f64>,
    params: &BicycleParams,
    path: &[Vec2<f64>],
) -> Model<'a> {
    let bicycle = BicycleTape::new(tape, initial_pos);

    bicycle
        .v_thrust
        .set(params.path_params.target_speed * 0.5)
        .unwrap();

    // let heading_weight = tape.term("heading_weight", HEADING_WEIGHT);

    let mut bicycle1 = bicycle;
    let mut hist1 = vec![bicycle1];
    for (_i, pos) in path.iter().enumerate().take(params.prediction_states) {
        bicycle1.simulate_model(tape, &mut hist1, *pos);
    }

    let loss = hist1
        .iter()
        .last()
        .map(|bicycle| {
            let diff = bicycle.target_pos - bicycle.pos;
            diff.x * diff.x + diff.y * diff.y
        })
        .unwrap();

    Model {
        predictions: hist1,
        loss,
    }
}
