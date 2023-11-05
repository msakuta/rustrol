use std::{collections::VecDeque, error::Error};

use rustograd::{Tape, TapeTerm};

use crate::{
    error::GradDoesNotExist,
    ops::{ClampOp, CosOp, SinOp, TanOp},
    vec2::Vec2,
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
    pub path: Vec<Vec2<f64>>,
    pub path_shape: BicyclePath,
    pub path_params: PathParams,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum BicyclePath {
    Circle,
    Sine,
    Crank,
    Point,
}

impl BicycleParams {
    fn gen_path(path: BicyclePath, path_params: &PathParams, len: usize) -> Vec<Vec2<f64>> {
        use std::f64::consts::PI;
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
                BicyclePath::Point => {
                    unreachable!()
                }
            })
            .collect()
    }

    pub fn reset_path(&mut self) {
        if let BicyclePath::Point = self.path_shape {
            if let Some(seg) = self.path_params.line_segment {
                let delta = seg[1] - seg[0];
                let length = delta.length();
                let len = length as usize / 2;
                self.path = (0..len)
                    .map(|i| {
                        let f = i as f64 / len as f64;
                        seg[1] * f + seg[0] * (1. - f)
                    })
                    .collect();
            }
            return;
        }
        self.path = Self::gen_path(
            self.path_shape,
            &self.path_params,
            self.max_iter + self.prediction_states,
        );
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
            path: Self::gen_path(path_shape, &path_params, 250),
            path_params,
        }
    }
}

pub struct PathParams {
    pub target_speed: f64,
    pub circle_radius: f64,
    pub sine_period: f64,
    pub sine_amplitude: f64,
    pub crank_period: f64,
    pub line_segment: Option<[Vec2<f64>; 2]>,
}

impl Default for PathParams {
    fn default() -> Self {
        Self {
            target_speed: TARGET_SPEED,
            circle_radius: CIRCLE_RADIUS,
            sine_period: SINE_PERIOD,
            sine_amplitude: SINE_AMPLITUDE,
            crank_period: CRANK_PERIOD,
            line_segment: None,
        }
    }
}

pub(crate) struct Bicycle {
    pub pos: Vec2<f64>,
    pub heading: f64,
    pub steering: f64,
    pub wheel_base: f64,
    pub pos_history: VecDeque<Vec2<f64>>,
    pub prev_path_node: usize,
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
            prev_path_node: 0,
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
    bicycle.pos += direction * v_thrust * playback_speed;

    let theta_dot = v_thrust * bicycle.steering.tan() / bicycle.wheel_base;
    bicycle.heading += theta_dot * playback_speed;
}

pub struct BicycleResultState {
    pub pos: Vec2<f64>,
    pub heading: f64,
    pub steering: f64,
    pub predictions: Vec<Vec2<f64>>,
    pub closest_path_node: usize,
}

#[derive(Default)]
pub struct BicycleResult {
    pub bicycle_states: Vec<BicycleResultState>,
}

pub(crate) fn simulate_bicycle(
    pos: Vec2<f64>,
    params: &BicycleParams,
) -> Result<BicycleResult, Box<dyn Error>> {
    let tape = Tape::new();
    let model = get_model(&tape, pos, params);

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

    let mut prev_path_node = 0;
    let bicycle_states = (0..params.max_iter)
        .map(|t| -> Result<BicycleResultState, Box<dyn Error>> {
            let (h_thrust, v_thrust, closest_path_node) = optimize(&model, prev_path_node, params)?;
            prev_path_node = closest_path_node;
            // tape.dump_nodes();
            let (pos, heading) = simulate_step(&model, t, h_thrust, v_thrust);
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
    pos: Vec2<f64>,
    params: &BicycleParams,
    prev_path_node: usize,
) -> Result<BicycleResultState, Box<dyn Error>> {
    let tape = Tape::new();
    let model = get_model(&tape, pos, params);

    let (h_thrust, v_thrust, closest_path_node) = optimize(&model, prev_path_node, params)?;
    let (_pos, heading) = simulate_step(&model, 0, h_thrust, v_thrust);
    let first = model.predictions.first().unwrap();
    println!(
        "Thrust: {} {} {}",
        h_thrust,
        first.heading.eval_noclear(),
        heading
    );
    let state = BicycleResultState {
        pos: first.pos.map(|v| v.eval_noclear()),
        heading,
        steering: first.steering.eval_noclear(),
        predictions: model
            .predictions
            .iter()
            .map(|b1| b1.pos.map(|x| x.data().unwrap()))
            .collect(),
        closest_path_node,
    };

    Ok(state)
}

fn optimize(
    model: &Model,
    prev_path_node: usize,
    params: &BicycleParams,
) -> Result<(f64, f64, usize), Box<dyn Error>> {
    let closest_path_node = model
        .predictions
        .first()
        .map(|init| {
            let pos = init.pos.map(|x| x.eval_noclear());
            params.path[prev_path_node..]
                .iter()
                .enumerate()
                .fold(None, |acc: Option<(usize, f64)>, cur| {
                    let dist2 = (pos - *cur.1).length2();
                    if let Some(acc) = acc {
                        if dist2 < acc.1 {
                            Some((cur.0, dist2))
                        } else {
                            Some(acc)
                        }
                    } else {
                        Some((cur.0, dist2))
                    }
                })
                .map(|(i, _)| i)
                .unwrap_or(0)
                + prev_path_node
        })
        .unwrap_or(0);

    for (i, state) in model.predictions.iter().enumerate() {
        if let Some(target) = params.path.get(closest_path_node + i) {
            state.target_pos.x.set(target.x)?;
            state.target_pos.y.set(target.y)?;
        }
    }

    let mut d_h_thrust = 0.;
    let mut d_v_thrust = 0.;
    let mut v_thrust = 0.;
    let mut h_thrust = 0.;

    for _ in 0..params.optim_iter {
        model.loss.eval();
        model.loss.backprop().unwrap();
        for hist in model.predictions.iter().take(model.predictions.len() - 2) {
            d_h_thrust = try_grad!(hist.h_thrust);
            d_v_thrust = try_grad!(hist.v_thrust);
            h_thrust = (hist.h_thrust.data().unwrap() - d_h_thrust * params.rate)
                .clamp(-STEERING_SPEED, STEERING_SPEED);
            v_thrust =
                (hist.v_thrust.data().unwrap() - d_v_thrust * params.rate).clamp(0., MAX_THRUST);
            hist.h_thrust.set(h_thrust).unwrap();
            hist.v_thrust.set(v_thrust).unwrap();
        }
    }

    let loss_val = model.loss.eval_noclear();
    // println!(
    //     "h_thrust: {}, v_thrust: {}, d_h_thrust: {}, d_v_thrust: {}, heading: {}, loss: {}",
    //     h_thrust,
    //     v_thrust,
    //     d_h_thrust,
    //     d_v_thrust,
    //     model.predictions.first().unwrap().heading.eval_noclear(),
    //     loss_val
    // );

    let h_thrust = model.predictions.first().unwrap().h_thrust.eval_noclear();
    let v_thrust = model.predictions.first().unwrap().v_thrust.eval_noclear();

    Ok((h_thrust, v_thrust, closest_path_node))
}

fn simulate_step(model: &Model, _t: usize, h_thrust: f64, v_thrust: f64) -> (Vec2<f64>, f64) {
    let bicycle = model.predictions.first().unwrap();
    let heading = bicycle.heading.data().unwrap();

    let steering = (bicycle.steering.data().unwrap() + h_thrust).clamp(-MAX_STEERING, MAX_STEERING);
    let theta_dot = v_thrust * steering.tan() / bicycle.wheel_base.data().unwrap();
    let next_heading = heading + theta_dot;
    let direction = Vec2::new(heading.cos(), heading.sin());
    let oldpos = bicycle.pos.map(|x| x.data().unwrap());
    let newpos = oldpos + direction * v_thrust;
    bicycle.pos.x.set(newpos.x).unwrap();
    bicycle.pos.y.set(newpos.y).unwrap();
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
            h_thrust: tape.term("h_thrust", 0.01),
            v_thrust: tape.term("v_thrust", 0.01),
            pos: Vec2 {
                x: tape.term("x1", initial_pos.x),
                y: tape.term("y1", initial_pos.y),
            },
            // velo: Vec2 {
            //     x: tape.term("vx1", 0.),
            //     y: tape.term("vy1", -0.01),
            // },
            heading: tape.term("heading", 0.1),
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
        self.v_thrust = tape.term(format!("v_thrust{}", hist.len()), 1.);
        hist.push(*self);
    }
}

/// A model for the simulation state
struct Model<'a> {
    predictions: Vec<BicycleTape<'a>>,
    loss: TapeTerm<'a>,
}

fn get_model<'a>(tape: &'a Tape<f64>, initial_pos: Vec2<f64>, params: &BicycleParams) -> Model<'a> {
    let bicycle = BicycleTape::new(tape, initial_pos);

    // let heading_weight = tape.term("heading_weight", HEADING_WEIGHT);

    let mut bicycle1 = bicycle;
    let mut hist1 = vec![bicycle1];
    for i in 0..params.prediction_states {
        bicycle1.simulate_model(tape, &mut hist1, params.path[i]);
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
