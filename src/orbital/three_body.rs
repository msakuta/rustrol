use rustograd::{error::RustogradError, tape::TapeNode, Tape, TapeTerm};

use crate::{error::GradDoesNotExist, ops::MinOp, vec2::Vec2};

use super::{
    model_simulate_step, optimize, simulate_step, AbstractModelState, Body, BodyModel,
    BodyModelState, BodyState, Model, OrbitalBody, OrbitalParams, Vec2d, Vec2t, EARTH_POS, GM,
    GM_MOON, THRUST_ACCEL,
};

const INIT_MOON_POS: Vec2<f64> = Vec2 { x: -3., y: 0. };

#[derive(Clone, Copy)]
pub struct ThreeBodyParams {
    pub moon_pos: Vec2<f64>,
    pub moon_gm: f64,
    pub target_r: f64,
}

impl Default for ThreeBodyParams {
    fn default() -> Self {
        Self {
            moon_pos: INIT_MOON_POS,
            moon_gm: GM_MOON,
            target_r: 0.5,
        }
    }
}

#[derive(Clone, Copy)]
pub struct ThreeBodyState {
    pub satellite: OrbitalBody,
    pub moon: OrbitalBody,
}

pub const THREE_BODY_STATE: ThreeBodyState = ThreeBodyState {
    satellite: OrbitalBody {
        pos: Vec2 { x: 2., y: 0. },
        velo: Vec2 { x: 0., y: 0.12 },
    },
    moon: OrbitalBody {
        pos: Vec2 { x: 0., y: 2. },
        velo: Vec2 { x: -0.12, y: 0. },
    },
};

#[derive(Default)]
pub struct ThreeBodyResult {
    pub before_optim: Vec<ThreeBodyState>,
    pub after_optim: Vec<ThreeBodyState>,
}

pub fn simulate_three_body(
    params: &OrbitalParams,
) -> Result<ThreeBodyResult, Box<dyn std::error::Error>> {
    assert!(params.three_body.is_some());
    let tape = Tape::new();
    let model = get_three_body_model(&tape, params);

    let last_state = model.states.last().unwrap();

    println!(
        "tape nodes: {}, size: {}, states: {}",
        tape.len(),
        std::mem::size_of::<TapeNode<f64>>() * tape.len(),
        model.states.len()
    );

    let first_state = model.states.first().unwrap();

    model.loss.eval();
    model.loss.backprop().unwrap();
    let xd = try_grad!(first_state.velo.x);
    let yd = try_grad!(first_state.velo.y);

    let eval = [last_state.pos.x.eval(), last_state.pos.y.eval()];
    println!(
        "eval: {eval:?}, accel.eval: {:?}",
        [last_state.accel.x.eval(), last_state.accel.y.eval()]
    );
    println!("simulate_orbital: derive(vx, vy): {:?}, {:?}", xd, yd);

    let before_optim = model.states.iter().map(|state| state.into()).collect();

    let mut best: Option<(Vec2<f64>, f64)> = None;
    let velocity_magnitude = params.initial_velo.length();
    for grid_y in -params.grid_search_size..params.grid_search_size {
        for grid_x in -params.grid_search_size..params.grid_search_size {
            let velo = Vec2 {
                x: grid_x as f64 * velocity_magnitude * 0.5 + params.initial_velo.x,
                y: grid_y as f64 * velocity_magnitude * 0.5 + params.initial_velo.y,
            };
            let loss = optimize(&model, &velo, params)?;
            if let Some(best_val) = best {
                if loss.1 < best_val.1 {
                    best = Some(loss);
                }
            } else {
                best = Some(loss);
            }
        }
    }

    if let Some(best) = best {
        first_state.velo.x.set(best.0.x).unwrap();
        first_state.velo.y.set(best.0.y).unwrap();

        Ok(ThreeBodyResult {
            before_optim,
            after_optim: model.states.iter().map(|state| state.into()).collect(),
        })
    } else {
        Err(format!("Optimization did not converge!").into())
    }
}

struct ThreeBodyModelState<'a> {
    accel: Vec2<TapeTerm<'a>>,
    pos: Vec2<TapeTerm<'a>>,
    velo: Vec2<TapeTerm<'a>>,
    moon: BodyState<'a>,
}

impl<'a> AbstractModelState<'a> for ThreeBodyModelState<'a> {
    fn get_velo(&self) -> Vec2t<'a> {
        self.velo
    }
    fn set_velo(&self, velo: &Vec2d) -> Result<(), RustogradError> {
        self.velo.x.set(velo.x)?;
        self.velo.y.set(velo.y)?;
        Ok(())
    }
}

impl<'a> From<&ThreeBodyModelState<'a>> for ThreeBodyState {
    fn from(state: &ThreeBodyModelState<'a>) -> Self {
        let moon = state.moon;
        Self {
            satellite: OrbitalBody {
                pos: state.pos.map(|x| x.eval()),
                velo: state.velo.map(|x| x.eval()),
            },
            moon: OrbitalBody {
                pos: moon.pos.map(|x| x.eval_noclear()),
                velo: moon.velo,
            },
        }
    }
}

pub fn three_body_simulate_step(
    state: &mut ThreeBodyState,
    h_thrust: f64,
    v_thrust: f64,
    delta_time: f64,
) -> Vec2<f64> {
    let thrust = Vec2 {
        x: h_thrust,
        y: v_thrust,
    } * THRUST_ACCEL;
    let earth = Body {
        pos: EARTH_POS,
        gm: GM,
    };
    let sat = &mut state.satellite;
    let moon_state = &mut state.moon;
    let moon = Body {
        pos: moon_state.pos,
        gm: GM_MOON,
    };
    let accel;
    (sat.pos, sat.velo, accel) =
        simulate_step(&[&earth, &moon], sat.pos, sat.velo, thrust, delta_time);
    (moon_state.pos, moon_state.velo, _) = simulate_step(
        &[&earth],
        moon_state.pos,
        moon_state.velo,
        Vec2::zero(),
        delta_time,
    );
    accel
}

fn get_three_body_model<'a>(
    tape: &'a Tape<f64>,
    params: &OrbitalParams,
) -> Model<'a, ThreeBodyModelState<'a>> {
    let mut pos = Vec2::new(
        tape.term("x", params.initial_pos.x),
        tape.term("y", params.initial_pos.y),
    );
    let mut velo = Vec2::new(
        tape.term("vx", params.initial_velo.x),
        tape.term("vy", params.initial_velo.y),
    );
    let moon_pos = params.three_body.unwrap().moon_pos;
    let moon = get_moon_model(tape, params, moon_pos);

    let gm = tape.term("GM", params.earth_gm);
    let zero = tape.zero();
    let half = tape.term("0.5", 0.5);
    let mut states = vec![ThreeBodyModelState {
        accel: Vec2 { x: zero, y: zero },
        pos,
        velo,
        moon: moon.states[0],
    }];

    let earth = BodyModelState {
        pos: Vec2::new(zero, zero),
        gm,
    };

    for i in 0..params.max_iter {
        let accel;
        let bodies = [
            earth,
            BodyModelState {
                pos: moon.states[i].pos,
                gm: moon.gm,
            },
        ];
        (pos, velo, accel) = model_simulate_step(pos, velo, &bodies, half);
        states.push(ThreeBodyModelState {
            accel,
            pos,
            velo,
            moon: moon.states[i],
        });
    }

    println!("Moon distance: {:?}", moon.target_r.eval_noclear());
    let loss = states
        .iter()
        .zip(moon.states.iter())
        .fold(None, |acc: Option<TapeTerm<'a>>, (state, moon_state)| {
            let diff = state.pos - moon_state.pos;
            let loss = diff
                .length2()
                .apply("sqrt", f64::sqrt, |x| -0.5 * x.powf(-0.5))
                - moon.target_r;
            let loss = loss * loss;
            if let Some(acc) = acc {
                Some(acc.apply_bin(loss, Box::new(MinOp)))
            } else {
                Some(loss)
            }
        })
        .unwrap();

    Model { states, loss }
}

fn get_moon_model<'a>(
    tape: &'a Tape,
    params: &OrbitalParams,
    moon_pos: Vec2<f64>,
) -> BodyModel<'a> {
    let moon_delta = moon_pos - params.earth_pos;
    let moon_r = moon_delta.length();
    let moon_phase = moon_delta.y.atan2(moon_delta.x);
    let period = 2. * std::f64::consts::PI * (moon_r.powi(3) / params.earth_gm).sqrt();
    let omega = 2. * std::f64::consts::PI / period;
    let moon_states: Vec<_> = (0..=params.max_iter)
        .map(|i| {
            let angle = i as f64 * omega + moon_phase;
            let pos = Vec2::new(
                tape.term("mx", params.earth_pos.x + moon_r * angle.cos()),
                tape.term("my", params.earth_pos.y + moon_r * angle.sin()),
            );
            let velo = Vec2::new(angle.cos(), angle.sin()).left90();
            BodyState { pos, velo }
        })
        .collect();
    BodyModel {
        states: moon_states,
        gm: tape.term("GMm", params.three_body.unwrap().moon_gm),
        target_r: tape.term("target_r", params.three_body.unwrap().target_r),
    }
}

pub fn calc_initial_moon(params: &OrbitalParams) -> Option<OrbitalBody> {
    let moon_pos = params.three_body?.moon_pos;
    let moon_delta = moon_pos - params.earth_pos;
    let moon_r = moon_delta.length();
    let velo = moon_delta.left90() / moon_r * (GM / moon_r).sqrt();
    Some(OrbitalBody {
        pos: moon_pos,
        velo,
    })
}
