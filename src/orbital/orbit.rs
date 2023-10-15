use rustograd::{error::RustogradError, tape::TapeNode, Tape, TapeTerm};

use crate::{error::GradDoesNotExist, ops::MinOp, orbital::optimize, vec2::Vec2};

use super::{
    gravity_f64, model_simulate_step, simulate_step, AbstractModelState, Body, BodyModelState,
    Model, OrbitalBody, OrbitalParams, Vec2d, Vec2t, EARTH_POS, GM, THRUST_ACCEL,
};

/// State of the whole system at an instant.
#[derive(Clone, Copy)]
pub struct OrbitalState {
    pub satellite: OrbitalBody,
    pub target: OrbitalBody,
}

pub const ORBITAL_STATE: OrbitalState = OrbitalState {
    satellite: OrbitalBody {
        pos: Vec2 { x: 2., y: 0. },
        velo: Vec2 { x: 0., y: 0.12 },
    },
    target: OrbitalBody {
        pos: Vec2 { x: 0., y: 2. },
        velo: Vec2 { x: -0.12, y: 0. },
    },
};

#[derive(Default)]
pub struct OrbitalResult {
    pub before_optim: Vec<OrbitalState>,
    pub after_optim: Vec<OrbitalState>,
}

pub fn simulate_orbital(
    params: &OrbitalParams,
) -> Result<OrbitalResult, Box<dyn std::error::Error>> {
    assert!(params.moon_pos.is_none());
    let tape = Tape::new();
    let model = get_model(&tape, params);

    let last_state = model.states.last().unwrap();

    // if let Ok(f) = std::fs::File::create("graph.dot") {
    //     let x2 = model.xs.first().unwrap();
    //     x2.x.dot_builder()
    //         .vertical(true)
    //         .output_term(x2.x, "x2.x")
    //         .output_term(x2.y, "x2.y")
    //         .dot(&mut std::io::BufWriter::new(f))
    //         .unwrap();
    // }

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

        Ok(OrbitalResult {
            before_optim,
            after_optim: model.states.iter().map(|state| state.into()).collect(),
        })
    } else {
        Err(format!("Optimization did not converge!").into())
    }
}

pub fn orbital_simulate_step(
    state: &mut OrbitalState,
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
    let accel;
    (sat.pos, sat.velo, accel) = simulate_step(&[&earth], sat.pos, sat.velo, thrust, delta_time);
    let target = &mut state.target;
    (target.pos, target.velo, _) =
        simulate_step(&[&earth], target.pos, target.velo, Vec2::zero(), delta_time);
    accel
}

struct ModelState<'a> {
    accel: Vec2<TapeTerm<'a>>,
    pos: Vec2<TapeTerm<'a>>,
    velo: Vec2<TapeTerm<'a>>,
    target_pos: Vec2<TapeTerm<'a>>,
    target_velo: Vec2<TapeTerm<'a>>,
}

impl<'a> AbstractModelState<'a> for ModelState<'a> {
    fn get_velo(&self) -> Vec2t<'a> {
        self.velo
    }
    fn set_velo(&self, velo: &Vec2d) -> Result<(), RustogradError> {
        self.velo.x.set(velo.x)?;
        self.velo.y.set(velo.y)?;
        Ok(())
    }
}

impl<'a> From<&ModelState<'a>> for OrbitalState {
    fn from(state: &ModelState<'a>) -> Self {
        Self {
            satellite: OrbitalBody {
                pos: state.pos.map(|x| x.eval()),
                velo: state.velo.map(|x| x.eval()),
            },
            target: OrbitalBody {
                pos: state.target_pos.map(|x| x.eval_noclear()),
                velo: state.target_velo.map(|x| x.eval_noclear()),
            },
        }
    }
}

fn get_model<'a>(tape: &'a Tape<f64>, params: &OrbitalParams) -> Model<'a, ModelState<'a>> {
    let mut pos = Vec2::new(
        tape.term("x", params.initial_pos.x),
        tape.term("y", params.initial_pos.y),
    );
    let mut velo = Vec2::new(
        tape.term("vx", params.initial_velo.x),
        tape.term("vy", params.initial_velo.y),
    );
    let mut target_pos;
    let mut target_velo;
    target_pos = Vec2::new(tape.term("target_x", 0.), tape.term("target_y", 2.));
    target_velo = Vec2::new(tape.term("target_vx", -0.12), tape.term("target_vy", 0.0));

    let gm = tape.term("GM", params.earth_gm);
    let zero = tape.zero();
    let half = tape.term("0.5", 0.5);
    let mut states = vec![ModelState {
        accel: Vec2 { x: zero, y: zero },
        pos,
        velo,
        target_pos,
        target_velo,
    }];

    let earth = BodyModelState {
        pos: Vec2::new(zero, zero),
        gm,
    };

    for i in 0..params.max_iter {
        let accel;
        let bodies = [earth];
        (pos, velo, accel) = model_simulate_step(pos, velo, &bodies, half);
        (target_pos, target_velo, _) = model_simulate_step(target_pos, target_velo, &bodies, half);
        states.push(ModelState {
            accel,
            pos,
            velo,
            target_pos,
            target_velo,
        });
    }

    let loss = states
        .iter()
        .fold(None, |acc: Option<TapeTerm<'a>>, state| {
            let diff = state.pos - state.target_pos;
            let velo_diff = state.velo - state.target_velo;
            let mut loss = diff.length2();
            if params.optim_velo {
                loss = loss + velo_diff.length2();
            }
            if let Some(acc) = acc {
                Some(acc.apply_bin(loss, Box::new(MinOp)))
            } else {
                Some(loss)
            }
        })
        .unwrap();

    Model { states, loss }
}
