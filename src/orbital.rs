use crate::{error::GradDoesNotExist, ops::MinOp, vec2::Vec2};
use rustograd::{tape::TapeNode, Tape, TapeTerm};

const RATE: f64 = 3e-4;

#[derive(Clone, Copy)]
pub struct OrbitalParams {
    pub initial_pos: Vec2<f64>,
    pub initial_velo: Vec2<f64>,
    pub earth_pos: Vec2<f64>,
    pub earth_gm: f64,
    pub moon_pos: Option<Vec2<f64>>,
    pub moon_gm: f64,
    /// Whether to optimize the relative velocity to 0. It is not generally possible with just the initial velocity.
    pub optim_velo: bool,
    /// The weight on loss function from deviation from the initial velocity.
    pub initial_velo_weight: f64,
    pub grid_search_size: i32,
    pub rate: f64,
    pub optim_iter: usize,
    pub max_iter: usize,
}

impl Default for OrbitalParams {
    fn default() -> Self {
        Self {
            initial_pos: Vec2 { x: 2., y: 0. },
            initial_velo: Vec2 { x: 0., y: 0.15 },
            earth_pos: Vec2 { x: 0., y: 0. },
            earth_gm: GM,
            moon_pos: Some(INIT_MOON_POS),
            moon_gm: GM_MOON,
            optim_velo: false,
            initial_velo_weight: 1.,
            grid_search_size: 2,
            rate: RATE,
            optim_iter: 60,
            max_iter: 100,
        }
    }
}

/// Instantaneous state of a celestial body.
#[derive(Clone, Copy)]
pub struct OrbitalBody {
    pub pos: Vec2<f64>,
    pub velo: Vec2<f64>,
}

/// State of the whole system at an instant.
#[derive(Clone, Copy)]
pub struct OrbitalState {
    pub satellite: OrbitalBody,
    pub target: OrbitalBody,
    pub moon: Option<OrbitalBody>,
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
    moon: None,
};

#[derive(Default)]
pub struct OrbitalResult {
    pub before_optim: Vec<OrbitalState>,
    pub after_optim: Vec<OrbitalState>,
}

pub fn simulate_orbital(
    params: &OrbitalParams,
) -> Result<OrbitalResult, Box<dyn std::error::Error>> {
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

fn optimize<'a>(
    model: &'a Model,
    velo: &Vec2<f64>,
    params: &OrbitalParams,
) -> Result<(Vec2<f64>, f64), GradDoesNotExist> {
    const RATE: f64 = 1e-5;

    let first_state = model.states.first().unwrap();
    first_state.velo.x.set(velo.x).unwrap();
    first_state.velo.y.set(velo.y).unwrap();

    let mut loss_val = 0.;

    // optimization loop
    for _i in 0..params.optim_iter {
        model.loss.eval();
        model.loss.backprop().unwrap();
        let xd = try_grad!(first_state.velo.x);
        let yd = try_grad!(first_state.velo.y);
        let first_velo = first_state.velo;
        first_velo
            .x
            .set(first_velo.x.data().unwrap() - xd * RATE)
            .unwrap();
        first_velo
            .y
            .set(first_velo.y.data().unwrap() - yd * RATE)
            .unwrap();
        loss_val = model.loss.eval();
        // println!(
        //     "simulate_orbital optimize[{i}]: derive(vx, vy): {:?}, {:?}, loss: {}",
        //     xd, yd, loss_val
        // );
    }

    let optimized_velo = Vec2 {
        x: first_state.velo.x.data().unwrap(),
        y: first_state.velo.y.data().unwrap(),
    };

    let velo_loss = (params.initial_velo - optimized_velo).length2();

    Ok((
        optimized_velo,
        loss_val + velo_loss * params.initial_velo_weight,
    ))
}

const THRUST_ACCEL: f64 = 0.001;

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
    let moon = state.moon.map(|body| Body {
        pos: body.pos,
        gm: GM_MOON,
    });
    let simulate_step = |moon, pos, velo, thrust| {
        let accel = gravity_f64(&earth, moon, pos) + thrust;
        let delta_x = (velo + accel * 0.5 * delta_time) * delta_time;
        let accel2 = gravity_f64(&earth, moon, pos + delta_x * 0.5) + thrust;
        let delta_x2 = (velo + accel2 * 0.5 * delta_time) * delta_time;
        let newpos = pos + delta_x2;
        let newvelo = velo + accel2 * delta_time;
        (newpos, newvelo, accel2)
    };
    let sat = &mut state.satellite;
    let accel;
    (sat.pos, sat.velo, accel) = simulate_step(moon.as_ref(), sat.pos, sat.velo, thrust);
    let target = &mut state.target;
    (target.pos, target.velo, _) =
        simulate_step(moon.as_ref(), target.pos, target.velo, Vec2::zero());
    if let Some(ref mut moon) = state.moon {
        (moon.pos, moon.velo, _) = simulate_step(None, moon.pos, moon.velo, Vec2::zero());
    }
    accel
}

const EARTH_POS: Vec2<f64> = Vec2 { x: 0., y: 0. };
pub const GM: f64 = 0.03;
pub const GM_MOON: f64 = GM / 3.;
const INIT_MOON_POS: Vec2<f64> = Vec2 { x: -3., y: 0. };

struct BodyModelState<'a> {
    pos: Vec2<TapeTerm<'a>>,
    velo: Vec2<TapeTerm<'a>>,
    gm: TapeTerm<'a>,
}

struct ModelState<'a> {
    accel: Vec2<TapeTerm<'a>>,
    pos: Vec2<TapeTerm<'a>>,
    velo: Vec2<TapeTerm<'a>>,
    target_pos: Vec2<TapeTerm<'a>>,
    target_velo: Vec2<TapeTerm<'a>>,
    moon: Option<BodyState<'a>>,
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
            moon: state.moon.map(|moon| OrbitalBody {
                pos: moon.pos.map(|x| x.eval_noclear()),
                velo: moon.velo,
            }),
        }
    }
}

struct Model<'a> {
    states: Vec<ModelState<'a>>,
    loss: TapeTerm<'a>,
}

fn get_model<'a>(tape: &'a Tape<f64>, params: &OrbitalParams) -> Model<'a> {
    let mut pos = Vec2::new(
        tape.term("x", params.initial_pos.x),
        tape.term("y", params.initial_pos.y),
    );
    let mut velo = Vec2::new(
        tape.term("vx", params.initial_velo.x),
        tape.term("vy", params.initial_velo.y),
    );
    let earth = Vec2::new(tape.term("bx", 0.), tape.term("by", 0.));
    let mut target_pos;
    let mut target_velo;
    let moon;
    if let Some(moon_pos) = params.moon_pos {
        let moon_model;
        (target_pos, target_velo, moon_model) = get_moon_model(tape, params, moon_pos);
        moon = Some(moon_model);
    } else {
        target_pos = Vec2::new(tape.term("target_x", 0.), tape.term("target_y", 2.));
        target_velo = Vec2::new(tape.term("target_vx", -0.12), tape.term("target_vy", 0.0));
        moon = None;
    }

    let gm = tape.term("GM", params.earth_gm);
    let zero = tape.zero();
    let half = tape.term("0.5", 0.5);
    let mut states = vec![ModelState {
        accel: Vec2 { x: zero, y: zero },
        pos,
        velo,
        target_pos,
        target_velo,
        moon: None,
    }];

    let simulate_step = |pos, vx, moon| {
        let accel = gravity(earth, moon, pos, gm);
        let delta_x = vx + accel * half;
        let accel2 = gravity(earth, moon, pos + delta_x * half, gm);
        let delta_x2 = vx + accel2 * half;
        let pos = pos + delta_x2;
        let vx = vx + accel2;
        (pos, vx, accel2)
    };

    for i in 0..params.max_iter {
        let moon_i = moon.as_ref().map(|moon| (moon.states[i], moon.gm));
        let (pos2, vx2, accel) = simulate_step(pos, velo, moon_i);
        (pos, velo) = (pos2, vx2);
        (target_pos, target_velo, _) = simulate_step(target_pos, target_velo, moon_i);
        states.push(ModelState {
            accel,
            pos,
            velo,
            target_pos,
            target_velo,
            moon: moon_i.map(|(s, _)| s),
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

#[derive(Clone, Copy)]
struct BodyState<'a> {
    pos: Vec2<TapeTerm<'a>>,
    /// Static body velocities are used only for visualization, so not need to be part of the tape.
    velo: Vec2<f64>,
}

struct BodyModel<'a> {
    states: Vec<BodyState<'a>>,
    gm: TapeTerm<'a>,
}

fn get_moon_model<'a>(
    tape: &'a Tape,
    params: &OrbitalParams,
    moon_pos: Vec2<f64>,
) -> (Vec2<TapeTerm<'a>>, Vec2<TapeTerm<'a>>, BodyModel<'a>) {
    let pos = Vec2::new(-0.5, 0.) + moon_pos;
    let delta_pos = pos - moon_pos;
    let r = delta_pos.length();
    let target_velo = delta_pos.left90() / r * (params.moon_gm / r).sqrt();
    let target_pos = Vec2::new(tape.term("target_x", pos.x), tape.term("target_y", pos.y));
    let moon_delta = moon_pos - params.earth_pos;
    let moon_r = moon_delta.length();
    let moon_phase = moon_delta.y.atan2(moon_delta.x);
    let period = 2. * std::f64::consts::PI * (moon_r.powi(3) / params.earth_gm).sqrt();
    let omega = 2. * std::f64::consts::PI / period;
    let moon_states: Vec<_> = (0..params.max_iter)
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
    let moon_velo = Vec2::new(moon_phase.cos(), moon_phase.sin()).left90() * omega;
    let target_velo = target_velo + moon_velo;
    let target_velo = Vec2::new(
        tape.term("target_vx", target_velo.x),
        tape.term("target_vy", target_velo.y),
    );
    let moon_model = BodyModel {
        states: moon_states,
        gm: tape.term("GMm", params.moon_gm),
    };
    (target_pos, target_velo, moon_model)
}

pub fn calc_initial_moon(params: &OrbitalParams) -> Option<OrbitalBody> {
    let moon_delta = params.moon_pos? - params.earth_pos;
    let moon_r = moon_delta.length();
    let moon_phase = moon_delta.y.atan2(moon_delta.x);
    let period = 2. * std::f64::consts::PI * (moon_r.powi(3) / params.earth_gm).sqrt();
    let omega = 2. * std::f64::consts::PI / period;
    Some(OrbitalBody {
        pos: Vec2::new(
            params.earth_pos.x + moon_r * moon_phase.cos(),
            params.earth_pos.y + moon_r * moon_phase.sin(),
        ),
        velo: Vec2::new(moon_phase.cos(), moon_phase.sin()).left90() * omega,
    })
}

// fn orbital_elements(pos: Vec2<f64>, velo: Vec2<f64>, mu: f64) -> f64 {
//     //...Magnitudes of R0 and V0:
//     let r0 = pos.length();
//     let v0 = velo.length();
//     //...Initial radial velocity:
//     let vr0 = pos.dot(velo)/r0;
//     //...Reciprocal of the semimajor axis (from the energy equation):
//     let alpha = 2./r0 - v0 * v0 / mu;
//     //...Compute the universal anomaly:
//     let x = kepler_U(t, r0, vr0, alpha);
//     //...Compute the f and g functions:
//     let (f, g) = f_and_g(x, t, r0, alpha);
//     //...Compute the final position vector:
//     let r = f*R0 + g*V0;
//     //...Compute the magnitude of R:
//     let rr = norm(R);
//     //...Compute the derivatives of f and g:
//     let (fdot, gdot) = fDot_and_gDot(x, r, r0, alpha);
//     //...Compute the final velocity:
//     let vv = fdot*R0 + gdot*V0;
// }

fn gravity<'a>(
    earth: Vec2<TapeTerm<'a>>,
    moon: Option<(BodyState<'a>, TapeTerm<'a>)>,
    pos: Vec2<TapeTerm<'a>>,
    gm: TapeTerm<'a>,
) -> Vec2<TapeTerm<'a>> {
    let gravity_body = |body: Vec2<TapeTerm<'a>>| -> Vec2<TapeTerm<'a>> {
        let diff = body - pos;
        let len2 = diff.x * diff.x + diff.y * diff.y;
        let len32 = len2.apply(
            "pow[3/2]",
            |x| x.powf(3. / 2.),
            |x| 3. / 2. * x.powf(1. / 2.),
        );
        diff / len32
    };
    if let Some(moon) = moon {
        gravity_body(earth) * gm + gravity_body(moon.0.pos) * moon.1
    } else {
        gravity_body(earth) * gm
    }
}

struct Body {
    pos: Vec2<f64>,
    gm: f64,
}

fn gravity_f64(earth: &Body, moon: Option<&Body>, pos: Vec2<f64>) -> Vec2<f64> {
    let gravity_body = |body: &Body| {
        let diff = body.pos - pos;
        let len2 = diff.x * diff.x + diff.y * diff.y;
        let len32 = len2.powf(3. / 2.);
        diff / len32 * body.gm
    };
    if let Some(moon) = moon {
        gravity_body(earth) + gravity_body(moon)
    } else {
        gravity_body(earth)
    }
}
