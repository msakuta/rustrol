use crate::{error::GradDoesNotExist, ops::MinOp, vec2::Vec2};
use rustograd::{Tape, TapeTerm};

const RATE: f64 = 3e-4;

pub struct OrbitalParams {
    pub initial_pos: Vec2<f64>,
    pub initial_velo: Vec2<f64>,
    /// Whether to optimize the relative velocity to 0. It is not generally possible with just the initial velocity.
    pub optim_velo: bool,
    /// The weight on loss function from deviation from the initial velocity.
    pub initial_velo_weight: f64,
    pub rate: f64,
    pub optim_iter: usize,
    pub max_iter: usize,
}

impl Default for OrbitalParams {
    fn default() -> Self {
        Self {
            initial_pos: Vec2 { x: 2., y: 0. },
            initial_velo: Vec2 { x: 0., y: 0.15 },
            optim_velo: false,
            initial_velo_weight: 1.,
            rate: RATE,
            optim_iter: 60,
            max_iter: 100,
        }
    }
}

#[derive(Clone, Copy)]
pub struct OrbitalState {
    pub pos: Vec2<f64>,
    pub velo: Vec2<f64>,
    pub target_pos: Vec2<f64>,
    pub target_velo: Vec2<f64>,
}

pub const ORBITAL_STATE: OrbitalState = OrbitalState {
    pos: Vec2 { x: 2., y: 0. },
    velo: Vec2 { x: 0., y: 0.12 },
    target_pos: Vec2 { x: 0., y: 2. },
    target_velo: Vec2 { x: -0.12, y: 0. },
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

    println!("tape nodes: {}, states: {}", tape.len(), model.states.len());

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

    let before_optim = model
        .states
        .iter()
        .map(|state| OrbitalState {
            pos: state.pos.map(|x| x.eval()),
            velo: state.velo.map(|x| x.eval()),
            target_pos: state.target_pos.map(|x| x.eval_noclear()),
            target_velo: state.target_velo.map(|x| x.eval_noclear()),
        })
        .collect();

    let mut best: Option<(Vec2<f64>, f64)> = None;
    let velocity_magnitude = params.initial_velo.length();
    for grid_y in -2..2 {
        for grid_x in -2..2 {
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
            after_optim: model
                .states
                .iter()
                .map(|state| OrbitalState {
                    pos: state.pos.map(|x| x.eval()),
                    velo: state.velo.map(|x| x.eval()),
                    target_pos: state.target_pos.map(|x| x.eval_noclear()),
                    target_velo: state.target_velo.map(|x| x.eval_noclear()),
                })
                .collect(),
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
    let simulate_step = |pos, velo, thrust| {
        let accel = gravity_f64(EARTH_POS, pos, GM) + thrust;
        let delta_x = (velo + accel * 0.5 * delta_time) * delta_time;
        let accel2 = gravity_f64(EARTH_POS, pos + delta_x * 0.5, GM) + thrust;
        let delta_x2 = (velo + accel2 * 0.5 * delta_time) * delta_time;
        let newpos = pos + delta_x2;
        let newvelo = velo + accel2 * delta_time;
        (newpos, newvelo, accel2)
    };
    let accel;
    (state.pos, state.velo, accel) = simulate_step(state.pos, state.velo, thrust);
    (state.target_pos, state.target_velo, _) =
        simulate_step(state.target_pos, state.target_velo, Vec2::zero());
    accel
}

const EARTH_POS: Vec2<f64> = Vec2 { x: 0., y: 0. };
pub const GM: f64 = 0.03;

struct ModelState<'a> {
    accel: Vec2<TapeTerm<'a>>,
    pos: Vec2<TapeTerm<'a>>,
    velo: Vec2<TapeTerm<'a>>,
    target_pos: Vec2<TapeTerm<'a>>,
    target_velo: Vec2<TapeTerm<'a>>,
}

struct Model<'a> {
    states: Vec<ModelState<'a>>,
    loss: TapeTerm<'a>,
}

fn get_model<'a>(tape: &'a Tape<f64>, params: &OrbitalParams) -> Model<'a> {
    let mut pos = Vec2 {
        x: tape.term("x", params.initial_pos.x),
        y: tape.term("y", params.initial_pos.y),
    };
    let mut velo = Vec2 {
        x: tape.term("vx", params.initial_velo.x),
        y: tape.term("vy", params.initial_velo.y),
    };
    let mut target_pos = Vec2 {
        x: tape.term("target_x", 0.),
        y: tape.term("target_y", 2.),
    };
    let mut target_velo = Vec2 {
        x: tape.term("target_vx", -0.12),
        y: tape.term("target_vy", 0.0),
    };
    let earth = Vec2 {
        x: tape.term("bx", 0.),
        y: tape.term("by", 0.),
    };

    let gm = tape.term("GM", GM);
    let zero = tape.zero();
    let half = tape.term("0.5", 0.5);
    let mut states = vec![ModelState {
        accel: Vec2 { x: zero, y: zero },
        pos,
        velo,
        target_pos,
        target_velo,
    }];

    let simulate_step = |pos, vx| {
        let accel = gravity(earth, pos, gm);
        let delta_x = vx + accel * half;
        let accel2 = gravity(earth, pos + delta_x * half, gm);
        let delta_x2 = vx + accel2 * half;
        let pos = pos + delta_x2;
        let vx = vx + accel2;
        (pos, vx, accel2)
    };

    for _ in 0..params.max_iter {
        let (pos2, vx2, accel) = simulate_step(pos, velo);
        (pos, velo) = (pos2, vx2);
        let (target_pos2, target_velo2, _target_accel2) = simulate_step(target_pos, target_velo);
        (target_pos, target_velo) = (target_pos2, target_velo2);
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

fn gravity<'a>(
    earth: Vec2<TapeTerm<'a>>,
    pos: Vec2<TapeTerm<'a>>,
    gm: TapeTerm<'a>,
) -> Vec2<TapeTerm<'a>> {
    let diff = earth - pos;
    let len2 = diff.x * diff.x + diff.y * diff.y;
    let len32 = len2.apply(
        "pow[3/2]",
        |x| x.powf(3. / 2.),
        |x| 3. / 2. * x.powf(1. / 2.),
    );
    diff / len32 * gm
}

fn gravity_f64(earth: Vec2<f64>, pos: Vec2<f64>, gm: f64) -> Vec2<f64> {
    let diff = earth - pos;
    let len2 = diff.x * diff.x + diff.y * diff.y;
    let len32 = len2.powf(3. / 2.);
    diff / len32 * gm
}