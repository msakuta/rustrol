use rustograd::{Tape, TapeTerm};

use crate::{error::GradDoesNotExist, ops::MinOp, vec2::Vec2, xor128::Xor128};

pub struct MissileParams {
    pub rate: f64,
    pub optim_iter: usize,
    pub max_iter: usize,
}

impl Default for MissileParams {
    fn default() -> Self {
        Self {
            rate: RATE,
            optim_iter: 60,
            max_iter: 60,
        }
    }
}

const HIT_DISTANCE: f64 = 1.;
const HIT_DISTANCE2: f64 = HIT_DISTANCE * HIT_DISTANCE;

pub struct MissileState {
    pub pos: Vec2<f64>,
    pub velo: Vec2<f64>,
    pub target: Vec2<f64>,
    pub heading: f64,
    pub prediction: Vec<Vec2<f64>>,
    pub target_prediction: Vec<Vec2<f64>>,
}

pub fn simulate_missile(
    pos: Vec2<f64>,
    heading: f64,
    params: &MissileParams,
) -> Result<Vec<MissileState>, GradDoesNotExist> {
    let tape = Tape::new();
    let model = get_model(&tape, pos, heading);

    // if let Ok(f) = std::fs::File::create("graph.dot") {
    //     let x2 = model.hist1.first().unwrap().pos;
    //     x2.x.dot_builder()
    //         .vertical(true)
    //         .output_term(x2.x, "x2.x")
    //         .output_term(x2.y, "x2.y")
    //         .dot(&mut std::io::BufWriter::new(f))
    //         .unwrap();
    // }

    println!(
        "size: {}, b1: {}, b2: {}",
        tape.len(),
        model.missile_hist.len(),
        model.target_hist.len()
    );

    let mut rng = Xor128::new(12321);
    let mut hist = vec![];
    for t in 0..params.max_iter {
        let (h_thrust, v_thrust) = optimize(&model, t, params)?;
        let first = model.missile_hist.first().unwrap();
        let heading = first.heading.data().unwrap();
        let Some((pos, target)) = simulate_step(&model, &mut rng, t, h_thrust, v_thrust) else {
            break;
        };
        hist.push(MissileState {
            pos,
            velo: first.velo.map(|x| x.eval_noclear()),
            target,
            heading,
            prediction: model
                .missile_hist
                .iter()
                .map(|b1| b1.pos.map(|x| x.eval()))
                .collect(),
            target_prediction: model
                .target_hist
                .iter()
                .map(|b1| b1.map(|x| x.eval()))
                .collect(),
        });
    }
    Ok(hist)
}

pub const MAX_THRUST: f64 = 0.09;
const RATE: f64 = 3e-4;
const GM: f64 = 0.03;
const DRAG: f64 = 0.05;
const TARGET_X: f64 = 20.;
const TARGET_VX: f64 = 0.5;

fn optimize(
    model: &Model,
    t: usize,
    params: &MissileParams,
) -> Result<(f64, f64), GradDoesNotExist> {
    for (t2, target) in model.target_hist.iter().enumerate() {
        target
            .x
            .set(TARGET_X - TARGET_VX * (t + t2) as f64)
            .unwrap();
        target.y.set(5.).unwrap();
        target.x.eval();
        target.y.eval();
    }

    for _ in 0..params.optim_iter {
        model.loss.eval();
        model.loss.backprop().unwrap();
        for state in model.missile_hist.iter().take(model.missile_hist.len() - 2) {
            let d_h_thrust = try_grad!(state.h_thrust);
            let d_v_thrust = try_grad!(state.v_thrust);
            state
                .v_thrust
                .set(
                    (state.v_thrust.data().unwrap() - d_v_thrust * RATE)
                        .min(MAX_THRUST)
                        .max(0.),
                )
                .unwrap();
            state
                .h_thrust
                .set(
                    (state.h_thrust.data().unwrap() - d_h_thrust * RATE)
                        .min(MAX_THRUST)
                        .max(-MAX_THRUST),
                )
                .unwrap();
        }
    }

    let loss_val = model.loss.eval_noclear();
    let first = model.missile_hist.first().unwrap();
    println!(
        "h_thrust: {}, v_thrust: {}, loss: {}",
        first.h_thrust.eval_noclear(),
        first.v_thrust.eval_noclear(),
        loss_val
    );

    let h_thrust = model.missile_hist.first().unwrap().h_thrust.data().unwrap();
    let v_thrust = model.missile_hist.first().unwrap().v_thrust.data().unwrap();

    Ok((h_thrust, v_thrust))
}

fn simulate_step(
    model: &Model,
    rng: &mut Xor128,
    t: usize,
    h_thrust: f64,
    v_thrust: f64,
) -> Option<(Vec2<f64>, Vec2<f64>)> {
    let missile = model.missile_hist.first().unwrap();
    let heading = missile.heading.data().unwrap();
    let thrust_vec = Vec2::<f64> {
        x: heading.sin() * v_thrust,
        y: heading.cos() * v_thrust,
    };
    let velo = missile.velo.map(|x| x.data().unwrap());
    let velolen2 = velo.x * velo.x + velo.y * velo.y;
    let velolen12 = velolen2.sqrt();
    let randomize = Vec2 {
        x: rng.next() - 0.5,
        y: rng.next() - 0.5,
    };
    let next_heading = heading + h_thrust;
    let accel =
        Vec2::<f64> { x: 0., y: -GM } + randomize * 0.02 - velo * DRAG / velolen12 + thrust_vec;
    let delta_x2 = velo + accel * 0.5;
    let oldpos = missile.pos.map(|x| x.data().unwrap());
    let newpos = oldpos + delta_x2;
    let target = model
        .target_hist
        .first()
        .unwrap()
        .map(|x| x.data().unwrap());
    if (newpos - target).length2() < HIT_DISTANCE2 {
        return None;
    }
    missile.pos.x.set(newpos.x).unwrap();
    missile.pos.y.set(newpos.y).unwrap();
    let newvelo = missile.velo.map(|x| x.data().unwrap()) + accel;
    missile.velo.x.set(newvelo.x).unwrap();
    missile.velo.y.set(newvelo.y).unwrap();
    missile.heading.set(next_heading).unwrap();
    for (t2, target) in model.target_hist.iter().enumerate() {
        target
            .x
            .set(TARGET_X - TARGET_VX * (t + t2) as f64)
            .unwrap();
    }
    let target_pos = model.target_hist.first().unwrap().map(|x| x.eval());
    Some((oldpos, target_pos))
}

pub fn missile_simulate_step(
    missile: &mut MissileState,
    // t: usize,
    h_thrust: f64,
    v_thrust: f64,
    delta_time: f64,
) {
    let heading = missile.heading;
    let thrust_vec = Vec2::<f64> {
        x: heading.sin() * v_thrust,
        y: heading.cos() * v_thrust,
    };
    let velo = missile.velo;
    let velolen2 = velo.x * velo.x + velo.y * velo.y;
    let next_heading = heading + h_thrust * delta_time;
    let mut accel = Vec2::<f64> { x: 0., y: -GM } + thrust_vec;
    if velolen2 != 0. {
        let velolen = velolen2.sqrt();
        accel -= velo * DRAG / velolen;
    }
    let delta_x2 = velo + accel * 0.5 * delta_time;
    let oldpos = missile.pos;
    let newpos = oldpos + delta_x2 * delta_time;
    missile.pos = newpos;
    let newvelo = missile.velo + accel * delta_time;
    missile.velo = newvelo;
    missile.heading = next_heading;
    missile.target.x += -TARGET_VX * delta_time;
}

#[derive(Clone, Copy)]
struct MissileTape<'a> {
    h_thrust: TapeTerm<'a>,
    v_thrust: TapeTerm<'a>,
    pos: Vec2<TapeTerm<'a>>,
    velo: Vec2<TapeTerm<'a>>,
    heading: TapeTerm<'a>,
    accel: Vec2<TapeTerm<'a>>,
}

impl<'a> MissileTape<'a> {
    fn simulate_model(
        &mut self,
        tape: &'a Tape,
        c: &Constants<'a>,
        hist: &mut Vec<MissileTape<'a>>,
    ) {
        let thrust_vec = Vec2 {
            x: self.heading.apply("sin", |x| x.sin(), |x| x.cos()) * self.v_thrust,
            y: self.heading.apply("cos", |x| x.cos(), |x| -x.sin()) * self.v_thrust,
        };
        let next_heading = self.heading + self.h_thrust;
        let velolen2 = self.velo.x * self.velo.x + self.velo.y * self.velo.y;
        let velolen12 = velolen2.apply("sqrt", |x| x.sqrt(), |x| 1. / 2. * x.powf(-1. / 2.));
        self.accel = gravity(c.zero, c.gm) - self.velo * c.drag / velolen12 + thrust_vec;
        let delta_x2 = self.velo + self.accel * c.half;
        self.pos = self.pos + delta_x2;
        self.velo = self.velo + self.accel;
        self.heading = next_heading;
        self.h_thrust = tape.term(format!("h_thrust{}", hist.len()), 0.);
        self.v_thrust = tape.term(format!("v_thrust{}", hist.len()), 0.);
        hist.push(*self);
    }
}

/// Even constants need nodes in autograd
struct Constants<'a> {
    gm: TapeTerm<'a>,
    zero: TapeTerm<'a>,
    half: TapeTerm<'a>,
    drag: TapeTerm<'a>,
}

/// A model for the simulation state
struct Model<'a> {
    missile_hist: Vec<MissileTape<'a>>,
    target_hist: Vec<Vec2<TapeTerm<'a>>>,
    loss: TapeTerm<'a>,
}

fn get_model<'a>(tape: &'a Tape<f64>, pos: Vec2<f64>, heading: f64) -> Model<'a> {
    let missile = MissileTape {
        h_thrust: tape.term("h_thrust", 0.0),
        v_thrust: tape.term("v_thrust", 0.01),
        pos: Vec2 {
            x: tape.term("x1", pos.x),
            y: tape.term("y1", pos.y),
        },
        velo: Vec2 {
            x: tape.term("vx1", 0.05),
            y: tape.term("vy1", 0.05),
        },
        heading: tape.term("heading", heading),
        accel: Vec2 {
            x: tape.term("ax1", 0.),
            y: tape.term("ay1", 0.),
        },
    };

    let constants = Constants {
        gm: tape.term("GM", GM),
        zero: tape.term("0.0", 0.0),
        half: tape.term("0.5", 0.5),
        drag: tape.term("drag", DRAG),
    };

    let mut missile1 = missile;
    let mut missile_hist = vec![missile1];
    let mut target_hist = vec![];

    target_hist.push(Vec2 {
        x: tape.term("x2", 11.),
        y: tape.term("x2", 5.),
    });
    for t in 0..20 {
        missile1.simulate_model(tape, &constants, &mut missile_hist);
        target_hist.push(Vec2 {
            x: tape.term("x2", TARGET_X - TARGET_VX * (t as f64)),
            y: tape.term("x2", 5.),
        });
    }

    let loss = missile_hist
        .iter()
        .zip(target_hist.iter())
        .fold(None, |acc: Option<TapeTerm<'a>>, cur| {
            let diff = *cur.1 - cur.0.pos;
            let loss = diff.x * diff.x + diff.y * diff.y;
            if let Some(acc) = acc {
                Some(acc.apply_bin(loss, Box::new(MinOp)))
            } else {
                Some(loss)
            }
        })
        .unwrap();

    Model {
        missile_hist,
        target_hist,
        loss,
    }
}

fn gravity<'a>(zero: TapeTerm<'a>, gm: TapeTerm<'a>) -> Vec2<TapeTerm<'a>> {
    Vec2 { x: zero, y: -gm }
}
