use rustograd::{Tape, TapeTerm};

use crate::{ops::MinOp, vec2::Vec2, xor128::Xor128};

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

pub struct MissileState {
    pub pos: Vec2<f64>,
    pub target: Vec2<f64>,
    pub thrust: f64,
    pub heading: f64,
    pub prediction: Vec<Vec2<f64>>,
    pub target_prediction: Vec<Vec2<f64>>,
}

pub fn simulate_missile(
    pos: Vec2<f64>,
    params: &MissileParams,
) -> Result<Vec<MissileState>, String> {
    let tape = Tape::new();
    let model = get_model(&tape, pos);

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
    Ok((0..params.max_iter)
        .map(|t| {
            let (thrust, heading) = optimize(&model, t, params);
            let (pos, target) = simulate_step(&model, &mut rng, t, heading, thrust);
            MissileState {
                pos,
                target,
                thrust,
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
            }
        })
        .collect())
}

const MAX_THRUST: f64 = 0.09;
const RATE: f64 = 3e-4;
const GM: f64 = 0.03;
const DRAG: f64 = 0.05;
const TARGET_X: f64 = 20.;
const TARGET_VX: f64 = 0.5;

fn optimize(model: &Model, t: usize, params: &MissileParams) -> (f64, f64) {
    for (t2, target) in model.target_hist.iter().enumerate() {
        target
            .x
            .set(TARGET_X - TARGET_VX * (t + t2) as f64)
            .unwrap();
        target.y.set(5.).unwrap();
        target.x.eval();
        target.y.eval();
    }

    let mut d_thrust = 0.;
    let mut d_heading = 0.;
    let mut thrust = 0.;
    let mut heading = 0.;

    for _ in 0..params.optim_iter {
        model.loss.eval();
        model.loss.backprop().unwrap();
        d_thrust = model.thrust.grad().unwrap();
        d_heading = model.heading.grad().unwrap();
        thrust = (model.thrust.data().unwrap() - d_thrust * RATE)
            .min(MAX_THRUST)
            .max(0.);
        model.thrust.set(thrust).unwrap();
        heading = model.heading.data().unwrap() - d_heading * RATE;
        model.heading.set(heading).unwrap();
    }

    let loss_val = model.loss.eval_noclear();
    println!(
        "thrust: {}, heading: {}, loss: {}",
        d_thrust, d_heading, loss_val
    );

    (thrust, heading)
}

fn simulate_step(
    model: &Model,
    rng: &mut Xor128,
    t: usize,
    heading: f64,
    thrust: f64,
) -> (Vec2<f64>, Vec2<f64>) {
    let thrust_vec = Vec2::<f64> {
        x: heading.sin() * thrust,
        y: heading.cos() * thrust,
    };
    let missile = model.missile_hist.first().unwrap();
    let velo = missile.velo.map(|x| x.data().unwrap());
    let velolen2 = velo.x * velo.x + velo.y * velo.y;
    let velolen12 = velolen2.sqrt();
    let randomize = Vec2 {
        x: rng.next() - 0.5,
        y: rng.next() - 0.5,
    };
    let accel =
        Vec2::<f64> { x: 0., y: -GM } + randomize * 0.02 - velo * DRAG / velolen12 + thrust_vec;
    let delta_x2 = velo + accel * 0.5;
    let oldpos = missile.pos.map(|x| x.data().unwrap());
    let newpos = oldpos + delta_x2;
    missile.pos.x.set(newpos.x).unwrap();
    missile.pos.y.set(newpos.y).unwrap();
    let newvelo = missile.velo.map(|x| x.data().unwrap()) + accel;
    missile.velo.x.set(newvelo.x).unwrap();
    missile.velo.y.set(newvelo.y).unwrap();
    for (t2, target) in model.target_hist.iter().enumerate() {
        target
            .x
            .set(TARGET_X - TARGET_VX * (t + t2) as f64)
            .unwrap();
    }
    let target_pos = model.target_hist.first().unwrap().map(|x| x.eval());
    (oldpos, target_pos)
}

#[derive(Clone, Copy)]
struct Missile<'a> {
    pos: Vec2<TapeTerm<'a>>,
    velo: Vec2<TapeTerm<'a>>,
    accel: Vec2<TapeTerm<'a>>,
}

impl<'a> Missile<'a> {
    fn simulate_model(
        &mut self,
        heading: TapeTerm<'a>,
        thrust: TapeTerm<'a>,
        c: &Constants<'a>,
        hist: &mut Vec<Missile<'a>>,
    ) {
        let thrust_vec = Vec2 {
            x: heading.apply("sin", |x| x.sin(), |x| x.cos()) * thrust,
            y: heading.apply("cos", |x| x.cos(), |x| -x.sin()) * thrust,
        };
        let velolen2 = self.velo.x * self.velo.x + self.velo.y * self.velo.y;
        let velolen12 = velolen2.apply("sqrt", |x| x.sqrt(), |x| 1. / 2. * x.powf(-1. / 2.));
        self.accel = gravity(c.zero, c.gm) - self.velo * c.drag / velolen12 + thrust_vec;
        let delta_x2 = self.velo + self.accel * c.half;
        self.pos = self.pos + delta_x2;
        self.velo = self.velo + self.accel;
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
    missile_hist: Vec<Missile<'a>>,
    target_hist: Vec<Vec2<TapeTerm<'a>>>,
    heading: TapeTerm<'a>,
    thrust: TapeTerm<'a>,
    loss: TapeTerm<'a>,
}

fn get_model<'a>(tape: &'a Tape<f64>, pos: Vec2<f64>) -> Model<'a> {
    let heading = tape.term("heading", std::f64::consts::PI / 4.);
    let thrust = tape.term("thrust", 0.01);
    let missile = Missile {
        pos: Vec2 {
            x: tape.term("x1", pos.x),
            y: tape.term("y1", pos.y),
        },
        velo: Vec2 {
            x: tape.term("vx1", 0.05),
            y: tape.term("vy1", 0.05),
        },
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
    let mut hist1 = vec![missile1];
    let mut hist2 = vec![];

    hist2.push(Vec2 {
        x: tape.term("x2", 11.),
        y: tape.term("x2", 5.),
    });
    for t in 0..20 {
        missile1.simulate_model(heading, thrust, &constants, &mut hist1);
        hist2.push(Vec2 {
            x: tape.term("x2", TARGET_X - TARGET_VX * (t as f64)),
            y: tape.term("x2", 5.),
        });
    }

    let loss = hist1
        .iter()
        .zip(hist2.iter())
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
        missile_hist: hist1,
        target_hist: hist2,
        thrust,
        heading,
        loss,
    }
}

fn gravity<'a>(zero: TapeTerm<'a>, gm: TapeTerm<'a>) -> Vec2<TapeTerm<'a>> {
    Vec2 { x: zero, y: -gm }
}
