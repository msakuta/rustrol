use crate::{error::GradDoesNotExist, vec2::Vec2};
use rustograd::{Tape, TapeTerm};

const RATE: f64 = 3e-4;

pub struct OrbitalParams {
    pub rate: f64,
    pub optim_iter: usize,
    pub max_iter: usize,
}

impl Default for OrbitalParams {
    fn default() -> Self {
        Self {
            rate: RATE,
            optim_iter: 60,
            max_iter: 60,
        }
    }
}

pub struct OrbitalState {
    pub pos: Vec2<f64>,
    pub velo: Vec2<f64>,
    pub target: Vec2<f64>,
}

#[derive(Default)]
pub struct OrbitalResult {
    pub before_optim: Vec<OrbitalState>,
    pub after_optim: Vec<OrbitalState>,
}

pub fn simulate_orbital(
    pos: Vec2<f64>,
    params: &OrbitalParams,
) -> Result<OrbitalResult, GradDoesNotExist> {
    let tape = Tape::new();
    let model = get_model(&tape, pos, params);

    let last_accel = model.accels.last().unwrap();
    let last_pos = model.xs.last().unwrap();

    // if let Ok(f) = std::fs::File::create("graph.dot") {
    //     let x2 = model.xs.first().unwrap();
    //     x2.x.dot_builder()
    //         .vertical(true)
    //         .output_term(x2.x, "x2.x")
    //         .output_term(x2.y, "x2.y")
    //         .dot(&mut std::io::BufWriter::new(f))
    //         .unwrap();
    // }

    println!("size: {}, xs: {}", tape.len(), model.xs.len());

    let v0 = model.vs.first().unwrap();

    model.loss.eval();
    model.loss.backprop().unwrap();
    let xd = try_grad!(v0.x);
    let yd = try_grad!(v0.y);

    let eval = [last_pos.x.eval(), last_pos.y.eval()];
    println!(
        "eval: {eval:?}, accel.eval: {:?}",
        [last_accel.x.eval(), last_accel.y.eval()]
    );
    println!("simulate_orbital: derive(vx, vy): {:?}, {:?}", xd, yd);

    let before_optim = model
        .xs
        .iter()
        .zip(model.vs.iter())
        .map(|(pos, velo)| OrbitalState {
            pos: pos.map(|x| x.eval()),
            velo: velo.map(|x| x.eval()),
            target: model.target.map(|x| x.eval_noclear()),
        })
        .collect();

    const RATE: f64 = 5e-5;

    // optimization loop
    for i in 0..params.optim_iter {
        model.loss.eval();
        model.loss.backprop().unwrap();
        let xd = v0.x.grad().unwrap();
        let yd = v0.y.grad().unwrap();
        let first_velo = model.vs.first().unwrap();
        first_velo
            .x
            .set(first_velo.x.data().unwrap() - xd * RATE)
            .unwrap();
        first_velo
            .y
            .set(first_velo.y.data().unwrap() - yd * RATE)
            .unwrap();
        let loss_val = model.loss.eval();
        println!(
            "simulate_orbital optimize[{i}]: derive(vx, vy): {:?}, {:?}, loss: {}",
            xd, yd, loss_val
        );
    }

    Ok(OrbitalResult {
        before_optim,
        after_optim: model
            .xs
            .iter()
            .zip(model.vs.iter())
            .map(|(pos, velo)| OrbitalState {
                pos: pos.map(|x| x.eval()),
                velo: velo.map(|x| x.eval()),
                target: model.target.map(|x| x.eval_noclear()),
            })
            .collect(),
    })
}

const GM: f64 = 0.03;

struct Model<'a> {
    accels: Vec<Vec2<TapeTerm<'a>>>,
    vs: Vec<Vec2<TapeTerm<'a>>>,
    xs: Vec<Vec2<TapeTerm<'a>>>,
    target: Vec2<TapeTerm<'a>>,
    loss: TapeTerm<'a>,
}

fn get_model<'a>(tape: &'a Tape<f64>, initial_pos: Vec2<f64>, params: &OrbitalParams) -> Model<'a> {
    let mut pos = Vec2 {
        x: tape.term("x", initial_pos.x),
        y: tape.term("y", initial_pos.y),
    };
    let mut vx = Vec2 {
        x: tape.term("vx", 0.),
        y: tape.term("vy", 0.15),
    };
    let earth = Vec2 {
        x: tape.term("bx", 0.),
        y: tape.term("by", 0.),
    };

    let gm = tape.term("GM", GM);

    let half = tape.term("0.5", 0.5);
    let mut accels = vec![];
    let mut vs = vec![vx];
    let mut xs = vec![pos];
    for _ in 0..params.max_iter {
        let accel = gravity(earth, pos, gm);
        let delta_x = vx + accel * half;
        let accel2 = gravity(earth, pos + delta_x * half, gm);
        let delta_x2 = vx + accel * half;
        pos = pos + delta_x2;
        accels.push(accel2);
        xs.push(pos);
        vx = vx + accel2;
        vs.push(vx);
    }

    let target = Vec2 {
        x: tape.term("target_x", -1.5),
        y: tape.term("target_y", 0.),
    };
    let last_pos = xs[18];
    let diff = last_pos - target;
    let loss = diff.x * diff.x + diff.y * diff.y;

    Model {
        accels,
        xs,
        vs,
        target,
        loss,
    }
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
