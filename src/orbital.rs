mod orbit;
mod three_body;

pub use self::orbit::{
    orbital_simulate_step, simulate_orbital, OrbitalResult, OrbitalState, ORBITAL_STATE,
};
pub use self::three_body::{
    calc_initial_moon, simulate_three_body, three_body_simulate_step, ThreeBodyParams,
    ThreeBodyResult, ThreeBodyState, THREE_BODY_STATE,
};
use crate::{error::GradDoesNotExist, vec2::Vec2};
use rustograd::{error::RustogradError, TapeTerm};

type Vec2d = Vec2<f64>;
type Vec2t<'a> = Vec2<TapeTerm<'a>>;

const RATE: f64 = 3e-4;
const THRUST_ACCEL: f64 = 0.001;
const EARTH_POS: Vec2<f64> = Vec2 { x: 0., y: 0. };
pub const GM: f64 = 0.03;
pub const GM_MOON: f64 = GM / 3.;

#[derive(Clone, Copy)]
pub struct OrbitalParams {
    pub initial_pos: Vec2<f64>,
    pub initial_velo: Vec2<f64>,
    pub earth_pos: Vec2<f64>,
    pub earth_gm: f64,
    pub three_body: Option<ThreeBodyParams>,
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
            three_body: None,
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

fn optimize<'a, S: AbstractModelState<'a>>(
    model: &'a Model<'a, S>,
    velo: &Vec2<f64>,
    params: &OrbitalParams,
) -> Result<(Vec2<f64>, f64), Box<dyn std::error::Error>> {
    const RATE: f64 = 1e-5;

    let first_state = model.states.first().unwrap();
    first_state.set_velo(velo)?;

    let mut loss_val = 0.;

    let first_velo = first_state.get_velo();

    // optimization loop
    for _i in 0..params.optim_iter {
        model.loss.eval();
        model.loss.backprop().unwrap();
        let xd = try_grad!(first_velo.x);
        let yd = try_grad!(first_velo.y);
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
        x: first_velo.x.data().unwrap(),
        y: first_velo.y.data().unwrap(),
    };

    let velo_loss = (params.initial_velo - optimized_velo).length2();

    Ok((
        optimized_velo,
        loss_val + velo_loss * params.initial_velo_weight,
    ))
}

fn simulate_step(
    bodies: &[&Body],
    pos: Vec2d,
    velo: Vec2d,
    thrust: Vec2d,
    delta_time: f64,
) -> (Vec2d, Vec2d, Vec2d) {
    let accel = gravity_f64(bodies, pos) + thrust;
    let delta_x = (velo + accel * 0.5 * delta_time) * delta_time;
    let accel2 = gravity_f64(bodies, pos + delta_x * 0.5) + thrust;
    let delta_x2 = (velo + accel2 * 0.5 * delta_time) * delta_time;
    let newpos = pos + delta_x2;
    let newvelo = velo + accel2 * delta_time;
    (newpos, newvelo, accel2)
}

#[derive(Clone, Copy)]
struct BodyModelState<'a> {
    pos: Vec2<TapeTerm<'a>>,
    gm: TapeTerm<'a>,
}

fn model_simulate_step<'a>(
    pos: Vec2t<'a>,
    vx: Vec2t<'a>,
    bodies: &[BodyModelState<'a>],
    half: TapeTerm<'a>,
) -> (Vec2t<'a>, Vec2t<'a>, Vec2t<'a>) {
    let accel = gravity(bodies, pos).unwrap();
    let delta_x = vx + accel * half;
    let accel2 = gravity(bodies, pos + delta_x * half).unwrap();
    let delta_x2 = vx + accel2 * half;
    let pos = pos + delta_x2;
    let vx = vx + accel2;
    (pos, vx, accel2)
}

struct Model<'a, S: AbstractModelState<'a>> {
    states: Vec<S>,
    loss: TapeTerm<'a>,
}

trait AbstractModelState<'a> {
    fn get_velo(&self) -> Vec2t<'a>;
    fn set_velo(&self, velo: &Vec2d) -> Result<(), RustogradError>;
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
    target_r: TapeTerm<'a>,
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
    bodies: &[BodyModelState<'a>],
    pos: Vec2<TapeTerm<'a>>,
) -> Option<Vec2<TapeTerm<'a>>> {
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
    bodies.iter().fold(None, |acc, cur| {
        Some(if let Some(acc) = acc {
            acc + gravity_body(cur.pos) * cur.gm
        } else {
            gravity_body(cur.pos) * cur.gm
        })
    })
}

struct Body {
    pos: Vec2<f64>,
    gm: f64,
}

fn gravity_f64(bodies: &[&Body], pos: Vec2<f64>) -> Vec2<f64> {
    let gravity_body = |body: &Body| {
        let diff = body.pos - pos;
        let len2 = diff.x * diff.x + diff.y * diff.y;
        let len32 = len2.powf(3. / 2.);
        diff / len32 * body.gm
    };
    bodies
        .iter()
        .fold(Vec2::zero(), |acc, cur| acc + gravity_body(cur))
}
