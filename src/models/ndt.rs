use rustograd::{error::RustogradError, Tape};

use crate::vec2::Vec2;

pub const GRIS: usize = 20;
pub const GRIS2: usize = GRIS * 2 + 1;
pub const GRIR: f64 = 0.5;

pub struct Model {
    pub descent_rate: f64,
    pub points: Vec<Point>,
    pub gauss: Gauss,
    pub value_grid: Vec<f64>,
}

impl Model {
    pub fn new() -> Self {
        Self {
            descent_rate: 0.1,
            points: vec![],
            gauss: Gauss::new(),
            value_grid: vec![0.; GRIS2 * GRIS2],
        }
    }

    pub fn run_ndt(&mut self) -> Result<(), Box<dyn std::error::Error>> {
        self.calc_grad()?;

        for pt in &mut self.points {
            if let Some(ref grad) = pt.grad {
                self.gauss.mu.x += grad.mu.x * self.descent_rate;
                self.gauss.mu.y += grad.mu.y * self.descent_rate;
                for (val, grad) in self.gauss.sigma.iter_mut().zip(grad.sigma.iter()) {
                    *val += grad * self.descent_rate;
                }
            }
        }

        Ok(())
    }

    pub fn calc_grad(&mut self) -> Result<(), Box<dyn std::error::Error>> {
        let tape = Tape::new();
        let x = tape.term("x", 0.);
        let y = tape.term("y", 0.);
        let sigma00 = tape.term("sigma00", self.gauss.sigma[0]);
        let sigma01 = tape.term("sigma01", self.gauss.sigma[1]);
        let sigma10 = tape.term("sigma10", self.gauss.sigma[2]);
        let sigma11 = tape.term("sigma11", self.gauss.sigma[3]);
        let mu_x = tape.term("mu_x", self.gauss.mu.x);
        let mu_y = tape.term("mu_y", self.gauss.mu.y);
        let dx = x - mu_x;
        let dy = y - mu_y;

        let sigma_det = sigma00 * sigma11 - sigma10 * sigma01;
        let sigma_inv00 = sigma11 / sigma_det;
        let sigma_inv01 = -sigma01 / sigma_det;
        let sigma_inv10 = -sigma10 / sigma_det;
        let sigma_inv11 = sigma00 / sigma_det;

        let vec_sigma_x = dx * sigma_inv00 + dy * sigma_inv10;
        let vec_sigma_y = dx * sigma_inv01 + dy * sigma_inv11;

        let sigma_det_sqrt = sigma_det.apply("sqrt", f64::sqrt, |x| 0.5 / x.sqrt());

        let param = -(vec_sigma_x * dx + vec_sigma_y * dy);

        let gauss = param.apply("gauss", f64::exp, f64::exp) / sigma_det_sqrt;

        println!("tape has {} nodes", tape.len());

        for (i, val) in self.value_grid.iter_mut().enumerate() {
            let ix = i % GRIS2;
            let iy = i / GRIS2;
            let dx = (ix as f64 - GRIS as f64) * GRIR;
            let dy = (iy as f64 - GRIS as f64) * GRIR;
            x.set(dx)?;
            y.set(dy)?;
            *val = gauss.eval();
        }

        for pt in &mut self.points {
            x.set(pt.pos.x)?;
            y.set(pt.pos.y)?;
            let val = gauss.eval();
            gauss.backprop()?;
            let dmu_x = mu_x.grad().ok_or_else(|| "No grad")?;
            let dmu_y = mu_y.grad().ok_or_else(|| "No grad")?;
            let dsigma00 = sigma00.grad().ok_or_else(|| "No grad")?;
            let dsigma01 = sigma01.grad().ok_or_else(|| "No grad")?;
            let dsigma10 = sigma10.grad().ok_or_else(|| "No grad")?;
            let dsigma11 = sigma11.grad().ok_or_else(|| "No grad")?;
            pt.grad = Some(Gauss {
                mu: Vec2::new(dmu_x, dmu_y),
                sigma: [dsigma00, dsigma01, dsigma10, dsigma11],
            });
        }

        Ok(())
    }

    pub fn reset(&mut self) -> Result<(), RustogradError> {
        self.gauss = Gauss::new();
        Ok(())
    }

    pub fn plot_sigma(&self, radius: f64, mut f: impl FnMut(Vec2<f64>)) {
        let sigma00 = self.gauss.sigma[0];
        let sigma01 = self.gauss.sigma[1];
        let sigma10 = self.gauss.sigma[2];
        let sigma11 = self.gauss.sigma[3];

        let sigma_det = sigma00 * sigma11 - sigma10 * sigma01;
        let sigma_inv00 = sigma11 / sigma_det;
        let sigma_inv01 = -sigma01 / sigma_det;
        let sigma_inv10 = -sigma10 / sigma_det;
        let sigma_inv11 = sigma00 / sigma_det;

        for i in 0..=36 {
            let angle = std::f64::consts::PI * 2. * i as f64 / 36.;
            let x = angle.cos() * radius;
            let y = angle.sin() * radius;
            let tr_x = x * sigma00 + y * sigma10 + self.gauss.mu.x;
            let tr_y = x * sigma01 + y * sigma11 + self.gauss.mu.y;

            f(Vec2::new(tr_x, tr_y));
        }
    }

    pub fn plot_grid(&self, mut f: impl FnMut(Vec2<f64>, f64)) {
        for (i, val) in self.value_grid.iter().enumerate() {
            let ix = i % GRIS2;
            let iy = i / GRIS2;
            let x = (ix as f64 - GRIS as f64) * GRIR;
            let y = (iy as f64 - GRIS as f64) * GRIR;
            f(Vec2::new(x, y), *val);
        }
    }
}

pub struct Point {
    pub pos: Vec2<f64>,
    pub grad: Option<Gauss>,
}

pub struct Gauss {
    pub mu: Vec2<f64>,
    pub sigma: [f64; 4],
}

impl Gauss {
    pub fn new() -> Self {
        Self {
            mu: Vec2::zero(),
            sigma: [1., 0., 0., 1.],
        }
    }
}
