//! Custom operators for rustograd

pub(crate) struct AbsOp;

impl rustograd::UnaryFn<f64> for AbsOp {
    fn name(&self) -> String {
        "min".to_string()
    }

    fn f(&self, v: f64) -> f64 {
        v.abs()
    }

    fn grad(&self, v: f64) -> f64 {
        if v < 0. {
            -1.
        } else {
            1.
        }
    }
}

pub(crate) struct ClampOp {
    pub min: f64,
    pub max: f64,
}

impl rustograd::UnaryFn<f64> for ClampOp {
    fn name(&self) -> String {
        format!("clamp({}, {})", self.min, self.max)
    }

    fn f(&self, v: f64) -> f64 {
        v.clamp(self.min, self.max)
    }

    fn grad(&self, _v: f64) -> f64 {
        // Technically, clamp should return 0 to the gradient outside the range, but the solution
        // tend to get stuck into infeasible if it does.
        // if v < self.min || self.max < v {
        //     0.
        // } else {
        1.
        // }
    }
}

pub(crate) struct SinOp;

impl rustograd::UnaryFn<f64> for SinOp {
    fn name(&self) -> String {
        "sin".to_string()
    }

    fn f(&self, v: f64) -> f64 {
        v.sin()
    }

    fn grad(&self, v: f64) -> f64 {
        v.cos()
    }
}

pub(crate) struct TanOp;

impl rustograd::UnaryFn<f64> for TanOp {
    fn name(&self) -> String {
        "tan".to_string()
    }

    fn f(&self, v: f64) -> f64 {
        v.tan()
    }

    fn grad(&self, v: f64) -> f64 {
        1. / v.cos().powi(2)
    }
}

pub(crate) struct CosOp;

impl rustograd::UnaryFn<f64> for CosOp {
    fn name(&self) -> String {
        "cos".to_string()
    }

    fn f(&self, v: f64) -> f64 {
        v.cos()
    }

    fn grad(&self, v: f64) -> f64 {
        -v.sin()
    }
}

pub(crate) struct MinOp;

impl rustograd::BinaryFn<f64> for MinOp {
    fn name(&self) -> String {
        "min".to_string()
    }

    fn f(&self, lhs: f64, rhs: f64) -> f64 {
        lhs.min(rhs)
    }

    fn t(&self, data: f64) -> (f64, f64) {
        (data, data)
    }

    fn grad(&self, lhs: f64, rhs: f64) -> (f64, f64) {
        if lhs < rhs {
            (1., 0.)
        } else {
            (0., 1.)
        }
    }
}

pub(crate) struct MaxOp;

impl rustograd::BinaryFn<f64> for MaxOp {
    fn name(&self) -> String {
        "max".to_string()
    }

    fn f(&self, lhs: f64, rhs: f64) -> f64 {
        lhs.max(rhs)
    }

    fn t(&self, data: f64) -> (f64, f64) {
        (data, data)
    }

    fn grad(&self, lhs: f64, rhs: f64) -> (f64, f64) {
        if lhs < rhs {
            (0., 1.)
        } else {
            (1., 0.)
        }
    }
}
