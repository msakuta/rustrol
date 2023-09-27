//! Custom operators for rustograd

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
