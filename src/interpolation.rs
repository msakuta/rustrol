use cgmath::{MetricSpace, Vector2};

pub fn lerp(a: &[f64; 2], b: &[f64; 2], f: f64) -> [f64; 2] {
    [a[0] * (1. - f) + b[0] * f, a[1] * (1. - f) + b[1] * f]
}

pub(crate) trait AsPoint {
    fn as_point(&self) -> [f64; 2];
}

/// Linearly interpolatable point-like object.
///
/// This trait is not object safe, thus made a separate trait from [`AsPoint`].
pub(crate) trait LerpPoint: AsPoint {
    fn lerp(&self, other: &Self, f: f64) -> Self;
}

/// Collision checking with linear interpolation. A closure to check the collision must be provided.
/// The closure shall return true if it has collision, and will be called multiple times to interpolate the range.
/// The function will return early if the closure returns true.
pub(crate) fn interpolate<P: LerpPoint>(
    start: P,
    target: P,
    interval: f64,
    mut f: impl FnMut(P) -> bool,
) -> bool {
    let start_p = start.as_point();
    let target_p = target.as_point();
    let distance = Vector2::from(start_p).distance(Vector2::from(target_p));
    let interpolates = (distance.abs() / interval).floor() as usize + 1;
    for i in 0..=interpolates {
        let point = start.lerp(&target, i as f64 / interpolates as f64);
        if f(point) {
            return true;
        }
    }
    return false;
}
