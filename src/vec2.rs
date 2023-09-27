/// A generic custom 2D vector type that can take f64 or TapeTerm as a type argument.
/// It can be a bit confusing with `egui::Vec2` so I may rename it.
#[derive(Clone, Copy, Debug)]
pub struct Vec2<T> {
    pub x: T,
    pub y: T,
}

impl<T: Copy> Vec2<T> {
    pub fn map<U>(&self, f: impl Fn(T) -> U) -> Vec2<U> {
        Vec2 {
            x: f(self.x),
            y: f(self.y),
        }
    }
}

impl<T> From<[T; 2]> for Vec2<T> {
    fn from([x, y]: [T; 2]) -> Self {
        Vec2 { x, y }
    }
}

impl<T: std::ops::Add<Output = T>> std::ops::Add for Vec2<T> {
    type Output = Self;
    fn add(self, rhs: Self) -> Self::Output {
        Self {
            x: self.x + rhs.x,
            y: self.y + rhs.y,
        }
    }
}

impl<T: std::ops::Sub<Output = T>> std::ops::Sub for Vec2<T> {
    type Output = Self;
    fn sub(self, rhs: Self) -> Self::Output {
        Self {
            x: self.x - rhs.x,
            y: self.y - rhs.y,
        }
    }
}

impl<T: std::ops::Mul<Output = T> + Copy> std::ops::Mul<T> for Vec2<T> {
    type Output = Self;
    fn mul(self, rhs: T) -> Self::Output {
        Self {
            x: self.x * rhs,
            y: self.y * rhs,
        }
    }
}

impl<T: std::ops::Div<Output = T> + Copy> std::ops::Div<T> for Vec2<T> {
    type Output = Self;
    fn div(self, rhs: T) -> Self::Output {
        Self {
            x: self.x / rhs,
            y: self.y / rhs,
        }
    }
}

impl<T: std::ops::Neg<Output = T>> std::ops::Neg for Vec2<T> {
    type Output = Self;
    fn neg(self) -> Self::Output {
        Self {
            x: -self.x,
            y: -self.y,
        }
    }
}
