use std::collections::VecDeque;

use crate::vec2::Vec2;

pub(crate) const MAX_THRUST: f64 = 0.5;
pub(crate) const MAX_STEERING: f64 = std::f64::consts::PI / 4.;
pub(crate) const STEERING_SPEED: f64 = std::f64::consts::PI * 0.01;

pub(crate) struct Bicycle {
    pub pos: Vec2<f64>,
    pub heading: f64,
    pub steering: f64,
    pub wheel_base: f64,
    pub pos_history: VecDeque<Vec2<f64>>,
}

impl Bicycle {
    const MAX_HISTORY: usize = 1000;
    pub fn new() -> Self {
        Self {
            pos: Vec2::zero(),
            heading: 0.,
            steering: 0.,
            wheel_base: 4.,
            pos_history: VecDeque::new(),
        }
    }

    pub fn append_history(&mut self) {
        if self
            .pos_history
            .back()
            .map(|hist| *hist != self.pos)
            .unwrap_or(true)
        {
            self.pos_history.push_back(self.pos);
        }
        if Self::MAX_HISTORY < self.pos_history.len() {
            self.pos_history.pop_front();
        }
    }
}

pub(crate) fn bicycle_simulate_step(
    bicycle: &mut Bicycle,
    h_thrust: f64,
    v_thrust: f64,
    playback_speed: f64,
) {
    let steering_speed = playback_speed * STEERING_SPEED;
    bicycle.steering = if (bicycle.steering - h_thrust).abs() < steering_speed {
        h_thrust
    } else if bicycle.steering < h_thrust {
        bicycle.steering + steering_speed
    } else {
        bicycle.steering - steering_speed
    };
    let direction = Vec2::new(bicycle.heading.cos(), -bicycle.heading.sin());
    bicycle.pos += direction * v_thrust * playback_speed;

    let theta_dot = v_thrust * bicycle.steering.tan() / bicycle.wheel_base;
    bicycle.heading += theta_dot * playback_speed;
}
