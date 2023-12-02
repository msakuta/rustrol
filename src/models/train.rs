use crate::vec2::Vec2;

use super::bicycle::{interpolate_path, interpolate_path_heading, spline_interp, spline_length};

const CAR_LENGTH: f64 = 1.;
const TRAIN_ACCEL: f64 = 0.001;
const MAX_SPEED: f64 = 1.;
const THRUST_ACCEL: f64 = 0.001;

const SEGMENT_LENGTH: f64 = 10.;
pub(crate) const C_POINTS: [Vec2<f64>; 11] = [
    Vec2::new(0., 0.),
    Vec2::new(50., 0.),
    Vec2::new(100., 0.),
    Vec2::new(200., 0.),
    Vec2::new(300., 100.),
    Vec2::new(400., 200.),
    Vec2::new(500., 200.),
    Vec2::new(550., 200.),
    Vec2::new(600., 200.),
    Vec2::new(700., 200.),
    Vec2::new(700., 100.),
];

pub(crate) struct Station {
    pub name: String,
    pub s: f64,
}

pub(crate) struct Train {
    pub control_points: Vec<Vec2<f64>>,
    /// Position along the track
    pub s: f64,
    /// Speed along s
    pub speed: f64,
    /// Interpolated points along the track in the interval SEGMENT_LENGTH
    pub track: Vec<Vec2<f64>>,
    pub stations: Vec<Station>,
    pub target_station: Option<usize>,
}

impl Train {
    pub fn new() -> Self {
        Self {
            control_points: C_POINTS.to_vec(),
            speed: 0.,
            s: 0.,
            track: compute_track(&C_POINTS),
            stations: vec![
                Station {
                    name: "Start".to_string(),
                    s: 10.,
                },
                Station {
                    name: "Goal".to_string(),
                    s: 70.,
                },
            ],
            target_station: None,
        }
    }

    pub fn recompute_track(&mut self) {
        self.track = compute_track(&self.control_points);
    }

    pub fn s_pos(&self, s: f64) -> Option<Vec2<f64>> {
        interpolate_path(&self.track, s)
    }

    pub fn train_pos(&self, car_idx: usize) -> Option<Vec2<f64>> {
        interpolate_path(&self.track, self.s - car_idx as f64 * CAR_LENGTH)
    }

    pub fn heading(&self, car_idx: usize) -> Option<f64> {
        interpolate_path_heading(&self.track, self.s - car_idx as f64 * CAR_LENGTH)
    }

    pub fn update(&mut self, thrust: f64) {
        if let Some(target) = self.target_station {
            let target_s = self.stations[target].s;
            if target_s < self.s {
                // speed / accel == t
                // speed * t / 2 == speed^2 / accel / 2 == dist
                // accel = sqrt(2 * dist)
                if self.speed < 0. && self.s - target_s < 0.5 * self.speed.powi(2) / TRAIN_ACCEL {
                    self.speed += TRAIN_ACCEL;
                } else {
                    self.speed -= TRAIN_ACCEL;
                }
            } else {
                if 0. < self.speed && target_s - self.s < 0.5 * self.speed.powi(2) / TRAIN_ACCEL {
                    self.speed -= TRAIN_ACCEL;
                } else {
                    self.speed += TRAIN_ACCEL;
                }
            }
        }
        self.speed = (self.speed + thrust * THRUST_ACCEL).clamp(-MAX_SPEED, MAX_SPEED);
        if self.s == 0. && self.speed < 0. {
            self.speed = 0.;
        }
        if self.track.len() as f64 <= self.s && 0. < self.speed {
            self.speed = 0.;
        }
        self.s = (self.s + self.speed).clamp(0., self.track.len() as f64);
    }
}

fn compute_track(control_points: &[Vec2<f64>]) -> Vec<Vec2<f64>> {
    let segment_lengths =
        control_points
            .windows(3)
            .enumerate()
            .fold(vec![], |mut acc, (cur_i, cur)| {
                if cur_i % 2 == 0 {
                    acc.push(spline_length(cur).unwrap_or(0.));
                }
                acc
            });
    let cumulative_lengths: Vec<f64> = segment_lengths.iter().fold(vec![], |mut acc, cur| {
        if let Some(v) = acc.last() {
            acc.push(v + *cur);
        } else {
            acc.push(*cur);
        }
        acc
    });
    let total_length: f64 = segment_lengths.iter().copied().sum();
    let num_nodes = (total_length / SEGMENT_LENGTH) as usize;
    println!("cumulative_lengths: {cumulative_lengths:?}");
    (0..num_nodes)
        .map(|i| {
            let fidx = i as f64 * SEGMENT_LENGTH;
            let (seg_idx, _) = cumulative_lengths
                .iter()
                .enumerate()
                .find(|(_i, l)| fidx < **l)
                .unwrap();
            let (frem, _cum_len) = if seg_idx == 0 {
                (fidx, 0.)
            } else {
                let cum_len = cumulative_lengths[seg_idx - 1];
                (fidx - cum_len, cum_len)
            };
            // println!("[{}, {}, {}],", seg_idx, frem, cum_len + frem);
            let seg_len = segment_lengths[seg_idx];
            spline_interp(
                &control_points[seg_idx * 2..seg_idx * 2 + 3],
                frem / seg_len,
            )
            .unwrap()
        })
        .collect()
}