use std::ops::Div;

use crate::vec2::Vec2;

use super::bicycle::{interpolate_path, interpolate_path_heading, spline_interp, spline_length};

const CAR_LENGTH: f64 = 1.;

const SEGMENT_LENGTH: f64 = 10.;
pub(crate) const C_POINTS: [Vec2<f64>; 9] = [
    Vec2::new(0., 0.),
    Vec2::new(50., 0.),
    Vec2::new(100., 0.),
    Vec2::new(200., 0.),
    Vec2::new(300., 100.),
    Vec2::new(400., 200.),
    Vec2::new(500., 200.),
    Vec2::new(550., 200.),
    Vec2::new(600., 200.),
];

pub(crate) struct Train {
    /// Position along the track
    pub s: f64,
    pub track: Vec<Vec2<f64>>,
}

impl Train {
    pub fn new() -> Self {
        let segment_lengths =
            C_POINTS
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
        let track = (0..num_nodes)
            .map(|i| {
                let fidx = i as f64 * SEGMENT_LENGTH;
                let (seg_idx, _) = cumulative_lengths
                    .iter()
                    .enumerate()
                    .find(|(_i, l)| fidx < **l)
                    .unwrap();
                let (frem, cum_len) = if seg_idx == 0 {
                    (fidx, 0.)
                } else {
                    let cum_len = cumulative_lengths[seg_idx - 1];
                    (fidx - cum_len, cum_len)
                };
                println!("[{}, {}, {}],", seg_idx, frem, cum_len + frem);
                let seg_len = segment_lengths[seg_idx];
                spline_interp(&C_POINTS[seg_idx * 2..seg_idx * 2 + 3], frem / seg_len).unwrap()
            })
            .collect();
        Self { s: 0., track }
    }

    pub fn pos(&self, car_idx: usize) -> Option<Vec2<f64>> {
        interpolate_path(&self.track, self.s - car_idx as f64 * CAR_LENGTH)
    }

    pub fn heading(&self, car_idx: usize) -> Option<f64> {
        interpolate_path_heading(&self.track, self.s - car_idx as f64 * CAR_LENGTH)
    }

    pub fn update(&mut self, thrust: f64) {
        self.s = (self.s + thrust).clamp(0., self.track.len() as f64);
    }
}
