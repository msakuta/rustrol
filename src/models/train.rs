use crate::{
    path_utils::{
        interpolate_path, interpolate_path_heading, wrap_angle, CircleArc, PathSegment,
        _bezier_interp, _bezier_length, wrap_angle_offset,
    },
    vec2::Vec2,
};

const CAR_LENGTH: f64 = 1.;
const TRAIN_ACCEL: f64 = 0.001;
const MAX_SPEED: f64 = 1.;
const THRUST_ACCEL: f64 = 0.001;
const MIN_RADIUS: f64 = 50.;
const SEGMENT_LENGTH: f64 = 10.;
pub(crate) const _C_POINTS: [Vec2<f64>; 11] = [
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

pub(crate) const PATH_SEGMENTS: [PathSegment; 4] = [
    PathSegment::Line([Vec2::new(0., 0.), Vec2::new(50., 0.)]),
    PathSegment::Line([Vec2::new(50., 0.), Vec2::new(200., 100.)]),
    PathSegment::Line([Vec2::new(200., 100.), Vec2::new(250., 100.)]),
    PathSegment::Arc(CircleArc::new(
        Vec2::new(250., 200.),
        100.,
        std::f64::consts::PI * 1.5,
        std::f64::consts::PI * 2.,
    )),
];

pub(crate) struct Station {
    pub name: String,
    pub s: f64,
}

pub(crate) struct Train {
    // pub control_points: Vec<Vec2<f64>>,
    pub path_segments: Vec<PathSegment>,
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
            // control_points: C_POINTS.to_vec(),
            path_segments: PATH_SEGMENTS.to_vec(),
            speed: 0.,
            s: 0.,
            track: compute_track_ps(&PATH_SEGMENTS),
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
        // self.track = compute_track(&self.control_points);
        self.track = compute_track_ps(&self.path_segments);
    }

    pub fn control_points(&self) -> Vec<Vec2<f64>> {
        self.path_segments.iter().map(|seg| seg.end()).collect()
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

    pub fn add_point(&mut self, pos: Vec2<f64>) {
        let Some(prev) = self.path_segments.last() else {
            return;
        };
        let prev_pos = prev.end();
        let prev_angle = prev.end_angle();
        let delta = pos - prev_pos;
        let normal = Vec2::new(-prev_angle.sin(), prev_angle.cos());
        let angle = delta.y.atan2(delta.x);
        let phi = wrap_angle(angle - prev_angle);
        let radius = delta.length() / 2. / phi.sin();
        let start = wrap_angle(prev_angle - radius.signum() * std::f64::consts::PI * 0.5);
        let end = start + phi * 2.;
        self.path_segments.push(PathSegment::Arc(CircleArc::new(
            prev_pos + normal * radius,
            radius.abs(),
            start,
            end,
        )))
    }

    pub fn add_straight(&mut self, pos: Vec2<f64>) -> Result<(), String> {
        let Some(prev) = self.path_segments.last() else {
            return Err("Path needs at least one segment to connect to".to_string());
        };
        let prev_pos = prev.end();
        let prev_angle = prev.end_angle();
        let delta = pos - prev_pos;
        let tangent = Vec2::new(prev_angle.cos(), prev_angle.sin());
        let dot = tangent.dot(delta);
        if dot < 0. {
            return Err(
                "Straight line cannot connect behind the current track direction".to_string(),
            );
        }
        let perpendicular_foot = prev_pos + tangent * dot;
        self.path_segments
            .push(PathSegment::Line([prev_pos, perpendicular_foot]));
        Ok(())
    }

    pub fn add_tight(&mut self, pos: Vec2<f64>) -> Result<(), String> {
        let Some(prev) = self.path_segments.last() else {
            return Err("Path needs at least one segment to connect to".to_string());
        };
        let prev_pos = prev.end();
        let prev_angle = prev.end_angle();
        let tangent = Vec2::new(prev_angle.cos(), prev_angle.sin());
        let normal_left = tangent.left90();
        let mut tangent_angle: Option<(f64, f64, Vec2<f64>)> = None;
        for cur in [-1., 1.] {
            let normal = normal_left * cur;
            let start_angle = (-normal.y).atan2(-normal.x);
            let arc_center = normal * MIN_RADIUS + prev_pos;
            let arc_dest = pos - arc_center;
            let arc_dest_len = arc_dest.length();
            if arc_dest_len < MIN_RADIUS {
                return Err(
                    "Clicked point requires tighter curvature radius than allowed".to_string(),
                );
            }
            let beta = (MIN_RADIUS / arc_dest_len).acos();
            let ad_angle = arc_dest.y.atan2(arc_dest.x);
            let angle = ad_angle - cur * beta;
            let end_angle = start_angle
                + wrap_angle_offset(angle - start_angle, (1. - cur) * std::f64::consts::PI);
            if let Some(acc) = tangent_angle {
                if (start_angle - acc.0).abs() < (start_angle - end_angle).abs() {
                } else {
                    tangent_angle = Some((end_angle, start_angle, arc_center));
                }
            } else {
                tangent_angle = Some((end_angle, start_angle, arc_center));
            }
        }
        if let Some((end_angle, start_angle, a)) = tangent_angle {
            let tangent_pos = a + Vec2::new(end_angle.cos(), end_angle.sin()) * MIN_RADIUS;
            self.path_segments.push(PathSegment::Arc(CircleArc::new(
                a,
                MIN_RADIUS,
                start_angle,
                end_angle,
            )));
            self.path_segments
                .push(PathSegment::Line([tangent_pos, pos]));
        } else {
            return Err("Cannot happen".to_string());
        }
        Ok(())
    }
}

fn _compute_track(control_points: &[Vec2<f64>]) -> Vec<Vec2<f64>> {
    let segment_lengths =
        control_points
            .windows(3)
            .enumerate()
            .fold(vec![], |mut acc, (cur_i, cur)| {
                if cur_i % 2 == 0 {
                    acc.push(_bezier_length(cur).unwrap_or(0.));
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
            _bezier_interp(
                &control_points[seg_idx * 2..seg_idx * 2 + 3],
                frem / seg_len,
            )
            .unwrap()
        })
        .collect()
}

fn compute_track_ps(path_segments: &[PathSegment]) -> Vec<Vec2<f64>> {
    let segment_lengths: Vec<_> = path_segments.iter().map(|seg| seg.length()).collect();
    let cumulative_lengths: Vec<f64> = segment_lengths.iter().fold(vec![], |mut acc, cur| {
        if let Some(v) = acc.last() {
            acc.push(v + *cur);
        } else {
            acc.push(*cur);
        }
        acc
    });
    let total_length = *cumulative_lengths.last().unwrap();
    let num_nodes = (total_length / SEGMENT_LENGTH) as usize + 1;
    println!("cumulative_lengths: {cumulative_lengths:?}, total_length: {total_length}");
    (0..=num_nodes)
        .map(|i| {
            let fidx = i as f64 * SEGMENT_LENGTH;
            let seg_idx = cumulative_lengths
                .iter()
                .enumerate()
                .find(|(_i, l)| fidx < **l)
                .map(|(i, _)| i)
                .unwrap_or_else(|| cumulative_lengths.len() - 1);
            let (frem, _cum_len) = if seg_idx == 0 {
                (fidx, 0.)
            } else {
                let cum_len = cumulative_lengths[seg_idx - 1];
                (fidx - cum_len, cum_len)
            };
            // println!("[{i}, {}, {}, {}],", seg_idx, frem, _cum_len + frem);
            let seg_len = segment_lengths[seg_idx];
            path_segments[seg_idx].interp((frem / seg_len).clamp(0., 1.))
        })
        .collect()
}
