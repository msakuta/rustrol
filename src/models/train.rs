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
    PathSegment::Line([Vec2::new(50., 0.), Vec2::new(500., 200.)]),
    PathSegment::Line([Vec2::new(500., 200.), Vec2::new(550., 200.)]),
    PathSegment::Arc(CircleArc::new(
        Vec2::new(550., 300.),
        100.,
        std::f64::consts::PI * 1.5,
        std::f64::consts::PI * 2.,
    )),
];

pub(crate) struct Station {
    pub name: String,
    pub s: f64,
}

#[derive(Clone, Copy, PartialEq, Eq)]
pub(crate) enum TrainTask {
    Idle,
    Goto(usize),
    Wait(usize),
}

pub(crate) struct Train {
    // pub control_points: Vec<Vec2<f64>>,
    pub path_segments: Vec<PathSegment>,
    /// Build ghost segment, which is not actually built yet
    pub ghost_segment: Option<(Vec<PathSegment>, Vec<Vec2<f64>>)>,
    /// Position along the track
    pub s: f64,
    /// Speed along s
    pub speed: f64,
    /// Interpolated points along the track in the interval SEGMENT_LENGTH
    pub track: Vec<Vec2<f64>>,
    pub stations: Vec<Station>,
    pub train_task: TrainTask,
    pub schedule: Vec<usize>,
}

impl Train {
    pub fn new() -> Self {
        Self {
            // control_points: C_POINTS.to_vec(),
            path_segments: PATH_SEGMENTS.to_vec(),
            ghost_segment: None,
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
            train_task: TrainTask::Idle,
            schedule: vec![],
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
        if let TrainTask::Wait(timer) = &mut self.train_task {
            *timer -= 1;
            if *timer <= 1 {
                self.train_task = TrainTask::Idle;
            }
        }
        if matches!(self.train_task, TrainTask::Idle) {
            if let Some(first) = self.schedule.first().copied() {
                self.train_task = TrainTask::Goto(first);
                self.schedule.remove(0);
                self.schedule.push(first);
            }
        }
        if let TrainTask::Goto(target) = self.train_task {
            let target_s = self.stations[target].s;
            if (target_s - self.s).abs() < 1. && self.speed.abs() < TRAIN_ACCEL {
                self.speed = 0.;
                self.train_task = TrainTask::Wait(120);
            } else if target_s < self.s {
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

    pub fn add_point(&mut self, pos: Vec2<f64>) -> Result<(), String> {
        match self.compute_point(pos) {
            Ok((path_segment, _)) => self.path_segments.push(path_segment),
            Err(e) => {
                self.ghost_segment = None;
                return Err(e);
            }
        }
        Ok(())
    }

    pub fn ghost_point(&mut self, pos: Vec2<f64>) {
        self.ghost_segment = self
            .compute_point(pos)
            .ok()
            .map(|(path_segment, track)| (vec![path_segment], track));
    }

    fn compute_point(&self, pos: Vec2<f64>) -> Result<(PathSegment, Vec<Vec2<f64>>), String> {
        let Some(prev) = self.path_segments.last() else {
            return Err("Path needs at least one segment to connect to".to_string());
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
        let path_segment = PathSegment::Arc(CircleArc::new(
            prev_pos + normal * radius,
            radius.abs(),
            start,
            end,
        ));
        let track = compute_track_ps(&[path_segment]);
        Ok((path_segment, track))
    }

    pub fn add_straight(&mut self, pos: Vec2<f64>) -> Result<(), String> {
        let (path_segment, _) = self.compute_straight(pos)?;
        self.path_segments.push(path_segment);
        Ok(())
    }

    pub fn ghost_straight(&mut self, pos: Vec2<f64>) {
        self.ghost_segment = self
            .compute_straight(pos)
            .ok()
            .map(|(path_segment, track)| (vec![path_segment], track));
    }

    fn compute_straight(&self, pos: Vec2<f64>) -> Result<(PathSegment, Vec<Vec2<f64>>), String> {
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
        let path_segment = PathSegment::Line([prev_pos, perpendicular_foot]);
        let track = compute_track_ps(&[path_segment]);
        Ok((path_segment, track))
    }

    pub fn add_tight(&mut self, pos: Vec2<f64>) -> Result<(), String> {
        let (path_segments, _) = self.compute_tight(pos)?;
        self.path_segments.extend_from_slice(&path_segments);
        Ok(())
    }

    pub fn ghost_tight(&mut self, pos: Vec2<f64>) {
        self.ghost_segment = self
            .compute_tight(pos)
            .ok()
            .map(|(path_segments, track)| (path_segments.to_vec(), track));
    }

    fn compute_tight(&self, pos: Vec2<f64>) -> Result<([PathSegment; 2], Vec<Vec2<f64>>), String> {
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
                continue;
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
            let path_segments = [
                PathSegment::Arc(CircleArc::new(a, MIN_RADIUS, start_angle, end_angle)),
                PathSegment::Line([tangent_pos, pos]),
            ];
            let track = compute_track_ps(&path_segments);
            Ok((path_segments, track))
        } else {
            Err("Clicked point requires tighter curvature radius than allowed".to_string())
        }
    }

    pub fn delete_node(&mut self, pos: Vec2<f64>, dist_thresh: f64) -> Result<(), String> {
        if let Some((i, _)) = self.find_node(pos, dist_thresh) {
            if i == self.path_segments.len() - 1 {
                self.path_segments.pop();
            } else {
                return Err("Only deleting the last node is supported yet".to_string());
            }
        } else {
            return Err("No node is selected for deleting".to_string());
        }
        Ok(())
    }

    pub fn find_node(&self, pos: Vec2<f64>, dist_thresh: f64) -> Option<(usize, Vec2<f64>)> {
        let dist2_thresh = dist_thresh.powi(2);
        let closest_node: Option<(usize, f64)> =
            self.path_segments
                .iter()
                .enumerate()
                .fold(None, |acc, cur| {
                    let dist2 = (cur.1.end() - pos).length2();
                    if let Some(acc) = acc {
                        if acc.1 < dist2 {
                            Some(acc)
                        } else {
                            Some((cur.0, dist2))
                        }
                    } else if dist2 < dist2_thresh {
                        Some((cur.0, dist2))
                    } else {
                        None
                    }
                });
        closest_node.map(|(i, _)| (i, self.path_segments[i].end()))
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
    (0..=num_nodes)
        .filter_map(|i| {
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
            let seg_len = segment_lengths[seg_idx];
            path_segments[seg_idx].interp((frem / seg_len).clamp(0., 1.))
        })
        .collect()
}
