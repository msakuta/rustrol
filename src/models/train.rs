mod path_bundle;

use std::collections::HashMap;

use crate::{
    path_utils::{
        interpolate_path, interpolate_path_heading, wrap_angle, wrap_angle_offset, CircleArc,
        PathSegment,
    },
    vec2::Vec2,
};

pub(crate) use path_bundle::PathBundle;

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
    pub path_id: usize,
    pub s: f64,
}

impl Station {
    pub fn new(name: impl Into<String>, s: f64) -> Self {
        Self {
            name: name.into(),
            path_id: 0,
            s,
        }
    }
}

#[derive(Clone, Copy, PartialEq, Eq)]
pub(crate) enum TrainTask {
    Idle,
    Goto(usize),
    Wait(usize),
}

pub(crate) struct Train {
    // pub control_points: Vec<Vec2<f64>>,
    /// A collection of paths. It could be a vec of `Rc`s, but we want serializable data structure.
    pub paths: HashMap<usize, PathBundle>,
    /// The next id of the path
    pub path_id_gen: usize,
    /// Build ghost segment, which is not actually built yet
    pub ghost_path: Option<PathBundle>,
    /// The index of the path_bundle that the train is on
    pub path_id: usize,
    /// Position along the track
    pub s: f64,
    /// Speed along s
    pub speed: f64,
    pub stations: Vec<Station>,
    pub train_task: TrainTask,
    pub schedule: Vec<usize>,
}

impl Train {
    pub fn new() -> Self {
        let mut paths = HashMap::new();
        paths.insert(0, PathBundle::multi(PATH_SEGMENTS.to_vec()));
        Self {
            // control_points: C_POINTS.to_vec(),
            paths,
            path_id_gen: 1,
            ghost_path: None,
            speed: 0.,
            s: 0.,
            path_id: 0,
            stations: vec![Station::new("Start", 10.), Station::new("Goal", 70.)],
            train_task: TrainTask::Idle,
            schedule: vec![],
        }
    }

    pub fn control_points(&self) -> Vec<Vec2<f64>> {
        self.paths
            .values()
            .map(|b| b.segments.iter())
            .flatten()
            .map(|seg| seg.end())
            .collect()
    }

    pub fn s_pos(&self, path_id: usize, s: f64) -> Option<Vec2<f64>> {
        interpolate_path(&self.paths.get(&path_id)?.track, s)
    }

    pub fn train_pos(&self, car_idx: usize) -> Option<Vec2<f64>> {
        interpolate_path(
            &self.paths[&self.path_id].track,
            self.s - car_idx as f64 * CAR_LENGTH,
        )
    }

    pub fn heading(&self, car_idx: usize) -> Option<f64> {
        interpolate_path_heading(
            &self.paths[&self.path_id].track,
            self.s - car_idx as f64 * CAR_LENGTH,
        )
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
        if self.paths[&self.path_id].track.len() as f64 <= self.s && 0. < self.speed {
            self.speed = 0.;
        }
        self.s = (self.s + self.speed).clamp(0., self.paths[&self.path_id].track.len() as f64);
    }

    pub fn add_gentle(&mut self, pos: Vec2<f64>) -> Result<(), String> {
        match self.compute_gentle(pos) {
            Ok(path_segments) => {
                if let Some(path) = self.paths.get_mut(&self.path_id) {
                    path.extend(&path_segments.segments);
                }
            }
            Err(e) => {
                self.ghost_path = None;
                return Err(e);
            }
        }
        Ok(())
    }

    pub fn ghost_gentle(&mut self, pos: Vec2<f64>) {
        self.ghost_path = self.compute_gentle(pos).ok();
    }

    fn compute_gentle(&self, pos: Vec2<f64>) -> Result<PathBundle, String> {
        let Some(prev) = self.paths[&self.path_id].segments.last() else {
            return Err("Path needs at least one segment to connect to".to_string());
        };
        let prev_pos = prev.end();
        let prev_angle = prev.end_angle();
        let delta = pos - prev_pos;
        let normal = Vec2::new(-prev_angle.sin(), prev_angle.cos());
        let angle = delta.y.atan2(delta.x);
        let phi = wrap_angle(angle - prev_angle);
        let radius = delta.length() / 2. / phi.sin();
        if radius < MIN_RADIUS {
            return Err("Clicked point requires tighter curvature radius than allowed".to_string());
        }
        let start = wrap_angle(prev_angle - radius.signum() * std::f64::consts::PI * 0.5);
        let end = start + phi * 2.;
        let path_segment = PathSegment::Arc(CircleArc::new(
            prev_pos + normal * radius,
            radius.abs(),
            start,
            end,
        ));
        Ok(PathBundle::single(path_segment))
    }

    pub fn add_straight(&mut self, pos: Vec2<f64>) -> Result<(), String> {
        let path_segments = self.compute_straight(pos)?;
        if let Some(path) = self.paths.get_mut(&self.path_id) {
            path.extend(&path_segments.segments);
        }
        Ok(())
    }

    pub fn ghost_straight(&mut self, pos: Vec2<f64>) {
        self.ghost_path = self.compute_straight(pos).ok();
    }

    fn compute_straight(&self, pos: Vec2<f64>) -> Result<PathBundle, String> {
        let Some(prev) = self.paths[&self.path_id].segments.last() else {
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
        Ok(PathBundle::single(path_segment))
    }

    pub fn add_tight(&mut self, pos: Vec2<f64>) -> Result<(), String> {
        let path_segments = self.compute_tight(pos)?;
        if let Some(path) = self.paths.get_mut(&self.path_id) {
            path.extend(&path_segments.segments);
        }
        Ok(())
    }

    pub fn ghost_tight(&mut self, pos: Vec2<f64>) {
        self.ghost_path = self.compute_tight(pos).ok();
    }

    fn compute_tight(&self, pos: Vec2<f64>) -> Result<PathBundle, String> {
        let Some(prev) = self.paths[&self.path_id].segments.last() else {
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
            Ok(PathBundle::multi(path_segments))
        } else {
            Err("Clicked point requires tighter curvature radius than allowed".to_string())
        }
    }

    /// Returns a tuple of (path id, global node id, segment id)
    pub fn find_path_node(&self, pos: Vec2<f64>, thresh: f64) -> Option<(usize, usize, usize)> {
        self.paths.iter().find_map(|(path_id, path)| {
            let (global_id, seg_id) = path.find_node(pos, thresh)?;
            Some((*path_id, global_id, seg_id))
        })
    }

    pub fn delete_segment(&mut self, pos: Vec2<f64>, dist_thresh: f64) -> Result<(), String> {
        let found_node = self.paths.iter_mut().find_map(|(id, path)| {
            let (node, seg) = path.find_node(pos, dist_thresh)?;
            Some(((id, path), seg, node))
        });
        if let Some(((&path_id, path), seg, i)) = found_node {
            if path_id == self.path_id && seg == path.find_seg_by_s(self.s as usize) {
                return Err("You can't delete a segment while a train is on it".to_string());
            }
            let new_path = path.delete_segment(i);
            let path_len = dbg!(path.segments.len());
            if let Some(new_path) = new_path {
                self.paths.insert(self.path_id_gen, new_path);
                self.path_id_gen += 1;

                let move_s = |path_id: &mut usize, s: &mut f64, name: &str| {
                    if i as f64 <= *s {
                        println!(
                            "Moving {name} path: {}, {}",
                            self.paths.len() - 1,
                            (*s - i as f64).max(0.)
                        );
                        *path_id = self.paths.len() - 1;
                        *s = (*s - i as f64).max(0.);
                    }
                };

                move_s(&mut self.path_id, &mut self.s, "train");
                for station in &mut self.stations {
                    move_s(
                        &mut station.path_id,
                        &mut station.s,
                        &format!("station {}", station.name),
                    );
                }
            }
            if path_len == 0 {
                println!("Path {path_id} length is 0! deleting path");
                self.paths.remove(&path_id);
            }
        } else {
            return Err("No segment is selected for deleting".to_string());
        }
        Ok(())
    }
}
