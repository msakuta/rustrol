use crate::{
    path_utils::{PathSegment, _bezier_interp, _bezier_length},
    vec2::Vec2,
};

use super::SEGMENT_LENGTH;

pub(crate) struct PathBundle {
    pub(super) segments: Vec<PathSegment>,
    /// Interpolated points along the track in the interval SEGMENT_LENGTH
    pub track: Vec<Vec2<f64>>,
    pub(super) track_ranges: Vec<usize>,
}

impl PathBundle {
    pub(super) fn single(path_segment: PathSegment) -> Self {
        let track = compute_track_ps(&[path_segment]);
        let track_len = track.len();
        PathBundle {
            segments: vec![path_segment],
            track,
            track_ranges: vec![track_len],
        }
    }

    pub(super) fn multi(path_segments: impl Into<Vec<PathSegment>>) -> Self {
        let path_segments = path_segments.into();
        let track = compute_track_ps(&path_segments);
        let track_len = track.len();
        PathBundle {
            segments: path_segments,
            track,
            track_ranges: vec![track_len],
        }
    }

    pub fn extend(&mut self, path_segments: &[PathSegment]) {
        self.segments.extend_from_slice(path_segments);
        self.track = compute_track_ps(&self.segments);
        self.track_ranges = vec![];
    }

    pub fn recompute_track(&mut self) {
        // self.track = compute_track(&self.control_points);
        self.track = compute_track_ps(&self.segments);
    }

    pub fn segments(&self) -> impl Iterator<Item = &PathSegment> {
        self.segments.iter()
    }

    pub fn find_node(&self, pos: Vec2<f64>, dist_thresh: f64) -> Option<(usize, Vec2<f64>)> {
        let dist2_thresh = dist_thresh.powi(2);
        let closest_node: Option<(usize, f64)> =
            self.segments.iter().enumerate().fold(None, |acc, cur| {
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
        closest_node.map(|(i, _)| (i, self.segments[i].end()))
    }
}

pub(super) fn _compute_track(control_points: &[Vec2<f64>]) -> Vec<Vec2<f64>> {
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

pub(super) fn compute_track_ps(path_segments: &[PathSegment]) -> Vec<Vec2<f64>> {
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
