use crate::vec2::Vec2;

pub(super) fn find_closest_node(path: &[Vec2<f64>], pos: Vec2<f64>) -> f64 {
    let closest_two =
        path.iter()
            .enumerate()
            .fold([None; 2], |mut acc: [Option<(usize, f64)>; 2], cur| {
                let dist2 = (pos - *cur.1).length2();
                // Insertion sort up to closest 2 elements
                if let [Some(acc0), _] = acc {
                    if dist2 < acc0.1 {
                        acc[1] = Some(acc0);
                        acc[0] = Some((cur.0, dist2));
                        return acc;
                    }
                }
                if let [Some(_), None] = acc {
                    acc[1] = Some((cur.0, dist2));
                    return acc;
                }
                if let [Some(_), Some(acc1)] = acc {
                    if dist2 < acc1.1 {
                        acc[1] = Some((cur.0, dist2));
                    }
                    return acc;
                }
                [Some((cur.0, dist2)), None]
            });

    // We make a strong assumption that the shortest segment's ends are closest vertices of the whole path,
    // which is not necessarily true.
    match closest_two {
        [Some(first), Some(second)] => {
            if first.0 == second.0 + 1 || first.0 + 1 == second.0 {
                let (prev, next) = if second.0 < first.0 {
                    (second, first)
                } else {
                    (first, second)
                };
                let segment = path[next.0] - path[prev.0];
                let segment_tangent = segment.normalized();
                // let segment_normal = Vec2::new(segment_tangent.y, -segment_tangent.x);
                let pos_delta = pos - path[prev.0];
                // let segment_dist = pos_delta.dot(segment_normal).abs();
                // let segment_dist2 = dbg!(segment_dist.powi(2));
                let segment_s = pos_delta.dot(segment_tangent) / segment.length();
                if 0. < segment_s && segment_s < 1. {
                    prev.0 as f64 + segment_s
                } else {
                    first.0 as f64
                }
            } else {
                first.0 as f64
            }
        }
        [Some(first), None] => first.0 as f64,
        _ => unreachable!(),
    }
}

#[test]
fn test_closest_node() {
    let path = vec![
        Vec2::new(0., 0.),
        Vec2::new(5., 5.),
        Vec2::new(10., 10.),
        Vec2::new(15., 15.),
    ];
    assert!((find_closest_node(&path, Vec2::new(-5., 0.)) - 0.).abs() < 1e-6);
    assert!((find_closest_node(&path, Vec2::new(0., 5.)) - 0.5).abs() < 1e-6);
    assert!((find_closest_node(&path, Vec2::new(5., 10.)) - 1.5).abs() < 1e-6);
    assert!((find_closest_node(&path, Vec2::new(15., 10.)) - 2.5).abs() < 1e-6);
    assert!((find_closest_node(&path, Vec2::new(20., 20.)) - 3.).abs() < 1e-6);
}

pub(crate) fn interpolate_path(path: &[Vec2<f64>], s: f64) -> Option<Vec2<f64>> {
    if path.len() == 0 {
        return None;
    }
    if s <= 0. {
        return Some(path[0]);
    }
    if (path.len() - 1) as f64 <= s {
        return Some(path[path.len() - 1]);
    }
    let i = s as usize;
    let (prev, next) = (path[i], path[i + 1]);
    let segment_delta = next - prev;
    let fr = s.rem_euclid(1.);
    Some(prev + segment_delta * fr)
}
