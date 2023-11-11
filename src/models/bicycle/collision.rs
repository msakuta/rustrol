//! Collision detection algorithms and data structures.
//!
//! The primary data structure for collision detection are
//!
//! * Obb (Oriented bounding box)
//! * CollisionShape
//! * BoundingSphere

use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::OnceLock;

use cgmath::{InnerSpace, Matrix2, Rad, Vector2};

pub(crate) type Aabb = [f64; 4];

/// Oriented bounding box
#[derive(Debug, Clone, Copy)]
pub struct Obb {
    pub center: Vector2<f64>,
    pub xs: f64,
    pub ys: f64,
    pub orient: f64,
}

#[derive(Debug, Clone, Copy)]
pub enum CollisionShape {
    // Circle(f64),
    BBox(Obb),
}

impl CollisionShape {
    pub fn to_vertices(&self) -> Option<[[f64; 2]; 4]> {
        let Self::BBox(Obb {
            center,
            xs,
            ys,
            orient,
        }) = *self;
        let mut bbox = [[-xs, -ys], [-xs, ys], [xs, ys], [xs, -ys]];
        let rot = cgmath::Matrix2::from_angle(cgmath::Rad(orient));
        for vertex in &mut bbox {
            *vertex = (center + rot * Vector2::from(*vertex)).into();
        }
        Some(bbox)
    }

    pub(crate) fn to_bounding_circle(&self) -> BoundingCircle {
        let Self::BBox(obb) = self;
        BoundingCircle {
            center: obb.center,
            radius: (obb.xs.powf(2.) + obb.ys.powf(2.)).sqrt(),
        }
    }

    #[allow(dead_code)]
    pub(crate) fn to_aabb(&self) -> Aabb {
        let bbox = self.to_vertices().unwrap();
        bbox.iter().fold(
            [bbox[0][0], bbox[0][1], bbox[0][0], bbox[0][1]],
            |acc, cur| {
                [
                    acc[0].min(cur[0]),
                    acc[1].min(cur[1]),
                    acc[2].max(cur[0]),
                    acc[3].max(cur[1]),
                ]
            },
        )
    }

    #[allow(dead_code)]
    pub(crate) fn buffer(&self, size: f64) -> Self {
        let Self::BBox(mut obb) = *self;
        obb.xs += size;
        obb.ys += size;
        Self::BBox(obb)
    }
}

#[allow(dead_code)]
pub(crate) fn aabb_intersects(a: &Aabb, b: &Aabb) -> bool {
    a[0].floor() <= b[2].ceil()
        && b[0].floor() <= a[2].ceil()
        && a[1].floor() <= b[3].ceil()
        && b[1].floor() <= a[3].ceil()
}

#[derive(Debug, Clone, Copy)]
pub(crate) struct BoundingCircle {
    pub center: Vector2<f64>,
    pub radius: f64,
}

impl BoundingCircle {
    pub(crate) fn _new(center: impl Into<Vector2<f64>>, radius: f64) -> Self {
        Self {
            center: center.into(),
            radius,
        }
    }
}

static TOTAL_CALLS: AtomicUsize = AtomicUsize::new(0);
const MAX_RECURSES: usize = 3;
static TOTAL_INTERSECTION_CHECKS: AtomicUsize = AtomicUsize::new(0);
static TERMINAL_INTERSECTION_CHECKS: AtomicUsize = AtomicUsize::new(0);

fn recurse_counts() -> &'static Vec<AtomicUsize> {
    static RECURSE_COUNTS: OnceLock<Vec<AtomicUsize>> = OnceLock::new();
    RECURSE_COUNTS.get_or_init(|| {
        let mut ret = vec![];
        for _ in 0..MAX_RECURSES {
            ret.push(AtomicUsize::new(0));
        }
        ret
    })
}

#[allow(dead_code)]
/// Binary search collision between 2 shapes in linear motion. Good for detecting
/// collisions between small, fast moving objects, but not so much for large objects like terrain walls.
///
/// This function returns if the collision happens in the first element of returned tuple,
/// and how many recursions it did to check in the second element.
pub(crate) fn bsearch_collision(
    obj1: &CollisionShape,
    obj1_velo: &Vector2<f64>,
    obj2: &CollisionShape,
    obj2_velo: &Vector2<f64>,
) -> (bool, usize) {
    // Assume obj2's stationary coordinate frame
    let rel_velo = obj1_velo - obj2_velo;

    let fetched = TOTAL_CALLS.fetch_add(1, Ordering::Relaxed);
    if fetched % 1000 == 0 {
        println!(
            "bsearch_collision: Total calls: {fetched} recurses: {:?} checks: {TOTAL_INTERSECTION_CHECKS:?} {TERMINAL_INTERSECTION_CHECKS:?}",
            recurse_counts(),
        );
    }

    // println!(
    //     "Circles: {:?} {:?}",
    //     obj1_bounding_circle.center, obj2_bounding_circle.center
    // );

    let (hit, level) = collision_internal(obj1, rel_velo, obj2, 0);
    if hit {
        (hit, level)
    } else if level == MAX_RECURSES {
        TERMINAL_INTERSECTION_CHECKS.fetch_add(1, Ordering::Relaxed);
        let obj1_copy = obj1
            .translated(rel_velo)
            .oriented(rel_velo[1].atan2(rel_velo[0]));
        (obj1_copy.intersects(obj2), level)
    } else {
        (hit, level)
    }
}

#[allow(dead_code)]
fn collision_internal(
    obj1: &CollisionShape,
    velo: Vector2<f64>,
    obj2: &CollisionShape,
    level: usize,
) -> (bool, usize) {
    let obj1_circle = obj1.to_bounding_circle();
    let obj2_circle = obj2.to_bounding_circle();

    // Potential collision radius
    let potential_radius = velo.magnitude() / 2. + obj1_circle.radius + obj2_circle.radius;
    let potential_center = velo / 2. + obj1_circle.center;

    // println!("collision recursing {level}");
    recurse_counts()
        .get(level)
        .map(|count| count.fetch_add(1, Ordering::Relaxed));

    // If the distance between the centers of the 2 objects is larger than the sum of the radii of bounding circles,
    // there will be no chance of collision.
    let dist2_centers = (potential_center - obj2_circle.center).magnitude2();
    if potential_radius.powf(2.) < dist2_centers {
        return (false, level);
    }

    let mut max_level = level;

    if level < MAX_RECURSES {
        // First half recusion
        let (hit, hitlevel) = collision_internal(obj1, velo / 2., obj2, level + 1);
        if hit {
            return (hit, hitlevel);
        }
        max_level = max_level.max(hitlevel);

        let (hit, hitlevel) =
            collision_internal(&obj1.translated(velo / 2.), velo / 2., obj2, level + 1);
        if hit {
            return (hit, hitlevel);
        }
        max_level = max_level.max(hitlevel);
        (false, max_level)
    } else {
        TOTAL_INTERSECTION_CHECKS.fetch_add(1, Ordering::Relaxed);
        let obj1_copy = obj1.oriented(velo[1].atan2(velo[0]));
        (obj1_copy.intersects(obj2), max_level)
    }
}

impl BoundingCircle {
    fn _translated(&self, offset: Vector2<f64>) -> Self {
        let mut copy = *self;
        copy.center += offset;
        copy
    }
}

impl CollisionShape {
    #[allow(dead_code)]
    /// Return a translated copy.
    pub(crate) fn translated(&self, offset: Vector2<f64>) -> Self {
        match *self {
            Self::BBox(mut obb) => {
                obb.center += offset;
                Self::BBox(obb)
            }
        }
    }

    #[allow(dead_code)]
    pub(crate) fn with_position(&self, position: Vector2<f64>) -> Self {
        match *self {
            Self::BBox(mut obb) => {
                obb.center = position;
                Self::BBox(obb)
            }
        }
    }

    #[allow(dead_code)]
    /// Return a copy with specified orientation
    pub(crate) fn oriented(&self, orient: f64) -> Self {
        match *self {
            Self::BBox(mut obb) => {
                obb.orient = orient;
                Self::BBox(obb)
            }
        }
    }

    pub(crate) fn intersects(&self, other: &Self) -> bool {
        self.intersects_inner(other) && other.intersects_inner(self)
    }

    fn intersects_inner(&self, other: &Self) -> bool {
        let Self::BBox(obb) = self;

        let rot_mat = Matrix2::from_angle(Rad(obb.orient));
        let x_normal = rot_mat * Vector2::new(1., 0.);
        let y_normal = rot_mat * Vector2::new(0., 1.);
        let org = obb.center;

        let Some(vertices) = other.to_vertices() else {
            println!("WARNING: OBB does not have vertices");
            return false;
        };

        let obb_vertices =
            vertices
                .into_iter()
                .fold(None, |acc: Option<(f64, f64, f64, f64)>, vertex| {
                    let rel_pos = Vector2::from(vertex) - org;
                    let xd = rel_pos.dot(x_normal);
                    let yd = rel_pos.dot(y_normal);
                    if let Some((x_min, x_max, y_min, y_max)) = acc {
                        Some((x_min.min(xd), x_max.max(xd), y_min.min(yd), y_max.max(yd)))
                    } else {
                        Some((xd, xd, yd, yd))
                    }
                });
        let Some((x_min, x_max, y_min, y_max)) = obb_vertices else {
            println!("WARNING: OBB does not have vertices");
            return false;
        };

        if obb.xs < x_min || x_max < -obb.xs || obb.ys < y_min || y_max < -obb.ys {
            return false;
        }

        true
    }
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn test_obb_collision() {
        let a = CollisionShape::BBox(Obb {
            center: Vector2::new(-1., 0.),
            xs: 1.,
            ys: 1.,
            orient: 0.,
        });

        let a2 = CollisionShape::BBox(Obb {
            center: Vector2::new(-1., 0.),
            xs: 1.,
            ys: 1.,
            orient: std::f64::consts::PI / 4.,
        });

        let b = CollisionShape::BBox(Obb {
            center: Vector2::new(1.2, 0.),
            xs: 1.,
            ys: 1.,
            orient: 0.,
        });

        assert_eq!(a.intersects(&b), false);
        assert_eq!(a2.intersects(&b), true);
    }

    #[test]
    fn test_obb_collision_long() {
        let a = CollisionShape::BBox(Obb {
            center: Vector2::new(-2., 0.),
            xs: 5.,
            ys: 0.5,
            orient: 0.,
        });

        let a2 = CollisionShape::BBox(Obb {
            center: Vector2::new(-2., 0.),
            xs: 5.,
            ys: 0.5,
            orient: std::f64::consts::PI / 4.,
        });

        let b = CollisionShape::BBox(Obb {
            center: Vector2::new(2., 0.),
            xs: 1.,
            ys: 1.,
            orient: 0.,
        });

        assert_eq!(a.intersects(&b), true);
        assert_eq!(a2.intersects(&b), false);
    }
}
