use cgmath::{Matrix3, Point2, Vector2};
use eframe::{
    egui::InputState,
    epaint::{pos2, Pos2, Vec2},
};

/// A type representing transformation, including scale and offset.
///
/// It does not include rotation.
#[derive(Clone, Copy, Debug)]
pub(crate) struct Transform {
    scale: f32,
    offset: [f32; 2],
}

impl Transform {
    pub(crate) fn new(scale: f32) -> Self {
        Self {
            scale,
            offset: [0.; 2],
        }
    }

    pub(crate) fn scale(&self) -> f32 {
        self.scale
    }

    pub(crate) fn _offset(&self) -> [f32; 2] {
        self.offset
    }

    pub(crate) fn transform_point(&self, v: impl Into<Pos2>) -> Pos2 {
        let m = self.view_transform();
        let v = v.into();
        let ret = <Matrix3<f32> as cgmath::Transform<Point2<f32>>>::transform_point(
            &m,
            cgmath::Point2::new(v.x, v.y),
        );
        pos2(ret.x, ret.y)
    }

    pub(crate) fn inverse_transform_point(&self, v: impl Into<Pos2>) -> Pos2 {
        let m = self.inverse_view_transform();
        let v = v.into();
        let ret = <Matrix3<f32> as cgmath::Transform<Point2<f32>>>::transform_point(
            &m,
            cgmath::Point2::new(v.x, v.y),
        );
        pos2(ret.x, ret.y)
    }

    fn view_transform(&self) -> Matrix3<f32> {
        Matrix3::from_scale(self.scale) * Matrix3::from_translation(self.offset.into())
    }

    fn inverse_view_transform(&self) -> Matrix3<f32> {
        Matrix3::from_translation(-cgmath::Vector2::from(self.offset))
            * Matrix3::from_scale(1. / self.scale)
    }

    pub(crate) fn handle_zoom(&mut self, i: &InputState, canvas_offset: [f32; 2]) {
        let scroll_delta = i.scroll_delta[1];
        let zoom_delta = if i.multi_touch().is_some() {
            i.zoom_delta()
        } else {
            1.
        };
        let interact_pos = i.pointer.interact_pos().unwrap_or(Pos2::ZERO);
        let delta = i.pointer.delta();

        if i.pointer.primary_down() {
            self.offset[0] += delta.x / self.scale;
            self.offset[1] -= delta.y / self.scale;
        }

        if scroll_delta != 0. || zoom_delta != 1. {
            let interact_pos_a: [f32; 2] = [
                -interact_pos.x + canvas_offset[0],
                interact_pos.y - canvas_offset[1],
            ];
            let old_offset = self.inverse_transform_point(interact_pos_a);
            if scroll_delta < 0. {
                self.scale /= 1.2;
            } else if 0. < scroll_delta {
                self.scale *= 1.2;
            } else if zoom_delta != 1. {
                self.scale *= zoom_delta;
            }
            let new_offset = self.inverse_transform_point(interact_pos_a);
            let diff = new_offset - old_offset;
            let diff = -Vector2::new(diff[0], diff[1]);
            self.offset = (Vector2::<f32>::from(self.offset) + diff).into();
        }
    }

    pub(crate) fn follow(&mut self, pos: impl Into<[f32; 2]>) {
        let pos = pos.into();
        let view_offset = Vec2::from(self.offset);
        let view_delta = -eframe::emath::vec2(pos[0] as f32, pos[1] as f32) - view_offset;
        let new_view_offset = view_offset + view_delta * 0.05;
        self.offset = new_view_offset.into();
    }
}
