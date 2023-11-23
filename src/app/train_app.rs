use eframe::{
    egui::{self, Context, Frame, Ui},
    epaint::{pos2, Color32, PathShape, Pos2, Stroke},
};

use crate::{
    models::train::{Train, C_POINTS},
    transform::{half_rect, Transform},
    vec2::Vec2,
};

use super::SCALE;

const MAX_THRUST: f64 = 1.;
const THRUST_ACCEL: f64 = 0.001;

pub struct TrainApp {
    transform: Transform,
    train: Train,
    thrust: f64,
}

impl TrainApp {
    pub fn new() -> Self {
        Self {
            transform: Transform::new(SCALE),
            train: Train::new(),
            thrust: 0.,
        }
    }

    pub fn update(&mut self, ctx: &Context) {
        ctx.input(|input| {
            for key in input.keys_down.iter() {
                match key {
                    egui::Key::W => self.thrust = (self.thrust + THRUST_ACCEL).min(MAX_THRUST),
                    egui::Key::S => self.thrust = (self.thrust - THRUST_ACCEL).max(-MAX_THRUST),
                    _ => {}
                }
            }
        });
        self.train.update(self.thrust);
    }

    pub fn paint_graph(&mut self, ui: &mut Ui) {
        Frame::canvas(ui.style()).show(ui, |ui| {
            let (response, painter) =
                ui.allocate_painter(ui.available_size(), egui::Sense::click());

            if ui.ui_contains_pointer() {
                ui.input(|i| self.transform.handle_mouse(i, half_rect(&response.rect)));
            }

            let train = &self.train;

            let paint_transform = self.transform.into_paint(&response);

            const GRID_SIZE: f32 = 50.;
            let render_grid = |grid_size: f32, stroke: Stroke| {
                let grid_scale =
                    grid_size as f32 / (10f32).powf(self.transform.scale().log10().floor());
                let target_min = paint_transform.from_pos2(response.rect.min);
                let target_max = paint_transform.from_pos2(response.rect.max);
                let target_min_i = target_min.map(|x| (x as f32).div_euclid(grid_scale) as i32);
                let target_max_i = target_max.map(|x| (x as f32).div_euclid(grid_scale) as i32);
                for i in target_min_i.x..=target_max_i.x {
                    let x = i as f64 * grid_scale as f64;
                    let tpos = paint_transform.to_pos2(Vec2::new(x, 0.));
                    painter.line_segment(
                        [
                            pos2(tpos.x, response.rect.min.y),
                            pos2(tpos.x, response.rect.max.y),
                        ],
                        stroke,
                    );
                }
                for i in target_max_i.y..=target_min_i.y {
                    let y = i as f64 * grid_scale as f64;
                    let tpos = paint_transform.to_pos2(Vec2::new(0., y));
                    painter.line_segment(
                        [
                            pos2(response.rect.min.x, tpos.y),
                            pos2(response.rect.max.x, tpos.y),
                        ],
                        stroke,
                    );
                }
            };

            render_grid(
                GRID_SIZE / 10.,
                (1., Color32::from_rgb(223, 223, 223)).into(),
            );
            render_grid(GRID_SIZE, (2., Color32::from_rgb(192, 192, 192)).into());

            let rotation_matrix = |angle: f32| {
                [
                    angle.cos() as f32,
                    angle.sin() as f32,
                    -angle.sin() as f32,
                    angle.cos() as f32,
                ]
            };
            let rotate_vec = |rotation: &[f32; 4], ofs: &[f32; 2]| {
                [
                    rotation[0] * ofs[0] + rotation[1] * ofs[1],
                    rotation[2] * ofs[0] + rotation[3] * ofs[1],
                ]
            };
            let scale_vec = |scale: f32, vec: &[f32; 2]| [vec[0] * scale, vec[1] * scale];

            let c_points = C_POINTS
                .iter()
                .map(|ofs| Pos2::from(paint_transform.to_pos2(*ofs)))
                .collect();

            let c_points_line = PathShape::line(c_points, (1., Color32::from_rgb(127, 0, 127)));
            painter.add(c_points_line);

            let track_points: Vec<_> = self
                .train
                .track
                .iter()
                .map(|ofs| Pos2::from(paint_transform.to_pos2(*ofs)))
                .collect();

            for track_point in &track_points {
                painter.circle_filled(*track_point, 3., Color32::from_rgb(255, 0, 255));
            }

            let track_line = PathShape::line(track_points, (2., Color32::from_rgb(255, 0, 255)));
            painter.add(track_line);

            let paint_train = |pos: &Vec2<f64>, heading: f64| {
                let base_pos = paint_transform.to_pos2(*pos).to_vec2();
                let rotation = rotation_matrix(heading as f32);
                let transform_delta =
                    |ofs: &[f32; 2]| scale_vec(self.transform.scale(), &rotate_vec(&rotation, ofs));
                let transform_vec = |ofs: &[f32; 2]| Pos2::from(transform_delta(ofs)) + base_pos;
                let convert_to_poly = |vertices: &[[f32; 2]]| {
                    PathShape::closed_line(
                        vertices.into_iter().map(|ofs| transform_vec(ofs)).collect(),
                        (1., Color32::RED),
                    )
                };

                painter.add(convert_to_poly(&[
                    [-2., -2.],
                    [6., -2.],
                    [6., 2.],
                    [-2., 2.],
                ]));

                let paint_wheel = |ofs: &[f32; 2], rotation: &[f32; 4]| {
                    use eframe::emath::Vec2;
                    let middle = transform_vec(ofs);
                    let front =
                        middle + Vec2::from(rotate_vec(rotation, &[self.transform.scale(), 0.]));
                    let back =
                        middle - Vec2::from(rotate_vec(rotation, &[self.transform.scale(), 0.]));

                    painter.line_segment([front, back], (2., Color32::BLACK));
                };

                paint_wheel(&[0., 0.], &rotation);
            };

            for i in 0..3 {
                if let Some((train_pos, train_heading)) =
                    self.train.pos(i).zip(self.train.heading(i))
                {
                    paint_train(&train_pos, train_heading);
                }
            }
        });
    }
}
