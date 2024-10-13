use eframe::{
    egui::{self, Context, Frame, Ui},
    epaint::{pos2, Color32, PathShape, Stroke},
};

use crate::{
    models::ndt::{Model, Point},
    transform::{half_rect, Transform},
    vec2::Vec2,
};

use super::SCALE;
const SELECT_PIXEL_RADIUS: f64 = 20.;

pub struct NdtApp {
    transform: Transform,
    model: Model,
}

impl NdtApp {
    pub fn new() -> Self {
        Self {
            transform: Transform::new(SCALE * 10.),
            model: Model::new(),
        }
    }

    pub fn update(&mut self, ctx: &Context) {}

    pub fn update_panel(&mut self, ui: &mut Ui) {
        if ui.button("Reset").clicked() {
            match self.model.calc_grad() {
                Ok(_) => {}
                Err(e) => println!("Rustograd error: {e}"),
            }
        }

        if ui.button("Calc grad").clicked() {
            match self.model.calc_grad() {
                Ok(_) => {}
                Err(e) => println!("Rustograd error: {e}"),
            }
        }

        if ui.button("Run NDT").clicked() {
            match self.model.run_ndt() {
                Ok(_) => {}
                Err(e) => println!("Rustograd error: {e}"),
            }
        }

        ui.add(egui::widgets::Slider::new(
            &mut self.model.descent_rate,
            (0.01)..=(1.0),
        ));
    }

    pub fn paint_graph(&mut self, ui: &mut Ui) {
        Frame::canvas(ui.style()).show(ui, |ui| {
            let (response, painter) =
                ui.allocate_painter(ui.available_size(), egui::Sense::click());

            if ui.ui_contains_pointer() {
                ui.input(|i| self.transform.handle_mouse(i, half_rect(&response.rect)));
            }

            let paint_transform = self.transform.into_paint(&response);

            if response.clicked() {
                if let Some(pointer) = response.interact_pointer_pos() {
                    let pos = paint_transform.from_pos2(pointer);
                    self.model.points.push(Point { pos, grad: None });
                    println!("points: {}", self.model.points.len());
                }
            }

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

            for base_radius in [0.25f64, 0.5, 0.75, 1.] {
                let radius = (1. + base_radius).exp();
                let mut sigma_pts = vec![];
                self.model.plot_sigma(radius, |pt| {
                    sigma_pts.push(paint_transform.to_pos2(pt));
                });
                let color = (base_radius * 255.) as u8;
                let sigma_line =
                    PathShape::line(sigma_pts, (1., Color32::from_rgb(color, 63, color)));
                painter.add(sigma_line);
            }

            painter.circle_filled(
                paint_transform.to_pos2(self.model.gauss.mu),
                5.,
                Color32::RED,
            );

            for pt in &self.model.points {
                let pos = paint_transform.to_pos2(pt.pos);
                painter.circle_filled(pos, 3., Color32::BLUE);
                if let Some(ref grad) = pt.grad {
                    painter.arrow(pos, paint_transform.to_vec2(grad.mu), (2., Color32::GREEN));
                }
            }
        });
    }
}
