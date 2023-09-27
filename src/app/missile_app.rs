use eframe::{
    egui::{self, Context, Frame, Ui},
    epaint::{pos2, vec2, Color32, PathShape, Pos2, Rect},
};

use crate::{
    missile::{simulate_missile, MissileParams, MissileState},
    vec2::Vec2,
};

use super::SCALE;

pub struct MissileApp {
    direct_control: bool,
    paused: bool,
    missile_params: MissileParams,
    t: f64,
    playback_speed: f64,
    missile_model: Vec<MissileState>,
    error_msg: Option<String>,
}

impl MissileApp {
    pub fn new() -> Self {
        let missile_params = MissileParams::default();
        let (missile_model, error_msg) =
            match simulate_missile(Vec2 { x: 0., y: 0. }, &missile_params) {
                Ok(res) => (res, None),
                Err(e) => (vec![], Some(e)),
            };
        Self {
            direct_control: false,
            paused: false,
            missile_params,
            t: 0.,
            playback_speed: 0.5,
            missile_model,
            error_msg,
        }
    }

    pub fn paint_graph(&mut self, ui: &mut Ui) {
        Frame::canvas(ui.style()).show(ui, |ui| {
            let (response, painter) =
                ui.allocate_painter(ui.available_size(), egui::Sense::click());

            let to_screen = egui::emath::RectTransform::from_to(
                Rect::from_min_size(Pos2::ZERO, response.rect.size()),
                response.rect,
            );
            let from_screen = to_screen.inverse();

            let canvas_offset_x = response.rect.width() * 0.5;
            let canvas_offset_y = response.rect.height() * 0.8;

            let to_pos2 = |pos: Vec2<f64>| {
                to_screen.transform_pos(pos2(
                    canvas_offset_x + pos.x as f32 * SCALE,
                    canvas_offset_y - pos.y as f32 * SCALE,
                ))
            };

            let to_vec2 = |pos: Vec2<f64>| vec2(pos.x as f32 * SCALE, -pos.y as f32 * SCALE);

            let from_pos2 = |pos: Pos2| {
                let model_pos = from_screen.transform_pos(pos);
                Vec2 {
                    x: ((model_pos.x - canvas_offset_x) / SCALE) as f64,
                    y: ((canvas_offset_y - model_pos.y) / SCALE) as f64,
                }
            };

            if response.clicked() {
                if let Some(mouse_pos) = response.interact_pointer_pos() {
                    match simulate_missile(from_pos2(mouse_pos), &self.missile_params) {
                        Ok(res) => self.missile_model = res,
                        Err(e) => self.error_msg = Some(e.to_string()),
                    }
                    self.t = 0.;
                    println!("New lander_model");
                }
            }

            if let Some(missile_state) = self.missile_model.get(self.t as usize) {
                let render_path = |prediction: &[Vec2<f64>], color: Color32| {
                    let pos = prediction.iter().map(|x| to_pos2(*x)).collect();
                    let path = PathShape::line(pos, (2., color));
                    painter.add(path);
                };
                render_path(&missile_state.prediction, Color32::from_rgb(127, 127, 0));
                render_path(
                    &missile_state.target_prediction,
                    Color32::from_rgb(0, 127, 0),
                );

                painter.circle(
                    to_pos2(missile_state.pos),
                    5.,
                    Color32::RED,
                    (1., Color32::BROWN),
                );

                painter.circle(
                    to_pos2(missile_state.target),
                    5.,
                    Color32::GREEN,
                    (1., Color32::YELLOW),
                );
            }

            if self.direct_control {
            } else if let Some(_state) = self.missile_model.get(self.t as usize) {
            } else {
                self.t = 0.;
            }
        });
    }

    pub fn update_panel(&mut self, ui: &mut Ui) {
        ui.checkbox(&mut self.direct_control, "direct_control");
        if ui.button("Reset").clicked() {
            // self.missile_state = LANDER_STATE;
            self.t = 0.;
        }
        ui.checkbox(&mut self.paused, "Paused");
        ui.label("Playback speed (times 60fps):");
        ui.add(egui::widgets::Slider::new(
            &mut self.playback_speed,
            (0.1)..=2.,
        ));
        ui.label("Max iter:");
        ui.add(egui::widgets::Slider::new(
            &mut self.missile_params.max_iter,
            1..=200,
        ));
        ui.label("Optim iter:");
        ui.add(egui::widgets::Slider::new(
            &mut self.missile_params.optim_iter,
            1..=200,
        ));
        ui.label("Descent rate:");
        ui.add(egui::widgets::Slider::new(
            &mut self.missile_params.rate,
            1e-4..=1e-3,
        ));
    }

    pub fn update(&mut self, _ctx: &Context) {
        if !self.paused {
            self.t += self.playback_speed;
        }
    }
}
