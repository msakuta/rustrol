use eframe::{
    egui::{self, Context, Frame, Ui},
    epaint::{pos2, vec2, Color32, Pos2, Rect},
};

use crate::missile::{simulate_missile, MissileState, Vec2};

use super::SCALE;

pub struct MissileApp {
    direct_control: bool,
    paused: bool,
    t: f64,
    playback_speed: f64,
    missile_model: Vec<MissileState>,
}

impl MissileApp {
    pub fn new() -> Self {
        Self {
            direct_control: false,
            paused: false,
            t: 0.,
            playback_speed: 0.5,
            missile_model: simulate_missile(),
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

            if let Some(missile_state) = self.missile_model.get(self.t as usize) {
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
    }

    pub fn update(&mut self, _ctx: &Context) {
        if !self.paused {
            self.t += self.playback_speed;
        }
    }
}
