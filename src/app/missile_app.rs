use eframe::{
    egui::{self, Context, Frame, Ui},
    emath::Align2,
    epaint::{pos2, vec2, Color32, FontId, PathShape, Pos2, Rect},
};

use crate::{
    missile::{missile_simulate_step, simulate_missile, MissileParams, MissileState},
    vec2::Vec2,
    xor128::Xor128,
};

use super::SCALE;

const MISSILE_STATE: MissileState = MissileState {
    pos: Vec2 { x: 2., y: 10. },
    velo: Vec2 { x: 0., y: 0. },
    target: Vec2 { x: 0., y: 0. },
    heading: 0.25 * std::f64::consts::PI,
    prediction: vec![],
    target_prediction: vec![],
};

pub struct MissileApp {
    direct_control: bool,
    paused: bool,
    missile_params: MissileParams,
    t: f64,
    randomize: bool,
    rng: Xor128,
    playback_speed: f64,
    missile_state: MissileState,
    missile_model: Vec<MissileState>,
    h_thrust: f64,
    v_thrust: f64,
    error_msg: Option<String>,
}

impl MissileApp {
    pub fn new() -> Self {
        let missile_params = MissileParams::default();
        let (missile_model, error_msg) =
            match simulate_missile(Vec2 { x: 0., y: 0. }, &missile_params) {
                Ok(res) => (res, None),
                Err(e) => (vec![], Some(e.to_string())),
            };
        Self {
            direct_control: false,
            paused: false,
            missile_params,
            t: 0.,
            randomize: true,
            rng: Xor128::new(3232123),
            playback_speed: 0.5,
            missile_state: MISSILE_STATE,
            missile_model,
            h_thrust: 0.,
            v_thrust: 0.,
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
            let canvas_offset_y = response.rect.height() * 0.5;

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

            if let Some(ref err) = self.error_msg {
                painter.text(
                    response.rect.center(),
                    Align2::CENTER_CENTER,
                    err,
                    FontId::default(),
                    Color32::RED,
                );
            }

            if response.clicked() {
                if let Some(mouse_pos) = response.interact_pointer_pos() {
                    self.try_simulate_missile(from_pos2(mouse_pos));
                }
            }

            let render_missile = |missile_state: &MissileState| {
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

                let missile_pos = to_pos2(missile_state.pos).to_vec2();
                let orientation = missile_state.heading;
                let rotation = [
                    orientation.cos() as f32,
                    -orientation.sin() as f32,
                    orientation.sin() as f32,
                    orientation.cos() as f32,
                ];
                let convert_to_poly = |vertices: &[[f32; 2]]| {
                    PathShape::convex_polygon(
                        vertices
                            .into_iter()
                            .map(|ofs| {
                                pos2(
                                    rotation[0] * ofs[0] + rotation[1] * ofs[1],
                                    rotation[2] * ofs[0] + rotation[3] * ofs[1],
                                ) + missile_pos
                            })
                            .collect(),
                        Color32::BLUE,
                        (1., Color32::RED),
                    )
                };

                painter.add(convert_to_poly(&[
                    [-1., -12.],
                    [1., -12.],
                    [3., -3.],
                    [3., 12.],
                    [-3., 12.],
                    [-3., -3.],
                ]));

                painter.add(convert_to_poly(&[[-5., 3.], [-5., 12.], [-10., 12.]]));

                painter.add(convert_to_poly(&[[5., 3.], [5., 12.], [10., 12.]]));

                painter.circle(
                    to_pos2(missile_state.target),
                    5.,
                    Color32::GREEN,
                    (1., Color32::YELLOW),
                );
            };

            if self.direct_control {
                render_missile(&self.missile_state);
            } else if let Some(missile_state) = self.missile_model.get(self.t as usize) {
                render_missile(missile_state);
            } else {
                if self.randomize {
                    let x = 30. * (self.rng.next() - 0.5);
                    let y = 30. * (self.rng.next() - 0.5);
                    println!("randomize x: {x}, y: {y}");
                    self.try_simulate_missile(Vec2 { x, y });
                }
                self.t = 0.;
            }
        });
    }

    fn try_simulate_missile(&mut self, pos: Vec2<f64>) {
        match simulate_missile(pos, &self.missile_params) {
            Ok(res) => self.missile_model = res,
            Err(e) => self.error_msg = Some(e.to_string()),
        }
        self.t = 0.;
    }

    pub fn update_panel(&mut self, ui: &mut Ui) {
        ui.checkbox(&mut self.direct_control, "direct_control");
        if ui.button("Reset").clicked() {
            self.missile_state = MISSILE_STATE;
            self.t = 0.;
        }
        ui.checkbox(&mut self.paused, "Paused");
        ui.checkbox(&mut self.randomize, "Randomize");
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

    pub fn update(&mut self, ctx: &Context) {
        if self.direct_control {
            ctx.input(|input| {
                self.h_thrust = 0.;
                self.v_thrust = 0.;
                for key in input.keys_down.iter() {
                    match key {
                        egui::Key::A => self.h_thrust = -crate::missile::MAX_THRUST,
                        egui::Key::D => self.h_thrust = crate::missile::MAX_THRUST,
                        egui::Key::W => self.v_thrust = crate::missile::MAX_THRUST,
                        _ => {}
                    }
                }
            });

            if !self.paused {
                missile_simulate_step(
                    &mut self.missile_state,
                    self.h_thrust,
                    self.v_thrust,
                    self.playback_speed as f64,
                );
            }
        }

        if !self.paused {
            self.t += self.playback_speed;
        }
    }
}
