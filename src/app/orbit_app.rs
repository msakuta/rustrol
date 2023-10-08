use eframe::{
    egui::{self, Context, Frame, Ui},
    emath::Align2,
    epaint::{pos2, vec2, Color32, FontId, PathShape, Pos2, Rect},
};

use crate::{
    orbital::{
        orbital_simulate_step, simulate_orbital, OrbitalParams, OrbitalResult, OrbitalState, GM,
        ORBITAL_STATE,
    },
    vec2::Vec2,
    xor128::Xor128,
};

const SCALE: f32 = 50.;

const DEFAULT_ORIENTATION: f64 = std::f64::consts::PI / 4.;

pub struct OrbitalApp {
    direct_control: bool,
    paused: bool,
    orbital_params: OrbitalParams,
    t: f64,
    randomize: bool,
    rng: Xor128,
    playback_speed: f64,
    orbital_state: OrbitalState,
    orbital_model: OrbitalResult,
    h_thrust: f64,
    v_thrust: f64,
    error_msg: Option<String>,
}

impl OrbitalApp {
    pub fn new() -> Self {
        let orbital_params = OrbitalParams::default();
        let orbital_state = ORBITAL_STATE;
        let (orbital_model, error_msg) = match simulate_orbital(&orbital_params) {
            Ok(res) => (res, None),
            Err(e) => (Default::default(), Some(e.to_string())),
        };
        Self {
            direct_control: false,
            paused: false,
            orbital_params,
            t: 0.,
            randomize: true,
            rng: Xor128::new(3232123),
            playback_speed: 0.5,
            orbital_state,
            orbital_model,
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
                    self.try_simulate_missile(from_pos2(mouse_pos), ORBITAL_STATE.velo);
                }
            }

            if !self.direct_control {
                let shortest_idx = self
                    .orbital_model
                    .after_optim
                    .iter()
                    .enumerate()
                    .fold(None, |acc: Option<(usize, f64)>, cur| {
                        let diff = cur.1.pos - cur.1.target_pos;
                        let dist = diff.x * diff.x + diff.y * diff.y;
                        if let Some(acc) = acc {
                            Some(if acc.1 < dist { acc } else { (cur.0, dist) })
                        } else {
                            Some((cur.0, dist))
                        }
                    })
                    .map(|(i, _)| i);

                let render_path = |poses: &[Vec2<f64>], color: Color32| {
                    let pos = poses.iter().map(|x| to_pos2(*x)).collect();
                    let path = PathShape::line(pos, (2., color));
                    painter.add(path);
                };
                render_path(
                    &self
                        .orbital_model
                        .before_optim
                        .iter()
                        .map(|x| x.pos)
                        .collect::<Vec<_>>(),
                    Color32::from_rgb(63, 63, 191),
                );
                render_path(
                    &self
                        .orbital_model
                        .after_optim
                        .iter()
                        .map(|x| x.pos)
                        .collect::<Vec<_>>(),
                    Color32::from_rgb(191, 191, 191),
                );
                render_path(
                    &self
                        .orbital_model
                        .before_optim
                        .iter()
                        .map(|x| x.target_pos)
                        .collect::<Vec<_>>(),
                    Color32::from_rgb(191, 63, 191),
                );

                if let Some(idx) = shortest_idx {
                    painter.circle(
                        to_pos2(self.orbital_model.after_optim[idx].pos),
                        3.,
                        Color32::from_rgb(191, 191, 191),
                        (1., Color32::BLACK),
                    );
                    painter.circle(
                        to_pos2(self.orbital_model.after_optim[idx].target_pos),
                        3.,
                        Color32::from_rgb(191, 63, 191),
                        (1., Color32::BLACK),
                    );
                }
            }

            let render_orbit = |orbital_state: &OrbitalState| {
                let missile_pos = to_pos2(orbital_state.pos).to_vec2();
                let convert_to_poly = |vertices: &[[f32; 2]]| {
                    PathShape::convex_polygon(
                        vertices
                            .into_iter()
                            .map(|ofs| pos2(ofs[0], ofs[1]) + missile_pos)
                            .collect(),
                        Color32::BLUE,
                        (1., Color32::RED),
                    )
                };

                painter.add(convert_to_poly(&[
                    [-5., -5.],
                    [5., -5.],
                    [5., 5.],
                    [-5., 5.],
                ]));

                painter.add(convert_to_poly(&[
                    [-8., 3.],
                    [-8., -3.],
                    [-18., -3.],
                    [-18., 3.],
                ]));

                painter.add(convert_to_poly(&[
                    [8., 3.],
                    [8., -3.],
                    [18., -3.],
                    [18., 3.],
                ]));

                painter.circle(
                    to_pos2(Vec2 { x: 0., y: 0. }),
                    10.,
                    Color32::WHITE,
                    (1., Color32::BLACK),
                );

                painter.circle(
                    to_pos2(orbital_state.target_pos),
                    5.,
                    Color32::GREEN,
                    (1., Color32::YELLOW),
                );
            };

            if self.direct_control {
                render_orbit(&self.orbital_state);
            } else if let Some(orbital_state) = self.orbital_model.after_optim.get(self.t as usize)
            {
                render_orbit(orbital_state);
            } else {
                if self.randomize {
                    let r = 3. * (self.rng.next() + 0.5);
                    let angle = self.rng.next() * 2. * std::f64::consts::PI;
                    let x = r * angle.cos();
                    let y = r * angle.sin();
                    let orientation = 2. * std::f64::consts::PI * (self.rng.next() - 0.5);
                    println!("randomize x: {x}, y: {y}, orientation: {orientation}");
                    self.try_simulate_missile(
                        Vec2 { x, y },
                        Vec2 { x: -y, y: x } / r * (GM / r).sqrt(),
                    );
                }
                self.t = 0.;
            }
        });
    }

    fn try_simulate_missile(&mut self, pos: Vec2<f64>, velo: Vec2<f64>) {
        if self.direct_control {
            self.orbital_state.pos = pos;
            self.orbital_state.velo = velo;
        } else {
            self.orbital_params.initial_pos = pos;
            self.orbital_params.initial_velo = velo;
            match simulate_orbital(&self.orbital_params) {
                Ok(res) => self.orbital_model = res,
                Err(e) => self.error_msg = Some(e.to_string()),
            }
        }
        self.t = 0.;
    }

    pub fn update_panel(&mut self, ui: &mut Ui) {
        ui.checkbox(&mut self.direct_control, "direct_control");
        if ui.button("Reset").clicked() {
            self.orbital_state = ORBITAL_STATE;
            self.try_simulate_missile(self.orbital_state.pos, self.orbital_state.velo);
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
            &mut self.orbital_params.max_iter,
            1..=200,
        ));
        ui.label("Optim iter:");
        ui.add(egui::widgets::Slider::new(
            &mut self.orbital_params.optim_iter,
            1..=200,
        ));
        ui.label("Descent rate:");
        ui.add(egui::widgets::Slider::new(
            &mut self.orbital_params.rate,
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
                orbital_simulate_step(
                    &mut self.orbital_state,
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
