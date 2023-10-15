use eframe::{
    egui::{self, Context, Frame, Painter, Ui},
    emath::Align2,
    epaint::{pos2, Color32, FontId, PathShape, Pos2, Rect},
};

use crate::{
    orbital::{
        calc_initial_moon, orbital_simulate_step, simulate_orbital, OrbitalParams, OrbitalResult,
        OrbitalState, GM, ORBITAL_STATE,
    },
    vec2::Vec2,
    xor128::Xor128,
};

const SCALE: f32 = 50.;

pub struct OrbitalApp {
    direct_control: bool,
    paused: bool,
    orbital_params: OrbitalParams,
    prediction_horizon: usize,
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
    pub fn new(use_moon: bool) -> Self {
        let (orbital_state, orbital_params) = Self::init_state(use_moon, OrbitalParams::default());
        let (orbital_model, error_msg) = match simulate_orbital(&orbital_params) {
            Ok(res) => (res, None),
            Err(e) => (Default::default(), Some(e.to_string())),
        };
        Self {
            direct_control: false,
            paused: false,
            orbital_params,
            prediction_horizon: 250,
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

    fn init_state(
        use_moon: bool,
        mut orbital_params: OrbitalParams,
    ) -> (OrbitalState, OrbitalParams) {
        let mut orbital_state = ORBITAL_STATE;
        if !use_moon {
            orbital_params.moon_pos = None;
        } else {
            orbital_params.grid_search_size = 1;
            orbital_params.max_iter *= 2;
            // orbital_params.earth_gm *= 0.5;
            // orbital_params.moon_gm *= 0.5;
            orbital_state.moon = calc_initial_moon(&orbital_params);
        }
        (orbital_state, orbital_params)
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
                    self.set_simulation(from_pos2(mouse_pos));
                }
            }

            let render_path = |poses: &[Vec2<f64>], color: Color32| {
                let pos = poses.iter().map(|x| to_pos2(*x)).collect();
                let path = PathShape::line(pos, (2., color));
                painter.add(path);
            };

            if self.direct_control {
                let mut state = self.orbital_state;
                let mut positions = vec![];
                let mut target_positions = vec![];
                let mut closest = None;
                for _ in 0..self.prediction_horizon {
                    orbital_simulate_step(&mut state, 0., 0., 1.);
                    let dist2 = (state.satellite.pos - state.target.pos).length2();
                    if closest
                        .map(|closest: (f64, Vec2<f64>, Vec2<f64>)| dist2 < closest.0)
                        .unwrap_or(true)
                    {
                        closest = Some((dist2, state.satellite.pos, state.target.pos));
                    }
                    positions.push(state.satellite.pos);
                    target_positions.push(state.target.pos);
                }
                render_path(&positions, Color32::from_rgb(63, 63, 191));
                render_path(&target_positions, Color32::from_rgb(191, 63, 191));

                if let Some((_, pos, target_pos)) = closest {
                    painter.circle(
                        to_pos2(pos),
                        3.,
                        Color32::from_rgb(191, 191, 191),
                        (1., Color32::BLACK),
                    );
                    painter.circle(
                        to_pos2(target_pos),
                        3.,
                        Color32::from_rgb(191, 63, 191),
                        (1., Color32::BLACK),
                    );
                }
            } else {
                let shortest_idx = self
                    .orbital_model
                    .after_optim
                    .iter()
                    .enumerate()
                    .fold(None, |acc: Option<(usize, f64)>, cur| {
                        let diff = cur.1.satellite.pos - cur.1.target.pos;
                        let dist = diff.x * diff.x + diff.y * diff.y;
                        if let Some(acc) = acc {
                            Some(if acc.1 < dist { acc } else { (cur.0, dist) })
                        } else {
                            Some((cur.0, dist))
                        }
                    })
                    .map(|(i, _)| i);

                render_path(
                    &self
                        .orbital_model
                        .before_optim
                        .iter()
                        .map(|x| x.satellite.pos)
                        .collect::<Vec<_>>(),
                    Color32::from_rgb(63, 63, 191),
                );
                render_path(
                    &self
                        .orbital_model
                        .after_optim
                        .iter()
                        .map(|x| x.satellite.pos)
                        .collect::<Vec<_>>(),
                    Color32::from_rgb(191, 191, 191),
                );
                render_path(
                    &self
                        .orbital_model
                        .before_optim
                        .iter()
                        .map(|x| x.target.pos)
                        .collect::<Vec<_>>(),
                    Color32::from_rgb(191, 63, 191),
                );

                if let Some(idx) = shortest_idx {
                    painter.circle(
                        to_pos2(self.orbital_model.after_optim[idx].satellite.pos),
                        3.,
                        Color32::from_rgb(191, 191, 191),
                        (1., Color32::BLACK),
                    );
                    painter.circle(
                        to_pos2(self.orbital_model.after_optim[idx].target.pos),
                        3.,
                        Color32::from_rgb(191, 63, 191),
                        (1., Color32::BLACK),
                    );
                }
            }

            let render_orbit = |orbital_state: &OrbitalState| {
                render_satellite(&painter, to_pos2(orbital_state.satellite.pos));

                painter.circle(
                    to_pos2(self.orbital_params.earth_pos),
                    10.,
                    Color32::WHITE,
                    (1., Color32::BLACK),
                );

                painter.circle(
                    to_pos2(orbital_state.target.pos),
                    5.,
                    Color32::GREEN,
                    (1., Color32::YELLOW),
                );
            };

            if self.direct_control {
                render_orbit(&self.orbital_state);

                if let Some(moon) = self.orbital_state.moon {
                    painter.circle(to_pos2(moon.pos), 5., Color32::WHITE, (1., Color32::BLACK));
                }
            } else if let Some(orbital_state) = self.orbital_model.after_optim.get(self.t as usize)
            {
                render_orbit(orbital_state);

                let moon_poses: Vec<_> = self
                    .orbital_model
                    .after_optim
                    .iter()
                    .filter_map(|state| state.moon.as_ref())
                    .map(|moon| moon.pos)
                    .collect();
                render_path(&moon_poses, Color32::from_rgb(63, 63, 63));
                if let Some(moon_pos) = self
                    .orbital_model
                    .after_optim
                    .get(self.t as usize)
                    .and_then(|state| state.moon)
                    .map(|moon| moon.pos)
                {
                    painter.circle(to_pos2(moon_pos), 5., Color32::WHITE, (1., Color32::BLACK));
                }
            } else {
                if self.randomize {
                    let r = 3. * (self.rng.next() + 0.5);
                    let angle = self.rng.next() * 2. * std::f64::consts::PI;
                    let x = r * angle.cos();
                    let y = r * angle.sin();
                    self.set_simulation(Vec2 { x, y });
                }
                self.t = 0.;
            }
        });
    }

    fn try_simulate(&mut self) {
        match simulate_orbital(&self.orbital_params) {
            Ok(res) => self.orbital_model = res,
            Err(e) => self.error_msg = Some(e.to_string()),
        }
        self.t = 0.;
    }

    fn set_simulation(&mut self, pos: Vec2<f64>) {
        let x = pos.x;
        let y = pos.y;
        let r = (x * x + y * y).sqrt();
        let velo = Vec2 { x: -y, y: x } / r * (GM / r).sqrt();

        if self.direct_control {
            self.orbital_state.satellite.pos = pos;
            self.orbital_state.satellite.velo = velo;
        } else {
            self.orbital_params.initial_pos = pos;
            self.orbital_params.initial_velo = velo;
            match simulate_orbital(&self.orbital_params) {
                Ok(res) => self.orbital_model = res,
                Err(e) => self.error_msg = Some(e.to_string()),
            }
        }
        self.try_simulate();
    }

    pub fn update_panel(&mut self, ui: &mut Ui) {
        ui.checkbox(&mut self.direct_control, "direct_control");
        if ui.button("Reset").clicked() {
            if self.direct_control {
                (self.orbital_state, self.orbital_params) =
                    Self::init_state(self.orbital_params.moon_pos.is_some(), self.orbital_params);
            } else {
                self.try_simulate();
            }
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
        ui.checkbox(&mut self.orbital_params.optim_velo, "Optimize velocity");
        ui.label("Velocity weight:");
        ui.add(egui::widgets::Slider::new(
            &mut self.orbital_params.initial_velo_weight,
            (0.)..=10.,
        ));
        ui.label("Prediction horizon:");
        ui.add(egui::widgets::Slider::new(
            &mut self.prediction_horizon,
            100..=500,
        ));
    }

    pub fn update(&mut self, ctx: &Context) {
        if self.direct_control {
            ctx.input(|input| {
                self.h_thrust = 0.;
                self.v_thrust = 0.;
                for key in input.keys_down.iter() {
                    match key {
                        egui::Key::A => self.h_thrust = -1.,
                        egui::Key::D => self.h_thrust = 1.,
                        egui::Key::S => self.v_thrust = -1.,
                        egui::Key::W => self.v_thrust = 1.,
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

pub(super) fn render_satellite(painter: &Painter, pos: Pos2) {
    let missile_pos = pos.to_vec2();
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
}
