use eframe::{
    egui::{
        self,
        plot::{Legend, Line, PlotPoints},
        widgets::plot::Plot,
        Context, Frame, Ui,
    },
    emath::Align2,
    epaint::{Color32, FontId, PathShape},
};

use crate::{
    models::orbital::{
        calc_initial_moon, simulate_three_body, three_body_simulate_step, OrbitalParams,
        ThreeBodyParams, ThreeBodyResult, ThreeBodyState, GM, THREE_BODY_STATE,
    },
    vec2::Vec2,
    xor128::Xor128,
};

use super::{orbit_app::render_satellite, transform::Transform};

const SCALE: f32 = 50.;

pub struct ThreeBodyApp {
    direct_control: bool,
    paused: bool,
    orbital_params: OrbitalParams,
    transform: Transform,
    prediction_horizon: usize,
    t: f64,
    randomize: bool,
    rng: Xor128,
    playback_speed: f64,
    three_body_state: ThreeBodyState,
    three_body_result: ThreeBodyResult,
    h_thrust: f64,
    v_thrust: f64,
    error_msg: Option<String>,
}

impl ThreeBodyApp {
    pub fn new() -> Self {
        let (three_body_state, orbital_params) = Self::init_state(OrbitalParams::default());
        let (three_body_result, error_msg) = match simulate_three_body(&orbital_params) {
            Ok(res) => (res, None),
            Err(e) => (Default::default(), Some(e.to_string())),
        };
        Self {
            direct_control: false,
            paused: false,
            orbital_params,
            prediction_horizon: 250,
            transform: Transform::new(SCALE),
            t: 0.,
            randomize: true,
            rng: Xor128::new(3232123),
            playback_speed: 0.5,
            three_body_state,
            three_body_result,
            h_thrust: 0.,
            v_thrust: 0.,
            error_msg,
        }
    }

    fn init_state(mut orbital_params: OrbitalParams) -> (ThreeBodyState, OrbitalParams) {
        let mut three_body_state = THREE_BODY_STATE;
        orbital_params.grid_search_size = 1;
        orbital_params.max_iter *= 2;
        orbital_params.three_body = Some(ThreeBodyParams::default());
        // orbital_params.earth_gm *= 0.5;
        // orbital_params.moon_gm *= 0.5;
        three_body_state.moon = calc_initial_moon(&orbital_params).unwrap();
        (three_body_state, orbital_params)
    }

    pub fn paint_graph(&mut self, ui: &mut Ui) {
        Frame::canvas(ui.style()).show(ui, |ui| {
            let (response, painter) =
                ui.allocate_painter(ui.available_size(), egui::Sense::click());

            let paint_transform = self.transform.into_paint(&response);

            if ui.ui_contains_pointer() {
                ui.input(|i| {
                    self.transform
                        .handle_mouse(i, paint_transform.canvas_offset())
                });
            }

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
                    self.set_simulation(paint_transform.from_pos2(mouse_pos));
                }
            }

            let render_path = |poses: &[Vec2<f64>], color: Color32| {
                let pos = poses.iter().map(|x| paint_transform.to_pos2(*x)).collect();
                let path = PathShape::line(pos, (2., color));
                painter.add(path);
            };

            if self.direct_control {
                let mut state = self.three_body_state;
                let mut positions = vec![];
                let mut moon_positions = vec![];
                for _ in 0..self.prediction_horizon {
                    three_body_simulate_step(&mut state, 0., 0., 1.);
                    positions.push(state.satellite.pos);
                    moon_positions.push(state.moon.pos);
                }
                render_path(&positions, Color32::from_rgb(63, 63, 191));
                render_path(&moon_positions, Color32::from_rgb(63, 63, 63));
            } else {
                if let Some(three_params) = &self.orbital_params.three_body {
                    let moon_pos = three_params.moon_pos;
                    let moon_orbit_r = moon_pos.length();
                    painter.circle_stroke(
                        paint_transform.to_pos2(Vec2::zero()),
                        (moon_orbit_r + three_params.target_r) as f32 * self.transform.scale(),
                        (1., Color32::from_rgb(191, 191, 191)),
                    );
                    painter.circle_stroke(
                        paint_transform.to_pos2(Vec2::zero()),
                        (moon_orbit_r - three_params.target_r) as f32 * self.transform.scale(),
                        (1., Color32::from_rgb(191, 191, 191)),
                    );
                }
                render_path(
                    &self
                        .three_body_result
                        .before_optim
                        .iter()
                        .map(|x| x.satellite.pos)
                        .collect::<Vec<_>>(),
                    Color32::from_rgb(63, 63, 191),
                );
                render_path(
                    &self
                        .three_body_result
                        .after_optim
                        .iter()
                        .map(|x| x.satellite.pos)
                        .collect::<Vec<_>>(),
                    Color32::from_rgb(127, 127, 0),
                );
            }

            let render_orbit = |orbital_state: &ThreeBodyState| {
                render_satellite(
                    &painter,
                    paint_transform.to_pos2(orbital_state.satellite.pos),
                );

                let pos = paint_transform.to_pos2(self.orbital_params.earth_pos);
                painter.circle(pos, 10., Color32::WHITE, (1., Color32::BLACK));

                painter.text(
                    pos,
                    Align2::CENTER_BOTTOM,
                    "Earth",
                    FontId::monospace(16.),
                    Color32::BLACK,
                );
            };

            if self.direct_control {
                render_orbit(&self.three_body_state);

                let moon = &self.three_body_state.moon;
                let moon_pos = paint_transform.to_pos2(moon.pos);
                painter.circle(moon_pos, 5., Color32::WHITE, (1., Color32::BLACK));
                painter.text(
                    moon_pos,
                    Align2::CENTER_BOTTOM,
                    "Moon",
                    FontId::monospace(16.),
                    Color32::BLACK,
                );
            } else if let Some(orbital_state) =
                self.three_body_result.after_optim.get(self.t as usize)
            {
                render_orbit(orbital_state);

                let moon_poses: Vec<_> = self
                    .three_body_result
                    .after_optim
                    .iter()
                    .map(|state| state.moon.pos)
                    .collect();
                render_path(&moon_poses, Color32::from_rgb(63, 63, 63));
                if let Some(moon_pos) = self
                    .three_body_result
                    .after_optim
                    .get(self.t as usize)
                    .map(|state| state.moon.pos)
                {
                    let moon_pos = paint_transform.to_pos2(moon_pos);
                    painter.circle(moon_pos, 5., Color32::WHITE, (1., Color32::BLACK));
                    painter.text(
                        moon_pos,
                        Align2::CENTER_BOTTOM,
                        "Moon",
                        FontId::monospace(16.),
                        Color32::BLACK,
                    );
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
        match simulate_three_body(&self.orbital_params) {
            Ok(res) => self.three_body_result = res,
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
            self.three_body_state.satellite.pos = pos;
            self.three_body_state.satellite.velo = velo;
        } else {
            self.orbital_params.initial_pos = pos;
            self.orbital_params.initial_velo = velo;
            match simulate_three_body(&self.orbital_params) {
                Ok(res) => self.three_body_result = res,
                Err(e) => self.error_msg = Some(e.to_string()),
            }
        }
        self.try_simulate();
    }

    pub fn update_panel(&mut self, ui: &mut Ui) {
        ui.checkbox(&mut self.direct_control, "direct_control");
        if ui.button("Reset").clicked() {
            if self.direct_control {
                (self.three_body_state, self.orbital_params) =
                    Self::init_state(self.orbital_params);
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
        ui.label("Orbit target raidus:");
        ui.add(egui::widgets::Slider::new(
            &mut self.orbital_params.three_body.as_mut().unwrap().target_r,
            (0.1)..=3.,
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
                three_body_simulate_step(
                    &mut self.three_body_state,
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

    fn loss_history(&self) -> Line {
        let points: PlotPoints = self
            .three_body_result
            .after_optim
            .iter()
            .enumerate()
            .filter_map(|(i, val)| Some([i as f64, (val.moon.pos - val.satellite.pos).length()]))
            .collect();
        Line::new(points)
            .color(eframe::egui::Color32::from_rgb(100, 200, 100))
            .name("Distance between the satellite and the Moon")
    }

    pub fn render_plot(&mut self, ctx: &Context) {
        eframe::egui::TopBottomPanel::bottom("bottom")
            .resizable(true)
            .show(ctx, |ui| {
                let plot = Plot::new("plot");
                plot.legend(Legend::default()).show(ui, |plot_ui| {
                    let hist = self.loss_history();
                    plot_ui.line(hist);
                })
            });
    }
}
