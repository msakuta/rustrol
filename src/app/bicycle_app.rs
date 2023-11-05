use eframe::{
    egui::{self, Context, Frame, Ui},
    emath::Align2,
    epaint::{pos2, Color32, FontId, PathShape, Pos2, Rect},
};

use crate::{
    models::bicycle::{
        bicycle_simulate_step, control_bicycle, simulate_bicycle, Bicycle, BicycleParams,
        BicyclePath, BicycleResult, MAX_STEERING, MAX_THRUST,
    },
    vec2::Vec2,
};

use super::SCALE;

pub struct BicycleApp {
    realtime: bool,
    paused: bool,
    bicycle: Bicycle,
    bicycle_result: BicycleResult,
    view_offset: Pos2,
    t: f64,
    playback_speed: f64,
    h_thrust: f64,
    v_thrust: f64,
    params: BicycleParams,
    error_msg: Option<String>,
}

impl BicycleApp {
    pub fn new() -> Self {
        let params = BicycleParams::default();
        let (bicycle_result, error_msg) = match simulate_bicycle(Vec2 { x: 0., y: 0. }, &params) {
            Ok(res) => (res, None),
            Err(e) => {
                eprintln!("bicycle_model error: {e:?}");
                (BicycleResult::default(), Some(e.to_string()))
            }
        };
        Self {
            realtime: false,
            paused: false,
            bicycle: Bicycle::new(),
            bicycle_result,
            view_offset: Pos2::ZERO,
            t: 0.,
            playback_speed: 0.5,
            h_thrust: 0.,
            v_thrust: 0.,
            params,
            error_msg,
        }
    }

    pub fn update(&mut self, ctx: &Context) {
        if self.realtime {
            ctx.input(|input| {
                self.h_thrust = 0.;
                self.v_thrust = 0.;
                for key in input.keys_down.iter() {
                    match key {
                        egui::Key::A => self.h_thrust = MAX_STEERING,
                        egui::Key::D => self.h_thrust = -MAX_STEERING,
                        egui::Key::W => self.v_thrust = MAX_THRUST,
                        egui::Key::S => self.v_thrust = -MAX_THRUST,
                        _ => {}
                    }
                }
            });

            if !self.paused {
                if matches!(self.params.path_shape, BicyclePath::DirectControl) {
                    bicycle_simulate_step(
                        &mut self.bicycle,
                        self.h_thrust,
                        self.v_thrust,
                        self.playback_speed as f64,
                    );
                } else {
                    match control_bicycle(&self.bicycle, &self.params) {
                        Ok(state) => {
                            self.bicycle.pos = state.pos;
                            self.bicycle.heading = state.heading;
                            self.bicycle.steering = state.steering;
                            self.bicycle.predictions = state.predictions;
                            self.bicycle.prev_path_node = state.closest_path_node;
                        }
                        Err(e) => eprintln!("Error: {e}"),
                    }
                }
                self.bicycle.append_history();
            }
        }

        if !self.paused {
            self.t += self.playback_speed;
            if self.bicycle_result.bicycle_states.len() <= self.t as usize {
                self.t = 0.;
            }
        }
    }

    pub fn update_panel(&mut self, ui: &mut Ui) {
        ui.checkbox(&mut self.realtime, "Realtime");
        let mut reset_path = false;
        if ui.button("Reset").clicked() {
            reset_path = true;
            // Reset button not only resets the path but also the current state of the vehicle
            if self.realtime {
                self.bicycle = Bicycle::new();
            }
            self.t = 0.;
        }
        ui.checkbox(&mut self.paused, "Paused");
        ui.label("Playback speed (times 60fps):");
        ui.add(egui::widgets::Slider::new(
            &mut self.playback_speed,
            (0.1)..=2.,
        ));
        ui.group(|ui| {
            ui.label("Path shape:");
            ui.add_enabled_ui(self.realtime, |ui| {
                reset_path |= ui
                    .radio_value(
                        &mut self.params.path_shape,
                        BicyclePath::DirectControl,
                        "Direct control (WASD keys)",
                    )
                    .changed();
                reset_path |= ui
                    .radio_value(
                        &mut self.params.path_shape,
                        BicyclePath::ClickedPoint,
                        "Clicked Point",
                    )
                    .changed();
            });
            reset_path |= ui
                .radio_value(&mut self.params.path_shape, BicyclePath::Circle, "Circle")
                .changed();
            reset_path |= ui
                .radio_value(&mut self.params.path_shape, BicyclePath::Sine, "Sine")
                .changed();
            reset_path |= ui
                .radio_value(&mut self.params.path_shape, BicyclePath::Crank, "Crank")
                .changed();
            ui.label("Target Speed:");
            ui.add(egui::widgets::Slider::new(
                &mut self.params.path_params.target_speed,
                0.1..=5.0,
            ));
            match self.params.path_shape {
                BicyclePath::Circle => {
                    ui.label("Circle Radius:");
                    ui.add(egui::widgets::Slider::new(
                        &mut self.params.path_params.circle_radius,
                        1f64..=100f64,
                    ));
                }
                BicyclePath::Sine => {
                    ui.label("Sine Period:");
                    ui.add(egui::widgets::Slider::new(
                        &mut self.params.path_params.sine_period,
                        1f64..=500f64,
                    ));
                    ui.label("Sine Amplitude:");
                    ui.add(egui::widgets::Slider::new(
                        &mut self.params.path_params.sine_amplitude,
                        1f64..=50f64,
                    ));
                }
                BicyclePath::Crank => {
                    ui.label("Crank Period:");
                    ui.add(egui::widgets::Slider::new(
                        &mut self.params.path_params.crank_period,
                        1f64..=500f64,
                    ));
                }
                _ => {}
            }
        });

        if reset_path {
            if matches!(self.params.path_shape, BicyclePath::ClickedPoint) {
                self.params.path_params.path_waypoints = vec![self.bicycle.pos];
            }
            self.params.reset_path();
            if !self.realtime {
                (self.bicycle_result, self.error_msg) =
                    match simulate_bicycle(Vec2 { x: 0., y: 0. }, &self.params) {
                        Ok(res) => (res, None),
                        Err(e) => {
                            eprintln!("bicycle_model error: {e:?}");
                            (BicycleResult::default(), Some(e.to_string()))
                        }
                    };
                self.t = 0.;
            }
        }

        ui.label("Max iter:");
        ui.add(egui::widgets::Slider::new(
            &mut self.params.max_iter,
            1..=500,
        ));
        ui.label("Prediction states:");
        ui.add(egui::widgets::Slider::new(
            &mut self.params.prediction_states,
            1..=100,
        ));
        ui.label("Optim iter:");
        ui.add(egui::widgets::Slider::new(
            &mut self.params.optim_iter,
            1..=200,
        ));
        ui.label("Descent rate:");
        ui.add(egui::widgets::Slider::new(
            &mut self.params.rate,
            1e-4..=1e-3,
        ));
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

            let bicycle = &self.bicycle;
            let bicycle_pos = if !self.realtime {
                let t = self.t as usize;
                if let Some(state) = self.bicycle_result.bicycle_states.get(t) {
                    state.pos
                } else {
                    bicycle.pos
                }
            } else {
                bicycle.pos
            };
            let view_delta = eframe::emath::vec2(bicycle_pos.x as f32, -bicycle_pos.y as f32)
                * SCALE
                - self.view_offset.to_vec2();
            self.view_offset += view_delta * 0.05;
            let canvas_offset_x = response.rect.width() * 0.5 - self.view_offset.x;
            let canvas_offset_y = response.rect.height() * 0.5 - self.view_offset.y;

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

            let base_pos = to_pos2(bicycle_pos).to_vec2();

            const GREEN: Color32 = Color32::from_rgb(0, 127, 0);
            const PURPLE: Color32 = Color32::from_rgb(127, 0, 127);

            if matches!(self.params.path_shape, BicyclePath::ClickedPoint) {
                if response.clicked() {
                    println!("Clicked {:?}", response.interact_pointer_pos());
                    if let Some(pointer_pos) = response.interact_pointer_pos() {
                        self.params
                            .path_params
                            .path_waypoints
                            .push(from_pos2(pointer_pos));
                        self.params.reset_path();
                        self.bicycle.prev_path_node = 0;
                    }
                }
            }

            let bicycle = &self.bicycle;

            const GRID_SIZE: f64 = 10.;
            for i in -10..10 {
                let x = (i as f64 + bicycle_pos.x.div_euclid(GRID_SIZE)) * GRID_SIZE;
                let tpos = to_pos2(Vec2::new(x, 0.));
                painter.line_segment(
                    [
                        pos2(tpos.x, response.rect.min.y),
                        pos2(tpos.x, response.rect.max.y),
                    ],
                    (1., Color32::GRAY),
                );
            }
            for i in -10..10 {
                let y = (i as f64 + bicycle_pos.y.div_euclid(GRID_SIZE)) * GRID_SIZE;
                let tpos = to_pos2(Vec2::new(0., y));
                painter.line_segment(
                    [
                        pos2(response.rect.min.x, tpos.y),
                        pos2(response.rect.max.x, tpos.y),
                    ],
                    (1., Color32::GRAY),
                );
            }

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

            let paint_bicycle = |heading: f64, steering: f64| {
                let rotation = rotation_matrix(heading as f32);
                let steering = rotation_matrix((heading + steering) as f32);
                let transform_delta =
                    |ofs: &[f32; 2]| scale_vec(SCALE, &rotate_vec(&rotation, ofs));
                let transform_vec = |ofs: &[f32; 2]| Pos2::from(transform_delta(ofs)) + base_pos;
                let convert_to_poly = |vertices: &[[f32; 2]]| {
                    PathShape::convex_polygon(
                        vertices.into_iter().map(|ofs| transform_vec(ofs)).collect(),
                        Color32::BLUE,
                        (1., Color32::RED),
                    )
                };

                painter.add(convert_to_poly(&[
                    [-2., -2.],
                    [6., -2.],
                    [6., 2.],
                    [-2., 2.],
                ]));
                painter.add(convert_to_poly(&[[7., -1.], [8., 0.], [7., 1.]]));

                let paint_wheel = |ofs: &[f32; 2], rotation: &[f32; 4]| {
                    let front_wheel = transform_vec(ofs);
                    let front_front =
                        front_wheel + eframe::emath::Vec2::from(rotate_vec(rotation, &[SCALE, 0.]));
                    let front_back =
                        front_wheel - eframe::emath::Vec2::from(rotate_vec(rotation, &[SCALE, 0.]));

                    painter.line_segment([front_front, front_back], (2., Color32::BLACK));
                };

                paint_wheel(&[4., 0.], &steering);
                paint_wheel(&[0., 0.], &rotation);
            };

            let t = self.t as usize;

            let (pos, heading, steering) = if self.realtime {
                if !matches!(self.params.path_shape, BicyclePath::DirectControl) {
                    painter.add(PathShape::line(
                        self.params.path.iter().map(|ofs| to_pos2(*ofs)).collect(),
                        (2., PURPLE),
                    ));
                }

                painter.add(PathShape::line(
                    bicycle
                        .pos_history
                        .iter()
                        .map(|ofs| to_pos2(*ofs))
                        .collect(),
                    (2., Color32::GREEN),
                ));

                paint_bicycle(self.bicycle.heading, self.bicycle.steering);

                if !matches!(self.params.path_shape, BicyclePath::DirectControl) {
                    painter.add(PathShape::line(
                        self.bicycle
                            .predictions
                            .iter()
                            .map(|ofs| to_pos2(*ofs))
                            .collect(),
                        (2., Color32::from_rgb(127, 127, 0)),
                    ));

                    let closest_path_node = self.bicycle.prev_path_node;
                    if 0 < self.params.path.len() {
                        let start = closest_path_node.min(self.params.path.len() - 1);
                        let end = (closest_path_node + self.params.prediction_states)
                            .min(self.params.path.len() - 1);

                        painter.add(PathShape::line(
                            self.params.path[start..end]
                                .iter()
                                .map(|ofs| to_pos2(*ofs))
                                .collect(),
                            (3., Color32::from_rgb(255, 0, 0)),
                        ));
                    }
                }

                (
                    self.bicycle.pos,
                    self.bicycle.heading,
                    self.bicycle.steering,
                )
            } else {
                painter.add(PathShape::line(
                    self.bicycle_result
                        .bicycle_states
                        .iter()
                        .map(|s| to_pos2(s.pos))
                        .collect(),
                    (2., GREEN),
                ));

                painter.add(PathShape::line(
                    self.params.path.iter().map(|ofs| to_pos2(*ofs)).collect(),
                    (2., Color32::from_rgb(127, 0, 127)),
                ));

                if let Some(state) = self.bicycle_result.bicycle_states.get(t) {
                    paint_bicycle(state.heading, state.steering);

                    painter.add(PathShape::line(
                        state.predictions.iter().map(|ofs| to_pos2(*ofs)).collect(),
                        (2., Color32::from_rgb(127, 127, 0)),
                    ));

                    let closest_path_node = state.closest_path_node;

                    if let Some(target) = self.params.path.get(closest_path_node) {
                        painter.circle(to_pos2(*target), 5., Color32::RED, (1., Color32::BLACK));
                    }

                    let start = closest_path_node.min(self.params.path.len() - 1);
                    let end = (closest_path_node + self.params.prediction_states)
                        .min(self.params.path.len() - 1);

                    painter.add(PathShape::line(
                        self.params.path[start..end]
                            .iter()
                            .map(|ofs| to_pos2(*ofs))
                            .collect(),
                        (3., Color32::from_rgb(255, 0, 0)),
                    ));

                    (state.pos, state.heading, state.steering)
                } else {
                    (Vec2::zero(), 0., 0.)
                }
            };

            const DEG_PER_RAD: f64 = 180. / std::f64::consts::PI;

            painter.text(
                response.rect.left_top(),
                Align2::LEFT_TOP,
                format!(
                    "t: {t}, pos: {:.3}, {:.3}, heading: {:.3}, steer: {:.3}",
                    pos.x,
                    pos.y,
                    heading * DEG_PER_RAD,
                    steering * DEG_PER_RAD
                ),
                FontId::proportional(16.),
                Color32::BLACK,
            );

            if let Some(ref err) = self.error_msg {
                painter.text(
                    response.rect.center(),
                    Align2::CENTER_CENTER,
                    err,
                    FontId::default(),
                    Color32::RED,
                );
            }
        });
    }
}
