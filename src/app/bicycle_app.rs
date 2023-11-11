use eframe::{
    egui::{self, Context, Frame, Ui},
    emath::Align2,
    epaint::{pos2, Color32, FontId, PathShape, Pos2, Rect, Stroke},
};

use crate::{
    models::bicycle::{
        bicycle_simulate_step, control_bicycle, interpolate_path, simulate_bicycle, AvoidanceMode,
        Bicycle, BicycleNavigation, BicycleParams, BicyclePath, BicycleResult, MAX_STEERING,
        MAX_THRUST,
    },
    transform::{half_rect, Transform},
    vec2::Vec2,
};

use super::SCALE;

pub struct BicycleApp {
    realtime: bool,
    paused: bool,
    bicycle: Bicycle,
    bicycle_result: BicycleResult,
    transform: Transform,
    follow_bicycle: bool,
    t: f64,
    playback_speed: f64,
    h_thrust: f64,
    v_thrust: f64,
    params: BicycleParams,
    nav: BicycleNavigation,
    error_msg: Option<String>,
}

impl BicycleApp {
    pub fn new() -> Self {
        let params = BicycleParams::default();
        let nav = BicycleNavigation::default();
        let (bicycle_result, error_msg) =
            match simulate_bicycle(Vec2 { x: 0., y: 0. }, &params, &nav.path, &[]) {
                Ok(res) => (res, None),
                Err(e) => {
                    eprintln!("bicycle_model error: {e:?}");
                    (BicycleResult::default(), Some(e.to_string()))
                }
            };
        Self {
            realtime: true,
            paused: false,
            bicycle: Bicycle::new(),
            bicycle_result,
            transform: Transform::new(SCALE),
            follow_bicycle: true,
            t: 0.,
            playback_speed: 0.5,
            h_thrust: 0.,
            v_thrust: 0.,
            params,
            nav,
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
                        &self.nav.obstacles,
                    );
                } else {
                    if matches!(self.params.path_shape, BicyclePath::PathSearch) {
                        self.nav.update_path(&self.bicycle, &self.params);
                    }
                    match control_bicycle(
                        &self.bicycle,
                        &self.nav,
                        &self.params,
                        self.playback_speed,
                        &self.nav.obstacles,
                    ) {
                        Ok(state) => {
                            self.bicycle.pos = state.pos;
                            self.bicycle.heading = state.heading;
                            self.bicycle.steering = state.steering;
                            self.bicycle.predictions = state.predictions;
                            self.nav.prev_path_node = state.closest_path_node;
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
        ui.checkbox(&mut self.follow_bicycle, "Follow bicycle");
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
                reset_path |= ui
                    .radio_value(
                        &mut self.params.path_shape,
                        BicyclePath::PathSearch,
                        "Path Search (click to set goal)",
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
                BicyclePath::PathSearch => {
                    ui.group(|ui| {
                        ui.label("Search:");
                        reset_path |= ui
                            .radio_value(
                                &mut self.params.path_params.avoidance,
                                AvoidanceMode::Kinematic,
                                "Kinematic",
                            )
                            .changed();
                        reset_path |= ui
                            .radio_value(
                                &mut self.params.path_params.avoidance,
                                AvoidanceMode::Rrt,
                                "RRT",
                            )
                            .changed();
                        reset_path |= ui
                            .radio_value(
                                &mut self.params.path_params.avoidance,
                                AvoidanceMode::RrtStar,
                                "RRTStar",
                            )
                            .changed();
                    });
                    ui.label("Expand states:");
                    ui.add(egui::widgets::Slider::new(
                        &mut self.params.path_params.expand_states,
                        1..=500,
                    ));
                    ui.label("Search size:");
                    ui.add(egui::widgets::Slider::new(
                        &mut self.params.path_params.search_size,
                        1f64..=500.,
                    ));
                }
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

        reset_path |= ui
            .checkbox(&mut self.params.use_obstacles, "Obstacles:")
            .changed();

        if reset_path {
            if matches!(self.params.path_shape, BicyclePath::ClickedPoint) {
                self.params.path_params.path_waypoints = vec![self.bicycle.pos];
            }
            self.nav.reset_path(&self.params);
            if !self.realtime {
                (self.bicycle_result, self.error_msg) = match simulate_bicycle(
                    Vec2 { x: 0., y: 0. },
                    &self.params,
                    &self.nav.path,
                    &self.nav.obstacles,
                ) {
                    Ok(res) => (res, None),
                    Err(e) => {
                        eprintln!("bicycle_model error: {e:?}");
                        (BicycleResult::default(), Some(e.to_string()))
                    }
                };
                self.t = 0.;
            }
            self.error_msg = None;
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

            if ui.ui_contains_pointer() {
                ui.input(|i| self.transform.handle_mouse(i, half_rect(&response.rect)));
            }

            let paint_transform = self.transform.into_paint(&response);

            if self.follow_bicycle {
                self.transform
                    .follow([bicycle_pos.x as f32, bicycle_pos.y as f32]);
            }

            let base_pos = paint_transform.to_pos2(bicycle_pos).to_vec2();

            const GREEN: Color32 = Color32::from_rgb(0, 127, 0);
            const PURPLE: Color32 = Color32::from_rgb(127, 0, 127);
            const YELLOW: Color32 = Color32::from_rgb(127, 127, 0);
            const RED: Color32 = Color32::from_rgb(255, 0, 0);

            if response.clicked() {
                if let Some(pointer_pos) = response.interact_pointer_pos() {
                    match self.params.path_shape {
                        BicyclePath::ClickedPoint => {
                            self.params
                                .path_params
                                .path_waypoints
                                .push(paint_transform.from_pos2(pointer_pos));
                            self.nav.reset_path(&self.params);
                            self.nav.prev_path_node = 0.;
                        }
                        BicyclePath::PathSearch => {
                            self.params.path_params.search_goal =
                                Some(paint_transform.from_pos2(pointer_pos));
                        }
                        _ => {}
                    }
                }
            }

            let bicycle = &self.bicycle;

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

            for obstacle in &self.nav.obstacles {
                let mut rect = Rect {
                    min: paint_transform.to_pos2(obstacle.min),
                    max: paint_transform.to_pos2(obstacle.max),
                };
                std::mem::swap(&mut rect.min.y, &mut rect.max.y);
                painter.rect(rect, 0., Color32::WHITE, (1., Color32::BLACK));
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
                    |ofs: &[f32; 2]| scale_vec(self.transform.scale(), &rotate_vec(&rotation, ofs));
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
                    use eframe::emath::Vec2;
                    let middle = transform_vec(ofs);
                    let front =
                        middle + Vec2::from(rotate_vec(rotation, &[self.transform.scale(), 0.]));
                    let back =
                        middle - Vec2::from(rotate_vec(rotation, &[self.transform.scale(), 0.]));

                    painter.line_segment([front, back], (2., Color32::BLACK));
                };

                paint_wheel(&[4., 0.], &steering);
                paint_wheel(&[0., 0.], &rotation);
            };

            let paint_predictions = |predictions: &[Vec2<f64>]| {
                painter.add(PathShape::line(
                    predictions
                        .iter()
                        .map(|ofs| paint_transform.to_pos2(*ofs))
                        .collect(),
                    (2., YELLOW),
                ));

                for prediction in predictions {
                    painter.circle_filled(paint_transform.to_pos2(*prediction), 2.0, YELLOW);
                }
            };

            let paint_prediction_path = |closest_path_s: f64| {
                if self.nav.path.len() == 0 {
                    return;
                }

                if let Some(target) = interpolate_path(&self.nav.path, closest_path_s) {
                    painter.circle(
                        paint_transform.to_pos2(target),
                        5.,
                        Color32::RED,
                        (1., Color32::BLACK),
                    );
                }

                let path_predictions: Vec<_> = (0..self.params.prediction_states)
                    .filter_map(|i| interpolate_path(&self.nav.path, closest_path_s + i as f64))
                    .map(|v| paint_transform.to_pos2(v))
                    .collect();
                painter.add(PathShape::line(path_predictions.clone(), (3., RED)));

                for prediction in path_predictions {
                    painter.circle_filled(prediction, 2.0, RED);
                }
            };

            let t = self.t as usize;

            let (pos, heading, steering) = if self.realtime {
                if !matches!(self.params.path_shape, BicyclePath::DirectControl) {
                    painter.add(PathShape::line(
                        self.nav
                            .path
                            .iter()
                            .map(|ofs| paint_transform.to_pos2(*ofs))
                            .collect(),
                        (2., PURPLE),
                    ));
                }

                if let Some(search_state) = &self.nav.search_state {
                    search_state.render_search_tree(&paint_transform, &painter);
                }

                painter.add(PathShape::line(
                    bicycle
                        .pos_history
                        .iter()
                        .map(|ofs| paint_transform.to_pos2(*ofs))
                        .collect(),
                    (2., GREEN),
                ));

                paint_bicycle(self.bicycle.heading, self.bicycle.steering);

                if !matches!(self.params.path_shape, BicyclePath::DirectControl) {
                    paint_predictions(&self.bicycle.predictions);
                    paint_prediction_path(self.nav.prev_path_node);
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
                        .map(|s| paint_transform.to_pos2(s.pos))
                        .collect(),
                    (2., GREEN),
                ));

                painter.add(PathShape::line(
                    self.nav
                        .path
                        .iter()
                        .map(|ofs| paint_transform.to_pos2(*ofs))
                        .collect(),
                    (2., PURPLE),
                ));

                if let Some(state) = self.bicycle_result.bicycle_states.get(t) {
                    paint_bicycle(state.heading, state.steering);

                    paint_predictions(&state.predictions);
                    paint_prediction_path(state.closest_path_node);

                    (state.pos, state.heading, state.steering)
                } else {
                    (Vec2::zero(), 0., 0.)
                }
            };

            if matches!(self.params.path_shape, BicyclePath::PathSearch) {
                if let Some(goal) = self.params.path_params.search_goal {
                    painter.circle(
                        paint_transform.to_pos2(goal),
                        7.,
                        Color32::from_rgb(0, 127, 127),
                        (1., Color32::BLACK),
                    );
                }
            }

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
