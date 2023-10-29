use std::collections::VecDeque;

use eframe::{
    egui::{self, Context, Frame, Ui},
    epaint::{pos2, Color32, PathShape, Pos2, Rect},
};

use crate::vec2::Vec2;

use super::SCALE;

const MAX_THRUST: f64 = 0.5;
const MAX_STEERING: f64 = std::f64::consts::PI / 4.;

pub struct BicycleApp {
    direct_control: bool,
    paused: bool,
    bicycle: Bicycle,
    t: f64,
    playback_speed: f64,
    h_thrust: f64,
    v_thrust: f64,
}

impl BicycleApp {
    pub fn new() -> Self {
        Self {
            direct_control: false,
            paused: false,
            bicycle: Bicycle::new(),
            t: 0.,
            playback_speed: 0.5,
            h_thrust: 0.,
            v_thrust: 0.,
        }
    }

    pub fn update(&mut self, ctx: &Context) {
        if self.direct_control {
            ctx.input(|input| {
                self.h_thrust = 0.;
                self.v_thrust = 0.;
                for key in input.keys_down.iter() {
                    match key {
                        egui::Key::A => self.h_thrust = -MAX_STEERING,
                        egui::Key::D => self.h_thrust = MAX_STEERING,
                        egui::Key::W => self.v_thrust = MAX_THRUST,
                        egui::Key::S => self.v_thrust = -MAX_THRUST,
                        _ => {}
                    }
                }
            });

            if !self.paused {
                bicycle_simulate_step(
                    &mut self.bicycle,
                    self.h_thrust,
                    self.v_thrust,
                    self.playback_speed as f64,
                );
                self.bicycle.append_history();
            }
        }

        if !self.paused {
            self.t += self.playback_speed;
        }
    }

    pub fn update_panel(&mut self, ui: &mut Ui) {
        ui.checkbox(&mut self.direct_control, "Direct control");
        if ui.button("Reset").clicked() {
            self.bicycle = Bicycle::new();
            self.t = 0.;
        }
        ui.checkbox(&mut self.paused, "Paused");
        ui.label("Playback speed (times 60fps):");
        ui.add(egui::widgets::Slider::new(
            &mut self.playback_speed,
            (0.1)..=2.,
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

            let bicycle = &self.bicycle;
            let base_pos = to_pos2(bicycle.pos).to_vec2();

            painter.add(PathShape::line(
                bicycle
                    .pos_history
                    .iter()
                    .map(|ofs| to_pos2(*ofs))
                    .collect(),
                (2., Color32::GREEN),
            ));

            let rotation_matrix = |angle: f32| {
                [
                    angle.cos() as f32,
                    -angle.sin() as f32,
                    angle.sin() as f32,
                    angle.cos() as f32,
                ]
            };
            let rotation = rotation_matrix(bicycle.heading as f32);
            let steering = rotation_matrix((bicycle.heading + bicycle.steering) as f32);
            let rotate_vec = |rotation: &[f32; 4], ofs: &[f32; 2]| {
                eframe::emath::vec2(
                    rotation[0] * ofs[0] + rotation[1] * ofs[1],
                    rotation[2] * ofs[0] + rotation[3] * ofs[1],
                )
            };
            let transform_delta = |ofs: &[f32; 2]| rotate_vec(&rotation, ofs);
            let transform_vec = |ofs: &[f32; 2]| (transform_delta(ofs) + base_pos).to_pos2();
            let convert_to_poly = |vertices: &[[f32; 2]]| {
                PathShape::convex_polygon(
                    vertices.into_iter().map(|ofs| transform_vec(ofs)).collect(),
                    Color32::BLUE,
                    (1., Color32::RED),
                )
            };

            painter.add(convert_to_poly(&[
                [-10., -10.],
                [30., -10.],
                [30., 10.],
                [-10., 10.],
            ]));

            let front_wheel = transform_vec(&[20., 0.]);
            let front_front = front_wheel + rotate_vec(&steering, &[5., 0.]);
            let front_back = front_wheel - rotate_vec(&steering, &[5., 0.]);

            painter.line_segment([front_front, front_back], (2., Color32::BLACK));
        });
    }
}

struct Bicycle {
    pos: Vec2<f64>,
    heading: f64,
    steering: f64,
    wheel_base: f64,
    pos_history: VecDeque<Vec2<f64>>,
}

impl Bicycle {
    const MAX_HISTORY: usize = 1000;
    fn new() -> Self {
        Self {
            pos: Vec2::zero(),
            heading: 0.,
            steering: 0.,
            wheel_base: 20.,
            pos_history: VecDeque::new(),
        }
    }

    fn append_history(&mut self) {
        if self
            .pos_history
            .back()
            .map(|hist| *hist != self.pos)
            .unwrap_or(true)
        {
            self.pos_history.push_back(self.pos);
        }
        if Self::MAX_HISTORY < self.pos_history.len() {
            self.pos_history.pop_front();
        }
    }
}

fn bicycle_simulate_step(bicycle: &mut Bicycle, h_thrust: f64, v_thrust: f64, playback_speed: f64) {
    let steering_speed = playback_speed * 0.1;
    bicycle.steering = if (bicycle.steering - h_thrust).abs() < steering_speed {
        h_thrust
    } else if bicycle.steering < h_thrust {
        bicycle.steering + steering_speed
    } else {
        bicycle.steering - steering_speed
    };
    let direction = Vec2::new(bicycle.heading.cos(), -bicycle.heading.sin());
    bicycle.pos += direction * v_thrust * playback_speed;

    let theta_dot = v_thrust * bicycle.steering.tan() / bicycle.wheel_base;
    bicycle.heading += theta_dot * playback_speed;
}
