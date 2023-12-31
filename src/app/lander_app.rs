use eframe::{
    egui::{self, Color32, Context, Frame, Ui},
    epaint::{vec2, PathShape},
};

use super::{LANDER_LEG_OFFSET, SCALE};
use crate::{
    models::lander::{
        lander_simulate_step, simulate_lander, LanderModel, LanderParams, LanderState,
    },
    transform::{half_rect, Transform},
    vec2::Vec2,
};

pub struct LanderApp {
    t: f32,
    playback_speed: f32,
    paused: bool,
    lander_params: LanderParams,
    transform: Transform,
    direct_control: bool,
    lander_state: LanderState,
    lander_model: LanderModel,
    h_thrust: f64,
    v_thrust: f64,
    error_msg: Option<String>,
}

const LANDER_STATE: LanderState = LanderState {
    pos: Vec2 { x: 2., y: 10. },
    velo: Vec2 { x: 0., y: 0. },
    heading: 0.,
    prediction: vec![],
};

impl LanderApp {
    pub fn new() -> Self {
        let lander_params = LanderParams::default();
        let lander_state = LANDER_STATE;
        let lander_model = simulate_lander(Vec2 { x: 2., y: 15. }, &lander_params).unwrap();
        Self {
            t: 0.,
            playback_speed: 0.5,
            paused: false,
            lander_params,
            transform: Transform::new(SCALE),
            direct_control: false,
            lander_state,
            lander_model,
            h_thrust: 0.,
            v_thrust: 0.,
            error_msg: None,
        }
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
                if let Some(mouse_pos) = response.interact_pointer_pos() {
                    match simulate_lander(paint_transform.from_pos2(mouse_pos), &self.lander_params)
                    {
                        Ok(res) => self.lander_model = res,
                        Err(e) => self.error_msg = Some(e.to_string()),
                    }
                    self.t = 0.;
                }
            }

            painter.line_segment(
                [
                    paint_transform.to_pos2(Vec2 {
                        x: -50.,
                        y: -LANDER_LEG_OFFSET,
                    }),
                    paint_transform.to_pos2(Vec2 {
                        x: 50.,
                        y: -LANDER_LEG_OFFSET,
                    }),
                ],
                (1., Color32::BLACK),
            );

            const GOAL_POST_HEIGHT: f64 = 2.;

            // Goal posts
            for x in [-3., 3.] {
                painter.line_segment(
                    [
                        paint_transform.to_pos2(Vec2 {
                            x,
                            y: -LANDER_LEG_OFFSET,
                        }),
                        paint_transform.to_pos2(Vec2 {
                            x,
                            y: GOAL_POST_HEIGHT,
                        }),
                    ],
                    (3., Color32::from_rgb(0, 127, 63)),
                );

                painter.add(PathShape::convex_polygon(
                    [[0., 0.], [1., -0.5], [0., -1.]]
                        .into_iter()
                        .map(|ofs| {
                            paint_transform.to_pos2(Vec2 {
                                x: x + ofs[0],
                                y: GOAL_POST_HEIGHT + ofs[1],
                            })
                        })
                        .collect(),
                    Color32::from_rgb(63, 95, 0),
                    (1., Color32::from_rgb(31, 63, 0)),
                ));
            }

            painter.circle(
                paint_transform.to_pos2(self.lander_model.target),
                5.,
                Color32::RED,
                (1., Color32::BROWN),
            );

            let render_lander = |lander_state: &LanderState| {
                let pos = lander_state
                    .prediction
                    .iter()
                    .map(|x| paint_transform.to_pos2(*x))
                    .collect();
                let path = PathShape::line(pos, (2., Color32::from_rgb(127, 127, 0)));
                painter.add(path);

                let lander_pos = paint_transform.to_pos2(lander_state.pos).to_vec2();
                let orientation = lander_state.heading;
                let rotation = [
                    orientation.cos() as f32,
                    orientation.sin() as f32,
                    -orientation.sin() as f32,
                    orientation.cos() as f32,
                ];
                let convert_to_poly = |vertices: &[[f32; 2]]| {
                    PathShape::convex_polygon(
                        vertices
                            .into_iter()
                            .map(|ofs| {
                                let vec = self.transform.transform_vector(vec2(
                                    rotation[0] * ofs[0] + rotation[1] * ofs[1],
                                    rotation[2] * ofs[0] + rotation[3] * ofs[1],
                                )) / SCALE
                                    + lander_pos;
                                vec.to_pos2()
                            })
                            .collect(),
                        Color32::BLUE,
                        (1., Color32::RED),
                    )
                };

                painter.add(convert_to_poly(&[
                    [-8., -6.],
                    [8., -6.],
                    [10., 6.],
                    [-10., 6.],
                ]));
                painter.add(convert_to_poly(&[
                    [-15., 0.],
                    [-12., 0.],
                    [-14., 12.],
                    [-17., 12.],
                ]));
                painter.add(convert_to_poly(&[
                    [15., 0.],
                    [12., 0.],
                    [14., 12.],
                    [17., 12.],
                ]));

                painter.arrow(
                    paint_transform.to_pos2(lander_state.pos),
                    paint_transform.to_vec2(lander_state.velo * 2.),
                    (2., Color32::from_rgb(127, 0, 127)).into(),
                );
            };

            if self.direct_control {
                render_lander(&self.lander_state);
            } else if let Some(lander_state) = self.lander_model.lander_states.get(self.t as usize)
            {
                render_lander(lander_state);
            } else {
                self.t = 0.;
            }
        });
    }

    pub fn update_panel(&mut self, ui: &mut Ui) {
        ui.checkbox(&mut self.direct_control, "direct_control");
        if ui.button("Reset").clicked() {
            self.lander_state = LANDER_STATE;
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
            &mut self.lander_params.max_iter,
            1..=200,
        ));
        ui.label("Optim iter:");
        ui.add(egui::widgets::Slider::new(
            &mut self.lander_params.optim_iter,
            1..=200,
        ));
        ui.label("Descent rate:");
        ui.add(egui::widgets::Slider::new(
            &mut self.lander_params.rate,
            1e-5..=1e-3,
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
                        egui::Key::W => self.v_thrust = 1.,
                        _ => {}
                    }
                }
            });

            if !self.paused {
                lander_simulate_step(
                    &mut self.lander_state,
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
