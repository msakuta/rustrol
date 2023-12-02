use eframe::{
    egui::{self, Context, Frame, Ui},
    emath::Align2,
    epaint::{pos2, Color32, FontId, PathShape, Pos2, Stroke},
};

use crate::{
    models::train::{Station, Train},
    transform::{half_rect, Transform},
    vec2::Vec2,
};

use super::SCALE;

pub struct TrainApp {
    transform: Transform,
    paused: bool,
    follow_train: bool,
    show_rail_ties: bool,
    show_track_nodes: bool,
    train: Train,
    new_station: String,
}

impl TrainApp {
    pub fn new() -> Self {
        Self {
            transform: Transform::new(SCALE),
            paused: false,
            follow_train: true,
            show_rail_ties: true,
            show_track_nodes: false,
            train: Train::new(),
            new_station: "New Station".to_string(),
        }
    }

    pub fn update(&mut self, ctx: &Context) {
        if self.paused {
            return;
        }
        let mut thrust = 0.;
        ctx.input(|input| {
            for key in input.keys_down.iter() {
                match key {
                    egui::Key::W => thrust += 1.,
                    egui::Key::S => thrust -= 1.,
                    _ => {}
                }
            }
        });
        self.train.update(thrust);
    }

    pub fn update_panel(&mut self, ui: &mut Ui) {
        ui.checkbox(&mut self.paused, "Paused");
        ui.checkbox(&mut self.follow_train, "Follow train");
        ui.checkbox(&mut self.show_rail_ties, "Show rail ties");
        ui.checkbox(&mut self.show_track_nodes, "Show track nodes");
        ui.group(|ui| {
            for (i, station) in self.train.stations.iter().enumerate() {
                ui.radio_value(&mut self.train.target_station, Some(i), &station.name);
            }
            ui.text_edit_singleline(&mut self.new_station);
            if ui.button("Add station").clicked() {
                self.train.stations.push(Station {
                    name: std::mem::take(&mut self.new_station),
                    s: self.train.track.len() as f64 - 10.,
                })
            }
        });
    }

    pub fn paint_graph(&mut self, ui: &mut Ui) {
        Frame::canvas(ui.style()).show(ui, |ui| {
            let (response, painter) =
                ui.allocate_painter(ui.available_size(), egui::Sense::click());

            if ui.ui_contains_pointer() {
                ui.input(|i| self.transform.handle_mouse(i, half_rect(&response.rect)));
            }

            let paint_transform = self.transform.into_paint(&response);

            if self.follow_train {
                if let Some(train_pos) = self.train.train_pos(0) {
                    self.transform
                        .follow([train_pos.x as f32, train_pos.y as f32]);
                }
            }

            if response.clicked() {
                if let Some(pointer) = response.interact_pointer_pos() {
                    let pos = paint_transform.from_pos2(pointer);
                    self.train.control_points.push(pos);
                    self.train.recompute_track();
                    println!("Added point {pos:?}");
                }
            }

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

            let c_points = self
                .train
                .control_points
                .iter()
                .map(|ofs| Pos2::from(paint_transform.to_pos2(*ofs)))
                .collect();

            let c_points_line = PathShape::line(c_points, (1., Color32::from_rgb(127, 0, 127)));
            painter.add(c_points_line);

            if 1. < self.transform.scale() {
                let parallel_offset = |ofs| {
                    let paint_transform = &paint_transform;
                    move |(prev, next): (&Vec2<f64>, &Vec2<f64>)| {
                        let delta = (*next - *prev).normalized();
                        Pos2::from(paint_transform.to_pos2(delta.left90() * ofs + *prev))
                    }
                };

                const RAIL_HALFWIDTH: f64 = 1.25;
                const TIE_HALFLENGTH: f64 = 1.5;
                const TIE_HALFWIDTH: f64 = 0.3;
                const TIE_INTERPOLATES: usize = 3;

                let track = &self.train.track;

                for ofs in [RAIL_HALFWIDTH, -RAIL_HALFWIDTH] {
                    let left_rail_points = track
                        .iter()
                        .zip(track.iter().skip(1))
                        .map(parallel_offset(ofs))
                        .collect();
                    let left_rail =
                        PathShape::line(left_rail_points, (1., Color32::from_rgb(255, 0, 255)));
                    painter.add(left_rail);
                }

                if self.show_rail_ties {
                    for (prev, next) in track.iter().zip(track.iter().skip(1)) {
                        let delta = *next - *prev;
                        let tangent = delta.normalized();
                        for i in 0..TIE_INTERPOLATES {
                            let offset = *prev + delta * i as f64 / TIE_INTERPOLATES as f64;
                            let left = tangent.left90() * TIE_HALFLENGTH + offset;
                            let right = tangent.left90() * -TIE_HALFLENGTH + offset;
                            let left_front = left + tangent * TIE_HALFWIDTH;
                            let left_back = left + tangent * -TIE_HALFWIDTH;
                            let right_front = right + tangent * TIE_HALFWIDTH;
                            let right_back = right + tangent * -TIE_HALFWIDTH;
                            let tie = PathShape::closed_line(
                                [left_front, right_front, right_back, left_back]
                                    .into_iter()
                                    .map(|v| paint_transform.to_pos2(v))
                                    .collect(),
                                (1., Color32::from_rgb(255, 0, 255)),
                            );
                            painter.add(tie);
                        }
                    }
                }
            } else {
                let track_points: Vec<_> = self
                    .train
                    .track
                    .iter()
                    .map(|ofs| Pos2::from(paint_transform.to_pos2(*ofs)))
                    .collect();

                if self.show_track_nodes {
                    for track_point in &track_points {
                        painter.circle_filled(*track_point, 3., Color32::from_rgb(255, 0, 255));
                    }
                }

                let track_line =
                    PathShape::line(track_points, (2., Color32::from_rgb(255, 0, 255)));
                painter.add(track_line);
            }

            const STATION_HEIGHT: f64 = 2.;

            let render_station = |station: &Station, is_target: bool| {
                let Some(pos) = self.train.s_pos(station.s) else {
                    return;
                };
                painter.line_segment(
                    [
                        paint_transform.to_pos2(pos),
                        paint_transform.to_pos2(pos + Vec2::new(0., STATION_HEIGHT)),
                    ],
                    (3., Color32::from_rgb(0, 127, 63)),
                );

                painter.add(PathShape::convex_polygon(
                    [[0., 0.], [1., -0.5], [0., -1.]]
                        .into_iter()
                        .map(|ofs| {
                            paint_transform
                                .to_pos2(pos + Vec2::new(ofs[0], STATION_HEIGHT + ofs[1]))
                        })
                        .collect(),
                    Color32::from_rgb(63, 95, 0),
                    (1., Color32::from_rgb(31, 63, 0)),
                ));

                painter.text(
                    paint_transform.to_pos2(pos),
                    Align2::CENTER_BOTTOM,
                    &station.name,
                    FontId::proportional(16.),
                    if is_target {
                        Color32::RED
                    } else {
                        Color32::BLACK
                    },
                );
            };

            for (i, station) in self.train.stations.iter().enumerate() {
                render_station(station, self.train.target_station == Some(i));
            }

            let paint_train = |pos: &Vec2<f64>, heading: f64| {
                let base_pos = paint_transform.to_pos2(*pos).to_vec2();
                let rotation = rotation_matrix(heading as f32);
                let transform_delta =
                    |ofs: &[f32; 2]| scale_vec(self.transform.scale(), &rotate_vec(&rotation, ofs));
                let transform_vec = |ofs: &[f32; 2]| Pos2::from(transform_delta(ofs)) + base_pos;
                let convert_to_poly = |vertices: &[[f32; 2]]| {
                    PathShape::closed_line(
                        vertices.into_iter().map(|ofs| transform_vec(ofs)).collect(),
                        (1., Color32::RED),
                    )
                };

                painter.add(convert_to_poly(&[
                    [-2., -2.],
                    [6., -2.],
                    [6., 2.],
                    [-2., 2.],
                ]));

                let paint_wheel = |ofs: &[f32; 2], rotation: &[f32; 4]| {
                    use eframe::emath::Vec2;
                    let middle = transform_vec(ofs);
                    let front =
                        middle + Vec2::from(rotate_vec(rotation, &[self.transform.scale(), 0.]));
                    let back =
                        middle - Vec2::from(rotate_vec(rotation, &[self.transform.scale(), 0.]));

                    painter.line_segment([front, back], (2., Color32::BLACK));
                };

                paint_wheel(&[0., 0.], &rotation);
            };

            for i in 0..3 {
                if let Some((train_pos, train_heading)) =
                    self.train.train_pos(i).zip(self.train.heading(i))
                {
                    paint_train(&train_pos, train_heading);
                }
            }
        });
    }
}