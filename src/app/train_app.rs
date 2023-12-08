use eframe::{
    egui::{self, Context, Frame, Painter, Ui},
    emath::Align2,
    epaint::{pos2, Color32, FontId, PathShape, Pos2, Stroke},
};

use crate::{
    models::train::{Station, Train, TrainTask},
    path_utils::PathSegment,
    transform::{half_rect, PaintTransform, Transform},
    vec2::Vec2,
};

use super::SCALE;
const SELECT_PIXEL_RADIUS: f64 = 20.;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum ClickMode {
    None,
    GentleCurve,
    TightCurve,
    StraightLine,
    DeleteSegment,
    AddStation,
}

pub struct TrainApp {
    transform: Transform,
    paused: bool,
    follow_train: bool,
    show_rail_ties: bool,
    show_track_nodes: bool,
    show_control_points: bool,
    click_mode: ClickMode,
    train: Train,
    selected_station: Option<usize>,
    new_station: String,
    error_msg: Option<(String, f64)>,
}

impl TrainApp {
    pub fn new() -> Self {
        Self {
            transform: Transform::new(SCALE),
            paused: false,
            follow_train: true,
            show_rail_ties: true,
            show_track_nodes: false,
            show_control_points: false,
            click_mode: ClickMode::None,
            train: Train::new(),
            selected_station: None,
            new_station: "New Station".to_string(),
            error_msg: None,
        }
    }

    pub fn update(&mut self, ctx: &Context) {
        // Decay error message even if paused
        if let Some((_, ref mut time)) = self.error_msg {
            let dt = ctx.input(|i| i.raw.predicted_dt);
            *time = *time - dt as f64;
            if *time < 0. {
                self.error_msg = None;
            }
        }

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
        ui.checkbox(&mut self.show_control_points, "Show control points");
        ui.group(|ui| {
            ui.label("Click mode:");
            ui.radio_value(&mut self.click_mode, ClickMode::None, "None");
            ui.radio_value(&mut self.click_mode, ClickMode::GentleCurve, "Gentle Curve");
            ui.radio_value(&mut self.click_mode, ClickMode::TightCurve, "Tight Curve");
            ui.radio_value(
                &mut self.click_mode,
                ClickMode::StraightLine,
                "Straight Line",
            );
            ui.radio_value(&mut self.click_mode, ClickMode::DeleteSegment, "Delete");
            ui.radio_value(&mut self.click_mode, ClickMode::AddStation, "Add Station");
        });
        ui.group(|ui| {
            ui.label("Stations:");
            for (i, station) in self.train.stations.iter().enumerate() {
                ui.radio_value(
                    &mut self.selected_station,
                    Some(i),
                    &format!("{} ({}, {})", station.name, station.path_id, station.s),
                );
            }
            if ui.button("Schedule station").clicked() {
                if let Some(target) = self.selected_station {
                    self.train.schedule.push(target);
                }
            }
            ui.text_edit_singleline(&mut self.new_station);
            // if ui.button("Add station").clicked() {
            //     self.train
            //         .add_station(std::mem::take(&mut self.new_station))
            // }
        });
        ui.group(|ui| {
            ui.label("Station schedule:");
            for (_i, station) in self.train.schedule.iter().enumerate() {
                if let Some(station) = self.train.stations.get(*station) {
                    ui.label(&station.name);
                }
            }
            if ui.button("Remove station schedule").clicked() {
                self.train.schedule.pop();
            }
        });
    }

    fn process_result(&mut self, pos: Vec2<f64>, res: Result<(), String>) {
        if let Err(e) = res {
            self.error_msg = Some((e, 10.));
        } else {
            println!("Added point {pos:?}");
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

            if self.follow_train {
                if let Some(train_pos) = self.train.train_pos(0) {
                    self.transform
                        .follow([train_pos.x as f32, train_pos.y as f32]);
                }
            }

            if response.clicked() {
                if let Some(pointer) = response.interact_pointer_pos() {
                    match self.click_mode {
                        ClickMode::None => {}
                        ClickMode::GentleCurve => {
                            let pos = paint_transform.from_pos2(pointer);
                            // self.train.control_points.push(pos);
                            let res = self.train.add_gentle(pos);
                            self.process_result(pos, res);
                        }
                        ClickMode::TightCurve => {
                            let pos = paint_transform.from_pos2(pointer);
                            let res = self.train.add_tight(pos);
                            self.process_result(pos, res);
                        }
                        ClickMode::StraightLine => {
                            let pos = paint_transform.from_pos2(pointer);
                            let res = self.train.add_straight(pos);
                            self.process_result(pos, res);
                        }
                        ClickMode::DeleteSegment => {
                            let pos = paint_transform.from_pos2(pointer);
                            let thresh = SELECT_PIXEL_RADIUS / self.transform.scale() as f64;
                            let res = self.train.delete_segment(pos, thresh);
                            self.process_result(pos, res);
                        }
                        ClickMode::AddStation => {
                            let pos = paint_transform.from_pos2(pointer);
                            let thresh = SELECT_PIXEL_RADIUS / self.transform.scale() as f64;
                            let next_name = (0..).find_map(|i| {
                                let cand = format!("New Station {i}");
                                if !self.train.stations.iter().any(|s| s.name == cand)
                                    && self.new_station != cand
                                {
                                    Some(cand)
                                } else {
                                    None
                                }
                            });
                            if let Some(name) = next_name {
                                self.train.add_station(
                                    std::mem::replace(&mut self.new_station, name),
                                    pos,
                                    thresh,
                                );
                            }
                        }
                    }
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

            match self.click_mode {
                ClickMode::None => self.train.ghost_path = None,
                ClickMode::GentleCurve => {
                    if let Some(pos) = response.hover_pos() {
                        self.train.ghost_gentle(paint_transform.from_pos2(pos));
                    } else {
                        self.train.ghost_path = None;
                    }
                }
                ClickMode::StraightLine => {
                    if let Some(pos) = response.hover_pos() {
                        self.train.ghost_straight(paint_transform.from_pos2(pos));
                    } else {
                        self.train.ghost_path = None;
                    }
                }
                ClickMode::TightCurve => {
                    if let Some(pos) = response.hover_pos() {
                        self.train.ghost_tight(paint_transform.from_pos2(pos));
                    } else {
                        self.train.ghost_path = None;
                    }
                }
                ClickMode::DeleteSegment => {
                    let found_node = response.hover_pos().and_then(|pointer| {
                        let thresh = SELECT_PIXEL_RADIUS / self.transform.scale() as f64;
                        self.train
                            .find_path_node(paint_transform.from_pos2(pointer), thresh)
                    });
                    if let Some((path_id, seg_id, _)) = found_node {
                        let color = Color32::from_rgba_premultiplied(127, 0, 127, 63);
                        if let Some(path) = self.train.paths.get(&path_id) {
                            let seg_track = path.seg_track(seg_id);
                            self.render_track_detail(
                                seg_track,
                                &painter,
                                &paint_transform,
                                5.,
                                color,
                            );
                        }
                    }
                }
                ClickMode::AddStation => {
                    if let Some(pointer) = response.hover_pos() {
                        let pos = paint_transform.from_pos2(pointer);
                        let thresh = SELECT_PIXEL_RADIUS / self.transform.scale() as f64;
                        if let Some((path_id, _, node_id)) = self.train.find_path_node(pos, thresh)
                        {
                            let station =
                                Station::new(self.new_station.clone(), path_id, node_id as f64);
                            self.render_station(&painter, &paint_transform, &station, false, true);
                        }
                    }
                }
            }

            if self.show_control_points {
                let c_points = self
                    .train
                    .control_points()
                    .iter()
                    .map(|ofs| Pos2::from(paint_transform.to_pos2(*ofs)))
                    .collect();

                let c_points_line = PathShape::line(c_points, (1., Color32::from_rgb(127, 0, 127)));
                painter.add(c_points_line);

                for seg in self.train.paths.values().map(|b| b.segments()).flatten() {
                    if let PathSegment::Arc(arc) = seg {
                        painter.circle_stroke(
                            paint_transform.to_pos2(arc.center),
                            arc.radius as f32 * self.transform.scale(),
                            (1., Color32::from_rgb(127, 0, 127)),
                        );
                    }
                }
            }

            if 1. < self.transform.scale() {
                if let Some(ghost_segments) = &self.train.ghost_path {
                    let color = Color32::from_rgba_premultiplied(255, 0, 255, 63);
                    self.render_track_detail(
                        &ghost_segments.track,
                        &painter,
                        &paint_transform,
                        1.,
                        color,
                    );
                }
                let color = Color32::from_rgba_premultiplied(255, 0, 255, 255);
                for bundle in self.train.paths.values() {
                    self.render_track_detail(&bundle.track, &painter, &paint_transform, 1., color);
                }
            } else {
                if let Some(ghost_segments) = &self.train.ghost_path {
                    self.render_track(&ghost_segments.track, &painter, &paint_transform, 63);
                }
                for bundle in self.train.paths.values() {
                    self.render_track(&bundle.track, &painter, &paint_transform, 255);
                }
            }

            for (i, station) in self.train.stations.iter().enumerate() {
                self.render_station(
                    &painter,
                    &paint_transform,
                    station,
                    self.train.train_task == TrainTask::Goto(i),
                    false,
                );
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

            painter.text(
                response.rect.left_top(),
                Align2::LEFT_TOP,
                format!("pid: {}, s: {:.3}", self.train.path_id, self.train.s),
                FontId::proportional(16.),
                Color32::BLACK,
            );

            if let Some((ref err, _)) = self.error_msg {
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

    fn render_station(
        &self,
        painter: &Painter,
        paint_transform: &PaintTransform,
        station: &Station,
        is_target: bool,
        is_ghost: bool,
    ) {
        const STATION_HEIGHT: f64 = 5.;

        let Some(pos) = self.train.s_pos(station.path_id, station.s) else {
            return;
        };
        let alpha = if is_ghost { 63 } else { 255 };
        painter.line_segment(
            [
                paint_transform.to_pos2(pos),
                paint_transform.to_pos2(pos + Vec2::new(0., STATION_HEIGHT)),
            ],
            (3., Color32::from_rgba_premultiplied(0, 127, 63, alpha)),
        );

        painter.add(PathShape::convex_polygon(
            [[0., 0.], [2., -1.], [0., -2.]]
                .into_iter()
                .map(|ofs| {
                    paint_transform.to_pos2(pos + Vec2::new(ofs[0], STATION_HEIGHT + ofs[1]))
                })
                .collect(),
            Color32::from_rgba_premultiplied(63, 95, 0, alpha),
            (1., Color32::from_rgba_premultiplied(31, 63, 0, alpha)),
        ));

        painter.text(
            paint_transform.to_pos2(pos),
            Align2::CENTER_BOTTOM,
            &station.name,
            FontId::proportional(16.),
            if is_target {
                Color32::RED
            } else if is_ghost {
                Color32::from_rgba_premultiplied(0, 0, 255, 255)
            } else {
                Color32::BLACK
            },
        );
    }

    fn render_track_detail(
        &self,
        track: &[Vec2<f64>],
        painter: &Painter,
        paint_transform: &PaintTransform,
        line_width: f32,
        color: Color32,
    ) {
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

        for ofs in [RAIL_HALFWIDTH, -RAIL_HALFWIDTH] {
            let left_rail_points = track
                .iter()
                .zip(track.iter().skip(1))
                .map(parallel_offset(ofs))
                .collect();
            let left_rail = PathShape::line(left_rail_points, (line_width, color));
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
                        (line_width, color),
                    );
                    painter.add(tie);
                }
            }
        }
    }

    fn render_track(
        &self,
        track: &[Vec2<f64>],
        painter: &Painter,
        paint_transform: &PaintTransform,
        alpha: u8,
    ) {
        let track_points: Vec<_> = track
            .iter()
            .map(|ofs| Pos2::from(paint_transform.to_pos2(*ofs)))
            .collect();

        let color = Color32::from_rgba_premultiplied(255, 0, 255, alpha);

        if self.show_track_nodes {
            for track_point in &track_points {
                painter.circle_filled(*track_point, 3., color);
            }
        }

        let track_line = PathShape::line(track_points, (2., color));
        painter.add(track_line);
    }
}
