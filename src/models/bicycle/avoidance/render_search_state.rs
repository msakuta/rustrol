//! Rendering the search tree, isolated from the rest of avoidance to minimize dependency to egui

use eframe::{egui::Painter, epaint::Color32};

use super::SearchState;
use crate::transform::PaintTransform;

impl SearchState {
    pub(crate) fn render_search_tree(&self, transform: &PaintTransform, painter: &Painter) {
        let nodes = &self.search_tree;
        for node in nodes {
            if let Some(from) = node.from {
                let brush = Color32::from_rgb(node.cost.min(255.) as u8, 0, 0);
                let from_node = &nodes[from];
                painter.line_segment(
                    [
                        transform.to_pos2(node.state.into()),
                        transform.to_pos2(from_node.state.into()),
                    ],
                    (1., brush),
                );
            }
        }
    }
}
