//! Copied from egui_web which is not maintained since 0.17

use eframe::{wasm_bindgen, web_sys};

pub fn canvas_element(canvas_id: &str) -> Option<web_sys::HtmlCanvasElement> {
    use wasm_bindgen::JsCast;
    let document = web_sys::window()?.document()?;
    let canvas = document.get_element_by_id(canvas_id)?;
    canvas.dyn_into::<web_sys::HtmlCanvasElement>().ok()
}

pub fn resize_canvas_to_screen_size(
    canvas_id: &str,
    max_size_points: eframe::egui::Vec2,
) -> Option<()> {
    let canvas = canvas_element(canvas_id)?;

    let screen_size_points = screen_size_in_native_points()?;
    let pixels_per_point = native_pixels_per_point();

    let max_size_pixels = pixels_per_point * max_size_points;

    let canvas_size_pixels = pixels_per_point * screen_size_points;
    let canvas_size_pixels = canvas_size_pixels.min(max_size_pixels);

    // Make sure that the height and width are always even numbers.
    // otherwise, the page renders blurry on some platforms.
    // See https://github.com/emilk/egui/issues/103
    fn round_to_even(v: f32) -> f32 {
        (v / 2.0).round() * 2.0
    }

    canvas.style().set_property("width", "100%").ok()?;
    canvas.style().set_property("height", "100%").ok()?;
    canvas.set_width(round_to_even(canvas_size_pixels.x) as u32);
    canvas.set_height(round_to_even(canvas_size_pixels.y) as u32);

    Some(())
}

pub fn screen_size_in_native_points() -> Option<eframe::egui::Vec2> {
    let window = web_sys::window()?;
    Some(eframe::egui::vec2(
        window.inner_width().ok()?.as_f64()? as f32,
        window.inner_height().ok()?.as_f64()? as f32,
    ))
}

pub fn native_pixels_per_point() -> f32 {
    let pixels_per_point = web_sys::window().unwrap().device_pixel_ratio() as f32;
    if pixels_per_point > 0.0 && pixels_per_point.is_finite() {
        pixels_per_point
    } else {
        1.0
    }
}
