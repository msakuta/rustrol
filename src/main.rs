mod app;
#[macro_use]
mod error;
mod models {
    pub mod bicycle;
    pub mod lander;
    pub mod missile;
    pub mod ndt;
    pub mod orbital;
    pub mod train;
}
mod interpolation;
mod ops;
mod path_utils;
mod transform;
mod vec2;
#[cfg(target_arch = "wasm32")]
mod web;
mod xor128;

use crate::app::RustrolApp;
use eframe::egui::{Style, Visuals};

#[cfg(not(target_arch = "wasm32"))]
fn main() {
    let native_options = eframe::NativeOptions::default();

    eframe::run_native(
        "rustrol GUI",
        native_options,
        Box::new(|cc| {
            // We insist to use light theme, because the canvas color is designed to work with light background.
            let style = Style {
                visuals: Visuals::light(),
                ..Style::default()
            };
            cc.egui_ctx.set_style(style);
            Ok(Box::new(RustrolApp::new()))
        }),
    )
    .unwrap();
}

// when compiling to web using trunk.
#[cfg(target_arch = "wasm32")]
fn main() {
    // Make sure panics are logged using `console.error`.
    console_error_panic_hook::set_once();

    // Redirect tracing to console.log and friends:
    tracing_wasm::set_as_global_default();

    use eframe::wasm_bindgen::JsCast as _;
    use eframe::web_sys;

    let web_options = eframe::WebOptions::default();

    self::web::resize_canvas_to_screen_size("the_canvas_id", eframe::egui::Vec2::new(1980., 1024.));

    wasm_bindgen_futures::spawn_local(async {
        let document = web_sys::window()
            .expect("No window")
            .document()
            .expect("No document");

        let canvas = document
            .get_element_by_id("the_canvas_id")
            .expect("Failed to find the_canvas_id")
            .dyn_into::<web_sys::HtmlCanvasElement>()
            .expect("the_canvas_id was not a HtmlCanvasElement");

        let start_result = eframe::WebRunner::new()
            .start(
                canvas,
                web_options,
                Box::new(|cc| {
                    // We insist to use light theme, because the canvas color is designed to work with light background.
                    let style = Style {
                        visuals: Visuals::light(),
                        ..Style::default()
                    };
                    cc.egui_ctx.set_style(style);
                    Ok(Box::new(RustrolApp::new()))
                }),
            )
            .await;

        // Remove the loading text and spinner:
        if let Some(loading_text) = document.get_element_by_id("loading_text") {
            match start_result {
                Ok(_) => {
                    loading_text.remove();
                }
                Err(e) => {
                    loading_text.set_inner_html(
                        "<p> The app has crashed. See the developer console for details. </p>",
                    );
                    panic!("Failed to start eframe: {e:?}");
                }
            }
        }
    });
}

#[cfg(not(target_arch = "wasm32"))]
fn measure_time<T>(f: impl FnOnce() -> T) -> (T, f64) {
    let start = std::time::Instant::now();
    let ret = f();
    (ret, start.elapsed().as_secs_f64())
}

#[cfg(target_arch = "wasm32")]
fn measure_time<T>(f: impl FnOnce() -> T) -> (T, f64) {
    (f(), 0.)
}
