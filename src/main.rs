mod app;
#[macro_use]
mod error;
mod models {
    pub mod bicycle;
    pub mod lander;
    pub mod missile;
    pub mod orbital;
}
mod interpolation;
mod ops;
mod transform;
mod vec2;
mod xor128;

use crate::app::RustrolApp;

#[cfg(not(target_arch = "wasm32"))]
fn main() {
    let mut native_options = eframe::NativeOptions::default();

    // We insist to use light theme, because the canvas color is designed to work with light background.
    native_options.follow_system_theme = false;
    native_options.default_theme = eframe::Theme::Light;

    eframe::run_native(
        "rustrol GUI",
        native_options,
        Box::new(|_cc| Box::new(RustrolApp::new())),
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

    let mut web_options = eframe::WebOptions::default();

    // We insist to use light theme, because the canvas color is designed to work with light background.
    web_options.follow_system_theme = false;
    web_options.default_theme = eframe::Theme::Light;

    wasm_bindgen_futures::spawn_local(async {
        eframe::start_web(
            "the_canvas_id", // hardcode it
            web_options,
            Box::new(|cc| Box::new(RustrolApp::new())),
        )
        .await
        .expect("failed to start eframe");
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
