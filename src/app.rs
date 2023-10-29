mod bicycle_app;
mod lander_app;
mod missile_app;
mod orbit_app;
mod three_body_app;

use eframe::egui::{self, Context};

use self::{
    bicycle_app::BicycleApp, lander_app::LanderApp, missile_app::MissileApp, orbit_app::OrbitalApp,
    three_body_app::ThreeBodyApp,
};

const SCALE: f32 = 10.;

const LANDER_LEG_OFFSET: f64 = 1.2;

#[derive(PartialEq, Eq)]
enum AppRadio {
    Lander,
    Missile,
    Orbital,
    ThreeBody,
    Bicycle,
}

pub enum AppSelect {
    Lander(LanderApp),
    Missile(MissileApp),
    Orbital(OrbitalApp),
    ThreeBody(ThreeBodyApp),
    Bicycle(BicycleApp),
}

pub struct RustrolApp {
    app_radio: AppRadio,
    app: AppSelect,
}

impl RustrolApp {
    pub fn new() -> Self {
        Self {
            app_radio: AppRadio::Bicycle,
            app: AppSelect::Bicycle(BicycleApp::new()),
        }
    }
}

impl eframe::App for RustrolApp {
    fn update(&mut self, ctx: &Context, _frame: &mut eframe::Frame) {
        ctx.request_repaint();

        eframe::egui::SidePanel::right("side_panel")
            .min_width(200.)
            .show(ctx, |ui| {
                let mut changed = ui
                    .radio_value(&mut self.app_radio, AppRadio::Lander, "Lunar Lander")
                    .changed();
                changed = changed
                    || ui
                        .radio_value(&mut self.app_radio, AppRadio::Missile, "Missile")
                        .changed();
                changed = changed
                    || ui
                        .radio_value(&mut self.app_radio, AppRadio::Orbital, "Orbital")
                        .changed();
                changed = changed
                    || ui
                        .radio_value(&mut self.app_radio, AppRadio::ThreeBody, "Three body")
                        .changed();
                changed = changed
                    || ui
                        .radio_value(
                            &mut self.app_radio,
                            AppRadio::Bicycle,
                            "Kinematic Bicycle Model",
                        )
                        .changed();

                if changed {
                    match self.app_radio {
                        AppRadio::Lander => self.app = AppSelect::Lander(LanderApp::new()),
                        AppRadio::Missile => self.app = AppSelect::Missile(MissileApp::new()),
                        AppRadio::Orbital => self.app = AppSelect::Orbital(OrbitalApp::new()),
                        AppRadio::ThreeBody => self.app = AppSelect::ThreeBody(ThreeBodyApp::new()),
                        AppRadio::Bicycle => self.app = AppSelect::Bicycle(BicycleApp::new()),
                    }
                }

                ui.group(|ui| match self.app {
                    AppSelect::Lander(ref mut lander) => lander.update_panel(ui),
                    AppSelect::Missile(ref mut missile) => missile.update_panel(ui),
                    AppSelect::Orbital(ref mut app) => app.update_panel(ui),
                    AppSelect::ThreeBody(ref mut app) => app.update_panel(ui),
                    AppSelect::Bicycle(ref mut app) => app.update_panel(ui),
                });
            });

        if let AppSelect::ThreeBody(ref mut app) = self.app {
            app.render_plot(ctx);
        }

        egui::CentralPanel::default()
            // .resizable(true)
            // .min_height(100.)
            .show(ctx, |ui| match self.app {
                AppSelect::Lander(ref mut lander) => lander.paint_graph(ui),
                AppSelect::Missile(ref mut app) => app.paint_graph(ui),
                AppSelect::Orbital(ref mut app) => app.paint_graph(ui),
                AppSelect::ThreeBody(ref mut app) => app.paint_graph(ui),
                AppSelect::Bicycle(ref mut app) => app.paint_graph(ui),
            });

        match self.app {
            AppSelect::Lander(ref mut lander) => lander.update(ctx),
            AppSelect::Missile(ref mut app) => app.update(ctx),
            AppSelect::Orbital(ref mut app) => app.update(ctx),
            AppSelect::ThreeBody(ref mut app) => app.update(ctx),
            AppSelect::Bicycle(ref mut app) => app.update(ctx),
        }
    }
}
