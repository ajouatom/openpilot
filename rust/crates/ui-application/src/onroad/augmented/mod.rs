mod border;
use crate::{
    context::Context,
    mici::onroad as compact,
    onroad,
    paint::{self, color},
    params::Read,
    state::{messages, Status},
};
use num_traits::ToPrimitive;
use openpilot_msgq::VisionStream;
use openpilot_ui_framework::{
    assets::Texture,
    canvas::Canvas,
    draw::{Draw, RoundedOutline},
    geometry::{Point, Rect},
    text::Font,
    text_layout::{float, Horizontal, Vertical},
    unified_label::UnifiedLabel,
    widget::{Frame, RenderResult, Widget, WidgetState},
    Error,
};
use std::{
    cell::{Cell, RefCell},
    rc::Rc,
};
enum Model {
    Big(Box<onroad::model_renderer::ModelRenderer>),
    Compact(Box<compact::model_renderer::ModelRenderer>),
}
impl Model {
    fn overlay_state(&mut self, state: crate::overlay_state::Shared) {
        match self {
            Self::Big(widget) => widget.set_overlay_state(state),
            Self::Compact(widget) => widget.set_overlay_state(state),
        }
    }
    fn transform(&mut self, matrix: onroad::calibration::Matrix) {
        match self {
            Self::Big(widget) => widget.set_transform(matrix),
            Self::Compact(widget) => widget.set_transform(matrix),
        }
    }
    fn widget(&mut self) -> &mut dyn Widget {
        match self {
            Self::Big(widget) => widget.as_mut(),
            Self::Compact(widget) => widget.as_mut(),
        }
    }
}
enum Hud {
    Big(Box<onroad::hud::Hud>),
    Compact(Box<compact::hud::Hud>),
}
impl Hud {
    fn overlay_state(&mut self, state: crate::overlay_state::Shared) {
        match self {
            Self::Big(widget) => widget.set_overlay_state(state),
            Self::Compact(widget) => widget.set_overlay_state(state),
        }
    }
    fn widget(&mut self) -> &mut dyn Widget {
        match self {
            Self::Big(widget) => widget.as_mut(),
            Self::Compact(widget) => widget.as_mut(),
        }
    }
}
enum Alerts {
    Big(Box<onroad::alert::Alerts>),
    Compact(Box<compact::alert::Alerts>),
}
impl Alerts {
    fn widget(&mut self) -> &mut dyn Widget {
        match self {
            Self::Big(widget) => widget.as_mut(),
            Self::Compact(widget) => widget.as_mut(),
        }
    }
}
pub struct Road {
    overlay_state: crate::overlay_state::Shared,
    state: WidgetState,
    context: Context,
    pub camera: onroad::camera::CameraView,
    calibration: onroad::calibration::Calibration,
    model: Model,
    hud: Hud,
    alerts: Alerts,
    driver: onroad::driver_preview::DriverPreview,
    vision: Option<compact::vision_renderer::VisionRenderer>,
    confidence: Option<compact::confidence_ball::ConfidenceBall>,
    traffic: Option<compact::traffic_light::TrafficLight>,
    fade: Option<Texture>,
    label: UnifiedLabel,
    pub suppress_camera: bool,
    recording: Rc<Cell<bool>>,
    plot_mode: i32,
    plot_next: f64,
    publisher: openpilot_messaging::runtime::PubMaster,
    border_params: crate::cache::TimedCache<border::Parameters>,
    nav_map: onroad::nav_map::NavMap,
    diagnostics: Option<crate::render_diagnostics::RenderDiagnostics>,
}
impl Road {
    pub fn new(
        context: Context,
        canvas: &mut Canvas,
        recording: Rc<Cell<bool>>,
    ) -> Result<Self, Error> {
        Self::with_camera(context, canvas, recording, "camerad")
    }
    pub fn with_camera(
        context: Context,
        canvas: &mut Canvas,
        recording: Rc<Cell<bool>>,
        name: &str,
    ) -> Result<Self, Error> {
        let big = context.big;
        let mut camera = onroad::camera::CameraView::new(
            context.clone(),
            onroad::camera::Config {
                name: name.into(),
                stream: VisionStream::Road,
                compact: !big,
            },
            canvas,
        )?;
        camera.background = Some(if big {
            color(0x12, 0x28, 0x39, 255)
        } else {
            color(0, 0, 0, 255)
        });
        let (mut model, mut hud, alerts) = if big {
            (
                Model::Big(Box::new(onroad::model_renderer::ModelRenderer::new(
                    context.clone(),
                )?)),
                Hud::Big(Box::new(onroad::hud::Hud::new(context.clone(), canvas)?)),
                Alerts::Big(Box::new(onroad::alert::Alerts::new(context.clone()))),
            )
        } else {
            (
                Model::Compact(Box::new(compact::model_renderer::ModelRenderer::new(
                    context.clone(),
                )?)),
                Hud::Compact(Box::new(compact::hud::Hud::new(
                    context.clone(),
                    canvas,
                    20.0,
                )?)),
                Alerts::Compact(Box::new(compact::alert::Alerts::new(
                    context.clone(),
                    canvas,
                    20.0,
                )?)),
            )
        };
        let overlay_state = Rc::new(RefCell::new(crate::overlay_state::State::default()));
        model.overlay_state(overlay_state.clone());
        hud.overlay_state(overlay_state.clone());
        let mut label = UnifiedLabel::new("start the car to\nuse openpilot");
        label.size = 54.0;
        label.font = Font::Display;
        label.color = color(255, 255, 255, 229);
        label.horizontal = Horizontal::Center;
        label.vertical = Vertical::Middle;
        Ok(Self {
            overlay_state,
            state: WidgetState::default(),
            calibration: Default::default(),
            camera,
            model,
            hud,
            alerts,
            driver: onroad::driver_preview::DriverPreview::new(context.clone(), canvas, name)?,
            vision: (!big).then(|| compact::vision_renderer::VisionRenderer::new(context.clone())),
            confidence: (!big).then(|| {
                compact::confidence_ball::ConfidenceBall::new(context.clone(), 20.0, false)
            }),
            traffic: (!big).then(|| compact::traffic_light::TrafficLight::new(context.clone())),
            fade: if big {
                None
            } else {
                Some(canvas.texture("icons_mici/onroad/onroad_fade.png", Default::default())?)
            },
            context,
            suppress_camera: false,
            recording,
            label,
            plot_mode: 0,
            plot_next: 0.0,
            publisher: openpilot_messaging::runtime::PubMaster::for_runtime(&["uiDebug"])
                .map_err(|error| Error::Io(std::io::Error::other(error)))?,
            border_params: crate::cache::TimedCache::new(border::Parameters::default()),
            nav_map: onroad::nav_map::NavMap::new(),
            diagnostics: if big {
                Some(crate::render_diagnostics::RenderDiagnostics::new("ui")?)
            } else {
                None
            },
        })
    }
    pub fn set_cluster_hud_connected(&mut self, connected: bool, show_camera: bool) {
        self.suppress_camera = connected && !show_camera;
    }
    pub fn is_swiping_left(&self) -> bool {
        false
    }
    pub fn road_view_mode(&self) -> Result<i32, crate::Error> {
        let ui = self.context.ui.borrow();
        Ok(if ui.started {
            ui.slow.show_model_view.clamp(0, 3)
        } else {
            0
        })
    }
    fn switch(&mut self) -> Result<(), Error> {
        let sm = self.context.messages.borrow();
        let wide = self
            .camera
            .available_streams
            .contains(&VisionStream::WideRoad);
        let experimental = messages::selfdrive_state(&sm.state)?.get_experimental_mode();
        let speed = f64::from(messages::car_state(&sm.state)?.get_v_ego());
        let target = if experimental && wide {
            let (low, high) = if self.context.big {
                (10.0, 15.0)
            } else {
                (5.0, 10.0)
            };
            if speed < low {
                VisionStream::WideRoad
            } else if speed > high {
                VisionStream::Road
            } else {
                self.camera.stream()
            }
        } else {
            VisionStream::Road
        };
        self.camera.switch_stream(target)
    }
    fn render_child(
        widget: &mut dyn Widget,
        rect: Rect,
        frame: &Frame<'_>,
        draw: &mut dyn Draw,
    ) -> Result<(), Error> {
        widget.set_rect(rect);
        widget.render(frame, draw)?;
        Ok(())
    }
}
impl Widget for Road {
    fn state(&self) -> &WidgetState {
        &self.state
    }
    fn state_mut(&mut self) -> &mut WidgetState {
        &mut self.state
    }
    fn update(&mut self, _: &Frame<'_>, _: &mut dyn Draw) -> Result<(), Error> {
        let ui = self.context.ui.borrow();
        self.label.text = (if ui.panda_type == 0 {
            "system booting"
        } else if ui.ignition && !ui.started {
            "openpilot can't start\ncheck alerts"
        } else {
            "start the car to\nuse openpilot"
        })
        .to_owned()
        .into();
        Ok(())
    }
    fn mouse_press(&mut self, _: Point, _: &Frame<'_>, _: &mut dyn Draw) -> Result<(), Error> {
        if let Hud::Big(hud) = &self.hud {
            if !hud.user_interacting() {
                if let Some(callback) = self.state.click.as_mut() {
                    callback();
                }
            }
        }
        Ok(())
    }
    fn mouse_release(
        &mut self,
        _: Point,
        frame: &Frame<'_>,
        _: &mut dyn Draw,
    ) -> Result<(), Error> {
        if !self.context.big {
            self.state.release(frame.now);
        }
        Ok(())
    }
    fn paint(&mut self, frame: &Frame<'_>, draw: &mut dyn Draw) -> Result<RenderResult, Error> {
        let now = (self.context.now_monotonic)();
        if now >= self.plot_next {
            self.plot_next = now + 2.0;
            self.plot_mode = self
                .context
                .params
                .integer("ShowPlotMode")
                .unwrap_or(0)
                .clamp(0, 255);
        }
        let big = self.context.big;
        let started = self.context.ui.borrow().started;
        if big && !started {
            return Ok(RenderResult::None);
        }
        let elapsed = std::time::Instant::now();
        let mut timing = self.diagnostics.take();
        if let Some(timing) = &mut timing {
            timing.start();
        }
        let rect = self.state.rect;
        let content = if big {
            Rect {
                x: float(f64::from(rect.x) + 30.0),
                y: float(f64::from(rect.y) + 30.0),
                width: rect.width - 60.0,
                height: rect.height - 60.0,
            }
        } else {
            Rect {
                width: rect.width - 60.0,
                ..rect
            }
        };
        if !self.suppress_camera {
            self.switch()?;
            self.calibration.update(&self.context)?;
        }
        draw.scissor(Some(Rect {
            x: content.x.trunc(),
            y: content.y.trunc(),
            width: content.width.trunc(),
            height: content.height.trunc(),
        }))?;
        let mode = if big { 0 } else { self.road_view_mode()? };
        let mut values = [0.0; 6];
        let mut measure =
            |index: usize, run: &mut dyn FnMut() -> Result<(), Error>| -> Result<(), Error> {
                let start = std::time::Instant::now();
                let result = match &mut timing {
                    Some(timing) => timing
                        .call(
                            ["camera", "model", "driver_state", "hud", "alert", "border"][index],
                            || run().map_err(crate::Error::from),
                        )
                        .map_err(Error::from),
                    None => run(),
                };
                values[index] += start.elapsed().as_secs_f64() * 1000.0;
                result
            };
        measure(0, &mut || {
            if self.suppress_camera || matches!(mode, 2 | 3) {
                draw.rounded(content, 0.0, color(0, 0, 0, 255))?;
            } else {
                self.camera.prepare(frame, draw)?;
                if self.camera.frame().is_some() {
                    let speed = f64::from(
                        messages::car_state(&self.context.messages.borrow().state)?.get_v_ego(),
                    );
                    let (camera, model) =
                        self.calibration
                            .matrices(content, self.camera.stream(), speed, big)?;
                    self.camera.transform = onroad::camera::Transform::Matrix(camera);
                    self.model.transform(model);
                }
                Self::render_child(
                    &mut self.camera,
                    if big { rect } else { content },
                    frame,
                    draw,
                )?;
            }
            Ok(())
        })?;
        self.overlay_state.borrow_mut().update(
            &self.context.ui.borrow(),
            &self.context.messages.borrow().state,
            self.camera.overlay_camera(),
            if big {
                crate::overlay_state::Mode::Large(self.suppress_camera)
            } else {
                crate::overlay_state::Mode::Compact((self.suppress_camera, mode))
            },
            !frame.events.is_empty(),
        )?;
        if !self.suppress_camera && (big || matches!(mode, 0 | 2)) {
            measure(1, &mut || {
                Self::render_child(self.model.widget(), content, frame, draw)
            })?;
        }
        if big {
            // Under the HUD, so speeds, signs and alerts stay on top of it.
            if !self.suppress_camera {
                self.nav_map.draw(&self.context, content, now, draw)?;
            }
            measure(3, &mut || {
                Self::render_child(self.hud.widget(), content, frame, draw)
            })?;
            measure(4, &mut || {
                Self::render_child(self.alerts.widget(), content, frame, draw)
            })?;
            measure(2, &mut || {
                let alert = match &self.alerts {
                    Alerts::Big(alerts) => alerts.current()?.is_some(),
                    _ => false,
                };
                self.driver
                    .draw_onroad(content, alert, frame.monotonic, draw)?;
                Ok(())
            })?;
        } else {
            if !self.suppress_camera {
                measure(5, &mut || {
                    if let Some(texture) = self.fade {
                        paint::image(
                            draw,
                            paint::Image {
                                texture,
                                rect: Rect {
                                    x: content.x,
                                    y: content.y,
                                    width: texture.width,
                                    height: texture.height,
                                },
                                origin: Point::default(),
                                rotation: 0.0,
                                tint: color(255, 255, 255, 255),
                            },
                        )?;
                    }
                    Ok(())
                })?;
            }
            let (alert, absent) = match &mut self.alerts {
                Alerts::Compact(alerts) => alerts.will_render()?,
                Alerts::Big(_) => return Err(Error::Contract("compact alert renderer")),
            };
            measure(2, &mut || {
                self.driver
                    .draw_onroad(rect, alert.is_some(), frame.monotonic, draw)?;
                Ok(())
            })?;
            if let Hud::Compact(hud) = &mut self.hud {
                hud.set_can_draw_top_icons(alert.is_none());
                hud.set_wheel_critical_icon(
                    alert
                        .as_ref()
                        .is_some_and(|alert| !absent && alert.visual_alert == 2),
                );
            }
            measure(3, &mut || {
                Self::render_child(self.hud.widget(), content, frame, draw)?;
                if let Some(vision) = &mut self.vision {
                    Self::render_child(vision, content, frame, draw)?;
                }
                Ok(())
            })?;
            if started {
                measure(4, &mut || {
                    Self::render_child(self.alerts.widget(), content, frame, draw)
                })?;
            }
            measure(5, &mut || {
                draw.rounded_outline(
                    content,
                    RoundedOutline {
                        roundness: 0.2 * 1.02,
                        segments: 10,
                        thickness: 50.0,
                        color: color(0, 0, 0, 255),
                    },
                )
            })?;
        }
        draw.scissor(None)?;
        measure(5, &mut || {
            if big {
                self.border(draw, rect)?;
            } else {
                if let Some(traffic) = &mut self.traffic {
                    Self::render_child(traffic, rect, frame, draw)?;
                    if !traffic.visible() {
                        if let Some(confidence) = &mut self.confidence {
                            Self::render_child(confidence, rect, frame, draw)?;
                        }
                    }
                }
                if !started {
                    draw.rounded(
                        Rect {
                            x: rect.x.trunc(),
                            y: rect.y.trunc(),
                            width: rect.width.trunc(),
                            height: rect.height.trunc(),
                        },
                        0.0,
                        color(0, 0, 0, 175),
                    )?;
                    Self::render_child(&mut self.label, content, frame, draw)?;
                }
                if self.recording.get() {
                    draw.circle(
                        Point {
                            x: float((f64::from(content.x) + 16.0).trunc()),
                            y: float(
                                (f64::from(content.y) + f64::from(content.height) - 16.0).trunc(),
                            ),
                        },
                        6.0,
                        color(255, 0, 0, 220),
                    )?;
                }
            }
            Ok(())
        })?;
        let mut message = capnp::message::Builder::new_default();
        let mut event = message.init_root::<openpilot_cereal::log_capnp::event::Builder>();
        event.set_valid(true);
        event.set_log_mono_time(
            (now * 1e9)
                .to_u64()
                .ok_or(Error::Contract("invalid UI diagnostic timestamp"))?,
        );
        let mut debug = event.init_ui_debug();
        debug.set_draw_time_millis(float(elapsed.elapsed().as_secs_f64() * 1000.0));
        debug.set_camera_time_millis(float(values[0]));
        debug.set_model_time_millis(float(values[1]));
        debug.set_driver_state_time_millis(float(values[2]));
        debug.set_hud_time_millis(float(values[3]));
        debug.set_alert_time_millis(float(values[4]));
        debug.set_extras_time_millis(float(values[5]));
        debug.set_plot_mode(
            u8::try_from(self.plot_mode).map_err(|_| Error::Contract("uiDebug plot mode"))?,
        );
        debug.set_recording(self.recording.get());
        self.publisher
            .send(
                "uiDebug",
                &capnp::serialize::write_message_to_words(&message),
            )
            .map_err(|error| Error::Io(std::io::Error::other(error)))?;
        if let Some(mut timing) = timing {
            timing.finish()?;
            self.diagnostics = Some(timing);
        }
        Ok(RenderResult::None)
    }
}
