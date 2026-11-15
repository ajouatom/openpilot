use super::SignalSide;
use crate::config::{
    CLUSTER_SCREEN_MODE_DEBUG, CLUSTER_SCREEN_MODE_DEBUG_GRAPH,
    CLUSTER_SCREEN_MODE_DEBUG_GRAPH_RIGHT, CLUSTER_SCREEN_MODE_DEBUG_SYSTEM,
    CLUSTER_SCREEN_MODE_DEFAULT, CLUSTER_SCREEN_MODE_FULLSCREEN_3D, CLUSTER_SCREEN_MODE_NAVI,
    CLUSTER_SCREEN_MODE_NAV_MODEL, CLUSTER_SCREEN_MODE_TRIP_REPORT,
};
use crate::models::ClusterUiState;
use crate::renderer::navigation::policy::{live_panel_visible, map_frame_present};
use crate::renderer::tokens::{
    DEBUG_PLOT_FULL_H, DEBUG_PLOT_FULL_W, DEBUG_PLOT_FULL_X, DEBUG_PLOT_FULL_Y, DEBUG_PLOT_RIGHT_H,
    DEBUG_PLOT_RIGHT_W, DEBUG_PLOT_RIGHT_X, DEBUG_PLOT_RIGHT_Y, FULLSCREEN_3D_CLOCK_CENTER_X,
    FULLSCREEN_3D_CORE_USAGE_RIGHT_X, FULLSCREEN_3D_SIDE_WIDGET_OFFSET_X,
    FULLSCREEN_3D_TRAFFIC_PANEL_RIGHT, NAVI_TRAFFIC_PANEL_RIGHT, NAVI_TRAFFIC_PANEL_Y,
    NAVI_WORLD_VIEW_SHIFT_X, TOP_CLOCK_CENTER_X,
};
use crate::renderer::{ClusterUiRenderer, FrameClock, MediaFit, Rect, Result};

impl ClusterUiRenderer {
    /// # Errors
    /// Returns child surface errors while restoring the active matrix stack.
    pub fn draw_hud(
        &mut self,
        state: &ClusterUiState,
        clock: FrameClock,
        lights: Option<(bool, bool)>,
    ) -> Result<()> {
        let lights = lights.unwrap_or_else(|| self.hud.signals.lights(state, clock.perf_counter));
        let profile_begin = self.profile_start();
        self.gfx.push_matrix()?;
        let result = (|| {
            self.gfx
                .scale(self.width / 1920.0, self.height / 480.0, 1.0)?;
            self.profile_add("hud.push_scale", profile_begin);
            self.draw_hud_contents(state, clock, lights)
        })();
        let profile_begin = self.profile_start();
        let cleanup = self.gfx.pop_matrix();
        self.profile_add("hud.pop_matrix", profile_begin);
        result.and(cleanup)
    }

    fn draw_hud_contents(
        &mut self,
        state: &ClusterUiState,
        clock: FrameClock,
        lights: (bool, bool),
    ) -> Result<()> {
        let mode = self.effective_screen_mode(state);
        if mode == CLUSTER_SCREEN_MODE_NAV_MODEL {
            self.draw_driving_hud_content(state, mode, lights)?;
            self.hud_stage("hud.nav_model_map", Self::draw_nav_model_map)?;
            return self.draw_status_footer(state, true);
        }
        if mode == CLUSTER_SCREEN_MODE_NAVI {
            self.draw_navi_dashboard(state)?;
            return self.draw_status_footer(state, true);
        }
        if mode == CLUSTER_SCREEN_MODE_DEBUG_GRAPH {
            self.hud_stage("hud.speed_block", |renderer| {
                renderer.draw_speed_block(state, 0.0)
            })?;
            self.hud_stage("hud.accel_block", |renderer| {
                renderer.draw_accel_block(state)
            })?;
            self.draw_steering_output_block(state)?;
            self.hud_stage("hud.debug_plot_full", |renderer| {
                renderer.draw_debug_plot(
                    state.debug_plot.as_ref(),
                    Rect::new(
                        DEBUG_PLOT_FULL_X,
                        DEBUG_PLOT_FULL_Y,
                        DEBUG_PLOT_FULL_W,
                        DEBUG_PLOT_FULL_H,
                    ),
                    clock.perf_counter,
                )
            })?;
            return self.draw_status_footer(state, true);
        }
        self.draw_driving_hud_content(state, mode, lights)?;
        if mode == CLUSTER_SCREEN_MODE_FULLSCREEN_3D {
            return self.draw_status_footer(state, true);
        }
        if mode == CLUSTER_SCREEN_MODE_DEBUG_SYSTEM {
            self.hud_stage("hud.system_dashboard", |renderer| {
                renderer.draw_system_dashboard_panel(state)
            })?;
            return self.draw_status_footer(state, true);
        }
        self.draw_hud_information(state, mode, clock)?;
        self.draw_status_footer(state, mode != CLUSTER_SCREEN_MODE_TRIP_REPORT)
    }

    fn draw_hud_information(
        &mut self,
        state: &ClusterUiState,
        mode: i64,
        clock: FrameClock,
    ) -> Result<()> {
        let debug = state.navi_debug.is_some();
        let navigation = live_panel_visible(state.navi_live.as_ref())
            || map_frame_present(state.navi_dashboard.as_ref())
            || mode == CLUSTER_SCREEN_MODE_DEFAULT && state.navi_dashboard.is_some();
        if mode == CLUSTER_SCREEN_MODE_DEBUG {
            self.hud_stage("hud.live_debug", |renderer| {
                renderer.draw_live_debug_panel(state)
            })?;
        }
        if mode == CLUSTER_SCREEN_MODE_DEBUG_GRAPH_RIGHT {
            self.hud_stage("hud.debug_plot_right", |renderer| {
                renderer.draw_debug_plot(
                    state.debug_plot.as_ref(),
                    Rect::new(
                        renderer.information_panel_x(DEBUG_PLOT_RIGHT_X),
                        DEBUG_PLOT_RIGHT_Y,
                        DEBUG_PLOT_RIGHT_W,
                        DEBUG_PLOT_RIGHT_H,
                    ),
                    clock.perf_counter,
                )
            })?;
        }
        if mode == CLUSTER_SCREEN_MODE_TRIP_REPORT {
            self.hud_stage("hud.trip_report", |renderer| {
                renderer.draw_trip_report_panel(state)
            })?;
        } else if debug {
            self.hud_stage("hud.navi_debug", |renderer| {
                renderer.draw_navi_debug_panel(state.navi_debug.as_ref())
            })?;
        } else if mode == CLUSTER_SCREEN_MODE_DEFAULT && navigation {
            self.hud_stage("hud.navi_live", |renderer| {
                renderer.draw_navi_live_panel(state, clock.wall_time)
            })?;
        }
        if !matches!(
            mode,
            CLUSTER_SCREEN_MODE_DEBUG
                | CLUSTER_SCREEN_MODE_DEBUG_GRAPH
                | CLUSTER_SCREEN_MODE_DEBUG_GRAPH_RIGHT
                | CLUSTER_SCREEN_MODE_TRIP_REPORT
        ) && !debug
            && !navigation
        {
            self.hud_stage("hud.route_overlay", |renderer| {
                renderer.draw_route_overlay(state.route_overlay.as_ref())
            })?;
        }
        Ok(())
    }

    fn draw_driving_hud_content(
        &mut self,
        state: &ClusterUiState,
        mode: i64,
        lights: (bool, bool),
    ) -> Result<()> {
        let offset = self.hud_offset(mode);
        let widgets = if mode == CLUSTER_SCREEN_MODE_FULLSCREEN_3D {
            FULLSCREEN_3D_SIDE_WIDGET_OFFSET_X
        } else {
            0.0
        };
        self.gfx.push_matrix()?;
        let result = (|| {
            if offset.abs() > 0.001 {
                self.gfx.translate(offset, 0.0, 0.0)?;
            }
            self.hud_stage("hud.speed_block", |renderer| {
                renderer.draw_speed_block(state, widgets)
            })?;
            self.draw_hud_side_gauges(state, widgets)?;
            self.hud_stage("hud.turn_signal_left", |renderer| {
                renderer.draw_turn_signal(
                    SignalSide::Left,
                    lights.0,
                    state.debug_ui_visible,
                    renderer.turn_signal_center_x_offset(state, "left"),
                )
            })?;
            self.hud_stage("hud.drive_status", |renderer| {
                renderer.draw_drive_status(state)
            })?;
            self.hud_stage("hud.turn_signal_right", |renderer| {
                renderer.draw_turn_signal(
                    SignalSide::Right,
                    lights.1,
                    state.debug_ui_visible,
                    renderer.turn_signal_center_x_offset(state, "right"),
                )
            })?;
            self.draw_hud_traffic(state, mode)?;
            let center = if mode == CLUSTER_SCREEN_MODE_FULLSCREEN_3D {
                FULLSCREEN_3D_CLOCK_CENTER_X
            } else {
                TOP_CLOCK_CENTER_X
            };
            self.hud_stage("hud.center_clock", |renderer| {
                renderer.draw_center_clock(state, center)
            })
        })();
        let cleanup = self.gfx.pop_matrix();
        result.and(cleanup)
    }

    fn draw_hud_side_gauges(&mut self, state: &ClusterUiState, offset: f64) -> Result<()> {
        let translated = offset.abs() > 0.001;
        if translated {
            self.gfx.push_matrix()?;
        }
        let result = (|| {
            if translated {
                self.gfx.translate(offset, 0.0, 0.0)?;
            }
            self.hud_stage("hud.accel_block", |renderer| {
                renderer.draw_accel_block(state)
            })?;
            self.draw_steering_output_block(state)
        })();
        let cleanup = if translated {
            self.gfx.pop_matrix()
        } else {
            Ok(())
        };
        result.and(cleanup)
    }

    fn draw_hud_traffic(&mut self, state: &ClusterUiState, mode: i64) -> Result<()> {
        if mode == CLUSTER_SCREEN_MODE_TRIP_REPORT
            || mode == CLUSTER_SCREEN_MODE_DEFAULT
                && map_frame_present(state.navi_dashboard.as_ref())
        {
            return Ok(());
        }
        let frame = state
            .navi_dashboard
            .as_ref()
            .filter(|dashboard| dashboard.connected)
            .and_then(|dashboard| {
                dashboard
                    .media
                    .iter()
                    .find(|frame| frame.key == "image:traffic_signal")
            });
        let Some(frame) = frame else {
            return Ok(());
        };
        let right = if mode == CLUSTER_SCREEN_MODE_FULLSCREEN_3D {
            FULLSCREEN_3D_TRAFFIC_PANEL_RIGHT
        } else {
            NAVI_TRAFFIC_PANEL_RIGHT
                - if self.panel_swap_active() {
                    NAVI_WORLD_VIEW_SHIFT_X
                } else {
                    0.0
                }
        };
        self.hud_stage("hud.navi_traffic", |renderer| {
            renderer
                .draw_navi_media(
                    Some(frame),
                    Rect::new(right - 230.0, NAVI_TRAFFIC_PANEL_Y, 230.0, 98.0),
                    MediaFit {
                        align_x: 1.0,
                        align_y: 0.0,
                        ..Default::default()
                    },
                )
                .map(|_| ())
        })
    }

    fn draw_status_footer(&mut self, state: &ClusterUiState, include_core: bool) -> Result<()> {
        let profile_begin = self.profile_start();
        let mode = self.effective_screen_mode(state);
        let offset = self.hud_offset(mode);
        let translated = offset.abs() > 0.001;
        if translated {
            self.gfx.push_matrix()?;
        }
        let result = (|| {
            if translated {
                self.gfx.translate(offset, 0.0, 0.0)?;
            }
            self.draw_git_status(
                state.git_status.as_ref(),
                state.network_address.as_deref(),
                state.actual_fps,
            )
        })();
        let cleanup = if translated {
            self.gfx.pop_matrix()
        } else {
            Ok(())
        };
        result.and(cleanup)?;
        self.profile_add("hud.git_status", profile_begin);
        if include_core {
            let right = if mode == CLUSTER_SCREEN_MODE_FULLSCREEN_3D {
                FULLSCREEN_3D_CORE_USAGE_RIGHT_X
            } else {
                self.information_panel_x(1920.0)
            };
            self.hud_stage("hud.cluster_core_usage", |renderer| {
                renderer.draw_cluster_core_usage(state.cluster_core_usage_text.as_deref(), right)
            })?;
        }
        Ok(())
    }
    fn hud_offset(&self, mode: i64) -> f64 {
        if mode == CLUSTER_SCREEN_MODE_FULLSCREEN_3D {
            0.0
        } else {
            self.driving_panel_offset_design_x()
        }
    }
    fn hud_stage(&mut self, name: &str, draw: impl FnOnce(&mut Self) -> Result<()>) -> Result<()> {
        let start = self.profile_start();
        draw(self)?;
        self.profile_add(name, start);
        Ok(())
    }
}
