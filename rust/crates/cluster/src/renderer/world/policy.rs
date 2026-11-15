use crate::{
    config::{
        cluster_camera_view_is_road_camera, CLUSTER_SCREEN_MODE_DEBUG,
        CLUSTER_SCREEN_MODE_DEBUG_GRAPH_RIGHT, CLUSTER_SCREEN_MODE_DEBUG_SYSTEM,
        CLUSTER_SCREEN_MODE_DEFAULT, CLUSTER_SCREEN_MODE_FULLSCREEN_3D,
        CLUSTER_SCREEN_MODE_TRIP_REPORT,
    },
    models::ClusterUiState,
    renderer::{
        navigation::policy::{live_panel_visible, map_frame_present},
        tokens::{
            CAMERA_BACKGROUND_W, CAMERA_BACKGROUND_X, LANE_TURN_SIGNAL_LEFT_CENTER_X,
            LANE_TURN_SIGNAL_RIGHT_CENTER_X, NAVI_WORLD_VIEW_SHIFT_X,
        },
        ClusterUiRenderer,
    },
};
impl ClusterUiRenderer {
    #[must_use]
    pub fn world_view_shift_x(&self, state: &ClusterUiState) -> f64 {
        let mode = self.effective_screen_mode(state);
        if cluster_camera_view_is_road_camera(state.camera_view_mode) {
            return 0.;
        }
        if matches!(
            mode,
            CLUSTER_SCREEN_MODE_DEBUG
                | CLUSTER_SCREEN_MODE_DEBUG_SYSTEM
                | CLUSTER_SCREEN_MODE_DEBUG_GRAPH_RIGHT
                | CLUSTER_SCREEN_MODE_TRIP_REPORT
                | crate::config::CLUSTER_SCREEN_MODE_NAV_MODEL
        ) {
            return NAVI_WORLD_VIEW_SHIFT_X * self.width / super::DESIGN_WIDTH;
        }
        if mode != CLUSTER_SCREEN_MODE_DEFAULT {
            return 0.;
        }
        if live_panel_visible(state.navi_live.as_ref())
            || map_frame_present(state.navi_dashboard.as_ref())
            || state.navi_dashboard.is_some()
        {
            NAVI_WORLD_VIEW_SHIFT_X * self.width / super::DESIGN_WIDTH
        } else {
            0.
        }
    }
    #[must_use]
    pub fn world_view_shift_design_x(&self, state: &ClusterUiState) -> f64 {
        if self.width <= 0. {
            0.
        } else {
            self.world_view_shift_x(state) * super::DESIGN_WIDTH / self.width
        }
    }
    #[must_use]
    pub fn turn_signal_center_x_offset(&self, state: &ClusterUiState, side: &str) -> f64 {
        let center = if side == "left" {
            LANE_TURN_SIGNAL_LEFT_CENTER_X
        } else {
            LANE_TURN_SIGNAL_RIGHT_CENTER_X
        };
        if self.effective_screen_mode(state) == CLUSTER_SCREEN_MODE_FULLSCREEN_3D {
            return 0.;
        }
        if cluster_camera_view_is_road_camera(state.camera_view_mode) {
            return CAMERA_BACKGROUND_X + center * CAMERA_BACKGROUND_W / super::DESIGN_WIDTH
                - center;
        }
        let shift = self.world_view_shift_design_x(state);
        -(if self.panel_swap_active() && shift.abs() <= 0.001 {
            NAVI_WORLD_VIEW_SHIFT_X
        } else {
            shift
        })
    }
}
