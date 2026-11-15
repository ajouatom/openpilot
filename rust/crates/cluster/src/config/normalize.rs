use super::{
    CLUSTER_PANEL_LAYOUT_DRIVING_LEFT, CLUSTER_PANEL_LAYOUT_DRIVING_RIGHT,
    CLUSTER_SCREEN_MODE_DEBUG, CLUSTER_SCREEN_MODE_DEBUG_GRAPH,
    CLUSTER_SCREEN_MODE_DEBUG_GRAPH_RIGHT, CLUSTER_SCREEN_MODE_DEBUG_SYSTEM,
    CLUSTER_SCREEN_MODE_DEFAULT, CLUSTER_SCREEN_MODE_FULLSCREEN_3D, CLUSTER_SCREEN_MODE_NAVI,
    CLUSTER_SCREEN_MODE_NAV_MODEL, CLUSTER_SCREEN_MODE_TRIP_REPORT,
};
use openpilot_ui_application::params::typed::integer_text;

#[must_use]
pub fn normalize_cluster_panel_layout(value: &str) -> i64 {
    let normalized = value
        .trim_matches(openpilot_startup_ui::text::whitespace)
        .to_lowercase();
    match normalized.as_str() {
        "default" | "driving-left" | "driving_left" | "camera-left" | "camera_left" => {
            CLUSTER_PANEL_LAYOUT_DRIVING_LEFT
        }
        "swapped" | "swap" | "driving-right" | "driving_right" | "camera-right"
        | "camera_right" => CLUSTER_PANEL_LAYOUT_DRIVING_RIGHT,
        _ => {
            if integer_text(&normalized).and_then(|v| v.parse::<i64>().ok())
                == Some(CLUSTER_PANEL_LAYOUT_DRIVING_RIGHT)
            {
                CLUSTER_PANEL_LAYOUT_DRIVING_RIGHT
            } else {
                CLUSTER_PANEL_LAYOUT_DRIVING_LEFT
            }
        }
    }
}

#[must_use]
pub fn normalize_cluster_screen_mode(value: &str) -> i64 {
    let normalized = value
        .trim_matches(openpilot_startup_ui::text::whitespace)
        .to_lowercase();
    match normalized.as_str() {
        "3d-fullscreen" | "3d_fullscreen" | "fullscreen-3d" | "fullscreen_3d" => {
            CLUSTER_SCREEN_MODE_FULLSCREEN_3D
        }
        "default" => CLUSTER_SCREEN_MODE_DEFAULT,
        "debug" => CLUSTER_SCREEN_MODE_DEBUG,
        "system" | "debug-system" | "debug_system" => CLUSTER_SCREEN_MODE_DEBUG_SYSTEM,
        "graph" | "graph-full" | "graph_full" | "debug-graph" | "debug_graph" => {
            CLUSTER_SCREEN_MODE_DEBUG_GRAPH
        }
        "graph-right" | "graph_right" | "debug-graph-right" | "debug_graph_right" => {
            CLUSTER_SCREEN_MODE_DEBUG_GRAPH_RIGHT
        }
        "trip-report" | "trip_report" | "report" | "driving-report" | "driving_report" => {
            CLUSTER_SCREEN_MODE_TRIP_REPORT
        }
        "navi" | "navigation" => CLUSTER_SCREEN_MODE_NAVI,
        "nav-model" => CLUSTER_SCREEN_MODE_NAV_MODEL,
        _ => integer_text(&normalized)
            .and_then(|v| v.parse::<i64>().ok())
            .filter(|value| {
                [
                    CLUSTER_SCREEN_MODE_FULLSCREEN_3D,
                    CLUSTER_SCREEN_MODE_DEFAULT,
                    CLUSTER_SCREEN_MODE_DEBUG,
                    CLUSTER_SCREEN_MODE_DEBUG_SYSTEM,
                    CLUSTER_SCREEN_MODE_DEBUG_GRAPH,
                    CLUSTER_SCREEN_MODE_DEBUG_GRAPH_RIGHT,
                    CLUSTER_SCREEN_MODE_TRIP_REPORT,
                    CLUSTER_SCREEN_MODE_NAVI,
                    CLUSTER_SCREEN_MODE_NAV_MODEL,
                ]
                .contains(value)
            })
            .unwrap_or(CLUSTER_SCREEN_MODE_DEFAULT),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn nav_model_mode_is_accepted_as_carrot_takes_it() {
        assert_eq!(
            normalize_cluster_screen_mode("7"),
            CLUSTER_SCREEN_MODE_NAV_MODEL
        );
        assert_eq!(
            normalize_cluster_screen_mode(" nav-model "),
            CLUSTER_SCREEN_MODE_NAV_MODEL
        );
        assert_eq!(
            normalize_cluster_screen_mode("8"),
            CLUSTER_SCREEN_MODE_DEFAULT
        );
    }
}
