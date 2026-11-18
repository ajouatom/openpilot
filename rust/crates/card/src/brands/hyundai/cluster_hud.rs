use super::{
    ccnc_display::{self, CcncDisplay, CcncInput, LANE_LINE_UNSEEN},
    cluster_fields,
    wire::{set, Values},
    Error,
};

pub struct HudInput {
    pub frame: u32,
    pub dm_alert: u8,
    pub enabled: bool,
    pub active: bool,
    pub main_enabled: bool,
    pub lat_enabled: bool,
    pub nav_active: bool,
    pub navi_available: bool,
    pub hdp_use: i32,
    pub set_speed: f64,
    pub gap: f64,
    pub lead_visible: bool,
    pub lead_distance: f64,
    pub paddle: bool,
    pub paddle_mode: i32,
    pub hda_mode: f64,
    pub steering_angle: f64,
    pub soft_hold: u8,
    pub trailer: bool,
    pub lane_check: i32,
    pub lane_lines: [i16; 2],
    pub lane_visible: [bool; 2],
    pub lane_depart: [bool; 2],
    pub change_available: [bool; 2],
    pub blindspot: [bool; 2],
    pub blinker: [bool; 2],
    pub desire: u16,
    pub standstill: bool,
    /// CcncClusterDisplay / CcncTrafficBackground inputs (carrot PR580).
    pub ccnc: CcncInput,
}

pub fn hud(data: &mut Values, i: &HudInput, display: &mut CcncDisplay) -> Result<(), Error> {
    let stock = data.clone();
    let hdp = match i.hdp_use {
        1 => i.enabled && i.nav_active,
        2 => i.enabled,
        _ => false,
    };
    let ccnc = &i.ccnc;
    // Steering pauses at standstill; keep the steering/lane-change icons as if
    // it were active.
    let lat_icon = i.active || (i.enabled && i.lat_enabled && i.standstill);
    let traffic_state = if ccnc.traffic_background {
        display.traffic_state(ccnc.traffic_state, i.frame)
    } else {
        0
    };
    set(
        data,
        &[
            (
                "SETSPEED",
                if i.main_enabled {
                    if hdp {
                        6.
                    } else if i.enabled {
                        3.
                    } else {
                        1.
                    }
                } else {
                    0.
                },
            ),
            (
                "SETSPEED_HUD",
                if i.main_enabled {
                    if hdp {
                        5.
                    } else if i.enabled {
                        3.
                    } else {
                        1.
                    }
                } else {
                    0.
                },
            ),
            ("vSetDis", (i.set_speed + 0.5).trunc()),
            // Real gap bars under the HDP colors too.
            ("DISTANCE", i.gap),
            (
                "DISTANCE_LEAD",
                if i.lead_visible {
                    if i.enabled {
                        2.
                    } else if i.main_enabled || ccnc.display {
                        1.
                    } else {
                        0.
                    }
                } else {
                    0.
                },
            ),
            (
                "DISTANCE_CAR",
                if hdp {
                    3.
                } else if i.enabled {
                    2.
                } else if i.main_enabled {
                    1.
                } else {
                    0.
                },
            ),
            (
                "DISTANCE_SPACING",
                if hdp && i.gap != 4. {
                    // HDP spacing style for gaps 1-3.
                    3.
                } else if hdp {
                    5.
                } else if i.enabled {
                    1.
                } else {
                    0.
                },
            ),
            (
                "BACKGROUND",
                ccnc_display::traffic_background(traffic_state).unwrap_or_else(|| {
                    background(
                        i.enabled,
                        i.active,
                        i.paddle,
                        i.paddle_mode,
                        [1, 2].contains(&i.desire),
                    )
                }),
            ),
            ("CENTERLINE", if i.hda_mode > 0. { 1. } else { 0. }),
            (
                "CAR_CIRCLE",
                if hdp {
                    2.
                } else if i.enabled {
                    1.
                } else {
                    0.
                },
            ),
            (
                "NAV_ICON",
                if i.nav_active || i.navi_available {
                    if i.enabled {
                        2.
                    } else if i.main_enabled {
                        1.
                    } else {
                        0.
                    }
                } else {
                    0.
                },
            ),
            (
                "HDA_ICON",
                if hdp {
                    5.
                } else if i.enabled {
                    2.
                } else if i.main_enabled {
                    1.
                } else {
                    0.
                },
            ),
            (
                "LFA_ICON",
                if hdp {
                    5.
                } else if lat_icon {
                    2.
                } else if i.lat_enabled {
                    1.
                } else {
                    0.
                },
            ),
            (
                "LKA_ICON",
                if lat_icon {
                    4.
                } else if i.lat_enabled {
                    3.
                } else {
                    0.
                },
            ),
            ("FCA_ALT_ICON", 0.),
        ],
    );
    if ccnc.display {
        // Blue when engaged, white otherwise; the ring marks the gap
        // openpilot keeps while engaged, else the stock value stays.
        data.insert(
            "TARGET".into(),
            if i.lead_visible {
                if i.enabled {
                    1.
                } else {
                    3.
                }
            } else {
                0.
            },
        );
        if i.enabled && ccnc.desired_distance > 0. {
            data.insert(
                "TARGET_DISTANCE".into(),
                ccnc.desired_distance.clamp(0.1, 204.5),
            );
        }
    } else {
        data.insert(
            "TARGET".into(),
            if i.lead_visible && i.enabled { 1. } else { 0. },
        );
        data.insert("TARGET_DISTANCE".into(), i.lead_distance);
    }
    cluster_fields::hide_alerts(data, true, i.soft_hold, i.trailer)?;
    if ccnc.display {
        display.alerts(data, ccnc, i.enabled, i.frame)?;
    }
    display.gap_popup(data, hdp, i.gap, i.frame)?;
    cluster_fields::driver_monitoring_alert(data, i.dm_alert, &stock)?;
    // An unseen line is drawn as nothing (14), a dashed camera line dashed.
    let lane_display = ccnc.display && !ccnc.lane_wip_car;
    let mut seen = i.lane_visible;
    if lane_display {
        for (index, seen) in seen.iter_mut().enumerate() {
            // Hide only when both miss: the model drops all lines for 2-3 s
            // mid lane change, the camera drops them in 26% of stops.
            let unseen = ccnc
                .lane_probs
                .is_some_and(|probs| display.lane_unseen(index, probs[index], i.frame));
            *seen = *seen && (!unseen || i.lane_lines[index].div_euclid(10) > 0);
        }
    }
    cluster_fields::lane_lines(data, (i.steering_angle, i.active, 0));
    for (side, index) in [("LEFT", 0), ("RIGHT", 1)] {
        let color = if i.change_available[index] { 6. } else { 2. };
        let warn = if i.lane_check >= 1 {
            !matches!(i.lane_lines[index].rem_euclid(10), 0 | 5)
        } else {
            i.lane_lines[index].div_euclid(10) == 2
        };
        let color = if warn || i.blindspot[index] {
            4.
        } else {
            color
        };
        let lane = if i.trailer {
            if i.lane_visible[index] {
                2.
            } else {
                0.
            }
        } else if i.lane_depart[index] {
            if (i.frame / 50).is_multiple_of(2) {
                4.
            } else {
                1.
            }
        } else if lane_display {
            // An orange warning line stays.
            if seen[index] || color == 4. {
                ccnc_display::dashed_lane_line(color, i.lane_lines[index], ccnc.non_dbc_car)
            } else {
                LANE_LINE_UNSEEN
            }
        } else if i.lane_visible[index] {
            color
        } else {
            0.
        };
        data.insert(
            std::sync::Arc::<str>::from(format!("LANELINE_{side}")),
            lane,
        );
        data.insert(
            std::sync::Arc::<str>::from(format!("LCA_{side}_ARROW")),
            if i.blinker[index] { 2. } else { 0. },
        );
        data.insert(
            std::sync::Arc::<str>::from(format!("LCA_{side}_ICON")),
            if lat_icon {
                if i.trailer || i.blindspot[index] {
                    1.
                } else {
                    2.
                }
            } else {
                0.
            },
        );
        let desire = match index {
            0 => matches!(i.desire, 1 | 3),
            1 => matches!(i.desire, 2 | 4),
            _ => false,
        };
        data.insert(
            std::sync::Arc::<str>::from(format!("LANE_{side}")),
            if !i.trailer && desire { 1. } else { 0. },
        );
    }
    if ccnc.display {
        ccnc_display::highlight(data, ccnc.lat_hud, i.desire)?;
    }
    Ok(())
}

pub fn background(
    enabled: bool,
    active: bool,
    paddle: bool,
    paddle_mode: i32,
    turning: bool,
) -> f64 {
    if (active && turning) || (paddle_mode > 0 && paddle) {
        6.
    } else if enabled {
        1.
    } else if active {
        3.
    } else {
        7.
    }
}
