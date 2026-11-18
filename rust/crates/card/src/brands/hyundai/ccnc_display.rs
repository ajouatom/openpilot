//! carrot PR580 `CcncClusterDisplay` / `CcncTrafficBackground` (hyundaicanfd.py):
//! make the stock CCNC cluster (0x161 / 0x162) show openpilot's state the way
//! the stock HDA2 display does. Both keys default to 0, which turns off only
//! the items gated on them. Evidence and per-value cluster tests are in
//! carrot's comments; the values below are copied unchanged.
use super::{wire::Values, Error};

/// 0x161 LANELINE_*: draws nothing on the IONIQ 5 PE cluster (not a DBC value).
pub const LANE_LINE_UNSEEN: f64 = 14.;
/// model laneLineProbs at low speed: parking 0.01..0.04, city stops 0.13..0.22.
const LANE_HIDE_PROB: f64 = 0.08;
const LANE_SHOW_PROB: f64 = 0.12;
const LANE_HIDE_FRAMES: u32 = 100;
/// 0x162 bit 188 (no DBC name): the HDP icon. Stock never sets it.
pub const HDP_ICON_BIT: usize = 188;
/// ALERTS_5 19 "always be ready to drive", 21 "HDP released".
const ENGAGE_POPUP: f64 = 19.;
const DISENGAGE_POPUP: f64 = 21.;
const ENGAGE_POPUP_FRAMES: u32 = 200;
/// Stock pairs CHECK_SURROUNDINGS with SOUNDS_4 2 (double chime) for ~1 s.
const CHIME_TEXT: (&str, f64) = ("ALERTS_3", 18.);
const CHIME_FRAMES: u32 = 100;

/// openpilot alert -> cluster text, keyed by event name. Cruise released and
/// the R-gear disengage are not mirrored: the cluster shows its own text.
fn alert_text(alert_type: &str) -> Option<(&'static str, f64)> {
    let name = alert_type.split('/').next().unwrap_or_default();
    Some(match name {
        "speedTooHigh" => ("ALERTS_4", 83.),
        "steerUnavailable" => ("ALERTS_5", 9.),
        "accFaulted" => ("ALERTS_5", 4.),
        "steerSaturated" => ("ALERTS_3", 18.),
        "controlsMismatch"
        | "canError"
        | "canBusMissing"
        | "vehicleSensorsInvalid"
        | "relayMalfunction"
        | "locationdPermanentError"
        | "paramsdPermanentError" => ("ALERTS_4", 147.),
        _ => return None,
    })
}

/// Of the texts above only ALERTS_5 9 is missing from the shared CAN-FD DBC.
fn dbc_listed(text: (&str, f64)) -> bool {
    text != ("ALERTS_5", 9.)
}

/// Clusters seen to draw values the DBC does not name (ALERTS_5 9, 0x162
/// bit 188, dashed lane codes 8/10): IONIQ 5 PE, GN7, EV4.
pub fn non_dbc_car(candidate: &str) -> bool {
    matches!(
        candidate,
        "HYUNDAI_IONIQ_5_PE" | "HYUNDAI_AZERA_7TH_GEN" | "KIA_EV4"
    )
}

/// IONIQ 9 / EV9 drew no line for a non-DBC lane code: they keep the
/// previous lane values instead of the unseen code 14.
pub fn lane_wip_car(candidate: &str) -> bool {
    matches!(candidate, "HYUNDAI_IONIQ_9" | "KIA_EV9")
}

/// The dashed codes seen on the IONIQ 5 PE cluster: 8 white, 10 green.
pub fn dashed_lane_line(color: f64, lane_line: i16, non_dbc: bool) -> f64 {
    if non_dbc && lane_line.div_euclid(10) > 0 && matches!(lane_line.rem_euclid(10), 0 | 5) {
        if color == 2. {
            return 8.;
        }
        if color == 6. {
            return 10.;
        }
    }
    color
}

/// Red light -> steady red 5, green -> steady green 8 (not DBC values).
pub fn traffic_background(traffic_state: i16) -> Option<f64> {
    match traffic_state {
        1 => Some(5.),
        2 => Some(8.),
        _ => None,
    }
}

/// hudControl.latHudState -> 0x161 LANE_HIGHLIGHT: laneless green, lane mode
/// sky blue, steer warning yellow, saturated red.
fn lane_highlight(lat_hud: u8) -> Option<f64> {
    match lat_hud {
        1 => Some(1.),
        2 => Some(3.),
        3 => Some(4.),
        4 => Some(5.),
        _ => None,
    }
}

/// What one 0x161 frame reads for the display options.
#[derive(Clone, Debug, Default)]
pub struct CcncInput {
    pub display: bool,
    pub traffic_background: bool,
    pub traffic_state: i16,
    pub desired_distance: f64,
    pub alert_type: String,
    pub lat_hud: u8,
    /// laneLineProbs[1], [2] when the model has them.
    pub lane_probs: Option<[f64; 2]>,
    pub non_dbc_car: bool,
    pub lane_wip_car: bool,
}

/// The state carrot keeps on `create_ccnc_messages` and its helpers.
#[derive(Debug, Default)]
pub struct CcncDisplay {
    lane_unseen: [(bool, Option<u32>); 2],
    traffic_last: i16,
    green_until: u32,
    was_enabled: Option<bool>,
    popup: (f64, u32),
    chime_shown: Option<(&'static str, f64)>,
    chime_until: u32,
    gap_bars: Option<f64>,
    gap_popup_until: u32,
}

impl CcncDisplay {
    /// `_lane_unseen`: hide a line after 1 s below 0.08, show again from 0.12.
    pub fn lane_unseen(&mut self, index: usize, prob: f64, frame: u32) -> bool {
        let (mut hidden, mut low_since) = self.lane_unseen[index];
        if prob >= LANE_SHOW_PROB {
            hidden = false;
            low_since = None;
        } else if prob < LANE_HIDE_PROB {
            let since = *low_since.get_or_insert(frame);
            hidden = hidden || frame.wrapping_sub(since) >= LANE_HIDE_FRAMES;
        } else {
            low_since = None;
        }
        self.lane_unseen[index] = (hidden, low_since);
        hidden
    }

    /// `_traffic_display_state`: red while red, green only for 2 s after it
    /// turns green (model green is on for most normal driving).
    pub fn traffic_state(&mut self, traffic_state: i16, frame: u32) -> i16 {
        if traffic_state == 2 && self.traffic_last != 2 {
            self.green_until = frame + 200;
        }
        self.traffic_last = traffic_state;
        if traffic_state == 2 && frame >= self.green_until {
            0
        } else {
            traffic_state
        }
    }

    /// Engage/disengage popups and stock texts for openpilot alerts
    /// (CcncClusterDisplay), after the stock alert/sound filter.
    pub fn alerts(
        &mut self,
        data: &mut Values,
        input: &CcncInput,
        enabled: bool,
        frame: u32,
    ) -> Result<(), Error> {
        let get = |data: &Values, key: &str| super::wire::get(data, key);
        if self.was_enabled.is_some_and(|was| was != enabled) {
            self.popup = (
                if enabled {
                    ENGAGE_POPUP
                } else {
                    DISENGAGE_POPUP
                },
                frame + ENGAGE_POPUP_FRAMES,
            );
        }
        self.was_enabled = Some(enabled);
        let (popup, popup_until) = self.popup;
        if frame < popup_until && get(data, "ALERTS_5")? == 0. {
            data.insert("ALERTS_5".into(), popup);
        }
        // The alert's own text beats the generic disengage popup; a stock
        // text still wins.
        let text = alert_text(&input.alert_type).filter(|t| input.non_dbc_car || dbc_listed(*t));
        let mut shown = None;
        if let Some((field, value)) = text {
            let current = get(data, field)?;
            if current == 0. || (field == "ALERTS_5" && current == popup && frame < popup_until) {
                data.insert(field.into(), value);
                shown = Some((field, value));
            }
        }
        if shown == Some(CHIME_TEXT) && self.chime_shown != shown {
            self.chime_until = frame + CHIME_FRAMES;
        }
        self.chime_shown = shown;
        if shown == Some(CHIME_TEXT)
            && frame < self.chime_until
            && get(data, "SOUNDS_4")? == 0.
            && get(data, "ALERTS_1")? == 0.
        {
            data.insert("SOUNDS_4".into(), 2.);
        }
        Ok(())
    }

    /// With HDP on, a gap change shows the stock "distance N" popup
    /// (ALERTS_3 7..10) for 2 s; an alert text beats it. Not gated.
    pub fn gap_popup(
        &mut self,
        data: &mut Values,
        hdp: bool,
        bars: f64,
        frame: u32,
    ) -> Result<(), Error> {
        if hdp && self.gap_bars.is_some_and(|previous| previous != bars) {
            self.gap_popup_until = frame + 200;
        }
        self.gap_bars = Some(bars);
        if frame < self.gap_popup_until
            && super::wire::get(data, "ALERTS_3")? == 0.
            && (1. ..=4.).contains(&bars)
        {
            data.insert("ALERTS_3".into(), 6. + bars);
        }
        Ok(())
    }
}

/// Lane highlight after the lane lines are decided (CcncClusterDisplay).
pub fn highlight(data: &mut Values, lat_hud: u8, desire: u16) -> Result<(), Error> {
    let mut lat_hud = lat_hud;
    if lat_hud != 0 && matches!(desire, 3 | 4) {
        // Lane change first: stock lights the lane green only while changing.
        lat_hud = 1;
    }
    if super::wire::get(data, "LANELINE_LEFT")? == LANE_LINE_UNSEEN
        || super::wire::get(data, "LANELINE_RIGHT")? == LANE_LINE_UNSEEN
    {
        lat_hud = 0;
    }
    if let Some(value) = lane_highlight(lat_hud) {
        data.insert("LANE_HIGHLIGHT".into(), value);
        // 0: only the patch around the car (stock lane change uses 150 m).
        data.insert("LANE_HIGHLIGHT_DISTANCE".into(), 0.);
    }
    Ok(())
}

/// Set 0x162 bits the DBC does not name, then refresh the CAN-FD checksum.
pub fn set_raw_bits(frame: &mut openpilot_can::Frame, bits: &[usize], value: u32) {
    for (i, bit) in bits.iter().enumerate() {
        let (byte, offset) = (bit / 8, bit % 8);
        if let Some(target) = frame.data.get_mut(byte) {
            *target = (*target & !(1 << offset)) | ((((value >> i) & 1) as u8) << offset);
        }
    }
    if frame.data.len() >= 2 {
        let crc = super::cluster_buttons::checksum(frame.address, &frame.data).to_le_bytes();
        frame.data[..2].copy_from_slice(&crc);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_line_hides_after_a_second_below_the_low_probability() {
        let mut d = CcncDisplay::default();
        assert!(!d.lane_unseen(0, 0.05, 0));
        assert!(!d.lane_unseen(0, 0.05, 99));
        assert!(d.lane_unseen(0, 0.05, 100));
        assert!(d.lane_unseen(0, 0.10, 150), "in between keeps hidden");
        assert!(!d.lane_unseen(0, 0.12, 151));
    }

    #[test]
    fn green_shows_for_two_seconds_and_red_while_red() {
        let mut d = CcncDisplay::default();
        assert_eq!(d.traffic_state(1, 0), 1);
        assert_eq!(d.traffic_state(2, 10), 2);
        assert_eq!(d.traffic_state(2, 209), 2);
        assert_eq!(d.traffic_state(2, 210), 0);
    }

    fn values(pairs: &[(&str, f64)]) -> Values {
        pairs
            .iter()
            .map(|(k, v)| (std::sync::Arc::<str>::from(*k), *v))
            .collect()
    }

    #[test]
    fn engage_popup_then_alert_text_and_chime() {
        let mut d = CcncDisplay::default();
        let input = CcncInput {
            display: true,
            ..CcncInput::default()
        };
        let mut data = values(&[
            ("ALERTS_1", 0.),
            ("ALERTS_3", 0.),
            ("ALERTS_5", 0.),
            ("SOUNDS_4", 0.),
        ]);
        d.alerts(&mut data, &input, false, 0).unwrap();
        d.alerts(&mut data, &input, true, 1).unwrap();
        assert_eq!(data["ALERTS_5"], ENGAGE_POPUP);
        let input = CcncInput {
            alert_type: "steerSaturated/warning".into(),
            ..input
        };
        let mut data = values(&[
            ("ALERTS_1", 0.),
            ("ALERTS_3", 0.),
            ("ALERTS_5", 0.),
            ("SOUNDS_4", 0.),
        ]);
        d.alerts(&mut data, &input, true, 300).unwrap();
        assert_eq!(data["ALERTS_3"], 18.);
        assert_eq!(data["SOUNDS_4"], 2.);
        // ALERTS_5 9 is not a DBC value: only checked clusters show it.
        let input = CcncInput {
            alert_type: "steerUnavailable/permanent".into(),
            ..input
        };
        let mut data = values(&[
            ("ALERTS_1", 0.),
            ("ALERTS_3", 0.),
            ("ALERTS_5", 0.),
            ("SOUNDS_4", 0.),
        ]);
        d.alerts(&mut data, &input, true, 400).unwrap();
        assert_eq!(data["ALERTS_5"], 0.);
    }

    #[test]
    fn dashed_codes_only_on_checked_clusters() {
        assert_eq!(dashed_lane_line(2., 10, true), 8.);
        assert_eq!(dashed_lane_line(6., 15, true), 10.);
        assert_eq!(dashed_lane_line(2., 11, true), 2.);
        assert_eq!(dashed_lane_line(2., 10, false), 2.);
    }
}
