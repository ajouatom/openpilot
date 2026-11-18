use super::{constants::*, source::position_continuous, Matcher};
use crate::{math::maximum, model::VisionLead, point::Point};

impl Matcher {
    pub fn reset(&mut self) {
        self._closing_range_history.clear();
        self.closing_range_match = None;
        self._corner_motion_history.clear();
        self._corner_motion_unreliable_until.clear();
        self.reset_moving();
        self.reset_stationary();
        self.reset_radar_only_moving();
        self.reset_far_vision();
        self.reset_rejected_moving();
        self._vision_fallback = None;
        self._vision_fallback_hold_frames = 0;
        self._stationary_front_evidence.clear();
        self._moving_vision_evidence.clear();
    }
    pub fn reset_moving(&mut self) {
        self.last_identity = None;
        self.low_probability_hold_frames = 0;
    }
    pub fn reset_stationary(&mut self) {
        self.stationary_identity = None;
        self._stationary_pending_identity = None;
        self._stationary_pending_since_s = None;
        self._stationary_pending_vision_support_frames = 0;
        self._stationary_pending_last_support_time_s = None;
        self._stationary_last_point = None;
        self._stationary_last_time_s = None;
        self._stationary_seed_probability = 0.;
        self._stationary_seed_score = 0.;
        self._stationary_path_outlier_since_s = None;
        self._stationary_front_departure_since_s = None;
        self._stationary_corner_supported = false;
        self._stationary_weak_pair_identity = None;
        self._stationary_weak_pair_last_vision_time_s = None;
        self._stationary_pending_weak_pair_supported = false;
        self._close_xyv_previous = None;
        self._close_xyv_since_s = None;
        self.reset_stationary_challenger();
    }
    pub fn reset_stationary_challenger(&mut self) {
        self._stationary_closer_challenger_identity = None;
        self._stationary_closer_challenger_since_s = None;
        self._stationary_closer_challenger_last_point = None;
        self._stationary_closer_challenger_last_time_s = None;
    }
    pub fn reset_radar_only_moving(&mut self) {
        self.radar_only_moving_identity = None;
        self._radar_only_moving_pending_identity = None;
        self._radar_only_moving_pending_since_s = None;
        self._radar_only_moving_pending_start_d_rel = None;
        self._radar_only_moving_integrated_v_rel_m = 0.;
        self._radar_only_moving_last_point = None;
        self._radar_only_moving_last_time_s = None;
        self.reset_moving_challenger();
    }
    pub fn reset_moving_challenger(&mut self) {
        self._radar_only_moving_challenger_identity = None;
        self._radar_only_moving_challenger_since_s = None;
        self._radar_only_moving_challenger_last_point = None;
        self._radar_only_moving_challenger_last_time_s = None;
    }
    pub fn reset_far_vision(&mut self) {
        self._far_vision_radar_identity = None;
        self._far_vision_radar_since_s = None;
        self._far_vision_radar_last_point = None;
        self._far_vision_radar_last_time_s = None;
    }
    pub fn reset_rejected_moving(&mut self) {
        self._rejected_radar_only_moving_identity = None;
        self._rejected_radar_only_moving_last_point = None;
        self._rejected_radar_only_moving_last_time_s = None;
    }
    pub fn moving_rejected(&mut self, point: &Point, now: f64) -> bool {
        if self._rejected_radar_only_moving_identity.as_ref() != Some(&point.identity()) {
            return false;
        }
        let continuous = self
            ._rejected_radar_only_moving_last_point
            .as_ref()
            .zip(self._rejected_radar_only_moving_last_time_s)
            .is_some_and(|(previous, time)| position_continuous(previous, time, point, now));
        if !continuous {
            self.reset_rejected_moving();
            return false;
        }
        self._rejected_radar_only_moving_last_point = Some(point.clone());
        self._rejected_radar_only_moving_last_time_s = Some(now);
        true
    }
    pub fn reject_moving(&mut self, point: &Point, now: f64) {
        self._rejected_radar_only_moving_identity = Some(point.identity());
        self._rejected_radar_only_moving_last_point = Some(point.clone());
        self._rejected_radar_only_moving_last_time_s = Some(now);
    }
    pub fn update_vision_fallback(&mut self, vision: Option<VisionLead>) {
        if vision.is_some_and(|vision| vision.probability >= VISION_LEAD_MIN_PROB) {
            self._vision_fallback = vision;
            self._vision_fallback_hold_frames = 0;
        } else if vision.is_some_and(|vision| vision.probability > VISION_LEAD_HOLD_MIN_PROB)
            && self._vision_fallback.is_some()
            && self._vision_fallback_hold_frames < VISION_LEAD_HOLD_MAX_FRAMES
        {
            self._vision_fallback = vision;
            self._vision_fallback_hold_frames += 1;
        } else {
            self._vision_fallback = None;
            self._vision_fallback_hold_frames = 0;
        }
    }
    pub fn update_moving_history(&mut self, point: &Point, now: f64) {
        let continuing = self._radar_only_moving_pending_identity.as_ref()
            == Some(&point.identity())
            && self._radar_only_moving_last_point.is_some()
            && self._radar_only_moving_last_time_s.is_some();
        if !continuing {
            self._radar_only_moving_pending_identity = Some(point.identity());
            self._radar_only_moving_pending_since_s = Some(now);
            self._radar_only_moving_pending_start_d_rel = Some(point.d_rel);
            self._radar_only_moving_integrated_v_rel_m = 0.;
        } else if let Some((previous, time)) = self
            ._radar_only_moving_last_point
            .as_ref()
            .zip(self._radar_only_moving_last_time_s)
        {
            let dt = now - time;
            if dt > 0. {
                self._radar_only_moving_integrated_v_rel_m +=
                    0.5 * (previous.v_rel + point.v_rel) * dt;
            }
        }
        self._radar_only_moving_last_point = Some(point.clone());
        self._radar_only_moving_last_time_s = Some(now);
    }
    pub fn moving_longitudinal_consistency(&self, point: &Point, now: f64) -> bool {
        if !point.corner() {
            return true;
        }
        let Some((since, start)) = self
            ._radar_only_moving_pending_since_s
            .zip(self._radar_only_moving_pending_start_d_rel)
        else {
            return false;
        };
        let duration = now - since;
        if duration < RADAR_ONLY_MOVING_CONFIRMATION_S {
            return true;
        }
        let observed = point.d_rel - start;
        (observed - self._radar_only_moving_integrated_v_rel_m).abs() / maximum(duration, 1e-3)
            <= RADAR_ONLY_MOVING_CORNER_MAX_LONGITUDINAL_ERROR_RATE_MPS
    }
}
