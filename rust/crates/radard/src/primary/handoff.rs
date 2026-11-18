use super::{
    constants::*, source::position_continuous, stationary_geometry::vision_base_cost, Matcher,
    VisionMatch,
};
use crate::{math::maximum, model::VisionLead};

impl Matcher {
    /// carrot `_close_xyv_handoff_ready` (e5f4ac8a1): in a slow queue, confirm
    /// repeated fresh x/y/v matches to a nearby overlapping front-radar body for
    /// 200 ms before it replaces the held lead. Any miss, track change, frame gap
    /// over 0.1 s or discontinuity restarts the confirmation.
    pub fn close_xyv_handoff_ready(
        &mut self,
        stationary: Option<&VisionMatch>,
        moving: Option<&VisionMatch>,
        vision: Option<VisionLead>,
        time: Option<f64>,
    ) -> bool {
        let matched = stationary.zip(moving).zip(vision).zip(time).filter(
            |(((stationary, moving), vision), now)| {
                let (held, near) = (&stationary.point, &moving.point);
                now.is_finite()
                    && held.source == "frontRadar"
                    && near.source == "frontRadar"
                    && held.measured
                    && near.measured
                    && held.track_id != near.track_id
                    && vision.probability >= STATIONARY_CLOSER_HANDOFF_MIN_VISION_PROB
                    && near.radar_track_state >= STATIONARY_RADAR_ONLY_FRONT_MIN_TRACK_STATE
                    && moving.score >= VISION_MATCH_FRESH_MIN_SCORE
                    && 0.5 < near.d_rel
                    && near.d_rel <= CLOSE_XYV_HANDOFF_MAX_DREL_M
                    && (near.v_lead - near.v_rel).abs() <= CLOSE_XYV_HANDOFF_MAX_EGO_SPEED_MPS
                    && maximum(held.v_lead.abs(), near.v_lead.abs()) <= STATIONARY_MAX_ABS_VLEAD_MPS
                    && STATIONARY_CLOSER_HANDOFF_MIN_DREL_GAIN_M <= held.d_rel - near.d_rel
                    && held.d_rel - near.d_rel <= VISION_RADAR_MAX_DISTANCE_ERROR_M
                    && moving.d_path.abs() < PREDICTOR_PATH_OVERLAP_HALF_WIDTH_M
                    && (near.y_rel - vision.y_rel).abs()
                        <= STATIONARY_CLOSER_HANDOFF_MAX_VISION_YREL_ERROR_M
                    && (near.d_rel - vision.d_rel).abs()
                        + STATIONARY_CLOSER_HANDOFF_MIN_VISION_RANGE_GAIN_M
                        <= (held.d_rel - vision.d_rel).abs()
            },
        );
        let Some((((stationary, moving), _), now)) = matched else {
            self._close_xyv_previous = None;
            self._close_xyv_since_s = None;
            return false;
        };
        let restart = self
            ._close_xyv_previous
            .as_ref()
            .is_none_or(|(held, near, time)| {
                *held != stationary.point.track_id
                    || near.track_id != moving.point.track_id
                    || !(0. < now - time && now - time <= CLOSE_XYV_HANDOFF_MAX_FRAME_GAP_S)
                    || !position_continuous(near, *time, &moving.point, now)
            });
        if restart || self._close_xyv_since_s.is_none() {
            self._close_xyv_since_s = Some(now);
        }
        self._close_xyv_previous = Some((stationary.point.track_id, moving.point.clone(), now));
        self._close_xyv_since_s
            .is_some_and(|since| now - since >= CLOSE_XYV_HANDOFF_CONFIRMATION_S - 1e-6)
    }

    pub fn closer_handoff_ready(
        &mut self,
        stationary: Option<&VisionMatch>,
        moving: Option<&VisionMatch>,
        vision: Option<VisionLead>,
        time: Option<f64>,
    ) -> bool {
        if self.close_xyv_handoff_ready(stationary, moving, vision, time) {
            return true;
        }
        let continuing = moving.zip(time).is_some_and(|(moving, now)| {
            self._stationary_closer_challenger_identity.as_ref() == Some(&moving.point.identity())
                && self._stationary_closer_challenger_since_s.is_some()
                && self
                    ._stationary_closer_challenger_last_point
                    .as_ref()
                    .zip(self._stationary_closer_challenger_last_time_s)
                    .is_some_and(|(previous, time)| {
                        position_continuous(previous, time, &moving.point, now)
                    })
        });
        let mut gain = STATIONARY_CLOSER_HANDOFF_MIN_COST_GAIN;
        if continuing
            && self
                ._stationary_closer_challenger_last_time_s
                .zip(self._stationary_closer_challenger_since_s)
                .is_some_and(|(last, since)| {
                    last - since >= STATIONARY_CLOSER_HANDOFF_COST_HOLD_MIN_S
                })
        {
            gain = STATIONARY_CLOSER_HANDOFF_HOLD_COST_GAIN;
        }
        let held_cost =
            stationary.and_then(|stationary| vision_base_cost(vision, &stationary.point));
        let challenger_cost = moving.and_then(|moving| vision_base_cost(vision, &moving.point));
        let cost_supported = stationary.zip(moving).is_some_and(|(stationary, moving)| {
            (stationary.point.y_rel - moving.point.y_rel).abs()
                <= STATIONARY_CLOSER_HANDOFF_MAX_YREL_DELTA_M
                && held_cost
                    .zip(challenger_cost)
                    .is_some_and(|(held, challenger)| challenger + gain <= held)
        });
        let range_supported =
            stationary
                .zip(moving)
                .zip(vision)
                .is_some_and(|((stationary, moving), vision)| {
                    (stationary.point.y_rel - moving.point.y_rel).abs()
                        <= STATIONARY_CLOSER_HANDOFF_RANGE_MAX_YREL_DELTA_M
                        && moving.d_path.abs() <= STATIONARY_CLOSER_HANDOFF_MAX_DPATH_M
                        && (moving.point.y_rel - vision.y_rel).abs()
                            <= STATIONARY_CLOSER_HANDOFF_MAX_VISION_YREL_ERROR_M
                        && (moving.point.d_rel - vision.d_rel).abs()
                            + STATIONARY_CLOSER_HANDOFF_MIN_VISION_RANGE_GAIN_M
                            <= (stationary.point.d_rel - vision.d_rel).abs()
                });
        let distinct =
            stationary
                .zip(moving)
                .zip(vision)
                .is_some_and(|((stationary, moving), vision)| {
                    cost_supported
                        && range_supported
                        && moving.point.radar_track_state
                            >= STATIONARY_RADAR_ONLY_FRONT_MIN_TRACK_STATE
                        && moving.score >= VISION_MATCH_FRESH_MIN_SCORE
                        && (moving.point.d_rel - vision.d_rel).abs()
                            <= STATIONARY_DISTINCT_HANDOFF_MAX_VISION_ERROR_M
                        && (moving.point.d_rel - vision.d_rel).abs()
                            <= STATIONARY_DISTINCT_HANDOFF_MAX_ERROR_RATIO
                                * (stationary.point.d_rel - vision.d_rel).abs()
                        && stationary.point.d_rel - moving.point.d_rel
                            <= VISION_RADAR_MAX_DISTANCE_ERROR_M
                });
        let eligible = stationary.zip(moving).zip(vision).zip(time).filter(
            |(((stationary, moving), vision), now)| {
                vision.probability >= STATIONARY_CLOSER_HANDOFF_MIN_VISION_PROB
                    && now.is_finite()
                    && stationary.point.identity() != moving.point.identity()
                    && stationary.point.source == "frontRadar"
                    && moving.point.source == stationary.point.source
                    && stationary.point.measured
                    && moving.point.measured
                    && stationary.point.v_lead.abs() <= STATIONARY_MAX_ABS_VLEAD_MPS
                    && moving.point.v_lead.abs() <= STATIONARY_MAX_ABS_VLEAD_MPS
                    && STATIONARY_CLOSER_HANDOFF_MIN_DREL_GAIN_M
                        <= stationary.point.d_rel - moving.point.d_rel
                    && (stationary.point.d_rel - moving.point.d_rel
                        <= STATIONARY_CLOSER_HANDOFF_MAX_DREL_DELTA_M
                        || distinct)
                    && (stationary.point.v_lead - moving.point.v_lead).abs()
                        <= STATIONARY_CLOSER_HANDOFF_MAX_VLEAD_DELTA_MPS
                    && (cost_supported || range_supported)
            },
        );
        let Some((((stationary, moving), _), now)) = eligible else {
            self.reset_stationary_challenger();
            return false;
        };
        if !continuing {
            self._stationary_closer_challenger_identity = Some(moving.point.identity());
            self._stationary_closer_challenger_since_s = Some(now);
        }
        self._stationary_closer_challenger_last_point = Some(moving.point.clone());
        self._stationary_closer_challenger_last_time_s = Some(now);
        let confirmation = if stationary.point.d_rel - moving.point.d_rel
            > STATIONARY_CLOSER_HANDOFF_MAX_DREL_DELTA_M
        {
            STATIONARY_DISTINCT_HANDOFF_CONFIRMATION_S
        } else {
            STATIONARY_CLOSER_HANDOFF_CONFIRMATION_S
        };
        self._stationary_closer_challenger_since_s
            .is_some_and(|since| now - since >= confirmation)
    }
    pub fn adopt_closer_handoff(&mut self, matched: &VisionMatch, vision: VisionLead, now: f64) {
        self.stationary_identity = Some(matched.point.identity());
        self._stationary_pending_identity = None;
        self._stationary_pending_since_s = None;
        self._stationary_pending_vision_support_frames = 0;
        self._stationary_pending_last_support_time_s = None;
        self._stationary_last_point = Some(matched.point.clone());
        self._stationary_last_time_s = Some(now);
        self._stationary_seed_probability = vision.probability;
        self._stationary_corner_supported = false;
        self._stationary_seed_score = vision_base_cost(Some(vision), &matched.point)
            .unwrap_or_else(|| maximum(0., 1. - matched.score));
        self._stationary_path_outlier_since_s = None;
        self._stationary_front_departure_since_s = None;
        self.reset_stationary_challenger();
    }
}

#[cfg(test)]
mod tests {
    use super::super::{Matcher, VisionMatch};
    use crate::{
        model::VisionLead,
        point::{Point, TrackId},
    };

    /// carrot test_close_offset_xyv_requires_continuous_overlap (e5f4ac8a1):
    /// held lead 14 m ahead, a nearer front-radar body 4 m ahead and 1.4 m to
    /// the side at 2 m/s, the vision lead at 6 m, 0.65 m to the same side.
    fn first_ready(side: f64, interruption: &str) -> Option<usize> {
        let point = |track: i128, d_rel: f64, y_rel: f64| Point {
            track_id: TrackId(track),
            source: "frontRadar".into(),
            d_rel,
            y_rel,
            v_lead: 2.,
            measured: true,
            radar_track_state: 2,
            ..Point::default()
        };
        let held = point(48, 14., 0.2 * side);
        let near = point(61, 4., 1.4 * side);
        let stationary = VisionMatch {
            point: held,
            probability: 1.,
            score: 0.01,
            d_path: 0.2 * side,
        };
        let vision = VisionLead {
            probability: 1.,
            d_rel: 6.,
            y_rel: 0.65 * side,
            velocity: 2.,
            x_std: 2.,
            y_std: 0.6,
            v_std: 1.5,
            acceleration: 0.,
        };
        let mut matcher = Matcher::default();
        (0..8).find(|&i| {
            let mut point = near.clone();
            if interruption == "id" && i >= 3 {
                point.track_id = TrackId(62);
            }
            let offset = if interruption == "parallel" { 2.4 } else { 1.4 };
            let moving = VisionMatch {
                point,
                probability: 1.,
                score: 0.01,
                d_path: offset * side,
            };
            let stamp = i as f64 * 0.05
                + if interruption == "gap" && i >= 3 {
                    0.2
                } else {
                    0.
                };
            let observation = (!(interruption == "vision" && i == 2)).then_some(vision);
            matcher.closer_handoff_ready(Some(&stationary), Some(&moving), observation, Some(stamp))
        })
    }

    #[test]
    fn a_close_offset_body_takes_the_lead_only_after_continuous_overlap() {
        for side in [-1., 1.] {
            for (interruption, expected) in [
                ("none", Some(4)),
                ("id", Some(7)),
                ("gap", Some(7)),
                ("vision", Some(7)),
                ("parallel", None),
            ] {
                assert_eq!(
                    first_ready(side, interruption),
                    expected,
                    "{interruption} {side}"
                );
            }
        }
    }
}
