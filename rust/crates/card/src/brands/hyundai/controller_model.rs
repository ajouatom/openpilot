use super::{lead::Lead, Error};
use openpilot_cereal::log_capnp::{model_data_v2, radar_state};

#[derive(Default)]
pub struct Model {
    pub frame_id: Option<u32>,
    pub leads: Vec<Lead>,
    pub x: Vec<f64>,
    pub y: Vec<f64>,
    pub y_std: Option<f64>,
    pub change_available: [bool; 2],
    pub changing: u8,
    pub desire: u16,
    pub lane_pair: Option<[f64; 2]>,
    /// laneLineProbs[1], [2] (CcncClusterDisplay).
    pub lane_probs: Option<[f64; 2]>,
    pub blinker_hold: u8,
}

impl Model {
    pub fn read(
        model: Option<model_data_v2::Reader<'_>>,
        radar: Option<radar_state::Reader<'_>>,
    ) -> Result<Self, Error> {
        let mut ret = Self::default();
        if let Some(model) = model {
            ret.frame_id = Some(model.get_frame_id());
            let position = model.get_position()?;
            ret.x = position.get_x()?.iter().map(f64::from).collect();
            ret.y = position.get_y()?.iter().map(f64::from).collect();
            let std = position.get_y_std()?;
            ret.y_std = if std.len() > 10 {
                Some(f64::from(std.get(10)))
            } else {
                None
            };
            let meta = model.get_meta()?;
            ret.change_available = [
                meta.get_lane_change_available_left(),
                meta.get_lane_change_available_right(),
            ];
            ret.desire = meta.get_desire()?.into();
            ret.blinker_hold = u8::try_from(u16::from(meta.get_lane_change_blinker_hold()?))
                .map_err(|_| Error::Numeric)?;
            let lanes = model.get_lane_lines()?;
            if lanes.len() >= 3 {
                let left = lanes.get(1).get_y()?;
                let right = lanes.get(2).get_y()?;
                if !left.is_empty() && !right.is_empty() {
                    ret.lane_pair = Some([f64::from(left.get(0)), f64::from(right.get(0))]);
                }
            }
            let probs = model.get_lane_line_probs()?;
            if probs.len() >= 3 {
                ret.lane_probs = Some([f64::from(probs.get(1)), f64::from(probs.get(2))]);
            }
            let desire = meta.get_desire_state()?;
            if desire.len() > 4 {
                for index in 1..=4 {
                    if desire.get(index) > 0.9 {
                        ret.changing = u8::try_from(index).map_err(|_| Error::Numeric)?;
                    }
                }
            }
        }
        if let Some(radar) = radar {
            for lead in [radar.get_lead_one()?, radar.get_lead_two()?] {
                ret.leads.push(Lead {
                    status: lead.get_status(),
                    radar: lead.get_radar(),
                    radar_track_id: i64::from(lead.get_radar_track_id()),
                    d_rel: f64::from(lead.get_d_rel()),
                    y_rel: f64::from(lead.get_y_rel()),
                    v_rel: f64::from(lead.get_v_rel()),
                });
            }
        }
        Ok(ret)
    }
}
