use crate::{polyfit, prediction::DrivingPrediction, wire_helpers::float};
use openpilot_cereal::log_capnp::event;
use serde::{Deserialize, Serialize};

pub fn driving(
    model_event: event::Reader<'_>,
    log_mono_time: u64,
) -> capnp::Result<capnp::message::Builder<capnp::message::HeapAllocator>> {
    let source = match model_event.which()? {
        event::ModelV2(model) => model?,
        _ => return Err(capnp::Error::failed("expected modelV2 event".to_owned())),
    };
    let mut message = capnp::message::Builder::new_default();
    let mut event = message.init_root::<event::Builder>();
    event.set_valid(model_event.get_valid());
    event.set_log_mono_time(log_mono_time);
    let mut target = event.init_driving_model_data();
    target.set_frame_id(source.get_frame_id());
    target.set_frame_id_extra(source.get_frame_id_extra());
    target.set_frame_drop_perc(source.get_frame_drop_perc());
    target.set_model_execution_time(source.get_model_execution_time());
    target.set_action(source.get_action()?)?;
    let mut meta = target.reborrow().init_meta();
    meta.set_lane_change_state(source.get_meta()?.get_lane_change_state()?);
    meta.set_lane_change_direction(source.get_meta()?.get_lane_change_direction()?);
    let lanes = source.get_lane_lines()?;
    let probs = source.get_lane_line_probs()?;
    if lanes.len() < 3 || probs.len() < 3 {
        return Err(capnp::Error::failed(
            "model has fewer than three lane lines".to_owned(),
        ));
    }
    let left = lanes.get(1).get_y()?;
    let right = lanes.get(2).get_y()?;
    if left.is_empty() || right.is_empty() {
        return Err(capnp::Error::failed("model has empty lane line".to_owned()));
    }
    let mut lane_meta = target.reborrow().init_lane_line_meta();
    lane_meta.set_left_y(left.get(0));
    lane_meta.set_right_y(right.get(0));
    lane_meta.set_left_prob(probs.get(1));
    lane_meta.set_right_prob(probs.get(2));
    let position = source.get_position()?;
    let xyz = [position.get_x()?, position.get_y()?, position.get_z()?];
    if xyz.iter().any(|axis| axis.len() != 33) {
        return Err(capnp::Error::failed(
            "model position must have 33 points".to_owned(),
        ));
    }
    let mut plan = [[0.0; 15]; 33];
    for (axis, values) in xyz.iter().enumerate() {
        for (row, value) in plan.iter_mut().zip(values.iter()) {
            row[axis] = value;
        }
    }
    let coefficients = polyfit::path_coefficients(&plan).map(|axis| axis.map(float));
    let mut path = target.init_path();
    path.set_x_coefficients(&coefficients[0][..])?;
    path.set_y_coefficients(&coefficients[1][..])?;
    path.set_z_coefficients(&coefficients[2][..])?;
    Ok(message)
}

#[derive(Debug, Clone, Copy, Deserialize, Serialize)]
pub struct PoseTiming {
    pub log_mono_time: u64,
    pub frame_id: u32,
    pub dropped_frames: u32,
    pub timestamp_eof: u64,
    pub live_calibration_seen: bool,
}

pub fn pose(
    data: &DrivingPrediction,
    timing: PoseTiming,
) -> capnp::Result<capnp::message::Builder<capnp::message::HeapAllocator>> {
    let mut message = capnp::message::Builder::new_default();
    let mut event = message.init_root::<event::Builder>();
    event.set_log_mono_time(timing.log_mono_time);
    event.set_valid(timing.live_calibration_seen && timing.dropped_frames < 1);
    let mut pose = event.init_camera_odometry();
    pose.set_frame_id(timing.frame_id);
    pose.set_timestamp_eof(timing.timestamp_eof);
    pose.set_trans(&data.pose.mean[..3])?;
    pose.set_rot(&data.pose.mean[3..])?;
    pose.set_trans_std(&data.pose.std[..3])?;
    pose.set_rot_std(&data.pose.std[3..])?;
    pose.set_wide_from_device_euler(&data.wide_euler.mean[..])?;
    pose.set_wide_from_device_euler_std(&data.wide_euler.std[..])?;
    pose.set_road_transform_trans(&data.road_transform.mean[..3])?;
    pose.set_road_transform_trans_std(&data.road_transform.std[..3])?;
    Ok(message)
}
