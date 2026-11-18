use crate::{config::text, inputs::*, Error};
use openpilot_cereal::{
    car_capnp::car_state::GearShifter,
    log_capnp::{self, event},
};
use openpilot_messaging::state::State;

macro_rules! service {
    ($state:expr, $name:literal, $variant:ident) => {
        match $state.topic($name)?.event()?.which()? {
            event::$variant(value) => value?,
            _ => return Err(Error::Contract("unexpected subscribed service")),
        }
    };
}
fn floats(values: capnp::primitive_list::Reader<'_, f32>) -> Vec<f64> {
    values.iter().map(f64::from).collect()
}
fn ordinal<T: Into<u16>>(value: Result<T, capnp::NotInSchema>) -> u16 {
    match value {
        Ok(v) => v.into(),
        Err(capnp::NotInSchema(v)) => v,
    }
}

pub fn decode_model(state: &State, centering: bool) -> Result<Model, Error> {
    let model = service!(state, "modelV2", ModelV2);
    let meta = model.get_meta()?;
    let jetlink = model.get_jetlink()?;
    let mut value = Model {
        desired_curvature: f64::from(model.get_action()?.get_desired_curvature()),
        jetlink_latched: jetlink.get_loss_latched(),
        jetlink: jetlink.get_source() == Ok(log_capnp::jetlink_frame_status::Source::Jetlink),
        lane_change: meta.get_lane_change_state() != Ok(log_capnp::LaneChangeState::Off),
        lane_left: meta.get_lane_change_direction() == Ok(log_capnp::LaneChangeDirection::Left),
        lane_right: meta.get_lane_change_direction() == Ok(log_capnp::LaneChangeDirection::Right),
        assistant_id: meta
            .get_assistant_lane_signal_id()
            .ok()
            .and_then(|t| t.to_str().ok())
            .unwrap_or_default()
            .to_owned(),
        assistant_route: meta
            .get_assistant_lane_signal_route()
            .ok()
            .and_then(|t| t.to_str().ok())
            .unwrap_or_default()
            .to_owned(),
        desire: floats(meta.get_desire_state()?),
        orientation_x: floats(model.get_orientation()?.get_x()?),
        orientation_y: floats(model.get_orientation()?.get_y()?),
        acceleration_y: floats(model.get_acceleration()?.get_y()?),
        ..Model::default()
    };
    if centering {
        let lines = model.get_lane_lines()?;
        if lines.len() > 1 {
            let left = lines.get(1);
            value.lane_x = floats(left.get_x()?);
            value.left_y = floats(left.get_y()?);
        }
        if lines.len() > 2 {
            value.right_y = floats(lines.get(2).get_y()?);
        }
        let probs = model.get_lane_line_probs()?;
        if probs.len() > 1 {
            value.left_prob = f64::from(probs.get(1));
        }
        if probs.len() > 2 {
            value.right_prob = f64::from(probs.get(2));
        }
        let position = model.get_position()?;
        value.path_x = floats(position.get_x()?);
        value.path_y = floats(position.get_y()?);
    }
    Ok(value)
}

pub fn decode(state: &State, now: f64) -> Result<Inputs, Error> {
    decode_with_model(state, now, decode_model(state, true)?)
}

pub fn decode_with_model(state: &State, now: f64, model: Model) -> Result<Inputs, Error> {
    let car = service!(state, "carState", CarState);
    let cruise = car.get_cruise_state()?;
    let car = CarState {
        speed: f64::from(car.get_v_ego()),
        accel: f64::from(car.get_a_ego()),
        steer_angle: f64::from(car.get_steering_angle_deg()),
        steer_rate: f64::from(car.get_steering_rate_deg()),
        steer_torque: f64::from(car.get_steering_torque()),
        steering_pressed: car.get_steering_pressed(),
        can_valid: car.get_can_valid(),
        can_timeout: car.get_can_timeout(),
        drive: matches!(car.get_gear_shifter(), Ok(GearShifter::Drive)),
        parking_brake: car.get_parking_brake(),
        brake_hold: car.get_brake_hold_active(),
        driving_gear: !matches!(
            car.get_gear_shifter(),
            Ok(GearShifter::Neutral
                | GearShifter::Park
                | GearShifter::Reverse
                | GearShifter::Unknown)
        ),
        lat_enabled: car.get_lat_enabled(),
        fault_temporary: car.get_steer_fault_temporary(),
        fault_permanent: car.get_steer_fault_permanent(),
        standstill: car.get_standstill(),
        brake: car.get_brake_pressed(),
        gas: car.get_gas_pressed(),
        soft_hold: car.get_soft_hold_active() > 0,
        carrot_cruise: car.get_carrot_cruise() != 0,
        cruise_enabled: cruise.get_enabled(),
        cruise_standstill: cruise.get_standstill(),
        cruise: f64::from(car.get_v_cruise()),
        cluster: f64::from(car.get_v_cruise_cluster()),
        cluster_ratio: f64::from(car.get_v_clu_ratio()),
        steer_curvature: f64::from(car.get_steering_curvature()),
    };
    let live = service!(state, "liveParameters", LiveParameters);
    let center_to_front_ratio = f64::from(live.get_center_to_front_ratio());
    let live = LiveParameters {
        stiffness: f64::from(live.get_stiffness_factor()),
        ratio: f64::from(live.get_steer_ratio()),
        offset: f64::from(live.get_angle_offset_deg()),
        roll: f64::from(live.get_roll()),
        valid: live.get_valid(),
        sensor_valid: live.get_sensor_valid(),
        posenet_valid: live.get_posenet_valid(),
    };
    let torque = service!(state, "liveTorqueParameters", LiveTorqueParameters);
    let torque = TorqueParameters {
        use_params: torque.get_use_params(),
        factor: torque.get_lat_accel_factor_filtered(),
        offset: torque.get_lat_accel_offset_filtered(),
        friction: torque.get_friction_coefficient_filtered(),
    };
    let driving = service!(state, "selfdriveState", SelfdriveState);
    let topic = state.topic("selfdriveState")?;
    // Use the service's ordinary ten-period liveness window, including during replay.
    let current = topic.seen
        && topic.valid
        && (0. ..10. / topic.service.frequency).contains(&(now - topic.receive_time));
    let selfdrive = Selfdrive {
        current,
        enabled: current && driving.get_enabled(),
        active: current && driving.get_active(),
        soft_disabling: driving.get_state()
            == Ok(log_capnp::selfdrive_state::OpenpilotState::SoftDisabling),
        personality: ordinal(driving.get_personality()),
        visual_alert: ordinal(driving.get_alert_hud_visual()),
        alert_type: driving
            .get_alert_type()
            .ok()
            .and_then(|text| text.to_str().ok())
            .unwrap_or_default()
            .to_owned(),
    };
    let plan = service!(state, "longitudinalPlan", LongitudinalPlan);
    let longitudinal = LongPlan {
        acceleration: f64::from(plan.get_a_target()),
        speed: f64::from(plan.get_v_target_now()),
        jerk: f64::from(plan.get_j_target_now()),
        stop: plan.get_should_stop(),
        speeds: floats(plan.get_speeds()?),
        has_lead: plan.get_has_lead(),
        x_state: plan.get_x_state(),
        coast_target: f64::from(plan.get_cruise_coasting_target()),
        coast_percent: plan.get_cruise_coasting_percent(),
        cruise_source: plan.get_longitudinal_plan_source()
            == Ok(log_capnp::longitudinal_plan::LongitudinalPlanSource::Cruise),
        fcw: plan.get_fcw(),
        cruise_target: f64::from(plan.get_cruise_target()),
        traffic_state: plan.get_traffic_state(),
        desired_distance: f64::from(plan.get_desired_distance()),
    };
    let lateral = service!(state, "lateralPlan", LateralPlan);
    let lateral = LateralPlan {
        lane_lines: lateral.get_use_lane_lines(),
        solution_valid: lateral.get_mpc_solution_valid(),
        psis: floats(lateral.get_psis()?),
        curvatures: floats(lateral.get_curvatures()?),
        distances: floats(lateral.get_distances()?),
    };
    let radar = service!(state, "radarState", RadarState);
    let lead = radar.get_lead_one()?;
    let radar = Radar {
        lead: Lead {
            status: lead.get_status(),
            distance: f64::from(lead.get_d_rel()),
            relative_speed: f64::from(lead.get_v_rel()),
            radar: lead.get_radar(),
            path: f64::from(lead.get_d_path()),
        },
        lead_two: radar.get_lead_two()?.get_status(),
        cut_in: radar.get_lead_cut_in_risk()?.get_status(),
    };
    let output = service!(state, "carOutput", CarOutput).get_actuators_output()?;
    let carrot = service!(state, "carrotMan", CarrotMan);
    let carrot = Carrot {
        desired: f64::from(carrot.get_desired_speed()),
        turn_speed: f64::from(carrot.get_v_turn_speed()),
        active: carrot.get_active_carrot(),
        turn_distance: f64::from(carrot.get_x_dist_to_turn()),
        speed_type: carrot.get_x_spd_type(),
        speed_limit: f64::from(carrot.get_x_spd_limit()),
        turn_type: text(carrot.get_atc_type()?)?,
        road_limit: f64::from(carrot.get_n_road_limit_speed()),
        turn_info: carrot.get_x_turn_info(),
        sdi: text(carrot.get_sz_sdi_descr()?)?,
        desired_source: text(carrot.get_desired_source()?)?,
    };
    let c = state.topic("carrotMan")?;
    let age = now - c.receive_time;
    let carrot_fresh = c.seen
        && c.alive
        && c.valid
        && (0. ..=1.).contains(&age)
        && carrot.desired > 0.
        && carrot.desired <= 250.;
    let mut input = Inputs {
        startup: crate::readiness::decode(state)?,
        car,
        live,
        torque,
        selfdrive,
        model,
        longitudinal,
        lateral,
        radar,
        carrot,
        output_torque: f64::from(output.get_torque()),
        output_angle: f64::from(output.get_steering_angle_deg()),
        output_curvature: f64::from(output.get_curvature()),
        delay: f64::from(service!(state, "liveDelay", LiveDelay).get_lateral_delay()),
        override_long: service!(state, "onroadEvents", OnroadEvents)
            .iter()
            .any(|event| event.get_override_longitudinal()),
        vehicle_fault: service!(state, "onroadEvents", OnroadEvents)
            .iter()
            .any(|event| {
                matches!(
                    event.get_name(),
                    Ok(log_capnp::onroad_event::EventName::PandaCanFault)
                )
            }),
        monitoring_disabled: service!(state, "driverMonitoringState", DriverMonitoringState)
            .get_dm2_disabled(),
        distracted: service!(state, "driverMonitoringState", DriverMonitoringState)
            .get_alert_level()
            == Ok(log_capnp::driver_monitoring_state::AlertLevel::Three),
        assistance_valid: state.topic("driverAssistance")?.valid,
        torque_checks: state.all_checks(&["liveTorqueParameters"])?,
        model_checks: state.all_checks(&["modelV2"])?,
        carrot_fresh,
        // Only export a current, enabled DM warning (including AlwaysOnDM).
        driver_monitoring_alert: {
            let dm = service!(state, "driverMonitoringState", DriverMonitoringState);
            if !state.all_checks(&["driverMonitoringState"])? || dm.get_dm2_disabled() {
                0
            } else {
                match dm.get_alert_level() {
                    Ok(log_capnp::driver_monitoring_state::AlertLevel::One) => 1,
                    Ok(log_capnp::driver_monitoring_state::AlertLevel::Two) => 2,
                    Ok(log_capnp::driver_monitoring_state::AlertLevel::Three) => 3,
                    _ => 0,
                }
            }
        },
        center_to_front_ratio,
        stop_object_inputs: state.all_checks(&["longitudinalPlan", "radarState"])?
            && (0. ..=0.2).contains(
                &((state.frame() - state.topic("longitudinalPlan")?.receive_frame) as f64 * 0.01),
            )
            && (0. ..=0.2).contains(
                &((state.frame() - state.topic("radarState")?.receive_frame) as f64 * 0.01),
            ),
        frame: state.frame(),
        plan_age: (state.frame() - state.topic("longitudinalPlan")?.receive_frame) as f64 * 0.01,
        long_time: state.topic("longitudinalPlan")?.log_mono_time,
        model_time: state.topic("modelV2")?.log_mono_time,
        ..Inputs::default()
    };
    if input.assistance_valid {
        let assistance = service!(state, "driverAssistance", DriverAssistance);
        input.left_depart = assistance.get_left_lane_departure();
        input.right_depart = assistance.get_right_lane_departure();
    }
    if state.topic("liveCalibration")?.updated {
        let values = service!(state, "liveCalibration", LiveCalibration).get_rpy_calib()?;
        if values.len() != 3 {
            return Err(Error::Contract("calibration dimensions"));
        }
        input.calibration = Some([values.get(0), values.get(1), values.get(2)].map(f64::from));
    }
    if state.topic("livePose")?.updated {
        let pose = service!(state, "livePose", LivePose);
        let orientation = pose.get_orientation_n_e_d()?;
        let angular = pose.get_angular_velocity_device()?;
        input.pose = Some(Pose {
            orientation: [
                orientation.get_x(),
                orientation.get_y(),
                orientation.get_z(),
            ]
            .map(f64::from),
            angular: [angular.get_x(), angular.get_y(), angular.get_z()].map(f64::from),
        });
    }
    Ok(input)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn engagement_expires_between_publications_and_returns_with_a_valid_state() {
        let mut state = State::new(&crate::SERVICES, Default::default()).unwrap();
        let mut message = capnp::message::Builder::new_default();
        let mut event = message.init_root::<event::Builder>();
        event.set_valid(true);
        let mut driving = event.init_selfdrive_state();
        driving.set_active(true);
        driving.set_enabled(true);
        let bytes = capnp::serialize::write_message_to_words(&message);
        state.update(1., std::slice::from_ref(&bytes)).unwrap();
        assert!(decode(&state, 1.02).unwrap().selfdrive.enabled);
        let expired = decode(&state, 1.11).unwrap().selfdrive;
        assert!(!expired.current && !expired.active && !expired.enabled);
        state.update(1.12, &[bytes]).unwrap();
        assert!(decode(&state, 1.13).unwrap().selfdrive.active);
        message
            .get_root::<event::Builder>()
            .unwrap()
            .set_valid(false);
        state
            .update(1.14, &[capnp::serialize::write_message_to_words(&message)])
            .unwrap();
        assert!(!decode(&state, 1.14).unwrap().selfdrive.enabled);
    }
}
