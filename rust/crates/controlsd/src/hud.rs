use crate::{
    controller::{Command, Controls},
    inputs::Inputs,
    parameters::Parameters,
    Error,
};
use openpilot_cereal::car_capnp::car_control;
use openpilot_control_policy::math::maximum;
pub(crate) fn write(
    controls: &mut Controls,
    input: &Inputs,
    command: &Command,
    params: &mut impl Parameters,
    mut hud: car_control::h_u_d_control::Builder<'_>,
    desired: f64,
    speed: f64,
) -> Result<(), Error> {
    hud.set_active_carrot(narrow(input.carrot.active)?);
    let now = {
        let t = rustix::time::clock_gettime(rustix::time::ClockId::Monotonic);
        t.tv_sec as f64 + t.tv_nsec as f64 * 1e-9
    };
    hud.set_physical_blinker_hold(
        controls
            .physical_hold
            .read(input.car.can_valid, now, params)?,
    );
    hud.set_atc_distance(input.carrot.turn_distance as f32);
    if controls.config.meb() {
        navigation(controls, input, params, hud.reborrow())?;
    }
    let speed = if controls.config.pcm_cruise {
        match params.integer("SpeedFromPCM")? {
            1 => input.car.cluster * (1. / 3.6),
            2 => maximum(30. / 3.6, desired * (1. / 3.6)),
            3 => {
                if input.longitudinal.x_state == 3 {
                    speed
                } else {
                    desired * (1. / 3.6)
                }
            }
            _ => maximum(30. / 3.6, speed),
        }
    } else if input.longitudinal.x_state == 3 {
        speed
    } else {
        desired * (1. / 3.6)
    };
    hud.set_set_speed(speed as f32);
    hud.set_speed_visible(command.enabled);
    hud.set_lanes_visible(command.enabled);
    hud.set_lead_visible(input.longitudinal.has_lead);
    hud.set_lead_distance_bars(
        (u32::from(input.selfdrive.personality) + 1)
            .try_into()
            .map_err(|_| Error::Contract("HUD distance bars out of range"))?,
    );
    hud.set_visual_alert(input.selfdrive.visual_alert.try_into()?);
    hud.set_driver_monitoring_alert(input.driver_monitoring_alert);
    // Read only by the Hyundai CAN-FD CCNC cluster (CcncClusterDisplay /
    // CcncTrafficBackground): traffic-light background, lane highlight,
    // target ring at the kept gap, and stock texts for openpilot alerts.
    hud.set_traffic_state(i16::try_from(input.longitudinal.traffic_state).unwrap_or_default());
    hud.set_lat_hud_state(lat_hud_state(controls, command));
    hud.set_desired_distance(input.longitudinal.desired_distance as f32);
    hud.set_alert_type(input.selfdrive.alert_type.as_str());
    let lead = &input.radar.lead;
    hud.set_lead_distance(if lead.status {
        lead.distance as f32
    } else {
        0.
    });
    hud.set_lead_rel_speed(if lead.status {
        lead.relative_speed as f32
    } else {
        0.
    });
    hud.set_lead_radar(i16::from(lead.radar));
    hud.set_lead_d_path(lead.path as f32);
    // A low-speed signal-stop CAN object, never a radar/planner observation.
    const STOP_OBJECT_SCOPE: u32 = (1 << 13) | (1 << 3); // HyundaiFlags CANFD | CAMERA_SCC
    hud.set_signal_stop_object(
        input.stop_object_inputs
            && controls.config.brand == "hyundai"
            && controls.config.flags & STOP_OBJECT_SCOPE == STOP_OBJECT_SCOPE
            && controls.config.openpilot_long
            && command.enabled
            && command.longitudinal
            && input.car.can_valid
            && !input.car.can_timeout
            && input.car.drive
            && !(input.car.gas
                || input.car.brake
                || input.car.parking_brake
                || input.car.brake_hold)
            && input.longitudinal.stop
            && matches!(input.longitudinal.x_state, 3 | 5)
            && !input.longitudinal.has_lead
            && !lead.status
            && !input.radar.lead_two,
    );
    let desire = if input.model.desire.len() > 4 {
        (1..5).find(|i| input.model.desire[*i] > 0.1).unwrap_or(0)
    } else {
        0
    };
    hud.set_model_desire(desire as i16);
    hud.set_right_lane_visible(true);
    hud.set_left_lane_visible(true);
    if input.assistance_valid {
        hud.set_left_lane_depart(input.left_depart);
        hud.set_right_lane_depart(input.right_depart);
    }
    Ok(())
}
/// carrot controlsd `lat_hud_state`: 0 lat off, 1 laneless, 2 lane mode,
/// 3 angle error over the 3 deg saturation threshold for 0.3 s, 4 saturated.
fn lat_hud_state(controls: &mut Controls, command: &Command) -> u8 {
    let log = &command.log;
    // Only the angle and PID states carry steering angles.
    let error = if controls.lateral.torque.is_some() && !controls.config.angle {
        0.
    } else {
        (log.desired - log.angle).abs()
    };
    controls.lat_err_frames = if command.lateral && error > 3. {
        controls.lat_err_frames.saturating_add(1)
    } else {
        0
    };
    if !command.lateral {
        0
    } else if log.saturated {
        4
    } else if controls.lat_err_frames >= 30 {
        3
    } else if controls.lane_lines {
        2
    } else {
        1
    }
}

fn navigation(
    controls: &mut Controls,
    input: &Inputs,
    params: &mut impl Parameters,
    mut hud: car_control::h_u_d_control::Builder<'_>,
) -> Result<(), Error> {
    let carrot = &input.carrot;
    hud.set_navi_speed_limit(
        if carrot.speed_type >= 0 && carrot.speed_type != 22 && carrot.speed_limit > 0. {
            narrow(integer(carrot.speed_limit)?)?
        } else {
            0
        },
    );
    if input.frame % 100 == 0 {
        controls.turn_speed = params.integer("AutoTurnControlSpeedTurn")?;
    }
    let turn = carrot.turn_speed.abs();
    let (kind, speed) = if turn > 0. && turn < 120. && turn < input.car.speed * 3.6 - 3. {
        (1, integer(carrot.turn_speed)?)
    } else if matches!(
        carrot.turn_type.as_str(),
        "turn left" | "turn right" | "atc left" | "atc right"
    ) {
        (2, controls.turn_speed)
    } else if matches!(carrot.turn_type.as_str(), "fork left" | "fork right")
        && carrot.road_limit > 0.
    {
        (3, integer(carrot.road_limit)?)
    } else if carrot.turn_info == 5 && carrot.turn_distance > 0. && carrot.turn_distance < 500. {
        (4, controls.turn_speed)
    } else if matches!(
        carrot.sdi.as_str(),
        "병목지점" | "Bottleneck point" | "瓶颈路段"
    ) {
        (5, 0)
    } else if carrot.desired_source == "road" && carrot.desired > 0. && carrot.desired < 200. {
        (8, integer(carrot.road_limit)?)
    } else {
        (0, 0)
    };
    hud.set_lead_limiting(false);
    hud.set_navi_event_type(kind);
    hud.set_navi_event_speed(narrow(speed)?);
    Ok(())
}
fn integer(value: f64) -> Result<i32, Error> {
    if !value.is_finite()
        || value.trunc() < f64::from(i32::MIN)
        || value.trunc() > f64::from(i32::MAX)
    {
        return Err(Error::Contract("navigation integer out of range"));
    }
    Ok(value as i32)
}

fn narrow(value: i32) -> Result<i16, Error> {
    value
        .try_into()
        .map_err(|_| Error::Contract("HUD integer out of range"))
}
