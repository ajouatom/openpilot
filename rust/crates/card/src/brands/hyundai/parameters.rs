use super::{
    bus::Fingerprints,
    detection::{detect, DetectionInput},
    flags, Error,
};
use crate::{
    firmware::Firmware,
    vehicle_params::{self, FinishOptions, TorqueOptions},
};
use capnp::message::{Builder, HeapAllocator};
use num_traits::ToPrimitive;
use openpilot_cereal::car_capnp::car_params::{self, SafetyModel, SteerControlType};
use openpilot_params::Params;

#[derive(Clone, Copy)]
pub struct ParamsInput<'a> {
    pub candidate: &'a str,
    pub fingerprints: &'a Fingerprints,
    pub firmware: &'a [Firmware],
    pub alpha_long: bool,
    pub is_release: bool,
    pub settings: &'a Params,
}

pub fn setting_int(settings: &Params, key: &str) -> Result<i32, Error> {
    match settings.get(key)? {
        Some(bytes) if !bytes.is_empty() => {
            let start = bytes
                .iter()
                .position(|byte| !byte.is_ascii_whitespace())
                .unwrap_or(bytes.len());
            let raw = bytes.get(start..).ok_or(Error::Numeric)?;
            let sign = usize::from(raw.first().is_some_and(|byte| matches!(byte, b'+' | b'-')));
            let digits = raw
                .iter()
                .skip(sign)
                .take_while(|byte| byte.is_ascii_digit())
                .count();
            // Not a number (or out of range) reads as 0 like a missing value;
            // see settings_float::read for why this must not fail card.
            Ok(
                std::str::from_utf8(raw.get(..sign + digits).unwrap_or_default())
                    .ok()
                    .and_then(|text| text.parse().ok())
                    .unwrap_or(0),
            )
        }
        Some(_) | None => Ok(0),
    }
}

pub fn parameters(input: ParamsInput<'_>) -> Result<Builder<HeapAllocator>, Error> {
    let mut message = vehicle_params::baseline(input.candidate)?;
    let platform = vehicle_params::platform(input.candidate)?;
    let mut cp = message.get_root::<car_params::Builder<'_>>()?;
    let mut detection = detect(DetectionInput {
        candidate: input.candidate,
        fingerprints: input.fingerprints,
        flags: cp.reborrow_as_reader().get_flags(),
        has_radar_dbc: platform.dbc_radar.is_some(),
        camera_scc: setting_int(input.settings, "HyundaiCameraSCC")?,
        hda2: setting_int(input.settings, "CanfdHDA2")?,
        radar_tracks: setting_int(input.settings, "EnableRadarTracks")?,
        alpha_long: input.alpha_long,
    });
    if detection.flags & flags::CANFD != 0
        && detection.flags & flags::CAMERA_SCC != 0
        && input.settings.get_bool("HyundaiCanfdClusterDirectTx")?
    {
        detection.flags |= flags::CLUSTER_DIRECT_TX;
        detection.safety_param |= flags::safety::CLUSTER_DIRECT_TX;
    }
    // carrot interface.py: opt-in SPAS indicators need both exact ECAN SPAS
    // messages and are exclusive with the HDA2 blinker path.
    if detection.flags & flags::CANFD != 0
        && input.settings.get_bool("CanfdBlinkerControl")?
        && {
            let size = |address| super::bus::size(input.fingerprints, detection.bus.ecan, address);
            super::spas::compatible_with(size) || super::spas::absent_with(size)
        }
        && detection.flags & flags::ENABLE_BLINKERS == 0
    {
        detection.flags |= flags::CANFD_SPAS_BLINKERS;
        detection.safety_param |= flags::safety::CANFD_SPAS_BLINKERS;
    }
    cp.set_brand("hyundai");
    cp.set_flags(detection.flags);
    cp.set_ext_flags(detection.ext_flags);
    cp.set_alpha_longitudinal_available(true);
    cp.set_enable_bsm(detection.bsm);
    let extra_safety = detection.flags & flags::CANFD != 0 && detection.bus.ecan >= 4;
    let mut configs = cp
        .reborrow()
        .init_safety_configs(if extra_safety { 2 } else { 1 });
    if extra_safety {
        configs
            .reborrow()
            .get(0)
            .set_safety_model(SafetyModel::NoOutput);
    }
    let mut safety = configs.get(u32::from(extra_safety));
    safety.set_safety_model(detection.safety_model);
    safety.set_safety_param(detection.safety_param);
    let center = (f64::from(cp.reborrow_as_reader().get_wheelbase()) * 0.4)
        .to_f32()
        .ok_or(Error::Numeric)?;
    cp.set_center_to_front(center);
    cp.set_steer_actuator_delay(if input.candidate == "KIA_OPTIMA_G4_FL" {
        0.2
    } else {
        0.1
    });
    cp.set_steer_limit_timer(0.4);
    if detection.flags & flags::ANGLE_CONTROL != 0 {
        cp.set_steer_control_type(SteerControlType::Angle);
    } else {
        vehicle_params::configure_torque(
            input.candidate,
            cp.reborrow().get_lateral_tuning(),
            TorqueOptions {
                deadzone_deg: 0.,
                use_steering_angle: true,
            },
        )?;
    }
    cp.set_radar_unavailable(detection.radar_unavailable);
    cp.set_openpilot_longitudinal_control(detection.longitudinal);
    cp.set_radar_time_step(0.05);
    cp.set_pcm_cruise(!detection.longitudinal);
    cp.set_starting_state(false);
    cp.set_v_ego_starting(0.1);
    cp.set_start_accel(1.);
    cp.set_longitudinal_actuator_delay(0.5);
    let mut tuning = cp.reborrow().get_longitudinal_tuning()?;
    tuning.reborrow().init_kp_b_p(1).set(0, 0.);
    tuning.reborrow().init_kp_v(1).set(0, 1.);
    tuning.set_kf(1.);
    // carrot interface.py: only an explicitly enabled, ACC-less classic-CAN
    // adapter with the 0x201 sensor uses the Comma Pedal protocol.
    if let Some(profile) = super::pedal::profile(input.candidate) {
        let fp = input.fingerprints;
        let has = |bus: u8, address: u32| super::bus::size(fp, bus, address).is_some();
        if detection.flags & flags::CC_ONLY_CAR != 0
            && input.settings.get_bool("CommaPedalEnabled")?
            && detection.flags & (flags::CANFD | flags::LEGACY | flags::CAMERA_SCC) == 0
            && super::bus::size(fp, super::pedal::BUS, super::pedal::SENSOR_ADDR) == Some(6)
            && ![0, 2]
                .iter()
                .any(|bus| has(*bus, 0x420) || has(*bus, 0x421))
        {
            detection.flags |= flags::PEDAL_INTERCEPTOR;
            cp.set_flags(detection.flags);
            cp.set_enable_gas_interceptor_d_e_p_r_e_c_a_t_e_d(true);
            cp.set_alpha_longitudinal_available(true);
            cp.set_openpilot_longitudinal_control(true);
            cp.set_pcm_cruise(false);
            cp.set_radar_unavailable(true);
            cp.set_auto_resume_sng(false);
            cp.set_min_enable_speed(profile.min_enable_speed);
            let mut configs = cp.reborrow().get_safety_configs()?;
            let last = configs.len() - 1;
            let mut safety = configs.reborrow().get(last);
            safety.set_safety_param(
                safety.reborrow_as_reader().get_safety_param()
                    | flags::safety::LONG
                    | flags::safety::PEDAL_INTERCEPTOR
                    | profile.safety_profile,
            );
        }
    }
    vehicle_params::finish(
        cp,
        input.settings,
        FinishOptions {
            firmware: input.firmware,
        },
    )?;
    Ok(message)
}
