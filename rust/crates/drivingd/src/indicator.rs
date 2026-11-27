//! Inputs of the lamp-only physical-indicator observers, gathered the way
//! carrot's `modeld.py` passes them to `DesireHelper.update`.
use crate::{parameters, Error};
use openpilot_cereal::{
    car_capnp::{car_control, car_params},
    log_capnp::event,
};
use openpilot_desire::{
    helper::DesireHelper,
    types::{Indicator, IndicatorConfig},
};
use openpilot_messaging::state::State;
use openpilot_params::Params;
use serde_json::Value;
use std::path::Path;

const RUNTIME: &str = "/dev/shm/carrot-bluetooth";

fn read(name: &str) -> Value {
    std::fs::read(Path::new(RUNTIME).join(name))
        .ok()
        .and_then(|bytes| serde_json::from_slice(&bytes).ok())
        .unwrap_or(Value::Null)
}

/// `HyundaiSafetyFlags.CANFD_BLINKER_KICK`.
const CANFD_BLINKER_KICK: u16 = 32768;
/// Hyundai ext flag `KICK_NO_LEVER` (card flags.rs ext): the Kick runs on a
/// car with no 0x3C1 lever message (CanfdBlinkerKickNoLever).
const HYUNDAI_EXT_KICK_NO_LEVER: u32 = 1 << 20;

/// Which observer paths this car runs (carrot modeld.py, 6af3073e3): the
/// external receivers on any non-passive car, the Kick on a Hyundai whose
/// safety config carries the Kick flag. Selected by protocol, not by model.
#[derive(Clone, Copy, Debug, Default)]
pub struct Support {
    pub external: bool,
    /// A Kick car with a verified 0x3C1 lever.
    pub kick: bool,
    /// A Kick car without a lever message: its lamp is isolated through the
    /// ownership journal like an external output, never as a verified lever.
    pub kick_no_lever: bool,
}

impl Support {
    pub fn read(cp: car_params::Reader<'_>) -> Result<Self, Error> {
        let passive = cp.get_passive();
        let any_kick = !passive
            && cp.get_brand()?.to_str()? == "hyundai"
            && cp
                .get_safety_configs()?
                .iter()
                .any(|config| config.get_safety_param() & CANFD_BLINKER_KICK != 0);
        let no_lever = any_kick && cp.get_ext_flags() & HYUNDAI_EXT_KICK_NO_LEVER != 0;
        Ok(Self {
            external: !passive,
            kick: any_kick && !no_lever,
            kick_no_lever: no_lever,
        })
    }

    /// carrot `boundary_indicator_supported`.
    pub fn any(self) -> bool {
        self.external || self.kick
    }
}

/// `boundary-indicator.json` as `(time, direction, mask)` for a restart.
pub fn startup_journal() -> Option<(f64, i64, Option<i64>)> {
    let journal = read("boundary-indicator.json");
    Some((
        journal.get("time")?.as_f64()?,
        journal.get("direction")?.as_i64()?,
        journal.get("mask").and_then(Value::as_i64),
    ))
}

fn periodic_journal() -> Option<(f64, i64)> {
    let journal = read("boundary-indicator.json");
    let mask = journal.get("mask").or_else(|| journal.get("direction"))?;
    Some((journal.get("time")?.as_f64()?, mask.as_i64()?))
}

fn flag(params: &Params, key: &str) -> Result<bool, Error> {
    Ok(params.get(key)?.is_some_and(|value| value == b"1"))
}

fn config(params: &Params, support: Support) -> Result<IndicatorConfig, Error> {
    let mode = parameters::integer(params, "AutoBlinkerControl")?;
    let kick = flag(params, "CanfdBlinkerKick")?;
    let supported = if kick {
        support.kick || support.kick_no_lever
    } else {
        support.external
    };
    let lamp = supported && matches!(mode, 1 | 2) && !flag(params, "CanfdBlinkerControl")?;
    Ok(IndicatorConfig {
        manual_minimum: if (support.kick || support.kick_no_lever) && kick {
            2.
        } else {
            0.
        },
        boundary: lamp && flag(params, "ManualLaneChangeIndicator")?,
        manual_turn: lamp && flag(params, "ManualTurnIndicator")?,
        model_desire: lamp && flag(params, "ModelDesireIndicator")?,
        model_path: lamp && flag(params, "ModelPathIndicator")?,
    })
}

fn fresh(state: &State, name: &str, now: f64, limit: f64) -> Result<bool, Error> {
    let topic = state.topic(name)?;
    Ok(topic.valid && topic.alive && (0. ..limit).contains(&(now - topic.receive_time)))
}

fn boottime_ns() -> u64 {
    let now = rustix::time::clock_gettime(rustix::time::ClockId::Boottime);
    (now.tv_sec as u64)
        .saturating_mul(1_000_000_000)
        .saturating_add(now.tv_nsec as u64)
}

fn yaw_rate(state: &State, now: f64) -> Result<Option<f64>, Error> {
    if !fresh(state, "livePose", now, 0.2)? {
        return Ok(None);
    }
    let reader = state.topic("livePose")?.event()?;
    let event::LivePose(pose) = reader.which()? else {
        return Ok(None);
    };
    let pose = pose?;
    let angular = pose.get_angular_velocity_device()?;
    let age = boottime_ns().saturating_sub(reader.get_log_mono_time());
    Ok((pose.get_inputs_o_k()
        && pose.get_sensors_o_k()
        && age < 200_000_000
        && pose.get_posenet_o_k()
        && angular.get_valid())
    .then(|| f64::from(angular.get_z())))
}

/// carrot `turn_signal_context` (Tesla only): whether a stopped turn may
/// keep its lamp, and the fresh heading `(logMonoTime ns, NED z)`.
fn turn_signal_context(
    state: &State,
    car: openpilot_cereal::car_capnp::car_state::Reader<'_>,
    now: f64,
    boot: u64,
    always_lateral: bool,
) -> Result<(bool, Option<(u64, f64)>), Error> {
    let heading = if fresh(state, "livePose", now, 0.35)? {
        let reader = state.topic("livePose")?.event()?;
        let stamp = reader.get_log_mono_time();
        match reader.which()? {
            event::LivePose(pose) => {
                let pose = pose?;
                let o = pose.get_orientation_n_e_d()?;
                let z = f64::from(o.get_z());
                (0 < stamp
                    && stamp <= boot
                    && boot - stamp <= 350_000_000
                    && pose.get_inputs_o_k()
                    && pose.get_sensors_o_k()
                    && pose.get_posenet_o_k()
                    && o.get_valid()
                    && z.is_finite())
                .then_some((stamp, z))
            }
            _ => None,
        }
    } else {
        None
    };
    let active = match state.topic("selfdriveState")?.event()?.which()? {
        event::SelfdriveState(s) => s?.get_active(),
        _ => false,
    };
    let allowed = heading.is_some()
        && fresh(state, "selfdriveState", now, 0.35)?
        && fresh(state, "carControl", now, 0.35)?
        && car.get_lat_enabled()
        && (active || always_lateral)
        && !car.get_steer_fault_temporary()
        && !car.get_steer_fault_permanent();
    Ok((allowed, heading))
}

/// Observer inputs for the next model frame. `valid` here covers carState
/// and carControl freshness; the caller joins it with modelV2 validity.
pub fn inputs(
    state: &State,
    control: car_control::Reader<'_>,
    params: &Params,
    helper: &DesireHelper,
    kick_no_lever: bool,
    now: f64,
) -> Result<Indicator, Error> {
    let valid = fresh(state, "carState", now, 0.35)? && fresh(state, "carControl", now, 0.35)?;
    let boot = boottime_ns();
    let (turn_hold_allowed, turn_pose) = if helper.tesla.enabled {
        let car = match state.topic("carState")?.event()?.which()? {
            event::CarState(car) => Some(car?),
            _ => None,
        };
        match car {
            Some(car) => {
                let always = !control.get_lat_active()
                    && params.get("AlwaysLateral")?.is_some_and(|v| v == b"1");
                let (allowed, pose) = turn_signal_context(state, car, now, boot, always)?;
                (allowed && valid, pose)
            }
            None => (false, None),
        }
    } else {
        (false, None)
    };
    // The helper increments its frame before reading these.
    let frame = helper.frame.wrapping_add(1);
    let periodic = helper.indicator_supported && (frame == 1 || frame.is_multiple_of(20));
    Ok(Indicator {
        now,
        valid,
        curvature: Some(f64::from(control.get_actuators()?.get_curvature())),
        yaw_rate: yaw_rate(state, now)?,
        feedback: helper
            .model_indicator
            .active
            .is_some()
            .then(|| read("physical-hold.json")),
        journal: if periodic { periodic_journal() } else { None },
        config: if periodic {
            // A Kick car is never passive, so `external` equals "any".
            Some(config(
                params,
                Support {
                    external: helper.indicator_supported,
                    kick: helper.kick_supported,
                    kick_no_lever,
                },
            )?)
        } else {
            None
        },
        assistant_valid: valid && fresh(state, "radarState", now, 0.35)?,
        turn_hold_allowed,
        turn_pose,
        boot: boot as f64 / 1e9,
    })
}

#[cfg(test)]
mod kick_support_tests {
    use super::*;
    use openpilot_cereal::car_capnp::car_params::SafetyModel;

    fn support(param: u16, ext: u32) -> Support {
        let mut message = capnp::message::Builder::new_default();
        let mut cp = message.init_root::<car_params::Builder<'_>>();
        cp.set_brand("hyundai");
        cp.set_ext_flags(ext);
        let mut configs = cp.reborrow().init_safety_configs(1);
        let mut config = configs.reborrow().get(0);
        config.set_safety_model(SafetyModel::HyundaiCanfd);
        config.set_safety_param(param);
        Support::read(cp.into_reader()).unwrap()
    }

    #[test]
    fn a_no_lever_kick_is_not_a_verified_lever_car() {
        let lever = support(CANFD_BLINKER_KICK, 0);
        assert!(lever.kick && !lever.kick_no_lever);
        let no_lever = support(CANFD_BLINKER_KICK, HYUNDAI_EXT_KICK_NO_LEVER);
        assert!(!no_lever.kick && no_lever.kick_no_lever && no_lever.external);
        let none = support(0, HYUNDAI_EXT_KICK_NO_LEVER);
        assert!(!none.kick && !none.kick_no_lever, "the ext flag alone means nothing");
    }
}
