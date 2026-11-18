use crate::Error;
use capnp::{message::ReaderOptions, serialize};
use openpilot_cereal::car_capnp::{car_params, car_params::lateral_tuning};
use openpilot_control_policy::{
    pid::{Gain, Gains},
    vehicle::Physical,
};

#[derive(Clone)]
pub struct Torque {
    pub gains: Gains,
    pub factor: f32,
    pub offset: f32,
    pub friction: f32,
    pub steering_angle: bool,
    pub deadzone: f64,
}
#[derive(Clone)]
pub enum Tuning {
    Pid(Gains),
    Torque(Torque),
    Other,
}
pub struct Config {
    pub fingerprint: String,
    pub brand: String,
    pub flags: u32,
    pub ext_flags: u32,
    pub passive: bool,
    pub firmware: String,
    pub physical: Physical,
    pub angle: bool,
    pub tuning: Tuning,
    pub long_gains: Gains,
    pub min_steer_speed: f64,
    pub standstill_steering: bool,
    pub saturation_time: f64,
    pub starting_state: bool,
    pub starting_speed: f64,
    pub stop_accel: f64,
    pub start_accel: f64,
    pub stop_rate: f64,
    pub openpilot_long: bool,
    pub pcm_cruise: bool,
    pub steer_delay: f64,
    pub bus_offset: i32,
}
impl Config {
    pub fn decode(bytes: &[u8]) -> Result<Self, Error> {
        let message = serialize::read_message(bytes, ReaderOptions::new())?;
        let cp = message.get_root::<car_params::Reader<'_>>()?;
        let tuning = match cp.get_lateral_tuning().which()? {
            lateral_tuning::Pid(pid) => {
                let p = pid?;
                Tuning::Pid(Gains {
                    p: gain(p.get_kp_b_p()?, p.get_kp_v()?),
                    i: gain(p.get_ki_b_p()?, p.get_ki_v()?),
                    d: Gain::constant(0.),
                    f: f64::from(p.get_kf()),
                })
            }
            lateral_tuning::Torque(torque) => {
                let t = torque?;
                Tuning::Torque(Torque {
                    gains: Gains::constants(
                        f64::from(t.get_kp()),
                        f64::from(t.get_ki()),
                        f64::from(t.get_kf()),
                    ),
                    factor: t.get_lat_accel_factor(),
                    offset: t.get_lat_accel_offset(),
                    friction: t.get_friction(),
                    steering_angle: t.get_use_steering_angle(),
                    deadzone: f64::from(t.get_steering_angle_deadzone_deg()),
                })
            }
            _ => Tuning::Other,
        };
        let long = cp.get_longitudinal_tuning()?;
        let mut firmware = String::new();
        for fw in cp.get_car_fw()? {
            if fw.get_ecu() == Ok(car_params::Ecu::Eps) {
                firmware = bytes_repr(fw.get_fw_version()?);
                break;
            }
        }
        Ok(Self {
            bus_offset: 4 * (cp.get_safety_configs()?.len() as i32 - 1),
            fingerprint: text(cp.get_car_fingerprint()?)?,
            brand: text(cp.get_brand()?)?,
            flags: cp.get_flags(),
            ext_flags: cp.get_ext_flags(),
            passive: cp.get_passive(),
            firmware,
            physical: Physical {
                mass: f64::from(cp.get_mass()),
                inertia: f64::from(cp.get_rotational_inertia()),
                wheelbase: f64::from(cp.get_wheelbase()),
                center_front: f64::from(cp.get_center_to_front()),
                rear_ratio: f64::from(cp.get_steer_ratio_rear()),
                stiffness_front: f64::from(cp.get_tire_stiffness_front()),
                stiffness_rear: f64::from(cp.get_tire_stiffness_rear()),
                steer_ratio: f64::from(cp.get_steer_ratio()),
            },
            angle: cp.get_steer_control_type() == Ok(car_params::SteerControlType::Angle),
            tuning,
            long_gains: Gains {
                p: gain(long.get_kp_b_p()?, long.get_kp_v()?),
                i: gain(long.get_ki_b_p()?, long.get_ki_v()?),
                d: Gain::constant(0.),
                f: f64::from(long.get_kf()),
            },
            min_steer_speed: f64::from(cp.get_min_steer_speed()),
            standstill_steering: cp.get_steer_at_standstill(),
            saturation_time: f64::from(cp.get_steer_limit_timer()),
            starting_state: cp.get_starting_state(),
            starting_speed: f64::from(cp.get_v_ego_starting()),
            stop_accel: f64::from(cp.get_stop_accel()),
            start_accel: f64::from(cp.get_start_accel()),
            stop_rate: f64::from(cp.get_stopping_decel_rate()),
            openpilot_long: cp.get_openpilot_longitudinal_control(),
            pcm_cruise: cp.get_pcm_cruise(),
            steer_delay: f64::from(cp.get_steer_actuator_delay()),
        })
    }
    pub fn meb(&self) -> bool {
        self.brand == "volkswagen" && self.flags & 16 != 0
    }
}
fn gain(
    x: capnp::primitive_list::Reader<'_, f32>,
    y: capnp::primitive_list::Reader<'_, f32>,
) -> Gain {
    Gain {
        x: x.iter().map(f64::from).collect(),
        y: y.iter().map(f64::from).collect(),
    }
}
pub fn text(value: capnp::text::Reader<'_>) -> Result<String, Error> {
    Ok(value
        .to_str()
        .map_err(|_| Error::Contract("non-UTF8 cereal text"))?
        .to_owned())
}
fn bytes_repr(bytes: &[u8]) -> String {
    let quote = if bytes.contains(&b'\'') && !bytes.contains(&b'"') {
        '"'
    } else {
        '\''
    };
    let mut out = format!("b{quote}");
    for &b in bytes {
        match b {
            b'\t' => out.push_str("\\t"),
            b'\n' => out.push_str("\\n"),
            b'\r' => out.push_str("\\r"),
            b'\\' => out.push_str("\\\\"),
            value if char::from(value) == quote => {
                out.push('\\');
                out.push(quote);
            }
            32..=126 => out.push(char::from(b)),
            value => {
                out.push_str(&format!("\\x{value:02x}"));
            }
        }
    }
    out.push(quote);
    out
}
