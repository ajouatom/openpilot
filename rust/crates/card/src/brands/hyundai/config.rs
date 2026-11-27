use super::{bus::CanBus, flags as f, parameters::setting_int, Error};
use openpilot_cereal::car_capnp::car_params;
use openpilot_params::Params;

#[derive(Clone)]
pub struct CarConfig {
    pub candidate: String,
    pub flags: u32,
    pub ext_flags: u32,
    pub bsm: bool,
    pub longitudinal: bool,
    pub pcm: bool,
    pub wheel_speed_factor: f64,
    pub wheelbase: f64,
    pub steer_ratio: f64,
    pub bus: CanBus,
    /// `HyundaiSafetyFlags.CANFD_BLINKER_KICK` on a non-passive car.
    pub kick: bool,
    /// The Kick runs without a lever message (`ext::KICK_NO_LEVER`).
    pub kick_no_lever: bool,
}

impl CarConfig {
    pub fn decode(bytes: &[u8], settings: &Params) -> Result<Self, Error> {
        let message = capnp::serialize::read_message(
            &mut std::io::Cursor::new(bytes),
            capnp::message::ReaderOptions::new(),
        )?;
        let cp = message.get_root::<car_params::Reader<'_>>()?;
        let flags = cp.get_flags();
        let count = cp.get_safety_configs()?.len();
        let offset = u8::try_from(count.checked_sub(1).ok_or(Error::Numeric)? * 4)
            .map_err(|_| Error::Numeric)?;
        Ok(Self {
            candidate: cp.get_car_fingerprint()?.to_str()?.to_owned(),
            flags,
            ext_flags: cp.get_ext_flags(),
            bsm: cp.get_enable_bsm(),
            longitudinal: cp.get_openpilot_longitudinal_control(),
            pcm: cp.get_pcm_cruise(),
            wheel_speed_factor: f64::from(cp.get_wheel_speed_factor()),
            wheelbase: f64::from(cp.get_wheelbase()),
            steer_ratio: f64::from(cp.get_steer_ratio()),
            kick: !cp.get_passive()
                && cp
                    .get_safety_configs()?
                    .iter()
                    .any(|config| config.get_safety_param() & f::safety::CANFD_BLINKER_KICK != 0),
            kick_no_lever: cp.get_ext_flags() & f::ext::KICK_NO_LEVER != 0,
            bus: CanBus::with_offset(
                offset,
                flags & f::HDA2 != 0,
                setting_int(settings, "HyundaiCameraSCC")?,
            ),
        })
    }

    pub fn gear_message(&self) -> &'static str {
        if self.ext_flags & f::ext::GEARS_69 != 0 {
            "GEAR"
        } else if self.flags & f::EV != 0 {
            "ACCELERATOR"
        } else if self.flags & f::ALT_GEARS != 0 {
            "GEAR_ALT"
        } else if self.flags & f::ALT_GEARS_2 != 0 {
            "GEAR_ALT_2"
        } else {
            "GEAR_SHIFTER"
        }
    }

    pub fn accelerator_message(&self) -> &'static str {
        if self.flags & f::EV != 0 {
            "ACCELERATOR"
        } else if self.flags & f::HYBRID != 0 {
            "ACCELERATOR_ALT"
        } else {
            "ACCELERATOR_BRAKE_ALT"
        }
    }

    pub fn button_message(&self) -> &'static str {
        if self.flags & f::ALT_BUTTONS != 0 {
            "CRUISE_BUTTONS_ALT"
        } else {
            "CRUISE_BUTTONS"
        }
    }
}
