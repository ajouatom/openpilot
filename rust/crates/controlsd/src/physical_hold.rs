//! Fresh opt-in hold lease for the existing Hyundai 0x1EA indicator fields.
//! Port of carrot's `carrot/bluetooth/physical.py`.
use crate::{parameters::Parameters, Error};
use serde_json::Value;
use std::path::PathBuf;

pub const HOLD_PATH: &str = "/dev/shm/carrot-bluetooth/physical-hold.json";
/// `HyundaiExtFlags.CANFD_INDICATOR_HOLD`: the camera's ADRV_0x1EA is DLC 32.
pub const CANFD_INDICATOR_HOLD: u32 = 1 << 19;

/// carrot `native_hold_supported` (6af3073e3): chosen by the observed
/// protocol, not by the model name.
pub fn native_hold_supported(config: &crate::config::Config) -> bool {
    config.brand == "hyundai"
        && !config.passive
        && config.openpilot_long
        && config.ext_flags & CANFD_INDICATOR_HOLD != 0
}

/// `hudControl.physicalBlinkerHold`: 0 legacy, 1 release, 2 left, 3 right.
pub fn lease_value(value: &Value, now: f64, fingerprint: &str) -> u8 {
    let stamp = value.get("stamp").and_then(Value::as_f64);
    let override_ = value.get("override") == Some(&Value::Bool(true));
    let fresh = stamp.is_some_and(|s| s.is_finite() && (0. ..0.35).contains(&(now - s)));
    let own = value.get("fingerprint").and_then(Value::as_str) == Some(fingerprint);
    let leased = fresh && own && matches!(value.get("mode").and_then(Value::as_i64), Some(1 | 2));
    if !leased {
        // A just-expired override still releases for one more second.
        return u8::from(
            own && override_ && stamp.is_some_and(|s| (0. ..1.35).contains(&(now - s))),
        );
    }
    if !override_ {
        return 0;
    }
    match value.get("direction").and_then(Value::as_str) {
        Some("left") => 2,
        Some("right") => 3,
        _ => 1,
    }
}

pub struct HoldReader {
    supported: bool,
    fingerprint: String,
    path: PathBuf,
    next_mode: f64,
    next_read: f64,
    mode: i32,
    value: Value,
}

impl HoldReader {
    pub fn new(config: &crate::config::Config) -> Self {
        Self {
            supported: native_hold_supported(config),
            fingerprint: config.fingerprint.clone(),
            path: HOLD_PATH.into(),
            next_mode: 0.,
            next_read: 0.,
            mode: 0,
            value: Value::Null,
        }
    }

    pub fn read(
        &mut self,
        can_valid: bool,
        now: f64,
        params: &mut impl Parameters,
    ) -> Result<u8, Error> {
        if !self.supported {
            return Ok(0);
        }
        if now >= self.next_mode {
            // A malformed setting disables the lamp feature, never control.
            self.mode = params.integer("AutoBlinkerControl").unwrap_or(0);
            self.next_mode = now + 0.2;
        }
        if !matches!(self.mode, 1 | 2) {
            // Preserve the existing lane-change hold when the feature is off.
            return Ok(0);
        }
        if !can_valid {
            return Ok(1);
        }
        if now >= self.next_read {
            self.value = std::fs::read(&self.path)
                .ok()
                .filter(|bytes| bytes.len() <= 2049)
                .and_then(|bytes| serde_json::from_slice::<Value>(&bytes).ok())
                .filter(Value::is_object)
                .unwrap_or(Value::Null);
            self.next_read = now + 0.05;
        }
        // Recheck age even between reads.
        Ok(lease_value(&self.value, now, &self.fingerprint))
    }
}

#[cfg(test)]
mod tests {
    use super::lease_value;
    use serde_json::json;

    #[test]
    fn lease_requires_fresh_matching_override() {
        let lease = |v: serde_json::Value, now| lease_value(&v, now, "HYUNDAI_IONIQ_5_PE");
        let base = json!({"stamp": 10.0, "fingerprint": "HYUNDAI_IONIQ_5_PE", "mode": 1,
                          "override": true, "direction": "left"});
        assert_eq!(lease(base.clone(), 10.1), 2);
        let mut right = base.clone();
        right["direction"] = json!("right");
        assert_eq!(lease(right, 10.1), 3);
        let mut release = base.clone();
        release["direction"] = json!("none");
        assert_eq!(lease(release, 10.1), 1);
        assert_eq!(
            lease(base.clone(), 10.5),
            1,
            "stale override still releases"
        );
        assert_eq!(lease(base.clone(), 11.5), 0);
        let mut other = base.clone();
        other["fingerprint"] = json!("KIA_EV6");
        assert_eq!(lease(other, 10.1), 0, "another car's record never releases");
        let mut off = base;
        off["override"] = json!(false);
        assert_eq!(lease(off, 10.1), 0);
    }
}
