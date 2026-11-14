use crate::{parameters::Parameters, Error};
use openpilot_params::Params;

pub struct RuntimeParameters(pub Params);

impl RuntimeParameters {
    fn optional_raw(&self, key: &str) -> Result<Option<Vec<u8>>, Error> {
        match self.0.get(key) {
            Ok(value) => Ok(value),
            Err(openpilot_params::Error::Io(_)) => Ok(None),
            Err(error) => Err(error.into()),
        }
    }

    fn raw(&self, key: &str) -> Result<Vec<u8>, Error> {
        Ok(self.optional_raw(key)?.unwrap_or_default())
    }

    pub fn car_params(&self) -> Result<Vec<u8>, Error> {
        self.raw("CarParams")
    }
}

pub fn integer(bytes: &[u8]) -> Result<i32, Error> {
    Ok(openpilot_beepd::integer(bytes)?)
}
pub fn float(bytes: &[u8]) -> Result<f64, Error> {
    Ok(openpilot_calibrationd::parameters::parse_float(bytes)?)
}

impl Parameters for RuntimeParameters {
    fn stopping_speed(&mut self) -> Result<f64, Error> {
        let observed = self.optional_raw("VEgoStopping")?;
        let value = openpilot_control_policy::stopping::speed(
            float(observed.as_deref().unwrap_or_default()).unwrap_or(f64::NAN),
        );
        if let Some(repair) = value.repair {
            stopping_repairs()
                .try_send((self.0.clone(), observed, repair))
                .ok();
        }
        Ok(value.meters_per_second)
    }
    // A stored value that is not a number reads as 0, like a missing one:
    // a single corrupt setting must not fail every planning cycle.
    fn integer(&mut self, key: &'static str) -> Result<i32, Error> {
        Ok(integer(&self.raw(key)?).unwrap_or(0))
    }
    fn float(&mut self, key: &'static str) -> Result<f64, Error> {
        Ok(float(&self.raw(key)?).unwrap_or(0.))
    }
}

// A bounded persistent worker keeps Params fsync and external-write repairs off
// the model cycle. A busy queue is retried on the next cycle while the bound is
// already enforced in memory.
type StoppingRepair = (Params, Option<Vec<u8>>, i32);

fn apply_stopping_repair((params, observed, value): StoppingRepair) {
    let _ = params.put_if_unchanged(
        "VEgoStopping",
        observed.as_deref(),
        value.to_string().as_bytes(),
    );
}

fn stopping_repairs() -> &'static std::sync::mpsc::SyncSender<StoppingRepair> {
    static WRITES: std::sync::OnceLock<std::sync::mpsc::SyncSender<StoppingRepair>> =
        std::sync::OnceLock::new();
    WRITES.get_or_init(|| {
        let (sender, receiver) = std::sync::mpsc::sync_channel::<StoppingRepair>(1);
        std::thread::Builder::new()
            .name("planner-params".into())
            .spawn(move || {
                while let Ok(repair) = receiver.recv() {
                    apply_stopping_repair(repair);
                }
            })
            .expect("planner Params repair worker");
        sender
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn queued_repair_preserves_a_newer_valid_external_write() {
        let root = tempfile::tempdir().unwrap();
        let params = Params::open(root.path(), "stopping").unwrap();
        params.put("VEgoStopping", b"NaN").unwrap();
        let observed = params.get("VEgoStopping").unwrap();
        params.put("VEgoStopping", b"75").unwrap();
        apply_stopping_repair((params.clone(), observed, 50));
        assert_eq!(params.get("VEgoStopping").unwrap(), Some(b"75".to_vec()));
    }

    #[test]
    fn current_missing_and_invalid_values_are_still_repaired() {
        let root = tempfile::tempdir().unwrap();
        let params = Params::open(root.path(), "stopping").unwrap();
        apply_stopping_repair((params.clone(), None, 50));
        assert_eq!(params.get("VEgoStopping").unwrap(), Some(b"50".to_vec()));
        params.put("VEgoStopping", b"-10").unwrap();
        let observed = params.get("VEgoStopping").unwrap();
        apply_stopping_repair((params.clone(), observed, 10));
        assert_eq!(params.get("VEgoStopping").unwrap(), Some(b"10".to_vec()));
    }
}
