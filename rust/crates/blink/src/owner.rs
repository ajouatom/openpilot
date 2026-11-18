//! First-ready physical indicator ownership shared by comma BLE and USB
//! sources. Port of carrot's `tools/jetlink/blink_owner.py`.
use serde_json::{json, Value};
use std::{
    fs::{self, OpenOptions},
    io,
    path::PathBuf,
};

pub const DIRECTORY: &str = "/dev/shm/carrot-bluetooth";
/// Longer than the old receiver's command lease and in-flight send.
pub const GRACE: f64 = 0.9;

fn current(record: &Value, now: f64) -> bool {
    record
        .get("until")
        .and_then(Value::as_f64)
        .is_some_and(|until| until.is_finite() && 0. < until - now && until - now <= GRACE + 1.)
}

fn token() -> String {
    let mut bytes = [0u8; 16];
    if let Ok(mut file) = fs::File::open("/dev/urandom") {
        let _ = io::Read::read_exact(&mut file, &mut bytes);
    }
    bytes.iter().map(|b| format!("{b:02x}")).collect()
}

pub struct Owner {
    name: String,
    directory: PathBuf,
    token: String,
    pub held: bool,
    renewed: f64,
}

impl Owner {
    pub fn new(name: &str) -> Self {
        Self::with_directory(name, DIRECTORY.into())
    }

    pub fn with_directory(name: &str, directory: PathBuf) -> Self {
        Self {
            name: name.to_owned(),
            directory,
            token: token(),
            held: false,
            renewed: 0.,
        }
    }

    pub fn acquire(&mut self, ready: bool, now: f64) -> bool {
        if !ready {
            self.release(now);
            return false;
        }
        // A stalled holder must stop before competing again; it cannot
        // resume an old intention after another adapter took ownership.
        if self.held && now - self.renewed >= 0.5 {
            self.release(now);
            return false;
        }
        self.update(false, now).unwrap_or_else(|_| {
            self.held = false;
            false
        })
    }

    /// When the lease was last written.
    pub fn renewed(&self) -> f64 {
        self.renewed
    }

    pub fn release(&mut self, now: f64) {
        if self.held {
            // The last bounded lease expires without a renewal on failure.
            let _ = self.update(true, now);
        }
        self.held = false;
    }

    fn update(&mut self, release: bool, now: f64) -> io::Result<bool> {
        fs::create_dir_all(&self.directory)?;
        let path = self.directory.join("indicator-owner.json");
        let lock = OpenOptions::new()
            .append(true)
            .create(true)
            .open(self.directory.join("indicator-owner.lock"))?;
        // Released when `lock` drops.
        rustix::fs::flock(&lock, rustix::fs::FlockOperation::LockExclusive)?;
        let record: Value = fs::read(&path)
            .ok()
            .and_then(|bytes| serde_json::from_slice(&bytes).ok())
            .unwrap_or(Value::Null);
        let ours = record.get("token").and_then(Value::as_str) == Some(self.token.as_str());
        let value = if release {
            if !ours {
                return Ok(false);
            }
            json!({"name": "switching", "token": null, "until": now + GRACE})
        } else {
            if current(&record, now) && !ours {
                self.held = false;
                return Ok(false);
            }
            json!({"name": self.name, "token": self.token, "until": now + GRACE})
        };
        let temporary = path.with_extension("tmp");
        fs::write(&temporary, value.to_string())?;
        fs::rename(&temporary, &path)?;
        self.held = !release;
        self.renewed = now;
        Ok(self.held)
    }
}

#[cfg(test)]
mod tests {
    use super::Owner;

    #[test]
    fn first_ready_owner_wins_until_release() {
        let dir = std::env::temp_dir().join(format!("blink-owner-{}", std::process::id()));
        let mut ble = Owner::with_directory("S3 BLE", dir.clone());
        let mut usb = Owner::with_directory("USB", dir.clone());
        assert!(ble.acquire(true, 10.));
        assert!(!usb.acquire(true, 10.1), "lease held by BLE");
        assert!(ble.acquire(true, 10.2));
        ble.release(10.3);
        // The switching grace still blocks a different adapter.
        assert!(!usb.acquire(true, 10.4));
        assert!(usb.acquire(true, 11.3));
        // A stalled holder must release before competing again.
        assert!(!usb.acquire(true, 12.));
        std::fs::remove_dir_all(&dir).ok();
    }
}
