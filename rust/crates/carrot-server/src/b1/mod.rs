//! Authenticated BLE output for the existing indicator policy; no raw CAN
//! over BLE. Port of carrot's `carrot/bluetooth/b1_monitor.py`.
mod firmware;
mod intent;
mod worker;

use intent::{command, Intent, Report, STATES};
use openpilot_blink::{owner::Owner, source::Feedback, source::Record};
use openpilot_bluetooth::{
    bluez::{gatt::B1_SERVICE, Action, Bluez},
    Address,
};
use openpilot_params::Params;
use serde_json::{json, Value};
use sha2::{Digest, Sha256};
use std::{
    collections::{HashMap, VecDeque},
    path::Path,
    sync::{Arc, Mutex},
    time::Duration,
};
use worker::monotonic;

pub const ENROLLMENT: &str = "/data/carrot/b1-monitor.json";
pub const SETUP: &str = "/data/carrot/b1-discovery.json";
const STAGED: &str = "/data/carrot/b1-pending.json";
const BADGE: &str = "/dev/shm/carrot-b1-link.json";
const RUNTIME: &str = "/dev/shm/carrot-bluetooth";
const META: &str = "db340a92-804c-4ead-af37-8a0e6af754a1";
const CONTROL: &str = "db340a93-804c-4ead-af37-8a0e6af754a1";
const CLAIM: &str = "db340a95-804c-4ead-af37-8a0e6af754a1";
const TIMEOUT: Duration = Duration::from_millis(350);

/// State the HTTP status and mutations share with the monitor.
pub struct Shared {
    pub state: Mutex<String>,
    pub firmware: Mutex<String>,
    pub next_discovery: Mutex<f64>,
}

pub fn compatible(name: Option<&str>, uuids: &[String]) -> bool {
    name == Some("Carrot-B1") && uuids.iter().any(|u| u == B1_SERVICE)
}

fn read_json(path: &str) -> Value {
    std::fs::read(path)
        .ok()
        .and_then(|bytes| serde_json::from_slice(&bytes).ok())
        .unwrap_or(Value::Null)
}

fn atomic_json(path: &str, value: &Value) -> std::io::Result<()> {
    let path = Path::new(path);
    if let Some(parent) = path.parent() {
        std::fs::create_dir_all(parent)?;
    }
    let temporary = path.with_extension("tmp");
    std::fs::write(&temporary, value.to_string())?;
    std::fs::rename(&temporary, path)
}

/// Keep the BLE connection but leave the ATT channel to the parked updater.
fn maintenance_active() -> bool {
    let Ok(lock) = std::fs::File::open(Path::new(RUNTIME).join("b1-ota.lock")) else {
        return false;
    };
    rustix::fs::flock(&lock, rustix::fs::FlockOperation::NonBlockingLockShared).is_err()
}

fn radio_present() -> bool {
    Path::new("/sys/class/bluetooth/hci0").exists()
}

async fn radio_enabled() -> bool {
    let child = tokio::process::Command::new("sudo")
        .args(["-n", "test", "-f", "/data/bluetooth/ENABLED"])
        .stdout(std::process::Stdio::null())
        .stderr(std::process::Stdio::null())
        .kill_on_drop(true)
        .status();
    matches!(tokio::time::timeout(Duration::from_secs(3), child).await, Ok(Ok(status)) if status.success())
}

fn parked() -> bool {
    let state = read_json(&format!("{RUNTIME}/status.json"));
    state.get("stationary") == Some(&Value::Bool(true))
        && !state
            .get("stopped")
            .and_then(Value::as_bool)
            .unwrap_or(false)
        && state
            .get("time")
            .and_then(Value::as_f64)
            .is_some_and(|t| (0. ..2.).contains(&(monotonic() - t)))
}

fn params_flag(params: &Params, key: &str) -> bool {
    params.get_bool(key).unwrap_or(false)
}

struct Monitor {
    shared: Arc<Shared>,
    owner: Owner,
    client: Bluez,
    owned: Option<Address>,
    worker: Option<worker::Handle>,
    intent: Intent,
    attrs: Option<HashMap<String, String>>,
    session: Option<u32>,
    next_start: f64,
    source: Option<Record>,
    params: Option<Params>,
    sent: VecDeque<u16>,
    next_badge: f64,
    last_display: Option<(String, u8, Option<String>)>,
    proven_at: f64,
    last_command: Option<Value>,
    last_signal_at: f64,
    firmware_image: Option<(Vec<u8>, (u64, String))>,
    firmware_checked: Option<(String, u32)>,
    firmware_next: f64,
}

impl Monitor {
    fn set_firmware(&self, state: &str) {
        if let Ok(mut slot) = self.shared.firmware.lock() {
            state.clone_into(&mut slot);
        }
    }

    /// Dark intervals in a one-touch sequence are not the end of the maneuver.
    fn idle(&self) -> bool {
        let state = self
            .shared
            .state
            .lock()
            .map(|s| s.clone())
            .unwrap_or_default();
        matches!(state.split(" · ").next(), Some("ready" | "missing"))
            && monotonic() - self.last_signal_at >= 4.
            && !self.intent.queued()
            && !self
                .source
                .as_ref()
                .is_some_and(|s| s.hold.is_some() || !s.events.is_empty())
    }

    fn check_session(&self, mac: &Address, session: u32) -> Result<(), String> {
        if self.session != Some(session) || self.owned.as_ref() != Some(mac) {
            return Err("board session changed".into());
        }
        if read_json(ENROLLMENT).get("address").and_then(Value::as_str) != Some(mac.as_str()) {
            return Err("registration changed".into());
        }
        Ok(())
    }

    /// `comma_update.Client.request`: retry a busy adapter.
    async fn gatt(&mut self, uuid: &str, write: Option<&[u8]>) -> Result<Vec<u8>, String> {
        let path = self
            .attrs
            .as_ref()
            .and_then(|a| a.get(uuid))
            .cloned()
            .ok_or("no characteristic")?;
        for _ in 0..40 {
            let result = match write {
                Some(data) => self
                    .client
                    .write_value(&path, data, Duration::from_secs(6))
                    .await
                    .map(|()| Vec::new()),
                None => self.client.read_value(&path, Duration::from_secs(6)).await,
            };
            match result {
                Ok(value) => return Ok(value),
                Err(error) => {
                    let text = error.to_string();
                    let retry = (write.is_none() && text.contains("ATT error: 0x0e"))
                        || text.contains("InProgress")
                        || text.contains("In Progress");
                    if !retry {
                        return Err(text);
                    }
                    tokio::time::sleep(Duration::from_millis(25)).await;
                }
            }
        }
        Err("BLE adapter is busy".into())
    }

    async fn transfer(
        &mut self,
        packet: &[u8],
        state: u8,
        offset: u32,
        mac: &Address,
        session: u32,
    ) -> Result<(), String> {
        self.check_session(mac, session)?;
        self.gatt(firmware::OTA, Some(packet)).await?;
        for _ in 0..80 {
            let raw = self.gatt(firmware::OTA, None).await?;
            if raw.len() == 5 {
                let written = u32::from_le_bytes([raw[1], raw[2], raw[3], raw[4]]);
                if raw[0] == 3 {
                    return Err("Update rejected: check board key, image and quiet CAN bus".into());
                }
                if (raw[0], written) == (state, offset) {
                    return Ok(());
                }
            }
            tokio::time::sleep(Duration::from_millis(15)).await;
        }
        Err("S3 did not acknowledge the flash offset".into())
    }

    async fn install(
        &mut self,
        image: &[u8],
        key: &[u8],
        mac: &Address,
        session: u32,
    ) -> Result<(), String> {
        if !(256..=0x30_0000).contains(&image.len()) || image[0] != 0xe9 || key.len() != 32 {
            return Err("Expected an ESP32-S3 app image and a 32-byte board key".into());
        }
        let meta = self.gatt(META, None).await?;
        let nonce = Report::parse(&meta)
            .map(|r| r.nonce)
            .ok_or("unsupported B1 report")?;
        self.check_session(mac, session)?;
        let quiet = !self
            .attrs
            .as_ref()
            .is_some_and(|a| a.contains_key(firmware::TRACE));
        let mut manifest = firmware::manifest(if quiet { 1 } else { 4 }, nonce, image);
        let tag = {
            use hmac::Mac;
            let mut mac = hmac::Hmac::<Sha256>::new_from_slice(key).map_err(|e| e.to_string())?;
            mac.update(&manifest);
            mac.finalize().into_bytes()
        };
        manifest.extend_from_slice(&tag);
        self.transfer(&manifest, 1, 0, mac, session).await?;
        for (index, chunk) in image.chunks(480).enumerate() {
            let offset = (index * 480) as u32;
            let mut packet = vec![2];
            packet.extend_from_slice(&offset.to_le_bytes());
            packet.extend_from_slice(chunk);
            self.transfer(&packet, 1, offset + chunk.len() as u32, mac, session)
                .await?;
        }
        self.transfer(&[3], 2, image.len() as u32, mac, session)
            .await
    }

    /// `auto_update.check`.
    async fn firmware_check(
        &mut self,
        mac: &Address,
        session: u32,
    ) -> Result<&'static str, String> {
        self.check_session(mac, session)?;
        let key = hex::decode(read_json(ENROLLMENT)["key"].as_str().unwrap_or_default())
            .map_err(|e| e.to_string())?;
        if !self
            .attrs
            .as_ref()
            .is_some_and(|a| a.contains_key(META) && a.contains_key(firmware::OTA))
        {
            return Err("OTA service unavailable".into());
        }
        if self.firmware_image.is_none() {
            let image = tokio::fs::read(firmware::IMAGE)
                .await
                .map_err(|e| e.to_string())?;
            let target = firmware::release_id(&image)?;
            self.firmware_image = Some((image, target));
        }
        let Some((image, target)) = self.firmware_image.clone() else {
            return Err("no image".into());
        };
        if !self.idle() || maintenance_active() {
            return Ok("waiting for indicators");
        }
        let report = if self
            .attrs
            .as_ref()
            .is_some_and(|a| a.contains_key(firmware::TRACE))
        {
            let path = self
                .attrs
                .as_ref()
                .and_then(|a| a.get(firmware::TRACE))
                .cloned()
                .unwrap_or_default();
            self.client
                .read_value(&path, TIMEOUT)
                .await
                .map_err(|e| e.to_string())?
        } else {
            Vec::new()
        };
        let previous = read_json(firmware::RECEIPT);
        // Retry receipts survive comma reboots: wall time.
        let timestamp = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .map_or(0., |d| d.as_secs_f64());
        let action = firmware::decision(&report, &target, &previous, mac.as_str(), timestamp)?;
        let record = |state: &str| {
            let _ = atomic_json(
                firmware::RECEIPT,
                &json!({"mac": mac.as_str(), "elf": target.1, "release": target.0,
                    "time": timestamp, "status": state}),
            );
        };
        let same = previous.get("mac").and_then(Value::as_str) == Some(mac.as_str())
            && previous.get("elf").and_then(Value::as_str) == Some(target.1.as_str())
            && previous.get("status").and_then(Value::as_str) == Some("current");
        if action == "current" && !same {
            record("current");
        }
        if action != "install" {
            return Ok(action);
        }
        // Manual updates share this lock; the S3 independently rejects active TX.
        let lock_path = Path::new(RUNTIME).join("b1-ota.lock");
        let lock = std::fs::OpenOptions::new()
            .append(true)
            .create(true)
            .open(&lock_path)
            .map_err(|e| e.to_string())?;
        rustix::fs::flock(&lock, rustix::fs::FlockOperation::NonBlockingLockExclusive)
            .map_err(|e| e.to_string())?;
        let meta = self.report(TIMEOUT).await?;
        if let Some(worker) = &self.worker {
            if let Some(latest) = worker.latest() {
                self.source = Some(latest);
            }
        }
        if meta.nonce != session
            || meta.version != 2
            || meta.state > 1
            || meta.lamps != 0
            || !self.idle()
        {
            return Ok("waiting for indicators");
        }
        self.check_session(mac, session)?;
        tokio::time::sleep(Duration::from_millis(750)).await;
        self.check_session(mac, session)?;
        record("attempt");
        self.set_firmware("updating");
        self.stop_source();
        self.install(&image, &key, mac, session).await?;
        drop(lock);
        record("uploaded");
        Ok("waiting for reboot")
    }
    fn set_state(&self, state: &str) {
        if let Ok(mut slot) = self.shared.state.lock() {
            state.clone_into(&mut slot);
        }
    }

    fn publish_badge(&mut self, state: &str, now: f64, lamps: u8, accepted: Option<String>) {
        let display = (state.to_owned(), lamps, accepted.clone());
        if self.last_display.as_ref() != Some(&display) || now >= self.next_badge {
            self.last_display = Some(display);
            self.next_badge = now + 0.35;
            let ready = self
                .source
                .as_ref()
                .is_some_and(|s| s.ok && (0. ..0.2).contains(&(now - s.stamp)));
            let _ = atomic_json(
                BADGE,
                &json!({"state": state, "time": now, "lamps": lamps, "accepted_id": accepted,
                    "source_ready": ready, "last_command": self.last_command}),
            );
        }
    }

    fn stop_source(&mut self) {
        self.worker = None;
        self.source = None;
        self.owner.release(monotonic());
    }

    async fn release(&mut self) {
        self.stop_source();
        if let Some(owned) = self.owned.take() {
            let _ = self.client.device_action(&owned, Action::Disconnect).await;
        }
        self.attrs = None;
        self.session = None;
        self.intent = Intent::new(monotonic());
        let _ = self.client.close().await;
    }

    async fn discover(
        &mut self,
        mac: &Address,
    ) -> Result<&'static str, openpilot_bluetooth::bluez::Error> {
        let Some(device) = self.client.discover_gatt(mac, B1_SERVICE).await? else {
            self.client.scan().await?;
            return Ok("scanning");
        };
        if !compatible(device.name.as_deref(), &device.uuids) {
            self.client.scan().await?;
            return Ok("scanning");
        }
        if !device.connected {
            if device.rssi.is_none() {
                self.client.scan().await?;
                return Ok("scanning");
            }
            self.client.device_action(mac, Action::Connect).await?;
            self.owned = Some(mac.clone());
            return Ok("connecting");
        }
        self.owned = Some(mac.clone());
        if device.characteristics.contains_key(META) && device.characteristics.contains_key(CONTROL)
        {
            self.attrs = Some(device.characteristics);
            return Ok("connected");
        }
        Ok("waiting for control firmware")
    }

    async fn report(&mut self, timeout: Duration) -> Result<Report, String> {
        let path = self
            .attrs
            .as_ref()
            .and_then(|a| a.get(META))
            .cloned()
            .ok_or("no META")?;
        let raw = self
            .client
            .read_value(&path, timeout)
            .await
            .map_err(|e| e.to_string())?;
        Report::parse(&raw).ok_or_else(|| "unsupported B1 report".to_owned())
    }

    async fn write(&mut self, uuid: &str, packet: &[u8], timeout: Duration) -> Result<(), String> {
        let path = self
            .attrs
            .as_ref()
            .and_then(|a| a.get(uuid))
            .cloned()
            .ok_or("no characteristic")?;
        self.client
            .write_value(&path, packet, timeout)
            .await
            .map_err(|e| e.to_string())
    }

    async fn exchange(&mut self, key: &[u8]) -> Result<(), String> {
        if self.params.is_none() {
            self.params = Params::for_runtime().ok();
        }
        // Conservative board-clock sample: GATT latency only shortens the lease.
        let report = self.report(TIMEOUT).await?;
        if report.nonce == 0 || report.version != 2 || usize::from(report.state) >= STATES.len() {
            return Err("unsupported B1 report".into());
        }
        let state = STATES[usize::from(report.state)];
        let now = monotonic();
        if self.session != Some(report.nonce) {
            self.stop_source();
            self.session = Some(report.nonce);
            self.intent = Intent::new(now);
            self.intent.seq = report.last_seq;
            self.sent.clear();
            self.last_signal_at = now;
            self.proven_at = -10.;
            self.last_command = None;
            self.next_start = now + 0.65;
        }
        if self.sent.contains(&report.last_seq) {
            self.proven_at = now;
        }
        let authenticated = now - self.proven_at < 1.;
        // carrot `external_output_allowed`: the SPAS option and the panda
        // Kick own their own output; offroad has no control lease.
        let onroad = self
            .params
            .as_ref()
            .is_some_and(|p| params_flag(p, "IsOnroad"));
        let spas = self.params.as_ref().is_some_and(|p| {
            params_flag(p, "CanfdBlinkerControl") || params_flag(p, "CanfdBlinkerKick")
        });
        let available = authenticated && !matches!(state, "missing" | "error") && onroad && !spas;
        let ready = available && (self.owner.held || (state == "ready" && report.lamps == 0));
        let available = self.owner.acquire(ready, now);
        if self
            .worker
            .as_ref()
            .is_some_and(|w| !available || w.exited())
        {
            self.stop_source();
            self.intent.clear();
            self.next_start = now + 1.;
        }
        if self.worker.is_none() && available && now >= self.next_start {
            self.worker = Some(worker::Handle::start());
            self.next_start = now + 2.;
        }
        if let Some((seq, _, sent_at)) = &self.intent.inflight {
            if state != "sending" && report.ack != *seq && now - sent_at > 0.45 {
                self.intent.clear();
            }
        }
        let accepted = self
            .intent
            .inflight
            .as_ref()
            .filter(|(seq, _, _)| report.ack == *seq)
            .map(|(_, id, _)| id.clone());
        if let Some(worker) = &self.worker {
            worker.send(Feedback {
                stamp: now,
                accepted_id: accepted.clone(),
                available: false,
                state,
                lamps: Some(report.lamps),
                first_sent: None,
            });
            if let Some(latest) = worker.latest() {
                self.source = Some(latest);
            }
        }
        // The worker can publish a newer stamp while its result is drained.
        let now = monotonic();
        let (op, ident, ttl) = self
            .intent
            .choose(self.source.as_ref(), state, report.ack, now);
        if report.lamps != 0
            || !matches!(state, "ready" | "missing")
            || ident.is_some()
            || self.intent.queued()
            || self
                .source
                .as_ref()
                .is_some_and(|s| s.hold.is_some() || !s.events.is_empty())
        {
            self.last_signal_at = now;
        }
        self.intent.seq = self.intent.seq.wrapping_add(1);
        if self.intent.seq == 0 {
            self.intent.seq = 1;
        }
        let packet = command(key, report.nonce, report.stamp, self.intent.seq, op, ttl);
        self.write(CONTROL, &packet, TIMEOUT).await?;
        if self.sent.len() == 6 {
            self.sent.pop_front();
        }
        self.sent.push_back(self.intent.seq);
        if let Some(ident) = ident {
            self.last_command = Some(json!({"id": ident, "seq": self.intent.seq, "time": now}));
            self.intent.inflight = Some((self.intent.seq, ident, now));
        }
        self.set_state(&if authenticated {
            format!(
                "{state} · L={} R={}",
                report.lamps & 1,
                (report.lamps >> 1) & 1
            )
        } else {
            "verifying board key".to_owned()
        });
        self.publish_badge(
            if authenticated { state } else { "auth" },
            now,
            report.lamps,
            accepted,
        );
        Ok(())
    }

    /// Automatic setup while parked; only an explicitly selected device can
    /// break a tie, and scanning never takes a HID remote.
    async fn enroll(&mut self) -> Result<(), String> {
        let setup = read_json(SETUP);
        if setup
            .get("disabled")
            .and_then(Value::as_bool)
            .unwrap_or(false)
            || !parked()
            || !radio_present()
            || !radio_enabled().await
        {
            return Ok(());
        }
        self.client.scan().await.map_err(|e| e.to_string())?;
        tokio::time::sleep(Duration::from_secs(5)).await;
        if !parked() {
            return Ok(());
        }
        let snapshot = self.client.snapshot().await.map_err(|e| e.to_string())?;
        let selected = setup.get("selected").and_then(Value::as_str);
        let candidates: Vec<_> = snapshot
            .devices
            .iter()
            .filter(|d| compatible(Some(&d.name), &d.uuids) && (d.rssi.is_some() || d.connected))
            .filter(|d| selected.is_none_or(|s| d.address.as_deref() == Some(s)))
            .collect();
        if candidates.len() != 1 {
            self.set_state(if candidates.is_empty() {
                "scanning for Carrot-B1"
            } else {
                "multiple Carrot-B1 devices"
            });
            return Ok(());
        }
        let device = candidates[0];
        let mac = Address::parse(device.address.as_deref().unwrap_or_default())
            .map_err(|e| e.to_string())?;
        if !device.paired {
            self.client
                .start_pair_headless(&mac)
                .await
                .map_err(|e| e.to_string())?;
            if !self.client.pair_wait().await.map_err(|e| e.to_string())? {
                self.set_state("waiting for encrypted pairing");
                return Ok(());
            }
        }
        if !parked() {
            return Ok(());
        }
        for _ in 0..10 {
            let _ = self.discover(&mac).await;
            if self.attrs.is_some() || !parked() {
                break;
            }
            tokio::time::sleep(Duration::from_millis(500)).await;
        }
        if !parked() || !self.attrs.as_ref().is_some_and(|a| a.contains_key(CLAIM)) {
            return Ok(());
        }
        // Save the proposed key before writing to the board; a retry after
        // power loss reuses the seed until the board proves ownership.
        let mut staged = read_json(STAGED);
        let seed_ok = staged.get("address").and_then(Value::as_str) == Some(mac.as_str())
            && staged
                .get("seed")
                .and_then(Value::as_str)
                .is_some_and(|s| s.len() == 32 && hex::decode(s).is_ok());
        if !seed_ok {
            let mut bytes = [0u8; 16];
            std::fs::File::open("/dev/urandom")
                .and_then(|mut f| std::io::Read::read_exact(&mut f, &mut bytes))
                .map_err(|e| e.to_string())?;
            staged = json!({"address": mac.as_str(), "seed": hex::encode(bytes)});
            atomic_json(STAGED, &staged).map_err(|e| e.to_string())?;
        }
        let seed =
            hex::decode(staged["seed"].as_str().unwrap_or_default()).map_err(|e| e.to_string())?;
        let key = Sha256::digest(&seed).to_vec();
        let first = self.report(Duration::from_secs(10)).await?;
        if first.nonce == 0 || !parked() {
            return Ok(());
        }
        let mut claim = first.nonce.to_le_bytes().to_vec();
        claim.extend_from_slice(&seed);
        self.write(CLAIM, &claim, Duration::from_secs(10)).await?;
        tokio::time::sleep(Duration::from_millis(120)).await;
        let report = self.report(Duration::from_secs(10)).await?;
        if report.nonce != first.nonce || report.version != 2 {
            return Ok(());
        }
        let seq = match report.last_seq.wrapping_add(1) {
            0 => 1,
            seq => seq,
        };
        self.write(
            CONTROL,
            &command(&key, report.nonce, report.stamp, seq, 3, 0.35),
            Duration::from_secs(10),
        )
        .await?;
        tokio::time::sleep(Duration::from_millis(120)).await;
        let proof = self.report(Duration::from_secs(10)).await?;
        if proof.nonce == report.nonce
            && proof.last_seq == seq
            && parked()
            && read_json(SETUP) == setup
        {
            atomic_json(
                ENROLLMENT,
                &json!({"address": mac.as_str(), "name": "Carrot-B1", "key": hex::encode(&key)}),
            )
            .map_err(|e| e.to_string())?;
        } else {
            self.set_state("board already enrolled or first-use window closed");
        }
        Ok(())
    }

    /// `auto_update.run`, checked every 5 s between exchanges.
    async fn auto_update(&mut self, mac: &Address) {
        let now = monotonic();
        if now < self.firmware_next {
            return;
        }
        self.firmware_next = now + 5.;
        let (Some(owned), Some(session)) = (self.owned.clone(), self.session) else {
            return;
        };
        if owned != *mac
            || self.firmware_checked == Some((owned.as_str().to_owned(), session))
            || now - self.proven_at >= 1.
            || !self.idle()
            || maintenance_active()
        {
            return;
        }
        match self.firmware_check(mac, session).await {
            Ok(result) => {
                self.set_firmware(result);
                if matches!(result, "current" | "newer" | "version conflict") {
                    self.firmware_checked = Some((owned.as_str().to_owned(), session));
                } else {
                    self.firmware_next = monotonic() + 30.;
                }
            }
            Err(error) => {
                self.set_firmware(&error.chars().take(100).collect::<String>());
                self.firmware_next = monotonic() + 30.;
            }
        }
    }

    async fn run(mut self) {
        let mut registration: Option<(Address, Vec<u8>)> = None;
        let mut next_config = 0.;
        loop {
            let now = monotonic();
            if now >= next_config {
                let record = read_json(ENROLLMENT);
                let new = (|| {
                    let key = hex::decode(record.get("key")?.as_str()?).ok()?;
                    let address = Address::parse(record.get("address")?.as_str()?).ok()?;
                    (key.len() == 32).then_some((address, key))
                })();
                if new
                    .as_ref()
                    .map(|(a, k)| (a.as_str().to_owned(), k.clone()))
                    != registration
                        .as_ref()
                        .map(|(a, k)| (a.as_str().to_owned(), k.clone()))
                {
                    self.release().await;
                    registration = new;
                }
                next_config = now + 0.25;
            }
            let Some((mac, key)) = registration.clone().filter(|_| radio_present()) else {
                if self.owned.is_some() || self.worker.is_some() {
                    self.release().await;
                }
                let due = self
                    .shared
                    .next_discovery
                    .lock()
                    .map(|n| now >= *n)
                    .unwrap_or(false);
                if registration.is_none() && due {
                    if let Ok(mut next) = self.shared.next_discovery.lock() {
                        *next = now + 30.;
                    }
                    if self.enroll().await.is_err() {
                        self.set_state("waiting for name discovery");
                    }
                    self.release().await;
                } else if registration.is_some() {
                    self.set_state("waiting for radio");
                }
                tokio::time::sleep(Duration::from_millis(500)).await;
                continue;
            };
            let result = if self.attrs.is_some() {
                if maintenance_active() {
                    self.stop_source();
                    self.intent.clear();
                    self.set_state("firmware update");
                    self.publish_badge("updating", monotonic(), 0, None);
                    tokio::time::sleep(Duration::from_millis(150)).await;
                    Ok(())
                } else {
                    let result = self.exchange(&key).await;
                    tokio::time::sleep(Duration::from_millis(35)).await;
                    if result.is_ok() {
                        self.auto_update(&mac).await;
                    }
                    result
                }
            } else {
                match self.discover(&mac).await {
                    Ok(state) => {
                        self.set_state(state);
                        tokio::time::sleep(Duration::from_millis(if self.owned.is_some() {
                            500
                        } else {
                            3000
                        }))
                        .await;
                        Ok(())
                    }
                    Err(error) => Err(error.to_string()),
                }
            };
            if result.is_err() {
                self.release().await;
                self.set_state("waiting to reconnect");
                tokio::time::sleep(Duration::from_secs(2)).await;
            }
        }
    }
}

/// Starts the monitor on its own thread and runtime.
pub fn start() -> Arc<Shared> {
    let shared = Arc::new(Shared {
        state: Mutex::new("idle".into()),
        firmware: Mutex::new("waiting".into()),
        next_discovery: Mutex::new(0.),
    });
    let monitor_shared = Arc::clone(&shared);
    let started = std::thread::Builder::new()
        .name("b1-monitor".into())
        .spawn(move || {
            let Ok(runtime) = tokio::runtime::Builder::new_current_thread()
                .enable_all()
                .build()
            else {
                return;
            };
            runtime.block_on(
                Monitor {
                    shared: monitor_shared,
                    owner: Owner::new("S3 BLE"),
                    client: Bluez::new(None),
                    owned: None,
                    worker: None,
                    intent: Intent::new(monotonic()),
                    attrs: None,
                    session: None,
                    next_start: 0.,
                    source: None,
                    params: None,
                    sent: VecDeque::with_capacity(6),
                    next_badge: 0.,
                    last_display: None,
                    proven_at: -10.,
                    last_command: None,
                    last_signal_at: monotonic(),
                    firmware_image: None,
                    firmware_checked: None,
                    firmware_next: 0.,
                }
                .run(),
            );
        });
    if let Err(error) = started {
        eprintln!("B1 monitor startup: {error}");
    }
    shared
}
