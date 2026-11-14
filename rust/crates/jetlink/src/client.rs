//! Strict original artifact identity and frame/sequence guard around the v2 peer.
//! Wire peer ported from third_party/jetlink/client.py (MIT, Copyright (c) 2026 Zeph Leggett).
use crate::{
    contract::{self, Identity},
    owner::Backend,
    rpc,
    wire::{self, Header},
    Deadline, Error,
};
use serde_json::{json, Value};
use std::{fs::File, io::Read, time::Duration};
mod boot;
mod model_cache;
mod upload;

pub struct Message {
    pub header: Header,
    pub payload: Vec<u8>,
}
pub trait Transport: Send + 'static {
    fn version(&self) -> wire::Version {
        wire::Version::V2
    }
    fn gadget(&self) -> bool {
        true
    }
    fn send(&mut self, bytes: &[u8], deadline: Deadline) -> Result<(), Error>;
    fn receive(&mut self, deadline: Deadline) -> Result<Message, Error>;
    fn close(&mut self);
}
impl<T: Transport + ?Sized> Transport for Box<T> {
    fn version(&self) -> wire::Version {
        (**self).version()
    }
    fn gadget(&self) -> bool {
        (**self).gadget()
    }
    fn send(&mut self, bytes: &[u8], deadline: Deadline) -> Result<(), Error> {
        (**self).send(bytes, deadline)
    }
    fn receive(&mut self, deadline: Deadline) -> Result<Message, Error> {
        (**self).receive(deadline)
    }
    fn close(&mut self) {
        (**self).close();
    }
}
type HelloCheck = Box<dyn Fn(&Value) -> Result<(), Error> + Send>;
/// INFER_REQ flag: a uint8 256x256 route map follows the packed floats
/// (the modified Jetlink app; the server keeps it until the next one).
pub const NAV_MAP_FLAG: u32 = 1 << 3;
/// carrot's map freshness gate (map_client.py).
const MAP_FRESH_NS: u64 = 500_000_000;
pub struct Client<T: Transport> {
    transport: T,
    sequence: u32,
    last_frame: Option<u32>,
    ready: bool,
    dead: bool,
    session: Option<crate::session_contract::SessionContract>,
    app: Option<AppSelection>,
    telemetry_at: u64,
    /// rendered_ns of the last route map sent to a navigation graph; Some(0)
    /// after a blank one; None when the server's copy is unknown (new
    /// session or a reset), so the next frame sends the current map.
    map_sent: Option<u64>,
    idle_at: u64,
    hello_timeout: Duration,
    hello_check: Option<HelloCheck>,
    source_sof: u64,
    phase: crate::phase::Publisher,
    phase_enabled: bool,
    wifi: Option<crate::wifi::Publisher>,
    app_request_pending: bool,
    client_nonce: Option<String>,
    pub status: std::sync::Arc<std::sync::Mutex<Value>>,
    pub hello_complete: std::sync::Arc<std::sync::atomic::AtomicBool>,
}
struct AppSelection {
    cache: std::path::PathBuf,
    connected: std::sync::Arc<dyn Fn() -> bool + Send + Sync>,
    /// Parked: `IsOffroad and not IsOnroad`, as carrot's daemon reads it.
    parked: std::sync::Arc<dyn Fn() -> bool + Send + Sync>,
    /// Our engine request was given back to the App and must be made again
    /// before the next modeld session.
    released: bool,
    /// `at` of the comma's pick named in the last hello; a new pick made
    /// while parked reconnects so the App hears it.
    pick_at: Option<f64>,
}
fn parked() -> bool {
    let read = |name: &str| std::fs::read_to_string(format!("/data/params/d/{name}"));
    read("IsOffroad").is_ok_and(|value| value.trim() == "1")
        && !read("IsOnroad").is_ok_and(|value| value.trim() == "1")
}
impl<T: Transport> Client<T> {
    /// A new, fresh, valid route map to send, or one blank map when it went
    /// stale so the server stops using the last one; None when unchanged.
    /// The hidden_state range a protocol-3 stateful session leaves out of
    /// replies. carrot's client asked for the whole vector (WANT_HIDDEN)
    /// though nothing on the comma reads a stateful graph's hidden state.
    fn omitted_hidden(&self) -> Option<[usize; 2]> {
        let contract = self.session_contract();
        (contract.version == wire::Version::V3 && contract.spec.stateful())
            .then(|| contract.spec.range("hidden_state").ok())
            .flatten()
    }

    /// HELLO: our identity, and on protocol 3 the model picked on the comma.
    fn hello_payload(
        &mut self,
        nonce: &str,
    ) -> Result<(Vec<u8>, Option<crate::app_pick::AppPick>), Error> {
        let mut hello = json!({"client":{"name":"jetlinkd-rs","nonce":nonce}});
        let pick = self
            .app
            .as_ref()
            // Only while parked: a link that drops and rejoins on the road
            // keeps the model the App runs; the pick goes at the next park.
            .filter(|app| self.transport.version() == wire::Version::V3 && (app.parked)())
            .and_then(|app| crate::app_pick::current(&app.cache));
        if let Some(pick) = &pick {
            hello["pick"] = pick.hello(crate::now_ns() as f64 / 1e9);
        }
        if let Some(app) = &mut self.app {
            app.pick_at = pick.as_ref().map(|pick| pick.at);
        }
        Ok((serde_json::to_vec(&hello)?, pick))
    }

    fn next_map(&mut self) -> Option<Vec<u8>> {
        let now = crate::now_ns();
        let map =
            openpilot_mapsd::output::read(std::path::Path::new(openpilot_mapsd::output::PATH))
                .filter(|m| m.valid && m.rendered_ns <= now && now - m.rendered_ns < MAP_FRESH_NS);
        match map {
            Some(map) if self.map_sent != Some(map.rendered_ns) => {
                self.map_sent = Some(map.rendered_ns);
                Some(map.image)
            }
            Some(_) => None,
            None if self.map_sent != Some(0) => {
                self.map_sent = Some(0);
                Some(vec![0; openpilot_mapsd::output::MAP_BYTES])
            }
            None => None,
        }
    }

    pub fn new(transport: T) -> Self {
        Self {
            transport,
            sequence: 0,
            last_frame: None,
            ready: false,
            dead: false,
            session: None,
            app: None,
            telemetry_at: 0,
            map_sent: None,
            idle_at: 0,
            hello_timeout: Duration::from_secs(5),
            hello_check: None,
            source_sof: 0,
            phase: crate::phase::Publisher::device(),
            phase_enabled: false,
            wifi: None,
            app_request_pending: false,
            client_nonce: None,
            status: std::sync::Arc::new(std::sync::Mutex::new(json!({}))),
            hello_complete: std::sync::Arc::new(std::sync::atomic::AtomicBool::new(false)),
        }
    }
    pub fn app(transport: T, cache: std::path::PathBuf) -> Self {
        let mut client = Self::new(transport);
        client.app = Some(AppSelection {
            cache,
            connected: std::sync::Arc::new(|| true),
            parked: std::sync::Arc::new(parked),
            released: false,
            pick_at: None,
        });
        client
    }
    fn report(&self, state: &str, stage: &str, peer: Option<&Value>, telemetry: Option<&Value>) {
        if let Ok(mut value) = self.status.lock() {
            value["state"] = json!(state);
            if let Some(sha256) = self
                .session
                .as_ref()
                .map(|session| session.spec.sha256.as_str())
                .or_else(|| self.app.is_none().then_some(contract::SHA256))
            {
                value["sha256"] = json!(sha256);
                value["model"] = json!(crate::status::model_name(sha256));
            }
            value["preparation"] = json!({"stage":stage});
            if let Some(peer) = peer {
                let mut identity = serde_json::Map::new();
                for key in ["protocol", "carrot_host", "backend", "device"] {
                    if let Some(item) = peer.get(key) {
                        identity.insert(
                            key.into(),
                            if let Some(text) = item.as_str() {
                                json!(text.chars().take(200).collect::<String>())
                            } else if item.is_number() || item.is_null() {
                                item.clone()
                            } else {
                                Value::Null
                            },
                        );
                    }
                }
                value["peer"] = Value::Object(identity);
            }
            if let Some(telemetry) = telemetry {
                // Bounded display snapshot; no executable model context is copied.
                let health = &telemetry["carrot_health"];
                let mut compact = serde_json::Map::new();
                for key in ["age_s", "temp_c", "severity", "reason"] {
                    if let Some(item) = health.get(key) {
                        compact.insert(
                            key.into(),
                            if let Some(text) = item.as_str() {
                                json!(text.chars().take(240).collect::<String>())
                            } else if item.is_number() || item.is_null() || item.is_boolean() {
                                item.clone()
                            } else {
                                Value::Null
                            },
                        );
                    }
                }
                if let Some(addresses) = health["addresses"].as_array() {
                    compact.insert("addresses".into(), Value::Array(addresses.iter().take(8).filter_map(|entry| entry["address"].as_str()).map(|address| json!({"address":address.chars().take(80).collect::<String>()})).collect()));
                }
                value["telemetry"] = json!({"carrot_health":compact});
                value["telemetry_updated"] = json!(crate::now_ns() as f64 / 1e9);
            }
        }
    }
    pub fn idle(&mut self) -> Result<(), Error> {
        let result = self.idle_inner();
        if result.is_err() {
            self.close();
        }
        result
    }
    fn idle_inner(&mut self) -> Result<(), Error> {
        self.provision_wifi(false)?;
        if !self.ready || self.app.is_none() || crate::now_ns() < self.idle_at {
            return Ok(());
        }
        // Carrot's daemon gives its engine request back only while parked, so
        // the App's "Use Model" does not compete with it there. While driving
        // the request must stay: without it the phone answers NOT_READY to the
        // first real frame, which made the link drop and reconnect in a loop.
        if self.app.as_ref().is_some_and(|app| (app.parked)()) {
            if let Some(app) = &self.app {
                let picked = crate::app_pick::current(&app.cache).map(|pick| pick.at);
                if self.transport.version() == wire::Version::V3
                    && picked.is_some()
                    && picked != app.pick_at
                {
                    // Rejoin: the next hello carries the new pick.
                    return Err(Error::Contract("model picked on the comma"));
                }
            }
            self.release_app_request()?;
        }
        self.idle_at = crate::now_ns().saturating_add(2_000_000_000);
        let deadline = Deadline::after(Duration::from_millis(500))?;
        let sequence = self.send(12, b"{}", deadline)?;
        let state: Value = serde_json::from_slice(&self.expect(13, sequence, deadline)?.payload)?;
        if !state.is_object() {
            return Err(Error::Contract("idle state object"));
        }
        crate::selection::check_loaded(&self.session_contract().spec, &state)?;
        self.report("ready", "ready", None, Some(&state));
        Ok(())
    }
    pub fn set_connected_check(
        &mut self,
        connected: std::sync::Arc<dyn Fn() -> bool + Send + Sync>,
    ) {
        if let Some(app) = &mut self.app {
            app.connected = connected;
        }
    }
    fn require_setup(&self) -> Result<(), Error> {
        let app = self
            .app
            .as_ref()
            .ok_or(Error::Contract("App setup missing"))?;
        if !(app.connected)() {
            return Err(Error::Closed);
        }
        Ok(())
    }
    fn receive_setup(&mut self, deadline: Deadline) -> Result<Message, Error> {
        loop {
            deadline.remaining()?;
            self.require_setup()?;
            self.provision_wifi(false)?;
            let poll = Deadline(
                deadline
                    .0
                    .min(crate::now_ns().saturating_add(1_000_000_000)),
            );
            match self.transport.receive(poll) {
                Err(Error::Deadline) => continue,
                result => return result,
            }
        }
    }
    fn release_app_request(&mut self) -> Result<(), Error> {
        if !self.app_request_pending {
            return Ok(());
        }
        self.require_setup()?;
        let deadline = Deadline::after(Duration::from_secs(2))?;
        let nonce = self.client_nonce.clone().unwrap_or_default();
        let payload = self.hello_payload(&nonce)?.0;
        let sequence = self.send(1, &payload, deadline)?;
        let hello: Value =
            serde_json::from_slice(&self.expect_setup(2, sequence, deadline)?.payload)?;
        if hello["protocol"].as_u64() != Some(self.transport.version() as u64) {
            return Err(Error::Contract("fresh App HELLO protocol"));
        }
        crate::selection::check_loaded(&self.session_contract().spec, &hello)?;
        self.app_request_pending = false;
        if let Some(app) = &mut self.app {
            app.released = true;
        }
        Ok(())
    }
    /// Before a modeld session: ask for the engine again if it was given back
    /// while parked (daemon.py `if not requested: ... ensure_engine`). Runs
    /// outside every frame deadline.
    pub fn session_start(&mut self) -> Result<(), Error> {
        if !self.ready || !self.app.as_ref().is_some_and(|app| app.released) {
            return Ok(());
        }
        let result = self.request_engine_again();
        if result.is_err() {
            self.close();
        }
        result
    }
    fn request_engine_again(&mut self) -> Result<(), Error> {
        let spec = self.session_contract().spec.clone();
        let deadline = Deadline::after(Duration::from_millis(500))?;
        let sequence = self.send(12, b"{}", deadline)?;
        let state: Value = serde_json::from_slice(&self.expect(13, sequence, deadline)?.payload)?;
        crate::selection::check_loaded(&spec, &state)?;
        let deadline = Deadline::after(Duration::from_secs(30))?;
        let sequence = self.send(
            3,
            &serde_json::to_vec(
                &json!({"sha256":spec.sha256,"nbytes":spec.nbytes,"frame_skip":spec.frame_skip}),
            )?,
            deadline,
        )?;
        let mut state: Value =
            serde_json::from_slice(&self.expect_setup(4, sequence, deadline)?.payload)?;
        loop {
            match state["state"].as_str() {
                Some("ready") => break,
                Some("failed" | "need_upload") => {
                    return Err(Error::Contract("App engine unavailable"))
                }
                _ => {}
            }
            let message = self.receive_setup(deadline)?;
            if message.header.kind == 14 {
                return Err(Error::Contract("App engine error"));
            }
            if message.header.kind == 4 {
                let candidate: Value = serde_json::from_slice(&message.payload)?;
                if candidate["sha256"].is_null() || candidate["sha256"] == spec.sha256 {
                    state = candidate;
                }
            }
        }
        let session = crate::session_contract::SessionContract::parse(
            state["spec"].clone(),
            self.transport.version(),
        )?;
        if session.spec != spec {
            return Err(Error::Contract(
                "model contract changed before local inference",
            ));
        }
        self.app_request_pending = true;
        if let Some(app) = &mut self.app {
            app.released = false;
        }
        Ok(())
    }
    fn provision_wifi(&mut self, initial: bool) -> Result<(), Error> {
        if let Some(wifi) = &self.wifi {
            if let Some(payload) = wifi.latest(initial)? {
                self.send(
                    crate::wifi::MESSAGE,
                    &payload,
                    Deadline::after(Duration::from_secs(1))?,
                )?;
            }
        }
        Ok(())
    }
    pub fn set_source_sof(&mut self, source_sof: u64) {
        self.source_sof = source_sof;
    }
    pub fn set_hello_check(
        &mut self,
        check: impl Fn(&Value) -> Result<(), Error> + Send + 'static,
    ) {
        self.hello_check = Some(Box::new(check));
    }
    pub fn set_hello_timeout(&mut self, timeout: Duration) -> Result<(), Error> {
        if timeout.is_zero() || timeout > Duration::from_secs(5) {
            return Err(Error::Contract(
                "HELLO timeout must be positive and bounded",
            ));
        }
        self.hello_timeout = timeout;
        Ok(())
    }
    fn send(&mut self, kind: u16, payload: &[u8], deadline: Deadline) -> Result<u32, Error> {
        self.sequence = self
            .sequence
            .checked_add(1)
            .ok_or(Error::Contract("sequence exhausted"))?;
        let bytes = wire::frame_version(
            kind,
            self.sequence,
            payload,
            self.transport.gadget(),
            self.transport.version(),
        )?;
        self.transport.send(&bytes, deadline)?;
        Ok(self.sequence)
    }
    fn expect(&mut self, kind: u16, sequence: u32, deadline: Deadline) -> Result<Message, Error> {
        loop {
            deadline.remaining()?;
            let message = self.transport.receive(deadline)?;
            if message.header.kind == kind && message.header.sequence == sequence {
                return Ok(message);
            }
            if message.header.kind == 13 {
                if let Some(session) = &self.session {
                    crate::selection::check_loaded(
                        &session.spec,
                        &serde_json::from_slice(&message.payload)?,
                    )?;
                }
            }
            if message.header.kind == 14 {
                return Err(Error::Contract("peer error"));
            }
            // Progress, engine notifications, and replies abandoned by an earlier call do not acquire this request's identity.
        }
    }
    fn expect_setup(
        &mut self,
        kind: u16,
        sequence: u32,
        deadline: Deadline,
    ) -> Result<Message, Error> {
        loop {
            let message = self.receive_setup(deadline)?;
            if message.header.kind == kind && message.header.sequence == sequence {
                return Ok(message);
            }
            if message.header.kind == 14 {
                return Err(Error::Contract("preparation peer error"));
            }
        }
    }
    fn connect_inner(&mut self) -> Result<Identity, Error> {
        if self.app.is_some() {
            return self.connect_app();
        }
        self.connect_legacy()
    }
    fn connect_app(&mut self) -> Result<Identity, Error> {
        if self.dead {
            return Err(Error::Closed);
        }
        let deadline = Deadline::after(self.hello_timeout)?;
        let mut nonce = [0; 4];
        File::open("/dev/urandom")?.read_exact(&mut nonce)?;
        let nonce: String = nonce.iter().map(|v| format!("{v:02x}")).collect();
        self.client_nonce = Some(nonce.clone());
        let (payload, pick) = self.hello_payload(&nonce)?;
        let seq = self.send(1, &payload, deadline)?;
        let mut hello: Value = serde_json::from_slice(&self.expect(2, seq, deadline)?.payload)?;
        if hello["protocol"].as_u64() != Some(self.transport.version() as u64) {
            return Err(Error::Contract(
                "HELLO selected protocol differs from transport",
            ));
        }
        self.report("loading", "hello", Some(&hello), None);
        self.phase_enabled = hello["carrot_host"] == "jetson";
        self.hello_complete
            .store(true, std::sync::atomic::Ordering::Release);
        if let Some(check) = &self.hello_check {
            check(&hello)?;
        }
        if hello["carrot_wifi_v1"] == true {
            self.wifi = Some(crate::wifi::Publisher::spawn()?);
            self.provision_wifi(true)?;
        }
        self.boot_update(&hello)?;
        let path = if self.transport.gadget() {
            "usb"
        } else {
            "tcp"
        };
        let follows_app = crate::peer::classify(&hello, path).is_some()
            && self.transport.version() == wire::Version::V3;
        if self.transport.version() == wire::Version::V3
            && !follows_app
            && hello.get("loaded").is_none()
        {
            return Err(Error::Contract(
                "select and prepare a driving model in the App",
            ));
        }
        // The App answers with the newer of its pick and ours; when that is
        // ours, ask for it whatever the App has loaded (hot swap), else
        // follow the App's own choice.
        let ours = pick.filter(|pick| follows_app && pick.won(&hello));
        if follows_app && ours.is_none() {
            let until = Deadline::after(Duration::from_secs(300))?;
            while hello["loaded"].is_null() {
                self.report("loading", "app-selection", None, None);
                until.remaining()?;
                self.require_setup()?;
                let deadline = Deadline::after(Duration::from_secs(2))?;
                let seq = self.send(12, b"{}", deadline)?;
                let state: Value =
                    serde_json::from_slice(&self.expect(13, seq, deadline)?.payload)?;
                let loaded = state
                    .get("loaded")
                    .ok_or(Error::Contract("App loaded state missing"))?
                    .clone();
                hello["loaded"] = loaded;
                if hello["loaded"].is_null() {
                    std::thread::sleep(Duration::from_secs(1));
                }
            }
        }
        let app = self
            .app
            .as_ref()
            .ok_or(Error::Contract("App selection missing"))?;
        crate::selection::approved(&app.cache)?;
        let fallback =
            crate::model_spec::ModelSpec::parse(crate::model_spec::LEGACY_JSON.as_bytes())?;
        let selected = if let Some(pick) = &ours {
            pick.spec.sha256.clone()
        } else if follows_app {
            hello["loaded"]
                .as_str()
                .ok_or(Error::Contract("App loaded model identity"))?
                .to_owned()
        } else {
            fallback.sha256.clone()
        };
        if !valid_sha(&selected) {
            return Err(Error::Contract("App loaded model identity"));
        }
        let preloaded = hello["loaded"] == selected;
        if !preloaded {
            self.require_setup()?;
        }
        if follows_app && ours.is_none() {
            let deadline = Deadline::after(Duration::from_secs(2))?;
            let sequence = self.send(12, b"{}", deadline)?;
            let current: Value =
                serde_json::from_slice(&self.expect_setup(13, sequence, deadline)?.payload)?;
            if current.get("loaded") != Some(&json!(selected)) {
                return Err(Error::Contract("App engine changed before approval"));
            }
        }
        let bytes = if let Some(pick) = &ours {
            pick.spec.nbytes
        } else if selected == fallback.sha256 {
            fallback.nbytes
        } else {
            0
        };
        let deadline = Deadline::after(Duration::from_secs(60))?;
        let seq = self.send(
            3,
            &serde_json::to_vec(&json!({"sha256":selected,"nbytes":bytes,"frame_skip":4}))?,
            deadline,
        )?;
        let mut state: Value =
            serde_json::from_slice(&self.expect_setup(4, seq, deadline)?.payload)?;
        if state["state"] == "need_upload" {
            if let Some(pick) = &ours {
                state = match self.upload_picked_model(pick) {
                    // A cable pulled mid-way is not the pick's fault.
                    Err(Error::Closed) => return Err(Error::Closed),
                    Err(error) => {
                        if let Some(app) = &self.app {
                            crate::app_pick::failed(&app.cache, pick);
                        }
                        return Err(error);
                    }
                    Ok(state) => state,
                };
            } else if !follows_app {
                state = self.prepare_legacy_model(&fallback)?;
            }
        }
        let deadline = Deadline::after(Duration::from_secs(900))?;
        loop {
            match state["state"].as_str() {
                Some("ready") => break,
                Some("failed" | "need_upload") => {
                    if let (Some(pick), Some(app)) = (&ours, &self.app) {
                        crate::app_pick::failed(&app.cache, pick);
                    }
                    return Err(Error::Contract("App engine unavailable"));
                }
                _ => {}
            }
            self.require_setup()?;
            let message = self.receive_setup(deadline)?;
            if message.header.kind == 14 {
                return Err(Error::Contract("App engine error"));
            }
            if message.header.kind == 4 {
                let candidate: Value = serde_json::from_slice(&message.payload)?;
                if candidate["sha256"].is_null() || candidate["sha256"] == selected {
                    state = candidate;
                }
            }
        }
        self.report("loading", "engine", None, Some(&state));
        let session = crate::session_contract::SessionContract::parse(
            state["spec"].clone(),
            self.transport.version(),
        )?;
        if session.spec.sha256 != selected || bytes != 0 && session.spec.nbytes != bytes {
            return Err(Error::Contract("App engine identity/extent changed"));
        }
        if selected == fallback.sha256 && session.spec != fallback {
            return Err(Error::Contract("source Cinque v2 model contract changed"));
        }
        if ours.is_some() {
            // Loaded now at our request; the hello still names the old one.
            hello["loaded"] = json!(selected);
        }
        if follows_app {
            crate::selection::check_loaded(&session.spec, &hello)?;
        }
        let app = self
            .app
            .as_ref()
            .ok_or(Error::Contract("App selection missing"))?;
        crate::selection::remember(&app.cache, &session.spec)?;
        let mut identity = Identity::new();
        for key in [
            "device_model",
            "android_api",
            "backend_requested",
            "app_version",
        ] {
            let value = &hello["telemetry"][key];
            let fallback = match key {
                "device_model" => &hello["device"],
                "backend_requested" => &hello["backend"],
                _ => &Value::Null,
            };
            let value = if value.is_null() { fallback } else { value };
            let text = if value.is_null() {
                String::new()
            } else if let Some(text) = value.as_str() {
                text.to_owned()
            } else {
                value.to_string()
            };
            identity.insert(key.into(), text.chars().take(200).collect());
        }
        identity.insert("artifact_sha256".into(), selected);
        identity.insert(
            "runtime_version".into(),
            hello["runtime_version"]
                .as_str()
                .unwrap_or_default()
                .chars()
                .take(200)
                .collect(),
        );
        let packed = vec![0.; session.packed_count()];
        self.session = Some(session);
        self.ready = true;
        let warped = vec![0; contract::WARPED_BYTES];
        self.report("loading", "warmup", None, Some(&state));
        for frame in 0..10 {
            self.infer_inner(
                frame,
                &warped,
                &packed,
                Deadline::after(Duration::from_secs(3))?,
                true,
            )?;
        }
        // Held until the device is parked; see idle_inner.
        self.app_request_pending = follows_app;
        self.last_frame = None;
        self.telemetry_at = 0;
        self.idle_at = crate::now_ns().saturating_add(2_000_000_000);
        self.report("ready", "ready", None, None);
        Ok(identity)
    }
    fn connect_legacy(&mut self) -> Result<Identity, Error> {
        if self.dead {
            return Err(Error::Closed);
        }
        let mut nonce = [0; 4];
        File::open("/dev/urandom")?.read_exact(&mut nonce)?;
        let nonce: String = nonce.iter().map(|v| format!("{v:02x}")).collect();
        self.client_nonce = Some(nonce.clone());
        let deadline = Deadline::after(self.hello_timeout)?;
        let seq = self.send(
            1,
            &serde_json::to_vec(&json!({"client":{"name":"jetlinkd-rs","nonce":nonce}}))?,
            deadline,
        )?;
        let hello: Value = serde_json::from_slice(&self.expect(2, seq, deadline)?.payload)?;
        if hello["protocol"] != 2
            || hello["backend"] != "ort"
            || hello["runtime_version"] != "1.22.0"
            || hello["telemetry"]["artifact_sha256"] != contract::SHA256
            || hello["telemetry"]["source_sha256"] != contract::SHA256
        {
            return Err(Error::Contract("peer runtime/artifact"));
        }
        self.report("loading", "hello", Some(&hello), None);
        self.phase_enabled = hello["carrot_host"] == "jetson";
        self.hello_complete
            .store(true, std::sync::atomic::Ordering::Release);
        if let Some(check) = &self.hello_check {
            check(&hello)?;
        }
        let deadline = Deadline::after(Duration::from_secs(60))?;
        let seq = self.send(
            3,
            &serde_json::to_vec(
                &json!({"sha256":contract::SHA256,"nbytes":766354845,"frame_skip":4}),
            )?,
            deadline,
        )?;
        let mut state: Value = serde_json::from_slice(&self.expect(4, seq, deadline)?.payload)?;
        let deadline = Deadline::after(Duration::from_secs(300))?;
        loop {
            match state["state"].as_str() {
                Some("ready") => break,
                Some("failed" | "need_upload") => {
                    return Err(Error::Contract("peer engine unavailable"))
                }
                _ => {}
            }
            let message = self.transport.receive(deadline)?;
            match message.header.kind {
                4 => {
                    let candidate: Value = serde_json::from_slice(&message.payload)?;
                    if candidate["sha256"].is_null() || candidate["sha256"] == contract::SHA256 {
                        state = candidate;
                    }
                }
                14 => return Err(Error::Contract("peer engine error")),
                _ => {}
            }
        }
        self.report("ready", "ready", None, Some(&state));
        if state["spec"] != contract::contract()? {
            return Err(Error::Contract("peer model contract"));
        }
        self.ready = true;
        let mut identity = Identity::new();
        for key in [
            "device_model",
            "android_api",
            "backend_requested",
            "app_version",
            "artifact_sha256",
        ] {
            let value = &hello["telemetry"][key];
            let text = if value.is_null() {
                String::new()
            } else if let Some(text) = value.as_str() {
                text.to_owned()
            } else {
                value.to_string()
            };
            identity.insert(key.to_owned(), text.chars().take(200).collect());
        }
        identity.insert("runtime_version".to_owned(), "1.22.0".to_owned());
        Ok(identity)
    }
    fn infer_inner(
        &mut self,
        frame: u32,
        warped: &[u8],
        packed: &[f32],
        deadline: Deadline,
        reset: bool,
    ) -> Result<Vec<f32>, Error> {
        if !self.ready || self.dead {
            return Err(Error::Closed);
        }
        deadline.remaining()?;
        if !reset && self.last_frame.is_some_and(|previous| frame <= previous) {
            return Err(Error::Contract("frame order"));
        }
        self.session_contract().check_input(warped, packed)?;
        if self.sequence >= 0xffff_fffe {
            return Err(Error::Contract("sequence exhausted; reconnect"));
        }
        let mut payload = Vec::with_capacity(8 + warped.len() + packed.len() * 4);
        payload.extend(frame.to_le_bytes());
        let want_state = self.app.is_some() && crate::now_ns() >= self.telemetry_at;
        if want_state {
            self.telemetry_at = crate::now_ns().saturating_add(1_000_000_000);
        }
        let map = if self.session_contract().spec.navigation() {
            if reset {
                self.map_sent = None; // RESET_QUEUES clears the server's map
            }
            self.next_map()
        } else {
            None
        };
        // A stateful graph's hidden_state never feeds back from the comma: ask
        // the server to leave it out (64 KB a frame) and zero-fill it below.
        let omitted = self.omitted_hidden();
        let flags = u32::from(reset)
            | if self.transport.version() == wire::Version::V3 && omitted.is_none() {
                4
            } else {
                0
            }
            | if want_state { 2 } else { 0 }
            | if map.is_some() { NAV_MAP_FLAG } else { 0 };
        payload.extend(flags.to_le_bytes());
        payload.extend(warped);
        for value in packed {
            payload.extend(value.to_le_bytes());
        }
        if let Some(map) = map {
            payload.extend_from_slice(&map);
        }
        let seq = self.send(8, &payload, deadline)?;
        if self.phase_enabled {
            self.phase.sent(self.source_sof);
        }
        let response = self.expect(9, seq, deadline)?;
        let full = self.session_contract().spec.output_count()?;
        let omitted_len = omitted.map_or(0, |[start, stop]| stop - start);
        let end = 20 + (full - omitted_len) * 4;
        if response.payload.len() < 20 {
            return Err(Error::Contract("inference response is missing its header"));
        }
        // Report which check failed, as the source client does: one merged
        // message hid why a phone refused every real frame while driving.
        let (answered, status) = (
            wire::word(&response.payload, 0),
            wire::word(&response.payload, 4),
        );
        if status != 0 {
            let name = match status {
                1 => "NOT_READY",
                2 => "BAD_SHAPE",
                3 => "INFER_FAILED",
                4 => "NOT_FINITE",
                _ => "unknown status",
            };
            return Err(Error::Peer(format!(
                "inference failed: {name} ({status}) for frame {answered}"
            )));
        }
        if answered != frame {
            return Err(Error::Peer(format!(
                "inference response frame {answered}, expected {frame}"
            )));
        }
        if response.payload.len() < end {
            return Err(Error::Peer(format!(
                "inference response is {} bytes, expected {end}",
                response.payload.len()
            )));
        }
        if let Ok(mut status) = self.status.lock() {
            status["timings"] = json!([
                wire::word(&response.payload, 8),
                wire::word(&response.payload, 12),
                wire::word(&response.payload, 16)
            ]);
        }
        if response.payload.len() > end {
            if !want_state {
                return Err(Error::Contract("unexpected inference telemetry"));
            }
            let state: Value = serde_json::from_slice(&response.payload[end..])?;
            if !state.is_object() {
                return Err(Error::Contract("inference telemetry object"));
            }
            crate::selection::check_loaded(&self.session_contract().spec, &state)?;
            self.report("ready", "ready", None, Some(&state));
        }
        let output = rpc::decode_floats(&response.payload[20..end])?;
        let output = match omitted {
            Some([start, stop]) => {
                let mut whole = Vec::with_capacity(full);
                whole.extend_from_slice(&output[..start]);
                whole.resize(stop, 0.);
                whole.extend_from_slice(&output[start..]);
                whole
            }
            None => output,
        };
        self.session_contract().check_output(&output)?;
        deadline.remaining()?;
        self.last_frame = Some(frame);
        Ok(output)
    }
}
impl<T: Transport> Backend for Client<T> {
    fn after_reply(&mut self) -> Result<(), Error> {
        self.provision_wifi(false)
    }
    fn idle(&mut self) -> Result<(), Error> {
        Client::idle(self)
    }
    fn session_start(&mut self) -> Result<(), Error> {
        Client::session_start(self)
    }
    fn source_sof(&mut self, source_sof: u64) {
        self.set_source_sof(source_sof);
    }
    fn last_timings(&self) -> Option<[f64; 3]> {
        // The peer reports microseconds; the status file keeps them as sent.
        let status = self.status.lock().ok()?;
        let timings = status.get("timings")?.as_array()?;
        Some([
            timings.first()?.as_f64()? / 1000.0,
            timings.get(1)?.as_f64()? / 1000.0,
            timings.get(2)?.as_f64()? / 1000.0,
        ])
    }
    fn session_contract(&self) -> &crate::session_contract::SessionContract {
        self.session
            .as_ref()
            .unwrap_or_else(|| crate::session_contract::SessionContract::native_default())
    }
    fn connect(&mut self) -> Result<Identity, Error> {
        let result = self.connect_inner();
        if result.is_err() {
            self.close();
        }
        result
    }
    fn infer(
        &mut self,
        frame: u32,
        warped: &[u8],
        packed: &[f32],
        deadline: Deadline,
        reset: bool,
    ) -> Result<Vec<f32>, Error> {
        let result = self.infer_inner(frame, warped, packed, deadline, reset);
        if result.is_err() {
            self.close();
        }
        result
    }
    fn dead(&self) -> bool {
        self.dead
    }
    fn close(&mut self) {
        if !self.dead {
            self.dead = true;
            self.wifi = None;
            self.transport.close();
        }
    }
}

fn valid_sha(value: &str) -> bool {
    value.len() == 64
        && value
            .bytes()
            .all(|v| v.is_ascii_digit() || (b'a'..=b'f').contains(&v))
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::VecDeque;
    struct WireFixture {
        replies: VecDeque<Message>,
        sent: usize,
        closed: bool,
    }
    impl Transport for WireFixture {
        fn send(&mut self, _: &[u8], _: Deadline) -> Result<(), Error> {
            self.sent += 1;
            Ok(())
        }
        fn receive(&mut self, _: Deadline) -> Result<Message, Error> {
            self.replies.pop_front().ok_or(Error::Closed)
        }
        fn close(&mut self) {
            self.closed = true;
        }
    }
    fn reply(sequence: u32, frame: u32, status: u32, values: &[f32]) -> Message {
        let mut payload = Vec::new();
        for value in [frame, status, 0, 0, 0] {
            payload.extend(value.to_le_bytes());
        }
        for value in values {
            payload.extend(value.to_le_bytes());
        }
        Message {
            header: Header {
                kind: 9,
                sequence,
                flags: 0,
                length: u32::try_from(payload.len()).unwrap(),
                reserved: 0,
            },
            payload,
        }
    }
    fn client(replies: Vec<Message>) -> Client<WireFixture> {
        let mut client = Client::new(WireFixture {
            replies: replies.into(),
            sent: 0,
            closed: false,
        });
        client.ready = true;
        client
    }
    #[test]
    fn sequence_exhaustion_closes_before_sending() {
        // Given the final safe wire sequence has already been used.
        let mut client = client(vec![]);
        client.sequence = 0xffff_fffe;
        // When a new frame is requested, then no sequence can wrap and reuse an identity.
        assert!(client
            .infer(
                42,
                &vec![0; contract::WARPED_BYTES],
                &[0.0; 12],
                Deadline::after(Duration::from_millis(50)).unwrap(),
                false
            )
            .is_err());
        assert!(client.dead());
        assert_eq!(client.transport.sent, 0);
    }
    #[test]
    fn reset_explicitly_allows_frame_restart_and_stale_sequence_is_discarded() {
        // Given an earlier frame and an unrelated old reply ahead of this result.
        let values = vec![0.25; contract::OUTPUT_FLOATS];
        let mut client = client(vec![reply(99, 42, 0, &values), reply(1, 42, 0, &values)]);
        client.last_frame = Some(100);
        // When a reset is explicit, then only the new request's sequence can supply output.
        assert_eq!(
            client
                .infer(
                    42,
                    &vec![0; contract::WARPED_BYTES],
                    &[0.0; 12],
                    Deadline::after(Duration::from_millis(50)).unwrap(),
                    true
                )
                .unwrap(),
            values
        );
        assert_eq!(client.last_frame, Some(42));
    }
    #[test]
    fn nonfinite_input_is_rejected_before_usb_send() {
        // Given a nonfinite scalar that otherwise has valid dimensions.
        let mut client = client(vec![]);
        let mut packed = [0.0; 12];
        packed[9] = f32::NAN;
        // When inference crosses the wire boundary, then the session dies without transmitting it.
        assert!(client
            .infer(
                42,
                &vec![0; contract::WARPED_BYTES],
                &packed,
                Deadline::after(Duration::from_millis(50)).unwrap(),
                true
            )
            .is_err());
        assert!(client.dead());
        assert_eq!(client.transport.sent, 0);
    }
    #[test]
    fn invalid_outputs_close_without_advancing_last_frame() {
        for (frame, status, count, value) in [
            (41, 0, contract::OUTPUT_FLOATS, 0.0),
            (42, 4, contract::OUTPUT_FLOATS, 0.0),
            (42, 0, 10, 0.0),
            (42, 0, contract::OUTPUT_FLOATS, f32::INFINITY),
        ] {
            // Given a mismatched, failed, truncated, or nonfinite peer response.
            let mut client = client(vec![reply(1, frame, status, &vec![value; count])]);
            // When this frame completes, then none of those responses gains a valid frame identity.
            assert!(client
                .infer(
                    42,
                    &vec![0; contract::WARPED_BYTES],
                    &[0.0; 12],
                    Deadline::after(Duration::from_millis(50)).unwrap(),
                    true
                )
                .is_err());
            assert!(client.dead());
            assert_eq!(client.last_frame, None);
        }
    }
    struct AppWire {
        replies: VecDeque<Message>,
        sent: Vec<Vec<u8>>,
        closed: bool,
    }
    impl Transport for AppWire {
        fn version(&self) -> wire::Version {
            wire::Version::V3
        }
        fn send(&mut self, bytes: &[u8], _: Deadline) -> Result<(), Error> {
            self.sent.push(bytes.to_vec());
            Ok(())
        }
        fn receive(&mut self, _: Deadline) -> Result<Message, Error> {
            self.replies.pop_front().ok_or(Error::Closed)
        }
        fn close(&mut self) {
            self.closed = true;
        }
    }
    fn json_reply(kind: u16, sequence: u32, value: Value) -> Message {
        let payload = serde_json::to_vec(&value).unwrap();
        Message {
            header: Header {
                kind,
                sequence,
                flags: 0,
                length: payload.len() as u32,
                reserved: 0,
            },
            payload,
        }
    }
    fn app_wire(spec: &Value, version: u16) -> AppWire {
        let hello = json!({"protocol":version,"backend":"ort","device":"coreml-Apple M4","runtime_version":"1.22.0","loaded":spec["sha256"],"telemetry":{"device_model":"Mac","android_api":"none","backend_requested":"coreml","app_version":"test"}});
        let mut transport = AppWire {
            replies: vec![
                json_reply(2, 1, hello.clone()),
                json_reply(13, 2, json!({"loaded":spec["sha256"]})),
                json_reply(
                    4,
                    3,
                    json!({"state":"ready","sha256":spec["sha256"],"spec":spec}),
                ),
            ]
            .into(),
            sent: Vec::new(),
            closed: false,
        };
        let parsed =
            crate::model_spec::ModelSpec::parse(&serde_json::to_vec(spec).unwrap()).unwrap();
        for frame in 0..10 {
            transport.replies.push_back(reply(
                4 + frame,
                frame,
                0,
                &vec![0.; parsed.output_count().unwrap()],
            ));
        }
        transport.replies.push_back(json_reply(2, 14, hello));
        transport
    }
    #[test]
    fn active_v3_app_contract_controls_real_wire_and_owned_ipc() {
        let cache = tempfile::tempdir().unwrap();
        let spec: Value = serde_json::from_str(crate::model_spec::LEGACY_JSON).unwrap();
        let count = crate::model_spec::ModelSpec::parse(crate::model_spec::LEGACY_JSON.as_bytes())
            .unwrap()
            .output_count()
            .unwrap();
        let mut transport = app_wire(&spec, 3);
        let mut output = reply(15, 7, 0, &vec![0.; count]);
        output
            .payload
            .extend(serde_json::to_vec(&json!({"loaded":spec["sha256"]})).unwrap());
        output.header.length = output.payload.len() as u32;
        transport.replies.push_back(output);
        let mut client = Client::app(transport, cache.path().to_owned());
        let identity = client.connect().unwrap();
        client.release_app_request().unwrap();
        assert_eq!(identity["artifact_sha256"], spec["sha256"]);
        assert_eq!(client.session_contract().packed_count(), 12);
        let hellos: Vec<Value> = client
            .transport
            .sent
            .iter()
            .filter_map(|bytes| {
                let header =
                    Header::decode_version(bytes[..32].try_into().unwrap(), wire::Version::V3)
                        .unwrap();
                if header.kind == 1 {
                    Some(serde_json::from_slice(&bytes[32..32 + header.length as usize]).unwrap())
                } else {
                    None
                }
            })
            .collect();
        assert_eq!(hellos.len(), 2);
        assert_eq!(hellos[0]["client"]["nonce"], hellos[1]["client"]["nonce"]);
        let output = client
            .infer(
                7,
                &vec![0; contract::WARPED_BYTES],
                &[0.; 12],
                Deadline::after(Duration::from_millis(50)).unwrap(),
                true,
            )
            .unwrap();
        assert_eq!(output.len(), count);
        let bytes = &client.transport.sent[14];
        let header =
            Header::decode_version(bytes[..32].try_into().unwrap(), wire::Version::V3).unwrap();
        assert_eq!(header.kind, 8);
        assert_eq!(wire::word(bytes, 36), 7);
        assert_eq!(header.length as usize, 8 + contract::WARPED_BYTES + 12 * 4);
        assert_eq!(
            crate::selection::approved(cache.path())
                .unwrap()
                .unwrap()
                .sha256,
            identity["artifact_sha256"]
        );
    }
    #[test]
    fn dynamic_app_session_crosses_real_owner_proxy_socket() {
        let cache = tempfile::tempdir().unwrap();
        let spec: Value = serde_json::from_str(crate::model_spec::LEGACY_JSON).unwrap();
        let parsed =
            crate::model_spec::ModelSpec::parse(crate::model_spec::LEGACY_JSON.as_bytes()).unwrap();
        let mut transport = app_wire(&spec, 3);
        transport.replies.pop_back();
        transport
            .replies
            .push_back(reply(14, 9, 0, &vec![0.; parsed.output_count().unwrap()]));
        let client = Client::app(transport, cache.path().to_owned());
        let directory = tempfile::tempdir().unwrap();
        let path = directory.path().join("owner.sock");
        let mut server = crate::owner::Server::spawn(client, &path).unwrap();
        let until = std::time::Instant::now() + Duration::from_secs(2);
        while !server.ready.load(std::sync::atomic::Ordering::Acquire) {
            assert!(
                std::time::Instant::now() < until,
                "{}",
                server.error.lock().unwrap()
            );
            std::thread::sleep(Duration::from_millis(1));
        }
        let mut proxy = crate::rpc::ProxyClient::connect(&path).unwrap();
        assert_eq!(proxy.session.spec, parsed);
        assert_eq!(proxy.session.version, wire::Version::V3);
        assert_eq!(
            proxy
                .infer(
                    9,
                    &vec![0; contract::WARPED_BYTES],
                    &[0.; 12],
                    Deadline::after(Duration::from_millis(50)).unwrap(),
                    true
                )
                .unwrap()
                .len(),
            parsed.output_count().unwrap()
        );
        proxy.close();
        server.stop(Duration::from_secs(2)).unwrap();
    }
    #[test]
    fn idle_state_updates_fresh_health_and_closes_on_app_unload() {
        let cache = tempfile::tempdir().unwrap();
        let spec: Value = serde_json::from_str(crate::model_spec::LEGACY_JSON).unwrap();
        let mut transport = app_wire(&spec, 3);
        transport.replies.push_back(json_reply(13, 15, json!({"loaded":spec["sha256"],"carrot_health":{"age_s":0.,"severity":"ok","temp_c":42.,"addresses":[{"address":"192.168.60.2"}]}})));
        transport
            .replies
            .push_back(json_reply(13, 16, json!({"loaded":null})));
        let mut client = Client::app(transport, cache.path().to_owned());
        client.connect().unwrap();
        client.release_app_request().unwrap();
        client.idle_at = 0;
        client.idle().unwrap();
        let snapshot = client.status.lock().unwrap().clone();
        assert_eq!(snapshot["telemetry"]["carrot_health"]["temp_c"], 42.);
        assert_eq!(snapshot["peer"]["device"], "coreml-Apple M4");
        assert!(snapshot["telemetry_updated"].as_f64().unwrap() > 0.);
        client.idle_at = 0;
        assert!(client.idle().is_err());
        assert!(client.transport.closed);
    }
    #[test]
    fn physical_hello_rejection_closes_before_engine_or_approval() {
        let cache = tempfile::tempdir().unwrap();
        let spec: Value = serde_json::from_str(crate::model_spec::LEGACY_JSON).unwrap();
        let mut client = Client::app(app_wire(&spec, 3), cache.path().to_owned());
        client.set_hello_check(|_| Err(Error::Contract("unsupported physical cable")));
        assert!(client.connect().is_err());
        assert!(client.transport.closed);
        assert_eq!(client.transport.sent.len(), 1);
        assert!(client
            .hello_complete
            .load(std::sync::atomic::Ordering::Acquire));
        assert!(crate::selection::approved(cache.path()).unwrap().is_none());
    }
    #[test]
    fn v3_hello_mismatch_closes_before_protocol_can_change() {
        let cache = tempfile::tempdir().unwrap();
        let spec: Value = serde_json::from_str(crate::model_spec::LEGACY_JSON).unwrap();
        let mut client = Client::app(app_wire(&spec, 2), cache.path().to_owned());
        assert!(client.connect().is_err());
        assert!(client.transport.closed);
        assert!(!client
            .hello_complete
            .load(std::sync::atomic::Ordering::Acquire));
    }
    #[test]
    fn app_current_pick_is_checked_before_engine_and_known_model_keeps_exact_contract() {
        let cache = tempfile::tempdir().unwrap();
        let spec: Value = serde_json::from_str(crate::model_spec::LEGACY_JSON).unwrap();
        let mut transport = app_wire(&spec, 3);
        transport.replies[1] = json_reply(13, 2, json!({"loaded":"b".repeat(64)}));
        let mut client = Client::app(transport, cache.path().to_owned());
        assert!(client.connect().is_err());
        assert_eq!(client.transport.sent.len(), 2);
        assert!(crate::selection::approved(cache.path()).unwrap().is_none());
        let mut changed = spec;
        changed["checkpoint"] = json!("different-checkpoint");
        let mut transport = app_wire(
            &serde_json::from_str(crate::model_spec::LEGACY_JSON).unwrap(),
            3,
        );
        transport.replies[2] = json_reply(
            4,
            3,
            json!({"state":"ready","sha256":changed["sha256"],"spec":changed}),
        );
        let mut client = Client::app(transport, cache.path().to_owned());
        assert!(client.connect().is_err());
        assert_eq!(client.transport.sent.len(), 3);
        assert!(crate::selection::approved(cache.path()).unwrap().is_none());
    }
    #[test]
    fn comma_pick_that_wins_is_requested_downloaded_and_uploaded() {
        let cache = tempfile::tempdir().unwrap();
        let data = b"carrot-n16-onnx-fixture";
        let mut spec =
            crate::model_spec::ModelSpec::parse(include_bytes!("../data/n16.json")).unwrap();
        {
            use sha2::Digest;
            spec.sha256 = format!("{:x}", sha2::Sha256::digest(data));
        }
        spec.nbytes = data.len() as u64;
        // A verified copy is cached already, so no network is touched.
        std::fs::write(cache.path().join(format!("{}.onnx", spec.sha256)), data).unwrap();
        let pick = crate::app_pick::AppPick {
            model: "n16".into(),
            name: "N16 Nav".into(),
            url: "https://invalid.example/big_driving_supercombo.onnx".into(),
            at: 1_700_000_000.,
            spec: spec.clone(),
        };
        crate::app_pick::write(cache.path(), &pick).unwrap();
        let legacy: Value = serde_json::from_str(crate::model_spec::LEGACY_JSON).unwrap();
        // The App runs Cinque v2 and answers with our pick as the newer one.
        let hello = json!({"protocol":3,"backend":"ort","device":"coreml-Apple M4","runtime_version":"1.22.0","loaded":legacy["sha256"],"pick":{"sha256":spec.sha256,"by":"comma"},"telemetry":{"device_model":"Mac","android_api":"none","backend_requested":"coreml","app_version":"test"}});
        let wire_spec = serde_json::to_value(&spec).unwrap();
        let mut replies: VecDeque<Message> = vec![
            json_reply(2, 1, hello),
            json_reply(4, 2, json!({"state":"need_upload","sha256":spec.sha256})),
            json_reply(
                4,
                3,
                json!({"state":"need_upload","sha256":spec.sha256,"chunk":8}),
            ),
            // chunks go out as 4, 5, 6 with no answer; UPLOAD_DONE is 7
            json_reply(
                4,
                7,
                json!({"state":"ready","sha256":spec.sha256,"spec":wire_spec}),
            ),
        ]
        .into();
        let mut count = spec.output_count().unwrap();
        if spec.stateful() {
            let [start, end] = spec.range("hidden_state").unwrap();
            count -= end - start;
        }
        for frame in 0..10 {
            replies.push_back(reply(8 + frame, frame, 0, &vec![0.; count]));
        }
        let mut client = Client::app(
            AppWire {
                replies,
                sent: Vec::new(),
                closed: false,
            },
            cache.path().to_owned(),
        );
        client.app.as_mut().unwrap().parked = std::sync::Arc::new(|| true);
        let identity = client.connect().unwrap();
        assert_eq!(identity["artifact_sha256"], spec.sha256);
        let sent: Vec<(Header, Vec<u8>)> = client
            .transport
            .sent
            .iter()
            .map(|bytes| {
                let header =
                    Header::decode_version(bytes[..32].try_into().unwrap(), wire::Version::V3)
                        .unwrap();
                let length = header.length as usize;
                (header, bytes[32..32 + length].to_vec())
            })
            .collect();
        let first: Value = serde_json::from_slice(&sent[0].1).unwrap();
        assert_eq!(first["pick"]["ref"], "n16");
        assert_eq!(first["pick"]["sha256"], spec.sha256);
        let request: Value = serde_json::from_slice(&sent[1].1).unwrap();
        assert_eq!(sent[1].0.kind, 3);
        assert_eq!(request["sha256"], spec.sha256);
        assert_eq!(request["nbytes"], spec.nbytes);
        let uploaded: Vec<u8> = sent
            .iter()
            .filter(|(header, _)| header.kind == 5)
            .flat_map(|(_, payload)| payload[8..].to_vec())
            .collect();
        assert_eq!(uploaded, data);
        // The App keeps its own copy now.
        assert!(!cache.path().join(format!("{}.onnx", spec.sha256)).exists());
        assert_eq!(
            crate::selection::approved(cache.path())
                .unwrap()
                .unwrap()
                .sha256,
            spec.sha256
        );
    }
    #[test]
    fn app_request_release_checks_fresh_loaded_identity() {
        let cache = tempfile::tempdir().unwrap();
        let spec: Value = serde_json::from_str(crate::model_spec::LEGACY_JSON).unwrap();
        let parsed =
            crate::model_spec::ModelSpec::parse(crate::model_spec::LEGACY_JSON.as_bytes()).unwrap();
        crate::selection::remember(cache.path(), &parsed).unwrap();
        let mut transport = app_wire(&spec, 3);
        transport
            .replies
            .push_back(json_reply(13, 15, json!({"loaded":spec["sha256"]})));
        let mut client = Client::app(transport, cache.path().to_owned());
        client.connect().unwrap();
        client.release_app_request().unwrap();
        assert!(!client.app_request_pending);
        assert_eq!(client.transport.sent.len(), 14);
        client.idle().unwrap();
        assert_eq!(client.transport.sent.len(), 14);
        client.idle_at = 0;
        client.idle().unwrap();
        assert!(!client.app_request_pending);
        assert_eq!(client.transport.sent.len(), 15);
        let mut transport = app_wire(&spec, 3);
        transport.replies.pop_back();
        transport.replies.push_back(json_reply(
            2,
            14,
            json!({"protocol":3,"loaded":"b".repeat(64)}),
        ));
        let mut client = Client::app(transport, cache.path().to_owned());
        client.connect().unwrap();
        assert!(client.app_request_pending);
        // Parked: giving the request back finds another model loaded and closes.
        client.app.as_mut().unwrap().parked = std::sync::Arc::new(|| true);
        client.idle_at = 0;
        assert!(client.idle().is_err());
        assert!(client.dead());
        assert_eq!(
            crate::selection::approved(cache.path()).unwrap().unwrap(),
            parsed
        );
    }
    #[test]
    fn engine_request_is_kept_while_driving_and_made_again_after_parking() {
        let cache = tempfile::tempdir().unwrap();
        let spec: Value = serde_json::from_str(crate::model_spec::LEGACY_JSON).unwrap();
        let mut transport = app_wire(&spec, 3);
        let release = transport.replies.pop_back().unwrap();
        // Driving idle: only the state poll (sequence 14).
        transport
            .replies
            .push_back(json_reply(13, 14, json!({"loaded":spec["sha256"]})));
        // Parked idle: release HELLO (15), then the state poll (16).
        transport.replies.push_back(json_reply(
            2,
            15,
            serde_json::from_slice(&release.payload).unwrap(),
        ));
        transport
            .replies
            .push_back(json_reply(13, 16, json!({"loaded":spec["sha256"]})));
        // Next modeld session: state poll (17) and the engine request (18).
        transport
            .replies
            .push_back(json_reply(13, 17, json!({"loaded":spec["sha256"]})));
        transport.replies.push_back(json_reply(
            4,
            18,
            json!({"state":"ready","sha256":spec["sha256"],"spec":spec}),
        ));
        let mut client = Client::app(transport, cache.path().to_owned());
        client.connect().unwrap();
        assert!(client.app_request_pending);
        let connected = client.transport.sent.len();

        client.app.as_mut().unwrap().parked = std::sync::Arc::new(|| false);
        client.idle_at = 0;
        client.idle().unwrap();
        assert!(client.app_request_pending);
        assert_eq!(client.transport.sent.len(), connected + 1);
        client.session_start().unwrap();
        assert_eq!(client.transport.sent.len(), connected + 1);

        client.app.as_mut().unwrap().parked = std::sync::Arc::new(|| true);
        client.idle_at = 0;
        client.idle().unwrap();
        assert!(!client.app_request_pending);
        assert_eq!(client.transport.sent.len(), connected + 3);

        client.session_start().unwrap();
        assert!(client.app_request_pending);
        assert_eq!(client.transport.sent.len(), connected + 5);
        client.session_start().unwrap();
        assert_eq!(client.transport.sent.len(), connected + 5);
    }
    #[test]
    fn android_first_connection_waits_for_app_selection_and_engine_preparation() {
        let cache = tempfile::tempdir().unwrap();
        let spec: Value = serde_json::from_str(crate::model_spec::LEGACY_JSON).unwrap();
        let mut wire = app_wire(&spec, 3);
        for reply in wire.replies.iter_mut().skip(2) {
            reply.header.sequence += 1;
        }
        let mut hello: Value = serde_json::from_slice(&wire.replies[0].payload).unwrap();
        hello["device"] = json!("htp-SM8550");
        hello["telemetry"]["android_api"] = json!(35);
        *wire.replies.back_mut().unwrap() = json_reply(2, 15, hello.clone());
        hello["loaded"] = Value::Null;
        wire.replies[0] = json_reply(2, 1, hello);
        wire.replies
            .insert(2, json_reply(13, 3, json!({"loaded":spec["sha256"]})));
        wire.replies.insert(
            3,
            json_reply(4, 4, json!({"state":"loading","sha256":spec["sha256"]})),
        );
        let mut client = Client::app(wire, cache.path().to_owned());
        let identity = client.connect().unwrap();
        client.release_app_request().unwrap();
        assert_eq!(identity["artifact_sha256"], spec["sha256"]);
        assert!(!client.app_request_pending);
        assert_eq!(client.transport.sent.len(), 15);
        assert_eq!(
            crate::selection::approved(cache.path()).unwrap().unwrap(),
            client.session_contract().spec
        );
    }
    #[test]
    fn app_model_selection_onroad_validates_and_persists_new_session() {
        let cache = tempfile::tempdir().unwrap();
        let approved =
            crate::model_spec::ModelSpec::parse(crate::model_spec::LEGACY_JSON.as_bytes()).unwrap();
        crate::selection::remember(cache.path(), &approved).unwrap();
        let mut next = serde_json::to_value(&approved).unwrap();
        next["sha256"] = json!("b".repeat(64));
        let mut client = Client::app(app_wire(&next, 3), cache.path().to_owned());
        client.connect().unwrap();
        client.release_app_request().unwrap();
        assert_eq!(client.session_contract().spec.sha256, "b".repeat(64));
        assert_eq!(
            crate::selection::approved(cache.path()).unwrap().unwrap(),
            crate::model_spec::ModelSpec::parse(&serde_json::to_vec(&next).unwrap()).unwrap()
        );
    }
}
