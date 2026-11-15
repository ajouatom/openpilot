//! Carrot auxiliary vision: xiaoge's camera models (lane-marking type, the
//! visual blind spot) run on the Jetlink App's device instead of the comma's
//! little cores.
//!
//! xiaoge sends one prepared float32 input tensor over a local socket
//! ([`SOCKET`]) and gets the model's float32 outputs back, which it decodes
//! exactly as when the model runs on the comma. jetlinkd puts the job on the
//! App link it already holds for the driving model: at most one auxiliary
//! message right after each driving reply ([`Hub::next_outbound`] from
//! `Client::after_reply`), so a driving frame never waits behind one. The App
//! answers whenever it is done; the driving client hands those messages here
//! ([`Hub::deliver`]) while it waits for its own reply.
//!
//! Only an App whose hello reply says [`CAPABILITY`] is sent these messages:
//! any other answers an unknown type with an error, which ends the session.
//! Without a live link every request fails at once and xiaoge runs the model
//! on the comma's CPU, as before.
use crate::Error;
use serde_json::{json, Value};
use sha2::{Digest, Sha256};
use std::{
    collections::{HashMap, VecDeque},
    fs::File,
    io::{Read, Seek, SeekFrom, Write},
    os::unix::{
        fs::PermissionsExt,
        net::{UnixListener, UnixStream},
    },
    path::{Path, PathBuf},
    sync::{Arc, Condvar, Mutex, MutexGuard, OnceLock},
    thread,
    time::{Duration, Instant, SystemTime},
};

/// The App's hello key that allows the messages below.
pub const CAPABILITY: &str = "carrot_aux_v1";
/// Comma to App: a model by sha256 and size.
pub const MSG_LOAD: u16 = 0x4010;
/// App to comma: a model's state (ready, need_upload, loading, failed).
pub const MSG_STATE: u16 = 0x4011;
/// Comma to App: a piece of a model upload.
pub const MSG_CHUNK: u16 = 0x4012;
/// Comma to App: one input tensor for a ready model.
pub const MSG_REQ: u16 = 0x4013;
/// App to comma: the outputs, or why there are none.
pub const MSG_RESP: u16 = 0x4014;
/// xiaoge's local socket.
pub const SOCKET: &str = "/dev/shm/carrot-jetlink-aux.sock";
/// Upload piece: one per driving frame, about 3 ms on USB 3.
const CHUNK: usize = 1 << 20;
/// Largest local message (a 2 MB input or output and its JSON).
const MAX_LOCAL: usize = 16 << 20;
/// Largest model uploaded.
const MAX_MODEL: u64 = 64 << 20;
/// Longest a local caller may wait for one job.
const MAX_WAIT: Duration = Duration::from_secs(2);
/// Only models in xiaoge's asset directory leave the comma.
const MODEL_DIR: &str = "/selfdrive/carrot/xiaoge/assets/";
/// A model asked about or loading with no word from the App for this long
/// is asked about again: its AUX_STATE may have arrived while the client was
/// in a setup step that drops messages.
const STATE_PATIENCE: Duration = Duration::from_secs(10);

/// jetlinkd's one hub: the local socket and every App link share it.
pub fn shared() -> Arc<Hub> {
    static HUB: OnceLock<Arc<Hub>> = OnceLock::new();
    Arc::clone(HUB.get_or_init(|| Arc::new(Hub::default())))
}

pub fn is_aux(kind: u16) -> bool {
    matches!(kind, MSG_STATE | MSG_RESP)
}

/// `[u32 LE JSON length][JSON][raw bytes]`, both ways and on both links.
pub fn encode(head: &Value, blob: &[&[u8]]) -> Result<Vec<u8>, Error> {
    let json = serde_json::to_vec(head)?;
    let length = u32::try_from(json.len()).map_err(|_| Error::Contract("aux json length"))?;
    let total = blob.iter().map(|part| part.len()).sum::<usize>();
    let mut bytes = Vec::with_capacity(4 + json.len() + total);
    bytes.extend(length.to_le_bytes());
    bytes.extend(json);
    for part in blob {
        bytes.extend_from_slice(part);
    }
    Ok(bytes)
}

pub fn decode(payload: &[u8]) -> Result<(Value, &[u8]), Error> {
    let length = payload
        .get(..4)
        .map(|b| u32::from_le_bytes([b[0], b[1], b[2], b[3]]) as usize)
        .ok_or(Error::Contract("aux payload"))?;
    let json = payload
        .get(4..4 + length)
        .ok_or(Error::Contract("aux payload json"))?;
    Ok((serde_json::from_slice(json)?, &payload[4 + length..]))
}

#[derive(Clone, Debug, PartialEq)]
enum State {
    /// Not asked about on this link yet.
    Unknown,
    /// AUX_LOAD sent, its AUX_STATE not back.
    Asked,
    /// Next piece to send.
    Uploading(u64),
    /// Every piece sent, or the App is loading it.
    Loading,
    Ready,
    Failed(String),
}

struct Model {
    path: PathBuf,
    nbytes: u64,
    state: State,
    /// When `state` last changed.
    since: Instant,
    device: String,
}

/// What a job came to.
#[derive(Debug)]
pub enum Outcome {
    Done { head: Value, blob: Vec<u8> },
    Failed { code: String, detail: String },
}

struct Job {
    id: u64,
    sha256: String,
    outputs: Vec<String>,
    input: Vec<u8>,
}

#[derive(Default)]
struct Inner {
    live: bool,
    models: HashMap<String, Model>,
    /// (len, mtime) -> sha256 of a model file.
    hashes: HashMap<PathBuf, (u64, SystemTime, String)>,
    waiting: VecDeque<Job>,
    sent: HashMap<u64, Instant>,
    results: HashMap<u64, Outcome>,
    next_id: u64,
}

#[derive(Default)]
pub struct Hub {
    inner: Mutex<Inner>,
    changed: Condvar,
}

fn failed(code: &str, detail: impl Into<String>) -> Outcome {
    Outcome::Failed {
        code: code.into(),
        detail: detail.into(),
    }
}

impl Hub {
    fn lock(&self) -> MutexGuard<'_, Inner> {
        self.inner
            .lock()
            .unwrap_or_else(|poisoned| poisoned.into_inner())
    }

    /// The App link came up with [`CAPABILITY`], or went down. Every model is
    /// asked about again on the next link (the App keeps its own cache, so
    /// that costs one message), and nothing waits on a link that is gone.
    pub fn set_live(&self, live: bool) {
        let mut guard = self.lock();
        let inner = &mut *guard;
        if inner.live == live {
            return;
        }
        inner.live = live;
        for model in inner.models.values_mut() {
            model.state = State::Unknown;
            model.since = Instant::now();
        }
        if !live {
            let ids: Vec<u64> = inner
                .waiting
                .drain(..)
                .map(|job| job.id)
                .chain(inner.sent.drain().map(|(id, _)| id))
                .collect();
            for id in ids {
                inner
                    .results
                    .insert(id, failed("unavailable", "the App link closed"));
            }
        }
        drop(guard);
        self.changed.notify_all();
    }

    pub fn live(&self) -> bool {
        self.lock().live
    }

    fn sha256(&self, path: &Path) -> Result<(String, u64), Outcome> {
        let meta = std::fs::metadata(path).map_err(|e| failed("bad_model", e.to_string()))?;
        let (len, mtime) = (
            meta.len(),
            meta.modified().unwrap_or(SystemTime::UNIX_EPOCH),
        );
        if len == 0 || len > MAX_MODEL {
            return Err(failed("bad_model", format!("{len} bytes")));
        }
        if let Some((l, m, sha)) = self.lock().hashes.get(path) {
            if *l == len && *m == mtime {
                return Ok((sha.clone(), len));
            }
        }
        let mut file = File::open(path).map_err(|e| failed("bad_model", e.to_string()))?;
        let mut hasher = Sha256::new();
        let mut buffer = vec![0; CHUNK];
        loop {
            let n = file
                .read(&mut buffer)
                .map_err(|e| failed("bad_model", e.to_string()))?;
            if n == 0 {
                break;
            }
            hasher.update(&buffer[..n]);
        }
        let sha: String = hasher
            .finalize()
            .iter()
            .map(|b| format!("{b:02x}"))
            .collect();
        self.lock()
            .hashes
            .insert(path.to_owned(), (len, mtime, sha.clone()));
        Ok((sha, len))
    }

    /// One job from a local caller: blocks until the App answers, the link
    /// goes, or `timeout`.
    pub fn run(
        &self,
        model: &Path,
        outputs: Vec<String>,
        input: Vec<u8>,
        timeout: Duration,
    ) -> Outcome {
        if !self.live() {
            return failed("unavailable", "no Jetlink App with auxiliary vision");
        }
        let path = match model.canonicalize() {
            Ok(path)
                if path.extension().is_some_and(|e| e == "onnx")
                    && path.to_string_lossy().contains(MODEL_DIR) =>
            {
                path
            }
            _ => return failed("bad_model", "not one of xiaoge's models"),
        };
        let (sha, nbytes) = match self.sha256(&path) {
            Ok(found) => found,
            Err(outcome) => return outcome,
        };
        let deadline = Instant::now() + timeout.min(MAX_WAIT);
        let mut inner = self.lock();
        if !inner.live {
            return failed("unavailable", "the App link closed");
        }
        let model = inner.models.entry(sha.clone()).or_insert_with(|| Model {
            path,
            nbytes,
            state: State::Unknown,
            since: Instant::now(),
            device: String::new(),
        });
        match &model.state {
            State::Failed(detail) => return failed("model_failed", detail.clone()),
            State::Ready => {}
            // Asked for, uploaded and loaded over the next driving frames;
            // until then each frame runs on the comma.
            _ => return failed("loading", "the App is getting the model"),
        }
        // One job per model: a newer frame replaces one not sent yet.
        let mut superseded = Vec::new();
        inner.waiting.retain(|job| {
            let keep = job.sha256 != sha;
            if !keep {
                superseded.push(job.id);
            }
            keep
        });
        for id in superseded {
            inner
                .results
                .insert(id, failed("superseded", "a newer frame replaced it"));
        }
        inner.next_id += 1;
        let id = inner.next_id;
        inner.waiting.push_back(Job {
            id,
            sha256: sha,
            outputs,
            input,
        });
        self.changed.notify_all();
        loop {
            if let Some(outcome) = inner.results.remove(&id) {
                return outcome;
            }
            let now = Instant::now();
            if now >= deadline {
                inner.waiting.retain(|job| job.id != id);
                inner.sent.remove(&id);
                return failed("timeout", "the App did not answer in time");
            }
            inner = self
                .changed
                .wait_timeout(inner, deadline - now)
                .map(|(guard, _)| guard)
                .unwrap_or_else(|poisoned| poisoned.into_inner().0);
        }
    }

    /// The one message to send after this driving reply, if any: a model's
    /// AUX_LOAD, the next upload piece, else the oldest job for a ready model.
    pub fn next_outbound(&self) -> Option<(u16, Vec<u8>)> {
        let mut inner = self.lock();
        if !inner.live {
            return None;
        }
        // A model asked for by a job, or one that never was.
        let wanted: Vec<String> = inner.models.keys().cloned().collect();
        for sha in wanted {
            let Some(model) = inner.models.get_mut(&sha) else {
                continue;
            };
            if matches!(model.state, State::Asked | State::Loading)
                && model.since.elapsed() >= STATE_PATIENCE
            {
                model.state = State::Unknown;
            }
            match model.state.clone() {
                State::Unknown => {
                    model.state = State::Asked;
                    model.since = Instant::now();
                    let head = json!({"sha256": sha, "nbytes": model.nbytes});
                    return encode(&head, &[]).ok().map(|bytes| (MSG_LOAD, bytes));
                }
                State::Uploading(offset) => {
                    let piece = (|| -> std::io::Result<Vec<u8>> {
                        let mut file = File::open(&model.path)?;
                        file.seek(SeekFrom::Start(offset))?;
                        let left = model.nbytes.saturating_sub(offset).min(CHUNK as u64);
                        let mut piece = vec![0; left as usize];
                        file.read_exact(&mut piece)?;
                        Ok(piece)
                    })();
                    match piece {
                        Ok(piece) => {
                            let next = offset + piece.len() as u64;
                            model.state = if next >= model.nbytes {
                                State::Loading
                            } else {
                                State::Uploading(next)
                            };
                            model.since = Instant::now();
                            let head = json!({"sha256": sha, "offset": offset});
                            return encode(&head, &[&piece])
                                .ok()
                                .map(|bytes| (MSG_CHUNK, bytes));
                        }
                        Err(error) => {
                            model.state = State::Failed(format!("model read: {error}"));
                        }
                    }
                }
                _ => {}
            }
        }
        let ready = |inner: &Inner, sha: &str| {
            inner
                .models
                .get(sha)
                .is_some_and(|model| model.state == State::Ready)
        };
        let index = inner
            .waiting
            .iter()
            .position(|job| ready(&inner, &job.sha256))?;
        let job = inner.waiting.remove(index)?;
        inner.sent.insert(job.id, Instant::now());
        let head = json!({"id": job.id, "sha256": job.sha256, "outputs": job.outputs});
        match encode(&head, &[&job.input]) {
            Ok(bytes) => Some((MSG_REQ, bytes)),
            Err(error) => {
                inner.sent.remove(&job.id);
                inner
                    .results
                    .insert(job.id, failed("encode", error.to_string()));
                drop(inner);
                self.changed.notify_all();
                None
            }
        }
    }

    /// AUX_STATE or AUX_RESP from the App.
    pub fn deliver(&self, kind: u16, payload: &[u8]) {
        let Ok((head, blob)) = decode(payload) else {
            return;
        };
        let mut inner = self.lock();
        match kind {
            MSG_STATE => {
                let sha = head["sha256"].as_str().unwrap_or_default().to_owned();
                let Some(model) = inner.models.get_mut(&sha) else {
                    return;
                };
                let detail = head["detail"].as_str().unwrap_or_default().to_owned();
                model.state = match head["state"].as_str() {
                    Some("ready") => {
                        model.device = head["device"].as_str().unwrap_or_default().to_owned();
                        State::Ready
                    }
                    Some("need_upload") => State::Uploading(0),
                    Some("loading") => State::Loading,
                    _ => State::Failed(if detail.is_empty() {
                        "the App could not load the model".into()
                    } else {
                        detail
                    }),
                };
                model.since = Instant::now();
                if let State::Failed(detail) = model.state.clone() {
                    let ids: Vec<u64> = inner
                        .waiting
                        .iter()
                        .filter(|job| job.sha256 == sha)
                        .map(|job| job.id)
                        .collect();
                    inner.waiting.retain(|job| job.sha256 != sha);
                    for id in ids {
                        inner
                            .results
                            .insert(id, failed("model_failed", detail.clone()));
                    }
                }
            }
            MSG_RESP => {
                let Some(id) = head["id"].as_u64() else {
                    return;
                };
                // A reply nobody waits for any more (timed out) is dropped.
                if inner.sent.remove(&id).is_none() {
                    return;
                }
                let outcome = if head["ok"] == true {
                    Outcome::Done {
                        head,
                        blob: blob.to_vec(),
                    }
                } else {
                    failed(
                        head["error"].as_str().unwrap_or("failed"),
                        head["detail"].as_str().unwrap_or_default(),
                    )
                };
                inner.results.insert(id, outcome);
            }
            _ => return,
        }
        drop(inner);
        self.changed.notify_all();
    }

    /// For jetlinkd's status: each model's state and device.
    pub fn status(&self) -> Value {
        let inner = self.lock();
        json!({
            "live": inner.live,
            "models": inner.models.iter().map(|(sha, model)| json!({
                "sha256": sha,
                "path": model.path.file_name().map(|n| n.to_string_lossy().into_owned()),
                "state": format!("{:?}", model.state),
                "device": model.device,
            })).collect::<Vec<_>>(),
        })
    }
}

fn read_local(stream: &mut UnixStream) -> std::io::Result<Vec<u8>> {
    let mut size = [0; 4];
    stream.read_exact(&mut size)?;
    let size = u32::from_le_bytes(size) as usize;
    if size == 0 || size > MAX_LOCAL {
        return Err(std::io::Error::other("aux local message size"));
    }
    let mut bytes = vec![0; size];
    stream.read_exact(&mut bytes)?;
    Ok(bytes)
}

fn write_local(stream: &mut UnixStream, bytes: &[u8]) -> std::io::Result<()> {
    let size = u32::try_from(bytes.len()).map_err(std::io::Error::other)?;
    stream.write_all(&size.to_le_bytes())?;
    stream.write_all(bytes)
}

/// One local request: `{"model": path, "outputs": [..], "timeout_ms": n}` and
/// the input's float32 bytes. The reply is the App's AUX_RESP JSON (plus
/// `round_ms`) and the outputs' float32 bytes, or `{"ok": false, ...}`.
fn answer(hub: &Hub, request: &[u8]) -> Result<Vec<u8>, Error> {
    let (head, input) = decode(request)?;
    let model = PathBuf::from(head["model"].as_str().unwrap_or_default());
    let outputs = head["outputs"]
        .as_array()
        .map(|names| {
            names
                .iter()
                .filter_map(|name| name.as_str().map(str::to_owned))
                .collect()
        })
        .unwrap_or_default();
    let timeout = Duration::from_millis(head["timeout_ms"].as_u64().unwrap_or(300));
    let started = Instant::now();
    match hub.run(&model, outputs, input.to_vec(), timeout) {
        Outcome::Done { mut head, blob } => {
            head["round_ms"] = json!(started.elapsed().as_secs_f64() * 1000.0);
            encode(&head, &[&blob])
        }
        Outcome::Failed { code, detail } => {
            encode(&json!({"ok": false, "error": code, "detail": detail}), &[])
        }
    }
}

/// Serves xiaoge's local socket for the life of jetlinkd. Each connection
/// is a camera thread asking one job at a time.
pub fn serve_local(hub: Arc<Hub>, path: &Path) -> Result<thread::JoinHandle<()>, Error> {
    match std::fs::remove_file(path) {
        Ok(()) => {}
        Err(error) if error.kind() == std::io::ErrorKind::NotFound => {}
        Err(error) => return Err(error.into()),
    }
    let listener = UnixListener::bind(path)?;
    std::fs::set_permissions(path, std::fs::Permissions::from_mode(0o600))?;
    Ok(thread::Builder::new()
        .name("jetlink-aux-local".into())
        .spawn(move || {
            for stream in listener.incoming() {
                let Ok(mut stream) = stream else {
                    thread::sleep(Duration::from_millis(100));
                    continue;
                };
                let hub = Arc::clone(&hub);
                let spawned = thread::Builder::new()
                    .name("jetlink-aux-conn".into())
                    .spawn(move || {
                        while let Ok(request) = read_local(&mut stream) {
                            let reply = answer(&hub, &request).unwrap_or_else(|error| {
                                encode(
                                    &json!({"ok": false, "error": "bad_request", "detail": error.to_string()}),
                                    &[],
                                )
                                .unwrap_or_default()
                            });
                            if write_local(&mut stream, &reply).is_err() {
                                break;
                            }
                        }
                    });
                if let Err(error) = spawned {
                    eprintln!("jetlink aux connection thread: {error}");
                }
            }
        })?)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn model() -> (tempfile::TempDir, PathBuf) {
        let dir = tempfile::tempdir().unwrap();
        let assets = dir.path().join("openpilot/selfdrive/carrot/xiaoge/assets");
        std::fs::create_dir_all(&assets).unwrap();
        let path = assets.join("lane.onnx");
        std::fs::write(&path, vec![7u8; CHUNK + 10]).unwrap();
        (dir, path)
    }

    fn state(sha: &str, state: &str) -> Vec<u8> {
        encode(
            &json!({"sha256": sha, "state": state, "device": "QNN(gpu)"}),
            &[],
        )
        .unwrap()
    }

    #[test]
    fn payload_round_trips() {
        let bytes = encode(&json!({"a": 1}), &[b"xy", b"z"]).unwrap();
        let (head, blob) = decode(&bytes).unwrap();
        assert_eq!(head["a"], 1);
        assert_eq!(blob, b"xyz");
        assert!(decode(&[1, 0]).is_err());
        assert!(decode(&[9, 0, 0, 0, b'{']).is_err());
    }

    #[test]
    fn without_a_live_link_every_job_fails_at_once() {
        let (_dir, path) = model();
        let hub = Hub::default();
        let started = Instant::now();
        match hub.run(&path, vec![], vec![0; 4], Duration::from_secs(1)) {
            Outcome::Failed { code, .. } => assert_eq!(code, "unavailable"),
            other => panic!("{other:?}"),
        }
        assert!(started.elapsed() < Duration::from_millis(100));
        assert!(hub.next_outbound().is_none());
    }

    #[test]
    fn only_xiaoges_models_leave_the_comma() {
        let hub = Hub::default();
        hub.set_live(true);
        let dir = tempfile::tempdir().unwrap();
        let other = dir.path().join("secret.onnx");
        std::fs::write(&other, b"x").unwrap();
        match hub.run(&other, vec![], vec![], Duration::from_millis(10)) {
            Outcome::Failed { code, .. } => assert_eq!(code, "bad_model"),
            other => panic!("{other:?}"),
        }
    }

    /// Load, upload in pieces, then a job whose reply comes back through
    /// `deliver`, as the driving client hands it over.
    #[test]
    fn uploads_then_runs_one_job_per_message() {
        let (_dir, path) = model();
        let hub = Arc::new(Hub::default());
        hub.set_live(true);
        // The first job only registers the model and runs on the comma.
        match hub.run(&path, vec![], vec![1; 8], Duration::from_secs(1)) {
            Outcome::Failed { code, .. } => assert_eq!(code, "loading"),
            other => panic!("{other:?}"),
        }
        let (kind, load) = hub.next_outbound().unwrap();
        assert_eq!(kind, MSG_LOAD);
        let sha = decode(&load).unwrap().0["sha256"]
            .as_str()
            .unwrap()
            .to_owned();
        assert!(hub.next_outbound().is_none(), "waits for the App's answer");
        hub.deliver(MSG_STATE, &state(&sha, "need_upload"));
        let (kind, first) = hub.next_outbound().unwrap();
        assert_eq!(kind, MSG_CHUNK);
        let (head, piece) = decode(&first).unwrap();
        assert_eq!((head["offset"].as_u64(), piece.len()), (Some(0), CHUNK));
        let (_, second) = hub.next_outbound().unwrap();
        let (head, piece) = decode(&second).unwrap();
        assert_eq!(
            (head["offset"].as_u64(), piece.len()),
            (Some(CHUNK as u64), 10)
        );
        assert!(hub.next_outbound().is_none());
        // While the App loads, jobs run on the comma.
        match hub.run(&path, vec![], vec![1; 8], Duration::from_millis(10)) {
            Outcome::Failed { code, .. } => assert_eq!(code, "loading"),
            other => panic!("{other:?}"),
        }
        hub.deliver(MSG_STATE, &state(&sha, "ready"));

        let worker = {
            let hub = Arc::clone(&hub);
            let path = path.clone();
            thread::spawn(move || {
                hub.run(
                    &path,
                    vec!["output0".into()],
                    vec![2; 8],
                    Duration::from_secs(2),
                )
            })
        };
        let request = loop {
            if let Some(message) = hub.next_outbound() {
                break message;
            }
            thread::sleep(Duration::from_millis(1));
        };
        assert_eq!(request.0, MSG_REQ);
        let (head, input) = decode(&request.1).unwrap();
        assert_eq!(input, [2; 8]);
        assert_eq!(head["outputs"], json!(["output0"]));
        let reply = encode(
            &json!({"id": head["id"], "ok": true, "outputs": [{"name": "output0", "shape": [1]}]}),
            &[&1.5f32.to_le_bytes()],
        )
        .unwrap();
        hub.deliver(MSG_RESP, &reply);
        match worker.join().unwrap() {
            Outcome::Done { head, blob } => {
                assert_eq!(head["outputs"][0]["name"], "output0");
                assert_eq!(blob, 1.5f32.to_le_bytes());
            }
            other => panic!("{other:?}"),
        }
        assert_eq!(hub.status()["models"][0]["device"], "QNN(gpu)");
    }

    #[test]
    fn a_closed_link_releases_waiting_jobs_and_asks_again_next_time() {
        let (_dir, path) = model();
        let hub = Arc::new(Hub::default());
        hub.set_live(true);
        let _ = hub.run(&path, vec![], vec![], Duration::from_millis(1));
        let (_, load) = hub.next_outbound().unwrap();
        let sha = decode(&load).unwrap().0["sha256"]
            .as_str()
            .unwrap()
            .to_owned();
        hub.deliver(MSG_STATE, &state(&sha, "ready"));
        let worker = {
            let hub = Arc::clone(&hub);
            thread::spawn(move || hub.run(&path, vec![], vec![0; 4], Duration::from_secs(2)))
        };
        while hub.lock().waiting.is_empty() {
            thread::sleep(Duration::from_millis(1));
        }
        hub.set_live(false);
        match worker.join().unwrap() {
            Outcome::Failed { code, .. } => assert_eq!(code, "unavailable"),
            other => panic!("{other:?}"),
        }
        hub.set_live(true);
        assert_eq!(hub.next_outbound().unwrap().0, MSG_LOAD);
    }

    #[test]
    fn a_failed_model_fails_its_jobs() {
        let (_dir, path) = model();
        let hub = Hub::default();
        hub.set_live(true);
        let _ = hub.run(&path, vec![], vec![], Duration::from_millis(1));
        let (_, load) = hub.next_outbound().unwrap();
        let sha = decode(&load).unwrap().0["sha256"]
            .as_str()
            .unwrap()
            .to_owned();
        hub.deliver(
            MSG_STATE,
            &encode(
                &json!({"sha256": sha, "state": "failed", "detail": "no QNN"}),
                &[],
            )
            .unwrap(),
        );
        match hub.run(&path, vec![], vec![], Duration::from_millis(1)) {
            Outcome::Failed { code, detail } => {
                assert_eq!(code, "model_failed");
                assert_eq!(detail, "no QNN");
            }
            other => panic!("{other:?}"),
        }
    }

    #[test]
    fn the_local_socket_answers_unavailable_without_a_link() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("aux.sock");
        let hub = Arc::new(Hub::default());
        let _server = serve_local(Arc::clone(&hub), &path).unwrap();
        let mut stream = UnixStream::connect(&path).unwrap();
        let request = encode(&json!({"model": "/x.onnx", "timeout_ms": 50}), &[&[0; 4]]).unwrap();
        write_local(&mut stream, &request).unwrap();
        let reply = read_local(&mut stream).unwrap();
        let (head, _) = decode(&reply).unwrap();
        assert_eq!(head["ok"], false);
        assert_eq!(head["error"], "unavailable");
    }
}
