//! Single RPC worker at the frame thread's priority; native execution stays on
//! the frame thread.
use crate::{
    adapter::Adapter,
    contract,
    rpc::ProxyClient,
    transition::{ControlState, Decision, Outcome, Phase, Source, Transition},
    Deadline, Error,
};
use openpilot_modeld::prediction::DrivingPrediction;
use std::{
    net::Shutdown,
    os::unix::net::UnixStream,
    path::{Path, PathBuf},
    sync::mpsc::{self, Receiver, SyncSender, TryRecvError},
    thread::{self, JoinHandle},
    time::{Duration, Instant},
};

struct Model {
    client: ProxyClient,
    adapter: Adapter,
}
enum Work {
    Connect(PathBuf),
    Infer {
        model: Box<Model>,
        warped: Vec<u8>,
        packed: Vec<f32>,
        frame: u32,
        source_sof: u64,
        deadline: Deadline,
        started: Instant,
        /// When the frame thread handed the warped frame to the worker.
        prepared: Instant,
    },
}
/// Where one peer frame spent its time, in the order it happened: camera warp
/// and input packing on the frame thread, the wait for the RPC worker, the
/// request/reply with the daemon (USB and the peer's inference), and parsing.
/// carrot's model.py reports the same split for a slow frame.
#[derive(Clone, Copy, Debug)]
struct Stages {
    warp: Duration,
    handoff: Duration,
    roundtrip: Duration,
    parse: Duration,
}
impl Stages {
    fn total(&self) -> Duration {
        self.warp + self.handoff + self.roundtrip + self.parse
    }
}
fn slow_frame_line(frame: u32, stages: &Stages) -> String {
    let ms = |duration: Duration| duration.as_secs_f64() * 1000.0;
    format!(
        "Jetlink slow frame {frame}: {:.1} ms; warp {:.1}, handoff {:.1}, roundtrip {:.1}, parse {:.1}",
        ms(stages.total()),
        ms(stages.warp),
        ms(stages.handoff),
        ms(stages.roundtrip),
        ms(stages.parse),
    )
}
enum Complete {
    Connected(Box<Model>),
    Inferred {
        model: Box<Model>,
        output: Box<DrivingPrediction>,
        frame: u32,
        stages: Stages,
    },
}
#[derive(Clone, Debug)]
pub struct Status {
    pub decision: Decision,
    pub generation: String,
    pub frame: u32,
    pub execution_ms: f64,
    pub validated: bool,
}
pub struct FrameInput {
    pub controls: ControlState,
    pub frame: u32,
    pub source_sof: u64,
    pub prepare_only: bool,
    pub camera_ready: bool,
    pub desire: [f32; 8],
    pub traffic: [f32; 2],
    pub action: [f32; 2],
    /// N16 `navigation` (zeros without a route); ignored by other graphs.
    pub navigation: [f32; crate::model_spec::NAVIGATION_FLOATS],
}
/// How long modeld waits for the peer's answer to one frame: carrot's local
/// client timeout (link.py `timeout=.15`). A frame slower than 50 ms is late,
/// not lost. With a 50 ms budget every late frame swapped the published model
/// to the internal one for five seconds and back, twice a minute in the car.
pub const FRAME_BUDGET_NS: u64 = 150_000_000;
/// A shadow frame slower than this does not qualify the peer for joining.
pub const PROBE_BUDGET: Duration = Duration::from_millis(100);

/// The peer's frame times over the last two seconds, judged against the
/// comma's 20 Hz. A recurrent graph (N16, Cinque Terre V3) steps its own
/// history once per frame it is sent; a phone that answers in 70 ms (N16 on a
/// Legion Y700's GPU) drops every third camera frame behind it, the graph
/// sees time run unevenly, and its path swung left and right in the car. Too
/// slow on average, the internal model drives and the peer only shadows;
/// it joins again at the usual gate (stopped, or the driver steering) once
/// it is fast again. One slow frame changes nothing.
#[derive(Default)]
pub struct SpeedGate {
    recent: std::collections::VecDeque<f64>,
}
impl SpeedGate {
    /// Joining needs this many frames averaging at most `JOIN_MEAN_MS`.
    pub const JOIN_FRAMES: usize = 20;
    pub const JOIN_MEAN_MS: f64 = 48.0;
    /// Driving, an average over this many frames above `LEAVE_MEAN_MS` leaves.
    pub const LEAVE_FRAMES: usize = 40;
    pub const LEAVE_MEAN_MS: f64 = 52.0;
    pub fn clear(&mut self) {
        self.recent.clear();
    }
    pub fn record(&mut self, ms: f64) {
        if self.recent.len() == Self::LEAVE_FRAMES {
            self.recent.pop_front();
        }
        self.recent.push_back(ms);
    }
    fn mean(&self, frames: usize) -> Option<f64> {
        (self.recent.len() >= frames)
            .then(|| self.recent.iter().rev().take(frames).sum::<f64>() / frames as f64)
    }
    pub fn may_join(&self) -> bool {
        self.mean(Self::JOIN_FRAMES)
            .is_some_and(|mean| mean <= Self::JOIN_MEAN_MS)
    }
    /// The average that makes the driving peer leave, if it is that slow.
    pub fn too_slow(&self) -> Option<f64> {
        self.mean(Self::LEAVE_FRAMES)
            .filter(|mean| *mean > Self::LEAVE_MEAN_MS)
    }
}

pub struct Runtime {
    work: Option<SyncSender<Work>>,
    results: Receiver<Result<Complete, Error>>,
    worker: Option<JoinHandle<()>>,
    identity: Option<contract::Identity>,
    selected_sha: String,
    abandoned: bool,
    path: PathBuf,
    transition: Transition,
    model: Option<Box<Model>>,
    pending: bool,
    connecting: bool,
    cancel: Option<UnixStream>,
    retry_at: Instant,
    /// Local outputs produced so far; a cold first execution precedes any join.
    small_runs: u32,
    controls: ControlState,
    deadline: Deadline,
    started: Instant,
    slow_logged: Option<Instant>,
    /// A shadow frame (sent while the internal model is published) answered
    /// within `PROBE_BUDGET` since the last loss. Joining waits for it, so a
    /// peer that stopped answering never blocks the frame thread.
    probe_passed: bool,
    /// The connected graph takes N16 navigation inputs.
    navigation_model: bool,
    speed: SpeedGate,
    /// Stage times of the peer frame collected since the last `begin`.
    last_stages: Option<Stages>,
    frame: u32,
    pub status: Status,
    pub detail: String,
}
impl Runtime {
    /// Construct before assigning the realtime frame thread's scheduler/affinity.
    pub fn new(path: &Path) -> Result<Self, Error> {
        let (work_tx, work_rx) = mpsc::sync_channel::<Work>(1);
        let (result_tx, results) = mpsc::sync_channel(1);
        let worker = thread::Builder::new()
            .name("jetlink-rpc".into())
            .spawn(move || {
                crate::platform::frame_worker();
                while let Ok(work) = work_rx.recv() {
                    let result = execute(work);
                    if result_tx.send(result).is_err() {
                        break;
                    }
                }
            })?;
        let now = Instant::now();
        Ok(Self {
            work: Some(work_tx),
            results,
            worker: Some(worker),
            identity: None,
            selected_sha: contract::SHA256.into(),
            abandoned: false,
            path: path.to_owned(),
            transition: Transition::default(),
            model: None,
            pending: false,
            connecting: false,
            cancel: None,
            retry_at: now,
            small_runs: 0,
            controls: ControlState {
                join_allowed: false,
            },
            deadline: Deadline(0),
            started: now,
            slow_logged: None,
            probe_passed: false,
            navigation_model: false,
            speed: SpeedGate::default(),
            last_stages: None,
            frame: 0,
            status: Status {
                decision: Decision {
                    source: Source::Native,
                    phase: Phase::Off,
                    loss_latched: false,
                    reset_required: false,
                },
                generation: String::new(),
                frame: 0,
                execution_ms: 0.0,
                validated: false,
            },
            detail: String::new(),
        })
    }
    fn send(&self, work: Work) -> Result<(), Error> {
        self.work
            .as_ref()
            .ok_or(Error::Closed)?
            .try_send(work)
            .map_err(|_| Error::Contract("RPC worker busy or stopped"))
    }
    fn cancel(&mut self) {
        if let Some(stream) = self.cancel.take() {
            if let Err(error) = stream.shutdown(Shutdown::Both) {
                if error.kind() != std::io::ErrorKind::NotConnected {
                    eprintln!("jetlink cancellation: {error}");
                }
            }
        }
    }
    fn lost(&mut self, error: impl std::fmt::Display) {
        if self.status.decision.source == Source::Jetlink {
            let fault = self.path.with_file_name("carrot-jetlink-fault");
            if let Err(error) = crate::fault::write(&fault, crate::now_ns() as f64 * 1e-9) {
                eprintln!("could not persist active Jetlink fault: {error}");
            }
        }
        self.detail = error.to_string().chars().take(240).collect();
        self.probe_passed = false;
        self.status.decision = self.transition.update(false, self.controls, Outcome::Lost);
        self.cancel();
        self.model = None;
        self.identity = None;
        self.abandoned = true;
        self.retry_at = Instant::now() + Duration::from_secs(5);
        // A cancelled worker is drained before any new work is submitted.
    }
    fn collect(&mut self, result: Result<Complete, Error>) -> Option<DrivingPrediction> {
        let was_connecting = self.connecting;
        self.pending = false;
        self.connecting = false;
        if self.abandoned {
            self.abandoned = false;
            return None;
        }
        match result {
            Ok(Complete::Connected(model)) => {
                self.status.generation = crate::rpc::encode_generation(model.client.generation);
                self.identity = Some(model.client.identity.clone());
                self.selected_sha = model.client.session.spec.sha256.clone();
                self.navigation_model = model.client.session.spec.navigation();
                self.speed.clear();
                match model.client.cancel_handle() {
                    Ok(handle) => self.cancel = Some(handle),
                    Err(error) => {
                        self.lost(error);
                        return None;
                    }
                }
                self.model = Some(model);
                self.detail.clear();
                None
            }
            Ok(Complete::Inferred {
                model,
                output,
                frame,
                stages,
            }) => {
                let elapsed = stages.total();
                self.last_stages = Some(stages);
                // Collected for a frame sent while the internal model was
                // published: a shadow frame, which qualifies (or not) a join.
                self.speed.record(elapsed.as_secs_f64() * 1000.0);
                if self.status.decision.source != Source::Jetlink {
                    self.probe_passed = elapsed <= PROBE_BUDGET && self.speed.may_join();
                } else if let Some(mean) = self.speed.too_slow() {
                    // Leaves at the next `begin`; the link stays up.
                    self.probe_passed = false;
                    self.detail = format!(
                        "Jetlink frames average {mean:.0} ms, over the {} ms a 20 Hz frame allows; the internal model drives",
                        SpeedGate::LEAVE_MEAN_MS
                    );
                    eprintln!("{}", self.detail);
                }
                self.status.execution_ms = elapsed.as_secs_f64() * 1000.0;
                // As carrot's model.py: a frame over 50 ms is reported, at most
                // once a second, and still used.
                if elapsed > Duration::from_millis(50)
                    && self
                        .slow_logged
                        .is_none_or(|at| at.elapsed() >= Duration::from_secs(1))
                {
                    self.slow_logged = Some(Instant::now());
                    eprintln!("{}", slow_frame_line(frame, &stages));
                }
                self.status.frame = frame;
                self.model = Some(model);
                Some(*output)
            }
            Err(error) => {
                if was_connecting {
                    self.detail = error.to_string().chars().take(240).collect();
                    self.retry_at = Instant::now() + Duration::from_secs(5);
                } else {
                    self.lost(error);
                }
                None
            }
        }
    }
    /// Whether frames should carry N16 navigation inputs.
    pub fn navigation_model(&self) -> bool {
        self.navigation_model
    }
    pub fn selected_sha(&self) -> &str {
        &self.selected_sha
    }
    pub fn begin(&mut self, input: FrameInput, warp: impl FnOnce() -> Result<Vec<u8>, Error>) {
        let FrameInput {
            controls,
            frame,
            source_sof,
            prepare_only,
            camera_ready,
            desire,
            traffic,
            action,
            navigation,
        } = input;
        self.started = Instant::now();
        self.last_stages = None;
        self.deadline = Deadline(crate::now_ns().saturating_add(FRAME_BUDGET_NS));
        self.frame = frame;
        self.controls = controls;
        if self.pending || self.connecting {
            match self.results.try_recv() {
                Ok(result) => {
                    self.collect(result);
                }
                Err(TryRecvError::Empty) => {}
                Err(TryRecvError::Disconnected) => self.lost(Error::Closed),
            }
        }
        if self.model.is_none()
            && !self.pending
            && !self.connecting
            && Instant::now() >= self.retry_at
        {
            self.abandoned = false;
            match self.send(Work::Connect(self.path.clone())) {
                Ok(()) => self.connecting = true,
                Err(error) => self.lost(error),
            }
        }
        let ready = self
            .model
            .as_ref()
            .is_some_and(|model| !model.client.dead())
            && camera_ready
            && self.small_runs >= 3
            && !self.pending;
        self.status.validated = ready;
        // carrot's JoiningModel switches on the first frame and, when the peer
        // does not answer, holds modeld for the whole 150 ms budget every 5 s
        // (dropped frames, commIssue at a standstill). Join only after a
        // shadow frame answered in time; until then frames still go to the
        // peer without being waited for.
        self.status.decision =
            self.transition
                .update(ready && self.probe_passed, controls, Outcome::None);
        if !ready {
            return;
        }
        let Some(mut model) = self.model.take() else {
            return;
        };
        if self.status.decision.reset_required {
            model.adapter.reset();
        }
        // A camera gap (`prepare_only`) is not a reason to restart the peer
        // model. carrot's model.py runs the full recurrent graph for each
        // received pair and lets the gap show in pose validity. Resetting here
        // cleared the phone's history every few frames whenever its answers
        // took longer than a frame period, and the published path shook.
        let _ = prepare_only;
        let prepared = warp().and_then(|warped| {
            let packed = model.adapter.prepare_session(
                &warped,
                desire,
                traffic,
                action,
                &navigation,
                model.client.session.packed_count(),
            )?;
            Ok((warped, packed))
        });
        match prepared {
            Ok((warped, packed)) => {
                match self.send(Work::Infer {
                    model,
                    warped,
                    packed,
                    frame,
                    source_sof,
                    deadline: self.deadline,
                    started: self.started,
                    prepared: Instant::now(),
                }) {
                    Ok(()) => self.pending = true,
                    Err(error) => self.lost(error),
                }
            }
            Err(error) => self.lost(error),
        }
    }
    /// Warp, hand-off, round trip and parse of this iteration's peer frame in
    /// ms, when one was collected.
    pub fn frame_stages_ms(&self) -> Option<[f64; 4]> {
        let ms = |duration: Duration| duration.as_secs_f64() * 1000.0;
        self.last_stages.map(|stages| {
            [
                ms(stages.warp),
                ms(stages.handoff),
                ms(stages.roundtrip),
                ms(stages.parse),
            ]
        })
    }
    /// True between `begin` and `finish` when this frame went to the peer and
    /// the peer is the published model. carrot's JoiningModel.run does not run
    /// the internal model for such a frame.
    pub fn awaiting_frame(&self) -> bool {
        self.pending && self.status.decision.source == Source::Jetlink
    }
    /// An internal output produced outside `finish` (the fallback run after a
    /// lost peer frame) counts toward the warm-up that precedes a join.
    pub fn count_native_output(&mut self) {
        self.small_runs = self.small_runs.saturating_add(1);
    }
    pub fn finish(&mut self, native: Option<DrivingPrediction>) -> Option<DrivingPrediction> {
        if native.is_some() {
            self.small_runs = self.small_runs.saturating_add(1);
        }
        // While the peer model is the published one its answer is used for
        // every received frame, including frames where the internal model only
        // advanced its history (`native` is None), as carrot's model.py does.
        if self.status.decision.source != Source::Jetlink {
            return native;
        }
        if !self.pending {
            self.lost("No external frame available");
            return native;
        }
        let result = self.deadline.remaining().and_then(|remaining| {
            self.results
                .recv_timeout(remaining)
                .map_err(|_| Error::Deadline)
        });
        match result {
            Ok(result) => {
                let output = self.collect(result);
                if output.is_none() {
                    return native;
                }
                if self.status.frame != self.frame || crate::now_ns() > self.deadline.0 {
                    self.lost(Error::Deadline);
                    native
                } else {
                    output
                }
            }
            Err(error) => {
                self.lost(error);
                native
            }
        }
    }
    pub fn valid_at_publish(&mut self) -> bool {
        if self.status.decision.source == Source::Jetlink && crate::now_ns() > self.deadline.0 {
            self.lost("Jetlink publication exceeded its frame budget");
            return false;
        }
        !(self.status.decision.loss_latched && crate::now_ns() > self.deadline.0)
    }
}
fn execute(work: Work) -> Result<Complete, Error> {
    match work {
        Work::Connect(path) => {
            let client = ProxyClient::connect(&path)?;
            let adapter = Adapter::for_spec(&client.session.spec)?;
            Ok(Complete::Connected(Box::new(Model { client, adapter })))
        }
        Work::Infer {
            mut model,
            warped,
            packed,
            frame,
            source_sof,
            deadline,
            started,
            prepared,
        } => {
            let picked_up = Instant::now();
            model.client.set_source_sof(source_sof);
            let values = model.client.infer(
                frame,
                &warped,
                &packed,
                deadline,
                model.adapter.reset_next(),
            )?;
            let replied = Instant::now();
            let output = Box::new(model.adapter.parse(&values, deadline)?);
            Ok(Complete::Inferred {
                model,
                output,
                frame,
                stages: Stages {
                    warp: prepared.saturating_duration_since(started),
                    handoff: picked_up.saturating_duration_since(prepared),
                    roundtrip: replied.saturating_duration_since(picked_up),
                    parse: replied.elapsed(),
                },
            })
        }
    }
}
impl Drop for Runtime {
    fn drop(&mut self) {
        self.cancel();
        self.work.take();
        // Connect is bounded at 500ms and inference by FRAME_BUDGET_NS.
        let until = Instant::now() + Duration::from_millis(600);
        while self
            .worker
            .as_ref()
            .is_some_and(|worker| !worker.is_finished())
            && Instant::now() < until
        {
            match self.results.recv_timeout(Duration::from_millis(5)) {
                Ok(_) => {}
                Err(mpsc::RecvTimeoutError::Timeout) => {}
                Err(mpsc::RecvTimeoutError::Disconnected) => break,
            }
        }
        if let Some(worker) = self.worker.take() {
            if worker.is_finished() {
                if worker.join().is_err() {
                    eprintln!("jetlink RPC worker panicked");
                }
            } else {
                eprintln!("jetlink RPC worker did not stop within 600ms");
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn speed_gate_keeps_a_fast_peer_and_benches_a_slow_one() {
        let mut gate = SpeedGate::default();
        for _ in 0..19 {
            gate.record(45.);
        }
        assert!(!gate.may_join(), "too few frames to judge");
        gate.record(45.);
        assert!(gate.may_join());
        // The user's N16 on a Snapdragon 8 Gen 2 (route cb4): 45.5 ms on
        // average, one frame in thirty over 50 ms, a rare one near 97 ms.
        for i in 0..80 {
            gate.record(match i % 30 {
                0 => 52.,
                15 if i == 45 => 97.,
                _ => 45.5,
            });
        }
        assert!(
            gate.too_slow().is_none(),
            "a rare slow frame never benches it"
        );
        for _ in 0..20 {
            gate.record(45.5);
        }
        assert!(gate.may_join());
        // N16 on a Legion Y700's GPU.
        let mut slow = SpeedGate::default();
        for _ in 0..40 {
            slow.record(70.);
        }
        assert!(!slow.may_join());
        assert!(slow
            .too_slow()
            .is_some_and(|mean| (mean - 70.).abs() < 1e-9));
        slow.clear();
        assert!(slow.too_slow().is_none());
    }

    #[test]
    fn slow_frame_line_keeps_the_total_first_and_names_each_stage() {
        let stages = Stages {
            warp: Duration::from_micros(4_200),
            handoff: Duration::from_micros(300),
            roundtrip: Duration::from_micros(57_000),
            parse: Duration::from_micros(1_500),
        };
        assert_eq!(stages.total(), Duration::from_micros(63_000));
        assert_eq!(
            slow_frame_line(812, &stages),
            "Jetlink slow frame 812: 63.0 ms; warp 4.2, handoff 0.3, roundtrip 57.0, parse 1.5"
        );
    }
}
