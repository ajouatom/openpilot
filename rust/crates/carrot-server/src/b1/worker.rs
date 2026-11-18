//! The B1-mode indicator worker (`Process('blink_source.py', 'ble')`), run
//! on its own thread with its own subscriptions.
use openpilot_blink::source::{Feedback, Mode, Record, Worker};
use openpilot_cereal::{car_capnp::car_params, log_capnp::event};
use openpilot_messaging::{runtime::SubMaster, state::Options};
use openpilot_params::Params;
use std::{
    sync::{
        atomic::{AtomicBool, Ordering},
        Arc, Mutex,
    },
    thread::JoinHandle,
    time::Duration,
};

const TOPICS: &[&str] = &[
    "carState",
    "carControl",
    "selfdriveState",
    "pandaStates",
    "modelV2",
    "carrotMan",
    "livePose",
    "radarState",
];

pub fn monotonic() -> f64 {
    let t = rustix::time::clock_gettime(rustix::time::ClockId::Monotonic);
    t.tv_sec as f64 + t.tv_nsec as f64 * 1e-9
}

#[derive(Default)]
struct Shared {
    feedback: Mutex<Feedback>,
    latest: Mutex<Option<Record>>,
}

pub struct Handle {
    stop: Arc<AtomicBool>,
    shared: Arc<Shared>,
    thread: Option<JoinHandle<()>>,
}

impl Handle {
    pub fn start() -> Self {
        let stop = Arc::new(AtomicBool::new(false));
        let shared = Arc::new(Shared::default());
        let (s, sh) = (Arc::clone(&stop), Arc::clone(&shared));
        let thread = std::thread::Builder::new()
            .name("b1-worker".into())
            .spawn(move || {
                if let Err(error) = run(&s, &sh) {
                    eprintln!("B1 indicator worker: {error}");
                }
            })
            .ok();
        Self {
            stop,
            shared,
            thread,
        }
    }

    /// `process.poll() is not None`.
    pub fn exited(&self) -> bool {
        self.thread.as_ref().is_none_or(JoinHandle::is_finished)
    }

    pub fn send(&self, feedback: Feedback) {
        if let Ok(mut slot) = self.shared.feedback.lock() {
            *slot = feedback;
        }
    }

    /// The worker's latest record, taken once.
    pub fn latest(&self) -> Option<Record> {
        self.shared
            .latest
            .lock()
            .ok()
            .and_then(|mut slot| slot.take())
    }
}

impl Drop for Handle {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        if let Some(thread) = self.thread.take() {
            let _ = thread.join();
        }
    }
}

fn run(stop: &AtomicBool, shared: &Shared) -> Result<(), String> {
    let params = Params::for_runtime().map_err(|e| e.to_string())?;
    let Some(bytes) = params.get("CarParams").map_err(|e| e.to_string())? else {
        return Ok(());
    };
    let message =
        capnp::serialize::read_message(&mut &bytes[..], capnp::message::ReaderOptions::new())
            .map_err(|e| e.to_string())?;
    let cp = message
        .get_root::<car_params::Reader<'_>>()
        .map_err(|e| e.to_string())?;
    // A passive car has no indicator output.
    if cp.get_passive() {
        return Ok(());
    }
    let fingerprint = cp
        .get_car_fingerprint()
        .ok()
        .and_then(|t| t.to_str().ok())
        .unwrap_or_default()
        .to_owned();
    // carrot `native_hold_supported` (6af3073e3): HyundaiExtFlags.CANFD_INDICATOR_HOLD.
    let native_hold = cp.get_brand().ok().and_then(|t| t.to_str().ok()) == Some("hyundai")
        && cp.get_openpilot_longitudinal_control()
        && cp.get_ext_flags() & (1 << 19) != 0;
    let mut worker = Worker::new(
        Mode::Ble,
        fingerprint,
        !cp.get_radar_unavailable(),
        native_hold,
    );
    let mut sm = SubMaster::for_runtime(TOPICS, Options::default()).map_err(|e| e.to_string())?;
    while !stop.load(Ordering::Relaxed) {
        sm.update(Duration::ZERO).map_err(|e| e.to_string())?;
        let now = monotonic();
        let topic = sm.state.topic("carState").map_err(|e| e.to_string())?;
        let car_fresh =
            topic.valid && topic.alive && (0. ..0.35).contains(&(now - topic.receive_time));
        let feedback = shared
            .feedback
            .lock()
            .map(|f| f.clone())
            .unwrap_or_default();
        if let Ok(reader) = topic.event() {
            if let Ok(event::Which::CarState(Ok(cs))) = reader.which() {
                if worker
                    .update(now, cs, car_fresh, &sm.state, &params, &feedback)
                    .is_err()
                {
                    worker.record = None;
                }
                if let Ok(mut slot) = shared.latest.lock() {
                    slot.clone_from(&worker.record);
                }
            }
        }
        let next = now + 0.05 - monotonic();
        std::thread::sleep(Duration::from_secs_f64(next.max(0.005)));
    }
    Ok(())
}
