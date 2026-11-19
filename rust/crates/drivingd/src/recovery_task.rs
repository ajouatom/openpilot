//! A recovery candidate is prepared and destroyed off the camera/frame thread.
use crate::recovery_policy::{MAX_PUBLICATION_GAP, READY_SECONDS, TASK_SECONDS};
use std::{
    sync::{
        atomic::{AtomicBool, AtomicU64, Ordering},
        Arc, Mutex,
    },
    thread,
    time::{Duration, Instant},
};

pub struct Health {
    origin: Instant,
    published: AtomicU64,
    good: AtomicBool,
}
impl Default for Health {
    fn default() -> Self {
        Self {
            origin: Instant::now(),
            published: AtomicU64::new(0),
            good: AtomicBool::new(false),
        }
    }
}
impl Health {
    pub fn observe(&self, good: bool) {
        self.published.store(
            self.origin.elapsed().as_nanos().min(u128::from(u64::MAX)) as u64,
            Ordering::Release,
        );
        self.good.store(good, Ordering::Release);
    }
    fn healthy(&self) -> bool {
        self.good.load(Ordering::Acquire)
            && self
                .origin
                .elapsed()
                .as_nanos()
                .saturating_sub(u128::from(self.published.load(Ordering::Acquire)))
                <= (MAX_PUBLICATION_GAP * 1e9) as u128
    }
}
pub trait Candidate: Send + 'static {
    fn alive(&mut self) -> bool;
}
impl Candidate for crate::usb_model::UsbModel {
    fn alive(&mut self) -> bool {
        self.client.alive()
    }
}
struct Shared<T> {
    candidate: Option<T>,
    ready_at: Option<Instant>,
    error: Option<String>,
    claimed: bool,
}
pub struct Task<T: Candidate> {
    shared: Arc<Mutex<Shared<T>>>,
    pub cancelled: Arc<AtomicBool>,
    health: Arc<Health>,
    done: Arc<AtomicBool>,
    claimed: Arc<AtomicBool>,
    /// The first eGPU start of a drive: once loaded, the idle worker is kept
    /// until the vehicle allows the handover, instead of expiring after
    /// READY_SECONDS and loading the whole model again.
    pub hold: bool,
}
impl<T: Candidate> Task<T> {
    pub fn start(
        health: Arc<Health>,
        prepare: impl FnOnce(Arc<AtomicBool>) -> Result<T, String> + Send + 'static,
    ) -> Self {
        Self::start_with_schedule(health, prepare, background_scheduling, false)
    }
    pub fn start_held(
        health: Arc<Health>,
        prepare: impl FnOnce(Arc<AtomicBool>) -> Result<T, String> + Send + 'static,
    ) -> Self {
        Self::start_with_schedule(health, prepare, background_scheduling, true)
    }
    pub fn start_with_schedule(
        health: Arc<Health>,
        prepare: impl FnOnce(Arc<AtomicBool>) -> Result<T, String> + Send + 'static,
        schedule: impl FnOnce() -> std::io::Result<()> + Send + 'static,
        hold: bool,
    ) -> Self {
        let shared = Arc::new(Mutex::new(Shared {
            candidate: None,
            ready_at: None,
            error: None,
            claimed: false,
        }));
        let cancelled = Arc::new(AtomicBool::new(false));
        let done = Arc::new(AtomicBool::new(false));
        let worker_shared = Arc::clone(&shared);
        let worker_cancel = Arc::clone(&cancelled);
        let worker_done = Arc::clone(&done);
        let watch_health = Arc::clone(&health);
        let launched = thread::Builder::new()
            .name("egpu-recovery".into())
            .spawn(move || {
                let started = Instant::now();
                if let Err(error) = schedule() {
                    worker_shared.lock().unwrap().error = Some(error.to_string());
                    worker_done.store(true, Ordering::Release);
                    return;
                }
                let watch_shared = Arc::clone(&worker_shared);
                let watch_cancel = Arc::clone(&worker_cancel);
                let watch_done = Arc::clone(&worker_done);
                let watching = thread::Builder::new()
                    .name("egpu-recovery-health".into())
                    .spawn(move || {
                        while !watch_done.load(Ordering::Acquire) {
                            {
                                let mut state = watch_shared.lock().unwrap();
                                if state.claimed {
                                    return;
                                }
                                // A held, loaded worker is idle: it no longer
                                // competes with the internal model or times out.
                                let held = hold && state.ready_at.is_some();
                                let error = if !watch_health.healthy() && !held {
                                    Some("internal model publication delayed")
                                } else if started.elapsed().as_secs_f64() >= TASK_SECONDS
                                    && state.ready_at.is_none()
                                {
                                    Some("eGPU recovery task timed out")
                                } else if !hold
                                    && state.ready_at.is_some_and(|ready| {
                                        ready.elapsed().as_secs_f64() >= READY_SECONDS
                                    })
                                {
                                    Some("eGPU ready wait expired")
                                } else {
                                    None
                                };
                                if let Some(error) = error {
                                    state.error = Some(error.into());
                                    watch_cancel.store(true, Ordering::Release);
                                    return;
                                }
                            }
                            thread::sleep(Duration::from_millis(20));
                        }
                    });
                if let Err(error) = watching {
                    worker_shared.lock().unwrap().error = Some(error.to_string());
                    worker_cancel.store(true, Ordering::Release);
                    worker_done.store(true, Ordering::Release);
                    return;
                }
                let prepared = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                    prepare(Arc::clone(&worker_cancel))
                }))
                .unwrap_or_else(|_| Err("eGPU recovery preparation panicked".into()));
                match prepared {
                    Ok(candidate) => {
                        if worker_cancel.load(Ordering::Acquire) {
                            drop(candidate);
                        } else {
                            let mut state = worker_shared.lock().unwrap();
                            state.candidate = Some(candidate);
                            state.ready_at = Some(Instant::now());
                        }
                    }
                    Err(error) => {
                        let mut state = worker_shared.lock().unwrap();
                        if state.error.is_none() {
                            state.error = Some(error);
                        }
                        worker_cancel.store(true, Ordering::Release);
                    }
                }
                loop {
                    let cleanup = {
                        let mut state = worker_shared.lock().unwrap();
                        if state.claimed {
                            break;
                        }
                        if state
                            .candidate
                            .as_mut()
                            .is_some_and(|candidate| !candidate.alive())
                        {
                            state.error = Some("eGPU prepared worker exited".into());
                            worker_cancel.store(true, Ordering::Release);
                        }
                        if worker_cancel.load(Ordering::Acquire) {
                            Some(state.candidate.take())
                        } else {
                            None
                        }
                    };
                    if let Some(candidate) = cleanup {
                        drop(candidate);
                        break;
                    }
                    thread::sleep(Duration::from_millis(20));
                }
                worker_done.store(true, Ordering::Release);
            });
        if let Err(error) = launched {
            shared.lock().unwrap().error = Some(error.to_string());
            cancelled.store(true, Ordering::Release);
            done.store(true, Ordering::Release);
        }
        Self {
            shared,
            cancelled,
            health,
            done,
            claimed: Arc::new(AtomicBool::new(false)),
            hold,
        }
    }
    /// A busy worker never stalls the frame thread. The caller also applies the
    /// fresh source vehicle-state join gate immediately before taking ownership.
    pub fn claim(&self) -> Option<T> {
        let mut state = self.shared.try_lock().ok()?;
        if self.cancelled.load(Ordering::Acquire)
            || !self.health.healthy()
            || state
                .ready_at
                .is_none_or(|ready| !self.hold && ready.elapsed().as_secs_f64() >= READY_SECONDS)
        {
            return None;
        }
        let model = state.candidate.take()?;
        state.claimed = true;
        self.claimed.store(true, Ordering::Release);
        Some(model)
    }
    pub fn ready(&self) -> bool {
        !self.cancelled.load(Ordering::Acquire)
            && self
                .shared
                .try_lock()
                .is_ok_and(|state| state.candidate.is_some() && state.ready_at.is_some())
    }
    pub fn finished(&self) -> bool {
        self.done.load(Ordering::Acquire)
    }
    pub fn error(&self) -> Option<String> {
        self.shared.try_lock().ok()?.error.clone()
    }
}
impl<T: Candidate> Drop for Task<T> {
    fn drop(&mut self) {
        if !self.claimed.load(Ordering::Acquire) {
            self.cancelled.store(true, Ordering::Release);
        }
    }
}
pub(crate) fn background_scheduling() -> std::io::Result<()> {
    // SAFETY: GNU/MUSL sched_param contains only C integers, integer timespecs,
    // and reserved storage; all-zero is valid for every field.
    let mut settings: libc::sched_param = unsafe { std::mem::zeroed() };
    settings.sched_priority = 0;
    // SAFETY: the current thread drops inherited realtime scheduling before
    // spawning inference. The parameter points to initialized storage.
    if unsafe { libc::sched_setscheduler(0, libc::SCHED_OTHER, &settings) } != 0 {
        return Err(std::io::Error::last_os_error());
    }
    let mut cores = rustix::thread::CpuSet::new();
    for core in 0..4 {
        cores.set(core);
    }
    rustix::thread::sched_setaffinity(None, &cores).map_err(std::io::Error::from)?;
    // SAFETY: PRIO_PROCESS with pid zero changes this thread's own nice value.
    if unsafe { libc::setpriority(libc::PRIO_PROCESS, 0, 10) } != 0 {
        return Err(std::io::Error::last_os_error());
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::{Candidate, Health, Task};
    use std::{sync::Arc, thread, time::Duration};

    struct Worker;
    impl Candidate for Worker {
        fn alive(&mut self) -> bool {
            true
        }
    }

    fn started(hold: bool) -> Task<Worker> {
        // Health::default() is not healthy: the internal model has published nothing.
        Task::start_with_schedule(Arc::new(Health::default()), |_| Ok(Worker), || Ok(()), hold)
    }

    #[test]
    fn a_held_startup_worker_survives_while_it_waits_for_handover() {
        let held = started(true);
        let retry = started(false);
        thread::sleep(Duration::from_millis(300));
        assert!(held.ready(), "held worker must stay loaded");
        assert!(!held.finished());
        assert!(
            !retry.ready(),
            "a recovery retry still yields to the internal model"
        );
        assert!(retry.finished());
        // Handover itself still requires a healthy internal model.
        assert!(held.claim().is_none());
    }
}
