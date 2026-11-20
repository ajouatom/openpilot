//! Bounded source recovery orchestration. Only the prepared worker is handed over.
use crate::{
    recovery_policy::{Budget, Policy, State, MAX_INTERNAL_WORK, TASK_SECONDS},
    recovery_prepare::Preparation,
    recovery_task::{Health, Task},
    usb_model::UsbModel,
    Error,
};
use openpilot_usbgpu::model_delivery::failure;
use std::{
    fs,
    path::{Path, PathBuf},
    sync::{atomic::Ordering, Arc},
};
pub const STATUS: &str = "/dev/shm/carrot-egpu-recovery.json";
pub const BUDGET: &str = "/dev/shm/carrot-egpu-recovery-budget.json";

pub struct Recovery {
    pub policy: Policy,
    preparation: Preparation,
    runtime_sha256: String,
    generic: bool,
    task: Option<Task<UsbModel>>,
    task_started: f64,
    health: Arc<Health>,
    status: PathBuf,
    budget: PathBuf,
    next_status: f64,
    last_wait_reason: Option<&'static str>,
    next_wait_log: f64,
}
impl Recovery {
    pub fn new(preparation: Preparation) -> Result<Self, Error> {
        Self::at(preparation, Path::new(STATUS), Path::new(BUDGET))
    }
    pub fn at(preparation: Preparation, status: &Path, budget: &Path) -> Result<Self, Error> {
        let saved = match fs::read(budget) {
            Ok(bytes) => Budget::parse(&bytes),
            Err(error) if error.kind() == std::io::ErrorKind::NotFound => Budget::default(),
            Err(_) => Budget {
                attempts: 3,
                stopped: true,
            },
        };
        let identity = &preparation.identity;
        let runtime_sha256 = identity["runtime"]["sha256"]
            .as_str()
            .or_else(|| identity["runtime_archive_sha256"].as_str())
            .unwrap_or("")
            .to_owned();
        let generic =
            identity["format"] == "comma-generic-onnx" || identity["kind"] == "native-generic-hcq";
        Ok(Self {
            policy: Policy::new(saved),
            preparation,
            runtime_sha256,
            generic,
            task: None,
            task_started: 0.,
            health: Arc::new(Health::default()),
            status: status.to_owned(),
            budget: budget.to_owned(),
            next_status: 0.,
            last_wait_reason: None,
            next_wait_log: 0.,
        })
    }
    fn save(&self) -> Result<(), Error> {
        write(&self.budget, &self.policy.budget)
    }
    pub fn failed(&mut self, now: f64, detail: &str) -> Result<(), Error> {
        let rejected = self
            .preparation
            .model
            .parent()
            .is_none_or(|parent| parent.join("rejected").exists());
        self.policy.failed(
            now,
            retryable(detail, &self.runtime_sha256),
            rejected,
            self.generic,
        );
        if self.policy.state == State::Stopped {
            self.save()?;
        }
        self.event("fallback", detail);
        if self.policy.state == State::Waiting {
            self.event(
                "retry_scheduled",
                "USB present and healthy internal output for 3s",
            );
        }
        self.publish(now, true)
    }
    pub fn observe(
        &mut self,
        now: f64,
        published: bool,
        work: f64,
        dropped: bool,
    ) -> Result<(), Error> {
        let good = published
            && (0. ..=MAX_INTERNAL_WORK).contains(&work)
            && !dropped
            && self.policy.healthy(now);
        self.health.observe(good);
        let was_stopped = self.policy.state == State::Stopped;
        // A loaded startup worker waiting for handover is idle; only a load in
        // progress can delay the internal model.
        let loading = self
            .task
            .as_ref()
            .is_some_and(|task| !(task.hold && task.ready()));
        if self.policy.observe(now, published, work, dropped, loading) {
            if let Some(task) = &self.task {
                task.cancelled.store(true, Ordering::Release);
            }
            if !was_stopped {
                self.save()?;
                self.event("retry_stopped", "internal_delayed");
            }
        }
        Ok(())
    }
    pub fn tick(
        &mut self,
        now: f64,
        present: bool,
        may_join: bool,
    ) -> Result<Option<UsbModel>, Error> {
        if let Some(task) = &self.task {
            if now - self.task_started >= TASK_SECONDS
                && !task.ready()
                && self.policy.state != State::Stopped
            {
                self.policy.stop("cleanup_timeout");
                task.cancelled.store(true, Ordering::Release);
                self.save()?;
            }
            if !present {
                task.cancelled.store(true, Ordering::Release);
            }
            if task.finished() {
                let detail = task
                    .error()
                    .unwrap_or_else(|| "worker ended without handover".into());
                self.task = None;
                self.event("retry_failed", &detail);
                self.policy
                    .completed(now, retryable_preparation(&detail, &self.runtime_sha256));
                self.save()?;
            } else if task.cancelled.load(Ordering::Acquire) {
                self.policy.state = if self.policy.budget.stopped {
                    State::Stopped
                } else {
                    State::Recovering
                };
            } else if task.ready() {
                if self.policy.state != State::Ready {
                    self.policy.state = State::Ready;
                    self.event(
                        "retry_ready",
                        "probe passed; waiting for handover permission",
                    );
                }
                if let Some(model) = (may_join && present).then(|| task.claim()).flatten() {
                    self.task = None;
                    self.policy.state = State::Idle;
                    self.event("retry_selected", "prepared worker handed to modeld");
                    self.publish(now, true)?;
                    return Ok(Some(model));
                }
            }
        }
        let startup = self.policy.starting();
        if self.policy.begin(now, present, self.task.is_some()) {
            // Persist consumption before any preparer thread or child can start.
            self.save()?;
            let preparation = self.preparation.clone();
            let prepare = move |cancel| preparation.prepare(cancel);
            self.task = Some(if startup {
                Task::start_held(Arc::clone(&self.health), prepare)
            } else {
                Task::start(Arc::clone(&self.health), prepare)
            });
            self.task_started = now;
            self.event("retry_started", "internal model remains active");
        }
        if self.policy.state == State::Waiting {
            let reason = if !present {
                "USB absent"
            } else if !self.policy.healthy_for_preparation(now) {
                "internal output not yet healthy for 3s"
            } else {
                "backoff delay"
            };
            if self.last_wait_reason != Some(reason) && now >= self.next_wait_log {
                self.event("retry_waiting", reason);
                self.last_wait_reason = Some(reason);
                self.next_wait_log = now + 5.;
            }
        }
        self.publish(now, false)?;
        Ok(None)
    }
    fn publish(&mut self, now: f64, force: bool) -> Result<(), Error> {
        if force || now >= self.next_status {
            write(
                &self.status,
                &serde_json::json!({"state":self.policy.state,"reason":self.policy.reason,"attempts":self.policy.budget.attempts,
                "limit":3,"updated":now,"retry_in_seconds":(self.policy.next_attempt-now).max(0.).round()}),
            )?;
            self.next_status = now + 1.;
        }
        Ok(())
    }
    fn event(&self, event: &str, detail: &str) {
        eprintln!(
            "eGPU {}",
            serde_json::json!({"event":event,"model":self.preparation.model.parent().and_then(Path::file_name).and_then(|name|name.to_str()),
              "attempt":self.policy.budget.attempts,"limit":3,"detail":detail,
              "retryable":retryable(detail,&self.runtime_sha256),"active_model":if event=="retry_selected" {"eGPU"} else {"internal"}})
        );
    }
}
fn write(path: &Path, value: &impl serde::Serialize) -> Result<(), Error> {
    let stage = path.with_extension(format!("tmp-{}", std::process::id()));
    fs::write(&stage, serde_json::to_vec(value)?)?;
    fs::rename(stage, path)?;
    Ok(())
}
#[must_use]
pub fn retryable(detail: &str, runtime: &str) -> bool {
    failure::gpu_transport_failure(detail)
        || failure::gpu_usb_link_failure(detail, runtime)
        || failure::gpu_unreachable(detail)
        || detail == "precompiled eGPU worker timed out"
}
#[must_use]
pub fn retryable_preparation(detail: &str, runtime: &str) -> bool {
    let line = detail.trim_end().rsplit('\n').next().unwrap_or("");
    let line = line.strip_prefix("RuntimeError: ").unwrap_or(line);
    let pcie = line
        .strip_prefix("PCIe link not up (LTSSM=0x")
        .and_then(|tail| tail.strip_suffix(')'))
        .is_some_and(|digits| {
            !digits.is_empty() && digits.bytes().all(|value| value.is_ascii_hexdigit())
        });
    retryable(detail, runtime)
        || detail == "eGPU ready wait expired"
        || failure::gpu_worker_busy(detail)
        || failure::gpu_bootloader_timeout(detail, "load")
        || pcie
}
#[must_use]
pub fn may_join(
    now: f64,
    valid: [bool; 3],
    received: [f64; 3],
    standstill: bool,
    v_ego: f64,
    steering_pressed: bool,
) -> bool {
    valid
        .into_iter()
        .zip(received)
        .all(|(valid, received)| valid && (0. ..0.25).contains(&(now - received)))
        && ((standstill && v_ego.abs() < 0.01) || steering_pressed)
}
