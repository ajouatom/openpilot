//! Bounded eGPU retry policy while native publications remain healthy.
use serde::{Deserialize, Serialize};
pub const DELAYS: [f64; 3] = [10., 30., 60.];
pub const HEALTH_SECONDS: f64 = 3.;
pub const MAX_PUBLICATION_GAP: f64 = 0.1;
pub const MAX_INTERNAL_WORK: f64 = 0.05;
pub const TASK_SECONDS: f64 = openpilot_usbgpu::client::MAX_LOAD.as_secs() as f64 + 40.;
pub const READY_SECONDS: f64 = 30.;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum State {
    Idle,
    Waiting,
    Recovering,
    Ready,
    Stopped,
}
#[derive(Clone, Copy, Debug, Default, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct Budget {
    pub attempts: usize,
    pub stopped: bool,
}
impl Budget {
    /// Damaged persisted state must never create another retry budget.
    pub fn parse(bytes: &[u8]) -> Self {
        serde_json::from_slice::<Self>(bytes)
            .ok()
            .filter(|v| v.attempts <= DELAYS.len())
            .unwrap_or(Self {
                attempts: DELAYS.len(),
                stopped: true,
            })
    }
}
pub struct Policy {
    pub state: State,
    pub reason: &'static str,
    pub budget: Budget,
    pub next_attempt: f64,
    first_start: bool,
    healthy_since: Option<f64>,
    last_publication: Option<f64>,
}
impl Policy {
    pub fn new(budget: Budget) -> Self {
        Self {
            state: State::Idle,
            reason: "",
            budget,
            next_attempt: 0.,
            first_start: false,
            healthy_since: None,
            last_publication: None,
        }
    }
    pub fn startup(&mut self) {
        if self.budget.stopped || self.budget.attempts >= DELAYS.len() {
            self.stop("attempts_exhausted");
        } else {
            self.first_start = true;
            self.state = State::Waiting;
        }
    }
    /// The next attempt is the drive's first eGPU start, not a recovery retry.
    pub fn starting(&self) -> bool {
        self.first_start
    }
    pub fn stop(&mut self, reason: &'static str) {
        if self.state == State::Stopped {
            return;
        }
        self.state = State::Stopped;
        self.reason = reason;
        self.budget.stopped = true;
    }
    pub fn failed(&mut self, now: f64, retryable: bool, rejected: bool, generic: bool) {
        self.healthy_since = None;
        self.last_publication = None;
        if self.budget.stopped || self.budget.attempts >= DELAYS.len() || !retryable {
            self.stop(if !retryable {
                "not_retryable"
            } else {
                "attempts_exhausted"
            });
        } else if rejected || !generic {
            self.stop("not_retryable");
        } else {
            self.state = State::Waiting;
            self.reason = "";
            self.next_attempt = now + DELAYS[self.budget.attempts];
        }
    }
    pub fn healthy(&self, now: f64) -> bool {
        self.last_publication
            .is_some_and(|last| (0. ..=MAX_PUBLICATION_GAP).contains(&(now - last)))
    }
    /// Caller cancels any in-flight preparation when this returns true.
    pub fn observe(
        &mut self,
        now: f64,
        published: bool,
        work: f64,
        dropped: bool,
        task: bool,
    ) -> bool {
        let good = published
            && (0. ..=MAX_INTERNAL_WORK).contains(&work)
            && !dropped
            && self
                .last_publication
                .is_none_or(|last| (0. ..=MAX_PUBLICATION_GAP).contains(&(now - last)));
        self.last_publication = published.then_some(now);
        if !good {
            self.healthy_since = None;
            if task {
                self.stop("internal_delayed");
                return true;
            }
        } else if self.healthy_since.is_none() {
            self.healthy_since = Some(now);
        }
        false
    }
    pub fn healthy_for_preparation(&self, now: f64) -> bool {
        self.healthy(now)
            && self
                .healthy_since
                .is_some_and(|start| now - start >= HEALTH_SECONDS)
    }
    /// Persist the resulting budget before spawning the candidate worker.
    pub fn begin(&mut self, now: f64, present: bool, task: bool) -> bool {
        if self.state != State::Waiting
            || task
            || !present
            || !self.healthy(now)
            || !self
                .healthy_since
                .is_some_and(|start| now - start >= HEALTH_SECONDS)
            || now < self.next_attempt
        {
            return false;
        }
        if !std::mem::take(&mut self.first_start) {
            self.budget.attempts += 1;
        }
        self.state = State::Recovering;
        true
    }
    pub fn completed(&mut self, now: f64, retryable: bool) {
        if self.state == State::Stopped {
            return;
        }
        if self.budget.attempts >= DELAYS.len() || !retryable {
            self.stop(if self.budget.attempts >= DELAYS.len() {
                "attempts_exhausted"
            } else {
                "probe_failed"
            });
        } else {
            self.state = State::Waiting;
            self.next_attempt = now + DELAYS[self.budget.attempts];
        }
    }
}
