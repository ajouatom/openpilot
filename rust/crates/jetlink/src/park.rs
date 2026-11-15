//! "Parked" for the App's model pick also means in P with the ignition on.
//!
//! Carrot's daemon applies a model picked on the comma only while parked
//! (`IsOffroad and not IsOnroad`): a link that drops and rejoins on the road
//! must keep the model the App runs. Where the comma takes its power from the
//! car's harness (an HDA connector), turning the ignition off turns the comma
//! off too, so that moment never comes and a pick made in Carrot Web never
//! reaches the App. The car standing in P, stopped, with openpilot not engaged
//! is as safe a moment: during the upload and the App's build the comma drives
//! on its own model, and the App's model joins only by the usual gate.
use openpilot_cereal::{car_capnp::car_state, log_capnp::event};
use openpilot_messaging::{runtime::SubMaster, state::Options};
use std::{
    sync::{
        atomic::{AtomicBool, Ordering},
        Once,
    },
    thread,
    time::{Duration, Instant},
};

/// Older than this, a carState or selfdriveState says nothing.
const FRESH: Duration = Duration::from_millis(500);

static IN_PARK: AtomicBool = AtomicBool::new(false);

/// In P, stopped and not engaged, by carState and selfdriveState received in
/// the last 500 ms. False until [`spawn`] has heard both.
pub fn in_park() -> bool {
    IN_PARK.load(Ordering::Acquire)
}

/// The rule, apart from freshness.
pub fn decide(gear_park: bool, standstill: bool, v_ego: f32, engaged: bool) -> bool {
    gear_park && standstill && v_ego.abs() < 0.01 && !engaged
}

#[derive(Default)]
struct Seen {
    car: Option<(Instant, bool, bool, f32)>,
    selfdrive: Option<(Instant, bool)>,
}

impl Seen {
    fn parked(&self, now: Instant) -> bool {
        let fresh = |at: Instant| now.saturating_duration_since(at) < FRESH;
        match (self.car, self.selfdrive) {
            (Some((car_at, park, standstill, v_ego)), Some((selfdrive_at, engaged))) => {
                fresh(car_at) && fresh(selfdrive_at) && decide(park, standstill, v_ego, engaged)
            }
            _ => false,
        }
    }
}

fn watch() -> Result<(), String> {
    let mut sub = SubMaster::for_runtime(&["carState", "selfdriveState"], Options::default())
        .map_err(|e| e.to_string())?;
    let mut seen = Seen::default();
    loop {
        sub.update(Duration::from_millis(100))
            .map_err(|e| e.to_string())?;
        let now = Instant::now();
        let car = sub.state.topic("carState").map_err(|e| e.to_string())?;
        if car.updated && car.valid {
            if let Ok(event) = car.event() {
                if let Ok(event::Which::CarState(Ok(state))) = event.which() {
                    let park = state.get_gear_shifter() == Ok(car_state::GearShifter::Park);
                    seen.car = Some((now, park, state.get_standstill(), state.get_v_ego()));
                }
            }
        }
        let selfdrive = sub
            .state
            .topic("selfdriveState")
            .map_err(|e| e.to_string())?;
        if selfdrive.updated && selfdrive.valid {
            if let Ok(event) = selfdrive.event() {
                if let Ok(event::Which::SelfdriveState(Ok(state))) = event.which() {
                    seen.selfdrive = Some((now, state.get_enabled()));
                }
            }
        }
        IN_PARK.store(seen.parked(now), Ordering::Release);
    }
}

/// Starts the watcher once for the process. A failure only leaves the pick
/// for the next time the comma is offroad, as before.
pub fn spawn() {
    static START: Once = Once::new();
    START.call_once(|| {
        let started = thread::Builder::new()
            .name("jetlink-park".into())
            .spawn(|| loop {
                if let Err(error) = watch() {
                    IN_PARK.store(false, Ordering::Release);
                    eprintln!("jetlink park watcher: {error}");
                }
                thread::sleep(Duration::from_secs(2));
            });
        if let Err(error) = started {
            eprintln!("jetlink park watcher thread: {error}");
        }
    });
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn only_p_stopped_and_disengaged_counts() {
        assert!(decide(true, true, 0.0, false));
        assert!(!decide(false, true, 0.0, false), "D or N while stopped");
        assert!(!decide(true, false, 0.0, false), "not at standstill");
        assert!(!decide(true, true, 0.5, false), "still rolling");
        assert!(!decide(true, true, 0.0, true), "engaged");
    }

    #[test]
    fn both_messages_must_be_fresh() {
        let now = Instant::now();
        let old = now - Duration::from_secs(1);
        let mut seen = Seen::default();
        assert!(!seen.parked(now), "nothing heard yet");
        seen.car = Some((now, true, true, 0.0));
        assert!(!seen.parked(now), "no selfdriveState yet");
        seen.selfdrive = Some((now, false));
        assert!(seen.parked(now));
        seen.selfdrive = Some((old, false));
        assert!(!seen.parked(now), "stale selfdriveState");
        seen.selfdrive = Some((now, false));
        seen.car = Some((old, true, true, 0.0));
        assert!(!seen.parked(now), "stale carState");
    }
}
