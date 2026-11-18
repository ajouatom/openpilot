//! N16 navigation input for a jetlink navigation graph: carrot's
//! `map_client.MapInput` `navigation` half. Subscribed only while such a
//! graph is connected: navInstructionCarrot is invalid without a route and
//! must not join modeld's ordinary input checks.
use crate::Error;
use openpilot_cereal::log_capnp::event;
use openpilot_messaging::{runtime::SubMaster, state::Options as SubscriberOptions};
use openpilot_modeld::navigation::{navigation_input, Maneuver, NAVIGATION_LEN};
use std::time::Duration;

#[derive(Default)]
pub struct NavigationInput {
    subscriber: Option<SubMaster>,
    demand_at: f64,
}

/// mapsd renders the route map only while a navigation graph asks for it.
const DEMAND_PATH: &str = "/dev/shm/carrot-nav-map-demand";

impl NavigationInput {
    /// Zeros unless `wanted` and a fresh (< 2 s) valid instruction exists.
    pub fn update(&mut self, wanted: bool, speed: f64) -> Result<[f32; NAVIGATION_LEN], Error> {
        if !wanted {
            self.subscriber = None;
            return Ok([0.; NAVIGATION_LEN]);
        }
        let subscriber = match &mut self.subscriber {
            Some(subscriber) => subscriber,
            None => self.subscriber.insert(SubMaster::for_runtime(
                &["navInstructionCarrot"],
                SubscriberOptions::default(),
            )?),
        };
        subscriber.update(Duration::ZERO)?;
        let now = crate::clock::monotonic();
        if now - self.demand_at >= 1. {
            self.demand_at = now;
            let _ = std::fs::write(DEMAND_PATH, b"1");
        }
        let topic = subscriber.state.topic("navInstructionCarrot")?;
        let age = now - topic.receive_time;
        if !topic.seen || !topic.valid || !(0. ..2.).contains(&age) {
            return Ok([0.; NAVIGATION_LEN]);
        }
        let event::Which::NavInstructionCarrot(instruction) = topic.event()?.which()? else {
            return Ok([0.; NAVIGATION_LEN]);
        };
        let maneuvers = instruction?.get_all_maneuvers()?;
        let mut owned = Vec::with_capacity(maneuvers.len() as usize);
        for maneuver in maneuvers.iter() {
            owned.push((
                maneuver.get_distance(),
                maneuver.get_type()?.to_str().unwrap_or("").to_owned(),
                maneuver.get_modifier()?.to_str().unwrap_or("").to_owned(),
            ));
        }
        let traveled = (speed.max(0.) * age) as f32;
        Ok(navigation_input(
            owned.iter().map(|(distance, kind, modifier)| Maneuver {
                distance: *distance,
                kind,
                modifier,
            }),
            traveled,
        ))
    }
}
