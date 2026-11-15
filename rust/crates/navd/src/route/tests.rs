//! The engine against navd.py's update order, with ports in memory.
use super::*;
use crate::position::NavigationPosition;
use std::collections::HashMap;

#[derive(Default)]
struct Fake {
    params: HashMap<String, String>,
    memory: HashMap<String, String>,
    submitted: Vec<String>,
    answer: Option<Result<Value, RequestError>>,
    instructions: Vec<bool>,
    routes: Vec<usize>,
}

impl Ports for Fake {
    fn parameter(&mut self, name: &str) -> Result<Option<String>, Error> {
        Ok(self.params.get(name).cloned())
    }
    fn remove_parameter(&mut self, name: &str) -> Result<(), Error> {
        self.params.remove(name);
        Ok(())
    }
    fn memory_parameter(&mut self, name: &str) -> Result<Option<String>, Error> {
        Ok(self.memory.get(name).cloned())
    }
    fn remove_memory_parameter(&mut self, name: &str) -> Result<(), Error> {
        self.memory.remove(name);
        Ok(())
    }
    fn submit(&mut self, url: &str) -> Result<(), Error> {
        self.submitted.push(url.to_owned());
        Ok(())
    }
    fn poll(&mut self) -> Option<Result<Value, RequestError>> {
        self.answer.take()
    }
    fn instruction(&mut self, message: &Instruction) -> Result<(), Error> {
        self.instructions.push(message.valid);
        Ok(())
    }
    fn route(&mut self, coordinates: &[Coordinate]) -> Result<(), Error> {
        self.routes.push(coordinates.len());
        Ok(())
    }
    fn diagnostic(&mut self, _event: Diagnostic) {}
}

fn here() -> NavigationPosition {
    NavigationPosition {
        latitude: 37.0,
        longitude: 127.0,
        bearing: 90.,
        speed: 0.,
        time: 0.,
        fix_time: 0.,
    }
}

fn tick(now: f64, external: bool) -> Tick {
    Tick {
        now,
        position: Some(here()),
        external,
        token: "pk.test".into(),
    }
}

fn answer() -> Value {
    serde_json::json!({"routes": [{"legs": [{"steps": [
        {"distance": 500.0, "duration": 50.0, "geometry": {"coordinates": [[127.0, 37.0], [127.005, 37.0]]},
         "bannerInstructions": []},
        {"distance": 0.0, "duration": 0.0, "geometry": {"coordinates": [[127.005, 37.0]]}},
    ]}]}]})
}

fn engine(ports: &mut Fake) -> RouteEngine {
    RouteEngine::new(
        Config {
            host: "https://api.mapbox.com".into(),
        },
        ports,
    )
    .unwrap()
}

#[test]
fn the_temporary_destination_wins_and_the_request_runs_off_the_loop() {
    let mut ports = Fake::default();
    ports.params.insert(
        "NavDestination".into(),
        r#"{"latitude": 35.0, "longitude": 129.0}"#.into(),
    );
    ports.memory.insert(
        "NavDestinationTemporary".into(),
        r#"{"latitude": 37.0, "longitude": 127.005, "place_name": "Web"}"#.into(),
    );
    let mut engine = engine(&mut ports);
    engine.update(tick(0., false), &mut ports).unwrap();
    assert_eq!(ports.submitted.len(), 1);
    assert!(
        ports.submitted[0].contains("127.005,37"),
        "{}",
        ports.submitted[0]
    );
    assert!(ports.submitted[0].contains("language=en"));
    // While it runs, the loop publishes and asks for nothing more.
    engine.update(tick(1., false), &mut ports).unwrap();
    assert_eq!(ports.submitted.len(), 1);
    assert!(engine.step_idx.is_none());
    ports.answer = Some(Ok(answer()));
    engine.update(tick(2., false), &mut ports).unwrap();
    assert_eq!(engine.step_idx, Some(0));
    assert_eq!(ports.instructions.last(), Some(&true));
}

#[test]
fn a_result_for_an_older_destination_is_dropped() {
    let mut ports = Fake::default();
    ports.memory.insert(
        "NavDestinationTemporary".into(),
        r#"{"latitude": 37.0, "longitude": 127.005}"#.into(),
    );
    let mut engine = engine(&mut ports);
    engine.update(tick(0., false), &mut ports).unwrap();
    ports.memory.insert(
        "NavDestinationTemporary".into(),
        r#"{"latitude": 36.0, "longitude": 127.5}"#.into(),
    );
    ports.answer = Some(Ok(answer()));
    engine.update(tick(1., false), &mut ports).unwrap();
    assert!(engine.route.is_none());
}

#[test]
fn an_external_navigation_silences_navd_and_drops_its_route_after() {
    let mut ports = Fake::default();
    ports.memory.insert(
        "NavDestinationTemporary".into(),
        r#"{"latitude": 37.0, "longitude": 127.005}"#.into(),
    );
    let mut engine = engine(&mut ports);
    engine.update(tick(0., false), &mut ports).unwrap();
    ports.answer = Some(Ok(answer()));
    engine.update(tick(1., false), &mut ports).unwrap();
    assert!(engine.route.is_some());
    engine.update(tick(2., true), &mut ports).unwrap();
    assert_eq!(ports.instructions.last(), Some(&false));
    engine.update(tick(3., false), &mut ports).unwrap();
    // navd.py: the route is cleared and asked for again in the same update.
    assert!(engine.route.is_none());
    assert_eq!(ports.submitted.len(), 2);
}

#[test]
fn no_token_means_no_request_and_no_position_an_invalid_instruction() {
    let mut ports = Fake::default();
    ports.memory.insert(
        "NavDestinationTemporary".into(),
        r#"{"latitude": 37.0, "longitude": 127.005}"#.into(),
    );
    let mut engine = engine(&mut ports);
    let mut quiet = tick(0., false);
    quiet.token.clear();
    engine.update(quiet, &mut ports).unwrap();
    assert!(ports.submitted.is_empty());
    let mut lost = tick(1., false);
    lost.position = None;
    engine.update(lost, &mut ports).unwrap();
    assert_eq!(ports.instructions.last(), Some(&false));
}

#[test]
fn bad_waypoints_send_no_request() {
    let mut ports = Fake::default();
    ports.memory.insert(
        "NavDestinationTemporary".into(),
        r#"{"latitude": 37.0, "longitude": 127.005}"#.into(),
    );
    ports
        .params
        .insert("NavDestinationWaypoints".into(), "[[200, 37]]".into());
    let mut engine = engine(&mut ports);
    engine.update(tick(0., false), &mut ports).unwrap();
    assert!(ports.submitted.is_empty());
}
