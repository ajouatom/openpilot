use super::{request::route_url, Diagnostic, Endpoint, Pending, Ports, RouteEngine};
use crate::{
    geometry::{minimum_distance, Coordinate},
    json::{field, number},
    position::MAX_LATITUDE,
    Error,
};
use serde_json::Value;

/// navd.py's limits on a Mapbox answer.
const MAX_STEPS: usize = 2048;
const MAX_POINTS: usize = 50_000;

impl RouteEngine {
    pub fn recompute_route<P: Ports>(&mut self, ports: &mut P) -> Result<(), Error> {
        if self.last_position.is_none() || self.mapbox_token.is_empty() {
            return Ok(());
        }
        let (mut destination, place) = self.destination_from_params(ports)?;
        if place.as_ref().and_then(Value::as_str) == Some("External Navi") {
            ports.remove_parameter("NavDestination")?;
            ports.remove_memory_parameter("NavDestinationTemporary")?;
            destination = None;
        }
        let Some(destination) = destination else {
            self.announced = None;
            self.clear_route(ports)?;
            self.reset_recompute_limits();
            return Ok(());
        };
        let mut should_recompute = self.should_recompute()?;
        if self.nav_destination.as_ref().map(|value| value.point) != Some(destination.point) {
            if self.announced != Some(destination.point) {
                self.announced = Some(destination.point);
                ports.diagnostic(Diagnostic::NewDestination {
                    new: destination.clone(),
                    previous: self.nav_destination.clone(),
                    place,
                });
            }
            if self.nav_destination.is_some() {
                self.clear_route(ports)?;
                self.reset_recompute_limits();
            }
            should_recompute = true;
        }
        // Don't recompute when GPS drifts in tunnels.
        if !self.gps_ok && self.step_idx.is_some() {
            return Ok(());
        }
        if self.pending.is_none() && self.recompute_countdown == 0 && should_recompute {
            self.recompute_countdown = 2_u64.pow(self.recompute_backoff);
            self.recompute_backoff = (self.recompute_backoff + 1).min(6);
            self.calculate_route(destination, ports)?;
            self.reroute_counter = 0;
        } else {
            self.recompute_countdown = self.recompute_countdown.saturating_sub(1);
        }
        Ok(())
    }

    pub fn should_recompute(&mut self) -> Result<bool, Error> {
        let (Some(index), Some(route)) = (self.step_idx, self.route.as_ref()) else {
            return Ok(true);
        };
        let route = route.as_array().ok_or(Error::Field("route"))?;
        // Don't recompute in the last segment: assume the destination is reached.
        if route.len().checked_sub(1) == Some(index) {
            return Ok(false);
        }
        let geometry = self
            .route_geometry
            .as_ref()
            .and_then(|geometry| geometry.get(index))
            .ok_or(Error::Field("route_geometry"))?;
        let position = self
            .last_position
            .as_ref()
            .ok_or(Error::Field("last_position"))?
            .point;
        let mut minimum: f64 = 26.;
        for segment in geometry.windows(2) {
            let (a, b) = (segment[0], segment[1]);
            if a.distance_to(b)? < 1. {
                continue;
            }
            minimum = minimum.min(minimum_distance(a, b, position)?);
        }
        if minimum > 25. {
            self.reroute_counter = self.reroute_counter.saturating_add(1);
        } else {
            self.reroute_counter = 0;
        }
        Ok(self.reroute_counter > 3)
    }

    /// Starts the Directions request off the loop (navd.py's executor): the
    /// 1 Hz instruction publisher never waits on the network.
    pub fn calculate_route<P: Ports>(
        &mut self,
        destination: Endpoint,
        ports: &mut P,
    ) -> Result<(), Error> {
        if self.pending.is_some() || self.mapbox_token.is_empty() {
            return Ok(());
        }
        if self.nav_destination.as_ref().map(|value| value.point) != Some(destination.point) {
            self.clear_route(ports)?;
        }
        let from = self
            .last_position
            .as_ref()
            .ok_or(Error::Field("last_position"))?
            .clone();
        let waypoints = ports
            .parameter("NavDestinationWaypoints")?
            .unwrap_or_default();
        let Some(url) = route_url(
            ports,
            (&from, &destination),
            (
                &self.config,
                &self.mapbox_token,
                self.last_bearing,
                &waypoints,
            ),
        )?
        else {
            return Ok(());
        };
        ports.diagnostic(Diagnostic::Calculating {
            from,
            to: destination.clone(),
        });
        ports.submit(&url)?;
        self.pending = Some(Pending {
            destination,
            token: self.mapbox_token.clone(),
            waypoints,
        });
        Ok(())
    }

    /// Takes a finished request's route, unless what it was asked for has
    /// changed meanwhile.
    pub fn poll_route<P: Ports>(&mut self, ports: &mut P) -> Result<(), Error> {
        if self.pending.is_none() {
            return Ok(());
        }
        let Some(result) = ports.poll() else {
            return Ok(());
        };
        let Some(pending) = self.pending.take() else {
            return Ok(());
        };
        let (destination, _) = self.destination_from_params(ports)?;
        let waypoints = ports
            .parameter("NavDestinationWaypoints")?
            .unwrap_or_default();
        if destination.map(|value| value.point) != Some(pending.destination.point)
            || self.mapbox_token != pending.token
            || waypoints != pending.waypoints
        {
            return Ok(());
        }
        let response = match result {
            Ok(response) => response,
            Err(super::RequestError::Interrupted) => return Err(Error::Interrupted),
            Err(error) => {
                ports.diagnostic(Diagnostic::RequestFailed(error));
                return Ok(());
            }
        };
        let Some(route) = response
            .get("routes")
            .and_then(Value::as_array)
            .and_then(|routes| routes.first())
        else {
            ports.diagnostic(Diagnostic::EmptyRoute);
            return Ok(());
        };
        match parse_route(route) {
            Ok((steps, geometry)) => {
                self.route = Some(Value::Array(steps));
                self.route_geometry = Some(geometry);
                self.nav_destination = Some(pending.destination);
                self.step_idx = Some(0);
                if !pending.waypoints.is_empty() {
                    ports.remove_parameter("NavDestinationWaypoints")?;
                }
                ports.geometry_changed(self.route_coordinates())?;
                self.send_route(ports)
            }
            Err(error) => {
                ports.diagnostic(Diagnostic::InvalidRoute(error));
                Ok(())
            }
        }
    }
}

fn finite_nonnegative(step: &Value, name: &'static str) -> Result<(), Error> {
    let value = number(field(step, name)?, name)?;
    if value.is_finite() && value >= 0. {
        Ok(())
    } else {
        Err(Error::Field(name))
    }
}

/// navd.py `poll_route`'s checks and conversion: the first leg's steps
/// (1..=2048, at most 50 000 points, every coordinate finite and on the map,
/// distance/duration finite and not negative) and their geometry with the
/// `maxspeed` annotations in m/s.
pub fn parse_route(route: &Value) -> Result<(Vec<Value>, Vec<Vec<Coordinate>>), Error> {
    let leg = field(route, "legs")?
        .as_array()
        .and_then(|legs| legs.first())
        .ok_or(Error::Field("legs"))?;
    let mut steps = field(leg, "steps")?
        .as_array()
        .ok_or(Error::Field("steps"))?
        .clone();
    if steps.is_empty() || steps.len() > MAX_STEPS {
        return Err(Error::Field("step count"));
    }
    let empty = Vec::new();
    let maxspeeds = leg
        .get("annotation")
        .and_then(|annotation| annotation.get("maxspeed"))
        .and_then(Value::as_array)
        .unwrap_or(&empty);
    let mut geometry = Vec::with_capacity(steps.len());
    let mut speed_index = 0_usize;
    let mut total = 0_usize;
    for step in &mut steps {
        let points = field(field(step, "geometry")?, "coordinates")?
            .as_array()
            .ok_or(Error::Field("coordinates"))?;
        total += points.len();
        if total > MAX_POINTS || points.is_empty() {
            return Err(Error::GeometrySize);
        }
        let mut converted = Vec::with_capacity(points.len());
        for value in points {
            let pair = value.as_array().ok_or(Error::Field("coordinate"))?;
            let longitude = number(pair.first().ok_or(Error::Field("longitude"))?, "longitude")?;
            let latitude = number(pair.get(1).ok_or(Error::Field("latitude"))?, "latitude")?;
            if !latitude.is_finite()
                || !longitude.is_finite()
                || latitude.abs() > MAX_LATITUDE
                || longitude.abs() > 180.
            {
                return Err(Error::Field("coordinate"));
            }
            let mut point = Coordinate::new(latitude, longitude);
            if let Some(speed) = maxspeeds.get(speed_index) {
                let multiplier = match speed.get("unit").and_then(Value::as_str) {
                    Some("km/h") => Some(1. / 3.6),
                    Some("mph") => Some(1.609344 / 3.6),
                    _ => None,
                };
                if let (Some(multiplier), Some(value)) =
                    (multiplier, speed.get("speed").and_then(Value::as_f64))
                {
                    point.maxspeed = Some(value * multiplier);
                }
            }
            speed_index += 1;
            converted.push(point);
        }
        speed_index = speed_index.saturating_sub(1);
        finite_nonnegative(step, "distance")?;
        finite_nonnegative(step, "duration")?;
        geometry.push(converted);
        let object = step.as_object_mut().ok_or(Error::Field("step"))?;
        object
            .entry("bannerInstructions")
            .or_insert_with(|| Value::Array(Vec::new()));
        object.entry("duration_typical").or_insert(Value::Null);
    }
    Ok((steps, geometry))
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;

    fn route(points: Value) -> Value {
        json!({"legs": [{"steps": [
            {"distance": 10.0, "duration": 2.0, "geometry": {"coordinates": points}},
        ], "annotation": {"maxspeed": [{"speed": 50, "unit": "km/h"}, {"unknown": true}]}}]})
    }

    #[test]
    fn a_valid_route_gets_defaults_and_maxspeeds() {
        let (steps, geometry) =
            parse_route(&route(json!([[127.0, 37.0], [127.001, 37.0]]))).unwrap();
        assert_eq!(steps[0]["bannerInstructions"], json!([]));
        assert!(steps[0]["duration_typical"].is_null());
        assert!((geometry[0][0].maxspeed.unwrap() - 50. / 3.6).abs() < 1e-9);
        assert!(
            geometry[0][1].maxspeed.is_none(),
            "unknown speed is skipped"
        );
    }

    #[test]
    fn bad_coordinates_and_sizes_are_refused() {
        assert!(parse_route(&route(json!([[127.0, 91.0]]))).is_err());
        assert!(parse_route(&route(json!([]))).is_err());
        let mut negative = route(json!([[127.0, 37.0]]));
        negative["legs"][0]["steps"][0]["duration"] = json!(-1.0);
        assert!(parse_route(&negative).is_err());
        assert!(parse_route(&json!({"legs": [{"steps": []}]})).is_err());
    }
}
