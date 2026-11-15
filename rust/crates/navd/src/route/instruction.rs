use super::{Diagnostic, Instruction, Maneuver, Ports, RouteEngine, SpeedLimitSign};
use crate::{
    geometry::distance_along_geometry,
    instructions::parse_banner_instructions,
    json::{field, number},
    Error,
};
use serde_json::Value;

impl RouteEngine {
    pub fn send_instruction<P: Ports>(&mut self, ports: &mut P) -> Result<(), Error> {
        let Some(index) = self.step_idx else {
            return ports.instruction(&Instruction::default());
        };
        let route = self
            .route
            .as_ref()
            .and_then(Value::as_array)
            .ok_or(Error::Field("route"))?;
        let step = route.get(index).ok_or(Error::Field("step_idx"))?;
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
        let along = distance_along_geometry(geometry, position)?;
        let distance = number(field(step, "distance")?, "distance")?;
        let distance_to_maneuver = distance - along;
        let mut banner_step = step;
        if field(step, "bannerInstructions")?
            .as_array()
            .ok_or(Error::Field("bannerInstructions"))?
            .is_empty()
            && index + 1 == route.len()
        {
            banner_step = &route[index.saturating_sub(1)];
        }
        let banner = parse_banner_instructions(
            field(banner_step, "bannerInstructions")?,
            distance_to_maneuver,
        )?;
        let mut maneuvers = Vec::new();
        for (next, step) in route.iter().enumerate() {
            let distance = if next < index {
                let prefix = &route[next + 1..index];
                let previous = if prefix.is_empty() {
                    0.
                } else {
                    -sum_distances(prefix)?
                };
                previous - along
            } else if next == index {
                distance_to_maneuver
            } else {
                distance_to_maneuver + sum_distances(&route[index + 1..=next])?
            };
            if let Some(instruction) =
                parse_banner_instructions(field(step, "bannerInstructions")?, distance)?
            {
                maneuvers.push(Maneuver {
                    distance,
                    kind: instruction.maneuver_type,
                    modifier: instruction.maneuver_modifier,
                });
            }
        }
        let remaining = 1. - along / if distance < 1. { 1. } else { distance };
        let mut distance_remaining = distance * remaining;
        let mut time_remaining = number(field(step, "duration")?, "duration")? * remaining;
        let typical = field(step, "duration_typical")?;
        let mut time_remaining_typical = if typical.is_null() {
            time_remaining
        } else {
            number(typical, "duration_typical")? * remaining
        };
        for step in &route[index + 1..] {
            distance_remaining += number(field(step, "distance")?, "distance")?;
            let duration = number(field(step, "duration")?, "duration")?;
            time_remaining += duration;
            let typical = field(step, "duration_typical")?;
            time_remaining_typical += if typical.is_null() {
                duration
            } else {
                number(typical, "duration_typical")?
            };
        }
        let mut closest = 0;
        let mut closest_distance = geometry[0].distance_to(position)?;
        for (point_index, point) in geometry.iter().enumerate().skip(1) {
            let distance = point.distance_to(position)?;
            if distance < closest_distance {
                closest_distance = distance;
                closest = point_index;
            }
        }
        if closest > 0 && along < distance_along_geometry(geometry, geometry[closest])? {
            closest -= 1;
        }
        let speed_limit = geometry[closest].maxspeed.filter(|_| self.localizer_valid);
        if let Some(speed) = speed_limit {
            ports.diagnostic(Diagnostic::SpeedLimit(speed));
        }
        let speed_limit_sign = match step.get("speedLimitSign").and_then(Value::as_str) {
            Some("mutcd") => Some(SpeedLimitSign::Mutcd),
            Some("vienna") => Some(SpeedLimitSign::Vienna),
            _ => None,
        };
        ports.instruction(&Instruction {
            valid: true,
            maneuver_distance: distance_to_maneuver,
            banner,
            maneuvers,
            distance_remaining,
            time_remaining,
            time_remaining_typical,
            speed_limit,
            speed_limit_sign,
        })?;
        if distance_to_maneuver < -10. {
            if index + 1 < route.len() {
                self.step_idx = Some(index + 1);
                self.reset_recompute_limits();
            } else {
                ports.diagnostic(Diagnostic::DestinationReached);
                if self
                    .nav_destination
                    .as_ref()
                    .ok_or(Error::Field("nav_destination"))?
                    .point
                    .distance_to(position)?
                    > 25.
                {
                    ports.remove_parameter("NavDestination")?;
                    ports.remove_memory_parameter("NavDestinationTemporary")?;
                    self.clear_route(ports)?;
                }
            }
        }
        Ok(())
    }
}

fn sum_distances(steps: &[Value]) -> Result<f64, Error> {
    let mut total = 0_f64;
    let mut correction = 0_f64;
    for step in steps {
        let value = number(field(step, "distance")?, "distance")?;
        let next = total + value;
        correction += if total.abs() >= value.abs() {
            (total - next) + value
        } else {
            (value - next) + total
        };
        total = next;
    }
    Ok(if correction != 0. && correction.is_finite() {
        total + correction
    } else {
        total
    })
}
