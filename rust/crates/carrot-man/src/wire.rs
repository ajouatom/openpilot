use crate::{
    serv::{CarrotServ, Decision},
    Error,
};
use num_traits::ToPrimitive;
use openpilot_cereal::log_capnp::{event, nav_instruction};
use openpilot_navd::geometry::Coordinate;

fn integer(value: f64) -> Result<i32, Error> {
    value
        .trunc()
        .to_i32()
        .ok_or(Error::Contract("cereal Int32 conversion"))
}
fn float(value: f64) -> Result<f32, Error> {
    value
        .to_f32()
        .ok_or(Error::Contract("cereal Float32 conversion"))
}
fn count(value: usize) -> Result<u32, Error> {
    u32::try_from(value).map_err(|_| Error::Contract("cereal list size"))
}

pub fn route(coordinates: &[Coordinate], timestamp: u64) -> Result<Vec<u8>, Error> {
    let mut output = capnp::message::Builder::new_default();
    let mut event = output.init_root::<event::Builder>();
    event.set_log_mono_time(timestamp);
    event.set_valid(true);
    let mut list = event
        .init_nav_route()
        .init_coordinates(count(coordinates.len())?);
    for (i, coordinate) in coordinates.iter().enumerate() {
        let mut point = list.reborrow().get(count(i)?);
        point.set_latitude(float(coordinate.latitude)?);
        point.set_longitude(float(coordinate.longitude)?);
    }
    Ok(capnp::serialize::write_message_to_words(&output))
}

pub fn carrot(
    state: &CarrotServ,
    decision: &Decision,
    remote: &str,
    timestamp: u64,
    paths: &str,
) -> Result<Vec<u8>, Error> {
    if state.main_text_python_json.is_some() {
        return Err(Error::Contract("cereal main text UTF-8 encoding"));
    }
    if state.command.handler_failed || !state.command.command_text || !state.command.argument_text {
        return Err(Error::Contract("command publication text"));
    }
    let mut output = capnp::message::Builder::new_default();
    let mut event = output.init_root::<event::Builder>();
    event.set_log_mono_time(timestamp);
    event.set_valid(true);
    let mut carrot = event.init_carrot_man();
    carrot.set_active_carrot(
        i32::try_from(state.nav.active_carrot)
            .map_err(|_| Error::Contract("active carrot Int32"))?,
    );
    carrot.set_n_road_limit_speed(integer(decision.published_road_limit)?);
    carrot.set_remote(remote);
    carrot.set_x_spd_type(
        i32::try_from(state.nav.speed_type).map_err(|_| Error::Contract("speed type Int32"))?,
    );
    carrot.set_x_spd_limit(integer(state.nav.speed_limit)?);
    carrot.set_x_spd_dist(integer(state.nav.speed_distance)?);
    carrot.set_x_spd_count_down(
        i32::try_from(state.speed.left_speed_seconds)
            .map_err(|_| Error::Contract("speed countdown Int32"))?,
    );
    carrot.set_x_turn_info(
        i32::try_from(state.nav.turn_info).map_err(|_| Error::Contract("turn Int32"))?,
    );
    carrot.set_x_dist_to_turn(integer(state.nav.x_turn_distance)?);
    carrot.set_x_turn_count_down(
        i32::try_from(state.speed.left_turn_seconds)
            .map_err(|_| Error::Contract("turn countdown Int32"))?,
    );
    carrot.set_atc_type(decision.atc_type.as_str());
    carrot.set_v_turn_speed(integer(decision.vturn_speed)?);
    carrot.set_sz_pos_road_name(format!("{} {}", state.nav.road_name, decision.debug_text).trim());
    carrot.set_sz_t_b_t_main_text(state.nav.main_text.as_str());
    carrot.set_desired_speed(integer(decision.desired_speed)?);
    carrot.set_desired_source(decision.source.as_str());
    carrot.set_carrot_cmd_index(
        i32::try_from(state.command.command_index).map_err(|_| Error::Contract("command Int32"))?,
    );
    carrot.set_carrot_cmd(state.command.command.as_str());
    carrot.set_carrot_arg(state.command.argument.as_str());
    carrot.set_traffic_state(
        i32::try_from(state.traffic_state).map_err(|_| Error::Contract("traffic Int32"))?,
    );
    carrot.set_x_pos_angle(float(state.gps.bearing)?);
    carrot.set_x_pos_lat(float(state.gps.latitude)?);
    carrot.set_x_pos_lon(float(state.gps.longitude)?);
    carrot.set_x_pos_speed(float(decision.position_speed)?);
    carrot.set_position_fix_mono_time((state.gps.last_calculate * 1e9).max(0.) as u64);
    carrot.set_position_mono_time((state.gps.position_mono_time * 1e9).max(0.) as u64);
    carrot.set_position_source(state.gps.position_source);
    carrot.set_position_heading_valid(state.gps.position_heading_valid);
    carrot.set_n_go_pos_dist(integer(state.nav.goal_distance)?);
    carrot.set_n_go_pos_time(integer(state.nav.goal_time)?);
    carrot.set_sz_sdi_descr(state.sdi_description(decision).as_str());
    carrot.set_navi_paths(paths);
    carrot.set_left_sec(
        i32::try_from(state.speed.carrot_left_seconds)
            .map_err(|_| Error::Contract("left seconds Int32"))?,
    );
    carrot.set_vehicle_navi_active(decision.vehicle_display.0);
    carrot.set_vehicle_navi_speed(
        i32::try_from(decision.vehicle_display.1)
            .map_err(|_| Error::Contract("vehicle speed Int32"))?,
    );
    carrot.set_vehicle_navi_section_active(decision.vehicle_display.2);
    carrot.set_vehicle_navi_available(decision.vehicle_available);
    let selection = state.projected.as_ref().map(|p| &p.selection);
    let owner = selection.and_then(|s| s.snapshot.as_ref());
    carrot.set_navi_owner(owner.map_or("", |s| s.source.name()));
    carrot.set_navi_session_id(owner.map_or("", |s| s.session_id.as_str()));
    carrot.set_navi_sequence(owner.map_or(0, |s| s.sequence));
    carrot.set_navi_owner_age_ms(
        selection
            .and_then(|s| s.owner_age_s)
            .map_or(Ok(-1), |v| integer(v * 1000.))?,
    );
    carrot.set_navi_safety_age_ms(
        selection
            .and_then(|s| s.safety_age_s)
            .map_or(Ok(-1), |v| integer(v * 1000.))?,
    );
    carrot.set_navi_lifecycle(owner.map_or("idle", |s| match s.lifecycle {
        crate::sources::Lifecycle::Idle => "idle",
        crate::sources::Lifecycle::Guiding => "guiding",
        crate::sources::Lifecycle::Stopped => "stopped",
        crate::sources::Lifecycle::Arrived => "arrived",
    }));
    carrot.set_navi_control_allowed(owner.is_some() && state.has_control);
    carrot.set_navi_safety_rejection(decision.safety_rejection.as_str());
    carrot.set_decel_provider(decision.decel_provider.as_str());
    carrot.set_decel_reason(decision.source.as_str());
    Ok(capnp::serialize::write_message_to_words(&output))
}

/// carrot_serv `_external_navigation_connected`: connection freshness, not
/// active guidance.
fn external_navigation_connected(state: &CarrotServ) -> bool {
    state.navi_active || state.nav.active_count > 0 || state.nav.active_kisa_count > 0
}

pub fn instruction(
    state: &CarrotServ,
    decision: &Decision,
    fallback: Option<nav_instruction::Reader<'_>>,
    timestamp: u64,
) -> Result<Vec<u8>, Error> {
    let mut output = capnp::message::Builder::new_default();
    let mut event = output.init_root::<event::Builder>();
    event.set_log_mono_time(timestamp);
    event.set_valid(false);
    if state.nav.active_carrot > 1 && state.nav.active_kisa_count <= 0 {
        event.set_valid(true);
        let mut instruction = event.init_nav_instruction_carrot();
        instruction.set_distance_remaining(float(state.nav.goal_distance)?);
        instruction.set_time_remaining(float(state.nav.goal_time)?);
        instruction.set_time_remaining_typical(float(state.nav.goal_time)?);
        instruction.set_speed_limit(float(if decision.published_road_limit > 0. {
            decision.published_road_limit / 3.6
        } else {
            0.
        })?);
        instruction.set_maneuver_distance(float(state.nav.turn_distance)?);
        let secondary = if state.nav.far_direction.is_empty() {
            state.nav.near_direction.clone()
        } else {
            format!("{}[{}]", state.nav.near_direction, state.nav.far_direction)
        };
        instruction.set_maneuver_secondary_text(secondary.as_str());
        instruction.set_maneuver_primary_text(state.nav.main_text.as_str());
        let current = crate::serv::turn_mapping(state.nav.turn_type, true);
        let next = crate::serv::turn_mapping(state.nav.next_turn_type, true);
        instruction.set_maneuver_type(current.0);
        instruction.set_maneuver_modifier(current.1);
        let length = if state.nav.turn_type < 0 {
            0
        } else if state.nav.next_turn_distance >= state.nav.turn_distance {
            2
        } else {
            1
        };
        let mut maneuvers = instruction.init_all_maneuvers(length);
        if length > 0 {
            let mut item = maneuvers.reborrow().get(0);
            item.set_distance(float(state.nav.x_turn_distance)?);
            item.set_type(current.0);
            item.set_modifier(current.1);
        }
        if length > 1 {
            let mut item = maneuvers.reborrow().get(1);
            item.set_distance(float(state.nav.next_turn_distance)?);
            item.set_type(next.0);
            item.set_modifier(next.1);
        }
    } else if let Some(fallback) = fallback.filter(|_| !external_navigation_connected(state)) {
        // carrot_serv.py: with no external navigation connected, navd's
        // (Mapbox) instruction is republished, valid, so a destination set on
        // Carrot Web drives the N16 navigation inputs without a phone app.
        event.set_valid(true);
        event.set_nav_instruction_carrot(fallback)?;
    } else {
        event.init_nav_instruction_carrot();
    }
    Ok(capnp::serialize::write_message_to_words(&output))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn serv() -> CarrotServ {
        let settings = serde_json::from_value(serde_json::json!({"bump_speed":0.,"bump_time":0.,"bump_end_distance":0.,"control_end":0.,"rear_hold_distance":0.,"control_mode":0,"vehicle_can_control":0,"school_control":false,"camera_control_mode":0,"safety_factor":0.,"deceleration":0.,"countdown_mode":0,"turn_speed_mode":0,"map_turn_factor":0.,"auto_turn_speed":0.,"auto_turn_map_change":0,"auto_turn_control":0,"auto_turn_end":0.,"curve_lower_limit":0.,"is_metric":true,"road_limit_offset":0.,"curve_factor":0.,"language":"en"})).unwrap();
        CarrotServ::new(settings)
    }

    /// navd's instruction stands in only while no external navigation is
    /// connected, as carrot_serv's `elif not _external_navigation_connected()`.
    #[test]
    fn navd_instruction_only_without_external_navigation() {
        let mut state = serv();
        assert!(!external_navigation_connected(&state));
        state.navi_active = true;
        assert!(external_navigation_connected(&state));
        state.navi_active = false;
        state.nav.active_count = 1;
        assert!(external_navigation_connected(&state));
        state.nav.active_count = 0;
        state.nav.active_kisa_count = 1;
        assert!(external_navigation_connected(&state));
    }
}
