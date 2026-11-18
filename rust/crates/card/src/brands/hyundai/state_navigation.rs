use super::{
    navigation::{Input, Output},
    parser_inputs::Channel,
    state::State,
    state_fields::{float32, timestamp},
    wire::Values,
    Error,
};
use openpilot_cereal::car_capnp::car_state;

pub struct NavigationData {
    hda: Option<Values>,
    position: Option<Values>,
    segment: Option<Values>,
    profile: Option<Values>,
    status: Option<Values>,
    profile_timestamp: u64,
    position_timestamp: u64,
    segment_timestamp: u64,
    hda_timestamp: u64,
    status_timestamp: u64,
    last_update: u64,
    pt_timeout: bool,
    alt_timeout: bool,
    hda_size: usize,
    status_size: usize,
    metric: bool,
}

impl NavigationData {
    pub fn input(&self) -> Input<'_> {
        Input {
            hda: self.hda.as_ref(),
            position: self.position.as_ref(),
            segment: self.segment.as_ref(),
            profile: self.profile.as_ref(),
            status: self.status.as_ref(),
            profile_timestamp: self.profile_timestamp,
            position_timestamp: self.position_timestamp,
            segment_timestamp: self.segment_timestamp,
            hda_timestamp: self.hda_timestamp,
            status_timestamp: self.status_timestamp,
            last_update: self.last_update,
            pt_timeout: self.pt_timeout,
            alt_timeout: self.alt_timeout,
            hda_size: self.hda_size,
            status_size: self.status_size,
            metric: self.metric,
        }
    }
}

pub fn data(state: &State) -> Result<NavigationData, Error> {
    Ok(NavigationData {
        hda: state.inputs.captured("hda_info_4a3")?,
        position: state.inputs.captured("navi_position_4b4")?,
        segment: state.inputs.captured("navi_segment_4b9")?,
        profile: state.inputs.captured("navi_profile_4be")?,
        status: state.inputs.captured("navi_status_380")?,
        profile_timestamp: timestamp(
            state,
            Channel::Pt,
            if state.navigation.wrapped || state.navigation.ev4 {
                "CANFD_NAVI_PROFILE_093"
            } else {
                "NEW_MSG_4BE"
            },
            None,
        ),
        position_timestamp: timestamp(state, Channel::Pt, "NEW_MSG_4B4", None),
        segment_timestamp: timestamp(
            state,
            Channel::Pt,
            if state.navigation.ev4 {
                "CANFD_NAVI_SEGMENT_099"
            } else {
                "NEW_MSG_4B9"
            },
            None,
        ),
        hda_timestamp: timestamp(state, Channel::Pt, "CANFD_HDA_INFO_364", None),
        status_timestamp: timestamp(state, Channel::Alt, "CANFD_NAVI_STATUS_380", None),
        last_update: state.inputs.pt.last_update,
        pt_timeout: state.inputs.pt.bus_timeout(),
        alt_timeout: state
            .inputs
            .alt
            .as_ref()
            .is_some_and(openpilot_can::parser::Parser::bus_timeout),
        hda_size: state.inputs.pt.raw.get(&0x364).map_or(0, Vec::len),
        status_size: state
            .inputs
            .alt
            .as_ref()
            .and_then(|parser| parser.raw.get(&0x380))
            .map_or(0, Vec::len),
        metric: state.metric,
    })
}

pub fn update(
    state: &mut State,
    mut ret: car_state::Builder<'_>,
    camera: bool,
    canfd: bool,
    accel_rising: bool,
) -> Result<(), Error> {
    let data = data(state)?;
    let input = data.input();
    let changed = state.navigation.refresh(&state.settings)?;
    if canfd && state.navigation.params_counter == 0 {
        state.navigation_config = super::navigation::policy::Config::read(
            &state.settings,
            &mut state.navigation.camera_policy,
        )?;
    }
    let mut output = Output {
        speed_limit: f64::from(ret.reborrow_as_reader().get_speed_limit()),
        ..Output::default()
    };
    let warning = (camera, output.speed_limit);
    let camera = if canfd {
        state
            .navigation
            .update_events(&input, &mut output, camera)?
            || camera
    } else {
        camera
    };
    let context = super::navigation::policy::Context {
        accel_rising,
        gas_tap: if canfd {
            state
                .navigation_policy
                .gas_tap(ret.reborrow_as_reader().get_gas_pressed())
        } else {
            false
        },
        engaged: state.speedcam_engaged,
        cruise_kph: state.speedcam_v_cruise,
        v_ego: f64::from(ret.reborrow_as_reader().get_v_ego()),
        cluster_ratio: f64::from(ret.reborrow_as_reader().get_v_clu_ratio()),
    };
    let camera = if canfd {
        let camera = state.navigation_policy.camera(
            &mut state.navigation,
            &output,
            camera,
            warning,
            context,
            &state.navigation_config,
        )?;
        state.navigation_policy.bump(
            &mut state.navigation,
            &mut output,
            context,
            &state.navigation_config,
        );
        camera
    } else {
        camera
    };
    state.navigation.speed_limit(
        &mut output,
        &input,
        camera,
        f64::from(ret.reborrow_as_reader().get_v_ego()),
        changed,
    );
    if canfd {
        state.navigation_long_press = state.navigation_policy.decel_long_press(
            state.cruise_buttons.back().copied().unwrap_or(0),
            &state.navigation_config,
        );
        state.navigation_policy.section(
            &state.navigation,
            &mut output,
            context,
            state.navigation_long_press,
            &state.navigation_config,
        );
        state.speedcam_gas_tok_consumed = state.navigation_policy.gas_consumed;
    }
    ret.set_speed_limit(float32(output.speed_limit)?);
    ret.set_speed_limit_distance(float32(output.speed_limit_distance)?);
    if canfd {
        ret.set_speed_bump_distance(float32(output.bump_distance)?);
        ret.set_school_zone_active(output.school_zone);
        ret.set_vehicle_navi_active(output.active);
        ret.set_vehicle_navi_section_active(output.section_active);
        ret.set_vehicle_navi_speed(float32(output.speed)?);
        ret.set_vehicle_navi_available(output.available);
    }
    Ok(())
}
