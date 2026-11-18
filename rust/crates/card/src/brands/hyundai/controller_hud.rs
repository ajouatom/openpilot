use super::{
    cluster_hud::HudInput, controller_control::Command, controller_model::Model,
    controller_settings::Settings, lead, state::State, Error,
};
use openpilot_cereal::car_capnp::{car_control, car_state};

pub fn input(
    input: (car_control::Reader<'_>, &State, &Model),
    context: (&Settings, &Command, u32, i32, (bool, bool)),
) -> Result<HudInput, Error> {
    let (cc, state, model) = input;
    let (settings, command, frame, lane_check, (cluster_display, traffic_background)) = context;
    let candidate = state.config.candidate.as_str();
    let cs = state.out.get_root_as_reader::<car_state::Reader<'_>>()?;
    let hud = cc.get_hud_control()?;
    let display = lead::nearest(&model.leads);
    let hda = state
        .inputs
        .captured("lfahda_cluster")?
        .map(|data| super::wire::get(&data, "HDA_CntrlModSta"))
        .transpose()?
        .unwrap_or(0.);
    Ok(HudInput {
        frame,
        dm_alert: hud.get_driver_monitoring_alert(),
        enabled: cc.get_enabled(),
        active: cc.get_lat_active(),
        main_enabled: cs.get_cruise_state()?.get_available(),
        lat_enabled: cs.get_lat_enabled(),
        nav_active: hud.get_active_carrot() > 1,
        navi_available: cs.get_vehicle_navi_available(),
        hdp_use: super::parameters::setting_int(&state.settings, "HDPuse")?,
        set_speed: command.speed,
        gap: f64::from(hud.get_lead_distance_bars()),
        lead_visible: display.is_some(),
        lead_distance: display.map_or(0., |lead| lead.d_rel.clamp(0.1, 204.5)),
        paddle: state.paddle > 0,
        paddle_mode: settings.paddle,
        hda_mode: hda,
        steering_angle: f64::from(cs.get_steering_angle_deg()),
        soft_hold: u8::try_from(state.soft_hold).map_err(|_| Error::Numeric)?,
        trailer: state.trailer_connected,
        lane_check,
        lane_lines: [cs.get_left_lane_line(), cs.get_right_lane_line()],
        lane_visible: [hud.get_left_lane_visible(), hud.get_right_lane_visible()],
        lane_depart: [hud.get_left_lane_depart(), hud.get_right_lane_depart()],
        change_available: model.change_available,
        blindspot: [cs.get_left_blindspot(), cs.get_right_blindspot()],
        blinker: [cs.get_left_blinker(), cs.get_right_blinker()],
        desire: model.desire,
        standstill: cs.get_standstill(),
        ccnc: super::ccnc_display::CcncInput {
            display: cluster_display,
            traffic_background,
            traffic_state: hud.get_traffic_state(),
            desired_distance: f64::from(hud.get_desired_distance()),
            alert_type: hud
                .get_alert_type()
                .ok()
                .and_then(|text| text.to_str().ok())
                .unwrap_or_default()
                .to_owned(),
            lat_hud: hud.get_lat_hud_state(),
            lane_probs: model.lane_probs,
            non_dbc_car: super::ccnc_display::non_dbc_car(candidate),
            lane_wip_car: super::ccnc_display::lane_wip_car(candidate),
        },
    })
}
