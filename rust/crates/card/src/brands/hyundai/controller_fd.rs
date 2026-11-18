use super::{
    canfd_acc::{self, AccInput, SccState},
    canfd_buttons::{self, ForwardInput},
    canfd_cluster::{self, ClusterInput, ClusterMessages},
    canfd_maintenance,
    canfd_steering::{self, SteeringInput, SteeringMessages},
    controller::Controller,
    controller_control::Command,
    controller_hud,
    controller_model::Model,
    flags as f, lead,
    state::State,
    wire::Values,
    Error,
};
use openpilot_can::Frame;
use openpilot_cereal::car_capnp::{car_control, car_state};
use std::collections::BTreeMap;

pub fn messages(
    c: &mut Controller,
    input: (car_control::Reader<'_>, &State),
    context: (&Command, &Model, &BTreeMap<&str, Values>),
) -> Result<Vec<Frame>, Error> {
    let (cc, state) = input;
    let (command, model, captures) = context;
    let config = &state.config;
    let camera = config.flags & f::CAMERA_SCC != 0;
    let hda2 = config.flags & f::HDA2 != 0;
    let steering = SteeringInput {
        bus: config.bus,
        flags: config.flags,
        frame: c.frame,
        enabled: cc.get_enabled(),
        lat_active: command.request,
        cc_lat_active: cc.get_lat_active(),
        longitudinal: config.longitudinal,
        torque: command.torque,
        angle: command.angle,
        max_torque: command.authority,
        angle_control: config.flags & f::ANGLE_CONTROL != 0,
    };
    let mut result = if camera {
        canfd_steering::camera_steering(
            &mut c.writer,
            &steering,
            &SteeringMessages {
                mdps: captures.get("mdps").cloned(),
                touch: captures.get("steer_touch_2af").cloned(),
                lfa: captures.get("lfa").cloned(),
                lfa_alt: captures.get("lfa_alt").cloned(),
                adrv_161: captures.get("adrv_0x161").cloned(),
            },
        )?
    } else {
        canfd_steering::steering(&mut c.writer, &steering)?
    };
    if c.frame.is_multiple_of(5) && hda2 && !camera {
        result.extend(canfd_maintenance::suppress_lfa(
            &mut c.writer,
            config.bus,
            (captures.get("cam_0x362"), captures.get("cam_0x2a4")),
        )?);
    }
    if c.frame.is_multiple_of(5) && (!hda2 || config.longitudinal || camera) {
        result.extend(canfd_cluster::lfa_cluster(
            &mut c.writer,
            config.bus,
            captures.get("lfahda_cluster"),
            [cc.get_long_active(), cc.get_lat_active()],
            &canfd_cluster::LfaClusterAlert {
                dm_alert: if captures.get("adrv_0x161").is_none()
                    || (camera && !config.longitudinal)
                {
                    cc.get_hud_control()?.get_driver_monitoring_alert()
                } else {
                    0
                },
                suppress: camera
                    && matches!(
                        config.candidate.as_str(),
                        "GENESIS_GV70_1ST_GEN" | "KIA_SORENTO_HEV_4TH_GEN"
                    ),
                lfa: captures.get("lfa"),
                mdps: captures.get("mdps"),
                scc: captures.get("scc_control"),
            },
        )?);
        if !camera {
            result.extend(canfd_cluster::lfa_icon(
                &mut c.writer,
                config.bus,
                captures.get("adrv_0x161"),
                [
                    cc.get_lat_active(),
                    state
                        .out
                        .get_root_as_reader::<car_state::Reader<'_>>()?
                        .get_lat_enabled(),
                ],
                cc.get_hud_control()?.get_driver_monitoring_alert(),
            )?);
        }
    }
    if hda2 && config.flags & f::ENABLE_BLINKERS != 0 {
        result.extend(canfd_maintenance::spas(
            &mut c.writer,
            config.bus,
            [cc.get_left_blinker(), cc.get_right_blinker()],
        )?);
    }
    if matches!(c.settings.camera, 2 | 3) {
        c.buttons.toggle(cc, state)?;
    }
    if config.longitudinal {
        let lateral = c.lateral.update(&model.leads, (&model.x, &model.y))?;
        c.jerk(input, command)?;
        if c.frame.is_multiple_of(100) {
            c.lane_check = super::parameters::setting_int(&state.settings, "LaneLineCheck")?;
            c.ccnc_keys = (
                super::parameters::setting_int(&state.settings, "CcncClusterDisplay")? > 0,
                super::parameters::setting_int(&state.settings, "CcncTrafficBackground")? > 0,
            );
        }
        let cs = state.out.get_root_as_reader::<car_state::Reader<'_>>()?;
        let blink_hold = if c.frame.is_multiple_of(5) && captures.get("adrv_0x1ea").is_some() {
            // The lane-change lamp hold is not started or kept while the
            // cruise CANCEL latch is set; an explicit lever request still is.
            let hold_desire = if cs.get_cruise_cancel_state() {
                0
            } else {
                model.desire
            };
            let hold = c
                .blink_hold
                .update(hold_desire, model.lane_pair, model.blinker_hold);
            // An external physical hold wins (hyundaicanfd.py `external_hold`).
            match c.external_hold {
                1 => 0,
                2 => 3,
                3 => 4,
                _ => hold,
            }
        } else {
            0
        };
        let cluster = ClusterInput {
            bus: config.bus,
            flags: config.flags,
            hud: controller_hud::input(
                (cc, state, model),
                (&c.settings, command, c.frame, c.lane_check, c.ccnc_keys),
            )?,
            main_mode: state.main_mode,
            acc_mode: state.acc_mode,
            speed: f64::from(cs.get_v_ego()),
            stopping: command.stopping,
            interlock: cs.get_brake_hold_active() || cs.get_parking_brake(),
            button_name: config.button_message(),
            lane_changing: blink_hold,
            scc_main_ready: cs.get_can_valid()
                && cs.get_gear_shifter()? == car_state::GearShifter::Drive
                && !cs.get_brake_pressed(),
            corner_radar: c.settings.corner,
            debug: c.settings.debug,
            leads: &model.leads,
            path: (&model.x, &model.y),
            hud_lateral: Some(lateral),
            lane_warnings: [command.lane_warning[0] != 0., command.lane_warning[1] != 0.],
            set_kph: f64::from(cs.get_v_cruise_cluster()),
            v_ego_cluster: f64::from(cs.get_v_ego_cluster()),
            metric: state.metric,
        };
        let source = ClusterMessages {
            lfahda: captures.get("lfahda_cluster"),
            adrv161: captures.get("adrv_0x161"),
            adrv200: captures.get("adrv_0x200"),
            adrv1ea: captures.get("adrv_0x1ea"),
            ccnc162: captures.get("ccnc_0x162"),
            buttons: captures.get("cruise_buttons_msg"),
            buttons_alt2: captures.get("cruise_buttons_alt2"),
            scc: captures.get("scc_control"),
        };
        result.extend(canfd_cluster::messages(
            &mut c.writer,
            &mut c.cluster,
            &cluster,
            &source,
        )?);
        result.extend(if hda2 {
            canfd_maintenance::adrv(&mut c.writer, (config.bus, config.flags, c.frame))?
        } else {
            canfd_maintenance::fca_warning(&mut c.writer, (config.bus, config.flags, c.frame))?
        });
        if c.frame.is_multiple_of(2) {
            let wheel = cs.get_wheel_speeds()?;
            let cruise = cs.get_cruise_state()?;
            let scc = SccState {
                original: captures.get("scc_control"),
                wheels: [
                    wheel.get_fl(),
                    wheel.get_fr(),
                    wheel.get_rl(),
                    wheel.get_rr(),
                ]
                .map(f64::from),
                v_ego: f64::from(cs.get_v_ego()),
                v_ego_raw: f64::from(cs.get_v_ego_raw()),
                a_ego: f64::from(cs.get_a_ego()),
                brake: cs.get_brake_pressed(),
                gas: cs.get_gas_pressed(),
                brake_hold: cs.get_brake_hold_active(),
                parking_brake: cs.get_parking_brake(),
                can_valid: cs.get_can_valid(),
                can_timeout: cs.get_can_timeout(),
                drive: cs.get_gear_shifter()? == car_state::GearShifter::Drive,
                available: cruise.get_available(),
                standstill: cs.get_standstill(),
                paddle: if state.paddle_hold > 0 {
                    state.paddle.max(1)
                } else {
                    state.paddle
                },
                soft_hold: state.soft_hold != 0,
                scc_hold: state.scc_hold,
            };
            let i = AccInput {
                bus: config.bus.ecan,
                enabled: cc.get_enabled(),
                accel_last: c.accel_last,
                value_last: c.value_last,
                accel: command.accel,
                stopping: command.stopping,
                gas_override: cc.get_cruise_control()?.get_override(),
                set_speed: command.speed,
                gap: f64::from(cc.get_hud_control()?.get_lead_distance_bars()),
                jerk_u: c.jerk.jerk_u,
                jerk_l: c.jerk.jerk_l,
                carrot_cruise: c.jerk.carrot_cruise,
                carrot_accel: c.jerk.carrot_accel,
                lead: lead::scc_fields(&model.leads, (&model.x, &model.y), Some(lateral))?,
                signal_stop_object: cc.get_hud_control()?.get_signal_stop_object(),
                has_lead: model.leads.iter().any(|lead| lead.status),
            };
            if camera {
                // Restart hand-over (stock_adopt): send nothing, so panda
                // keeps forwarding the camera's cruising SCC until we engage.
                if !state.stock_adopt.pending {
                    let (frame, value) =
                        canfd_acc::camera_acc(&mut c.writer, &scc, c.stopping.as_mut(), &i)?;
                    c.value_last = value;
                    if let Some(frame) = frame {
                        result.push(frame);
                    }
                }
                result.extend(canfd_maintenance::tcs(
                    &mut c.writer,
                    config.bus,
                    captures.get("tcs"),
                )?);
            } else {
                let (frame, value) = canfd_acc::acc(&mut c.writer, &scc, c.stopping.as_mut(), &i)?;
                c.value_last = value;
                result.push(frame);
                c.accel_last = command.accel;
            }
        }
    } else if c.settings.camera == 3 {
        let button = c.buttons.spam(input, &c.settings, c.frame)?;
        result.extend(canfd_buttons::forward(
            &mut c.writer,
            ForwardInput {
                bus: config.bus,
                frame: c.frame,
                message: config.button_message(),
                button: f64::from(button),
                main_trigger: c.buttons.main_trigger,
                lfa_trigger: c.buttons.lfa_trigger,
            },
            captures.get("cruise_buttons_msg"),
        )?);
    } else {
        result.extend(c.buttons.messages(
            &mut c.writer,
            input,
            (&c.settings, c.frame, captures),
        )?);
    }
    Ok(result)
}
