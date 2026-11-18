use super::{
    flags as f,
    parser_inputs::Channel,
    state::State,
    state_fields::{buttons, byte, extend, CRUISE_BUTTONS},
    wire::get,
    Error,
};
use openpilot_cereal::car_capnp::car_state::button_event::Type;

pub fn main(state: &mut State, now: u64) -> Result<u8, Error> {
    let previous = state.main_buttons.back().copied().ok_or(Error::Numeric)?;
    let alt = state.inputs.captured("cruise_buttons_alt2")?;
    if let Some(data) = &alt {
        extend(
            &mut state.main_buttons,
            [u8::from(get(data, "CRUISE_BUTTONS")?.trunc() == 8.)],
        );
    } else {
        let name = state.config.button_message();
        let adaptive = state
            .inputs
            .all(Channel::Pt, name, "ADAPTIVE_CRUISE_MAIN_BTN", now)?;
        let normal = state
            .inputs
            .all(Channel::Pt, name, "NORMAL_CRUISE_MAIN_BTN", now)?;
        if adaptive.len() != normal.len() {
            return Err(Error::Numeric);
        }
        extend(
            &mut state.main_buttons,
            adaptive
                .into_iter()
                .zip(normal)
                .map(|(a, n)| u8::from(a != 0. || n != 0.)),
        );
    }
    let current = state.main_buttons.back().copied().ok_or(Error::Numeric)?;
    let released = current != previous && current == 0;
    let cruise = match &alt {
        Some(data) => get(data, "CRUISE_BUTTONS")?,
        None => state.inputs.signal(
            Channel::Pt,
            state.config.button_message(),
            "CRUISE_BUTTONS",
            now,
        )?,
    };
    if state.config.flags & f::CAMERA_SCC != 0 {
        state.main_mode = state
            .inputs
            .signal(Channel::Cam, "SCC_CONTROL", "MainMode_ACC", now)?
            == 1.;
        state.acc_mode = state
            .inputs
            .signal(Channel::Cam, "SCC_CONTROL", "ACCMode", now)?;
        state.lfa_icon =
            state
                .inputs
                .signal(Channel::Cam, "LFAHDA_CLUSTER", "HDA_LFA_SymSta", now)?;
        let seen = state.inputs.cam.seen_addresses.contains(&0x1a0);
        let set_speed = state
            .inputs
            .signal(Channel::Cam, "SCC_CONTROL", "VSetDis", now)?
            * if state.metric { 1. / 3.6 } else { 0.44704 };
        // Any cancel or MAIN activity, or a cancel/off request inside
        // openpilot from the previous frame, ends the wait.
        let off_requested = {
            let out = state
                .out
                .get_root_as_reader::<openpilot_cereal::car_capnp::car_state::Reader<'_>>()?;
            cruise == 4.
                || current != 0
                || released
                || out.get_cruise_cancel_state()
                || out.get_activate_cruise() < 0
        };
        if state.stock_adopt.update(
            seen,
            set_speed,
            off_requested,
            state.main_mode,
            state.acc_mode,
        ) {
            state.main_enabled = true;
            state.manual_main_off = false;
        }
    }
    if [1., 2.].contains(&cruise) && state.config.longitudinal {
        state.main_enabled = true;
        state.manual_main_off = false;
    }
    if released {
        state.main_enabled = !state.main_enabled;
        state.manual_main_off = !state.main_enabled;
        state.inputs.diagnostics.prints.push(format!(
            "main_enabled = {}",
            if state.main_enabled { "True" } else { "False" }
        ));
    }
    // CameraSCC MAIN can be requested automatically for cluster objects. Driver
    // intent comes from AutoEngage initialization and the physical buttons above.
    if state.config.longitudinal
        && state.config.flags & f::CAMERA_SCC == 0
        && state.main_mode
        && !state.manual_main_off
    {
        state.main_enabled = true;
    }
    Ok(previous)
}

pub fn events(state: &mut State, previous_main: u8, now: u64) -> Result<Vec<(bool, Type)>, Error> {
    let previous = state.cruise_buttons.back().copied().ok_or(Error::Numeric)?;
    let alt = state.inputs.captured("cruise_buttons_alt2")?;
    let cruise = if let Some(data) = alt {
        if data.get("LFA_BTN").copied().unwrap_or(0.).trunc() == 1. {
            vec![5]
        } else {
            let value = byte(data.get("CRUISE_BUTTONS").copied().unwrap_or(0.).trunc())?;
            vec![if value < 5 { value } else { 0 }]
        }
    } else if state
        .inputs
        .signal(Channel::Pt, state.config.button_message(), "LFA_BTN", now)?
        != 0.
    {
        vec![5]
    } else {
        state
            .inputs
            .all(
                Channel::Pt,
                state.config.button_message(),
                "CRUISE_BUTTONS",
                now,
            )?
            .into_iter()
            .map(byte)
            .collect::<Result<Vec<_>, _>>()?
    };
    extend(&mut state.cruise_buttons, cruise);
    state.buttons_counter =
        state
            .inputs
            .signal(Channel::Pt, state.config.button_message(), "COUNTER", now)?;
    let mut paddle = state.paddle;
    let paddle_name = if state.config.button_message() == "CRUISE_BUTTONS" {
        Some("CRUISE_BUTTONS")
    } else if state.config.gear_message() == "GEAR" {
        Some("GEAR")
    } else {
        None
    };
    if let Some(name) = paddle_name {
        paddle = if state.inputs.signal(Channel::Pt, name, "LEFT_PADDLE", now)? == 1. {
            1
        } else if state
            .inputs
            .signal(Channel::Pt, name, "RIGHT_PADDLE", now)?
            == 1.
        {
            2
        } else {
            0
        };
    }
    let mut events = buttons(
        state.cruise_buttons.back().copied().ok_or(Error::Numeric)?,
        previous,
        CRUISE_BUTTONS,
    );
    events.extend(buttons(
        paddle,
        state.paddle,
        &[(1, Type::PaddleLeft), (2, Type::PaddleRight)],
    ));
    events.extend(buttons(
        state.main_buttons.back().copied().ok_or(Error::Numeric)?,
        previous_main,
        &[(1, Type::MainCruise)],
    ));
    state.paddle = paddle;
    // carrot PR580: a held paddle reads 0 for 0.1-0.2 s every few frames;
    // bridge 0.3 s so SCC does not re-engage under the still-held paddle.
    state.paddle_hold = if paddle > 0 {
        30
    } else {
        state.paddle_hold.saturating_sub(1)
    };
    Ok(events)
}
