use super::style::{self, Anchor, Text};
use crate::{context::Context, onroad::model_renderer::math, paint::color, state::messages};
use openpilot_ui_framework::{draw::Draw, geometry::Rect, text::Font, text_layout, Error};

pub const WHITE: u32 = color(255, 255, 255, 255);
pub const GREEN: u32 = color(0, 255, 0, 230);

pub fn torque(context: &Context) -> Result<f64, crate::Error> {
    let messages = context.messages.borrow();
    let controls = messages::controls_state(&messages.state)?;
    if matches!(
        controls.get_lateral_control_state().which()?,
        openpilot_cereal::log_capnp::controls_state::lateral_control_state::Which::AngleState(_)
    ) {
        let speed = f64::from(messages::car_state(&messages.state)?.get_v_ego());
        let actual = f64::from(controls.get_curvature()) * speed.powi(2);
        let desired = f64::from(controls.get_desired_curvature()) * speed.powi(2);
        let difference = desired - actual;
        let compensation = f64::from(messages::live_parameters(&messages.state)?.get_roll())
            * 9.81
            * math::interp(speed, &[5.0, 15.0], &[0.0, 1.0])?;
        let maximum = context
            .ui
            .borrow()
            .slow
            .car
            .map_or(3.0, |car| car.max_lateral_accel);
        Ok(
            if messages::car_control(&messages.state)?.get_lat_active() {
                math::clip((actual - compensation + difference) / maximum, -1.0, 1.0)
            } else {
                0.0
            },
        )
    } else {
        Ok(-f64::from(
            messages::car_output(&messages.state)?
                .get_actuators_output()?
                .get_torque(),
        ))
    }
}

pub fn gear(context: &Context) -> Result<String, crate::Error> {
    use openpilot_cereal::car_capnp::car_state::GearShifter;
    let messages = context.messages.borrow();
    let car = messages::car_state(&messages.state)?;
    Ok(match car.get_gear_shifter() {
        Ok(GearShifter::Drive) => {
            if car.get_gear_step() > 0 {
                car.get_gear_step().to_string()
            } else {
                "D".into()
            }
        }
        Ok(GearShifter::Park) => "P".into(),
        Ok(GearShifter::Reverse) => "R".into(),
        Ok(GearShifter::Neutral) => "N".into(),
        Ok(GearShifter::Sport) => "S".into(),
        Ok(GearShifter::Low) => "L".into(),
        Ok(GearShifter::Brake) => "B".into(),
        Ok(GearShifter::Eco) => "E".into(),
        Ok(GearShifter::Unknown) => "U".into(),
        Ok(GearShifter::Manumatic) | Err(_) => "M".into(),
    })
}

pub fn driving_mode(context: &Context) -> Result<(String, u32), crate::Error> {
    let messages = context.messages.borrow();
    let car = messages::car_state(&messages.state)?;
    let (text, tint) = if car.get_brake_hold_active() {
        ("brake hold", color(255, 0, 0, 230))
    } else if car.get_soft_hold_active() != 0 {
        ("soft hold", color(255, 165, 0, 230))
    } else if car.get_carrot_cruise() != 0 {
        ("carrot", GREEN)
    } else {
        match messages::longitudinal_plan(&messages.state)?.get_my_driving_mode() {
            1 => ("eco", color(0, 255, 0, 200)),
            2 => ("safe", color(255, 165, 0, 200)),
            3 => ("norm", color(255, 255, 255, 200)),
            4 => ("high", color(255, 0, 0, 200)),
            _ => ("", color(255, 255, 255, 200)),
        }
    };
    Ok((context.tr(text), tint))
}

pub fn styled(
    draw: &mut dyn Draw,
    text: &str,
    position: [f64; 2],
    size: f64,
    tint: u32,
    anchor: Anchor,
) -> Result<(), Error> {
    style::text(
        draw,
        Text {
            value: text,
            position,
            size,
            font: Font::Display,
            color: tint,
            anchor,
            border: 1.0,
            shadow: 3.0,
            y_offset: 0.0,
        },
    )
}

pub fn badge(context: &Context, draw: &mut dyn Draw, rect: Rect) -> Result<(), Error> {
    let ui = context.ui.borrow();
    let status = &ui.slow;
    if !(status.jetlink_badge.is_some()
        || status.delivery_badge.is_some()
        || status.usbgpu_present
        || status.usbgpu_active
        || status.usbgpu_loading
        || status.usbgpu_startup_failed
        || status.internal_model_badge.is_some())
    {
        return Ok(());
    }
    let mut style = if status.usbgpu_startup_failed {
        "error"
    } else if status.usbgpu_loading {
        "loading"
    } else if status.usbgpu_compile_pending {
        "compile_pending"
    } else if status.usbgpu_active {
        "active"
    } else if !status.usbgpu_compiled {
        "not_compiled"
    } else {
        "ready"
    };
    let mut text = if style == "compile_pending" {
        "eGPU REBOOT"
    } else if style == "error" {
        "eGPU ERROR"
    } else {
        "eGPU"
    };
    // The internal model's badge has the lowest priority: any eGPU state,
    // delivery or Jetlink badge replaces it.
    if !(status.usbgpu_present
        || status.usbgpu_active
        || status.usbgpu_loading
        || status.usbgpu_startup_failed)
    {
        if let Some(badge) = &status.internal_model_badge {
            text = &badge.text;
            style = &badge.style;
        }
    }
    if !(status.usbgpu_active || status.usbgpu_loading || status.usbgpu_startup_failed) {
        if let Some(badge) = &status.delivery_badge {
            text = &badge.text;
            style = &badge.style;
        }
    }
    if !status.usbgpu_active {
        for badge in [&status.jetlink_badge, &status.recovery_badge]
            .into_iter()
            .flatten()
        {
            text = &badge.text;
            style = &badge.style;
        }
    }
    let big = context.big;
    let tint = match style {
        "active" => color(0, 255, 0, if big { 210 } else { 230 }),
        "loading" => color(255, 255, 0, if big { 210 } else { 230 }),
        "error" => color(255, 0, 0, if big { 210 } else { 230 }),
        "compile_pending" | "not_compiled" => color(255, 165, 0, 230),
        _ => color(255, 255, 255, 210),
    };
    let font = if text.is_ascii() {
        Font::SemiBold
    } else {
        Font::Display
    };
    let font_size = if big {
        if text.is_ascii() {
            38.
        } else {
            30.
        }
    } else {
        22.
    };
    let (pad_x, pad_y) = if big { (18.0, 8.0) } else { (10.0, 5.0) };
    let size = text_layout::measure(draw, font, text, font_size, 0.0);
    let width = f64::from(size.x) + pad_x * 2.0;
    let badge = Rect {
        x: math::float(
            f64::from(rect.x) + f64::from(rect.width)
                - if big { 30.0 + 192.0 } else { 0.0 }
                - width
                - 24.0,
        ),
        y: math::float(f64::from(rect.y) + if big { 24.0 } else { 12.0 }),
        width: math::float(width),
        height: math::float(f64::from(size.y) + pad_y * 2.0),
    };
    draw.rounded_segments(badge, 0.35, 8, color(0, 0, 0, 150), false)?;
    draw.rounded_outline(
        badge,
        openpilot_ui_framework::draw::RoundedOutline {
            roundness: 0.35,
            segments: 8,
            thickness: if big { 3.0 } else { 2.0 },
            color: tint,
        },
    )?;
    crate::paint::text(
        draw,
        openpilot_ui_framework::geometry::Point {
            x: math::float(f64::from(badge.x) + pad_x),
            y: math::float(f64::from(badge.y) + pad_y),
        },
        crate::paint::Text {
            value: text,
            font,
            size: font_size,
            spacing: 0.0,
            color: tint,
        },
    )
}
