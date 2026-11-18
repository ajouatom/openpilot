use super::{
    lead::{self, Lead},
    wire::{get, set, Values},
    Error,
};
use openpilot_control_policy::math::clip;

pub fn lane_desire(data: &mut Values, desire: u16) {
    match desire {
        1 | 2 => set(
            data,
            &[
                ("LANE_CHANGING", f64::from(desire)),
                ("LANELINE_CURVATURE", 15.),
                ("LANELINE_CURVATURE_DIRECTION", f64::from(desire - 1)),
            ],
        ),
        3 | 4 => {
            data.insert("LANE_CHANGING".into(), f64::from(desire));
        }
        _ => {}
    }
}

pub fn lane_lines(data: &mut Values, input: (f64, bool, u16)) {
    let (angle, active, desire) = input;
    let curvature = (angle / 3.).round_ties_even();
    set(
        data,
        &[
            (
                "LANELINE_CURVATURE",
                if active {
                    curvature.abs().min(15.) + if curvature < 0. { -1. } else { 0. }
                } else {
                    0.
                },
            ),
            (
                "LANELINE_CURVATURE_DIRECTION",
                if active && curvature < 0. { 1. } else { 0. },
            ),
        ],
    );
    if desire != 0 {
        lane_desire(data, desire);
    }
}

pub fn normalize_corner(data: &mut Values, ccnc: bool) -> Result<(), Error> {
    for side in ["LF", "RF", "LR", "RR"] {
        let distance = format!("{side}_DETECT_DISTANCE");
        let detect = format!("{side}_DETECT");
        if get(data, &distance)? != 0. && (ccnc || get(data, &detect)? >= 4.) {
            data.insert(
                std::sync::Arc::<str>::from(detect),
                if ccnc { 3. } else { 1. },
            );
        } else if ccnc && matches!(get(data, &detect)?, 1. | 2.) {
            // carrot PR580: stock flicks a box with distance 0 between car
            // frames; keep the grey car.
            data.insert(std::sync::Arc::<str>::from(detect), 3.);
        }
    }
    Ok(())
}

pub type LeadDisplayInput<'a> = (&'a [Lead], (&'a [f64], &'a [f64]), bool, Option<f64>);

pub fn front_lead(data: &mut Values, input: LeadDisplayInput<'_>) -> Result<(), Error> {
    let (leads, path, enabled, hud) = input;
    let Some(lead) = lead::nearest(leads) else {
        set(
            data,
            &[
                ("FF_DETECT", 0.),
                ("FF_DISTANCE", 204.6),
                ("FF_LATERAL", 0.),
            ],
        );
        return Ok(());
    };
    let stock = get(data, "FF_LATERAL")?;
    let stock = if stock >= 6.4 { stock - 12.8 } else { stock };
    let lateral = lead::lateral(lead, path)?;
    let same = get(data, "FF_DETECT")? != 0.
        && (get(data, "FF_DISTANCE")? - lead.d_rel).abs() <= 3.
        && (stock - lateral).abs() <= 1.;
    if !same {
        data.insert("FF_DETECT".into(), if enabled { 4. } else { 3. });
    }
    set(
        data,
        &[
            ("FF_DISTANCE", clip(lead.d_rel, 0.1, 204.5)),
            ("FF_LATERAL", clip(hud.unwrap_or(lateral), -6.4, 6.3)),
        ],
    );
    Ok(())
}

pub fn hide_service_warning(data: &mut Values) {
    let mut hidden = false;
    for name in ["FAULT_LCA", "FAULT_HDA"] {
        if data.get(name) == Some(&1.) {
            data.insert(name.into(), 0.);
            hidden = true;
        }
    }
    if hidden && data.get("FAULT_DAS") == Some(&1.) {
        data.insert("FAULT_DAS".into(), 0.);
    }
}

pub fn hide_alerts(
    data: &mut Values,
    extended: bool,
    soft_hold: u8,
    trailer: bool,
) -> Result<(), Error> {
    if [1., 2., 5., 6., 10., 21., 22.].contains(&get(data, "ALERTS_2")?) {
        set(data, &[("ALERTS_2", 0.), ("DAW_ICON", 0.)]);
    }
    if get(data, "ALERTS_1")? == 0. {
        set(
            data,
            &[("SOUNDS_1", 0.), ("SOUNDS_2", 0.), ("SOUNDS_4", 0.)],
        );
    }
    let alert = get(data, "ALERTS_3")?;
    if [3., 4., 11., 12., 13., 14., 17., 19., 26., 7., 8., 9., 10.].contains(&alert)
        || (extended && [20., 27., 28.].contains(&alert))
    {
        set(data, &[("ALERTS_3", 0.), ("SOUNDS_3", 0.)]);
    }
    let alert = get(data, "ALERTS_5")?;
    if [1., 2., 3., 4., 5.].contains(&alert)
        || (extended && ((alert == 11. && soft_hold == 0) || (trailer && alert == 6.)))
    {
        data.insert("ALERTS_5".into(), 0.);
    }
    Ok(())
}

/// Preserve OEM popup/sound precedence before adding the source DM hands-on warning.
pub fn driver_monitoring_alert(data: &mut Values, level: u8, stock: &Values) -> Result<(), Error> {
    if !(1..=3).contains(&level) {
        return Ok(());
    }
    let stock_alert = get(stock, "ALERTS_2")?;
    if [7., 8., 9., 10., 14., 21.].contains(&stock_alert) {
        data.insert("ALERTS_2".into(), stock_alert);
        for index in 1..=4 {
            let name = format!("SOUNDS_{index}");
            data.insert(
                std::sync::Arc::<str>::from(name.clone()),
                get(stock, &name)?,
            );
        }
    }
    if get(data, "ALERTS_2")? == 0. {
        data.insert("ALERTS_2".into(), if level == 3 { 2. } else { 1. });
        if level == 3 {
            let silent = (1..=4)
                .map(|index| get(data, &format!("SOUNDS_{index}")))
                .collect::<Result<Vec<_>, _>>()?
                .into_iter()
                .all(|sound| sound == 0.);
            if silent {
                data.insert("SOUNDS_2".into(), 3.);
            }
        }
    }
    Ok(())
}
