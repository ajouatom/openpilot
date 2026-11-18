use super::{
    canfd_cluster::{keep_stock_scc_main, ClusterInput, ClusterMessages},
    wire::{get, values, CanWriter},
    Error,
};
use openpilot_can::Frame;

pub(super) fn checksum(address: u32, data: &[u8]) -> u16 {
    let mut crc = 0u16;
    for byte in data
        .iter()
        .skip(2)
        .chain(address.to_le_bytes().iter().take(2))
    {
        crc ^= u16::from(*byte) << 8;
        for _ in 0..8 {
            crc = if crc & 0x8000 != 0 {
                (crc << 1) ^ 0x1021
            } else {
                crc << 1
            };
        }
    }
    crc ^ match data.len() {
        8 => 0x5f29,
        16 => 0x041d,
        24 => 0x819d,
        32 => 0x9f5b,
        _ => 0,
    }
}

pub fn alt2_request(
    writer: &mut CanWriter,
    i: &ClusterInput<'_>,
    source: &ClusterMessages<'_>,
) -> Result<Frame, Error> {
    let original = source
        .buttons_alt2
        .ok_or_else(|| Error::Signal("CRUISE_BUTTONS_ALT2".into()))?;
    let mut lfa = 0.;
    let mut button = 0.;
    if get(original, "LFA_BTN")? == 0. && get(original, "CRUISE_BUTTONS")? == 0. {
        let lfa_off = match source.lfahda {
            Some(data) => get(data, "HDA_LFA_SymSta")? == 0.,
            None => false,
        };
        if lfa_off && !i.hud.frame.is_multiple_of(200) && i.hud.frame % 200 < 12 {
            lfa = 1.;
        }
        if (i.hud.enabled || keep_stock_scc_main(i, source)) && !i.interlock && lfa == 0. {
            let pulse = i.hud.frame % 200 > 20 && i.hud.frame % 200 <= 26 && i.speed > 3.;
            if !i.main_mode {
                button = if pulse { 8. } else { 0. };
            } else if i.hud.enabled && [0., 4.].contains(&i.acc_mode) {
                button = if pulse { 2. } else { 0. };
            } else if i.hud.enabled
                && source
                    .scc
                    .is_some_and(|data| data.get("InfoDisplay") == Some(&4.))
                && !i.stopping
            {
                button = if i.hud.frame % 30 > 10 && i.hud.frame % 30 <= 16 {
                    2.
                } else {
                    0.
                };
            }
        }
    }
    let mut frame = writer.frame(
        "CRUISE_BUTTONS_ALT2",
        i.bus.cam,
        &values(&[("LFA_BTN", lfa), ("CRUISE_BUTTONS", button)]),
        None,
    )?;
    let checked = checksum(frame.address, &frame.data).to_le_bytes();
    frame
        .data
        .get_mut(..2)
        .ok_or(Error::Numeric)?
        .copy_from_slice(&checked);
    Ok(frame)
}
