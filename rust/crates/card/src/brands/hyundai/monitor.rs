use super::{
    config::CarConfig,
    flags as f,
    parameters::setting_int,
    parser_inputs::{Channel, Inputs},
    Error,
};
use openpilot_params::Params;

pub struct Monitor {
    pub count: u32,
    hint_enabled: bool,
    hint: bool,
}

impl Monitor {
    pub fn new(config: &CarConfig, settings: &Params) -> Result<Self, Error> {
        let enabled =
            setting_int(settings, "HyundaiCameraSCC")? == 0 && config.flags & f::CAMERA_SCC == 0;
        settings.put_bool("HyundaiCameraSccHint", false)?;
        Ok(Self {
            count: 0,
            hint_enabled: enabled,
            hint: false,
        })
    }

    pub fn update(
        &mut self,
        inputs: &mut Inputs,
        config: &CarConfig,
        settings: &Params,
        now: u64,
    ) -> Result<(), Error> {
        let fd = config.flags & f::CANFD != 0;
        let hint_name = if fd { "SCC_CONTROL" } else { "SCC12" };
        if self.hint_enabled
            && !self.hint
            && inputs
                .cam
                .dbc
                .names
                .get(hint_name)
                .is_some_and(|address| inputs.cam.seen_addresses.contains(address))
        {
            self.hint = true;
            settings.put_bool("HyundaiCameraSccHint", true)?;
        }
        if self.count > 200 {
            if fd {
                self.discover_camera_templates(inputs, now)?;
            }
            return Ok(());
        }
        if settings.get_bool("ControlsReady")? {
            self.count += 1;
        }
        inputs.diagnostics.monitor(self.count, inputs.alt.is_some());
        if self.count == 50 {
            inputs.pt.controls_ready = true;
            inputs.cam.controls_ready = true;
            if let Some(alt) = &mut inputs.alt {
                alt.controls_ready = true;
            }
        }
        if !fd {
            match self.count {
                104 => {
                    if !inputs.capture((Channel::Cam, "FCA11", "fca11"), false, now)? {
                        inputs.capture((Channel::Pt, "FCA11", "fca11"), false, now)?;
                    }
                    inputs.capture((Channel::Cam, "LKAS11", "lkas11"), false, now)?;
                    inputs.capture((Channel::Pt, "CLU11", "clu11"), false, now)?;
                }
                105 => {
                    if !config.longitudinal || config.flags & f::CAMERA_SCC != 0 {
                        let channel = if config.flags & f::CAMERA_SCC != 0 {
                            Channel::Cam
                        } else {
                            Channel::Pt
                        };
                        for (name, key) in [
                            ("SCC11", "scc11"),
                            ("SCC12", "scc12"),
                            ("SCC13", "scc13"),
                            ("SCC14", "scc14"),
                        ] {
                            inputs.capture((channel, name, key), false, now)?;
                        }
                    }
                }
                _ => {}
            }
            return Ok(());
        }
        match self.count {
            120 => {
                let channel = if config.flags & f::CAMERA_SCC != 0 {
                    Channel::Cam
                } else {
                    Channel::Pt
                };
                inputs.capture((channel, "SCC_CONTROL", "scc_control"), false, now)?;
            }
            121 => {
                for (channel, name, key) in
                    [(Channel::Pt, "TCS", "tcs"), (Channel::Pt, "MDPS", "mdps")]
                {
                    inputs.capture((channel, name, key), false, now)?;
                }
            }
            123 => {
                if config.candidate == "KIA_PV5" {
                    inputs.capture(
                        (Channel::Pt, "CANFD_HDA_INFO_364", "hda_info_4a3"),
                        false,
                        now,
                    )?;
                    inputs.capture(
                        (Channel::Pt, "CANFD_NAVI_PROFILE_093", "navi_profile_4be"),
                        false,
                        now,
                    )?;
                    inputs.capture(
                        (Channel::Alt, "CANFD_NAVI_STATUS_380", "navi_status_380"),
                        false,
                        now,
                    )?;
                } else {
                    let ev4 = config.candidate == "KIA_EV4";
                    for (name, key) in [
                        (
                            if ev4 {
                                "CANFD_HDA_INFO_364"
                            } else {
                                "HDA_INFO_4A3"
                            },
                            "hda_info_4a3",
                        ),
                        ("NEW_MSG_4B4", "navi_position_4b4"),
                        (
                            if ev4 {
                                "CANFD_NAVI_SEGMENT_099"
                            } else {
                                "NEW_MSG_4B9"
                            },
                            "navi_segment_4b9",
                        ),
                        (
                            if ev4 {
                                "CANFD_NAVI_PROFILE_093"
                            } else {
                                "NEW_MSG_4BE"
                            },
                            "navi_profile_4be",
                        ),
                    ] {
                        inputs.capture((Channel::Pt, name, key), false, now)?;
                    }
                }
                inputs.capture(
                    (Channel::Pt, "STEER_TOUCH_2AF", "steer_touch_2af"),
                    false,
                    now,
                )?;
            }
            124 => {
                inputs.capture(
                    (Channel::Pt, config.button_message(), "cruise_buttons_msg"),
                    false,
                    now,
                )?;
                if !inputs.capture((Channel::Cam, "CAM_0x362", "cam_0x362"), false, now)? {
                    inputs.capture((Channel::Alt, "CAM_0x362", "cam_0x362"), false, now)?;
                }
                if !inputs.capture((Channel::Alt, "CAM_0x2a4", "cam_0x2a4"), false, now)? {
                    inputs.capture((Channel::Cam, "CAM_0x2a4", "cam_0x2a4"), false, now)?;
                }
            }
            125 => {
                inputs.capture(
                    (
                        Channel::Pt,
                        "MANUAL_SPEED_LIMIT_ASSIST",
                        "manual_speed_limit_assist",
                    ),
                    true,
                    now,
                )?;
                if config.gear_message() == "ACCELERATOR" {
                    inputs.capture((Channel::Pt, "ACCELERATOR", "accelerator"), true, now)?;
                }
                for (name, key) in [("BLINKERS", "blinkers"), ("BLINKERS_ALT", "blinkers_alt")] {
                    inputs.capture((Channel::Pt, name, key), false, now)?;
                }
                inputs.capture((Channel::Pt, "BLINKER_STALKS", "blinker_stalks"), true, now)?;
                inputs.capture(
                    (Channel::Pt, "DOORS_SEATBELTS", "doors_seatbelts"),
                    false,
                    now,
                )?;
            }
            126 => {
                inputs.capture(
                    (Channel::Pt, "CRUISE_BUTTONS_ALT2", "cruise_buttons_alt2"),
                    true,
                    now,
                )?;
                inputs.capture((Channel::Pt, "TRAILER_STATUS", "trailer_status"), true, now)?;
            }
            _ => {}
        }
        self.discover_camera_templates(inputs, now)?;
        Ok(())
    }
    fn discover_camera_templates(&self, inputs: &mut Inputs, now: u64) -> Result<(), Error> {
        for (name, key, ready) in [
            ("LFA", "lfa", 121),
            ("LFA_ALT", "lfa_alt", 121),
            ("LFAHDA_CLUSTER", "lfahda_cluster", 121),
            ("ADRV_0x161", "adrv_0x161", 122),
            ("ADRV_0x200", "adrv_0x200", 122),
            ("ADRV_0x1ea", "adrv_0x1ea", 122),
            ("ADRV_0x160", "adrv_0x160", 122),
            ("CCNC_0x162", "ccnc_0x162", 122),
        ] {
            if self.count < ready || inputs.captures.contains_key(key) {
                continue;
            }
            let parser = &mut inputs.cam;
            let Some(address) = parser.dbc.names.get(name).copied() else {
                continue;
            };
            if !parser.seen_addresses.contains(&address) {
                continue;
            }
            if !parser.states.contains_key(&address) {
                parser.add(name, None, false, now)?;
            }
            let timestamp = parser
                .states
                .get(&address)
                .and_then(|state| state.timestamps.back())
                .copied()
                .unwrap_or(0);
            let age = i128::from(parser.last_update) - i128::from(timestamp);
            let size = parser.dbc.messages[&address].size;
            if timestamp > 0
                && (0..=150_000_000).contains(&age)
                && parser
                    .raw
                    .get(&address)
                    .is_some_and(|data| data.len() == size)
            {
                inputs.captures.insert(key, (Channel::Cam, name));
                inputs.diagnostics.logs.push(crate::core::VehicleLog {
                    level: crate::query::DiagnosticLevel::Info,
                    message: format!(
                        "CAN-FD camera TX template ready: {name} bus={} ready_count={}",
                        parser.bus, self.count
                    ),
                });
            }
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use openpilot_can::{dbc::Dbc, packer::Packer, parser::Parser, Frame, Packet};
    use std::{collections::BTreeMap, sync::Arc};

    fn inputs() -> Inputs {
        let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../../opendbc_repo/opendbc/dbc/hyundai_canfd_generated.dbc");
        let dbc = Arc::new(Dbc::load(&path).unwrap());
        let mut cam = Parser::new(Arc::clone(&dbc), 2, 1_000_000_000);
        cam.controls_ready = true;
        Inputs {
            diagnostics: super::super::diagnostics::Diagnostics::default(),
            pt: Parser::new(dbc, 0, 1_000_000_000),
            cam,
            alt: None,
            pedal: None,
            captures: BTreeMap::new(),
            values_cache: Default::default(),
        }
    }
    fn receive(inputs: &mut Inputs, now: u64, frame: Option<Frame>) {
        inputs
            .cam
            .update(&[Packet {
                mono_time: now,
                frames: frame.into_iter().collect(),
            }])
            .unwrap();
    }
    // Port of the pinned test_canfd_camera_discovery.py startup/late/live cases.
    #[test]
    fn camera_templates_require_real_fresh_frames_and_keep_discovering_after_startup() {
        for (name, key, ready) in [
            ("LFA", "lfa", 121),
            ("LFA_ALT", "lfa_alt", 121),
            ("LFAHDA_CLUSTER", "lfahda_cluster", 121),
            ("ADRV_0x161", "adrv_0x161", 122),
            ("ADRV_0x200", "adrv_0x200", 122),
            ("ADRV_0x1ea", "adrv_0x1ea", 122),
            ("ADRV_0x160", "adrv_0x160", 122),
            ("CCNC_0x162", "ccnc_0x162", 122),
        ] {
            for count in [ready, 201] {
                let mut inputs = inputs();
                let monitor = Monitor {
                    count,
                    hint_enabled: false,
                    hint: false,
                };
                let mut packer = Packer::new(Arc::clone(&inputs.cam.dbc));
                let address = inputs.cam.dbc.names[name];
                for (counter, now) in [(1, 3_000_000_000), (2, 3_010_000_000), (3, 3_020_000_000)] {
                    let data = packer
                        .pack(name, &[("COUNTER", f64::from(counter))], None)
                        .unwrap();
                    receive(
                        &mut inputs,
                        now,
                        Some(Frame {
                            address,
                            bus: 2,
                            data,
                        }),
                    );
                    monitor.discover_camera_templates(&mut inputs, now).unwrap();
                    assert_eq!(
                        inputs.captures.contains_key(key),
                        counter >= 2,
                        "{name} count={count} frame={counter}"
                    );
                    if counter >= 2 {
                        assert_eq!(
                            inputs.captured(key).unwrap().unwrap()["COUNTER"],
                            f64::from(counter)
                        );
                    }
                }
                assert_eq!(inputs.diagnostics.logs.len(), 1);
                assert!(!inputs.cam.states[&address].ignore_counter);
                assert!(!inputs.cam.states[&address].ignore_checksum);
            }
        }
    }
    #[test]
    fn absent_echo_invalid_and_stale_camera_frames_cannot_supply_templates() {
        let mut inputs = inputs();
        let monitor = Monitor {
            count: 201,
            hint_enabled: false,
            hint: false,
        };
        let mut packer = Packer::new(Arc::clone(&inputs.cam.dbc));
        let address = inputs.cam.dbc.names["LFA"];
        for bus in [0, 1, 128, 130] {
            let data = packer.pack("LFA", &[], None).unwrap();
            receive(
                &mut inputs,
                2_000_000_000,
                Some(Frame { address, bus, data }),
            );
            monitor
                .discover_camera_templates(&mut inputs, 2_000_000_000)
                .unwrap();
        }
        assert!(inputs.cam.states.is_empty());
        assert!(inputs.cam.can_valid());
        let data = packer.pack("LFA", &[("COUNTER", 1.)], None).unwrap();
        receive(
            &mut inputs,
            3_000_000_000,
            Some(Frame {
                address,
                bus: 2,
                data,
            }),
        );
        monitor
            .discover_camera_templates(&mut inputs, 3_000_000_000)
            .unwrap();
        let data = packer.pack("LFA", &[("COUNTER", 2.)], None).unwrap();
        let mut corrupt = data.clone();
        corrupt[0] ^= 1;
        receive(
            &mut inputs,
            3_010_000_000,
            Some(Frame {
                address,
                bus: 2,
                data: corrupt,
            }),
        );
        monitor
            .discover_camera_templates(&mut inputs, 3_010_000_000)
            .unwrap();
        assert!(!inputs.captures.contains_key("lfa"));
        receive(
            &mut inputs,
            3_020_000_000,
            Some(Frame {
                address,
                bus: 2,
                data,
            }),
        );
        receive(&mut inputs, 4_000_000_000, None);
        monitor
            .discover_camera_templates(&mut inputs, 4_000_000_000)
            .unwrap();
        assert!(!inputs.captures.contains_key("lfa"));
        let data = packer.pack("LFA", &[("COUNTER", 3.)], None).unwrap();
        receive(
            &mut inputs,
            4_010_000_000,
            Some(Frame {
                address,
                bus: 2,
                data,
            }),
        );
        monitor
            .discover_camera_templates(&mut inputs, 4_010_000_000)
            .unwrap();
        assert!(inputs.captures.contains_key("lfa"));
    }
}
