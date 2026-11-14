use crate::Predicate;

#[derive(Debug, Clone, Copy)]
pub struct ImportConfig {
    pub pc: bool,
    pub tici: bool,
    pub webcam: bool,
    pub carrot_web_external: bool,
    pub darwin: bool,
    pub bodyteleop_available: bool,
}
impl ImportConfig {
    /// Capture once, as the original module does at import. Bodyteleop module
    /// availability is supplied by the embedding installation, without Python.
    pub fn from_environment(bodyteleop_available: bool) -> Self {
        let tici = std::path::Path::new("/TICI").is_file();
        Self {
            pc: !tici,
            tici,
            webcam: std::env::var_os("USE_WEBCAM").is_some(),
            carrot_web_external: std::env::var_os("CARROT_WEB_EXTERNAL")
                .is_some_and(|value| value == "1"),
            darwin: cfg!(target_os = "macos"),
            bodyteleop_available,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SourceProcess {
    Python {
        module: &'static str,
    },
    Native {
        cwd: &'static str,
        argv: &'static [&'static str],
    },
    Persistent {
        module: &'static str,
        pid_key: &'static str,
    },
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RustAvailability {
    NotPorted,
    /// An isolated candidate exists; this does not authorize manager selection.
    Candidate {
        package: &'static str,
        binary: &'static str,
        limitation: &'static str,
    },
}

#[derive(Debug, Clone, Copy)]
pub struct Descriptor {
    pub name: &'static str,
    pub source: SourceProcess,
    pub enabled: bool,
    pub sigkill: bool,
    pub restart_if_crash: bool,
    pub daemon: bool,
    pub predicate: Predicate,
    pub rust: RustAvailability,
}
impl Descriptor {
    fn new(
        name: &'static str,
        source: SourceProcess,
        predicate: Predicate,
        enabled: bool,
        restart_if_crash: bool,
    ) -> Self {
        Self {
            name,
            source,
            predicate,
            enabled,
            restart_if_crash,
            sigkill: false,
            daemon: false,
            rust: availability(name),
        }
    }
}

fn availability(name: &str) -> RustAvailability {
    let (package, binary) = match name {
        "xiaoge_data" => ("openpilot-xiaoge", "openpilot-xiaoge"),
        "carrot_cluster" => ("openpilot-cluster", "openpilot-cluster-autorun"),
        "_pandad" => ("openpilot-pandad", "openpilot-pandad"),
        "beep" => ("openpilot-beepd", "openpilot-beepd"),
        "bridge" => ("openpilot-bridge", "bridge"),
        "camerad" => ("openpilot-camerad-runtime", "openpilot-camerad"),
        "calibrationd" => ("openpilot-calibrationd", "openpilot-calibrationd"),
        "carrot_navi" => ("openpilot-carrot-navi", "openpilot-carrot-navi"),
        "carrot_server" => ("openpilot-carrot-server", "openpilot-carrot-server"),
        "card" => ("openpilot-card", "openpilot-card"),
        "carrot_bluetooth" => ("openpilot-bluetooth", "openpilot-bluetoothd"),
        "carrot_man" => ("openpilot-carrot-man", "openpilot-carrot-man"),
        "controlsd" => ("openpilot-controlsd", "openpilot-controlsd"),
        "cweb_push" => ("openpilot-cweb-push", "openpilot-cweb-push"),
        "deleter" => ("openpilot-deleter", "openpilot-deleter"),
        "dmonitoringd" => ("openpilot-dmonitoringd", "openpilot-dmonitoringd"),
        "dmonitoringmodeld" => ("openpilot-dmonitoringmodeld", "openpilot-dmonitoringmodeld"),
        "encoderd" | "stream_encoderd" | "carrot_vision_encoderd" => {
            ("openpilot-encoderd", "openpilot-encoderd")
        }
        "feedbackd" => ("openpilot-feedbackd", "openpilot-feedbackd"),
        "hardwared" => ("openpilot-hardwared", "openpilot-hardwared"),
        "jetlinkd" => ("openpilot-jetlink", "jetlinkd-rs"),
        "journald" => ("openpilot-journald", "journald-rs"),
        "joystick" => ("openpilot-control-tools", "openpilot-joystick"),
        "joystickd" => ("openpilot-control-tools", "openpilot-joystickd"),
        "lagd" => ("openpilot-lagd", "openpilot-lagd"),
        "locationd" => ("openpilot-locationd", "openpilot-locationd"),
        "lateral_maneuversd" => ("openpilot-control-tools", "openpilot-lateral-maneuversd"),
        "loggerd" => ("openpilot-loggerd", "openpilot-loggerd"),
        "logmessaged" => ("openpilot-logmessaged", "openpilot-logmessaged"),
        "manage_athenad" => ("openpilot-athena", "openpilot-manage-athenad"),
        "maneuversd" => (
            "openpilot-control-tools",
            "openpilot-longitudinal-maneuversd",
        ),
        "micd" => ("openpilot-micd", "openpilot-micd"),
        "modeld" => ("openpilot-driving-modeld", "openpilot-driving-modeld"),
        "modem" => ("openpilot-modem", "openpilot-modem"),
        "mapsd" => ("openpilot-mapsd", "openpilot-mapsd"),
        "navd" => ("openpilot-navd", "openpilot-navd"),
        "pandad" => ("openpilot-pandad", "openpilot-pandad-supervisor"),
        "paramsd" => ("openpilot-paramsd", "openpilot-paramsd"),
        "pigeond" => ("openpilot-ublox", "openpilot-pigeond"),
        "proclogd" => ("openpilot-proclogd", "openpilot-proclogd-runtime"),
        "plannerd" => ("openpilot-plannerd", "openpilot-plannerd"),
        "qcomgpsd" => ("openpilot-qcomgpsd", "openpilot-qcomgpsd"),
        "radarcan" => ("openpilot-radarcan", "openpilot-radarcan"),
        "radard" => ("openpilot-radard", "openpilot-radard"),
        "selfdrived" => ("openpilot-selfdrived", "openpilot-selfdrived"),
        "signalcolord" => ("openpilot-signal-observer", "openpilot-signalcolord"),
        "sensord" => ("openpilot-sensord", "openpilot-sensord"),
        "soundd" => ("openpilot-soundd", "openpilot-soundd"),
        "statsd" => ("openpilot-statsd", "statsd-rs"),
        "timed" => ("openpilot-timed", "openpilot-timed"),
        "tombstoned" => ("openpilot-tombstoned", "openpilot-tombstoned"),
        "torqued" => ("openpilot-torqued", "openpilot-torqued"),
        "ubloxd" => ("openpilot-ublox", "openpilot-ubloxd"),
        "ui" => ("openpilot-ui-application", "openpilot-ui"),
        "updated" => ("openpilot-updated", "openpilot-updated"),
        "uploader" => ("openpilot-uploader", "openpilot-uploader"),
        "webrtcd" => ("openpilot-webrtc", "openpilot-webrtcd"),
        "carrot_webrtcd" => ("openpilot-webrtc", "openpilot-carrot-webrtcd"),
        "webcamerad" => ("openpilot-webcam", "openpilot-webcam"),
        "webjoystick" => ("openpilot-webjoystick", "openpilot-webjoystick"),
        _ => return RustAvailability::NotPorted,
    };
    RustAvailability::Candidate { package, binary, limitation: "Isolated host candidate; native external dependencies remain; manager selection, complete startup/upload and AGNOS/device acceptance pending. See rust/port-status.json for component-specific limits." }
}

/// All registered entries, in source order, including disabled and unported
/// processes. SourceProcess must never be used to launch a Python fallback.
pub fn catalog(config: ImportConfig) -> Vec<Descriptor> {
    use Predicate::*;
    use SourceProcess::*;
    vec![
        Descriptor::new(
            "manage_athenad",
            Persistent {
                module: "openpilot.system.athena.manage_athenad",
                pid_key: "AthenadPid",
            },
            Always,
            true,
            false,
        ),
        Descriptor::new(
            "loggerd",
            Native {
                cwd: "openpilot/system/loggerd",
                argv: &["./loggerd"],
            },
            Logging,
            true,
            false,
        ),
        Descriptor::new(
            "encoderd",
            Native {
                cwd: "openpilot/system/loggerd",
                argv: &["./encoderd"],
            },
            Onroad,
            true,
            false,
        ),
        Descriptor::new(
            "stream_encoderd",
            Native {
                cwd: "openpilot/system/loggerd",
                argv: &["./encoderd", "--stream"],
            },
            NotCar,
            true,
            false,
        ),
        Descriptor::new(
            "carrot_vision_encoderd",
            Native {
                cwd: "openpilot/system/loggerd",
                argv: &["./encoderd", "--carrot-vision-road"],
            },
            All(&[IsCar, WebRtc]),
            true,
            false,
        ),
        Descriptor::new(
            "logmessaged",
            Python {
                module: "openpilot.system.logmessaged",
            },
            Always,
            true,
            false,
        ),
        Descriptor::new(
            "camerad",
            Native {
                cwd: "openpilot/system/camerad",
                argv: &["./camerad"],
            },
            DriverView,
            !config.webcam,
            false,
        ),
        Descriptor::new(
            "webcamerad",
            Python {
                module: "openpilot.tools.webcam.camerad",
            },
            DriverView,
            config.webcam,
            false,
        ),
        Descriptor::new(
            "proclogd",
            Python {
                module: "openpilot.system.proclogd",
            },
            Onroad,
            !config.darwin,
            false,
        ),
        Descriptor::new(
            "journald",
            Python {
                module: "openpilot.system.journald",
            },
            Onroad,
            !config.darwin,
            false,
        ),
        Descriptor::new(
            "micd",
            Python {
                module: "openpilot.system.micd",
            },
            IsCar,
            true,
            false,
        ),
        Descriptor::new(
            "timed",
            Python {
                module: "openpilot.system.timed",
            },
            Always,
            !config.pc,
            false,
        ),
        Descriptor::new(
            "modeld",
            Python {
                module: "openpilot.selfdrive.modeld.modeld",
            },
            Onroad,
            true,
            false,
        ),
        Descriptor::new(
            "signalcolord",
            Python {
                module: "openpilot.selfdrive.modeld.signal_color_shadow",
            },
            SignalObserver,
            true,
            false,
        ),
        Descriptor::new(
            "dmonitoringmodeld",
            Python {
                module: "openpilot.selfdrive.modeld.dmonitoringmodeld",
            },
            DriverMonitoringModel,
            config.webcam || !config.pc,
            false,
        ),
        Descriptor::new(
            "sensord",
            Python {
                module: "openpilot.system.sensord.sensord",
            },
            Onroad,
            !config.pc,
            false,
        ),
        Descriptor::new(
            "ui",
            Python {
                module: "openpilot.selfdrive.ui.ui",
            },
            Always,
            true,
            true,
        ),
        Descriptor::new(
            "soundd",
            Python {
                module: "openpilot.selfdrive.ui.soundd",
            },
            DriverView,
            true,
            false,
        ),
        Descriptor::new(
            "locationd",
            Python {
                module: "openpilot.selfdrive.locationd.locationd",
            },
            Onroad,
            true,
            false,
        ),
        Descriptor::new(
            "_pandad",
            Native {
                cwd: "openpilot/selfdrive/pandad",
                argv: &["./pandad"],
            },
            Always,
            false,
            false,
        ),
        Descriptor::new(
            "calibrationd",
            Python {
                module: "openpilot.selfdrive.locationd.calibrationd",
            },
            Onroad,
            true,
            false,
        ),
        Descriptor::new(
            "torqued",
            Python {
                module: "openpilot.selfdrive.locationd.torqued",
            },
            Onroad,
            true,
            false,
        ),
        Descriptor::new(
            "controlsd",
            Python {
                module: "openpilot.selfdrive.controls.controlsd",
            },
            All(&[NotJoystick, IsCar]),
            true,
            false,
        ),
        Descriptor::new(
            "joystickd",
            Python {
                module: "openpilot.tools.joystick.joystickd",
            },
            Any(&[Joystick, NotCar]),
            true,
            false,
        ),
        Descriptor::new(
            "selfdrived",
            Python {
                module: "openpilot.selfdrive.selfdrived.selfdrived",
            },
            Onroad,
            true,
            false,
        ),
        Descriptor::new(
            "card",
            Python {
                module: "openpilot.selfdrive.car.card",
            },
            Onroad,
            true,
            false,
        ),
        Descriptor::new(
            "deleter",
            Python {
                module: "openpilot.system.loggerd.deleter",
            },
            Always,
            true,
            false,
        ),
        Descriptor::new(
            "dmonitoringd",
            Python {
                module: "openpilot.selfdrive.monitoring.dm2d",
            },
            DriverMonitoring,
            config.webcam || !config.pc,
            false,
        ),
        Descriptor::new(
            "qcomgpsd",
            Python {
                module: "openpilot.system.qcomgpsd.qcomgpsd",
            },
            QcomGps,
            config.tici,
            false,
        ),
        Descriptor::new(
            "navd",
            Python {
                module: "openpilot.selfdrive.navd.navd",
            },
            Onroad,
            true,
            false,
        ),
        // carrot's route-map renderer for N16 (NOO); idle without demand.
        Descriptor::new(
            "mapsd",
            Python {
                module: "openpilot.selfdrive.navigation.mapsd",
            },
            Onroad,
            true,
            false,
        ),
        Descriptor::new(
            "pandad",
            Python {
                module: "openpilot.selfdrive.pandad.pandad",
            },
            Always,
            true,
            false,
        ),
        Descriptor::new(
            "paramsd",
            Python {
                module: "openpilot.selfdrive.locationd.paramsd",
            },
            Onroad,
            true,
            false,
        ),
        Descriptor::new(
            "lagd",
            Python {
                module: "openpilot.selfdrive.locationd.lagd",
            },
            Onroad,
            true,
            false,
        ),
        Descriptor::new(
            "ubloxd",
            Python {
                module: "openpilot.system.ubloxd.ubloxd",
            },
            Ublox,
            config.tici,
            false,
        ),
        Descriptor::new(
            "pigeond",
            Python {
                module: "openpilot.system.ubloxd.pigeond",
            },
            Ublox,
            config.tici,
            false,
        ),
        Descriptor::new(
            "plannerd",
            Python {
                module: "openpilot.selfdrive.controls.plannerd",
            },
            NotLongManeuver,
            true,
            false,
        ),
        Descriptor::new(
            "maneuversd",
            Python {
                module: "openpilot.tools.longitudinal_maneuvers.maneuversd",
            },
            LongManeuver,
            true,
            false,
        ),
        Descriptor::new(
            "lateral_maneuversd",
            Python {
                module: "openpilot.tools.lateral_maneuvers.lateral_maneuversd",
            },
            LatManeuver,
            true,
            false,
        ),
        Descriptor::new(
            "radard",
            Python {
                module: "openpilot.selfdrive.carrot.radar.radard_dpath",
            },
            Onroad,
            true,
            false,
        ),
        Descriptor::new(
            "radarcan",
            Python {
                module: "openpilot.selfdrive.carrot.radar.radarcan",
            },
            Onroad,
            true,
            false,
        ),
        Descriptor::new(
            "hardwared",
            Python {
                module: "openpilot.system.hardware.hardwared",
            },
            Always,
            true,
            false,
        ),
        Descriptor::new(
            "jetlinkd",
            Python {
                module: "openpilot.selfdrive.modeld.jetlink.daemon",
            },
            Always,
            config.tici,
            true,
        ),
        Descriptor::new(
            "modem",
            Python {
                module: "openpilot.system.hardware.tici.modem",
            },
            Always,
            config.tici,
            false,
        ),
        Descriptor::new(
            "tombstoned",
            Python {
                module: "openpilot.system.tombstoned",
            },
            Always,
            !config.pc,
            false,
        ),
        Descriptor::new(
            "updated",
            Python {
                module: "openpilot.system.updated.updated",
            },
            Updated,
            !config.pc,
            false,
        ),
        Descriptor::new(
            "statsd",
            Python {
                module: "openpilot.system.statsd",
            },
            Always,
            true,
            false,
        ),
        Descriptor::new(
            "feedbackd",
            Python {
                module: "openpilot.selfdrive.ui.feedback.feedbackd",
            },
            Onroad,
            true,
            false,
        ),
        Descriptor::new(
            "bridge",
            Native {
                cwd: "openpilot/cereal/messaging",
                argv: &["./bridge"],
            },
            NotCar,
            true,
            false,
        ),
        Descriptor::new(
            "webrtcd",
            Python {
                module: "openpilot.system.webrtc.webrtcd",
            },
            NotCar,
            true,
            false,
        ),
        Descriptor::new(
            "carrot_webrtcd",
            Python {
                module: "openpilot.system.webrtc.carrot_webrtcd",
            },
            All(&[IsCar, WebRtc]),
            true,
            false,
        ),
        Descriptor::new(
            "webjoystick",
            Python {
                module: "openpilot.tools.bodyteleop.web",
            },
            NotCar,
            config.bodyteleop_available,
            false,
        ),
        Descriptor::new(
            "joystick",
            Python {
                module: "openpilot.tools.joystick.joystick_control",
            },
            All(&[Joystick, IsCar]),
            true,
            false,
        ),
        Descriptor::new(
            "carrot_man",
            Python {
                module: "openpilot.selfdrive.carrot.carrot_man",
            },
            Always,
            true,
            true,
        ),
        Descriptor::new(
            "carrot_navi",
            Python {
                module: "openpilot.selfdrive.carrot.carrot_navi",
            },
            Always,
            true,
            true,
        ),
        Descriptor::new(
            "carrot_server",
            Python {
                module: "openpilot.selfdrive.carrot.carrot_server",
            },
            Always,
            !config.carrot_web_external,
            false,
        ),
        Descriptor::new(
            "carrot_bluetooth",
            Python {
                module: "openpilot.selfdrive.carrot.bluetooth.daemon",
            },
            Always,
            config.tici,
            true,
        ),
        Descriptor::new(
            "cweb_push",
            Python {
                module: "openpilot.selfdrive.carrot.cweb_push",
            },
            Always,
            !config.pc,
            false,
        ),
        Descriptor::new(
            "carrot_cluster",
            Python {
                module: "openpilot.selfdrive.carrot.cluster_autorun",
            },
            ClusterHud,
            true,
            true,
        ),
        Descriptor::new(
            "xiaoge_data",
            Python {
                module: "openpilot.selfdrive.carrot.xiaoge_data",
            },
            ShareData,
            true,
            false,
        ),
        Descriptor::new(
            "beep",
            Python {
                module: "openpilot.selfdrive.controls.beep",
            },
            C3xLite,
            config.tici,
            false,
        ),
    ]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn carrot_man_candidate_keeps_source_registration() {
        let descriptor = catalog(ImportConfig {
            pc: true,
            tici: false,
            webcam: false,
            carrot_web_external: false,
            darwin: false,
            bodyteleop_available: false,
        })
        .into_iter()
        .find(|descriptor| descriptor.name == "carrot_man")
        .expect("registered CarrotMan");
        assert_eq!(
            descriptor.source,
            SourceProcess::Python {
                module: "openpilot.selfdrive.carrot.carrot_man"
            }
        );
        assert!(descriptor.enabled && descriptor.restart_if_crash);
        assert!(matches!(
            descriptor.rust,
            RustAvailability::Candidate {
                package: "openpilot-carrot-man",
                binary: "openpilot-carrot-man",
                ..
            }
        ));
    }

    #[test]
    fn signal_observer_is_registered_and_runs_only_onroad_with_a_selected_mode() {
        use crate::{Error, GpsPaths, Parameters, Predicate, State};
        struct Mode(Result<i32, ()>);
        impl Parameters for Mode {
            fn get_bool(&mut self, _key: &str) -> Result<bool, Error> {
                Ok(false)
            }
            fn get_int(&mut self, key: &str) -> Result<i32, Error> {
                assert_eq!(key, "SignalAssistMode");
                self.0.map_err(|()| {
                    Error::Parameter(openpilot_params::Error::Io(std::io::Error::other(
                        "unreadable",
                    )))
                })
            }
            fn get_bool_default(&mut self, _key: &str) -> Result<bool, Error> {
                Ok(false)
            }
            fn put_bool(&mut self, _key: &str, _value: bool) -> Result<(), Error> {
                Ok(())
            }
        }
        struct NoPaths;
        impl GpsPaths for NoPaths {
            fn exists(&mut self, _path: &str) -> bool {
                false
            }
        }
        let descriptor = catalog(ImportConfig {
            pc: false,
            tici: true,
            webcam: false,
            carrot_web_external: false,
            darwin: false,
            bodyteleop_available: false,
        })
        .into_iter()
        .find(|descriptor| descriptor.name == "signalcolord")
        .expect("registered signal observer");
        assert!(descriptor.enabled && !descriptor.restart_if_crash);
        assert_eq!(descriptor.predicate, Predicate::SignalObserver);
        assert!(matches!(
            descriptor.rust,
            RustAvailability::Candidate {
                package: "openpilot-signal-observer",
                binary: "openpilot-signalcolord",
                ..
            }
        ));
        let run = |started, mode| {
            descriptor
                .predicate
                .evaluate(
                    State {
                        started,
                        not_car: false,
                    },
                    &mut Mode(mode),
                    &mut NoPaths,
                )
                .unwrap()
        };
        assert!(run(true, Ok(1)) && run(true, Ok(2)));
        assert!(!run(true, Ok(0)) && !run(true, Ok(3)) && !run(true, Err(())));
        assert!(!run(false, Ok(1)));
    }
}
