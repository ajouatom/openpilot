use crate::alerts::{Alert, Priority};
use crate::events::Callback;
use openpilot_cereal::car_capnp::car_control::h_u_d_control::{AudibleAlert, VisualAlert};
use openpilot_cereal::log_capnp::selfdrive_state::{AlertSize, AlertStatus};
use openpilot_cereal::log_capnp::LongitudinalPersonality;
use openpilot_ui_framework::multilang::Multilang;
use serde::Deserialize;

#[derive(Debug, thiserror::Error)]
pub enum Error {
    #[error("cannot round nonfinite alert value to an integer")]
    NonFiniteInteger,
    #[error("missing alert text parameter {0}")]
    MissingText(&'static str),
    #[error("invalid {expected} alert parameter {key}")]
    ParameterType { key: String, expected: &'static str },
    #[error(transparent)]
    Parameter(#[from] openpilot_params::Error),
    #[error(transparent)]
    TextParameter(#[from] openpilot_params_typed::Error),
    #[error("fatal Cython get_int conversion for {key}: {source}")]
    IntegerParameter {
        key: String,
        source: openpilot_beepd::IntegerError,
    },
}

pub trait AlertParams {
    fn text(&mut self, key: &str) -> Result<Option<String>, Error>;
    fn integer(&mut self, key: &str) -> Result<i32, Error>;
    fn boolean(&mut self, key: &str) -> Result<bool, Error>;
}

pub struct NativeParams<'a> {
    pub params: &'a openpilot_params::Params,
    pub logger: &'a mut openpilot_logging::producer::Logger,
}

impl NativeParams<'_> {
    fn raw(&self, key: &str) -> Result<Vec<u8>, Error> {
        match self.params.get(key) {
            Ok(value) => Ok(value.unwrap_or_default()),
            Err(openpilot_params::Error::Io(_)) => Ok(Vec::new()),
            Err(error) => Err(error.into()),
        }
    }
}

impl AlertParams for NativeParams<'_> {
    fn text(&mut self, key: &str) -> Result<Option<String>, Error> {
        Ok(openpilot_params_typed::get_string(
            self.params,
            key,
            self.logger,
        )?)
    }

    fn integer(&mut self, key: &str) -> Result<i32, Error> {
        openpilot_beepd::integer(&self.raw(key)?).map_err(|source| Error::IntegerParameter {
            key: key.to_owned(),
            source,
        })
    }

    fn boolean(&mut self, key: &str) -> Result<bool, Error> {
        Ok(self.raw(key)? == b"1")
    }
}

#[derive(Default, Deserialize)]
pub struct Process {
    pub name: String,
    pub running: bool,
    pub should_be_running: bool,
}

#[derive(Default, Deserialize)]
pub struct Health {
    pub service: String,
    pub all_checks: bool,
}

#[derive(Default, Deserialize)]
pub struct Snapshot {
    pub brand: String,
    pub flags: u32,
    pub min_enable_speed: f64,
    pub min_steer_speed: f64,
    pub ego_speed: f64,
    pub model_velocity: Vec<f64>,
    pub calibration_recalibrating: bool,
    pub calibration_percent: i8,
    pub calibration_rpy: Vec<f64>,
    pub feedback_block: u16,
    pub free_space_percent: f64,
    pub processes: Vec<Process>,
    pub health: Vec<Health>,
    pub angle_offset_valid: bool,
    pub angle_offset: f64,
    pub steer_ratio_valid: bool,
    pub steer_ratio: f64,
    pub stiffness_factor_valid: bool,
    pub stiffness_factor: f64,
    pub cpu_temps: Vec<f64>,
    pub gpu_temps: Vec<f64>,
    pub memory_temp: f64,
    pub memory_usage_percent: i8,
    pub frame_drop_percent: f64,
    pub accel: f64,
    pub torque: f64,
    pub debug_text_1: String,
    pub debug_text_2: String,
}

pub struct Context<'a> {
    pub snapshot: &'a Snapshot,
    pub language: &'a Multilang,
    pub params: &'a mut dyn AlertParams,
    pub metric: bool,
    pub soft_disable_time: u32,
    pub personality: LongitudinalPersonality,
    pub branch: &'a str,
    pub replay: bool,
    pub mici: bool,
}

fn normal(first: impl Into<String>, second: impl Into<String>) -> Alert {
    let second = second.into();
    Alert {
        alert_text_1: first.into(),
        alert_size: if second.is_empty() {
            AlertSize::Small
        } else {
            AlertSize::Mid
        },
        alert_text_2: Some(second),
        priority: Priority::Lower,
        duration: 20,
        ..Alert::default()
    }
}

fn integer(value: f64) -> Result<String, Error> {
    if !value.is_finite() {
        return Err(Error::NonFiniteInteger);
    }
    let rounded = value.round_ties_even();
    Ok(if rounded == 0.0 {
        "0".into()
    } else {
        format!("{rounded:.0}")
    })
}

fn fixed(value: f64, digits: usize) -> String {
    if value.is_nan() {
        "nan".into()
    } else {
        format!("{value:.digits$}")
    }
}

fn maximum(values: &[f64], default: f64) -> f64 {
    let Some((&first, rest)) = values.split_first() else {
        return default;
    };
    rest.iter().fold(
        first,
        |current, value| if *value > current { *value } else { current },
    )
}

impl Context<'_> {
    fn tr(&self, text: &str) -> String {
        self.language.tr(text).into()
    }

    fn no_entry(&self, second: impl Into<String>, first: impl Into<String>) -> Alert {
        let (first, second) = (first.into(), second.into());
        let (first, second) = if self.mici {
            (second, first)
        } else {
            (first, second)
        };
        Alert {
            alert_text_1: first,
            alert_text_2: Some(second),
            alert_size: AlertSize::Mid,
            priority: Priority::Low,
            audible_alert: AudibleAlert::Refuse,
            duration: 300,
            ..Alert::default()
        }
    }

    fn display_speed(&self, speed: f64) -> Result<String, Error> {
        let speed = integer(
            speed
                * if self.metric {
                    3.6
                } else {
                    3.6 * (1.0 / 1.609344)
                },
        )?;
        Ok(format!(
            "{speed} {}",
            if self.metric { "km/h" } else { "mph" }
        ))
    }

    pub fn resolve(&mut self, callback: &Callback) -> Result<Alert, Error> {
        let s = self.snapshot;
        Ok(match callback {
            Callback::SoftDisableAlert { text } | Callback::UserSoftDisableAlert { text } => {
                let immediate = self.soft_disable_time < 50;
                let user = matches!(callback, Callback::UserSoftDisableAlert { .. });
                Alert {
                    alert_text_1: if user && !immediate {
                        "openpilot will disengage"
                    } else {
                        "TAKE CONTROL IMMEDIATELY"
                    }
                    .into(),
                    alert_text_2: Some(text.clone()),
                    alert_status: if immediate {
                        AlertStatus::Critical
                    } else {
                        AlertStatus::UserPrompt
                    },
                    alert_size: AlertSize::Full,
                    priority: if immediate {
                        Priority::Highest
                    } else {
                        Priority::Mid
                    },
                    visual_alert: VisualAlert::SteerRequired,
                    audible_alert: if immediate {
                        AudibleAlert::WarningImmediate
                    } else {
                        AudibleAlert::WarningSoft
                    },
                    duration: if immediate { 400 } else { 200 },
                    ..Alert::default()
                }
            }
            Callback::StartupMasterAlert => Alert {
                alert_text_1: "WARNING: This branch is not tested".into(),
                alert_text_2: Some(if self.replay { "replay" } else { self.branch }.into()),
                alert_status: AlertStatus::UserPrompt,
                alert_size: if self.mici {
                    AlertSize::Small
                } else {
                    AlertSize::Mid
                },
                priority: Priority::Lower,
                duration: 500,
                ..Alert::default()
            },
            Callback::BelowEngageSpeedAlert => self.no_entry(
                self.tr("Drive above {speed} to engage")
                    .replace("{speed}", &self.display_speed(s.min_enable_speed)?),
                "openpilot Unavailable",
            ),
            Callback::BelowSteerSpeedAlert => Alert {
                alert_text_1: self
                    .tr("Steer Assist Unavailable Below {speed}")
                    .replace("{speed}", &self.display_speed(s.min_steer_speed)?),
                alert_status: AlertStatus::UserPrompt,
                alert_size: AlertSize::Small,
                priority: Priority::Low,
                audible_alert: AudibleAlert::Prompt,
                duration: 40,
                ..Alert::default()
            },
            Callback::CalibrationIncompleteAlert => Alert {
                alert_text_1: self
                    .tr("{calibration_state}: {percent:.0f}%")
                    .replace(
                        "{calibration_state}",
                        &self.tr(if s.calibration_recalibrating {
                            "Recalibrating"
                        } else {
                            "Calibrating"
                        }),
                    )
                    .replace("{percent:.0f}", &s.calibration_percent.to_string()),
                alert_text_2: Some(self.tr("Drive Above {speed}").replace(
                    "{speed}",
                    &self.display_speed(15.0 * (1.609344 * (1.0 / 3.6)))?,
                )),
                alert_size: AlertSize::Mid,
                duration: 20,
                ..Alert::default()
            },
            Callback::AudioFeedbackAlert => {
                let seconds = (10.0 - (f64::from(s.feedback_block) + 1.0) * 800.0 / 16000.0)
                    .round_ties_even() as i64;
                Alert {
                    priority: Priority::Low,
                    ..normal(
                        "Recording Audio Feedback",
                        self.language
                            .trn(
                                "{seconds} second remaining. Press again to save early.",
                                "{seconds} seconds remaining. Press again to save early.",
                                seconds,
                            )
                            .replace("{seconds}", &seconds.to_string()),
                    )
                }
            }
            Callback::TorqueNnLoadAlert => {
                let name = self.params.text("NNFFModelName")?;
                let empty = name.as_deref() == Some("");
                Alert {
                    alert_text_1: if empty {
                        "NNFF Torque Controller not available"
                    } else {
                        "NNFF Torque Controller loaded"
                    }
                    .into(),
                    alert_text_2: if empty {
                        Some("Donate logs to Twilsonco to get it added!".into())
                    } else {
                        name
                    },
                    alert_status: AlertStatus::UserPrompt,
                    alert_size: AlertSize::Mid,
                    priority: Priority::Low,
                    audible_alert: AudibleAlert::Prompt,
                    duration: if empty { 600 } else { 500 },
                    ..Alert::default()
                }
            }
            Callback::OutOfSpaceAlert => normal(
                "Out of Storage",
                self.tr("{percent}% full")
                    .replace("{percent}", &integer(100.0 - s.free_space_percent)?),
            ),
            Callback::PosenetInvalidAlert => self.no_entry(
                self.tr("Speed Error: {error:.1f} m/s").replace(
                    "{error:.1f}",
                    &fixed(
                        s.ego_speed - s.model_velocity.first().copied().unwrap_or(f64::NAN),
                        1,
                    ),
                ),
                "Posenet Speed Invalid",
            ),
            Callback::ProcessNotRunningAlert => self.no_entry(
                s.processes
                    .iter()
                    .filter(|p| !p.running && p.should_be_running)
                    .map(|p| p.name.as_str())
                    .collect::<Vec<_>>()
                    .join(", "),
                "Process Not Running",
            ),
            Callback::CommIssueAlert => self.no_entry(
                s.health
                    .iter()
                    .filter(|h| !h.all_checks)
                    .take(4)
                    .map(|h| h.service.as_str())
                    .collect::<Vec<_>>()
                    .join(", "),
                "Communication Issue Between Processes",
            ),
            Callback::CameraMalfunctionAlert => normal(
                "Camera Malfunction",
                [
                    "roadCameraState",
                    "driverCameraState",
                    "wideRoadCameraState",
                ]
                .into_iter()
                .filter(|name| s.health.iter().any(|h| h.service == *name && !h.all_checks))
                .map(|name| name.replace("State", ""))
                .collect::<Vec<_>>()
                .join(", "),
            ),
            Callback::CalibrationInvalidAlert => {
                let (pitch, yaw) = if s.calibration_rpy.len() == 3 {
                    (
                        s.calibration_rpy[1].to_degrees(),
                        s.calibration_rpy[2].to_degrees(),
                    )
                } else {
                    (f64::NAN, f64::NAN)
                };
                normal(
                    "Calibration Invalid",
                    self.tr("Remount Device (Pitch: {pitch:.1f}°, Yaw: {yaw:.1f}°)")
                        .replace("{pitch:.1f}", &fixed(pitch, 1))
                        .replace("{yaw:.1f}", &fixed(yaw, 1)),
                )
            }
            Callback::ParamsdInvalidAlert => {
                let (title, text, key, value) = if !s.angle_offset_valid {
                    (
                        "Steering misalignment detected",
                        "Angle offset too high (Offset: {angle_offset:.1f}°)",
                        "{angle_offset:.1f}",
                        s.angle_offset,
                    )
                } else if !s.steer_ratio_valid {
                    (
                        "Steer ratio mismatch",
                        "Steering rack geometry may be off (Ratio: {steer_ratio:.1f})",
                        "{steer_ratio:.1f}",
                        s.steer_ratio,
                    )
                } else if !s.stiffness_factor_valid {
                    (
                        "Abnormal tire stiffness",
                        "Check tires, pressure, or alignment (Factor: {stiffness_factor:.1f})",
                        "{stiffness_factor:.1f}",
                        s.stiffness_factor,
                    )
                } else {
                    return Ok(self.no_entry("paramsd Temporary Error", "openpilot Unavailable"));
                };
                self.no_entry(self.tr(text).replace(key, &fixed(value, 1)), self.tr(title))
            }
            Callback::OverheatAlert => normal(
                "System Overheated",
                self.tr("{temperature:.0f} °C").replace(
                    "{temperature:.0f}",
                    &fixed(
                        maximum(
                            &[
                                maximum(&s.cpu_temps, 0.0),
                                maximum(&s.gpu_temps, 0.0),
                                s.memory_temp,
                            ],
                            0.0,
                        ),
                        0,
                    ),
                ),
            ),
            Callback::LowMemoryAlert => normal(
                "Low Memory",
                self.tr("{percent}% used")
                    .replace("{percent}", &s.memory_usage_percent.to_string()),
            ),
            Callback::ModeldLaggingAlert => normal(
                "Driving Model Lagging",
                self.tr("{percent:.1f}% frames dropped")
                    .replace("{percent:.1f}", &fixed(s.frame_drop_percent, 1)),
            ),
            Callback::WrongCarModeAlert => self.no_entry(
                self.tr(if s.brand == "honda" {
                    "Enable Main Switch to Engage"
                } else {
                    "Enable Adaptive Cruise to Engage"
                }),
                "openpilot Unavailable",
            ),
            Callback::JoystickAlert => normal(
                "Joystick Mode",
                self.tr("Gas: {gas}%, Steer: {steer}%")
                    .replace("{gas}", &integer(s.accel / 4.0 * 100.0)?)
                    .replace("{steer}", &integer(s.torque * 100.0)?),
            ),
            Callback::LongitudinalManeuverAlert => Alert {
                alert_text_1: s.debug_text_1.clone(),
                alert_text_2: Some(s.debug_text_2.clone()),
                alert_status: if s.debug_text_1.contains("Active") {
                    AlertStatus::UserPrompt
                } else {
                    AlertStatus::Normal
                },
                alert_size: if s.debug_text_2.is_empty() {
                    AlertSize::Small
                } else {
                    AlertSize::Mid
                },
                priority: Priority::Low,
                audible_alert: if s.debug_text_1.contains("Active") {
                    AudibleAlert::Prompt
                } else {
                    AudibleAlert::None
                },
                duration: 20,
                ..Alert::default()
            },
            Callback::PersonalityChangedAlert => Alert {
                duration: 150,
                ..normal(
                    self.tr("Driving Personality: {personality}").replace(
                        "{personality}",
                        match self.personality {
                            LongitudinalPersonality::Aggressive => "Aggressive",
                            LongitudinalPersonality::Standard => "Standard",
                            LongitudinalPersonality::Relaxed => "Relaxed",
                            LongitudinalPersonality::MoreRelaxed => "Morerelaxed",
                        },
                    ),
                    "",
                )
            },
            Callback::InvalidLkasSettingAlert => normal(
                "Invalid LKAS setting",
                self.tr(match s.brand.as_str() {
                    "tesla" => "Switch to Traffic-Aware Cruise Control to engage",
                    "mazda" => "Enable your car's LKAS to engage",
                    "nissan" => "Disable your car's stock LKAS to engage",
                    _ => "Toggle stock LKAS on or off to engage",
                }),
            ),
            Callback::CarParserResult => {
                let results = self.params.text("CanParserResult")?.unwrap_or_default();
                let hint = s.brand == "hyundai"
                    && s.flags & 8 == 0
                    && self.params.integer("HyundaiCameraSCC")? == 0
                    && self.params.boolean("HyundaiCameraSccHint")?;
                Alert {
                    alert_text_1: if hint {
                        "CAN Error: Enable CameraSCC"
                    } else {
                        "CAN Error: Check Connections!!"
                    }
                    .into(),
                    alert_size: if hint || !results.is_empty() {
                        // Small alerts never draw text2.
                        AlertSize::Mid
                    } else {
                        AlertSize::Small
                    },
                    alert_text_2: Some(if hint {
                        "SCC detected on camera bus".into()
                    } else {
                        results
                    }),
                    priority: Priority::Low,
                    duration: 100,
                    creation_delay: 1.0,
                    ..Alert::default()
                }
            }
        })
    }
}
