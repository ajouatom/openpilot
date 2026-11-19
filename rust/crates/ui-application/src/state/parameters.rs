use crate::{params::Read, Error};
use serde::{Deserialize, Serialize};
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct RealtimeParams {
    pub record_audio: bool,
    pub is_metric: bool,
    pub always_on_dm: bool,
}
impl RealtimeParams {
    pub fn read(params: &impl Read) -> Result<Self, Error> {
        Ok(Self {
            record_audio: params.boolean("RecordAudio")?,
            is_metric: params.boolean("IsMetric")?,
            always_on_dm: params.boolean("AlwaysOnDM")?,
        })
    }
}
#[derive(Clone, Copy, Debug, Default, Serialize, Deserialize)]
pub struct CarConfig {
    pub alpha_longitudinal_available: bool,
    pub max_lateral_accel: f64,
    pub openpilot_longitudinal_control: bool,
}
impl CarConfig {
    pub fn decode(bytes: &[u8]) -> Result<Self, Error> {
        let reader =
            capnp::serialize::read_message(&mut std::io::Cursor::new(bytes), Default::default())?;
        let cp = reader.get_root::<openpilot_cereal::car_capnp::car_params::Reader<'_>>()?;
        Ok(Self {
            alpha_longitudinal_available: cp.get_alpha_longitudinal_available(),
            max_lateral_accel: f64::from(cp.get_max_lateral_accel()),
            openpilot_longitudinal_control: cp.get_openpilot_longitudinal_control(),
        })
    }
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Badge {
    pub text: String,
    pub style: String,
}
impl From<openpilot_jetlink::status::Badge> for Badge {
    fn from(value: openpilot_jetlink::status::Badge) -> Self {
        Self {
            text: value.text,
            style: value.style,
        }
    }
}
#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct ModelStatus {
    pub compiled: bool,
    pub compile_pending: bool,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub delivery_badge: Option<Badge>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub recovery_badge: Option<Badge>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub jetlink_badge: Option<Badge>,
}
#[derive(Clone, Debug, Default, Serialize)]
pub struct SlowParams {
    pub car: Option<CarConfig>,
    pub has_longitudinal_control: bool,
    pub show_debug_ui: i32,
    pub show_date_time: i32,
    pub show_radar_info: i32,
    pub show_brightness_ratio: f64,
    pub show_model_view: i32,
    pub share_data: bool,
    pub show_camera_with_cluster: bool,
    pub usbgpu_present: bool,
    pub usbgpu_compiled: bool,
    pub usbgpu_compile_pending: bool,
    pub usbgpu_loading: bool,
    pub usbgpu_active: bool,
    pub usbgpu_startup_failed: bool,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub delivery_badge: Option<Badge>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub recovery_badge: Option<Badge>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub jetlink_badge: Option<Badge>,
    /// Cheddar running (white) or chosen but not loaded (red); none for CD210.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub internal_model_badge: Option<Badge>,
}

/// The badge for drivingd's InternalDrivingModelStatus.
fn internal_model_badge(bytes: Option<&[u8]>) -> Option<Badge> {
    let status: serde_json::Value = serde_json::from_slice(bytes?).ok()?;
    let badge = |text: &str, style: &str| Badge {
        text: text.into(),
        style: style.into(),
    };
    match (status["selected"].as_str(), status["running"].as_str()) {
        (_, Some("cheddar")) => Some(badge("CHEDDAR", "ready")),
        (Some("cheddar"), _) => Some(badge("CHEDDAR ERROR", "error")),
        _ => None,
    }
}

impl SlowParams {
    pub fn refresh(&mut self, params: &impl Read, models: ModelStatus) -> Result<(), Error> {
        if let Some(bytes) = params.bytes("CarParamsPersistent")? {
            let car = CarConfig::decode(&bytes)?;
            self.has_longitudinal_control = if car.alpha_longitudinal_available {
                params.boolean("AlphaLongitudinalEnabled")?
            } else {
                car.openpilot_longitudinal_control
            };
            self.car = Some(car);
        }
        self.show_debug_ui = params.integer("ShowDebugUI")?;
        self.show_date_time = params.integer("ShowDateTime")?;
        self.show_radar_info = params.integer("ShowRadarInfo")?;
        self.show_brightness_ratio = f64::from(params.integer("ShowCustomBrightness")?) / 100.0;
        self.show_model_view = params.integer("ShowModelView")?;
        self.share_data = params.boolean("ShareData")?;
        self.show_camera_with_cluster = params.integer("ShowCameraWithCluster")? == 1;
        self.usbgpu_present = params.boolean("UsbGpuPresent")?;
        self.usbgpu_compiled = models.compiled;
        self.usbgpu_compile_pending = models.compile_pending;
        self.delivery_badge = models.delivery_badge;
        self.recovery_badge = models.recovery_badge;
        self.jetlink_badge = models.jetlink_badge;
        self.usbgpu_loading = params.boolean("UsbGpuLoading")?;
        self.usbgpu_active = params.boolean("UsbGpuActive")?;
        self.usbgpu_startup_failed = params.boolean("UsbGpuStartupFailed")?;
        self.internal_model_badge =
            internal_model_badge(params.bytes("InternalDrivingModelStatus")?.as_deref());
        Ok(())
    }
}

#[cfg(test)]
mod internal_model_badge_tests {
    use super::internal_model_badge;

    fn badge(json: &str) -> Option<(String, String)> {
        internal_model_badge(Some(json.as_bytes())).map(|badge| (badge.text, badge.style))
    }

    #[test]
    fn shows_cheddar_and_a_failed_choice_but_not_cd210() {
        assert_eq!(
            badge(r#"{"selected":"cheddar","running":"cheddar","error":null}"#),
            Some(("CHEDDAR".into(), "ready".into()))
        );
        assert_eq!(
            badge(r#"{"selected":"cheddar","running":"cd210","error":"load: x"}"#),
            Some(("CHEDDAR ERROR".into(), "error".into()))
        );
        assert_eq!(
            badge(r#"{"selected":"cd210","running":"cd210","error":null}"#),
            None
        );
        assert_eq!(internal_model_badge(None).map(|badge| badge.text), None);
    }
}
