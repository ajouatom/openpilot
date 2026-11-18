mod diagnostics;
mod input;
mod publication;
mod readiness;
mod step;
use crate::{firmware_query::StartupIo, toggle::MainToggle, xiaoge::VisionResult};
use capnp::message::{Builder, HeapAllocator};
pub use diagnostics::Diagnostics;
use openpilot_can::{Frame, Packet};
use openpilot_cereal::{
    car_capnp::{car_control, car_params, car_state},
    log_capnp::{model_data_v2, radar_state},
};
use openpilot_messaging::state::State;
use openpilot_params::Params;

pub type Message = Builder<HeapAllocator>;
pub const SERVICES: [&str; 13] = [
    "pandaStates",
    "carControl",
    "onroadEvents",
    "carrotMan",
    "longitudinalPlan",
    "radarState",
    "modelV2",
    "drivingModelData",
    "customReservedRawData0",
    "liveParameters",
    "livePose",
    "selfdriveState",
    "lateralPlan",
];

#[derive(Debug, thiserror::Error)]
pub enum Error {
    #[error(transparent)]
    Utf8(#[from] std::str::Utf8Error),
    #[error(transparent)]
    Schema(#[from] capnp::Error),
    #[error(transparent)]
    Enum(#[from] capnp::NotInSchema),
    #[error(transparent)]
    Count(#[from] std::num::TryFromIntError),
    #[error(transparent)]
    Params(#[from] openpilot_params::Error),
    #[error(transparent)]
    Subscription(#[from] openpilot_messaging::state::Error),
    #[error(transparent)]
    CanWire(#[from] crate::can_wire::Error),
    #[error(transparent)]
    Query(#[from] crate::isotp::Error),
    #[error(transparent)]
    Hyundai(#[from] crate::brands::hyundai::Error),
    #[error(transparent)]
    Tesla(#[from] crate::brands::tesla::Error),
    #[error(transparent)]
    Mazda(#[from] crate::brands::mazda::Error),
    #[error(transparent)]
    Nissan(#[from] crate::brands::nissan::Error),
    #[error(transparent)]
    Chrysler(#[from] crate::brands::chrysler::Error),
    #[error(transparent)]
    Rivian(#[from] crate::brands::rivian::Error),
    #[error(transparent)]
    Psa(#[from] crate::brands::psa::Error),
    #[error(transparent)]
    Ford(#[from] crate::brands::ford::Error),
    #[error(transparent)]
    Subaru(#[from] crate::brands::subaru::Error),
    #[error(transparent)]
    Toyota(#[from] crate::brands::toyota::Error),
    #[error(transparent)]
    Gm(#[from] crate::brands::gm::Error),
    #[error(transparent)]
    Honda(#[from] crate::brands::honda::Error),
    #[error(transparent)]
    Volkswagen(#[from] crate::brands::volkswagen::Error),
    #[error(transparent)]
    Can(#[from] openpilot_can::Error),
    #[error(transparent)]
    Policy(#[from] openpilot_control_policy::Error),
    #[error(transparent)]
    VehicleParams(#[from] crate::vehicle_params::Error),
    #[error("native vehicle interface is not implemented: {brand} ({candidate})")]
    UnsupportedVehicle { brand: String, candidate: String },
    #[cfg(feature = "native")]
    #[error(transparent)]
    RuntimeSubscription(#[from] openpilot_messaging::runtime::Error),
    #[cfg(feature = "native")]
    #[error(transparent)]
    Queue(#[from] openpilot_msgq::Error),
    #[cfg(feature = "native")]
    #[error(transparent)]
    Logging(#[from] openpilot_logging::Error),
    #[error(transparent)]
    Io(#[from] std::io::Error),
    #[error("unexpected cereal service for {0}")]
    Event(&'static str),
    #[error("replay controls arrived before any CAN timestamp")]
    ReplayTimestamp,
    #[error("card numeric conversion")]
    Numeric,
    #[error("unknown carlog LOGPRINT level: {0}")]
    LoggingLevel(String),
}

pub struct ApplyInput<'a> {
    pub control: car_control::Reader<'a>,
    pub now_ns: u64,
    /// The card's subscriptions, for vehicle features that check other
    /// services' freshness (Hyundai SPAS indicators).
    pub subscribers: Option<&'a openpilot_messaging::state::State>,
    pub initialized: bool,
    /// carrot `lateral_startup.update(...)`.
    pub lateral_ready: bool,
    pub impact_reboot: bool,
    /// Latest Xiaoge vision result and `CLOCK_MONOTONIC` ns.
    pub vision: Option<&'a VisionResult>,
    pub monotonic_ns: u64,
    pub model: Option<model_data_v2::Reader<'a>>,
    pub radar: Option<radar_state::Reader<'a>>,
}
pub struct ApplyOutput {
    pub actuators: Message,
    pub can: Vec<Frame>,
}

pub struct VehicleLog {
    pub level: crate::query::DiagnosticLevel,
    pub message: String,
}

#[derive(Clone, Copy)]
pub struct SettingsFlags {
    pub metric: bool,
    pub experimental: bool,
}

pub trait Vehicle {
    fn take_param_writes(&mut self) -> Vec<(String, Vec<u8>)> {
        Vec::new()
    }
    fn emit_diagnostics(&mut self, io: &mut impl StepIo) -> Result<(), Error> {
        for line in self.take_diagnostics() {
            io.print(&line)?;
        }
        for line in self.take_warnings() {
            io.car_warning(&line)?;
        }
        for diagnostic in self.take_logs() {
            io.vehicle_log(&diagnostic)?;
        }
        Ok(())
    }
    fn take_logs(&mut self) -> Vec<VehicleLog> {
        Vec::new()
    }
    fn take_diagnostics(&mut self) -> Vec<String> {
        Vec::new()
    }
    fn take_warnings(&mut self) -> Vec<String> {
        Vec::new()
    }
    fn set_speedcam_context(&mut self, _engaged: bool, _cruise_kph: f64) {}
    fn speedcam_gas_tok_consumed(&self) -> bool {
        false
    }
    /// Names of timed-out CAN messages for the canError text2.
    fn invalid_can_messages(&self) -> Vec<String> {
        Vec::new()
    }
    /// Stock SCC set speed (km/h) while a restart hand-over waits (carrot
    /// PR580 `stock_adopt_kph`); 0 = nothing to adopt.
    fn stock_adopt_kph(&self) -> f64 {
        0.
    }
    fn update(&mut self, packets: &[Packet], now_ns: u64) -> Result<Message, Error>;
    fn init(&mut self, io: &mut impl StartupIo) -> Result<(), Error>;
    fn apply(&mut self, input: ApplyInput<'_>) -> Result<ApplyOutput, Error>;
    fn commit_state(&mut self, state: car_state::Reader<'_>) -> Result<(), Error>;
    fn set_soft_hold(&mut self, active: i16);
}
pub trait StateTail {
    fn cruise_speed_kph(&self) -> f64 {
        0.
    }
    fn set_speedcam_gas_tok_consumed(&mut self, _consumed: bool) {}
    fn set_stock_adopt_kph(&mut self, _kph: f64) {}
    fn take_prints(&mut self) -> Vec<String> {
        Vec::new()
    }
    fn take_param_writes(&mut self) -> Vec<(String, Vec<u8>)> {
        Vec::new()
    }
    fn project(&self, state: car_state::Builder<'_>) -> Result<(), Error>;
    fn update(
        &mut self,
        state: car_state::Builder<'_>,
        subscriptions: &State,
        is_metric: bool,
        now: f64,
    ) -> Result<(), Error>;
    fn initialize(
        &mut self,
        previous: car_state::Reader<'_>,
        experimental_mode: bool,
    ) -> Result<(), Error>;
}
pub trait StepIo: StartupIo {
    fn settings_flags(&self) -> Option<SettingsFlags> {
        None
    }
    fn vehicle_log(&mut self, diagnostic: &VehicleLog) -> Result<(), Error> {
        match diagnostic.level {
            crate::query::DiagnosticLevel::Info => {
                Err(Error::Event("vehicle info diagnostic needs a typed sink"))
            }
            crate::query::DiagnosticLevel::Warning => self.warning(&diagnostic.message),
            crate::query::DiagnosticLevel::Error | crate::query::DiagnosticLevel::Exception => {
                Err(Error::Event("vehicle error diagnostic needs a typed sink"))
            }
        }
    }
    fn put_nonblocking(&mut self, key: &str, bytes: &[u8]) -> Result<(), Error>;
    fn print(&mut self, message: &str) -> Result<(), Error> {
        use std::io::Write;
        writeln!(std::io::stdout().lock(), "{message}")?;
        Ok(())
    }
    fn receive_can_raw(&mut self) -> Result<Vec<Vec<u8>>, Error>;
    fn update_subscribers(&mut self) -> Result<(), Error>;
    fn subscribers(&self) -> &State;
    fn monotonic_ns(&mut self) -> u64;
    fn thread_cpu_ns(&mut self) -> u64;
    fn publish(&mut self, topic: &str, bytes: &[u8]) -> Result<(), Error>;
    fn warning(&mut self, message: &str) -> Result<(), Error>;
    fn car_warning(&mut self, message: &str) -> Result<(), Error> {
        self.warning(message)
    }
    fn diagnostics(&mut self, values: &[(&'static str, f64)]) -> Result<(), Error>;
}

pub struct Card {
    pub params: Message,
    pub settings: Params,
    pub initialized_previous: bool,
    lateral_startup: openpilot_control_policy::lateral_readiness::Gate,
    pub can_timeouts: u64,
    /// Last `CanParserResult` written (canError text2).
    can_parser_result: String,
    pub last_actuators: Message,
    previous_state: Message,
    previous_enabled: bool,
    has_controller: bool,
    replay: bool,
    replay_time: Option<u64>,
    toggle: MainToggle,
    vision: Option<VisionResult>,
    vision_error_at: u64,
    vision_sources: [bool; 4],
    vision_sources_published: Option<[bool; 4]>,
    vision_sources_published_ns: u64,
    settings_updated: Option<u64>,
    metric: bool,
    experimental: bool,
    pub remaining: f64,
    pub diagnostics: Diagnostics,
    native_voice: Option<crate::voice_indicator::NativeVoiceIndicator>,
}
impl Card {
    pub fn new(
        params: Message,
        settings: Params,
        replay: bool,
        has_controller: bool,
    ) -> Result<Self, Error> {
        params.get_root_as_reader::<car_params::Reader>()?;
        let metric = settings.get_bool("IsMetric")?;
        let experimental = settings.get_bool("ExperimentalMode")?;
        let mut previous_state = Message::new_default();
        previous_state.init_root::<car_state::Builder>();
        let mut last_actuators = Message::new_default();
        last_actuators.init_root::<car_control::actuators::Builder>();
        let native_voice =
            crate::voice_indicator::native(params.get_root_as_reader::<car_params::Reader>()?)
                .map(crate::voice_indicator::NativeVoiceIndicator::new);
        Ok(Self {
            params,
            settings,
            initialized_previous: false,
            lateral_startup: Default::default(),
            can_timeouts: 0,
            can_parser_result: String::new(),
            last_actuators,
            previous_state,
            previous_enabled: false,
            has_controller,
            replay,
            replay_time: None,
            toggle: MainToggle::new(car_state::button_event::Type::MainCruise as u16),
            vision: None,
            vision_error_at: 0,
            vision_sources: [false; 4],
            vision_sources_published: None,
            vision_sources_published_ns: 0,
            settings_updated: None,
            metric,
            experimental,
            remaining: 0.,
            diagnostics: Diagnostics::default(),
            native_voice,
        })
    }
}
