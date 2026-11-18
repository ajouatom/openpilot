use openpilot_logging::{native::Logger, rate::RateLimit, record::Level, site::Site};
use std::sync::{Mutex, OnceLock};

static LOGGER: OnceLock<Logger> = OnceLock::new();
pub static SEGMENT_QUEUE: Mutex<RateLimit> = Mutex::new(RateLimit::new(2, 100));

pub fn initialize(device: &str) {
    if let Ok(logger) = Logger::for_runtime(env!("LOGGERD_VERSION"), device) {
        let _ = LOGGER.set(logger);
    }
}

pub fn emit(site: Site, level: Level, message: String) {
    if let Some(logger) = LOGGER.get() {
        let _ = logger.emit(site, level, message);
    }
}

pub fn limited(limit: &Mutex<RateLimit>, site: Site, message: impl FnOnce() -> String) {
    let Ok(now) = crate::clock::now() else { return };
    let Ok(mut limit) = limit.lock() else { return };
    let Ok(decision) = limit.admit(now) else {
        return;
    };
    drop(limit);
    if decision.suppressed > 0 {
        emit(
            site,
            Level::Warning,
            format!("cloudlog: {} messages suppressed", decision.suppressed),
        );
    }
    if decision.emit {
        emit(site, Level::Error, message());
    }
}

pub fn close() {
    if let Some(logger) = LOGGER.get() {
        let _ = logger.close();
    }
}

pub fn errno_text(errno: i32) -> String {
    let text = std::io::Error::from_raw_os_error(errno).to_string();
    text.strip_suffix(&format!(" (os error {errno})"))
        .unwrap_or(&text)
        .to_owned()
}
