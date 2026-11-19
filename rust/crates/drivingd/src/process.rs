use openpilot_driving_modeld::{camera::Source, runtime::DrivingRuntime, Error};
use openpilot_logging::{
    log_site,
    producer::Logger,
    record::{Level, Record},
};
use openpilot_model_runtime::catalog::{Catalog, Kind};
use openpilot_modeld::camera::{self, CameraStream};
use openpilot_msgq::{VisionClient, VisionLayout, VisionStream};
use openpilot_params::Params;
use std::{
    sync::{
        atomic::{AtomicBool, Ordering},
        Arc,
    },
    thread,
    time::{Duration, Instant},
};

pub fn configure() -> Result<(), Error> {
    if std::path::Path::new("/TICI").is_file() {
        // SAFETY: sched_param contains integers/timespecs; zero initializes musl's additional fields.
        let mut settings: libc::sched_param = unsafe { std::mem::zeroed() };
        settings.sched_priority = 54;
        // SAFETY: sched_setscheduler borrows the initialized parameter only during this call.
        if unsafe { libc::sched_setscheduler(0, libc::SCHED_FIFO, &settings) } != 0 {
            return Err(std::io::Error::last_os_error().into());
        }
        let mut cores = rustix::thread::CpuSet::new();
        cores.set(7);
        rustix::thread::sched_setaffinity(None, &cores).map_err(std::io::Error::from)?;
    }
    Ok(())
}

pub use openpilot_driving_modeld::clock::{monotonic, timestamp};

pub fn cameras(
    stop: &Arc<AtomicBool>,
    use_wide: bool,
    logger: &mut Logger,
) -> Result<Option<(Source, Option<Source>, bool)>, Error> {
    let selected = loop {
        if stop.load(Ordering::Relaxed) {
            return Ok(None);
        }
        let available = VisionClient::available_streams("camerad")?;
        if let Some(selected) = camera::select_streams(
            available.contains(&VisionStream::Road),
            available.contains(&VisionStream::WideRoad),
            use_wide,
        ) {
            break selected;
        }
        thread::sleep(Duration::from_millis(100));
    };
    let main_wide = selected.0 == CameraStream::WideRoad;
    let mut main = Source::new(
        VisionClient::new(
            "camerad",
            if main_wide {
                VisionStream::WideRoad
            } else {
                VisionStream::Road
            },
            true,
        )?,
        Arc::clone(stop),
    );
    let mut extra = if selected.1 {
        Some(Source::new(
            VisionClient::new("camerad", VisionStream::WideRoad, false)?,
            Arc::clone(stop),
        ))
    } else {
        None
    };
    let [wide, main_is_wide, has_extra] =
        [use_wide, main_wide, selected.1].map(|value| if value { "True" } else { "False" });
    logger.emit(log_site!(), Record::text(Level::Warning, format!(
        "vision stream set up, use_wide_camera: {wide}, main_wide_camera: {main_is_wide}, use_extra_client: {has_extra}")))?;
    for source in std::iter::once(&mut main).chain(extra.iter_mut()) {
        while !stop.load(Ordering::Relaxed) && !source.client.connect()? {
            thread::sleep(Duration::from_millis(100));
        }
    }
    if stop.load(Ordering::Relaxed) {
        return Ok(None);
    }
    log_camera(logger, "main", &main.client)?;
    if let Some(extra) = &extra {
        log_camera(logger, "extra", &extra.client)?;
    }
    Ok(Some((main, extra, main_wide)))
}

fn log_camera(logger: &mut Logger, name: &str, client: &VisionClient) -> Result<(), Error> {
    let layout = client
        .layout()
        .ok_or(Error::Contract("connected camera has no layout"))?;
    logger.emit(
        log_site!(),
        Record::text(
            Level::Warning,
            format!(
                "connected {name} cam with buffer size: {} ({} x {})",
                layout.len, layout.width, layout.height
            ),
        ),
    )?;
    Ok(())
}

pub fn car_params(params: &Params, stop: &Arc<AtomicBool>) -> Result<Option<Vec<u8>>, Error> {
    loop {
        if stop.load(Ordering::Relaxed) {
            return Ok(None);
        }
        if let Some(bytes) = params.get("CarParams")?.filter(|bytes| !bytes.is_empty()) {
            return Ok(Some(bytes));
        }
        thread::sleep(Duration::from_millis(100));
    }
}

pub fn model<'a>(
    catalog: &'a Catalog,
    layout: &VisionLayout,
    logger: &mut Logger,
) -> Result<DrivingRuntime<'a>, Error> {
    let camera = [
        u32::try_from(layout.width).map_err(|_| Error::Contract("camera width overflow"))?,
        u32::try_from(layout.height).map_err(|_| Error::Contract("camera height overflow"))?,
    ];
    let started = Instant::now();
    logger.emit(
        log_site!(),
        Record::text(Level::Warning, "loading model".into()),
    )?;
    let bundle = catalog.select(Kind::Driving, camera)?;
    let priority = if bundle.backend == "qcom-cl" {
        std::env::var("QCOM_PRIORITY")
            .map_or(Ok(8), |value| value.parse::<u8>())
            .map_err(|_| Error::Contract("invalid QCOM_PRIORITY"))?
    } else {
        8
    };
    // SAFETY: the daemon's --trusted-catalog contract requires immutable trusted executable artifacts.
    let runtime = unsafe { DrivingRuntime::load(bundle, priority) }?;
    logger.emit(
        log_site!(),
        Record::text(
            Level::Warning,
            format!(
                "{} loaded in {:.1}s, modeld starting",
                openpilot_driving_modeld::runtime::model_name(bundle),
                started.elapsed().as_secs_f64()
            ),
        ),
    )?;
    Ok(runtime)
}
