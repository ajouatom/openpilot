pub mod config;
mod diagnostics;
pub mod http;
mod http_body;
mod parameters;
mod ports;
mod publisher;
mod stop;
use crate::{
    position::{CarPark, CarrotFix, GpsFix, ParkedPosition, Sample},
    route::{RouteEngine, Tick},
    Error,
};
use num_traits::ToPrimitive;
use openpilot_beepd::Clock;
use openpilot_cereal::log_capnp::event;
use openpilot_logging::{producer::Factory, record::Level, Value};
use openpilot_messaging::{
    runtime::SubMaster,
    state::{Options as SubscriptionOptions, State},
};
use openpilot_params::Params;
use ports::NativePorts;
use publisher::Publisher;
use std::{num::NonZeroU64, path::PathBuf, time::Duration};
use stop::Stop;

#[derive(Debug, Default)]
pub struct Options {
    pub frames: Option<NonZeroU64>,
    pub mapbox_host: Option<String>,
    pub persist_root: Option<PathBuf>,
}

pub fn set_destination(argument: Option<&str>) -> Result<(), Error> {
    let params = Params::for_runtime()?;
    let destination = crate::destination::Destination::from_argument(argument)?;
    if destination.default_waypoint {
        println!("Setting to Taco Bell");
    }
    let value = destination.parameter()?;
    parameters::write_status(params.put("NavDestination", value.as_bytes()))?;
    // set_destination.py: a destination set here replaces Carrot Web's
    // session destination.
    let memory = Params::for_runtime_at(std::path::Path::new("/dev/shm/params"))?;
    parameters::write_status(memory.remove("NavDestinationTemporary"))?;
    if destination.default_waypoint {
        parameters::write_status(params.put(
            "NavDestinationWaypoints",
            b"[[-117.16020713111648, 32.71997612490662]]",
        ))?;
        println!("{}", value.replace('"', "'"));
        println!("[(-117.16020713111648, 32.71997612490662)]");
    } else {
        parameters::write_status(params.remove("NavDestinationWaypoints"))?;
    }
    Ok(())
}

pub fn timestamp() -> Result<u64, Error> {
    let time = rustix::time::clock_gettime(rustix::time::ClockId::Monotonic);
    (openpilot_beepd::monotonic_seconds(time)? * 1e9)
        .to_u64()
        .ok_or(Error::Runtime(
            "monotonic clock outside unsigned 64-bit range",
        ))
}

/// What `ParkedPosition` and the external-navigation check read, as navd.py's
/// SubMaster gives them; a service not heard yet is absent.
fn sample(state: &State, now: f64) -> Result<(Sample, bool), Error> {
    let mut sample = Sample {
        now,
        ..Sample::default()
    };
    let mut external = false;
    let carrot = state.topic("carrotMan")?;
    if carrot.seen {
        if let event::CarrotMan(value) = carrot.event()?.which().map_err(capnp::Error::from)? {
            let value = value?;
            external = crate::external::external_navigation(
                carrot.valid && carrot.alive,
                value.get_active_carrot(),
                !value.get_navi_owner()?.is_empty(),
            );
            sample.carrot = Some(CarrotFix {
                valid: carrot.valid,
                received: carrot.receive_time,
                position_source_empty: value.get_position_source()?.is_empty(),
                heading_valid: value.get_position_heading_valid(),
                fix_mono_ns: value.get_position_fix_mono_time(),
                position_mono_ns: value.get_position_mono_time(),
                latitude: f64::from(value.get_x_pos_lat()),
                longitude: f64::from(value.get_x_pos_lon()),
                angle: f64::from(value.get_x_pos_angle()),
                speed: f64::from(value.get_x_pos_speed()),
            });
        }
    }
    for name in ["gpsLocation", "gpsLocationExternal"] {
        let topic = state.topic(name)?;
        if !topic.seen {
            continue;
        }
        let gps = match topic.event()?.which().map_err(capnp::Error::from)? {
            event::GpsLocation(value) | event::GpsLocationExternal(value) => value?,
            _ => continue,
        };
        sample.gps.push(GpsFix {
            valid: topic.valid,
            has_fix: gps.get_has_fix(),
            received: topic.receive_time,
            log_mono_time: topic.log_mono_time,
            latitude: gps.get_latitude(),
            longitude: gps.get_longitude(),
            bearing: f64::from(gps.get_bearing_deg()),
            speed: f64::from(gps.get_speed()),
        });
    }
    let car = state.topic("carState")?;
    if car.seen {
        if let event::CarState(value) = car.event()?.which().map_err(capnp::Error::from)? {
            let value = value?;
            sample.car = Some(CarPark {
                valid: car.valid,
                alive: car.alive,
                received: car.receive_time,
                park: value.get_gear_shifter()
                    == Ok(openpilot_cereal::car_capnp::car_state::GearShifter::Park),
                standstill: value.get_standstill(),
                v_ego: f64::from(value.get_v_ego()),
            });
        }
    }
    Ok((sample, external))
}

/// navd.py `refresh_token`: `MAPBOX_TOKEN`, else `MapboxPublicKey`.
fn token(
    params: &Params,
    logger: &mut openpilot_logging::producer::Logger,
) -> Result<String, Error> {
    if let Ok(token) = std::env::var("MAPBOX_TOKEN") {
        if !token.is_empty() {
            return Ok(token);
        }
    }
    Ok(openpilot_params_typed::get_string(params, "MapboxPublicKey", logger)?.unwrap_or_default())
}

fn ui_pid(state: &State) -> Result<Option<i32>, Error> {
    let event::ManagerState(manager) = state
        .topic("managerState")?
        .event()?
        .which()
        .map_err(capnp::Error::from)?
    else {
        return Err(Error::Runtime(
            "managerState service has a different event type",
        ));
    };
    for process in manager?.get_processes()? {
        if process.get_running()
            && process
                .get_name()?
                .to_str()
                .map_err(|_| Error::Runtime("manager name is not UTF-8"))?
                == "ui"
        {
            return Ok(Some(process.get_pid()));
        }
    }
    Ok(None)
}

pub fn run(options: Options) -> Result<(), Error> {
    let stop = Stop::new()?;
    let factory = Factory::for_runtime()?;
    factory.bind_global(
        [("daemon".into(), Value::Text("navd".into()))]
            .into_iter()
            .collect(),
    )?;
    let publisher = Publisher::new()?;
    let mut subscriber = SubMaster::for_runtime(
        &[
            "managerState",
            "carrotMan",
            "gpsLocation",
            "gpsLocationExternal",
            "carState",
        ],
        SubscriptionOptions::default(),
    )?;
    let mut ports = NativePorts {
        params: Params::for_runtime()?,
        memory: Params::for_runtime_at(std::path::Path::new("/dev/shm/params"))?,
        logger: factory.logger(),
        publisher,
        stop: &stop,
        pending: None,
    };
    let mut parked = ParkedPosition::default();
    let mut engine = RouteEngine::new_configured(&mut ports, |ports| {
        config::load(&ports.params, &mut ports.logger, &options)
    })?;
    let clock = openpilot_beepd::SystemClock;
    let mut next = None;
    let mut frame = 0_u64;
    let result = (|| loop {
        stop.check()?;
        subscriber.update(Duration::ZERO)?;
        let now = clock.monotonic()?;
        if subscriber.state.topic("managerState")?.updated
            && engine.update_ui_pid(ui_pid(&subscriber.state)?, now)
        {
            diagnostics::text(
                &mut ports.logger,
                Level::Warning,
                "UI restarting, sending route".into(),
            );
            println!("########## UI restarting, sending route");
        }
        let (sample, external) = sample(&subscriber.state, now)?;
        let tick = Tick {
            now,
            // navd polls at 1 Hz: a 1.5 s gap still counts as continuous.
            position: parked.update(&sample, 1.5),
            external,
            token: token(&ports.params, &mut ports.logger)?,
        };
        engine.update(tick, &mut ports)?;
        let deadline = match next {
            Some(value) => value,
            None => clock.monotonic()? + 1.,
        };
        let remaining = deadline - clock.monotonic()?;
        next = Some(deadline + 1.);
        if remaining < 0. {
            println!(
                "openpilot.selfdrive.navd.navd lagging by {:.2} ms",
                -remaining * 1000.
            );
        }
        if remaining > 0. {
            stop.wait(openpilot_beepd::sleep_duration(remaining)?)?;
        }
        frame = frame.saturating_add(1);
        if options.frames.is_some_and(|limit| frame >= limit.get()) {
            return Ok(());
        }
    })();
    match result {
        Err(Error::Interrupted) => {
            diagnostics::text(
                &mut ports.logger,
                Level::Warning,
                "child openpilot.selfdrive.navd.navd got SIGINT".into(),
            );
            Ok(())
        }
        result => result,
    }
}
