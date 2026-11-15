use crate::route::Diagnostic;
use openpilot_logging::{
    log_site,
    producer::Logger,
    record::{Level, Record},
    Fields, Value,
};

pub fn text(logger: &mut Logger, level: Level, message: String) {
    if let Err(error) = logger.emit(log_site!(), Record::text(level, message)) {
        eprintln!("navd logging: {error}");
    }
}

pub fn exception(logger: &mut Logger, message: &str, details: &str) {
    if let Err(error) = logger.emit(
        log_site!(),
        Record::text(Level::Error, message.to_owned()).with_exception(details.to_owned()),
    ) {
        eprintln!("navd logging: {error}");
    }
}

pub fn api_failure(logger: &mut Logger, status: u16, body: &str) {
    let fields: Fields = [
        ("status_code".into(), Value::Integer(i128::from(status))),
        ("text".into(), Value::Text(body.into())),
        ("error".into(), Value::Bool(true)),
    ]
    .into_iter()
    .collect();
    match Record::event("API request failed", Vec::new(), fields)
        .and_then(|record| logger.emit(log_site!(), record))
    {
        Ok(_) => {}
        Err(error) => eprintln!("navd logging: {error}"),
    }
}

pub fn event(logger: &mut Logger, event: Diagnostic) {
    match event {
        Diagnostic::NewDestination {
            new,
            previous,
            place,
        } => {
            text(
                logger,
                Level::Warning,
                format!("Got new destination from NavDestination param {new}"),
            );
            let previous = previous.map_or_else(|| "None".into(), |value| value.to_string());
            let place = place.map_or_else(
                || "None".into(),
                |value| {
                    value
                        .as_str()
                        .map(str::to_owned)
                        .unwrap_or_else(|| value.to_string())
                },
            );
            println!("Got new destination from NavDestination param {new} {previous} {place}");
        }
        Diagnostic::Calculating { from, to } => {
            text(
                logger,
                Level::Warning,
                format!("Calculating route {from} -> {to}"),
            );
            println!("############## Calculating route {from} -> {to}");
        }
        Diagnostic::EmptyRoute => {
            text(logger, Level::Warning, "Got empty route response".into());
            println!("Got empty route response");
        }
        Diagnostic::RequestFailed(error) => {
            exception(logger, "failed to get route", &error.to_string());
            println!("failed to get route");
        }
        Diagnostic::ComputeFailed(error) => {
            exception(logger, "navd.failed_to_compute", &error.to_string())
        }
        Diagnostic::RouteLimited { original, sent } => {
            text(
                logger,
                Level::Warning,
                format!("navd route limited from {original} to {sent} points"),
            );
            println!("navd route limited: {original} -> {sent}");
        }
        Diagnostic::InvalidRoute(error) => {
            text(
                logger,
                Level::Warning,
                format!("navd: invalid Mapbox route response: {error}"),
            );
        }
        Diagnostic::DestinationReached => {
            text(logger, Level::Warning, "Destination reached".into());
            println!("Destination reached");
        }
        Diagnostic::SpeedLimit(speed) => {
            let mut value = String::new();
            match openpilot_runtime_core::python_float::write_float(speed, &mut value) {
                Ok(()) => println!("{{'maxspeed': {value}}}"),
                Err(error) => eprintln!("navd speed-limit output: {error}"),
            }
        }
    }
}
