use crate::{
    clock, diagnostics,
    encoder::{Encoder, Stream},
    metadata::Environment,
    rotation::Rotation,
    writer::{route_name, Logger},
    Error,
};
use openpilot_cereal::log_capnp::event;
use openpilot_logging::{log_site, record::Level};
use openpilot_messaging::services::{Service, SERVICES};
use openpilot_msgq::{MultiSubscriber, Subscription};
use openpilot_params::Params;
use std::{
    sync::{
        atomic::{AtomicUsize, Ordering},
        Arc,
    },
    time::Duration,
};

struct ServiceState {
    service: &'static Service,
    counter: u64,
    encoder: Option<Encoder>,
}

pub fn run(environment: Environment) -> Result<(), Error> {
    diagnostics::initialize(environment.device_name()?);
    let signal = Arc::new(AtomicUsize::new(0));
    for value in [
        signal_hook::consts::SIGINT,
        signal_hook::consts::SIGTERM,
        rustix::process::Signal::POWER.as_raw(),
    ] {
        signal_hook::flag::register_usize(value, Arc::clone(&signal), usize::try_from(value)?)?;
    }
    if environment.device != openpilot_cereal::log_capnp::init_data::DeviceType::Pc {
        let mut cores = rustix::thread::CpuSet::new();
        for index in 0..4 {
            cores.set(index);
        }
        rustix::thread::sched_setaffinity(None, &cores).map_err(std::io::Error::from)?;
    }
    let params = Params::for_runtime()?;
    let record_audio = crate::encoder::record_audio(&params)?;
    let mut states = Vec::new();
    for service in SERVICES {
        let stream = Stream::from_service(service.name);
        if service.should_log
            || stream.is_some()
            || (service.name == "rawAudioData" && record_audio)
        {
            states.push(ServiceState {
                service,
                counter: 0,
                encoder: stream
                    .map(|stream| Encoder::new(stream, &params))
                    .transpose()?,
            });
        }
    }
    states.sort_unstable_by_key(|state| state.service.name);
    for state in &states {
        diagnostics::emit(
            log_site!(),
            Level::Debug,
            format!("logging {}", state.service.name),
        );
    }
    let specifications: Vec<_> = states
        .iter()
        .map(|state| Subscription {
            endpoint: state.service.name,
            capacity: state.service.queue_size,
            polled: true,
        })
        .collect();
    let mut subscribers = MultiSubscriber::queued_for_runtime(&specifications)?;
    let route = route_name(&params)?;
    let mut logger = Logger::new(&environment.log_root, route, environment.init_data()?);
    let mut rotation = Rotation {
        test_mode: std::env::var_os("LOGGERD_TEST").is_some(),
        ..Rotation::default()
    };
    rotate(&mut logger, &mut rotation)?;
    params.put("CurrentRoute", logger.route.as_bytes())?;
    let mut message_count = 0_u64;
    let mut byte_count = 0_u64;
    let mut receive_buffer = Vec::new();
    let started = clock::now()?;
    while signal.load(Ordering::Relaxed) == 0 {
        for index in subscribers.poll_ready(Duration::from_millis(1000))? {
            if signal.load(Ordering::Relaxed) != 0 {
                break;
            }
            let name = states
                .get(index)
                .ok_or(Error::Invalid("native service index"))?
                .service
                .name;
            if matches!(name, "userBookmark" | "audioFeedback") {
                logger.preserve(&params)?;
            }
            for count in 0..200 {
                if signal.load(Ordering::Relaxed) != 0 {
                    break;
                }
                // Encoder packets can be retained across rotation; give those
                // their own allocation. Ordinary log packets are consumed here.
                let mut encoder_bytes = Vec::new();
                let bytes = if states[index].encoder.is_some() {
                    &mut encoder_bytes
                } else {
                    &mut receive_buffer
                };
                if !subscribers.receive_one_into(index, bytes)? {
                    break;
                }
                if name == "rawAudioData" && record_audio {
                    let message = capnp::serialize::read_message_from_flat_slice(
                        &mut bytes.as_slice(),
                        Default::default(),
                    )?;
                    let event = message.get_root::<event::Reader>()?;
                    let event::RawAudioData(data) = event.which()? else {
                        return Err(Error::Invalid("audio event does not match service"));
                    };
                    let data = data?;
                    for state in &mut states {
                        if let Some(encoder) = &mut state.encoder {
                            if encoder.include_audio {
                                if let Some(writer) = &mut encoder.writer {
                                    writer.write_audio(
                                        data.get_data()?,
                                        event.get_log_mono_time() / 1000,
                                        data.get_sample_rate(),
                                    )?;
                                    encoder.audio_initialized = true;
                                }
                            }
                        }
                    }
                }
                let state = states
                    .get_mut(index)
                    .ok_or(Error::Invalid("native service index"))?;
                match &mut state.encoder {
                    Some(encoder) => {
                        rotation.last_camera_ns = clock::now()?;
                        let outcome =
                            encoder.handle(&mut logger, std::mem::take(bytes), rotation.ready)?;
                        byte_count = byte_count.wrapping_add(u64::try_from(outcome.bytes)?);
                        if outcome.ready_part.is_some() {
                            rotation.ready += 1;
                        }
                    }
                    None => {
                        let qlog = state.service.decimation.is_some_and(|frequency| {
                            state.counter.is_multiple_of(u64::from(frequency))
                        });
                        state.counter = state.counter.wrapping_add(1);
                        logger.write(bytes, qlog)?;
                        byte_count = byte_count.wrapping_add(u64::try_from(bytes.len())?);
                    }
                }
                let now = clock::now()?;
                if let Some(reason) = rotation.timeout_reason(now) {
                    diagnostics::emit(log_site!(), Level::Error, reason.into());
                }
                if rotation.due(now) {
                    rotate(&mut logger, &mut rotation)?;
                }
                message_count = message_count.wrapping_add(1);
                if message_count.is_multiple_of(10000) {
                    let seconds = (clock::now()? - started) as f64 / 1e9;
                    diagnostics::emit(
                        log_site!(),
                        Level::Debug,
                        format!(
                            "{message_count} messages, {:.2} msg/sec, {:.2} KB/sec",
                            message_count as f64 / seconds,
                            byte_count as f64 * 0.001 / seconds
                        ),
                    );
                }
                if count == 199 {
                    diagnostics::emit(
                        log_site!(),
                        Level::Debug,
                        format!("large volume of '{name}' messages"),
                    );
                }
            }
        }
    }
    let signal = i32::try_from(signal.load(Ordering::Relaxed))?;
    diagnostics::emit(log_site!(), Level::Warning, "closing logger".into());
    if signal == rustix::process::Signal::POWER.as_raw() {
        diagnostics::emit(log_site!(), Level::Error, "power failure".into());
        rustix::fs::sync();
        diagnostics::emit(log_site!(), Level::Error, "sync done".into());
    }
    logger.close(signal, clock::now()?)?;
    for state in &mut states {
        if let Some(encoder) = &mut state.encoder {
            encoder.close()?;
        }
    }
    Ok(())
}

fn rotate(logger: &mut Logger, rotation: &mut Rotation) -> Result<(), Error> {
    rotate_with_clock(logger, rotation, clock::now)
}

fn rotate_with_clock(
    logger: &mut Logger,
    rotation: &mut Rotation,
    mut now: impl FnMut() -> Result<u64, Error>,
) -> Result<(), Error> {
    logger.next(now()?)?;
    rotation.ready = 0;
    rotation.last_rotation_ns = now()?;
    diagnostics::emit(
        log_site!(),
        Level::Warning,
        format!(
            "{} {}",
            if logger.part == 0 {
                "logging to"
            } else {
                "rotated to"
            },
            logger.path()?.display()
        ),
    );
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn starts_rotation_timeout_after_opening_and_writing_the_new_segment() {
        let root = tempfile::tempdir().unwrap();
        let path = root.path().join("route--0/rlog.zst");
        let mut logger = Logger::new(root.path(), "route".into(), Vec::new());
        let mut rotation = Rotation::default();
        let mut calls = 0;
        rotate_with_clock(&mut logger, &mut rotation, || {
            calls += 1;
            assert_eq!(path.exists(), calls == 2);
            Ok(if calls == 1 {
                1_000_000_000
            } else {
                9_000_000_000
            })
        })
        .unwrap();
        assert_eq!(calls, 2);
        assert!(!rotation.due(61_000_000_001));
        assert!(rotation.due(69_000_000_001));
        logger.close(0, 70_000_000_000).unwrap();
    }
}
