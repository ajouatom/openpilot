use crate::{process, Options};
use openpilot_cereal::car_capnp::car_params;
use openpilot_desire::{command::CommandReader, helper::DesireHelper};
use openpilot_driving_modeld::{
    bus::{self, Inputs},
    diagnostics::{thread_cpu, FrameTiming},
    indicator, jetlink, parameters,
    publication::{Output, Publication, Sources},
    state::State,
    usb_selection, Error,
};
use openpilot_logging::{
    log_site,
    producer::Logger,
    record::{Level, Record},
    runtime::RuntimeDiagnostics,
};
use openpilot_messaging::{
    runtime::{PubMaster, SubMaster},
    state::Options as SubscriberOptions,
};
use openpilot_model_runtime::catalog::Catalog;
use openpilot_modeld::{
    calibration::DrivingCalibration, camera::receive_pair_traced, inputs::DropTracker,
    model_wire::ModelTiming,
};
use std::{
    env,
    io::Cursor,
    path::Path,
    sync::{
        atomic::{AtomicBool, Ordering},
        Arc,
    },
    time::{Duration, Instant},
};

pub struct Configuration {
    pub options: Options,
    pub usb: usb_selection::Configuration,
}

pub fn run(
    config: Configuration,
    logger: &mut Logger,
    stop: &Arc<AtomicBool>,
) -> Result<(), Error> {
    let Configuration {
        options,
        usb: usb_config,
    } = config;
    let params = parameters::open()?;
    let catalog = Catalog::load(&options.catalog)?;
    let Some((mut main, mut extra, main_wide)) =
        process::cameras(stop, parameters::use_wide(&params)?, logger)?
    else {
        return Ok(());
    };
    let layout = main
        .client
        .layout()
        .ok_or(Error::Contract("connected camera has no layout"))?;
    // InternalDrivingModel selects a model from internal_model::MODELS; each
    // non-CD210 model is a catalog next to CD210's. Any failure to load it
    // keeps driving on CD210, and the screen badge and Carrot Web show it
    // through InternalDrivingModelStatus.
    let wanted = openpilot_driving_modeld::internal_model::selected(
        parameters::integer(&params, "InternalDrivingModel").unwrap_or(0),
    );
    let mut selection_error = None;
    let alternative =
        if wanted.directory == openpilot_driving_modeld::internal_model::CD210.directory {
            None
        } else {
            let path = options.catalog.with_file_name(wanted.directory);
            Catalog::load(&path)
                .map_err(|error| selection_error = Some(format!("model files: {error}")))
                .ok()
        };
    let mut runtime = match alternative
        .as_ref()
        .map(|alternative| process::model(alternative, &layout, logger))
    {
        Some(Ok(runtime)) => runtime,
        Some(Err(error)) => {
            selection_error = Some(format!("load: {error}"));
            process::model(&catalog, &layout, logger)?
        }
        None => process::model(&catalog, &layout, logger)?,
    };
    if let Some(error) = &selection_error {
        logger.emit(
            log_site!(),
            Record::text(
                Level::Error,
                format!("{} unavailable, using CD210: {error}", wanted.name),
            ),
        )?;
    }
    params.put(
        "InternalDrivingModelStatus",
        serde_json::json!({
            "selected": wanted.id,
            "running": runtime.model_id(),
            "error": selection_error,
        })
        .to_string()
        .as_bytes(),
    )?;
    let camera = [
        u32::try_from(layout.width).map_err(|_| Error::Contract("camera width"))?,
        u32::try_from(layout.height).map_err(|_| Error::Contract("camera height"))?,
    ];
    let usb_discovery = usb_selection::monitor(usb_config, params.clone(), camera)
        .map_err(|error| eprintln!("eGPU discovery unavailable: {error}"))
        .ok();
    let mut recovery = None;
    let mut usb_first_publication = false;
    let mut subscribers = SubMaster::for_runtime(bus::TOPICS, SubscriberOptions::default())?;
    let mut publishers = PubMaster::for_runtime(bus::OUTPUTS)?;
    let Some(car_params) = process::car_params(&params, stop)? else {
        return Ok(());
    };
    let car_params = capnp::serialize::read_message(
        Cursor::new(car_params),
        capnp::message::ReaderOptions::new(),
    )?;
    let longitudinal_delay = f64::from(
        car_params
            .get_root::<car_params::Reader>()?
            .get_longitudinal_actuator_delay(),
    );
    logger.emit(
        log_site!(),
        Record::text(
            Level::Info,
            format!(
                "modeld got CarParams: {}",
                car_params
                    .get_root::<car_params::Reader>()?
                    .get_brand()?
                    .to_str()?
            ),
        ),
    )?;
    let mut state = State::new(
        longitudinal_delay,
        parameters::float(&params, "VEgoStopping")? * 0.01,
        parameters::float(&params, "CameraYawTrimDeg")? * 0.01,
    );
    let mut calibration = DrivingCalibration::default();
    let mut drops = DropTracker::default();
    let fingerprint = car_params
        .get_root::<car_params::Reader>()?
        .get_car_fingerprint()?
        .to_str()?
        .to_owned();
    let support = indicator::Support::read(car_params.get_root::<car_params::Reader>()?)?;
    let supported = support.any();
    let (tesla_signals, native_voice) = {
        let cp = car_params.get_root::<car_params::Reader>()?;
        let brand = cp.get_brand()?.to_str()?;
        let flags = cp.get_flags();
        // carrot voice_indicator.native_indicator: EA_02 or Hyundai SPAS2.
        const HYUNDAI_NATIVE: u32 = (1 << 13) | 1 | (1 << 5); // CANFD | CANFD_HDA2 | ENABLE_BLINKERS
                                                              // carrot 2accb1fa3: a Kick car has no native voice route.
        let native = !cp.get_passive()
            && !support.kick
            && ((brand == "volkswagen" && flags & 16384 != 0)
                || (brand == "hyundai" && flags & HYUNDAI_NATIVE == HYUNDAI_NATIVE));
        (brand == "tesla" && flags & 32 != 0, native) // TeslaFlags.AUTO_TURN_SIGNALS
    };
    let mut publication = Publication::new(
        DesireHelper::new(
            supported,
            if supported {
                indicator::startup_journal()
            } else {
                None
            },
        )
        .with_kick(support.kick)
        .with_tesla(tesla_signals, (process::monotonic() * 1e9) as u64)
        .with_assistant(&fingerprint, native_voice),
    );
    let mut commands = CommandReader::new(
        Path::new("/dev/shm/carrot-bluetooth"),
        "lane",
        process::monotonic(),
    );
    let raw = env::var_os("SEND_RAW_PRED").is_some_and(|value| !value.is_empty());
    let simulation = env::var("SIMULATION")
        .unwrap_or_else(|_| "0".to_owned())
        .trim()
        .parse::<i64>()
        .map_err(|_| Error::Contract("invalid SIMULATION"))?
        != 0;
    let mut navigation_input = openpilot_driving_modeld::navigation::NavigationInput::default();
    let mut external =
        openpilot_jetlink::runtime::Runtime::new(Path::new("/dev/shm/carrot-jetlink.sock"))?;
    let mut external_stored = None;
    let mut external_display = jetlink::ModelStatusWriter::new(std::path::Path::new(
        openpilot_jetlink::status::MODEL_STATUS,
    ));
    let mut frame_count = 0u64;
    // Set while frames are answered by the peer model only.
    let mut internal_stale = false;
    let mut camera_gap = openpilot_driving_modeld::camera_gap::CameraGapTrace::default();
    let mut diagnostics = RuntimeDiagnostics::new("modeld", 1.0);
    while !stop.load(Ordering::Relaxed) {
        if let Some(candidate) = usb_discovery.as_ref().and_then(|rx| rx.try_recv().ok()) {
            recovery = Some(candidate);
        }
        let loop_start = process::monotonic();
        let cpu_start = thread_cpu();
        if state.begin_iteration() {
            state.refresh(parameters::settings(&params)?);
        }
        camera_gap.received.clear();
        let Some(pair) =
            receive_pair_traced(&mut main, extra.as_mut(), Some(&mut camera_gap.received))?
        else {
            logger.emit(
                log_site!(),
                Record::text(
                    Level::Debug,
                    "camera pair unavailable or out of sync".into(),
                ),
            )?;
            continue;
        };
        let camera_ready = process::monotonic();
        let frame = pair.main();
        let extra_frame = pair.extra();
        runtime.validate_frame(&frame.buffer.layout)?;
        runtime.validate_frame(&extra_frame.buffer.layout)?;
        subscribers.update(Duration::ZERO)?;
        let sources = Inputs::read(&subscribers.state)?;
        sources.update_calibration(
            &subscribers.state,
            &mut calibration,
            state.settings.camera_yaw_trim,
            main_wide,
            extra.is_some(),
        )?;
        let dropped = drops.observe(frame.metadata.frame_id);
        if dropped.prepare_only {
            logger.emit(
                log_site!(),
                Record::text(
                    Level::Error,
                    format!(
                        "camera dropped {} frames; advancing model history",
                        dropped.dropped
                    ),
                ),
            )?;
        }
        let (lateral, longitudinal) = state.action_times();
        let model_controls = openpilot_driving_modeld::usb_model::Controls {
            desire: i32::from(publication.desire.desire),
            is_rhd: sources.monitoring.get_is_r_h_d(),
            lateral_time: lateral,
            longitudinal_time: longitudinal,
        };
        runtime.update_inputs(model_controls);
        let start = Instant::now();
        let inference_started = process::monotonic();
        let camera_age_at_run_ms =
            (process::monotonic() - frame.metadata.timestamp_eof as f64 * 1e-9) * 1000.0;
        let inference_cpu_start = thread_cpu();
        let join_now = process::monotonic();
        let mut join_valid = [false; 3];
        let mut join_received = [0.; 3];
        for (index, name) in ["carState", "selfdriveState", "carControl"]
            .into_iter()
            .enumerate()
        {
            let topic = subscribers.state.topic(name)?;
            join_valid[index] = topic.valid && topic.alive;
            join_received[index] = topic.receive_time;
        }
        let controls = openpilot_jetlink::transition::ControlState {
            join_allowed: openpilot_driving_modeld::recovery::may_join(
                join_now,
                join_valid,
                join_received,
                sources.car.get_standstill(),
                f64::from(sources.car.get_v_ego()),
                sources.car.get_steering_pressed(),
            ),
        };
        let mut desire = [0.0; 8];
        let desire_index = usize::from(u16::from(publication.desire.desire));
        if let Some(value) = desire.get_mut(desire_index) {
            *value = 1.0;
        }
        let traffic = if sources.monitoring.get_is_r_h_d() {
            [0.0, 1.0]
        } else {
            [1.0, 0.0]
        };
        let usb_navigation = runtime.usb_navigation_model();
        let navigation = navigation_input
            .update(
                usb_navigation.is_some() || external.navigation_model(),
                f64::from(sources.car.get_v_ego()),
            )
            .unwrap_or([0.; openpilot_modeld::navigation::NAVIGATION_LEN]);
        if let Some(model) = usb_navigation {
            model.set_navigation(&navigation)?;
        }
        external.begin(
            openpilot_jetlink::runtime::FrameInput {
                controls,
                frame: frame.metadata.frame_id,
                source_sof: frame.metadata.timestamp_sof,
                prepare_only: dropped.prepare_only,
                camera_ready: calibration.seen() && !runtime.uses_usb(),
                desire,
                traffic,
                action: [lateral as f32, longitudinal as f32],
                navigation,
            },
            || {
                runtime
                    .warp_for_jetlink(
                        &frame.buffer,
                        &extra_frame.buffer,
                        [calibration.main(), calibration.extra()],
                    )
                    .map_err(|error| openpilot_jetlink::Error::Native(error.to_string()))
            },
        );
        jetlink::persist(&params, &mut external_stored, &external.status)?;
        // carrot's JoiningModel.run: a frame the peer model answers is not also
        // run on the internal model. The internal model's history goes stale
        // meanwhile and is cleared before its next run.
        let external_frame = external.awaiting_frame() && !runtime.uses_usb();
        let prediction = if external_frame {
            internal_stale = true;
            Ok(None)
        } else {
            if internal_stale {
                runtime.reset_internal()?;
                runtime.update_inputs(model_controls);
                internal_stale = false;
            }
            runtime.infer(
                &frame.buffer,
                &extra_frame.buffer,
                [calibration.main(), calibration.extra()],
                dropped.prepare_only,
            )
        };
        let prediction = match prediction {
            Ok(prediction) => prediction,
            Err(error) if runtime.uses_usb() => {
                logger.emit(
                    log_site!(),
                    Record::text(
                        Level::Error,
                        format!("eGPU model failed, falling back to internal GPU: {error}"),
                    ),
                )?;
                if let Some(active_recovery) = &mut recovery {
                    if let Err(arm_error) =
                        active_recovery.failed(process::monotonic(), &error.to_string())
                    {
                        eprintln!("eGPU recovery could not be armed: {arm_error}");
                        recovery = None;
                    }
                }
                usb_selection::fail(&params)?;
                runtime.disable_usb();
                runtime.reset_internal_after_usb()?;
                drops.reset_warmup();
                // A failed USB wait can outlive this camera pair. The consumed
                // frame is already accounted for by DropTracker; receive a fresh
                // synchronized pair before native inference or publication.
                continue;
            }
            Err(error) => return Err(error),
        };
        let mut prediction = external.finish(prediction);
        if external_frame && prediction.is_none() {
            // "Jetlink failed; restoring local model": the peer's frame was
            // lost (the fault file already asks for disengagement), so this
            // frame is answered by the internal model from a cleared history.
            runtime.reset_internal()?;
            runtime.update_inputs(model_controls);
            internal_stale = false;
            prediction = runtime.infer(
                &frame.buffer,
                &extra_frame.buffer,
                [calibration.main(), calibration.extra()],
                dropped.prepare_only,
            )?;
            if prediction.is_some() {
                external.count_native_output();
            }
        }
        jetlink::persist(&params, &mut external_stored, &external.status)?;
        external_display.update_selected(
            &external.status,
            &external.detail,
            openpilot_jetlink::now_ns() as f64 / 1e9,
            external.selected_sha(),
        )?;
        let model_execution_time = start.elapsed().as_secs_f64();
        let inference_finished = process::monotonic();
        let inference_cpu_ms = (thread_cpu() - inference_cpu_start) * 1000.0;
        let published = prediction.is_some();
        if let Some(prediction) = prediction {
            let config = if (publication.desire.frame + 1).is_multiple_of(100) {
                Some(parameters::desire_config(&params)?)
            } else {
                None
            };
            let now = process::timestamp()?;
            let mut messages = publication.build(
                &mut state,
                Output {
                    prediction: &prediction,
                    timing: ModelTiming {
                        log_mono_time: now,
                        frame_id: frame.metadata.frame_id,
                        frame_id_extra: extra_frame.metadata.frame_id,
                        camera_state_frame_id: sources.road.get_frame_id(),
                        frame_drop: dropped.ratio,
                        timestamp_eof: frame.metadata.timestamp_eof,
                        model_execution_time,
                        valid: calibration.seen(),
                    },
                    simulation,
                    dropped: dropped.dropped,
                    raw_predictions: jetlink::raw_predictions(
                        raw,
                        external.status.decision.source,
                        runtime.raw_predictions(),
                    )?,
                },
                Sources {
                    car: sources.car,
                    navigation: sources.navigation,
                    radar: sources.radar,
                    lateral_active: sources.control.get_lat_active(),
                    live_lateral_delay: sources.delay.get_lateral_delay().into(),
                    indicator: indicator::inputs(
                        &subscribers.state,
                        sources.control,
                        &params,
                        &publication.desire,
                        process::monotonic(),
                    )?,
                },
                || config.clone().unwrap_or_default(),
                |allowed| commands.read(allowed, process::monotonic()),
            )?;
            let fresh = external.valid_at_publish();
            jetlink::persist(&params, &mut external_stored, &external.status)?;
            jetlink::publication(&mut messages, &external.status, fresh)?;
            publishers.send(
                "modelV2",
                &capnp::serialize::write_message_to_words(&messages.model),
            )?;
            publishers.send(
                "drivingModelData",
                &capnp::serialize::write_message_to_words(&messages.driving),
            )?;
            publishers.send(
                "cameraOdometry",
                &capnp::serialize::write_message_to_words(&messages.pose),
            )?;
            if usb_first_publication && runtime.uses_usb() {
                usb_selection::published(&params)?;
                usb_first_publication = false;
            }
        }
        if !runtime.uses_usb() {
            if let Some(active_recovery) = &mut recovery {
                let now = process::monotonic();
                let camera_age = now - frame.metadata.timestamp_eof as f64 * 1e-9;
                let healthy = published
                    && !external_frame
                    && calibration.seen()
                    && (0. ..0.15).contains(&camera_age);
                let join_topics = ["carState", "selfdriveState", "carControl"];
                let mut valid = [false; 3];
                let mut received = [0.; 3];
                for (index, name) in join_topics.into_iter().enumerate() {
                    let topic = subscribers.state.topic(name)?;
                    valid[index] = topic.valid && topic.alive;
                    received[index] = topic.receive_time;
                }
                let join = openpilot_driving_modeld::recovery::may_join(
                    now,
                    valid,
                    received,
                    sources.car.get_standstill(),
                    f64::from(sources.car.get_v_ego()),
                    sources.car.get_steering_pressed(),
                );
                let join = join
                    && !external_frame
                    && !sources.control.get_lat_active()
                    && !sources.control.get_long_active();
                let previous = active_recovery.policy.state;
                let recovered = active_recovery
                    .observe(now, healthy, now - camera_ready, dropped.dropped > 0)
                    .and_then(|()| {
                        active_recovery.tick(
                            now,
                            params.get_bool("UsbGpuPresent")?
                                && params.get_bool("UsbGpuCompiled")?,
                            join,
                        )
                    });
                match recovered {
                    Ok(Some(model)) => {
                        runtime.enable_usb(model);
                        usb_first_publication = true;
                        params.put_bool("UsbGpuStartupFailed", false)?;
                        params.put_bool("UsbGpuLoading", true)?;
                        drops.reset_warmup();
                    }
                    Ok(None) if previous != active_recovery.policy.state => {
                        use openpilot_driving_modeld::recovery_policy::State;
                        match active_recovery.policy.state {
                            State::Recovering | State::Ready => {
                                params.put_bool("UsbGpuLoading", true)?
                            }
                            State::Stopped => usb_selection::fail(&params)?,
                            _ => params.put_bool("UsbGpuLoading", false)?,
                        }
                    }
                    Ok(None) => {}
                    Err(error) => {
                        usb_selection::fail(&params)?;
                        eprintln!("eGPU recovery disabled; retaining internal model: {error}");
                        recovery = None;
                    }
                }
            }
        }
        {
            let now = process::monotonic();
            let backend = if external.status.decision.source
                == openpilot_jetlink::transition::Source::Jetlink
            {
                "jetlink"
            } else if runtime.uses_usb() {
                "usbgpu"
            } else {
                "internal"
            };
            camera_gap.record(
                logger,
                now,
                frame.metadata.frame_id,
                dropped.dropped,
                backend,
                &[
                    ("camera_wait", (camera_ready - loop_start) * 1000.0),
                    (
                        "before_inference",
                        (inference_started - camera_ready) * 1000.0,
                    ),
                    ("inference", model_execution_time * 1000.0),
                    ("postprocess", (now - inference_finished) * 1000.0),
                    ("loop", (now - loop_start) * 1000.0),
                ],
            );
        }
        FrameTiming {
            frame_id: frame.metadata.frame_id,
            loop_start,
            cpu_start,
            camera_ready,
            camera_age_at_run_ms,
            inference_seconds: model_execution_time,
            inference_cpu_ms,
            inference_finished,
            postprocess_end: process::monotonic(),
            loop_end: process::monotonic(),
            cpu_end: thread_cpu(),
            dropped: dropped.dropped,
            published,
            jetlink: external.frame_stages_ms(),
        }
        .record(&mut diagnostics, logger);
        frame_count += 1;
        if options.frames == Some(frame_count) {
            break;
        }
    }
    Ok(())
}
