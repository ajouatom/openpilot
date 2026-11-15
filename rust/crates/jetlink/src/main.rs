use openpilot_jetlink::{
    ffs::Session,
    mobile::{
        self,
        backend::{Cancel, MobileBackend, Options as BackendOptions},
        cable::MobileCable,
        config::Mode as TransportMode,
    },
    owner::{GadgetOwner, Server},
    Error,
};
use openpilot_params::Params;
use openpilot_runtime_core::jetlink_power::{self, InputPower};
use std::{
    env,
    fs::{self, OpenOptions},
    io::{Read, Seek, SeekFrom},
    path::{Path, PathBuf},
    process::{Command, Stdio},
    sync::{
        atomic::{AtomicBool, Ordering},
        Arc, Mutex,
    },
    thread,
    time::{Duration, Instant},
};
#[derive(Debug, thiserror::Error)]
enum EntryError {
    #[error("{0}")]
    Arguments(&'static str),
    #[error(transparent)]
    Io(#[from] std::io::Error),
    #[error(transparent)]
    Params(#[from] openpilot_params::Error),
    #[error(transparent)]
    Json(#[from] serde_json::Error),
    #[error(transparent)]
    Parse(#[from] std::num::ParseIntError),
    #[error(transparent)]
    Jetlink(#[from] Error),
}

/// Runs the root cut helper; Ok(true) when the zero-current vote was applied.
fn cut_input() -> std::io::Result<bool> {
    let mut command = if rustix::process::geteuid().is_root() {
        Command::new(env::current_exe()?)
    } else {
        let mut command = Command::new("sudo");
        command.arg("-n").arg(env::current_exe()?);
        command
    };
    let mut child = command.arg("--usb-power").stdin(Stdio::null()).spawn()?;
    let until = Instant::now() + Duration::from_secs(5);
    loop {
        if let Some(status) = child.try_wait()? {
            return match status.code() {
                Some(0) => Ok(true),
                Some(3) => Ok(false),
                _ => Err(std::io::Error::other("USB input-current helper failed")),
            };
        }
        if Instant::now() >= until {
            child.kill()?;
            child.wait()?;
            return Err(std::io::Error::new(
                std::io::ErrorKind::TimedOut,
                "USB input-current helper exceeded 5s",
            ));
        }
        thread::sleep(Duration::from_millis(5));
    }
}

fn setup_backoff(detail: &str) -> Duration {
    if detail.contains("already owns the controller")
        || detail.contains("already holds the device controller")
    {
        Duration::from_secs(2)
    } else {
        // Carrot retries a failed session after 2 s.
        Duration::from_secs(2)
    }
}
fn setup_plain_gadget() -> Result<(), Error> {
    let mut output = tempfile::tempfile()?;
    let mut child = Command::new("sudo")
        .arg("-n")
        .arg(env::current_exe()?)
        .arg("--setup-gadget")
        .stdin(Stdio::null())
        .stdout(output.try_clone()?)
        .stderr(output.try_clone()?)
        .spawn()?;
    let until = Instant::now() + Duration::from_secs(15);
    loop {
        if let Some(status) = child.try_wait()? {
            return if status.success() {
                Ok(())
            } else {
                output.seek(SeekFrom::Start(0))?;
                let mut detail = String::new();
                output.take(4096).read_to_string(&mut detail)?;
                Err(Error::Native(format!(
                    "gadget setup failed: {}",
                    detail.trim()
                )))
            };
        }
        if Instant::now() >= until {
            child.kill()?;
            child.wait()?;
            return Err(Error::Deadline);
        }
        thread::sleep(Duration::from_millis(20));
    }
}

fn setup_gadget(mode: TransportMode) -> Result<(), Error> {
    if mode.composite() {
        return mobile::root("gadget");
    }
    let gadget = Path::new(openpilot_jetlink::gadget::GADGET);
    if gadget.join("functions/ncm.jetlink").exists()
        || gadget.join("configs/c.1/ncm.jetlink").is_symlink()
    {
        mobile::root("--teardown")?;
    }
    setup_plain_gadget()
}
struct Link {
    session: Arc<Session>,
    cable: Option<Arc<MobileCable>>,
    server: Option<Server>,
    cancel: Option<Arc<Cancel>>,
    hello: Arc<AtomicBool>,
    tcp: Arc<AtomicBool>,
    status: Arc<Mutex<serde_json::Value>>,
}
impl Link {
    fn spawn(&mut self, socket: &Path, mut options: BackendOptions) -> Result<(), Error> {
        match fs::remove_file(socket) {
            Ok(()) => {}
            Err(error) if error.kind() == std::io::ErrorKind::NotFound => {}
            Err(error) => return Err(error.into()),
        }
        options.status = Arc::clone(&self.status);
        let backend = MobileBackend::new(Arc::clone(&self.session), self.cable.clone(), options);
        self.hello = Arc::clone(&backend.hello_complete);
        self.tcp = Arc::clone(&backend.selected_tcp);
        self.cancel = Some(Arc::clone(&backend.cancel));
        self.server = Some(Server::spawn(backend, socket)?);
        Ok(())
    }
    fn stop_inference(&mut self) -> Result<(), Error> {
        if let Some(cancel) = &self.cancel {
            cancel.cancel()?;
        }
        if let Some(server) = &mut self.server {
            server.stop(Duration::from_secs(2))?;
        }
        self.server = None;
        self.cancel = None;
        Ok(())
    }
    fn teardown(&mut self) -> Result<(), Error> {
        if let Some(cancel) = &self.cancel {
            cancel.cancel()?;
        }
        if let Some(cable) = &self.cable {
            cable.close()?;
        }
        self.session.teardown()?;
        self.stop_inference()?;
        self.cable = None;
        Ok(())
    }
}
fn publish_link(
    socket: &Path,
    link: Option<&Link>,
    configured: TransportMode,
    state: &str,
    detail: &str,
) -> Result<(), Error> {
    let mut value = link
        .and_then(|link| link.status.lock().ok().map(|value| value.clone()))
        .unwrap_or_else(|| serde_json::json!({}));
    let transport = if link.is_some_and(|link| link.tcp.load(Ordering::Acquire)) {
        "ios"
    } else {
        match configured {
            TransportMode::Ios => "ios",
            TransportMode::Android => "android",
            TransportMode::Auto
                if openpilot_jetlink::peer::classify(&value["peer"], "usb")
                    == Some(openpilot_jetlink::peer::Kind::Android) =>
            {
                "android"
            }
            _ => "usb",
        }
    };
    let configured = match configured {
        TransportMode::Auto => "auto",
        TransportMode::Usb => "usb",
        TransportMode::Android => "android",
        TransportMode::Ios => "ios",
    };
    let state = if state == "connecting" && value["state"] == "updating" {
        "updating"
    } else if state == "connecting" && value["state"] == "loading" {
        "loading"
    } else {
        state
    };
    value["state"] = serde_json::json!(state);
    if value.get("model").is_none() {
        value["model"] = serde_json::json!("App selection");
    }
    value["updated"] = serde_json::json!(openpilot_jetlink::now_ns() as f64 / 1e9);
    value["transport"] = serde_json::json!(transport);
    value["configured_transport"] = serde_json::json!(configured);
    value["error"] = serde_json::json!(detail);
    if !matches!(state, "connecting" | "loading" | "updating") {
        value
            .as_object_mut()
            .ok_or(Error::Contract("link status object"))?
            .remove("preparation");
    }
    openpilot_jetlink::status::write(&socket.with_file_name("carrot-jetlink.json"), &value)?;
    Ok(())
}
fn options(
    mode: TransportMode,
    admission_mode: TransportMode,
    version: openpilot_jetlink::wire::Version,
    grace: Duration,
    cache: &Path,
    legacy: bool,
) -> BackendOptions {
    BackendOptions {
        mode,
        admission_mode,
        version,
        grace,
        cache: cache.to_owned(),
        legacy,
        status: Arc::new(Mutex::new(serde_json::json!({}))),
    }
}

fn run() -> Result<(), EntryError> {
    let helper_args = env::args().skip(1).collect::<Vec<_>>();
    if helper_args == ["--usb-power"] {
        if !rustix::process::geteuid().is_root() {
            return Err(EntryError::Arguments(
                "USB input-current helper requires root",
            ));
        }
        let cut = jetlink_power::cut_input(
            Path::new(jetlink_power::POWER_ROLE),
            Path::new(jetlink_power::VOTER),
        )?;
        std::process::exit(if cut == jetlink_power::Cut::Applied {
            0
        } else {
            3
        });
    }
    if helper_args == ["--wifi-publisher"] {
        openpilot_jetlink::wifi::run()?;
        return Ok(());
    }
    if helper_args.len() == 2 && helper_args[0] == "--mobile-root" {
        openpilot_jetlink::mobile::helper(&helper_args[1])?;
        return Ok(());
    }
    if env::args().skip(1).collect::<Vec<_>>() == ["--setup-gadget"] {
        openpilot_jetlink::gadget::provision_device()?;
        return Ok(());
    }
    let mut args = env::args().skip(1);
    let mut socket = PathBuf::from("/dev/shm/carrot-jetlink.sock");
    let mut iterations = None;
    match args.next().as_deref() {
        Some("--run") => {}
        _ => {
            return Err(EntryError::Arguments(
                "usage: jetlinkd-rs --run [--socket PATH] [--iterations N]",
            ))
        }
    }
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--socket" => {
                socket = args
                    .next()
                    .ok_or(EntryError::Arguments("missing socket"))?
                    .into()
            }
            "--iterations" => {
                iterations = Some(
                    args.next()
                        .ok_or(EntryError::Arguments("missing iterations"))?
                        .parse::<usize>()?,
                )
            }
            _ => return Err(EntryError::Arguments("unknown option")),
        }
    }
    let lock_path = socket.with_extension("owner.lock");
    let lock = OpenOptions::new()
        .read(true)
        .write(true)
        .create(true)
        .truncate(false)
        .open(lock_path)?;
    rustix::fs::flock(&lock, rustix::fs::FlockOperation::NonBlockingLockExclusive)
        .map_err(std::io::Error::from)?;
    let params = Arc::new(Params::for_runtime()?);
    // Carrot's auxiliary vision: xiaoge asks here for its camera models to run
    // on the App. Without the socket xiaoge keeps running them on the CPU.
    if let Err(error) = openpilot_jetlink::aux::serve_local(
        openpilot_jetlink::aux::shared(),
        Path::new(openpilot_jetlink::aux::SOCKET),
    ) {
        eprintln!("jetlink aux socket: {error}");
    }
    let stop = Arc::new(AtomicBool::new(false));
    signal_hook::flag::register(signal_hook::consts::SIGINT, Arc::clone(&stop))?;
    signal_hook::flag::register(signal_hook::consts::SIGTERM, Arc::clone(&stop))?;
    let legacy_native = match env::var("JETLINK_MODEL_CONTRACT").as_deref() {
        Err(env::VarError::NotPresent) | Ok("app") => false,
        Ok("native-cinque-v3") => true,
        _ => {
            return Err(EntryError::Arguments(
                "JETLINK_MODEL_CONTRACT must be app or native-cinque-v3",
            ))
        }
    };
    let protocol_setting = env::var("JETLINK_PROTOCOL").unwrap_or_else(|_| "auto".into());
    let mut protocol =
        openpilot_jetlink::protocol_choice::ProtocolChoice::parse(if legacy_native {
            "2"
        } else {
            &protocol_setting
        })?;
    let cache = env::var_os("JETLINK_MODEL_CACHE")
        .map(PathBuf::from)
        .unwrap_or_else(|| PathBuf::from("/data/jetlink"));
    let transport_mode = if legacy_native {
        TransportMode::Usb
    } else {
        TransportMode::device()?
    };
    if matches!(transport_mode, TransportMode::Ios | TransportMode::Android) {
        if protocol_setting == "2" {
            return Err(EntryError::Arguments(
                "Phone Jetlink mode requires protocol3",
            ));
        }
        protocol = openpilot_jetlink::protocol_choice::ProtocolChoice::parse("3")?;
    }
    let mut effective_transport = transport_mode;
    let mut reconnect_at = Instant::now();
    let mut owner = GadgetOwner::default();
    let mut input_power = InputPower::default();
    let clock = Instant::now();
    let agnos = Path::new("/AGNOS").is_file();
    let mut link: Option<Link> = None;
    let mut usb_probes = 0usize;
    let mut grace = Duration::from_secs(5);
    let mut failed = false;
    let mut failure_detail = String::new();
    let mut count = 0;
    while !stop.load(Ordering::Acquire) {
        // Carrot: no setting gates the link; a USB host attaching starts it.
        if agnos {
            input_power.step(
                clock.elapsed().as_secs_f64(),
                fs::read_to_string(jetlink_power::POWER_ROLE)
                    .ok()
                    .as_deref()
                    .map(str::trim),
                cut_input,
            );
        }
        if failed && Instant::now() >= reconnect_at {
            failed = false;
        }
        let attached = openpilot_jetlink::physical::device_attached(transport_mode);
        let lost = link
            .as_ref()
            .and_then(|link| link.server.as_ref())
            .is_some_and(|server| server.error.lock().is_ok_and(|error| !error.is_empty()));
        if owner.enabled() && (lost || !attached) {
            let current = link
                .as_mut()
                .ok_or(Error::Contract("owned Jetlink session missing"))?;
            let selected_tcp = current.tcp.load(Ordering::Acquire);
            let hello_failed = !current.hello.load(Ordering::Acquire);
            if attached && selected_tcp && current.session.configured()? {
                // An established NCM selection never triggers a USB probe until
                // physical detach; reconnect over the retained cable owner.
                current.stop_inference()?;
            } else {
                owner.disable(|| current.teardown())?;
                link = None;
                if !attached {
                    protocol = openpilot_jetlink::protocol_choice::ProtocolChoice::parse(
                        if legacy_native {
                            "2"
                        } else if matches!(
                            transport_mode,
                            TransportMode::Ios | TransportMode::Android
                        ) {
                            "3"
                        } else {
                            &protocol_setting
                        },
                    )?;
                    effective_transport = transport_mode;
                    usb_probes = 0;
                    grace = Duration::from_secs(5);
                } else if hello_failed && !selected_tcp {
                    protocol.handshake_failed();
                    usb_probes += 1;
                    if transport_mode == TransportMode::Auto
                        && protocol_setting == "auto"
                        && usb_probes >= 2
                    {
                        usb_probes = 0;
                        grace = Duration::from_secs(30);
                    } else {
                        grace = Duration::from_secs(5);
                    }
                }
            }
            reconnect_at = Instant::now() + Duration::from_secs(5);
            failed = false;
        }
        let result = if attached && !failed && Instant::now() >= reconnect_at {
            if owner.enabled() {
                if let Some(link) = &mut link {
                    if link.server.is_none() {
                        link.spawn(
                            &socket,
                            options(
                                TransportMode::Ios,
                                transport_mode,
                                openpilot_jetlink::wire::Version::V3,
                                grace,
                                &cache,
                                legacy_native,
                            ),
                        )
                    } else {
                        Ok(())
                    }
                } else {
                    Err(Error::Contract("enabled gadget has no owner"))
                }
            } else {
                owner.enable(false, || {
                    setup_gadget(effective_transport)?;
                    let session = Session::open(
                        Path::new(openpilot_jetlink::gadget::FFS_MOUNT),
                        Path::new(openpilot_jetlink::gadget::GADGET),
                        Path::new("/sys/class/udc"),
                    )?;
                    let mut candidate = Link {
                        session,
                        cable: None,
                        server: None,
                        cancel: None,
                        hello: Arc::new(AtomicBool::new(false)),
                        tcp: Arc::new(AtomicBool::new(false)),
                        status: Arc::new(Mutex::new(serde_json::json!({}))),
                    };
                    let prepared = (|| {
                        if effective_transport.composite() {
                            candidate.cable = Some(MobileCable::start(
                                Arc::clone(&candidate.session),
                                Duration::from_secs(5),
                            )?);
                        }
                        candidate.spawn(
                            &socket,
                            options(
                                effective_transport,
                                transport_mode,
                                protocol.version,
                                grace,
                                &cache,
                                legacy_native,
                            ),
                        )
                    })();
                    if let Err(error) = prepared {
                        if let Err(cleanup) = candidate.teardown() {
                            return Err(Error::Native(format!(
                                "{error}; unsafe Jetlink cleanup: {cleanup}"
                            )));
                        }
                        return Err(error);
                    }
                    link = Some(candidate);
                    Ok(())
                })
            }
        } else {
            Ok(())
        };
        let attempt_error = result.is_err();
        if let Err(error) = result {
            failed = true;
            failure_detail = error.to_string();
            reconnect_at = Instant::now() + setup_backoff(&failure_detail);
            if transport_mode == TransportMode::Auto
                && effective_transport.composite()
                && link.is_none()
                && setup_backoff(&failure_detail) != Duration::from_secs(2)
            {
                // Optional NCM setup must not disable the existing bulk path.
                // The root helper verifies the gadget is detached and owns the
                // network resources before a fresh USB-only attempt is allowed.
                match mobile::root("--teardown") {
                    Ok(()) => {
                        effective_transport = TransportMode::Usb;
                        reconnect_at = Instant::now() + Duration::from_secs(5);
                        failed = false;
                    }
                    Err(cleanup) => {
                        eprintln!("Jetlink cannot safely retry after NCM failure: {cleanup}")
                    }
                }
            }
            params.put(
                "JetlinkStatus",
                serde_json::to_vec(
                    &serde_json::json!({"phase":"ERROR","detail":error.to_string()}),
                )?
                .as_slice(),
            )?;
            publish_link(
                &socket,
                link.as_ref(),
                transport_mode,
                if failed { "blocked" } else { "retrying" },
                &error.to_string(),
            )?;
        }
        if !failed && !attempt_error {
            let server = link.as_ref().and_then(|link| link.server.as_ref());
            let detail = server
                .map(|s| {
                    s.error
                        .lock()
                        .map(|v| v.clone())
                        .unwrap_or_else(|_| "owner status poisoned".into())
                })
                .unwrap_or_default();
            let ready = server.is_some_and(|s| s.ready.load(Ordering::Acquire));
            let phase = if !owner.enabled() {
                "OFF"
            } else if ready {
                "READY"
            } else if !detail.is_empty() {
                "LOST"
            } else {
                "PREPARING"
            };
            params.put("JetlinkStatus",&serde_json::to_vec(&serde_json::json!({"phase":phase,"detail":detail,"generation":server.as_ref().map(|s|openpilot_jetlink::rpc::encode_generation(s.generation)).unwrap_or_default()}))?)?;
            publish_link(
                &socket,
                link.as_ref(),
                transport_mode,
                match phase {
                    "OFF" => "waiting",
                    "READY" => "ready",
                    "LOST" => "retrying",
                    _ => "connecting",
                },
                &detail,
            )?;
        }
        if failed && !attempt_error {
            publish_link(
                &socket,
                link.as_ref(),
                transport_mode,
                "blocked",
                &failure_detail,
            )?;
        }
        count += 1;
        if iterations == Some(count) {
            break;
        }
        thread::sleep(Duration::from_millis(500));
    }
    if let Some(link) = &mut link {
        link.teardown()?;
    }
    Ok(())
}

fn main() -> std::process::ExitCode {
    match openpilot_managed_entry::runtime(
        "openpilot.selfdrive.modeld.jetlink.daemon",
        "jetlinkd",
        || run().map_err(openpilot_managed_entry::StepError::raised),
    ) {
        Ok(_) => std::process::ExitCode::SUCCESS,
        Err(error) => {
            eprintln!("jetlinkd: {error}");
            std::process::ExitCode::FAILURE
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn controller_conflict_can_recover_after_adb_releases_without_hot_retry() {
        assert_eq!(
            setup_backoff("USB gadget 'g1' already holds the device controller (a600000.dwc3)"),
            Duration::from_secs(2)
        );
        assert_eq!(
            setup_backoff("a USB gadget already owns the controller"),
            Duration::from_secs(2)
        );
        // Carrot's daemon sleeps 2 s after every failed session, whatever the cause.
        assert_eq!(setup_backoff("USB mount failed"), Duration::from_secs(2));
    }
}
