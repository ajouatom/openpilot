use crate::{config::Config, runtime::pause, Error};
use openpilot_logging::{
    producer::Factory,
    record::{Level, Record},
};
use std::{
    fs::{self, File},
    io::{Read, Write},
    process::{Child, Command},
    sync::atomic::{AtomicBool, Ordering},
    time::Duration,
};
pub struct Downloader(Child);
impl Downloader {
    pub fn start(config: &Config) -> Result<Self, Error> {
        Ok(Self(
            Command::new(std::env::current_exe()?)
                .arg("--assistance-worker")
                .arg(serde_json::to_string(config)?)
                .spawn()?,
        ))
    }
    pub fn stop_retrying(&mut self) -> Result<(), Error> {
        if self.0.try_wait()?.is_none() {
            let pid = rustix::process::Pid::from_raw(
                i32::try_from(self.0.id()).map_err(|_| Error::Protocol("downloader PID"))?,
            )
            .ok_or(Error::Protocol("downloader PID zero"))?;
            match rustix::process::kill_process(pid, rustix::process::Signal::USR1) {
                Ok(()) | Err(rustix::io::Errno::SRCH) => (),
                Err(error) => return Err(error.into()),
            }
        }
        Ok(())
    }
}
impl Drop for Downloader {
    fn drop(&mut self) {
        if let Err(error) = self.0.kill() {
            eprintln!("qcomgpsd downloader kill: {error}");
        }
        if let Err(error) = self.0.wait() {
            eprintln!("qcomgpsd downloader wait: {error}");
        }
    }
}
pub fn run(config: &Config, stop: &AtomicBool) -> Result<(), Error> {
    if config.assistance.exists() {
        fs::remove_file(&config.assistance)?;
    }
    if let Some(path) = config.alternate.as_ref().filter(|path| path.exists()) {
        fs::copy(path, &config.assistance)?;
    }
    let mut logger = Factory::for_runtime()?.logger();
    let agent = openpilot_http_transport::socket_timeout_agent(
        ureq::Agent::config_builder()
            .http_status_as_error(false)
            .timeout_connect(Some(Duration::from_secs(5)))
            .max_redirects(30)
            .build(),
        Duration::from_secs(5),
    );
    let download = std::path::PathBuf::from(format!("{}.download", config.assistance.display()));
    while !config.assistance.exists() && !stop.load(Ordering::Relaxed) {
        match agent.get(&config.assistance_url).call() {
            Ok(mut response) => {
                let mut file = File::create(&download)?;
                let mut source = response.body_mut().as_reader();
                let mut size = 0;
                let mut complete = true;
                loop {
                    let mut chunk = Vec::with_capacity(8192);
                    match source.by_ref().take(8192).read_to_end(&mut chunk) {
                        Ok(0) => break,
                        Ok(count) => {
                            file.write_all(&chunk[..count])?;
                            size += count;
                        }
                        Err(error) => {
                            logger.emit(openpilot_logging::log_site!(), {
                                let mut record = Record::text(
                                    Level::Error,
                                    "Failed to download assistance file".into(),
                                );
                                record.exception = Some(error.to_string());
                                record
                            })?;
                            complete = false;
                            break;
                        }
                    }
                    if size > 100_000 {
                        logger.emit(
                            openpilot_logging::log_site!(),
                            Record::text(
                                Level::Error,
                                "Qcom assistance data larger than expected".into(),
                            ),
                        )?;
                        complete = false;
                        break;
                    }
                }
                if complete {
                    fs::rename(&download, &config.assistance)?;
                }
            }
            Err(error) => {
                logger.emit(openpilot_logging::log_site!(), {
                    let mut record =
                        Record::text(Level::Error, "Failed to download assistance file".into());
                    record.exception = Some(error.to_string());
                    record
                })?;
            }
        }
        match pause(Duration::from_secs(10), stop) {
            Ok(()) | Err(Error::Stopped) => (),
            Err(error) => return Err(error),
        }
    }
    Ok(())
}
/// Load XTRA data over AT (Quectel GNSS app note: upload to UFS, set the
/// reference time, apply, delete). carrot/openpilot's `mmcli` injection fails
/// on every start since its own modem daemon masks ModemManager; the modem's
/// XTRA data then stays expired. Failures stay non-fatal like the source.
pub fn inject(config: &Config, stop: &AtomicBool) -> Result<(), Error> {
    const NAME: &str = "xtra3grc.bin";
    let data = fs::read(&config.assistance)?;
    for _ in 0..5 {
        let at = &config.at;
        let _ = at.command(&format!("AT+QFDEL=\"UFS:{NAME}\""), stop);
        let result = at.upload(NAME, &data, stop).and_then(|()| {
            if openpilot_timed::clock::valid(&openpilot_timed::clock::SystemClock, &config.systemd)? {
                at.command(
                    &format!(
                        "AT+QGPSXTRATIME=0,\"{}\",1,1,5",
                        chrono::Utc::now().format("%Y/%m/%d,%H:%M:%S")
                    ),
                    stop,
                )?;
            }
            at.command(&format!("AT+QGPSXTRADATA=\"UFS:{NAME}\""), stop)
        });
        let _ = at.command(&format!("AT+QFDEL=\"UFS:{NAME}\""), stop);
        match result {
            Ok(_) => {
                Factory::for_runtime()?.logger().emit(
                    openpilot_logging::log_site!(),
                    Record::text(Level::Info, "successfully loaded assistance data".into()),
                )?;
                return Ok(());
            }
            Err(Error::Stopped) => return Err(Error::Stopped),
            Err(error) => println!("inject_assistance failed ({error}), trying again"),
        }
        pause(Duration::from_millis(200), stop)?;
    }
    println!("inject_assistance failed after retry");
    Ok(())
}
