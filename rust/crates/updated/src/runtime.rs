use crate::{
    common,
    process::args,
    report,
    signals::{UserRequest, Wake},
    updater::Updater,
    Error,
};
use openpilot_logging::{
    log_site,
    record::{Level, Record},
};
use std::{
    fs::{File, OpenOptions},
    time::Duration,
};

#[derive(Debug, Clone, Copy, serde::Serialize)]
pub struct Cycle {
    pub wait_seconds: u64,
    pub warmup: bool,
    pub failed_count: u64,
}
pub struct LoopState {
    pub first_run: bool,
    pub failed_count: u64,
}
impl Default for LoopState {
    fn default() -> Self {
        Self {
            first_run: true,
            failed_count: 0,
        }
    }
}
impl LoopState {
    pub fn cycle(&mut self, updater: &mut Updater<'_>, wake: &Wake) -> Result<Cycle, Error> {
        wake.clear_ready()?;
        let attempt = (|| -> Result<bool, Error> {
            updater.init_overlay()?;
            updater.set_params(false, self.failed_count, None)?;
            if !openpilot_timed::clock::valid(
                updater.context.clock,
                &updater.context.paths.system("/lib/systemd/systemd"),
            )? || self.first_run
            {
                self.first_run = false;
                return Ok(true);
            }
            self.failed_count = self
                .failed_count
                .checked_add(1)
                .ok_or(Error::Contract("update failure counter overflow"))?;
            updater.params.put("UpdaterState", b"checking...")?;
            updater.check_for_update()?;
            let timed_out = match updater.params.date("UpdaterLastFetchTime")? {
                None => true,
                Some(crate::ParamTime::Naive(last)) => {
                    updater.context.now()?.naive_utc() - last > chrono::Duration::days(3)
                }
                Some(crate::ParamTime::Aware(_)) => {
                    return Err(Error::Contract(
                        "can't subtract offset-naive and offset-aware datetimes",
                    ))
                }
            };
            let user_fetch = wake.request()? == UserRequest::Fetch;
            if updater
                .context
                .paths
                .finalized()
                .join(".runtime-update-ready.json")
                .try_exists()?
                && !user_fetch
            {
                report::info(
                    updater.context.logger,
                    "preserving prepared update until an explicit download request",
                )?;
            } else if updater.params.boolean("IsOnroad")? && !user_fetch {
                // The original runs updated only offroad. Here it stays up
                // onroad for requests, but the automatic fetch copies the whole
                // checkout into finalized (GBs) and on a small disk that makes
                // the deleter remove the drive's logs.
                report::info(
                    updater.context.logger,
                    "skipping fetch while onroad; a download request still runs",
                )?;
            } else if updater.params.boolean("NetworkMetered")? && !timed_out && !user_fetch {
                report::info(updater.context.logger, "skipping fetch, connection metered")?;
            } else if wake.request()? == UserRequest::Check {
                report::info(updater.context.logger, "skipping fetch, only checking")?;
            } else {
                updater.fetch_update()?;
                updater
                    .params
                    .put_date("UpdaterLastFetchTime", updater.context.now()?)?;
            }
            self.failed_count = 0;
            Ok(false)
        })();
        // Retrying every 5 min would fetch again just to meet the same full disk.
        let storage_short = matches!(attempt, Err(Error::Storage { .. }));
        let failure = match attempt {
            Ok(true) => {
                return Ok(Cycle {
                    wait_seconds: 60,
                    warmup: true,
                    failed_count: self.failed_count,
                })
            }
            Ok(false) => None,
            Err(Error::Interrupted) => return Err(Error::Interrupted),
            Err(error) => {
                let text = match &error {
                    Error::Command {
                        command,
                        code,
                        output,
                    } => {
                        report::event(
                            updater.context.logger,
                            "update process failed",
                            serde_json::json!({"cmd":command,"output":output,"returncode":code}),
                        )?;
                        format!("command failed: {}\n{output}", python_arguments(command))
                    }
                    Error::Storage { .. } => {
                        report::info(updater.context.logger, &error.to_string())?;
                        error.to_string()
                    }
                    _ => {
                        report::exception(
                            updater.context.logger,
                            "uncaught updated exception, shouldn't happen",
                            &error,
                        )?;
                        error.to_string()
                    }
                };
                common::unlink_missing_ok(&updater.context.paths.overlay_init())?;
                Some(text)
            }
        };
        let params_result = (|| {
            updater.params.put("UpdaterState", b"idle")?;
            updater.set_params(
                self.failed_count == 0,
                self.failed_count,
                failure.as_deref(),
            )
        })();
        if let Err(error) = params_result {
            if matches!(error, Error::Interrupted) {
                return Err(error);
            }
            report::exception(
                updater.context.logger,
                "uncaught updated exception while setting params, shouldn't happen",
                &error,
            )?;
        }
        wake.clear_request()?;
        Ok(Cycle {
            wait_seconds: if self.failed_count > 0 && !storage_short {
                300
            } else {
                5400
            },
            warmup: false,
            failed_count: self.failed_count,
        })
    }
}
fn python_arguments(args: &[String]) -> String {
    format!(
        "[{}]",
        args.iter()
            .map(|arg| {
                let quote = if arg.contains('\'') && !arg.contains('"') {
                    '"'
                } else {
                    '\''
                };
                format!(
                    "{quote}{}{quote}",
                    arg.replace('\\', "\\\\")
                        .replace(quote, &format!("\\{quote}"))
                        .replace('\n', "\\n")
                        .replace('\r', "\\r")
                        .replace('\t', "\\t")
                )
            })
            .collect::<Vec<_>>()
            .join(", ")
    )
}
pub fn acquire_lock(updater: &mut Updater<'_>) -> Result<Option<File>, Error> {
    if updater.params.boolean("DisableUpdates")? {
        updater.context.logger.emit(
            log_site!(),
            Record::text(
                Level::Warning,
                "updates are disabled by the DisableUpdates param".into(),
            ),
        )?;
        return Ok(None);
    }
    let lock = OpenOptions::new()
        .create(true)
        .truncate(true)
        .write(true)
        .open(&updater.context.paths.lock)?;
    lock.try_lock()
        .map_err(|_| Error::Contract("couldn't get overlay lock; is another instance running?"))?;
    Ok(Some(lock))
}
pub fn run(updater: &mut Updater<'_>, wake: &Wake, cycles: Option<u64>) -> Result<(), Error> {
    let Some(_lock) = acquire_lock(updater)? else {
        return Ok(());
    };
    updater.context.commands.run(
        &args(&[
            "ionice",
            "-c",
            "2",
            "-n",
            "7",
            "-p",
            &std::process::id().to_string(),
        ]),
        None,
    )?;
    if updater.context.paths.staging.join("old_openpilot").is_dir() {
        report::event(
            updater.context.logger,
            "update installed",
            serde_json::json!({}),
        )?;
    }
    if updater.params.date("InstallDate")?.is_none() {
        updater
            .params
            .put_date("InstallDate", updater.context.now()?)?;
    }
    common::set_consistent_flag(
        updater.context.paths,
        &updater.context.paths.finalized(),
        false,
    )?;
    updater.params.put("UpdaterState", b"idle")?;
    let mut state = LoopState::default();
    let mut count = 0;
    while !wake.stopped() {
        let cycle = match state.cycle(updater, wake) {
            Ok(cycle) => cycle,
            Err(Error::Interrupted) => break,
            Err(error) => return Err(error),
        };
        count += 1;
        if cycles.is_some_and(|limit| count >= limit) {
            break;
        }
        wake.sleep(Duration::from_secs(cycle.wait_seconds))?;
    }
    Ok(())
}
