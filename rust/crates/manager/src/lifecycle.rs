use crate::{
    parameters::{write_onroad, Parameters},
    Error,
};
use openpilot_process_supervision::ProcessState;

#[derive(Clone, Copy, Debug, Default)]
pub struct Input {
    pub started: bool,
    pub ignition: bool,
    pub not_car: bool,
    pub device_checks: bool,
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ExitAction {
    Uninstall,
    Reboot,
    Shutdown,
}

pub trait Runtime {
    fn start(&mut self) -> Result<(), Error>;
    fn cleanup_finished(&mut self) -> Result<(), Error>;
    fn poll(&mut self) -> Result<Input, Error>;
    fn initial_not_car(&self) -> Result<bool, Error>;
    fn stop_monitoring(&mut self) -> Result<(), Error>;
    fn xiaoge_ready(&mut self, started: bool, controls_ready: bool) -> Result<bool, Error>;
    fn ensure_running(
        &mut self,
        started: bool,
        not_car: bool,
        ignore: &[String],
    ) -> Result<(), Error>;
    fn states(&mut self) -> Result<Vec<ProcessState>, Error>;
    fn publish(&mut self, states: &[ProcessState]) -> Result<(), Error>;
    fn watchdog(&mut self) -> Result<(), Error>;
    fn timestamp(&mut self) -> String;
    fn log_running(&mut self, states: &[ProcessState], print: bool) -> Result<(), Error>;
    fn warning(&mut self, message: &str) -> Result<(), Error>;
    fn stop(&mut self, block: bool) -> Result<(), Error>;
    fn capture_exception(&mut self, error: &Error) -> Result<(), Error>;
    fn exit(&mut self, action: ExitAction) -> Result<(), Error>;
}
#[derive(Default)]
pub struct Environment {
    pub no_board: bool,
    pub block: String,
    pub prepare_only: bool,
}
impl Environment {
    pub fn capture() -> Self {
        Self {
            no_board: std::env::var_os("NOBOARD").is_some(),
            block: std::env::var("BLOCK").unwrap_or_default(),
            prepare_only: std::env::var_os("PREPAREONLY").is_some(),
        }
    }
}

pub fn ignore_list(params: &impl Parameters, env: &Environment) -> Result<Vec<String>, Error> {
    let mut ignore = Vec::new();
    let dongle = params.get("DongleId")?;
    if dongle
        .as_deref()
        .and_then(|bytes| std::str::from_utf8(bytes).ok())
        .is_none_or(|value| {
            value == openpilot_registration::UNREGISTERED_DONGLE_ID || value.is_empty()
        })
    {
        ignore.extend(["manage_athenad".into(), "uploader".into()]);
    }
    if env.no_board {
        ignore.push("pandad".into());
    }
    ignore.extend(
        env.block
            .split(',')
            .filter(|value| !value.is_empty())
            .map(str::to_owned),
    );
    if params.boolean("HardwareC3xLite")? {
        // C3X Lite has no microphone or speaker. carrot also leaves loggerd
        // out here, which left Lite drives without rlog/qlog for problem
        // reports; loggerd itself records no audio on Lite (it never waits
        // for audio that cannot come), so it runs.
        ignore.extend(["micd".into(), "soundd".into()]);
        params.put_bool("RecordAudio", false)?;
    }
    Ok(ignore)
}

pub fn manager_thread(
    params: &impl Parameters,
    runtime: &mut impl Runtime,
    env: &Environment,
) -> Result<(), Error> {
    runtime.start()?;
    let ignore = ignore_list(params, env)?;
    write_onroad(params, false)?;
    runtime.ensure_running(false, runtime.initial_not_car()?, &ignore)?;
    let mut started_prev = false;
    let mut ignition_prev = false;
    let mut print_timer = 0;
    loop {
        let input = runtime.poll()?;
        if input.started && !started_prev {
            params.clear(openpilot_params::CLEAR_ON_ONROAD_TRANSITION)?;
        } else if !input.started && started_prev {
            params.clear(openpilot_params::CLEAR_ON_OFFROAD_TRANSITION)?;
        }
        if input.ignition && !ignition_prev {
            // Drain the old monitor's asynchronous session-disable writes first.
            runtime.stop_monitoring()?;
            params.clear(openpilot_params::CLEAR_ON_IGNITION_ON)?;
        }
        if input.started != started_prev {
            write_onroad(params, input.started)?;
        }
        started_prev = input.started;
        ignition_prev = input.ignition;
        let ready = runtime.xiaoge_ready(input.started, params.boolean("ControlsReady")?)?;
        let mut not_run = ignore.clone();
        if !ready {
            not_run.push("xiaoge_data".into());
        }
        runtime.ensure_running(input.started, input.not_car, &not_run)?;
        // State production and transport remain one native operation. Diagnostics do
        // not affect transition state and print every tenth iteration.
        let states = runtime.states()?;
        print_timer = (print_timer + 1) % 10;
        runtime.log_running(&states, print_timer == 0)?;
        runtime.publish(&states)?;
        if input.device_checks {
            // Source explicitly catches every watchdog write exception.
            match runtime.watchdog() {
                Ok(()) | Err(_) => {}
            }
        }
        let mut shutdown = false;
        for key in ["DoUninstall", "DoShutdown", "DoReboot"] {
            if params.boolean(key)? {
                shutdown = true;
                params.put(
                    "LastManagerExitReason",
                    format!("{key} {}", runtime.timestamp()).as_bytes(),
                )?;
                runtime.warning(&format!("Shutting down manager - {key} set"))?;
            }
        }
        if shutdown {
            return Ok(());
        }
    }
}

pub fn run(
    params: &impl Parameters,
    runtime: &mut impl Runtime,
    env: &Environment,
) -> Result<(), Error> {
    if env.prepare_only {
        return Ok(());
    }
    let result = manager_thread(params, runtime, env);
    let result = match result {
        Ok(()) => Ok(()),
        Err(Error::Interrupted) => Err(Error::Interrupted),
        Err(error) => runtime.capture_exception(&error),
    };
    // Source cleanup is two complete ordered passes, including disabled entries.
    runtime.stop(false)?;
    runtime.stop(true)?;
    runtime.cleanup_finished()?;
    result?;
    let action = if params.boolean("DoUninstall")? {
        Some(ExitAction::Uninstall)
    } else if params.boolean("DoReboot")? {
        Some(ExitAction::Reboot)
    } else if params.boolean("DoShutdown")? {
        Some(ExitAction::Shutdown)
    } else {
        None
    };
    if let Some(action) = action {
        runtime.exit(action)?;
    }
    Ok(())
}
