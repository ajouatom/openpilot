use crate::{
    config::Config, controller::Controls, interface::ControlInterface, parameters, platform, Error,
    SERVICES,
};
use openpilot_control_policy::numerics::Numerics;
use openpilot_logging::{
    log_site,
    producer::Factory,
    record::{Level, Record},
};
use openpilot_messaging::{
    runtime::{PubMaster, SubMaster},
    state::{Options, Poll},
};
use openpilot_params::Params;
use std::{
    collections::VecDeque,
    path::Path,
    sync::{
        atomic::{AtomicBool, Ordering},
        Arc,
    },
    thread,
    time::Duration,
};

pub struct Monitor {
    last: f64,
    next: f64,
    pub remaining: f64,
    pub frames: u64,
    intervals: VecDeque<f64>,
}
impl Default for Monitor {
    fn default() -> Self {
        Self {
            last: -1.,
            next: -1.,
            remaining: 0.,
            frames: 0,
            intervals: VecDeque::from([0.01]),
        }
    }
}
impl Monitor {
    pub fn monitor(&mut self, mut clock: impl FnMut() -> f64) {
        if self.last < 0. {
            self.next = clock() + 0.01;
            self.last = clock();
        }
        let previous = self.last;
        self.last = clock();
        if self.intervals.len() == 100 {
            self.intervals.pop_front();
        }
        self.intervals.push_back(self.last - previous);
        self.remaining = self.next - clock();
        self.next += 0.01;
        self.frames += 1;
    }
}
pub fn run(frames: Option<u64>, assets: &Path, numerics: &Path) -> Result<(), Error> {
    platform::configure()?;
    let stop = Arc::new(AtomicBool::new(false));
    signal_hook::flag::register(signal_hook::consts::SIGINT, Arc::clone(&stop))?;
    signal_hook::flag::register(signal_hook::consts::SIGTERM, Arc::clone(&stop))?;
    let mut params = Params::for_runtime()?;
    let mut logger = Factory::for_runtime()?.logger();
    logger.emit(
        log_site!(),
        Record::text(Level::Info, "controlsd is waiting for CarParams".into()),
    )?;
    let config = loop {
        if stop.load(Ordering::Relaxed) {
            return Ok(());
        }
        let bytes = parameters::raw(&params, "CarParams")?;
        if !bytes.is_empty() {
            break Config::decode(&bytes)?;
        }
        thread::sleep(Duration::from_millis(100));
    };
    logger.emit(
        log_site!(),
        Record::text(Level::Info, "controlsd got CarParams".into()),
    )?;
    let interface = ControlInterface::new(&config, &mut params, assets, Numerics::load(numerics)?)?;
    let mut subscriber = SubMaster::for_runtime(
        &SERVICES,
        Options {
            poll: Poll::One("selfdriveState".into()),
            ..Options::default()
        },
    )?;
    let mut publisher = PubMaster::for_runtime(&["carControl", "controlsState"])?;
    let mut controls = Controls::new(config, interface, &mut params)?;
    let mut monitor = Monitor::default();
    let mut model = crate::inputs::Model::default();
    while !stop.load(Ordering::Relaxed) {
        subscriber.update(Duration::from_millis(15))?;
        if subscriber.state.topic("modelV2")?.updated {
            use crate::parameters::Parameters;
            model = crate::input_decode::decode_model(
                &subscriber.state,
                params.integer("LanelessCentering")? > 0,
            )?;
        }
        let input = crate::input_decode::decode_with_model(
            &subscriber.state,
            platform::monotonic(),
            model,
        )?;
        let command = controls.control(&input, &mut params)?;
        for error in &command.errors {
            logger.emit(log_site!(), Record::text(Level::Error, error.clone()))?;
        }
        let packets =
            crate::publication::publish(&mut controls, &input, &command, &mut params, || {
                (platform::monotonic() * 1e9) as u64
            })?;
        publisher.send("controlsState", &packets[0])?;
        publisher.send("carControl", &packets[1])?;
        model = input.model;
        monitor.monitor(platform::monotonic);
        if frames.is_some_and(|limit| monitor.frames >= limit) {
            break;
        }
    }
    Ok(())
}
