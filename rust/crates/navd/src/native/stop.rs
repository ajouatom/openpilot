use crate::Error;
use crossbeam_channel::{bounded, select, Receiver};
use signal_hook::iterator::{Handle, Signals};
use std::{
    sync::{
        atomic::{AtomicBool, Ordering},
        Arc,
    },
    thread::{self, JoinHandle},
    time::Duration,
};

pub struct Stop {
    pub requested: Arc<AtomicBool>,
    receiver: Receiver<()>,
    handle: Handle,
    worker: Option<JoinHandle<()>>,
}

impl Stop {
    pub fn new() -> Result<Self, Error> {
        let mut signals = Signals::new([signal_hook::consts::SIGINT])?;
        let handle = signals.handle();
        let requested = Arc::new(AtomicBool::new(false));
        let flag = Arc::clone(&requested);
        let (sender, receiver) = bounded(1);
        let worker = thread::Builder::new()
            .name("navd-signal".into())
            .spawn(move || {
                if signals.forever().next().is_some() {
                    flag.store(true, Ordering::Release);
                    if sender.send(()).is_err() {
                        eprintln!("navd interrupt receiver closed");
                    }
                }
            })?;
        Ok(Self {
            requested,
            receiver,
            handle,
            worker: Some(worker),
        })
    }

    pub fn check(&self) -> Result<(), Error> {
        if self.requested.load(Ordering::Acquire) {
            Err(Error::Interrupted)
        } else {
            Ok(())
        }
    }

    pub fn wait(&self, duration: Duration) -> Result<(), Error> {
        self.check()?;
        select! {
            recv(self.receiver) -> _ => Err(Error::Interrupted),
            default(duration) => self.check(),
        }
    }
}

impl Drop for Stop {
    fn drop(&mut self) {
        self.handle.close();
        if let Some(worker) = self.worker.take() {
            if worker.join().is_err() {
                eprintln!("navd signal thread panicked");
            }
        }
    }
}
