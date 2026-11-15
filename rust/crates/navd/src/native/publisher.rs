use crate::Error;
use openpilot_messaging::runtime::PubMaster;
use std::{
    sync::mpsc::{self, Receiver, SyncSender},
    thread::{self, JoinHandle},
};

enum Command {
    Message {
        service: &'static str,
        bytes: Vec<u8>,
        done: SyncSender<Result<(), Error>>,
    },
}

pub struct Publisher {
    sender: Option<SyncSender<Command>>,
    worker: Option<JoinHandle<()>>,
}

impl Publisher {
    pub fn new() -> Result<Self, Error> {
        let (sender, receiver) = mpsc::sync_channel(1);
        let (ready, readiness) = mpsc::sync_channel(1);
        let worker = thread::Builder::new()
            .name("navd-publisher".into())
            .spawn(move || {
                let mut publisher =
                    match PubMaster::for_runtime(&["navInstruction", "navRouteNavd"]) {
                        Ok(publisher) => publisher,
                        Err(error) => {
                            if ready.send(Err(Error::from(error))).is_err() {
                                eprintln!("navd publisher startup receiver closed");
                            }
                            return;
                        }
                    };
                if ready.send(Ok(())).is_err() {
                    return;
                }
                run(&mut publisher, receiver);
            })?;
        readiness
            .recv()
            .map_err(|_| Error::Runtime("publisher initialization stopped"))??;
        Ok(Self {
            sender: Some(sender),
            worker: Some(worker),
        })
    }

    fn command(&self, command: Command) -> Result<(), Error> {
        self.sender
            .as_ref()
            .ok_or(Error::Runtime("publisher closed"))?
            .send(command)
            .map_err(|_| Error::Runtime("publisher worker stopped"))
    }

    pub fn send(&self, service: &'static str, bytes: Vec<u8>) -> Result<(), Error> {
        let (done, result) = mpsc::sync_channel(1);
        self.command(Command::Message {
            service,
            bytes,
            done,
        })?;
        result
            .recv()
            .map_err(|_| Error::Runtime("publisher acknowledgement closed"))?
    }
}

impl Drop for Publisher {
    fn drop(&mut self) {
        self.sender.take();
        if let Some(worker) = self.worker.take() {
            if worker.join().is_err() {
                eprintln!("navd publisher thread panicked");
            }
        }
    }
}

fn run(publisher: &mut PubMaster, receiver: Receiver<Command>) {
    // The engine resends the route itself every 5 s (navd.py `resend_at`).
    while let Ok(Command::Message {
        service,
        bytes,
        done,
    }) = receiver.recv()
    {
        let result = publisher.send(service, &bytes).map_err(Error::from);
        if done.send(result).is_err() {
            eprintln!("navd publication receiver closed");
        }
    }
}
