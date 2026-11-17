//! Sequential cable discovery before HELLO; never race USB and TCP greetings.
use super::{cable::MobileCable, config::Mode};
use crate::{
    client::{Client, Transport},
    contract::Identity,
    ffs::{FfsTransport, Session},
    owner::Backend,
    session_contract::SessionContract,
    wire::Version,
    Deadline, Error,
};
use std::{
    net::{Shutdown, TcpStream},
    path::PathBuf,
    sync::{
        atomic::{AtomicBool, Ordering},
        Arc, Mutex,
    },
    time::{Duration, Instant},
};
#[derive(Default)]
pub struct Cancel {
    stopped: AtomicBool,
    socket: Mutex<Option<TcpStream>>,
}
impl Cancel {
    pub fn cancel(&self) -> Result<(), Error> {
        self.stopped.store(true, Ordering::Release);
        if let Some(stream) = self
            .socket
            .lock()
            .map_err(|_| Error::Contract("mobile cancellation poisoned"))?
            .take()
        {
            match stream.shutdown(Shutdown::Both) {
                Ok(()) => {}
                Err(error) if error.kind() == std::io::ErrorKind::NotConnected => {}
                Err(error) => return Err(error.into()),
            }
        }
        Ok(())
    }
}
pub struct MobileBackend {
    owner: Arc<Session>,
    cable: Option<Arc<MobileCable>>,
    mode: Mode,
    admission_mode: Mode,
    version: Version,
    grace: Duration,
    cache: PathBuf,
    pub hello_complete: Arc<AtomicBool>,
    pub selected_tcp: Arc<AtomicBool>,
    pub cancel: Arc<Cancel>,
    client: Option<Client<Box<dyn Transport>>>,
    legacy: bool,
    status: Arc<Mutex<serde_json::Value>>,
}
pub struct Options {
    pub mode: Mode,
    pub admission_mode: Mode,
    pub version: Version,
    pub grace: Duration,
    pub cache: PathBuf,
    pub legacy: bool,
    pub status: Arc<Mutex<serde_json::Value>>,
}
impl MobileBackend {
    pub fn new(owner: Arc<Session>, cable: Option<Arc<MobileCable>>, options: Options) -> Self {
        Self {
            owner,
            cable,
            mode: options.mode,
            admission_mode: options.admission_mode,
            version: options.version,
            grace: options.grace,
            cache: options.cache,
            hello_complete: Arc::new(AtomicBool::new(false)),
            selected_tcp: Arc::new(AtomicBool::new(false)),
            cancel: Arc::new(Cancel::default()),
            client: None,
            legacy: options.legacy,
            status: options.status,
        }
    }
    fn choose(&self) -> Result<Box<dyn Transport>, Error> {
        if matches!(self.mode, Mode::Usb | Mode::Android) {
            return Ok(Box::new(FfsTransport::with_version(
                Arc::clone(&self.owner),
                self.version,
            )));
        }
        let cable = self
            .cable
            .as_ref()
            .ok_or(Error::Contract("composite listener missing"))?;
        let grace = Instant::now() + self.grace;
        loop {
            if self.cancel.stopped.load(Ordering::Acquire) || !self.owner.bound()? {
                return Err(Error::Closed);
            }
            match cable.accept(Deadline::after(Duration::from_secs(1))?) {
                Ok(transport) => {
                    self.selected_tcp.store(true, Ordering::Release);
                    *self
                        .cancel
                        .socket
                        .lock()
                        .map_err(|_| Error::Contract("mobile cancellation poisoned"))? =
                        Some(transport.cancel_handle()?);
                    return Ok(Box::new(transport));
                }
                Err(Error::Deadline) => {}
                Err(error) => return Err(error),
            }
            if self.mode == Mode::Auto && Instant::now() >= grace && self.owner.configured()? {
                return Ok(Box::new(FfsTransport::with_version(
                    Arc::clone(&self.owner),
                    self.version,
                )));
            }
        }
    }
}
impl Backend for MobileBackend {
    fn after_reply(&mut self) -> Result<(), Error> {
        self.client.as_mut().ok_or(Error::Closed)?.after_reply()
    }
    fn idle(&mut self) -> Result<(), Error> {
        self.client.as_mut().ok_or(Error::Closed)?.idle()
    }
    fn session_start(&mut self) -> Result<(), Error> {
        self.client.as_mut().ok_or(Error::Closed)?.session_start()
    }
    fn source_sof(&mut self, source_sof: u64) {
        if let Some(client) = &mut self.client {
            client.set_source_sof(source_sof);
        }
    }
    /// The tablet's own inference, queue and total times, so `runtimeTiming`
    /// (jetlinkd) carries `server_*` for a tablet as it does for a Jetson.
    fn last_timings(&self) -> Option<[f64; 3]> {
        self.client.as_ref().and_then(Backend::last_timings)
    }
    fn connect(&mut self) -> Result<Identity, Error> {
        if self.mode == Mode::Ios && self.version != Version::V3 {
            return Err(Error::Contract("iOS cable requires protocol3"));
        }
        let transport = self.choose()?;
        let tcp = self.selected_tcp.load(Ordering::Acquire);
        let mut client = if self.legacy {
            Client::new(transport)
        } else {
            Client::app(transport, self.cache.clone())
        };
        client.status = Arc::clone(&self.status);
        if !self.legacy {
            client.set_aux(crate::aux::shared());
        }
        let owner = Arc::clone(&self.owner);
        let cancel = Arc::clone(&self.cancel);
        client.set_connected_check(Arc::new(move || {
            !cancel.stopped.load(Ordering::Acquire) && owner.configured().unwrap_or(false)
        }));
        let owner = Arc::clone(&self.owner);
        let mode = self.admission_mode;
        client
            .set_hello_check(move |peer| crate::physical::admit(&owner.speed()?, mode, peer, tcp));
        client.hello_complete = Arc::clone(&self.hello_complete);
        if self.mode == Mode::Auto && !tcp {
            client.set_hello_timeout(Duration::from_secs(3))?;
        }
        self.client = Some(client);
        self.client.as_mut().ok_or(Error::Closed)?.connect()
    }
    fn session_contract(&self) -> &SessionContract {
        if let Some(client) = &self.client {
            client.session_contract()
        } else {
            SessionContract::native_default()
        }
    }
    fn infer(
        &mut self,
        frame: u32,
        warped: &[u8],
        packed: &[f32],
        deadline: Deadline,
        reset: bool,
    ) -> Result<Vec<f32>, Error> {
        self.client
            .as_mut()
            .ok_or(Error::Closed)?
            .infer(frame, warped, packed, deadline, reset)
    }
    fn dead(&self) -> bool {
        self.cancel.stopped.load(Ordering::Acquire)
            || self.client.as_ref().is_some_and(Backend::dead)
    }
    fn close(&mut self) {
        if let Err(error) = self.cancel.cancel() {
            eprintln!("mobile socket cancellation: {error}");
        }
        if let Some(client) = &mut self.client {
            client.close();
        }
    }
}
