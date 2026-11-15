use crate::{
    mapped::WordMapping,
    queue_core::{Read, Reader, Ready, HEADER_BYTES},
    Error,
};
use std::{
    env,
    fs::{File, OpenOptions},
    io::Read as _,
    os::fd::{AsFd, AsRawFd},
    os::unix::fs::OpenOptionsExt,
    path::{Path, PathBuf},
};

#[derive(Clone, Copy)]
pub(crate) enum Namespace {
    Isolated,
    Runtime,
}

#[derive(Clone, Copy)]
pub(crate) enum PublisherMode {
    Exclusive,
    Transient,
    Original,
}

pub(crate) enum Kind {
    Publisher(PublisherMode),
    Subscriber { conflate: bool },
}

enum Role {
    Publisher(u64),
    Subscriber(Reader),
}

pub(crate) struct Queue {
    mapping: WordMapping,
    _publisher_lock: Option<File>,
    role: Role,
    capacity: usize,
}

pub(crate) fn component(value: &str, maximum: usize, empty: bool) -> bool {
    (empty || !value.is_empty())
        && value.len() <= maximum
        && value
            .bytes()
            .all(|byte| byte.is_ascii_alphanumeric() || byte == b'_' || byte == b'-')
}

pub(crate) fn prefix() -> Result<Option<String>, Error> {
    if env::var_os("CEREAL_FAKE").is_some() {
        return Err(Error::Invalid("CEREAL_FAKE is unsupported"));
    }
    env::var_os("OPENPILOT_PREFIX")
        .map(|prefix| {
            prefix
                .into_string()
                .map_err(|_| Error::Invalid("invalid runtime namespace"))
        })
        .transpose()
}

fn path(endpoint: &str, namespace: Namespace) -> Result<PathBuf, Error> {
    let prefix = prefix()?;
    let value = prefix.as_deref().unwrap_or_default();
    if matches!(namespace, Namespace::Isolated)
        && (!component(value, 100, false) || !value.starts_with("rust-probe-") || value.len() <= 11)
    {
        return Err(Error::Invalid(
            "OPENPILOT_PREFIX must be an isolated rust-probe-NAME namespace",
        ));
    }
    if !component(value, 100, true) {
        return Err(Error::Invalid("invalid runtime namespace"));
    }
    if !component(endpoint, 100, false) {
        return Err(Error::Invalid("invalid endpoint"));
    }
    Ok(PathBuf::from(match prefix {
        Some(prefix) => format!("/dev/shm/msgq_{prefix}/{endpoint}"),
        None => format!("/dev/shm/msgq_{endpoint}"),
    }))
}

fn publisher_lock(path: &Path) -> Result<File, Error> {
    let mut lock_path = path.as_os_str().to_owned();
    lock_path.push(".rust-publisher-lock");
    let file = OpenOptions::new()
        .read(true)
        .write(true)
        .create(true)
        .truncate(false)
        .mode(0o600)
        .open(lock_path)
        .map_err(|error| Error::Io("open publisher lock", error))?;
    // SAFETY: file owns this live FD; flock has no pointer argument and the
    // exclusive nonblocking lock is retained by keeping file alive in Queue.
    if unsafe { libc::flock(file.as_raw_fd(), libc::LOCK_EX | libc::LOCK_NB) } != 0 {
        return Err(Error::last("another Rust publisher owns this endpoint"));
    }
    Ok(file)
}

extern "C" fn notify_signal(_: libc::c_int) {}

fn install_signal_handler() -> Result<(), Error> {
    // SAFETY: the C-ABI handler never unwinds or accesses non-signal-safe state.
    // signal installs the same process-wide no-op SIGUSR2 handler as original msgq.
    if unsafe {
        libc::signal(
            libc::SIGUSR2,
            notify_signal as *const () as libc::sighandler_t,
        )
    } == libc::SIG_ERR
    {
        return Err(Error::last("install msgq signal handler"));
    }
    Ok(())
}

fn notify_reader(tid: u32) {
    // SAFETY: tkill takes scalar arguments; stale/empty reader IDs are expected
    // and its failure is intentionally ignored, matching original notification semantics.
    unsafe { libc::syscall(libc::SYS_tkill, libc::c_long::from(tid), libc::SIGUSR2) };
}

fn uid() -> Result<u64, Error> {
    let mut random = [0; 4];
    File::open("/dev/urandom")
        .and_then(|mut file| file.read_exact(&mut random))
        .map_err(|error| Error::Io("read msgq registration entropy", error))?;
    // SAFETY: gettid has no pointer arguments and identifies the calling thread.
    let tid = unsafe { libc::syscall(libc::SYS_gettid) };
    let tid = u32::try_from(tid).map_err(|_| Error::last("read thread ID"))?;
    Ok((u64::from(u32::from_ne_bytes(random)) << 32) | u64::from(tid))
}

fn validate_file(file: &File, length: u64) -> Result<(), Error> {
    let metadata = file
        .metadata()
        .map_err(|error| Error::Io("inspect msgq queue", error))?;
    if !metadata.is_file() || (metadata.len() != 0 && metadata.len() != length) {
        return Err(Error::Invalid(
            "existing msgq queue has incompatible size or type",
        ));
    }
    Ok(())
}

impl Queue {
    pub(crate) fn open(
        endpoint: &str,
        kind: Kind,
        capacity: usize,
        namespace: Namespace,
    ) -> Result<Self, Error> {
        let path = path(endpoint, namespace)?;
        if !(4096..=64 * 1024 * 1024).contains(&capacity) || !capacity.is_multiple_of(8) {
            return Err(Error::Invalid(
                "queue capacity must be 4096..67108864 bytes and aligned to 8 bytes",
            ));
        }
        let length = u64::try_from(capacity + HEADER_BYTES)
            .map_err(|_| Error::Invalid("queue mapping length overflow"))?;
        match std::fs::metadata(&path) {
            Ok(metadata) => {
                if !metadata.is_file() || (metadata.len() != 0 && metadata.len() != length) {
                    return Err(Error::Invalid(
                        "existing msgq queue has incompatible size or type",
                    ));
                }
            }
            Err(error) if error.kind() == std::io::ErrorKind::NotFound => {}
            Err(error) => return Err(Error::Io("inspect msgq", error)),
        }
        let mut lock = match kind {
            Kind::Publisher(PublisherMode::Exclusive | PublisherMode::Transient) => {
                Some(publisher_lock(&path)?)
            }
            Kind::Publisher(PublisherMode::Original) | Kind::Subscriber { .. } => None,
        };
        install_signal_handler()?;
        let file = OpenOptions::new()
            .read(true)
            .write(true)
            .create(true)
            .truncate(false)
            .mode(0o664)
            .open(&path)
            .map_err(|error| Error::Io("open msgq", error))?;
        validate_file(&file, length)?;
        file.set_len(length)
            .map_err(|error| Error::Io("size msgq queue", error))?;
        let mapping = WordMapping::new(
            file.as_fd(),
            capacity + HEADER_BYTES,
            env::var_os("MSGQ_PREALLOC").is_some(),
        )?;
        let uid = uid()?;
        let role = match kind {
            Kind::Publisher(mode) => {
                mapping.memory()?.initialize_publisher(uid);
                if matches!(mode, PublisherMode::Transient) {
                    drop(lock.take());
                }
                Role::Publisher(uid)
            }
            Kind::Subscriber { conflate } => {
                Role::Subscriber(mapping.memory()?.register(uid, conflate, notify_reader)?)
            }
        };
        Ok(Self {
            mapping,
            _publisher_lock: lock,
            role,
            capacity,
        })
    }

    pub(crate) fn send(&mut self, bytes: &[u8]) -> Result<(), Error> {
        let Role::Publisher(uid) = self.role else {
            return Err(Error::Invalid("send requires a publisher"));
        };
        if bytes.is_empty() || bytes.len() > self.capacity / 3 - 16 {
            return Err(Error::Invalid(
                "send requires a nonempty payload fitting one third of the queue",
            ));
        }
        self.mapping.memory()?.send(uid, bytes, notify_reader)
    }

    pub(crate) fn readers_caught_up(&self) -> bool {
        if !matches!(self.role, Role::Publisher(_)) {
            return false;
        }
        match self
            .mapping
            .memory()
            .and_then(|memory| memory.readers_caught_up())
        {
            Ok(caught_up) => caught_up,
            Err(error) => {
                eprintln!("{error}");
                false
            }
        }
    }

    fn reregister(&mut self, reader: Reader) -> Result<(), Error> {
        self.role = Role::Subscriber(self.mapping.memory()?.register(
            uid()?,
            reader.conflated(),
            notify_reader,
        )?);
        Ok(())
    }

    pub(crate) fn ready(&mut self) -> Result<bool, Error> {
        loop {
            let Role::Subscriber(reader) = self.role else {
                return Err(Error::Invalid("receive requires a subscriber"));
            };
            match reader.ready(&self.mapping.memory()?)? {
                Ready::Empty => return Ok(false),
                Ready::Message => return Ok(true),
                Ready::Evicted => self.reregister(reader)?,
            }
        }
    }

    fn receive_now(&mut self) -> Result<Option<Vec<u8>>, Error> {
        loop {
            let Role::Subscriber(reader) = self.role else {
                return Err(Error::Invalid("receive requires a subscriber"));
            };
            match reader.receive(&self.mapping.memory()?)? {
                Read::Empty => return Ok(None),
                Read::Message(bytes) => return Ok(Some(bytes)),
                Read::Evicted => self.reregister(reader)?,
            }
        }
    }

    pub(crate) fn receive_now_into(&mut self, bytes: &mut Vec<u8>) -> Result<bool, Error> {
        loop {
            let Role::Subscriber(reader) = self.role else {
                return Err(Error::Invalid("receive requires a subscriber"));
            };
            match reader.receive_into(&self.mapping.memory()?, bytes)? {
                Ready::Empty => return Ok(false),
                Ready::Message => return Ok(true),
                Ready::Evicted => self.reregister(reader)?,
            }
        }
    }

    pub(crate) fn receive(&mut self, timeout_ms: i32) -> Result<Option<Vec<u8>>, Error> {
        if timeout_ms < 0 {
            return Err(Error::Invalid("invalid receive timeout"));
        }
        let first = self.receive_now()?;
        if first.is_some() || timeout_ms == 0 {
            return Ok(first);
        }
        poll(timeout_ms, || self.ready())?;
        self.receive_now()
    }
}

pub(crate) use crate::notification_wait::{poll, poll_not_before};

#[cfg(test)]
mod tests {
    #[test]
    fn notification_after_empty_check_does_not_wait_for_timeout() {
        super::install_signal_handler().unwrap();
        let tid = u32::try_from(super::uid().unwrap() & u64::from(u32::MAX)).unwrap();
        let started = std::time::Instant::now();
        let mut checks = 0;
        super::poll(200, || {
            checks += 1;
            if checks == 1 {
                super::notify_reader(tid);
                Ok(false)
            } else {
                Ok(true)
            }
        })
        .unwrap();
        assert!(started.elapsed() < std::time::Duration::from_millis(100));
    }

    #[test]
    fn infinite_poll_keeps_waiting_after_a_completed_sleep_interval() {
        let mut checks = 0;
        super::poll(-1, || {
            checks += 1;
            Ok(checks == 3)
        })
        .unwrap();
        assert_eq!(checks, 3);
        assert!(super::poll(-2, || Ok(false)).is_err());
    }
}
