use crate::{runtime::pause, Error};
use rustix::fs::{flock, FlockOperation};
use serialport::SerialPort;
use std::{
    fs::OpenOptions,
    io::{self, Read, Write},
    os::unix::fs::OpenOptionsExt,
    path::PathBuf,
    sync::atomic::{AtomicBool, Ordering},
    time::{Duration, Instant},
};
#[derive(Clone, serde::Serialize, serde::Deserialize)]
pub struct AtPort {
    pub path: PathBuf,
    pub lock: PathBuf,
}
fn unsupported_line_control(error: &serialport::Error) -> bool {
    // serialport's pinned nix adapter retains errno descriptions but not errno numbers.
    matches!(
        error.description.as_str(),
        "Invalid argument" | "Not a typewriter"
    )
}
impl AtPort {
    /// Hold the modem lock and open the port with DTR/RTS raised.
    fn open(&self, stop: &AtomicBool) -> Result<(std::fs::File, serialport::TTYPort), Error> {
        let lock = OpenOptions::new()
            .create(true)
            .truncate(false)
            .read(true)
            .write(true)
            .mode(0o666)
            .open(&self.lock)?;
        loop {
            if stop.load(Ordering::Relaxed) {
                return Err(Error::Stopped);
            }
            match flock(&lock, FlockOperation::NonBlockingLockExclusive) {
                Ok(()) => break,
                Err(rustix::io::Errno::WOULDBLOCK | rustix::io::Errno::INTR) => {
                    pause(Duration::from_millis(20), stop)?
                }
                Err(error) => return Err(error.into()),
            }
        }
        let mut port = serialport::new(self.path.to_string_lossy(), 115200)
            .exclusive(false)
            .timeout(Duration::from_millis(50))
            .preserve_dtr_on_open()
            .open_native()?;
        match port
            .write_data_terminal_ready(true)
            .and_then(|()| port.write_request_to_send(true))
        {
            Ok(()) => (),
            Err(error) if unsupported_line_control(&error) => (),
            Err(error) => return Err(error.into()),
        }
        port.clear(serialport::ClearBuffer::Input)?;
        Ok((lock, port))
    }

    fn exchange(&self, command: &str, stop: &AtomicBool) -> Result<String, Error> {
        let (_lock, mut port) = self.open(stop)?;
        port.write_all(format!("{command}\r").as_bytes())?;
        let mut lines = Vec::new();
        loop {
            let mut raw = Vec::new();
            let mut deadline = Instant::now() + Duration::from_secs(5);
            loop {
                if stop.load(Ordering::Relaxed) {
                    return Err(Error::Stopped);
                }
                let mut byte = [0];
                match port.read(&mut byte) {
                    Ok(0) => break,
                    Ok(_) => {
                        raw.push(byte[0]);
                        if byte[0] == b'\n' {
                            break;
                        }
                        deadline = Instant::now() + Duration::from_secs(5);
                    }
                    Err(error) if error.kind() == io::ErrorKind::TimedOut => {
                        if Instant::now() >= deadline {
                            break;
                        }
                    }
                    Err(error) if error.kind() == io::ErrorKind::Interrupted => (),
                    Err(error) => return Err(error.into()),
                }
            }
            if raw.is_empty() {
                return Err(Error::AtTimeout(command.into()));
            }
            let text = String::from_utf8_lossy(&raw);
            let line =
                text.trim_matches(|c: char| c.is_whitespace() || matches!(c, '\x1c'..='\x1f'));
            if matches!(line, "OK" | "ERROR") || line.starts_with("+CME ERROR") {
                return Ok(lines.join("\n"));
            }
            if !line.is_empty() && line != command {
                lines.push(line.to_owned());
            }
        }
    }
    /// Read until a line starting with one of `ends`, or the deadline.
    fn read_until(
        port: &mut serialport::TTYPort,
        ends: &[&str],
        timeout: Duration,
        stop: &AtomicBool,
    ) -> Result<String, Error> {
        let deadline = Instant::now() + timeout;
        let mut text = String::new();
        let mut buffer = [0; 256];
        while Instant::now() < deadline {
            if stop.load(Ordering::Relaxed) {
                return Err(Error::Stopped);
            }
            match port.read(&mut buffer) {
                Ok(n) => text.push_str(&String::from_utf8_lossy(&buffer[..n])),
                Err(error)
                    if matches!(error.kind(), io::ErrorKind::TimedOut | io::ErrorKind::Interrupted) => {}
                Err(error) => return Err(error.into()),
            }
            if text
                .lines()
                .any(|line| ends.iter().any(|end| line.trim().starts_with(end)))
            {
                return Ok(text);
            }
        }
        Err(Error::AtTimeout(ends.join("|")))
    }

    /// `AT+QFUPL` a file to the modem's UFS. ModemManager is masked by the
    /// modem daemon, so `mmcli --location-inject-assistance-data` cannot work.
    pub fn upload(&self, name: &str, data: &[u8], stop: &AtomicBool) -> Result<(), Error> {
        let (_lock, mut port) = self.open(stop)?;
        port.write_all(format!("AT+QFUPL=\"UFS:{name}\",{},60\r", data.len()).as_bytes())?;
        let reply = Self::read_until(&mut port, &["CONNECT", "ERROR", "+CME ERROR"], Duration::from_secs(5), stop)?;
        if !reply.contains("CONNECT") {
            return Err(Error::Protocol("modem refused the XTRA upload"));
        }
        for chunk in data.chunks(1024) {
            port.write_all(chunk)?;
        }
        port.flush()?;
        let reply = Self::read_until(&mut port, &["OK", "ERROR", "+CME ERROR"], Duration::from_secs(20), stop)?;
        if !reply.contains(&format!("+QFUPL: {},", data.len())) || !reply.lines().any(|l| l.trim() == "OK") {
            return Err(Error::Protocol("XTRA upload incomplete"));
        }
        Ok(())
    }

    pub fn command(&self, command: &str, stop: &AtomicBool) -> Result<String, Error> {
        for attempt in 0..5 {
            match self.exchange(command, stop) {
                Ok(value) => return Ok(value),
                Err(Error::Stopped) => return Err(Error::Stopped),
                Err(error) => {
                    println!("at_cmd failed, trying again");
                    pause(Duration::from_secs(1), stop)?;
                    if attempt == 4 {
                        return Err(error);
                    }
                }
            }
        }
        Err(Error::Protocol("AT retry loop exhausted"))
    }
}
