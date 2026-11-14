use crate::Error;
use rustix::{
    fs::{fcntl_setfl, open, Mode, OFlags},
    termios::{
        self, ControlModes as C, InputModes as I, LocalModes as L, OptionalActions,
        OutputModes as O, QueueSelector, SpecialCodeIndex,
    },
};
use std::{
    fs::File,
    io::{Read, Write},
    os::fd::AsRawFd,
    path::Path,
};

pub struct Serial {
    file: File,
}
impl Serial {
    pub fn open(path: &Path) -> Result<Self, Error> {
        let fd = open(
            path,
            OFlags::RDWR | OFlags::NOCTTY | OFlags::NONBLOCK | OFlags::CLOEXEC,
            Mode::empty(),
        )
        .map_err(std::io::Error::from)?;
        let mut port = Self {
            file: File::from(fd),
        };
        port.baud(9600)?;
        crate::bridge::raise_modem_lines(port.file.as_raw_fd())?;
        termios::tcflush(&port.file, QueueSelector::IFlush).map_err(std::io::Error::from)?;
        Ok(port)
    }
    pub fn baud(&mut self, baud: u32) -> Result<(), Error> {
        let mut attributes = termios::tcgetattr(&self.file).map_err(std::io::Error::from)?;
        attributes.control_modes.insert(C::CLOCAL | C::CREAD);
        attributes.local_modes.remove(
            L::ICANON
                | L::ECHO
                | L::ECHOE
                | L::ECHOK
                | L::ECHONL
                | L::ISIG
                | L::IEXTEN
                | L::ECHOCTL
                | L::ECHOKE,
        );
        attributes
            .output_modes
            .remove(O::OPOST | O::ONLCR | O::OCRNL);
        attributes.input_modes.remove(
            I::INLCR
                | I::IGNCR
                | I::ICRNL
                | I::IGNBRK
                | I::IUCLC
                | I::PARMRK
                | I::INPCK
                | I::ISTRIP
                | I::IXON
                | I::IXOFF
                | I::IXANY,
        );
        attributes
            .control_modes
            .remove(C::CSIZE | C::CSTOPB | C::PARENB | C::PARODD | C::CMSPAR | C::CRTSCTS);
        attributes.control_modes.insert(C::CS8);
        attributes
            .set_output_speed(baud)
            .map_err(std::io::Error::from)?;
        attributes.special_codes[SpecialCodeIndex::VMIN] = 0;
        attributes.special_codes[SpecialCodeIndex::VTIME] = 0;
        termios::tcsetattr(&self.file, OptionalActions::Now, &attributes)
            .map_err(std::io::Error::from)?;
        fcntl_setfl(&self.file, OFlags::empty()).map_err(std::io::Error::from)?;
        Ok(())
    }
    pub fn send(&mut self, bytes: &[u8]) -> Result<(), Error> {
        self.file.write_all(bytes)?;
        Ok(())
    }
    pub fn receive(&mut self) -> Result<Vec<u8>, Error> {
        let mut data = Vec::new();
        while data.len() < 0x1000 {
            let mut buffer = [0; 0x40];
            let mut size = 0;
            while size < buffer.len() {
                match self.file.read(&mut buffer[size..]) {
                    Ok(0) => break,
                    Ok(count) => size += count,
                    Err(error) if error.kind() == std::io::ErrorKind::Interrupted => continue,
                    Err(error) => return Err(error.into()),
                }
            }
            data.extend_from_slice(&buffer[..size]);
            if size == 0 {
                break;
            }
        }
        Ok(data)
    }
}
