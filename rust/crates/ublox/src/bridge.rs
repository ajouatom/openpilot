//! Serial modem-line control (formerly a C++ shim, `native/serial.cc`).
use std::{io, os::fd::RawFd};

/// Raise DTR and RTS like pyserial's open; a port without modem lines
/// (EINVAL/ENOTTY) is accepted.
pub(crate) fn raise_modem_lines(fd: RawFd) -> io::Result<()> {
    for flag in [libc::TIOCM_DTR, libc::TIOCM_RTS] {
        // SAFETY: the caller owns the live descriptor; ioctl reads one c_int.
        if unsafe { libc::ioctl(fd, libc::TIOCMBIS, &flag as *const libc::c_int) } < 0 {
            let error = io::Error::last_os_error();
            if matches!(error.raw_os_error(), Some(libc::EINVAL | libc::ENOTTY)) {
                return Ok(());
            }
            return Err(error);
        }
    }
    Ok(())
}
