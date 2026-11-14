use crate::helpers;
use std::net::UdpSocket;

/// SIOCGIFADDR for one interface (formerly the C++ shim `native/address_io.cc`).
#[allow(unsafe_code)]
fn interface_address(name: &[u8]) -> std::io::Result<String> {
    // SAFETY: plain socket syscall; the descriptor is owned below.
    let fd = unsafe { libc::socket(libc::AF_INET, libc::SOCK_DGRAM | libc::SOCK_CLOEXEC, 0) };
    if fd < 0 {
        return Err(std::io::Error::last_os_error());
    }
    // SAFETY: `fd` was just returned by socket and is closed exactly once.
    let socket = unsafe { <std::os::fd::OwnedFd as std::os::fd::FromRawFd>::from_raw_fd(fd) };
    // SAFETY: ifreq is plain data; all-zero is a valid empty request.
    let mut request: libc::ifreq = unsafe { std::mem::zeroed() };
    for (target, &byte) in request
        .ifr_name
        .iter_mut()
        .zip(name.iter().take(libc::IFNAMSIZ - 1))
    {
        *target = byte as libc::c_char;
    }
    // SAFETY: `request` is a live ifreq the kernel fills for this call.
    let result = unsafe {
        libc::ioctl(
            std::os::fd::AsRawFd::as_raw_fd(&socket),
            libc::SIOCGIFADDR,
            &mut request,
        )
    };
    if result < 0 {
        return Err(std::io::Error::last_os_error());
    }
    // SAFETY: SIOCGIFADDR filled ifru_addr; sa_data[2..6] holds the IPv4 address.
    let data = unsafe { request.ifr_ifru.ifru_addr.sa_data };
    Ok(std::net::Ipv4Addr::new(data[2] as u8, data[3] as u8, data[4] as u8, data[5] as u8).to_string())
}
pub fn interface_ipv4(name: &str) -> String {
    let name: String = name.chars().take(15).collect();
    interface_address(name.as_bytes())
        .map(|value| helpers::usable_ip(&value))
        .unwrap_or_default()
}
pub fn route_ipv4() -> String {
    let address = (|| -> std::io::Result<_> {
        let socket = UdpSocket::bind("0.0.0.0:0")?;
        socket.set_read_timeout(Some(std::time::Duration::from_millis(500)))?;
        socket.connect("8.8.8.8:80")?;
        Ok(socket.local_addr()?.ip().to_string())
    })();
    address
        .map(|value| helpers::usable_ip(&value))
        .unwrap_or_default()
}
pub fn local_ip(iface: &str) -> String {
    let address = interface_ipv4(iface);
    if address.is_empty() {
        route_ipv4()
    } else {
        address
    }
}

#[cfg(test)]
mod tests {
    #[test]
    fn reads_an_interface_address_and_rejects_a_missing_one() {
        assert_eq!(super::interface_address(b"lo").unwrap(), "127.0.0.1");
        assert!(super::interface_address(b"no-such-if0").is_err());
    }
}
