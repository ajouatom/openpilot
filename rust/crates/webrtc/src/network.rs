use crate::Error;
mod dns;
mod mdns;
mod mdns_socket;
#[cfg(test)]
mod mdns_tests;
mod remote;
use md5::{Digest, Md5};
pub(crate) use mdns::Lease;
pub(crate) use remote::Remote;
use rtc::ice::candidate::candidate_server_reflexive::CandidateServerReflexiveConfig;
use rtc::peer_connection::transport::{
    CandidateConfig, CandidateHostConfig, RTCIceCandidate, RTCIceCandidateInit,
};
use rtc::stun::{
    message::Getter,
    message::{Message, TransactionId, BINDING_REQUEST, BINDING_SUCCESS},
    xoraddr::XorMappedAddress,
};
use std::{
    net::{IpAddr, Ipv4Addr, SocketAddr, UdpSocket},
    sync::Arc,
    time::Duration,
};
use tokio::{
    task::JoinSet,
    time::{timeout, Instant},
};

#[derive(Clone)]
pub struct Network {
    addresses: Option<Vec<IpAddr>>,
    stun: Option<(String, u16)>,
    mdns: Arc<mdns::Resolver>,
}

pub(crate) struct Gathered {
    pub sockets: Vec<UdpSocket>,
    pub candidates: Vec<RTCIceCandidateInit>,
}

fn interfaces() -> Result<Vec<IpAddr>, Error> {
    let mut addresses = Vec::new();
    for interface in nix::ifaddrs::getifaddrs()? {
        let Some(address) = interface.address else {
            continue;
        };
        if let Some(address) = address.as_sockaddr_in() {
            let ip = address.ip();
            if ip != Ipv4Addr::LOCALHOST {
                addresses.push(IpAddr::V4(ip));
            }
        } else if let Some(address) = address.as_sockaddr_in6() {
            let ip = address.ip();
            if !ip.is_loopback() && address.scope_id() == 0 {
                addresses.push(IpAddr::V6(ip));
            }
        }
    }
    Ok(addresses)
}

fn base(kind: &str, address: SocketAddr, local: IpAddr) -> CandidateConfig {
    CandidateConfig {
        foundation: format!(
            "{:x}",
            Md5::digest(format!("{kind}|udp|{local}").as_bytes())
        ),
        network: "udp".to_owned(),
        address: address.ip().to_string(),
        port: address.port(),
        component: 1,
        priority: (if kind == "host" { 126_u32 } else { 100_u32 } << 24) + (65_535 << 8) + 255,
        ..Default::default()
    }
}

/// How long one answer's STUN step may take, the server's lookup apart.
const STUN_TIMEOUT: Duration = Duration::from_secs(2);

/// The address the kernel sends to `recipient` from: the only local address
/// whose STUN answer can come back. One bound to Jetlink's USB network
/// (192.168.60.1) or tailscale's (100.x) waited out the whole timeout, and two
/// of them made an answer take 10 s, past carrot-server's limit: Carrot
/// Vision kept "connecting" whenever an external model had the USB link up.
fn route_source(recipient: SocketAddr) -> Option<IpAddr> {
    let probe = UdpSocket::bind(SocketAddr::new(IpAddr::V4(Ipv4Addr::UNSPECIFIED), 0)).ok()?;
    probe.connect(recipient).ok()?;
    probe.local_addr().ok().map(|address| address.ip())
}

async fn binding(socket: UdpSocket, recipient: SocketAddr) -> Result<RTCIceCandidateInit, Error> {
    let local = socket.local_addr()?;
    let socket = tokio::net::UdpSocket::from_std(socket)?;
    let mut request = Message::new();
    request.build(&[Box::new(BINDING_REQUEST), Box::new(TransactionId::new())])?;
    let mut delay = Duration::from_millis(500);
    let mut buffer = vec![0_u8; 65_536].into_boxed_slice();
    loop {
        socket.send_to(&request.raw, recipient).await?;
        let deadline = Instant::now() + delay;
        loop {
            let Ok(result) = tokio::time::timeout_at(deadline, socket.recv_from(&mut buffer)).await
            else {
                break;
            };
            let (length, _) = result?;
            let mut response = Message::new();
            if response.unmarshal_binary(&buffer[..length]).is_err()
                || response.transaction_id != request.transaction_id
            {
                continue;
            }
            if response.typ != BINDING_SUCCESS {
                return Err(Error::Contract("STUN binding rejected"));
            }
            let mut mapped = XorMappedAddress::default();
            mapped.get_from(&response)?;
            let candidate = CandidateServerReflexiveConfig {
                base_config: base("srflx", SocketAddr::new(mapped.ip, mapped.port), local.ip()),
                rel_addr: local.ip().to_string(),
                rel_port: local.port(),
                ..Default::default()
            }
            .new_candidate_server_reflexive()?;
            return Ok(RTCIceCandidate::from(&candidate).to_json()?);
        }
        delay = delay.saturating_mul(2);
    }
}

impl Network {
    pub(crate) fn close(&self) -> Result<(), Error> {
        self.mdns.close()
    }
    #[must_use]
    pub fn for_runtime() -> Self {
        Self {
            addresses: None,
            stun: Some(("stun.l.google.com".to_owned(), 19302)),
            mdns: Arc::new(mdns::Resolver::new(None)),
        }
    }

    #[must_use]
    pub fn owned(addresses: Vec<IpAddr>, stun: Option<(String, u16)>) -> Self {
        Self {
            addresses: Some(addresses),
            stun,
            mdns: Arc::new(mdns::Resolver::new(None)),
        }
    }

    /// Uses an owned loopback recipient for the original mDNS query/answer boundary.
    ///
    /// # Errors
    /// Rejects recipients outside the owned loopback interface.
    pub fn owned_mdns(mut self, recipient: SocketAddr) -> Result<Self, Error> {
        if !recipient.ip().is_loopback() || !recipient.is_ipv4() {
            return Err(Error::Contract(
                "mDNS control recipient must be IPv4 loopback",
            ));
        }
        self.mdns = Arc::new(mdns::Resolver::new(Some(recipient)));
        Ok(self)
    }

    pub(crate) async fn gather(&self) -> Result<Gathered, Error> {
        let addresses = match &self.addresses {
            Some(addresses) => addresses.clone(),
            None => interfaces()?,
        };
        let mut result = Gathered {
            sockets: Vec::new(),
            candidates: Vec::new(),
        };
        let mut requests = JoinSet::new();
        // The STUN server is looked up once, and asked only from the address
        // the default route uses.
        let stun = match &self.stun {
            Some(server) => match timeout(
                STUN_TIMEOUT,
                tokio::net::lookup_host((server.0.as_str(), server.1)),
            )
            .await
            {
                Ok(Ok(mut found)) => found
                    .find(SocketAddr::is_ipv4)
                    .and_then(|recipient| Some((recipient, route_source(recipient)?))),
                _ => {
                    eprintln!("WebRTC STUN server not resolved; host candidates only");
                    None
                }
            },
            None => None,
        };
        for address in addresses {
            let socket = match UdpSocket::bind(SocketAddr::new(address, 0)) {
                Ok(socket) => socket,
                Err(error) => {
                    eprintln!("WebRTC could not bind {address}: {error}");
                    continue;
                }
            };
            socket.set_nonblocking(true)?;
            if let Err(error) =
                nix::sys::socket::setsockopt(&socket, nix::sys::socket::sockopt::RcvBuf, &262_144)
            {
                eprintln!("WebRTC could not configure {address}: {error}");
                continue;
            }
            let local = socket.local_addr()?;
            let candidate = CandidateHostConfig {
                base_config: base("host", local, local.ip()),
                ..Default::default()
            }
            .new_candidate_host()?;
            result
                .candidates
                .push(RTCIceCandidate::from(&candidate).to_json()?);
            if let Some((recipient, source)) = stun {
                if local.ip() == source {
                    let cloned = socket.try_clone()?;
                    requests.spawn(async move {
                        timeout(STUN_TIMEOUT, binding(cloned, recipient)).await
                    });
                }
            }
            result.sockets.push(socket);
        }
        while let Some(result_of_request) = requests.join_next().await {
            match result_of_request {
                Ok(Ok(Ok(candidate))) => result.candidates.push(candidate),
                Ok(Ok(Err(error))) => eprintln!("WebRTC STUN gathering failed: {error}"),
                Ok(Err(error)) => eprintln!("WebRTC STUN gathering timed out: {error}"),
                Err(error) => eprintln!("WebRTC STUN gathering task failed: {error}"),
            }
        }
        Ok(result)
    }
}
