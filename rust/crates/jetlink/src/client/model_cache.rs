//! Verified Cinque v2 provisioning; no graph execution occurs here.
use crate::{model_spec::ModelSpec, Error};
use sha2::{Digest, Sha256};
use std::{
    fs::{self, File},
    io::{Read, Write},
    path::{Path, PathBuf},
    time::{Duration, Instant},
};
pub const MODEL_URL: &str =
    "https://upload.shind0.synology.me/models/carrot-jetlink-cinque-v2/big_driving_supercombo.onnx";

fn agent() -> ureq::Agent {
    let certificates = rustls_native_certs::load_native_certs()
        .certs
        .iter()
        .map(|cert| ureq::tls::Certificate::from_der(cert.as_ref()).to_owned())
        .collect::<Vec<_>>();
    let tls = ureq::tls::TlsConfig::builder()
        .root_certs(ureq::tls::RootCerts::Specific(certificates.into()))
        .build();
    let config = ureq::Agent::config_builder()
        .tls_config(tls)
        .timeout_connect(Some(Duration::from_secs(10)))
        .timeout_global(None)
        .http_status_as_error(false)
        .accept_encoding(ureq::config::AutoHeaderValue::None)
        .max_redirects(0)
        .build();
    openpilot_http_transport::socket_timeout_agent(config, Duration::from_secs(10))
}

pub(super) fn model_file(
    cache: &Path,
    spec: &ModelSpec,
    check: impl FnMut() -> Result<(), Error>,
    progress: impl FnMut(&str, f64),
) -> Result<PathBuf, Error> {
    model_file_from(cache, spec, MODEL_URL, check, progress)
}

/// The ONNX `spec` names, from `url` unless a verified copy is cached.
pub(super) fn model_file_from(
    cache: &Path,
    spec: &ModelSpec,
    url: &str,
    check: impl FnMut() -> Result<(), Error>,
    progress: impl FnMut(&str, f64),
) -> Result<PathBuf, Error> {
    ensure(cache, spec, check, progress, || {
        let response = agent()
            .get(url)
            .call()
            .map_err(|error| Error::Native(error.to_string()))?;
        if response.status() != 200 {
            return Err(Error::Contract(
                "Jetlink model download HTTP status/redirect",
            ));
        }
        Ok(response.into_parts().1.into_reader())
    })
}

fn ensure<R: Read>(
    cache: &Path,
    spec: &ModelSpec,
    mut check: impl FnMut() -> Result<(), Error>,
    mut progress: impl FnMut(&str, f64),
    open: impl FnOnce() -> Result<R, Error>,
) -> Result<PathBuf, Error> {
    check()?;
    let deadline = Instant::now() + Duration::from_secs(1800);
    fs::create_dir_all(cache)?;
    let target = cache.join(format!("{}.onnx", spec.sha256));
    if target.is_file() && target.metadata()?.len() == spec.nbytes {
        let mut input = File::open(&target)?;
        let mut hash = Sha256::new();
        let mut block = vec![0; 4 << 20];
        loop {
            let count = input.read(&mut block)?;
            if count == 0 {
                break;
            }
            check()?;
            hash.update(&block[..count]);
            progress("verify", 0.0);
        }
        if format!("{:x}", hash.finalize()) == spec.sha256 {
            return Ok(target);
        }
    }
    let space = rustix::fs::statvfs(cache).map_err(std::io::Error::from)?;
    if space.f_bavail.saturating_mul(space.f_frsize) < spec.nbytes.saturating_add(64 << 20) {
        return Err(std::io::Error::new(
            std::io::ErrorKind::StorageFull,
            "not enough space for Jetlink model",
        )
        .into());
    }
    let mut temporary = tempfile::Builder::new()
        .prefix("download-")
        .suffix(".part")
        .tempfile_in(cache)?;
    progress("download", 0.0);
    let mut input = open()?;
    let mut hash = Sha256::new();
    let mut total = 0u64;
    let mut block = vec![0; 1 << 20];
    loop {
        check()?;
        if Instant::now() >= deadline {
            return Err(Error::Deadline);
        }
        let mut count = 0;
        while count < block.len() {
            check()?;
            if Instant::now() >= deadline {
                return Err(Error::Deadline);
            }
            match input.read(&mut block[count..]) {
                Ok(0) => break,
                Ok(read) => count += read,
                Err(error) if error.kind() == std::io::ErrorKind::Interrupted => continue,
                Err(error) => return Err(error.into()),
            }
        }
        if count == 0 {
            break;
        }
        total = total.saturating_add(count as u64);
        if total > spec.nbytes {
            return Err(Error::Contract("oversized Jetlink model download"));
        }
        temporary.write_all(&block[..count])?;
        hash.update(&block[..count]);
        progress("download", total as f64 / spec.nbytes as f64);
    }
    if total != spec.nbytes || format!("{:x}", hash.finalize()) != spec.sha256 {
        return Err(Error::Contract("Jetlink model checksum/size mismatch"));
    }
    check()?;
    temporary.flush()?;
    temporary.as_file().sync_all()?;
    temporary.persist(&target).map_err(|error| error.error)?;
    Ok(target)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::Cursor;
    fn spec(data: &[u8]) -> ModelSpec {
        let mut spec = ModelSpec::parse(crate::model_spec::LEGACY_JSON.as_bytes()).unwrap();
        spec.sha256 = format!("{:x}", Sha256::digest(data));
        spec.nbytes = data.len() as u64;
        spec
    }
    #[test]
    fn verified_cache_skips_download_and_bad_content_is_replaced_atomically() {
        let root = tempfile::tempdir().unwrap();
        let data = b"owned-model-fixture";
        let spec = spec(data);
        let target = root.path().join(format!("{}.onnx", spec.sha256));
        fs::write(&target, data).unwrap();
        assert_eq!(
            ensure(
                root.path(),
                &spec,
                || Ok(()),
                |_, _| {},
                || -> Result<Cursor<Vec<u8>>, Error> { panic!("valid cache downloaded again") }
            )
            .unwrap(),
            target
        );
        fs::write(&target, vec![0; data.len()]).unwrap();
        ensure(
            root.path(),
            &spec,
            || Ok(()),
            |_, _| {},
            || Ok(Cursor::new(data)),
        )
        .unwrap();
        assert_eq!(fs::read(&target).unwrap(), data);
        assert_eq!(fs::read_dir(root.path()).unwrap().count(), 1);
    }
    #[test]
    fn failed_or_cancelled_download_never_replaces_existing_cache() {
        let root = tempfile::tempdir().unwrap();
        let data = b"owned-model-fixture";
        let spec = spec(data);
        let target = root.path().join(format!("{}.onnx", spec.sha256));
        let old = vec![0; data.len()];
        fs::write(&target, &old).unwrap();
        for wrong in [
            b"short".as_slice(),
            b"a much longer invalid model fixture".as_slice(),
        ] {
            assert!(ensure(
                root.path(),
                &spec,
                || Ok(()),
                |_, _| {},
                || Ok(Cursor::new(wrong))
            )
            .is_err());
            assert_eq!(fs::read(&target).unwrap(), old);
            assert_eq!(fs::read_dir(root.path()).unwrap().count(), 1);
        }
        let mut checks = 0;
        assert!(ensure(
            root.path(),
            &spec,
            || {
                checks += 1;
                if checks > 2 {
                    Err(Error::Closed)
                } else {
                    Ok(())
                }
            },
            |_, _| {},
            || Ok(Cursor::new(data))
        )
        .is_err());
        assert_eq!(fs::read(&target).unwrap(), old);
        assert_eq!(fs::read_dir(root.path()).unwrap().count(), 1);
    }
    #[test]
    fn download_identity_matches_signed_public_release_and_source_contract() {
        let release: serde_json::Value =
            serde_json::from_slice(include_bytes!("../../data/host_release.json")).unwrap();
        let spec = ModelSpec::parse(crate::model_spec::LEGACY_JSON.as_bytes()).unwrap();
        assert_eq!(release["model"]["url"], MODEL_URL);
        assert_eq!(release["model"]["sha256"], spec.sha256);
        assert_eq!(release["model"]["size"], spec.nbytes);
    }
}

#[cfg(test)]
mod http_tests {
    use super::*;
    #[test]
    fn native_download_agent_never_follows_model_redirects() {
        use std::{net::TcpListener, thread};
        let server = TcpListener::bind("127.0.0.1:0").unwrap();
        let url = format!("http://{}/model.onnx", server.local_addr().unwrap());
        let reply = thread::spawn(move || {
            let (mut stream, _) = server.accept().unwrap();
            stream
                .set_read_timeout(Some(Duration::from_secs(2)))
                .unwrap();
            let mut request = Vec::new();
            let mut byte = [0];
            while !request.ends_with(b"\r\n\r\n") {
                stream.read_exact(&mut byte).unwrap();
                request.push(byte[0]);
            }
            stream.write_all(b"HTTP/1.1 302 Found\r\nLocation: /redirected.onnx\r\nContent-Length: 0\r\nConnection: close\r\n\r\n").unwrap();
        });
        let response = agent().get(url).call().unwrap();
        reply.join().unwrap();
        assert_eq!(response.status(), 302);
    }
}
