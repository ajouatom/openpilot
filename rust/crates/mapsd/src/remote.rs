//! Remote archives for `HybridTiles`: Mapbox Streets v8 tiles when a token
//! is set (as carrot), otherwise a PMTiles archive at a configured URL read
//! by byte ranges. carrot's other remote, comma's tile API, would send the
//! dongle to comma's servers and is not used.
use crate::{
    pmtiles::{PmTiles, Source},
    tiles::Remote,
    Error,
};
use std::{io::Read, time::Duration};

fn agent(timeout: Duration) -> ureq::Agent {
    ureq::Agent::config_builder()
        .http_status_as_error(false)
        .timeout_global(Some(timeout))
        .max_redirects(5)
        .build()
        .into()
}

fn read_body(response: &mut ureq::http::Response<ureq::Body>, limit: u64) -> Result<Vec<u8>, Error> {
    let mut out = Vec::new();
    response
        .body_mut()
        .as_reader()
        .take(limit + 1)
        .read_to_end(&mut out)
        .map_err(|e| Error::Remote(e.kind().to_string()))?;
    if out.len() as u64 > limit {
        return Err(Error::Remote("response exceeds its limit".into()));
    }
    Ok(out)
}

pub struct Mapbox {
    token: String,
    agent: ureq::Agent,
}

impl Mapbox {
    pub fn new(token: String) -> Self {
        Self { token, agent: agent(Duration::from_secs(8)) }
    }
}

impl Remote for Mapbox {
    fn version(&mut self) -> Result<Option<String>, Error> {
        Ok(None)
    }
    fn fetch(&mut self, z: u8, x: u64, y: u64) -> Result<Option<Vec<u8>>, Error> {
        let url = format!("https://api.mapbox.com/v4/mapbox.mapbox-streets-v8/{z}/{x}/{y}.mvt");
        // The token is a query parameter: keep the URL out of errors and logs.
        let mut response = self
            .agent
            .get(&url)
            .query("access_token", &self.token)
            .call()
            .map_err(|_| Error::Remote("Mapbox tiles unavailable".into()))?;
        match response.status().as_u16() {
            200 => Ok(Some(read_body(&mut response, 1 << 20)?)),
            404 => Ok(None),
            status => Err(Error::Remote(format!("Mapbox tiles answered {status}"))),
        }
    }
}

/// A PMTiles archive read by HTTP range requests.
pub struct HttpRange {
    url: String,
    size: u64,
    etag: String,
    agent: ureq::Agent,
}

impl HttpRange {
    pub fn open(url: &str) -> Result<Self, Error> {
        let mut source = Self { url: url.to_owned(), size: 0, etag: String::new(), agent: agent(Duration::from_secs(10)) };
        source.range(0, 127)?;
        Ok(source)
    }

    fn range(&mut self, offset: u64, length: u64) -> Result<Vec<u8>, Error> {
        let mut request = self
            .agent
            .get(&self.url)
            .header("Range", &format!("bytes={offset}-{}", offset + length - 1))
            .header("Accept-Encoding", "identity");
        if !self.etag.is_empty() {
            request = request.header("If-Match", &self.etag);
        }
        let mut response = request.call().map_err(|_| Error::Remote("map archive unavailable".into()))?;
        if response.status().as_u16() != 206 {
            return Err(Error::Remote(format!("map archive range answered {}", response.status().as_u16())));
        }
        let header = |name: &str| response.headers().get(name).and_then(|v| v.to_str().ok()).map(str::to_owned);
        let total = header("Content-Range")
            .and_then(|v| v.rsplit('/').next().and_then(|t| t.parse::<u64>().ok()))
            .ok_or(Error::Remote("map archive range without a size".into()))?;
        let etag = header("ETag").unwrap_or_default();
        if (self.size != 0 && total != self.size) || (!self.etag.is_empty() && etag != self.etag) {
            return Err(Error::Remote("map archive changed during read".into()));
        }
        self.size = total;
        self.etag = etag;
        let data = read_body(&mut response, length)?;
        if data.len() as u64 != length {
            return Err(Error::Remote("truncated map archive range".into()));
        }
        Ok(data)
    }
}

impl Source for HttpRange {
    fn size(&self) -> u64 {
        self.size
    }
    fn read(&mut self, offset: u64, length: u64) -> Result<Vec<u8>, Error> {
        if length == 0 {
            return Ok(Vec::new());
        }
        self.range(offset, length)
    }
}

pub struct Archive {
    url: String,
    archive: Option<PmTiles<HttpRange>>,
}

impl Archive {
    pub fn new(url: String) -> Self {
        Self { url, archive: None }
    }
    fn open(&mut self) -> Result<&mut PmTiles<HttpRange>, Error> {
        if self.archive.is_none() {
            self.archive = Some(PmTiles::open(HttpRange::open(&self.url)?)?);
        }
        Ok(self.archive.as_mut().expect("opened"))
    }
}

impl Remote for Archive {
    /// A new ETag (a newer build at the same URL) is a new version.
    fn version(&mut self) -> Result<Option<String>, Error> {
        // Reopen to learn the current ETag; directories are re-read for it.
        self.archive = None;
        let archive = self.open()?;
        Ok(Some(archive.source.etag_or_size()))
    }
    fn fetch(&mut self, z: u8, x: u64, y: u64) -> Result<Option<Vec<u8>>, Error> {
        let result = self.open()?.get(z, x, y);
        if result.is_err() {
            self.archive = None;
        }
        result
    }
}

impl HttpRange {
    fn etag_or_size(&self) -> String {
        if self.etag.is_empty() { format!("size{}", self.size) } else { self.etag.clone() }
    }
}
