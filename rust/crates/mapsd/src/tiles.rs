//! Hybrid tile storage. Tiles are kept on the device, per remote archive
//! version, so the map keeps working offline; with the network up, a tile is
//! fetched when the remote archive is newer than the stored copy (or, for a
//! version-less source such as Mapbox, when the copy is older than
//! `MAX_AGE`). carrot's mapsd used remote tiles only.
use crate::{renderer::TileSource, Error};
use flate2::{read::GzDecoder, write::GzEncoder, Compression};
use std::{
    fs,
    io::{Read, Write},
    path::{Path, PathBuf},
    time::{Duration, SystemTime},
};

/// Refresh a version-less tile after this long.
pub const MAX_AGE: Duration = Duration::from_secs(30 * 24 * 3600);
/// Bytes kept on disk across versions; oldest files go first.
pub const STORE_LIMIT: u64 = 1 << 30;
/// No remote attempt for this long after a failure (offline, server down).
pub const OFFLINE_BACKOFF: f64 = 30.;

/// A remote archive.
pub trait Remote {
    /// The archive's current version (ETag/build); None for a source without one.
    fn version(&mut self) -> Result<Option<String>, Error>;
    /// The decompressed MVT, or None when the archive has no such tile.
    fn fetch(&mut self, z: u8, x: u64, y: u64) -> Result<Option<Vec<u8>>, Error>;
}

fn sanitize(version: &str) -> String {
    let clean: String = version
        .chars()
        .map(|c| if c.is_ascii_alphanumeric() || c == '-' || c == '_' { c } else { '_' })
        .take(64)
        .collect();
    if clean.is_empty() { "unversioned".into() } else { clean }
}

pub struct HybridTiles<R: Remote> {
    root: PathBuf,
    remote: Option<R>,
    /// The remote version, refreshed hourly.
    version: Option<(String, f64)>,
    offline_until: f64,
    clock: Box<dyn FnMut() -> f64 + Send>,
    written: u64,
}

impl<R: Remote> HybridTiles<R> {
    pub fn new(root: &Path, remote: Option<R>, clock: Box<dyn FnMut() -> f64 + Send>) -> Result<Self, Error> {
        fs::create_dir_all(root)?;
        Ok(Self { root: root.to_owned(), remote, version: None, offline_until: 0., clock, written: 0 })
    }

    fn path(&self, version: &str, z: u8, x: u64, y: u64) -> PathBuf {
        self.root.join(sanitize(version)).join(format!("{z}_{x}_{y}.gz"))
    }

    /// The newest stored copy: (version dir, bytes or None for "no tile", age).
    fn stored(&self, z: u8, x: u64, y: u64) -> Option<(String, Option<Vec<u8>>, Duration)> {
        let mut best: Option<(String, PathBuf, SystemTime)> = None;
        for entry in fs::read_dir(&self.root).ok()?.flatten() {
            let dir = entry.file_name().to_string_lossy().into_owned();
            let path = entry.path().join(format!("{z}_{x}_{y}.gz"));
            if let Ok(modified) = path.metadata().and_then(|m| m.modified()) {
                if best.as_ref().is_none_or(|b| modified > b.2) {
                    best = Some((dir, path, modified));
                }
            }
        }
        let (dir, path, modified) = best?;
        let raw = fs::read(&path).ok()?;
        let age = SystemTime::now().duration_since(modified).unwrap_or_default();
        if raw.is_empty() {
            return Some((dir, None, age));
        }
        let mut out = Vec::new();
        GzDecoder::new(raw.as_slice()).take(16 << 20).read_to_end(&mut out).ok()?;
        Some((dir, Some(out), age))
    }

    fn store(&mut self, version: &str, z: u8, x: u64, y: u64, tile: Option<&[u8]>) -> Result<(), Error> {
        let path = self.path(version, z, x, y);
        if let Some(parent) = path.parent() {
            fs::create_dir_all(parent)?;
        }
        let bytes = match tile {
            None => Vec::new(), // a known gap: do not ask again
            Some(data) => {
                let mut encoder = GzEncoder::new(Vec::new(), Compression::fast());
                encoder.write_all(data)?;
                encoder.finish()?
            }
        };
        let temporary = path.with_extension("tmp");
        fs::write(&temporary, &bytes)?;
        fs::rename(&temporary, &path)?;
        self.written += bytes.len() as u64;
        if self.written > 16 << 20 {
            self.written = 0;
            self.trim()?;
        }
        Ok(())
    }

    /// Oldest files first until the store fits STORE_LIMIT.
    pub fn trim(&self) -> Result<(), Error> {
        let mut files = Vec::new();
        for dir in fs::read_dir(&self.root)?.flatten() {
            for file in fs::read_dir(dir.path()).into_iter().flatten().flatten() {
                if let Ok(meta) = file.metadata() {
                    files.push((meta.modified().unwrap_or(SystemTime::UNIX_EPOCH), meta.len(), file.path()));
                }
            }
        }
        let mut total: u64 = files.iter().map(|f| f.1).sum();
        files.sort();
        for (_, size, path) in files {
            if total <= STORE_LIMIT {
                break;
            }
            if fs::remove_file(&path).is_ok() {
                total -= size;
            }
        }
        for dir in fs::read_dir(&self.root)?.flatten() {
            let _ = fs::remove_dir(dir.path()); // only when empty
        }
        Ok(())
    }

    fn remote_version(&mut self, now: f64) -> Option<String> {
        if let Some((version, checked)) = &self.version {
            if now - checked < 3600. {
                return Some(version.clone());
            }
        }
        let remote = self.remote.as_mut()?;
        match remote.version() {
            Ok(version) => {
                let version = version.unwrap_or_else(|| "unversioned".into());
                self.version = Some((version.clone(), now));
                Some(version)
            }
            Err(_) => {
                self.offline_until = now + OFFLINE_BACKOFF;
                None
            }
        }
    }
}

impl<R: Remote> TileSource for HybridTiles<R> {
    fn tile(&mut self, z: u8, x: u64, y: u64) -> Result<Option<Vec<u8>>, Error> {
        let now = (self.clock)();
        let local = self.stored(z, x, y);
        let online = self.remote.is_some() && now >= self.offline_until;
        let remote_version = if online { self.remote_version(now) } else { None };
        let fresh = match (&local, &remote_version) {
            (None, _) => false,
            (Some(_), None) => true, // offline or no remote: the stored copy is what we have
            (Some((dir, _, age)), Some(version)) => {
                *dir == sanitize(version) && (version != "unversioned" || *age < MAX_AGE)
            }
        };
        if fresh {
            return Ok(local.and_then(|l| l.1));
        }
        if let (Some(version), Some(remote)) = (remote_version, self.remote.as_mut()) {
            match remote.fetch(z, x, y) {
                Ok(tile) => {
                    self.store(&version, z, x, y, tile.as_deref())?;
                    return Ok(tile);
                }
                Err(_) => self.offline_until = now + OFFLINE_BACKOFF,
            }
        }
        match local {
            Some((_, tile, _)) => Ok(tile),
            None => Err(Error::Remote("tile unavailable offline".into())),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::{Arc, Mutex};

    struct Fake {
        version: Arc<Mutex<Option<String>>>,
        online: Arc<Mutex<bool>>,
        fetches: Arc<Mutex<u32>>,
    }
    impl Remote for Fake {
        fn version(&mut self) -> Result<Option<String>, Error> {
            if !*self.online.lock().unwrap() {
                return Err(Error::Remote("offline".into()));
            }
            Ok(self.version.lock().unwrap().clone())
        }
        fn fetch(&mut self, _: u8, x: u64, _: u64) -> Result<Option<Vec<u8>>, Error> {
            if !*self.online.lock().unwrap() {
                return Err(Error::Remote("offline".into()));
            }
            *self.fetches.lock().unwrap() += 1;
            let tag = self.version.lock().unwrap().clone().unwrap_or_default();
            Ok((x != 9).then(|| tag.into_bytes()))
        }
    }

    #[test]
    fn local_first_newer_remote_wins_and_offline_keeps_the_stored_copy() {
        let root = std::env::temp_dir().join(format!("mapsd-tiles-{}", std::process::id()));
        let _ = fs::remove_dir_all(&root);
        let version = Arc::new(Mutex::new(Some("v1".to_string())));
        let online = Arc::new(Mutex::new(true));
        let fetches = Arc::new(Mutex::new(0));
        let now = Arc::new(Mutex::new(0.0f64));
        let clock_now = now.clone();
        let fake = Fake { version: version.clone(), online: online.clone(), fetches: fetches.clone() };
        let mut tiles = HybridTiles::new(&root, Some(fake), Box::new(move || *clock_now.lock().unwrap())).unwrap();
        assert_eq!(tiles.tile(15, 1, 2).unwrap().unwrap(), b"v1");
        assert_eq!(tiles.tile(15, 1, 2).unwrap().unwrap(), b"v1");
        assert_eq!(*fetches.lock().unwrap(), 1, "stored copy reused");
        assert_eq!(tiles.tile(15, 9, 2).unwrap(), None, "a gap is stored too");
        assert_eq!(tiles.tile(15, 9, 2).unwrap(), None);
        assert_eq!(*fetches.lock().unwrap(), 2);
        // A newer archive (after the hourly version check) replaces the tile.
        *version.lock().unwrap() = Some("v2".into());
        *now.lock().unwrap() = 4000.;
        assert_eq!(tiles.tile(15, 1, 2).unwrap().unwrap(), b"v2");
        // Offline: the stored copy is used; an unknown tile is unavailable.
        *online.lock().unwrap() = false;
        *now.lock().unwrap() = 9000.;
        assert_eq!(tiles.tile(15, 1, 2).unwrap().unwrap(), b"v2");
        assert!(tiles.tile(15, 5, 5).is_err());
        let _ = fs::remove_dir_all(&root);
    }
}
