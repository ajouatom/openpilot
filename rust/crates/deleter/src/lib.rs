//! Log-space retention policy from system/loggerd/deleter.py.
mod names;
pub mod platform;
pub use names::sort_key as directory_sort_key;

use std::{
    collections::HashSet,
    ffi::OsString,
    fs, io,
    path::{Path, PathBuf},
    time::Duration,
};

pub const MIN_BYTES: u128 = 5 * 1024 * 1024 * 1024;
pub const MIN_PERCENT: f64 = 30.0;

/// Full-resolution camera files. A minute with RecordRoadCam 2 is about 150 MB
/// of these against about 15 MB of rlog, qlog and qcamera, so dropping them
/// first keeps ten times more drives that can still be analysed.
pub const VIDEO_FILES: [&str; 3] = ["fcamera.hevc", "ecamera.hevc", "dcamera.hevc"];
/// Segments kept without their videos stay below this many bytes and below half
/// of the log root, so recent full-resolution video still has its own share.
pub const STRIPPED_BUDGET_BYTES: u128 = 3 * 1024 * 1024 * 1024;

/// Free bytes under which the deleter starts removing logs on a filesystem of
/// `total_bytes` (the same test as `cycle`: below MIN_BYTES or MIN_PERCENT).
pub fn free_floor(total_bytes: u128) -> u128 {
    let percent = (total_bytes as f64 * MIN_PERCENT / 100.0).ceil() as u128;
    MIN_BYTES.max(percent)
}

#[derive(Default)]
struct Usage {
    bytes: u128,
    video: bool,
    locked: bool,
}

fn usage(path: &Path) -> io::Result<Usage> {
    let mut usage = Usage::default();
    for entry in fs::read_dir(path)? {
        let entry = entry?;
        let name = entry.file_name();
        let name = name.as_encoded_bytes();
        if name.ends_with(b".lock") {
            usage.locked = true;
        }
        if VIDEO_FILES.iter().any(|video| name == video.as_bytes()) {
            usage.video = true;
        }
        let metadata = entry.metadata()?;
        if metadata.is_file() {
            usage.bytes += u128::from(metadata.len());
        }
    }
    Ok(usage)
}

#[derive(Debug, thiserror::Error)]
pub enum Error {
    #[error(transparent)]
    Io(#[from] io::Error),
    #[error("filesystem block count is zero")]
    ZeroBlocks,
    #[error("{0}")]
    Arguments(&'static str),
}

pub struct Cycle {
    pub deleted: Option<OsString>,
    pub wait: Duration,
}

pub struct Deleter {
    root: PathBuf,
}

impl Deleter {
    pub fn new(root: &Path) -> Self {
        Self {
            root: root.to_owned(),
        }
    }

    fn directories(&self) -> Vec<OsString> {
        if !self.root.is_dir() {
            return Vec::new();
        }
        let result = (|| -> io::Result<Vec<OsString>> {
            let mut names = Vec::new();
            for entry in fs::read_dir(&self.root)? {
                let entry = entry?;
                if entry.path().is_dir() {
                    names.push(entry.file_name());
                }
            }
            names.sort_by_cached_key(|name| names::sort_key(name));
            Ok(names)
        })();
        result.unwrap_or_else(|error| {
            eprintln!("deleter: listdir_by_creation failed: {error}");
            Vec::new()
        })
    }

    fn has_preserve(&self, name: &OsString) -> Result<bool, Error> {
        // Logger can add a bookmark after an earlier pressure scan. Read the
        // current attribute; only the exact one-byte marker is preserved.
        let mut buffer = [0_u8; 1];
        match rustix::fs::getxattr(self.root.join(name), "user.preserve", &mut buffer[..]) {
            Ok(length) => Ok(&buffer[..length] == b"1"),
            Err(rustix::io::Errno::NODATA | rustix::io::Errno::RANGE) => Ok(false),
            Err(error) => Err(io::Error::from(error).into()),
        }
    }

    fn preserved_from(&mut self, directories: &[OsString]) -> Result<HashSet<OsString>, Error> {
        let mut preserved = HashSet::new();
        let mut seen = 0;
        for directory in directories.iter().rev() {
            if self.has_preserve(directory)? {
                if seen == 5 {
                    break;
                }
                seen += 1;
                preserved.extend(names::segment_and_prior(directory));
            }
        }
        Ok(preserved)
    }

    pub fn preserved(&mut self) -> Result<HashSet<OsString>, Error> {
        self.preserved_from(&self.directories())
    }

    pub fn cycle(&mut self, available_bytes: u128, available_percent: f64) -> Result<Cycle, Error> {
        if !(available_bytes < MIN_BYTES || available_percent < MIN_PERCENT) {
            return Ok(Cycle {
                deleted: None,
                wait: Duration::from_secs(30),
            });
        }
        let mut directories = self.directories();
        let preserved = self.preserved_from(&directories)?;
        directories
            .sort_by_key(|name| (name == "boot" || name == "crash", preserved.contains(name)));
        if let Some(directory) = self.drop_oldest_video(&directories, &preserved) {
            return Ok(Cycle {
                deleted: Some(directory),
                wait: Duration::from_millis(100),
            });
        }
        let mut deleted = None;
        for directory in directories {
            let path = self.root.join(&directory);
            let result = (|| -> io::Result<bool> {
                for entry in fs::read_dir(&path)? {
                    if entry?.file_name().as_encoded_bytes().ends_with(b".lock") {
                        return Ok(false);
                    }
                }
                eprintln!("deleter: deleting {}", path.display());
                if fs::symlink_metadata(&path)?.file_type().is_symlink() {
                    return Err(io::Error::other(
                        "cannot remove a symbolic link as a directory",
                    ));
                }
                fs::remove_dir_all(&path)?;
                Ok(true)
            })();
            match result {
                Ok(true) => {
                    deleted = Some(directory);
                    break;
                }
                Ok(false) => {}
                Err(error) if error.kind() == io::ErrorKind::NotFound => {}
                Err(error) => eprintln!("deleter: issue deleting {}: {error}", path.display()),
            }
        }
        Ok(Cycle {
            deleted,
            wait: Duration::from_millis(100),
        })
    }

    /// Removes the videos of the oldest unlocked, unpreserved segment that has
    /// any, while the segments already without video fit their budget. Returns
    /// that segment, or None to fall back to deleting whole directories.
    fn drop_oldest_video(
        &self,
        directories: &[OsString],
        preserved: &HashSet<OsString>,
    ) -> Option<OsString> {
        let mut total = 0;
        let mut stripped = 0;
        let mut candidate = None;
        for directory in directories {
            if directory == "boot" || directory == "crash" {
                continue;
            }
            let path = self.root.join(directory);
            let Ok(usage) = usage(&path) else {
                continue;
            };
            total += usage.bytes;
            if !usage.video {
                stripped += usage.bytes;
            } else if candidate.is_none() && !usage.locked && !preserved.contains(directory) {
                candidate = Some(directory);
            }
        }
        if stripped >= STRIPPED_BUDGET_BYTES.min(total / 2) {
            return None;
        }
        let directory = candidate?;
        let path = self.root.join(directory);
        eprintln!("deleter: deleting videos of {}", path.display());
        let mut removed = false;
        for video in VIDEO_FILES {
            match fs::remove_file(path.join(video)) {
                Ok(()) => removed = true,
                Err(error) if error.kind() == io::ErrorKind::NotFound => {}
                Err(error) => eprintln!(
                    "deleter: issue deleting {}: {error}",
                    path.join(video).display()
                ),
            }
        }
        removed.then(|| directory.clone())
    }

    pub fn tick(&mut self) -> Result<Cycle, Error> {
        let bytes = platform::available_bytes(&self.root);
        let percent = platform::available_percent(&self.root)?;
        self.cycle(bytes, percent)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn segment(root: &Path, name: &str, files: &[(&str, usize)]) {
        let path = root.join(name);
        fs::create_dir_all(&path).unwrap();
        for (file, size) in files {
            fs::write(path.join(file), vec![0_u8; *size]).unwrap();
        }
    }

    fn names(root: &Path, name: &str) -> Vec<String> {
        let mut files: Vec<_> = fs::read_dir(root.join(name))
            .map(|entries| {
                entries
                    .map(|entry| entry.unwrap().file_name().into_string().unwrap())
                    .collect()
            })
            .unwrap_or_default();
        files.sort();
        files
    }

    const PRESSURE: (u128, f64) = (MIN_BYTES - 1, 50.0);

    #[test]
    fn plenty_of_space_deletes_nothing() {
        let root = tempfile::tempdir().unwrap();
        segment(
            root.path(),
            "00000001--a--0",
            &[("fcamera.hevc", 10), ("rlog.zst", 1)],
        );
        let cycle = Deleter::new(root.path()).cycle(MIN_BYTES, 50.0).unwrap();
        assert!(cycle.deleted.is_none());
        assert_eq!(cycle.wait, Duration::from_secs(30));
        assert_eq!(names(root.path(), "00000001--a--0").len(), 2);
    }

    #[test]
    fn videos_of_the_oldest_segment_go_before_any_log() {
        let root = tempfile::tempdir().unwrap();
        let video = [
            ("fcamera.hevc", 1000),
            ("ecamera.hevc", 1000),
            ("qcamera.ts", 20),
            ("rlog.zst", 50),
            ("qlog.zst", 5),
        ];
        segment(root.path(), "00000001--a--0", &video);
        segment(root.path(), "00000001--a--1", &video);
        let mut deleter = Deleter::new(root.path());
        let cycle = deleter.cycle(PRESSURE.0, PRESSURE.1).unwrap();
        assert_eq!(cycle.deleted.as_deref(), Some("00000001--a--0".as_ref()));
        assert_eq!(
            names(root.path(), "00000001--a--0"),
            ["qcamera.ts", "qlog.zst", "rlog.zst"]
        );
        assert_eq!(names(root.path(), "00000001--a--1").len(), 5);
        deleter.cycle(PRESSURE.0, PRESSURE.1).unwrap();
        assert_eq!(
            names(root.path(), "00000001--a--1"),
            ["qcamera.ts", "qlog.zst", "rlog.zst"]
        );
        // No video left anywhere: back to deleting the oldest segment.
        let cycle = deleter.cycle(PRESSURE.0, PRESSURE.1).unwrap();
        assert_eq!(cycle.deleted.as_deref(), Some("00000001--a--0".as_ref()));
        assert!(!root.path().join("00000001--a--0").exists());
    }

    #[test]
    fn locked_and_preserved_segments_keep_their_videos() {
        let root = tempfile::tempdir().unwrap();
        let segment_files = [("fcamera.hevc", 1000), ("rlog.zst", 10)];
        segment(root.path(), "00000001--a--0", &segment_files);
        fs::write(root.path().join("00000001--a--0/rlog.lock"), b"").unwrap();
        segment(root.path(), "00000001--a--5", &segment_files);
        segment(root.path(), "00000001--a--9", &segment_files);
        rustix::fs::setxattr(
            root.path().join("00000001--a--5"),
            "user.preserve",
            b"1",
            rustix::fs::XattrFlags::empty(),
        )
        .unwrap();
        let cycle = Deleter::new(root.path())
            .cycle(PRESSURE.0, PRESSURE.1)
            .unwrap();
        assert_eq!(cycle.deleted.as_deref(), Some("00000001--a--9".as_ref()));
        assert_eq!(names(root.path(), "00000001--a--0").len(), 3);
        assert_eq!(names(root.path(), "00000001--a--5").len(), 2);
        assert_eq!(names(root.path(), "00000001--a--9"), ["rlog.zst"]);
    }

    #[test]
    fn video_less_segments_beyond_half_the_logs_are_deleted_whole() {
        let root = tempfile::tempdir().unwrap();
        segment(root.path(), "00000001--a--0", &[("rlog.zst", 600)]);
        segment(
            root.path(),
            "00000001--a--1",
            &[("fcamera.hevc", 500), ("rlog.zst", 100)],
        );
        let cycle = Deleter::new(root.path())
            .cycle(PRESSURE.0, PRESSURE.1)
            .unwrap();
        assert_eq!(cycle.deleted.as_deref(), Some("00000001--a--0".as_ref()));
        assert!(!root.path().join("00000001--a--0").exists());
        assert_eq!(names(root.path(), "00000001--a--1").len(), 2);
    }

    #[test]
    fn floor_matches_the_pressure_test() {
        let gib = 1024 * 1024 * 1024;
        assert_eq!(free_floor(10 * gib), MIN_BYTES);
        assert_eq!(free_floor(100 * gib), 30 * gib);
    }
}
