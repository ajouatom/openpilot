use crate::{paths::Paths, Error};
use std::{
    fs::{self, OpenOptions},
    os::unix::fs::{symlink, MetadataExt, PermissionsExt},
    path::Path,
};

pub fn get_consistent_flag(path: &Path) -> bool {
    path.join(".overlay_consistent").is_file()
}

/// The next start compiles what changed, so finalizing only marks the overlay consistent.
pub fn finalize_runtime_package(
    paths: &Paths,
    _commands: &mut dyn crate::process::Commands,
) -> Result<(), Error> {
    let finalized = paths.finalized();
    set_consistent_flag(paths, &finalized, false)?;
    set_consistent_flag(paths, &finalized, true)
}
pub fn touch(path: &Path) -> Result<(), Error> {
    let now = rustix::fs::Timestamps {
        last_access: rustix::time::Timespec {
            tv_sec: 0,
            tv_nsec: rustix::fs::UTIME_NOW,
        },
        last_modification: rustix::time::Timespec {
            tv_sec: 0,
            tv_nsec: rustix::fs::UTIME_NOW,
        },
    };
    if rustix::fs::utimensat(rustix::fs::CWD, path, &now, rustix::fs::AtFlags::empty()).is_err() {
        OpenOptions::new()
            .create(true)
            .truncate(false)
            .write(true)
            .open(path)?;
    }
    Ok(())
}
pub fn unlink_missing_ok(path: &Path) -> Result<(), Error> {
    match fs::remove_file(path) {
        Ok(()) => Ok(()),
        Err(error) if error.kind() == std::io::ErrorKind::NotFound => Ok(()),
        Err(error) => Err(error.into()),
    }
}
pub fn set_consistent_flag(paths: &Paths, path: &Path, consistent: bool) -> Result<(), Error> {
    paths.sync()?;
    let flag = path.join(".overlay_consistent");
    if consistent {
        touch(&flag)?;
    } else {
        unlink_missing_ok(&flag)?;
    }
    paths.sync()?;
    Ok(())
}
/// Bytes a copy_tree of `path` writes: regular files rounded up to 4 KiB
/// blocks, symlinks and directories one block each. Nothing is followed.
pub fn tree_bytes(path: &Path) -> Result<u64, Error> {
    const BLOCK: u64 = 4096;
    let metadata = fs::symlink_metadata(path)?;
    if !metadata.is_dir() {
        return Ok(metadata.len().div_ceil(BLOCK).max(1) * BLOCK);
    }
    let mut total = BLOCK;
    for entry in fs::read_dir(path)? {
        total += tree_bytes(&entry?.path())?;
    }
    Ok(total)
}
/// (available, total) bytes of the filesystem holding `path`, as the deleter
/// measures them.
pub fn filesystem_bytes(path: &Path) -> Result<(u64, u64), Error> {
    let stat = rustix::fs::statvfs(path).map_err(std::io::Error::from)?;
    Ok((
        stat.f_bavail.saturating_mul(stat.f_frsize),
        stat.f_blocks.saturating_mul(stat.f_frsize),
    ))
}
/// Space a drive keeps for its logs beyond the deleter floor after an update copy.
pub const LOG_RESERVE_BYTES: u64 = 2 * 1024 * 1024 * 1024;
/// The error for a copy of `needed` bytes that would leave less than the
/// deleter floor plus LOG_RESERVE_BYTES free; `reclaim` is the old copy it replaces.
pub fn copy_space_shortage(available: u64, reclaim: u64, needed: u64, total: u64) -> Option<Error> {
    let floor = u64::try_from(openpilot_deleter::free_floor(u128::from(total)))
        .unwrap_or(u64::MAX)
        .saturating_add(LOG_RESERVE_BYTES);
    let available = available.saturating_add(reclaim);
    (available < needed.saturating_add(floor)).then_some(Error::Storage {
        needed_mb: needed >> 20,
        available_mb: available >> 20,
        floor_mb: floor >> 20,
    })
}
pub fn is_mount(path: &Path) -> bool {
    let Ok(meta) = fs::symlink_metadata(path) else {
        return false;
    };
    if meta.file_type().is_symlink() {
        return false;
    }
    let Ok(parent) = fs::symlink_metadata(path.join("..")) else {
        return false;
    };
    meta.dev() != parent.dev() || meta.ino() == parent.ino()
}
pub fn remove_tree(path: &Path) -> Result<(), Error> {
    if fs::symlink_metadata(path)?.file_type().is_symlink() {
        return Err(Error::Contract("cannot rmtree a symlink"));
    }
    Ok(fs::remove_dir_all(path)?)
}
pub fn copy_tree(source: &Path, target: &Path) -> Result<(), Error> {
    let mut failures = Vec::new();
    copy_directory(source, target, &mut failures);
    if failures.is_empty() {
        Ok(())
    } else {
        Err(Error::CopyTree(failures))
    }
}
fn copy_directory(source: &Path, target: &Path, failures: &mut Vec<(std::path::PathBuf, String)>) {
    let copy = (|| -> Result<(), Error> {
        let entries = fs::read_dir(source)?.collect::<Result<Vec<_>, _>>()?;
        fs::create_dir(target)?;
        for entry in entries {
            let from = entry.path();
            let to = target.join(entry.file_name());
            let result = (|| -> Result<(), Error> {
                let metadata = fs::symlink_metadata(&from)?;
                if metadata.file_type().is_symlink() {
                    symlink(fs::read_link(&from)?, &to)?;
                } else if metadata.is_dir() {
                    copy_directory(&from, &to, failures);
                    return Ok(());
                } else if metadata.is_file() {
                    fs::copy(&from, &to)?;
                } else {
                    return Err(Error::Contract("copytree source is not a regular file"));
                }
                copy_stat(&from, &to, &fs::symlink_metadata(&from)?)
            })();
            if let Err(error) = result {
                failures.push((from, error.to_string()));
            }
        }
        copy_stat(source, target, &fs::metadata(source)?)
    })();
    if let Err(error) = copy {
        failures.push((source.into(), error.to_string()));
    }
}
fn copy_stat(source: &Path, target: &Path, metadata: &fs::Metadata) -> Result<(), Error> {
    let link = metadata.file_type().is_symlink();
    let times = rustix::fs::Timestamps {
        last_access: rustix::time::Timespec {
            tv_sec: metadata.atime(),
            tv_nsec: metadata.atime_nsec(),
        },
        last_modification: rustix::time::Timespec {
            tv_sec: metadata.mtime(),
            tv_nsec: metadata.mtime_nsec(),
        },
    };
    rustix::fs::utimensat(
        rustix::fs::CWD,
        target,
        &times,
        if link {
            rustix::fs::AtFlags::SYMLINK_NOFOLLOW
        } else {
            rustix::fs::AtFlags::empty()
        },
    )
    .map_err(std::io::Error::from)?;
    copy_xattrs(source, target, link)?;
    if !link {
        fs::set_permissions(target, fs::Permissions::from_mode(metadata.mode()))?;
    }
    Ok(())
}
fn copy_xattrs(source: &Path, target: &Path, link: bool) -> Result<(), Error> {
    use rustix::{
        fs::{getxattr, lgetxattr, listxattr, llistxattr, lsetxattr, setxattr, XattrFlags},
        io::Errno,
    };
    use std::{ffi::OsStr, os::unix::ffi::OsStrExt};
    let mut names = vec![0_u8; 65536];
    let result = if link {
        llistxattr(source, names.as_mut_slice())
    } else {
        listxattr(source, names.as_mut_slice())
    };
    let length = match result {
        Ok(length) => length,
        Err(Errno::NOTSUP | Errno::NODATA | Errno::INVAL) => return Ok(()),
        Err(error) => return Err(std::io::Error::from(error).into()),
    };
    let mut value = vec![0_u8; 65536];
    for name in names[..length].split(|b| *b == 0).filter(|v| !v.is_empty()) {
        let name = OsStr::from_bytes(name);
        let result = (|| {
            let length = if link {
                lgetxattr(source, name, value.as_mut_slice())
            } else {
                getxattr(source, name, value.as_mut_slice())
            }?;
            if link {
                lsetxattr(target, name, &value[..length], XattrFlags::empty())
            } else {
                setxattr(target, name, &value[..length], XattrFlags::empty())
            }
        })();
        match result {
            Ok(()) | Err(Errno::PERM | Errno::NOTSUP | Errno::NODATA | Errno::INVAL) => {}
            Err(error) => return Err(std::io::Error::from(error).into()),
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    const MB: u64 = 1024 * 1024;

    #[test]
    fn copy_that_would_starve_the_logs_is_refused() {
        // C3X Lite on the K5, route 00000287: an ~18 GB /data, 43.7 % free
        // before the copy, 29 % after it.
        let total = 18 * 1024 * MB;
        let available = total * 437 / 1000;
        let needed = 2400 * MB;
        let error = copy_space_shortage(available, 0, needed, total).unwrap();
        assert!(matches!(
            error,
            Error::Storage {
                needed_mb: 2400,
                ..
            }
        ));
        assert!(error.to_string().contains("no log was deleted"));
        // Replacing an old copy of the same size does not help either.
        assert!(copy_space_shortage(available - needed, needed, needed, total).is_some());
    }

    #[test]
    fn copy_on_a_roomy_disk_goes_ahead() {
        let total = 100 * 1024 * MB;
        assert!(copy_space_shortage(60 * 1024 * MB, 0, 6 * 1024 * MB, total).is_none());
        // Exactly the deleter floor (30 %) + reserve + copy is enough.
        let floor = 30 * 1024 * MB + LOG_RESERVE_BYTES;
        assert!(copy_space_shortage(floor + 6 * 1024 * MB, 0, 6 * 1024 * MB, total).is_none());
        assert!(copy_space_shortage(floor + 6 * 1024 * MB - 1, 0, 6 * 1024 * MB, total).is_some());
    }

    #[test]
    fn tree_bytes_counts_blocks_without_following_links() {
        let root = tempfile::tempdir().unwrap();
        fs::write(root.path().join("a"), vec![0_u8; 5000]).unwrap();
        fs::create_dir(root.path().join("d")).unwrap();
        fs::write(root.path().join("d/b"), b"x").unwrap();
        symlink("/usr", root.path().join("link")).unwrap();
        // root + a (2 blocks) + d + b + link
        assert_eq!(tree_bytes(root.path()).unwrap(), 6 * 4096);
    }
}
