pub(super) mod startup;
use super::{git_config, process};
use crate::{config::Config, Error};
use num_traits::ToPrimitive;
use openpilot_process_supervision::CapturedCommand;
use std::{
    ffi::OsString,
    fs::{self, File, Metadata, OpenOptions, TryLockError},
    os::{
        fd::AsFd,
        unix::{
            fs::{MetadataExt, OpenOptionsExt},
            process::ExitStatusExt,
        },
    },
    path::{Path, PathBuf},
    time::{Duration, SystemTime, UNIX_EPOCH},
};

pub(super) enum Failure {
    Busy(String),
    Runtime(Error),
}
impl From<Error> for Failure {
    fn from(error: Error) -> Self {
        Self::Runtime(error)
    }
}
impl From<std::io::Error> for Failure {
    fn from(error: std::io::Error) -> Self {
        Self::Runtime(error.into())
    }
}

fn identity(metadata: &Metadata) -> (u64, u64, i64, i64, u64) {
    (
        metadata.dev(),
        metadata.ino(),
        metadata.mtime(),
        metadata.mtime_nsec(),
        metadata.size(),
    )
}
fn git_running() -> bool {
    let Ok(entries) = fs::read_dir("/proc") else {
        return true;
    };
    for entry in entries {
        let Ok(entry) = entry else {
            return true;
        };
        if !entry
            .file_name()
            .as_encoded_bytes()
            .iter()
            .all(u8::is_ascii_digit)
        {
            continue;
        }
        let bytes = match fs::read(entry.path().join("comm")) {
            Ok(bytes) => bytes,
            Err(error) if error.kind() == std::io::ErrorKind::NotFound => continue,
            Err(_) => return true,
        };
        let Ok(name) = String::from_utf8(bytes) else {
            return true;
        };
        let name = name.trim_matches(git_config::text::whitespace);
        if name == "git" || name.starts_with("git-") {
            return true;
        }
    }
    false
}
fn recover(config: &Config, lock: &File) -> Result<bool, Failure> {
    let request = process::request(
        config,
        &[
            "git",
            "rev-parse",
            "--path-format=absolute",
            "--git-path",
            "index.lock",
        ],
        &config.repository,
    );
    let output = process::capture(&request, 10, Some(lock.as_fd()))?;
    if output.code != 0 {
        let error = output.stderr.trim_matches(git_config::text::whitespace);
        return Err(Error::Source(if error.is_empty() {
            "Unable to locate Git index lock".into()
        } else {
            error.into()
        })
        .into());
    }
    let path = config
        .repository
        .join(output.stdout.trim_matches(git_config::text::whitespace));
    let inspect = || match fs::symlink_metadata(&path) {
        Ok(metadata) => Ok(Some(metadata)),
        Err(error) if error.kind() == std::io::ErrorKind::NotFound => Ok(None),
        Err(error) => Err(Failure::Runtime(error.into())),
    };
    let Some(before) = inspect()? else {
        return Ok(false);
    };
    if !before.file_type().is_file() {
        return Err(Failure::Busy(
            "Git index lock is not a regular file; manual inspection required.".into(),
        ));
    }
    let now = match SystemTime::now().duration_since(UNIX_EPOCH) {
        Ok(time) => time.as_secs_f64(),
        Err(time) => -time.duration().as_secs_f64(),
    };
    let mtime = before.mtime().to_f64().unwrap_or(f64::INFINITY)
        + before.mtime_nsec().to_f64().unwrap_or(0.) / 1_000_000_000.;
    if now - mtime < 60. || git_running() {
        return Err(Failure::Busy(
            "Git index is in use; waiting before retrying.".into(),
        ));
    }
    std::thread::sleep(Duration::from_millis(100));
    let Some(after) = inspect()? else {
        return Ok(false);
    };
    if identity(&before) != identity(&after) || git_running() {
        return Err(Failure::Busy(
            "Git index lock changed or Git is running; retry later.".into(),
        ));
    }
    fs::remove_file(path)?;
    Ok(true)
}
fn acquire(config: &Config) -> Result<File, Failure> {
    let path = std::env::var_os("CARROT_REPO_LOCK_PATH")
        .map(PathBuf::from)
        .unwrap_or_else(|| "/tmp/carrot_repo_update.lock".into());
    let lock = OpenOptions::new()
        .read(true)
        .write(true)
        .create(true)
        .truncate(false)
        .mode(0o600)
        .open(path)?;
    match lock.try_lock() {
        Ok(()) => {}
        Err(TryLockError::WouldBlock | TryLockError::Error(_)) => {
            return Err(Failure::Busy(
                "Build or another Git operation is running; retry when it finishes.".into(),
            ));
        }
    }
    if recover(config, &lock)? {
        println!("[repo_update] removed abandoned index.lock after checking Git processes");
    }
    Ok(lock)
}
pub(super) fn capture(
    config: &Config,
    argv: &[&str],
    timeout: u64,
) -> Result<process::Completed, Failure> {
    let lock = acquire(config)?;
    Ok(process::capture(
        &process::request(config, argv, &config.repository),
        timeout,
        Some(lock.as_fd()),
    )?)
}
fn status(status: std::process::ExitStatus) -> i32 {
    status
        .code()
        .unwrap_or_else(|| -status.signal().unwrap_or(1))
}
fn inherited(config: &Config, argv: &[OsString], lock: &File) -> Result<i32, Error> {
    let request = CapturedCommand {
        launcher: config.launcher.clone(),
        cwd: config.repository.clone(),
        argv: argv.into(),
    };
    Ok(status(
        request
            .spawn_inherited_with_lock(lock.as_fd())?
            .process
            .wait()?,
    ))
}
fn native_capture(
    config: &Config,
    argv: &[&str],
    timeout: u64,
    lock: &File,
) -> Result<process::Completed, Error> {
    process::capture(
        &process::request(config, argv, &config.repository),
        timeout,
        Some(lock.as_fd()),
    )
}
fn checked_git(config: &Config, args: &[&str], lock: &File) -> Result<String, Error> {
    let mut argv = vec!["git"];
    argv.extend_from_slice(args);
    let output = native_capture(config, &argv, 180, lock)?;
    if output.code != 0 {
        return Err(Error::Source(output.combined()));
    }
    Ok(output.stdout.trim().into())
}
fn stage(
    config: &Config,
    target: &str,
    branch: Option<&str>,
    ff_only: bool,
    lock: &File,
) -> Result<i32, Error> {
    let helper = config.repository.join("rust/bin/openpilot-runtime-install");
    let mut argv = vec![
        helper.to_string_lossy().into_owned(),
        "--stage-update".into(),
        "--root".into(),
        config.repository.to_string_lossy().into_owned(),
        "--target".into(),
        target.into(),
    ];
    if let Some(branch) = branch {
        argv.extend(["--branch".into(), branch.into()]);
    } else if !ff_only {
        argv.push("--detach".into());
    }
    if ff_only {
        argv.push("--ff-only".into());
    }
    let borrowed: Vec<_> = argv.iter().map(String::as_str).collect();
    let output = native_capture(config, &borrowed, 900, lock)?;
    print!("{}", output.combined());
    if output.code == 0 {
        println!("Verified source/package update staged; reboot to activate.");
    }
    Ok(output.code)
}
fn pull(config: &Config, lock: &File) -> Result<i32, Error> {
    let repository = git_config::Repository {
        directory: &config.repository,
        launcher: &config.launcher,
        lock: Some(lock.as_fd()),
    };
    let (code, output, target) = git_config::prepare_git_pull(&repository)?;
    println!("{output}");
    if code != 0 {
        return Ok(code);
    }
    stage(config, &target, None, true, lock)
}
fn native_action(
    config: &Config,
    action: &str,
    value: Option<&str>,
    lock: &File,
) -> Result<i32, Error> {
    if action == "__stage-reset-repo" {
        checked_git(config, &["remote", "set-branches", "origin", "*"], lock)?;
    }
    if action != "__stage-repair" {
        checked_git(config, &["fetch", "origin", "--prune"], lock)?;
    }
    if action == "__fetch-native" {
        return Ok(0);
    }
    if action == "__stage-repair" {
        let branch = checked_git(config, &["branch", "--show-current"], lock)?;
        return stage(
            config,
            "HEAD",
            (!branch.is_empty()).then_some(branch.as_str()),
            false,
            lock,
        );
    }
    let value = value.ok_or_else(|| Error::Source("missing native update target".into()))?;
    if action == "__stage-commit" {
        let target = checked_git(
            config,
            &[
                "rev-parse",
                "--verify",
                "--end-of-options",
                &format!("{value}^{{commit}}"),
            ],
            lock,
        )?;
        return stage(config, &target, None, false, lock);
    }
    let branch = value.strip_prefix("origin/").unwrap_or(value);
    checked_git(config, &["check-ref-format", "--branch", branch], lock)?;
    let reference = if action == "__stage-reset-repo" || value.starts_with("origin/") {
        format!("refs/remotes/origin/{branch}")
    } else {
        format!("refs/heads/{branch}")
    };
    let target = checked_git(
        config,
        &[
            "rev-parse",
            "--verify",
            "--end-of-options",
            &format!("{reference}^{{commit}}"),
        ],
        lock,
    )?;
    stage(config, &target, Some(branch), false, lock)
}
pub(super) fn internal_command(arguments: &[OsString]) -> Result<Option<i32>, Error> {
    if arguments.first().is_none_or(|arg| arg != "__repo-update") {
        return Ok(None);
    }
    let [_, flag, root, command @ ..] = arguments else {
        return Err(Error::Source(
            "usage: __repo-update --repo REPO [COMMAND ...]".into(),
        ));
    };
    if flag != "--repo" {
        return Err(Error::Source("__repo-update requires --repo".into()));
    }
    let mut config = Config::original()?;
    config.repository = Path::new(root).into();
    let lock = match acquire(&config) {
        Ok(lock) => lock,
        Err(Failure::Busy(message)) => {
            println!("{message}");
            return Ok(Some(75));
        }
        Err(Failure::Runtime(error)) => return Err(error),
    };
    Ok(Some(match command {
        [] => 0,
        [action] if action == "__git-pull" => pull(&config, &lock)?,
        [action] if matches!(action.to_str(), Some("__fetch-native" | "__stage-repair")) => {
            native_action(&config, action.to_str().unwrap(), None, &lock)?
        }
        [action, value]
            if matches!(
                action.to_str(),
                Some("__stage-checkout" | "__stage-commit" | "__stage-reset-repo")
            ) =>
        {
            native_action(
                &config,
                action.to_str().unwrap(),
                Some(
                    value
                        .to_str()
                        .ok_or_else(|| Error::Source("non UTF8 update target".into()))?,
                ),
                &lock,
            )?
        }
        argv => inherited(&config, argv, &lock)?,
    }))
}
