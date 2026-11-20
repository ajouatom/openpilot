use crate::{
    worker::{Info, Input, Metadata},
    Error,
};
mod frame;
mod protocol;
use protocol::receive;
use rustix::process::{kill_process, Pid, Signal};
use std::{
    io::Write,
    path::{Path, PathBuf},
    process::{Child, Command, Stdio},
    sync::{atomic::AtomicBool, Arc},
    thread,
    time::{Duration, Instant},
};

pub struct Frame<'a> {
    pub main: &'a [u8],
    pub extra: &'a [u8],
    pub transforms: [[f32; 9]; 2],
    pub inputs: &'a [f32],
}
pub struct Client {
    child: Child,
    shared: tempfile::NamedTempFile,
    progress: Option<tempfile::NamedTempFile>,
    pub info: Info,
    packed: Vec<u8>,
    output: Vec<u8>,
    values: Vec<f32>,
    first: bool,
    failed: bool,
    cancelled: Arc<AtomicBool>,
    /// carrot's own precompiled_worker.py (pinned tinygrad USB+AMD runtime)
    /// instead of the Rust native worker: only 'r' commands, and '0' means the
    /// worker held the previous output for this cycle.
    python: bool,
}

/// The original carrot eGPU worker for an artifact installed from carrot's
/// precompiled catalog (model.pkl + installed.json + runtime-<sha>). The Rust
/// native runtime never ran on a real eGPU (IONIQ 9 / comma four failed in it
/// while the same user's eGPU worked on carrot-wip), so the original path is
/// used unless USBGPU_NATIVE_WORKER=1.
fn python_worker(worker: &Path, model: &Path) -> Option<(PathBuf, PathBuf)> {
    if std::env::var_os("USBGPU_NATIVE_WORKER").is_some_and(|value| value == "1") {
        return None;
    }
    let directory = model.parent()?;
    let installed: serde_json::Value =
        serde_json::from_slice(&std::fs::read(directory.join("installed.json")).ok()?).ok()?;
    let runtime = installed["runtime_directory"].as_str()?;
    if !directory.join(runtime).is_dir() {
        return None;
    }
    // worker = <repo>/rust/bin/openpilot-usbgpu-worker
    let repository = worker.parent()?.parent()?.parent()?.to_path_buf();
    let script = repository.join("openpilot/selfdrive/modeld/precompiled_worker.py");
    script.is_file().then_some((script, repository))
}

fn worker_command(worker: &Path, model: &Path) -> (Command, bool) {
    match python_worker(worker, model) {
        Some((script, repository)) => {
            // Loaded in the background while driving (carrot loads it before
            // modeld runs): on the 3.5 GiB comma four the kernel must kill this
            // worker first, never a driving process, if memory runs out.
            let mut command = Command::new("sh");
            command.args([
                "-c",
                "echo 1000 > /proc/self/oom_score_adj 2>/dev/null; exec \"$0\" \"$@\"",
                "python3",
            ]);
            let mut path = std::ffi::OsString::from(repository.join("pydeps"));
            path.push(":");
            path.push(&repository);
            if let Some(existing) = std::env::var_os("PYTHONPATH").filter(|v| !v.is_empty()) {
                path.push(":");
                path.push(existing);
            }
            command
                .arg(script)
                .env("PYTHONPATH", path)
                .current_dir(&repository);
            (command, true)
        }
        None => (Command::new(worker), false),
    }
}

pub const LOAD_INACTIVITY: Duration = Duration::from_secs(110);
pub const MAX_LOAD: Duration = Duration::from_secs(240);

pub struct Launch<'a> {
    pub worker: &'a Path,
    pub model: &'a Path,
    pub camera: [u32; 2],
    pub timeout: Duration,
    pub cancelled: Arc<AtomicBool>,
}

fn validate(info: &Info, camera: [u32; 2]) -> Result<(), Error> {
    let mut inputs = info
        .layout
        .iter()
        .filter(|(name, _)| {
            matches!(
                name.as_str(),
                "desire" | "traffic_convention" | "action_t" | "navigation" | "nav_map"
            )
        })
        .collect::<Vec<_>>();
    inputs.sort_by_key(|(_, view)| view.offset);
    let metadata = Metadata {
        model_sha256: String::new(),
        checkpoint: info.checkpoint.clone(),
        output_count: info.output_count,
        output_slices: info.output_slices.clone(),
        inputs: inputs
            .into_iter()
            .map(|(name, view)| Input {
                name: name.clone(),
                shape: view.shape.clone(),
            })
            .collect(),
    };
    let [width, height] = camera;
    let plane_bytes = usize::try_from(
        width.next_multiple_of(128)
            * (height.next_multiple_of(32) + (height / 2).next_multiple_of(16)),
    )
    .map_err(|_| Error::Contract("worker camera size overflow"))?;
    let expected = if info.frame_size == plane_bytes {
        Info::new(metadata, camera)?
    } else {
        Info::local(metadata, camera)?
    };
    if &expected != info {
        return Err(Error::Contract("worker ready layout mismatch"));
    }
    Ok(())
}

impl Client {
    pub fn load(worker: &Path, model: &Path, camera: [u32; 2]) -> Result<Self, Error> {
        Self::launch(Launch {
            worker,
            model,
            camera,
            timeout: LOAD_INACTIVITY,
            cancelled: Arc::new(AtomicBool::new(false)),
        })
    }
    pub fn launch(launch: Launch<'_>) -> Result<Self, Error> {
        let (command, python) = worker_command(launch.worker, launch.model);
        Self::from_command(command, launch, python, false)
    }
    pub fn launch_with_assets(
        launch: Launch<'_>,
        binding: &crate::worker_artifact::Binding,
    ) -> Result<Self, Error> {
        let (mut command, python) = worker_command(launch.worker, launch.model);
        command
            .env("USBGPU_ASSETS_ROOT", binding.root())
            .env("USBGPU_ASSETS_MANIFEST_SHA256", binding.manifest_sha256());
        Self::from_command(command, launch, python, false)
    }
    /// Background recovery must never allocate or probe the internal QCOM GPU.
    pub fn launch_recovery_with_assets(
        launch: Launch<'_>,
        binding: &crate::worker_artifact::Binding,
    ) -> Result<Self, Error> {
        let (mut command, python) = worker_command(launch.worker, launch.model);
        command
            .env("USBGPU_ASSETS_ROOT", binding.root())
            .env("USBGPU_ASSETS_MANIFEST_SHA256", binding.manifest_sha256())
            .env("USBGPU_AMD_ONLY", "1");
        // carrot's worker takes '--recovery' after the positional arguments:
        // AMD-only warp and the 'z' reset used after the probe.
        Self::from_command(command, launch, python, python)
    }
    pub fn reset(&mut self) -> Result<(), Error> {
        if self.failed {
            return Err(Error::Contract("model worker stopped after failure"));
        }
        let result = (|| {
            self.child
                .stdin
                .as_mut()
                .ok_or(Error::Contract("missing worker input"))?
                .write_all(b"z")?;
            let stdout = self
                .child
                .stdout
                .as_mut()
                .ok_or(Error::Contract("missing worker output"))?;
            if receive(stdout, Duration::from_secs(5), &self.cancelled)? != b"1\n" {
                return Err(Error::Contract("invalid worker reset response"));
            }
            self.packed.fill(0);
            self.output.fill(0);
            self.values.fill(0.0);
            Ok(())
        })();
        let result = result.map_err(|error| self.capture_failure(error));
        if result.is_err() {
            self.failed = true;
            self.stop();
        }
        result
    }
    pub fn load_command(command: Command, model: &Path, camera: [u32; 2]) -> Result<Self, Error> {
        Self::from_command(
            command,
            Launch {
                worker: Path::new(""),
                model,
                camera,
                timeout: LOAD_INACTIVITY,
                cancelled: Arc::new(AtomicBool::new(false)),
            },
            false,
            false,
        )
    }
    fn from_command(
        mut command: Command,
        launch: Launch<'_>,
        python: bool,
        recovery: bool,
    ) -> Result<Self, Error> {
        let Launch {
            model,
            camera,
            timeout,
            cancelled,
            ..
        } = launch;
        let shared = tempfile::NamedTempFile::new()?;
        let progress = tempfile::NamedTempFile::new().ok();
        if let Some(progress) = &progress {
            command.env("USBGPU_PROGRESS_PATH", progress.path());
        } else {
            command.env_remove("USBGPU_PROGRESS_PATH");
        }
        command
            .arg(model)
            .arg(shared.path())
            .args(camera.map(|dimension| dimension.to_string()));
        if recovery {
            command.arg("--recovery");
        }
        let child = command
            .stdin(Stdio::piped())
            .stdout(Stdio::piped())
            .spawn()?;
        let placeholder = Info {
            size: 0,
            input_bytes: 0,
            output_count: 0,
            layout: Default::default(),
            input_shapes: Default::default(),
            output_slices: Default::default(),
            checkpoint: String::new(),
            frame_size: 0,
        };
        let mut client = Self {
            child,
            shared,
            progress,
            info: placeholder,
            packed: Vec::new(),
            output: Vec::new(),
            values: Vec::new(),
            first: true,
            failed: false,
            cancelled,
            python,
        };
        if python {
            eprintln!("eGPU worker: carrot precompiled_worker.py (pinned tinygrad runtime)");
        }
        let stdout = client
            .child
            .stdout
            .as_mut()
            .ok_or(Error::Contract("missing worker output"))?;
        let limit = if timeout == LOAD_INACTIVITY {
            MAX_LOAD
        } else {
            timeout
        };
        let response = protocol::receive_bounded(stdout, timeout, limit, &client.cancelled, || {
            client
                .progress
                .as_ref()
                .and_then(|file| crate::worker_progress::upload_generation(file.path()))
        })
        .map_err(|error| client.capture_failure(error))?;
        let info: Info = serde_json::from_slice(&response)?;
        validate(&info, camera)?;
        if client.shared.as_file().metadata()?.len() != info.size as u64 {
            return Err(Error::Contract("worker shared file length mismatch"));
        }
        client.packed = vec![0; info.input_bytes];
        client.output = vec![0; info.output_count * 4];
        client.values = vec![0.0; info.output_count];
        client.info = info;
        Ok(client)
    }
    pub fn run(&mut self, frame: Frame<'_>) -> Result<&[f32], Error> {
        self.run_with_prepare(frame, false)?
            .ok_or(Error::Contract("model worker omitted inference output"))
    }
    pub fn run_with_prepare(
        &mut self,
        frame: Frame<'_>,
        prepare_only: bool,
    ) -> Result<Option<&[f32]>, Error> {
        if self.failed {
            return Err(Error::Contract("model worker stopped after failure"));
        }
        match self.run_inner(frame, prepare_only) {
            Ok(true) => Ok(Some(&self.values)),
            Ok(false) => Ok(None),
            Err(error) => {
                let error = self.capture_failure(error);
                self.failed = true;
                self.stop();
                Err(error)
            }
        }
    }
    pub fn alive(&mut self) -> bool {
        matches!(self.child.try_wait(), Ok(None))
    }
    pub fn raw_output(&self) -> &[u8] {
        &self.output
    }
    fn capture_failure(&self, error: Error) -> Error {
        if matches!(error, Error::Cancelled) {
            return error;
        }
        Error::Worker {
            source: Box::new(error),
            progress: crate::worker_progress::snapshot(
                self.progress
                    .as_ref()
                    .map_or(Path::new(""), |file| file.path()),
                self.child.id(),
                Path::new("/proc"),
            ),
        }
    }
    fn stop(&mut self) {
        if matches!(self.child.try_wait(), Ok(Some(_))) {
            return;
        }
        if let Some(pid) = i32::try_from(self.child.id()).ok().and_then(Pid::from_raw) {
            let _ = kill_process(pid, Signal::TERM);
        }
        let started = Instant::now();
        while started.elapsed() < Duration::from_secs(2) {
            if matches!(self.child.try_wait(), Ok(Some(_))) {
                return;
            }
            thread::sleep(Duration::from_millis(5));
        }
        let _ = self.child.kill();
        let _ = self.child.wait();
    }
}
impl Drop for Client {
    fn drop(&mut self) {
        self.stop();
    }
}

#[cfg(test)]
mod worker_choice_tests {
    use super::python_worker;
    use std::fs;

    #[test]
    fn carrot_precompiled_installs_use_carrots_worker() {
        let root = tempfile::tempdir().unwrap();
        let repository = root.path().join("openpilot");
        let worker = repository.join("rust/bin/openpilot-usbgpu-worker");
        let script = repository.join("openpilot/selfdrive/modeld/precompiled_worker.py");
        fs::create_dir_all(worker.parent().unwrap()).unwrap();
        fs::create_dir_all(script.parent().unwrap()).unwrap();
        fs::write(&script, "").unwrap();
        let installed = root.path().join("models/precompiled/abc");
        fs::create_dir_all(installed.join("runtime-0123456789abcdef")).unwrap();
        fs::write(
            installed.join("installed.json"),
            r#"{"runtime_directory":"runtime-0123456789abcdef"}"#,
        )
        .unwrap();
        let model = installed.join("model.pkl");
        let (chosen, repo) = python_worker(&worker, &model).expect("carrot worker");
        assert_eq!(chosen, script);
        assert_eq!(repo, repository);
        // Without the extracted runtime (or for a native/local artifact) the
        // Rust worker stays in use.
        fs::remove_dir(installed.join("runtime-0123456789abcdef")).unwrap();
        assert!(python_worker(&worker, &model).is_none());
    }
}
