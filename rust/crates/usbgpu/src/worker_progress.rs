//! Optional source-compatible 48-byte worker progress record.
//! The worker writes only its private mapping; the supervisor reads before termination.
use serde_json::{json, Value};
use std::{
    cell::RefCell,
    fs::{File, OpenOptions},
    io::{Read, Seek, SeekFrom},
    path::Path,
    sync::atomic::{AtomicU64, Ordering},
};

mod watchdog;
pub(crate) use watchdog::Watchdog;

pub const STAGES: [&str; 16] = [
    "starting",
    "verify_model",
    "load_model",
    "prepare_runtime",
    "bind_buffers",
    "idle",
    "input_upload",
    "warp",
    "model_call",
    "output_read",
    "publish",
    "local_prepare",
    "open_gpu",
    "upload_weights",
    "bind_model",
    "create_queues",
];
const SIZE: usize = 48;

fn seconds(clock: rustix::time::ClockId) -> f64 {
    let value = rustix::time::clock_gettime(clock);
    value.tv_sec as f64 + value.tv_nsec as f64 * 1e-9
}

struct Mapping {
    address: *mut std::ffi::c_void,
    generation: u64,
    frame: u64,
}
impl Mapping {
    fn open(path: &Path) -> std::io::Result<Self> {
        let file = OpenOptions::new()
            .create(true)
            .truncate(true)
            .read(true)
            .write(true)
            .open(path)?;
        file.set_len(SIZE as u64)?;
        // SAFETY: this owned fixed-size temporary file remains 48 bytes throughout the
        // worker's lifetime. Only this thread writes, through aligned atomic words.
        let address = unsafe {
            rustix::mm::mmap(
                std::ptr::null_mut(),
                SIZE,
                rustix::mm::ProtFlags::READ | rustix::mm::ProtFlags::WRITE,
                rustix::mm::MapFlags::SHARED,
                &file,
                0,
            )?
        };
        Ok(Self {
            address,
            generation: 0,
            frame: 0,
        })
    }
    fn mark(&mut self, stage: usize) {
        self.generation = self.generation.wrapping_add(2).max(2);
        // SAFETY: mmap is page aligned and contains six initialized u64 slots;
        // all access within this process uses atomics and the mapping outlives them.
        let words = unsafe { std::slice::from_raw_parts(self.address.cast::<AtomicU64>(), 6) };
        words[0].store((self.generation - 1).to_le(), Ordering::SeqCst);
        for (slot, value) in words[1..].iter().zip([
            stage as u64,
            self.frame,
            seconds(rustix::time::ClockId::Monotonic).to_bits(),
            seconds(rustix::time::ClockId::ThreadCPUTime).to_bits(),
            rustix::thread::gettid().as_raw_nonzero().get() as u64,
        ]) {
            slot.store(value.to_le(), Ordering::SeqCst);
        }
        words[0].store(self.generation.to_le(), Ordering::SeqCst);
    }
}
impl Drop for Mapping {
    fn drop(&mut self) {
        // SAFETY: this is the sole owner of the original mapping and exact length.
        let _ = unsafe { rustix::mm::munmap(self.address, SIZE) };
    }
}
thread_local! { static PROGRESS: RefCell<Option<Mapping>> = const { RefCell::new(None) }; }

/// Diagnostics are optional: failure to map must never prevent inference.
pub fn initialize(path: &Path) {
    PROGRESS.with(|value| *value.borrow_mut() = Mapping::open(path).ok());
    mark("starting");
}
pub fn mark(stage: &str) {
    let Some(stage) = STAGES.iter().position(|value| *value == stage) else {
        return;
    };
    PROGRESS.with(|value| {
        if let Some(value) = value.borrow_mut().as_mut() {
            value.mark(stage);
        }
    });
}
pub fn next_frame() {
    PROGRESS.with(|value| {
        if let Some(value) = value.borrow_mut().as_mut() {
            value.frame = value.frame.saturating_add(1);
        }
    });
}

pub fn load_stage(stage: &str) {
    mark(stage);
    eprintln!("eGPU worker startup: {stage}");
}

fn read_record(path: &Path) -> std::io::Result<[u8; SIZE]> {
    let mut file = File::open(path)?;
    let mut first = [0; SIZE];
    let mut second = [0; SIZE];
    file.read_exact(&mut first)?;
    file.seek(SeekFrom::Start(0))?;
    file.read_exact(&mut second)?;
    if first != second {
        return Err(std::io::ErrorKind::WouldBlock.into());
    }
    Ok(first)
}

/// Only completed weight uploads can renew the ready handshake's inactivity timer.
pub(crate) fn upload_generation(path: &Path) -> Option<u64> {
    let record = read_record(path).ok()?;
    let generation = u64::from_le_bytes(record[..8].try_into().ok()?);
    let stage = usize::try_from(u64::from_le_bytes(record[8..16].try_into().ok()?)).ok()?;
    (generation != 0
        && generation.is_multiple_of(2)
        && STAGES.get(stage) == Some(&"upload_weights"))
    .then_some(generation)
}

/// Bounded, nonblocking best-effort reads, also usable with a fixture proc root.
pub fn snapshot(path: &Path, pid: u32, proc_root: &Path) -> Value {
    let mut result = json!({"pid":pid});
    if let Ok(record) = read_record(path) {
        let fields: Vec<u64> = record
            .chunks_exact(8)
            .map(|bytes| u64::from_le_bytes(bytes.try_into().unwrap()))
            .collect();
        let age = seconds(rustix::time::ClockId::Monotonic) - f64::from_bits(fields[3]);
        let stage = usize::try_from(fields[1])
            .ok()
            .and_then(|index| STAGES.get(index));
        if fields[0] != 0 && fields[0].is_multiple_of(2) && (0.0..86400.0).contains(&age) {
            if let Some(stage) = stage {
                result["stage"] = json!(stage);
                result["frame"] = json!(fields[2]);
                result["stage_age_ms"] = json!((age * 1e6).round_ties_even() / 1000.0);
                result["thread_cpu_seconds"] = json!(f64::from_bits(fields[4]));
                result["thread_id"] = json!(fields[5]);
            }
        }
    }
    let tid = result["thread_id"].as_u64().unwrap_or(u64::from(pid));
    let task = proc_root
        .join(pid.to_string())
        .join("task")
        .join(tid.to_string());
    for name in ["stat", "schedstat", "wchan"] {
        if let Ok(file) = File::open(task.join(name)) {
            let mut bytes = Vec::new();
            if file.take(2048).read_to_end(&mut bytes).is_ok() {
                result[name] = json!(String::from_utf8_lossy(&bytes).trim());
            }
        }
    }
    result
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn actual_shared_record_tracks_all_source_stages_and_frames() {
        let root = tempfile::tempdir().unwrap();
        let path = root.path().join("progress");
        initialize(&path);
        for (index, stage) in STAGES.iter().enumerate() {
            mark(stage);
            let value = snapshot(&path, std::process::id(), Path::new("/proc"));
            assert_eq!(value["stage"], *stage);
            assert_eq!(value["frame"], index as u64);
            assert!(value["stage_age_ms"].as_f64().unwrap() >= 0.0);
            assert!(value["thread_cpu_seconds"].as_f64().unwrap() >= 0.0);
            assert!(value["thread_id"].as_u64().unwrap() > 0);
            next_frame();
        }
        let bytes = std::fs::read(&path).unwrap();
        assert_eq!(bytes.len(), 48);
        assert_eq!(u64::from_le_bytes(bytes[..8].try_into().unwrap()) % 2, 0);
        // Missing/invalid diagnostic storage never stops worker execution.
        initialize(&root.path().join("absent/progress"));
        mark("model_call");
    }
    #[test]
    fn malformed_records_never_supply_a_stage_and_proc_reads_are_bounded() {
        let root = tempfile::tempdir().unwrap();
        let path = root.path().join("progress");
        let task = root.path().join("42/task/42");
        std::fs::create_dir_all(&task).unwrap();
        std::fs::write(task.join("stat"), "x".repeat(4096)).unwrap();
        for bytes in [vec![], vec![0; 47], vec![0; 48], vec![255; 48]] {
            std::fs::write(&path, bytes).unwrap();
            let value = snapshot(&path, 42, root.path());
            assert!(value.get("stage").is_none());
            assert_eq!(value["stat"].as_str().unwrap().len(), 2048);
        }
    }
}
