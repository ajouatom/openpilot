use crate::Error;
use rustix::event::{poll, PollFd, PollFlags, Timespec};
use std::{
    io::Read,
    process::ChildStdout,
    sync::atomic::{AtomicBool, Ordering},
    time::{Duration, Instant},
};

pub(super) fn receive(
    stdout: &mut ChildStdout,
    timeout: Duration,
    cancelled: &AtomicBool,
) -> Result<Vec<u8>, Error> {
    receive_bounded(stdout, timeout, timeout, cancelled, || None)
}

pub(super) fn receive_bounded(
    stdout: &mut ChildStdout,
    inactivity: Duration,
    limit: Duration,
    cancelled: &AtomicBool,
    mut progress: impl FnMut() -> Option<u64>,
) -> Result<Vec<u8>, Error> {
    let started = Instant::now();
    let mut last_progress = started;
    let mut generation = None;
    let mut next_sample = Duration::ZERO;
    let mut line = Vec::new();
    loop {
        if cancelled.load(Ordering::Relaxed) {
            return Err(Error::Cancelled);
        }
        let elapsed = started.elapsed();
        if limit > inactivity && elapsed >= next_sample {
            next_sample = elapsed + Duration::from_secs(1);
            if let Some(current) = progress() {
                if generation != Some(current) {
                    generation = Some(current);
                    last_progress = Instant::now();
                }
            }
        }
        let remaining = limit
            .saturating_sub(elapsed)
            .min(inactivity.saturating_sub(last_progress.elapsed()));
        if remaining.is_zero() {
            return Err(Error::Contract("precompiled eGPU worker timed out"));
        }
        let mut descriptors = [PollFd::new(&*stdout, PollFlags::IN)];
        let interval_duration = remaining.min(Duration::from_millis(100));
        let interval = Timespec {
            tv_sec: i64::try_from(interval_duration.as_secs())
                .map_err(|_| Error::Contract("worker timeout overflow"))?,
            tv_nsec: i64::from(interval_duration.subsec_nanos()),
        };
        match poll(&mut descriptors, Some(&interval)) {
            Ok(0) => continue,
            Ok(_) => {}
            Err(rustix::io::Errno::INTR) => continue,
            Err(error) => return Err(std::io::Error::from(error).into()),
        }
        let mut byte = [0];
        if stdout.read(&mut byte)? == 0 {
            return Err(Error::Contract("precompiled eGPU worker exited"));
        }
        line.push(byte[0]);
        if line.len() > 128 << 10 {
            return Err(Error::Contract("worker response size limit"));
        }
        if byte[0] == b'\n' {
            break;
        }
    }
    if let Some(error) = line.strip_prefix(b"ERROR ") {
        return Err(Error::Protocol(serde_json::from_slice::<String>(error)?));
    }
    Ok(line)
}
