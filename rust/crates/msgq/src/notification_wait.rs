use crate::Error;
use nix::{
    errno::Errno,
    poll::ppoll,
    sys::signal::{SigSet, SigmaskHow, Signal},
};
use std::time::{Duration, Instant};

struct NotificationMask(Option<SigSet>);

impl NotificationMask {
    fn block() -> Result<Self, Error> {
        let mut blocked = SigSet::empty();
        blocked.add(Signal::SIGUSR2);
        let previous = blocked
            .thread_swap_mask(SigmaskHow::SIG_BLOCK)
            .map_err(|error| Error::Io("block msgq notification", error.into()))?;
        Ok(Self(Some(previous)))
    }

    fn restore(mut self) -> Result<(), Error> {
        if let Some(previous) = self.0.take() {
            previous
                .thread_set_mask()
                .map_err(|error| Error::Io("restore msgq notification mask", error.into()))?;
        }
        Ok(())
    }
}

impl Drop for NotificationMask {
    fn drop(&mut self) {
        if let Some(previous) = self.0.take() {
            if let Err(error) = previous.thread_set_mask() {
                eprintln!("msgq notification mask restoration failed: {error}");
            }
        }
    }
}

pub(crate) fn poll(
    timeout_ms: i32,
    mut ready: impl FnMut() -> Result<bool, Error>,
) -> Result<(), Error> {
    if timeout_ms < -1 {
        return Err(Error::Invalid("invalid poll timeout"));
    }
    // A nonblocking probe cannot miss a wakeup: it never enters a wait. Avoid
    // changing the caller's signal mask and scanning the same queues twice.
    if timeout_ms == 0 {
        return ready().map(|_| ());
    }
    // Blocking before the queue check keeps an intervening SIGUSR2 pending until
    // ppoll atomically restores the caller's mask while entering its wait.
    let mask = NotificationMask::block()?;
    let result = wait(timeout_ms, &mut ready, mask.0);
    mask.restore()?;
    result
}

/// `poll`, but returning no earlier than `not_before`. Notifications that
/// arrive meanwhile stay pending (SIGUSR2 blocked, so they coalesce into one),
/// and the queues are scanned once at the end rather than once per message: a
/// subscriber of ~140 queues woke ~1,150 times a second and spent ~180 us a
/// wake reading every queue's shared header. Only for a subscriber that can
/// wait (loggerd); control loops keep `poll`.
pub(crate) fn poll_not_before(
    timeout_ms: i32,
    not_before: Instant,
    mut ready: impl FnMut() -> Result<bool, Error>,
) -> Result<(), Error> {
    if timeout_ms < -1 {
        return Err(Error::Invalid("invalid poll timeout"));
    }
    let mask = NotificationMask::block()?;
    let now = Instant::now();
    if not_before > now {
        std::thread::sleep(not_before - now);
    }
    let result = wait(timeout_ms, &mut ready, mask.0);
    mask.restore()?;
    result
}

fn wait(
    timeout_ms: i32,
    ready: &mut impl FnMut() -> Result<bool, Error>,
    original_mask: Option<SigSet>,
) -> Result<(), Error> {
    let deadline = u32::try_from(timeout_ms)
        .ok()
        .map(|milliseconds| Instant::now() + Duration::from_millis(u64::from(milliseconds)));
    if ready()? {
        return Ok(());
    }
    loop {
        let remaining = deadline.map_or(Duration::from_millis(100), |end| {
            end.saturating_duration_since(Instant::now())
        });
        match ppoll(&mut [], Some(remaining.into()), original_mask) {
            Ok(_) | Err(Errno::EINTR) => {}
            Err(error) => return Err(Error::Io("poll msgq", error.into())),
        }
        if ready()? || deadline.is_some_and(|end| Instant::now() >= end) {
            return Ok(());
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn not_before_waits_out_the_interval_then_scans_once() {
        let original = SigSet::thread_get_mask().unwrap();
        let start = Instant::now();
        let mut scans = 0;
        poll_not_before(1000, start + Duration::from_millis(30), || {
            scans += 1;
            Ok(true)
        })
        .unwrap();
        assert!(start.elapsed() >= Duration::from_millis(30));
        assert_eq!(
            scans, 1,
            "a ready queue is found by the one scan after the interval"
        );
        assert_eq!(SigSet::thread_get_mask().unwrap(), original);
        // A deadline already past costs no wait.
        let start = Instant::now();
        poll_not_before(1000, start - Duration::from_millis(5), || Ok(true)).unwrap();
        assert!(start.elapsed() < Duration::from_millis(20));
    }

    #[test]
    fn caller_mask_is_restored_after_ready_error_and_unwind() {
        let original = SigSet::thread_get_mask().unwrap();
        let restore = NotificationMask(Some(original));
        let mut caller_mask = original;
        caller_mask.add(Signal::SIGUSR1);
        caller_mask.thread_set_mask().unwrap();

        for timeout in [0, 1] {
            poll(timeout, || Ok(true)).unwrap();
            assert_eq!(SigSet::thread_get_mask().unwrap(), caller_mask);
            assert!(poll(timeout, || Err(Error::Invalid("fixture queue read"))).is_err());
            assert_eq!(SigSet::thread_get_mask().unwrap(), caller_mask);
            let panic = std::panic::catch_unwind(|| poll(timeout, || panic!("fixture unwind")));
            assert!(panic.is_err());
            assert_eq!(SigSet::thread_get_mask().unwrap(), caller_mask);
        }
        restore.restore().unwrap();
    }

    #[test]
    fn empty_zero_timeout_checks_without_waiting() {
        let started = Instant::now();
        let mut checks = 0;
        poll(0, || {
            checks += 1;
            Ok(false)
        })
        .unwrap();
        assert_eq!(checks, 1);
        assert!(started.elapsed() < Duration::from_millis(100));
    }

    #[test]
    fn nonblocking_check_preserves_mask_during_callback() {
        let original = SigSet::thread_get_mask().unwrap();
        let restore = NotificationMask(Some(original));
        let mut caller_mask = original;
        caller_mask.remove(Signal::SIGUSR2);
        caller_mask.thread_set_mask().unwrap();
        poll(0, || {
            assert_eq!(SigSet::thread_get_mask().unwrap(), caller_mask);
            Ok(false)
        })
        .unwrap();
        restore.restore().unwrap();
    }
}
