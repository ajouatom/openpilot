# Jetlink USB watchdog latency, 2026-10-21

The owner requested locating and fixing model-call wall times above 50 ms while
parked with actual NAV and USB display input. GPU state is already enabled by
the normal signed runtime; this follow-up changes only the comma USB watchdog.

## Evidence

Paired controller traces captured a 50.092 ms model call. The Jetson response
URB took 12.141 ms. On the comma, the five reply reads took 11.705 ms from first
to last completion, including an 8.496 ms userspace gap before the second read.
The first read had already completed and returned; its worker queue wait was
0.051 ms and completion-to-return was 0.094 ms. The reply reader slept in that
gap, woke after about 5 ms, slept again, and was finally awakened by the normal
priority write-watchdog thread. The main inference thread was also asleep.
This localizes this event to comma userspace reply rearming, rather than GPU
inference or USB bandwidth. The trace does not directly name the Python lock;
GIL contention/priority inversion is the mechanism inferred from the wake path.

A separate trace including that watchdog found runnable waits up to 25.533 ms.
It did not reproduce another >50 ms call. A 120 s initial trace also had none.
These are intermittent events, not a constant transfer-rate problem. First two
traces reported no kernel buffer overruns. Low-priority trace observers missed
some messages and are not used to claim complete 20 Hz stream continuity.

An independent guarded priority-only trial kept GPU, transport, model/DM and
kernel-worker policies unchanged. Baseline 60 s: 1,200 calls, mean42.364,
p99 46.046, max48.759 ms, zero >50. Watchdog FIFO1: 61.650 s, 1,232 calls,
mean42.389, p99 45.353, max47.605 ms, zero >50. Its observer saw one common
camera/model frame-ID gap and stopped/restored automatically; its own longest
poll gap was 88.650 ms. This does not prove a camera regression or justify
permanent priority elevation. The watchdog remains SCHED_OTHER on cores0-3.

## Correction

Previously every write arm and disarm notified the background watchdog. That
thread shares Python execution/locks with the realtime reader. Repeated short
writes therefore awakened it many times even with a 15-second pending timer.

Retain the pending wake when it is earlier than the newly armed deadline. On
that wake, inspect the current armed/cancelled state under the same condition
lock. An earlier deadline or close still notifies immediately. An idle watchdog
still wakes on arm. Preserve rejection of overlapping writes, single expiry,
cancellation-versus-expiry serialization, and callback execution outside the
lock. No timeout, validity rule, scheduler priority, model or wire bytes change.

An actual-thread regression with 1,000 completed writes fails on the old source
(2,002 notifications versus the initial one) and passes on the correction.
Six tests cover coalescing, earlier budgets, rearming across old expiry, idle
and spurious wakes, close, and nonblocking cancellation during the abort callback.
Together with resource, scheduling and daemon-timing regressions:73 desktop
checks pass. Two existing POSIX reader-startup cases cannot run on Windows due
to missing pthread_sigmask; Linux integration CI covers those cases.

## Delivery and remaining validation

Both maintained branches receive the common watchdog correction, preserving
their separate model architectures. Normal comma Git Pull and restart/reboot
activates it; no new Jetson runtime bundle is required. Live post-install
measurements and CI are recorded below when completed. Reduced watchdog wakes
are not yet claimed to eliminate every >50 ms event or locationd warning.
Raw traces, vehicle logs and identifiers remain private.
