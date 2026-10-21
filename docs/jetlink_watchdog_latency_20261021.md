# Jetlink USB watchdog latency, 2026-10-21

The owner requested locating and fixing model-call wall times above 50 ms while
parked with actual NAV and USB display input. GPU state is already enabled by
the normal signed runtime; this follow-up changes only the comma USB watchdog.

## Evidence

Paired controller traces captured a 50.092 ms model call. The Jetson response
URB took 12.141 ms; its same-frame server log reports GPU24.1, queue1.2,
total25.6 and send12.4 ms. On the comma, the five reply reads took 11.705 ms from first
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
Together with resource, scheduling and daemon-timing regressions:73 NAV and
62 Wip desktop checks pass. Two existing POSIX reader-startup cases cannot run on Windows due
to missing pthread_sigmask; Linux integration CI covers those cases.

## Delivery and remaining validation

Both maintained branches receive the common watchdog correction, preserving
their separate model architectures. Normal comma Git Pull and restart/reboot
activates it; no new Jetson runtime bundle is required. Post-install measurements and CI are recorded below. Reduced watchdog wakes
are not claimed to eliminate every possible >50 ms event or locationd warning.
Raw traces, vehicle logs and identifiers remain private.

## Publication and activation

Source commits NAV `cd986d7e9d` and Wip `2ea4415ac2` are single-parent.
[Nav CI](https://github.com/ajouatom/openpilot/actions/runs/39592549359) passed
94 update/ownership, 993 broad (56 skips), 119 navigation, 11 real-storage and
11 network-namespace tests. [Wip CI](https://github.com/ajouatom/openpilot/actions/runs/39592549206)
passed 786 broad (two skips), 119 navigation and 11 storage tests. Counts
overlap with the focused tests. Both Windows installer jobs passed.

Fresh Park/zero/inactive guards preceded an actual offroad maintenance hold.
Normal comma Git Pull installed NAV `cd986d7e9d`, tracked clean; normal reboot
activated it. Onroad and valid N16 NAV input recovered automatically, snapshot
hold cleared, and all earlier diagnostic trace instances disappeared. Jetson
stayed on signed source `9cfe86696ea7` with the same normal service process,
no GPU env/service override, and the original N16 model. Its expected USB
closures/protocol negotiation during the deliberate comma reboot are excluded
from steady-state intervals.

The pre-change trace observed 1,178 watchdog context switches over 11.009 s
(106.913/s). Post-change verification uses thread context-switch counters,
not a claim that an OS wake count equals a notification count. The changed
process is still normal-priority; no diagnostic priority promotion persists.

## Normal-runtime verification

Two 120-second parked intervals used GPU-default N16, normal NAV, mode7 USB HUD,
original watchdog SCHED_OTHER/cores0-3, and the original FIFO2 USB-worker helper.
No trace or priority trial ran during these intervals. The Jetson observer used
the already available NCM management address after comma reboot; it did not
stream logs or images across that channel during timing.

| Interval | Model calls | Mean ms | p99 ms | Max ms | >50 ms |
| --- | ---: | ---: | ---: | ---: | ---: |
| First120s | 2400* | 42.516* | 45.290 | 48.327 | 0 |
| Buffered repeat120s | 2400 | 42.370 | 45.741 | 48.238 | 0 |

*The first conflating observer saw2,399 model messages and one common apparent
model/camera gap. The corresponding private original rlog contains all three
missing valid messages, including the model's44.273ms call. This proves an
observer omission at that point; count/mean above include that recovered call.
The first p99 is the2,399-message observer value. The repeat used nonconflating
buffered sockets and captured2,400 consecutive valid model, DM and each camera
messages, despite a42.821ms maximum observer polling gap. All observed odometry
and pose validity/health flags passed in both windows. The earlier stopped
priority-only trial is not retroactively claimed validated by this different
log recovery.

Total4,800 model calls in the two windows, zero >50ms, maximum48.327ms. Mean
42.443ms is essentially unchanged from the preceding GPU-ON42.427ms; this is a
latency-tail correction, not another inference-speed gain. The previous GPU-ON
two120s windows had two >50ms events, maximum60.985ms. The before/after comparison
includes a comma reboot and different observation times; four minutes cannot
establish a universal hard deadline or road/long-duration stability.

NAV status was ready240/240; HUD and model-path preview were fresh240/240 host
samples. Phone-map decode-drop counter stayed zero. Watchdog context-switch
increments were91/120.005s and94/120.003s (~0.758/0.783 per second), versus
106.913/s in the earlier short trace. This corroborates reduced worker wake
activity without raising priority. The intervals/methods differ, so this is
not an exact CPU-saving ratio. Whole-commaCPU76.32/76.72% and serverCPU8.32/8.06%
of one Jetson core are descriptive, not causal CPU or memory-leak claims.

Final read-only checks find the same normal process identities, correct N16
SHA, ready NAV, all live model/DM/odometry/pose validity flags true, and only the
production USB-priority trace instance. No experimental priority/trace/hold
remains. The correction is active on the owner's comma; no further user pull
or reboot is required for its code. Later record-only commits do not change
the installed implementation. Recurring locationd problems outside these
bounded windows are not declared universally fixed.
