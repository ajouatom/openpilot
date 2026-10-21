# Jetson USB completion scheduling, 2026-10-21

The owner requested retaining the measured USB scheduling improvement instead
of ending with restored diagnostic settings. Delivery is normal comma Git Pull
and reboot. No Jetson image, model, USB bootstrap installation or host update is
needed for this comma-side change.

## Evidence and scope

Earlier parked kernel traces located a 10.215 ms DWC3 completion queue wait;
a separate sample spent 12.187 ms runnable before its worker ran. This is
evidence of comma CPU scheduling delay, not proof that every link failure is
caused by it. Kernel workers originally used SCHED_OTHER/nice -20.

Manual priority-only trials with normal NAV input produced:

| Trial | Duration | Model calls | Mean | Maximum | Above 50 ms |
|---|---:|---:|---:|---:|---:|
| USB workers FIFO 2, original userspace placement/serial reads | 120 s | 2400 | 44.50 ms | 49.81 ms | 0 |
| USB workers FIFO 6, same placement/reads | 60 s | 1200 | 44.58 ms | 49.32 ms | 0 |

These are model-call wall times including communication, not GPU-only inference.
Full rlogs confirm consecutive valid model/DM/camera frames and healthy pose
inputs in those intervals. FIFO 6 did not demonstrate an additional benefit.
Short workqueue traces reduced FIFO-2 completion queue p99 to 0.130 ms versus
3.273 ms in a restored 60-second interval; samples have unequal durations.

AIO plus core 7 and FIFO-2 workers improved average latency but still reached
79.96 ms. Serial plus core 7 and FIFO-2 workers reached 60.44 ms. Neither is
enabled by this change. Those later comparisons had invalid route_position
NAV input and cannot stand in for normal NAV operation.

## Implementation

`jetlinkd` starts a root helper only for an identified Jetson using bulk USB,
after model preparation and before warmup. The helper currently accepts only
the physically tested comma mici/aarch64 Linux 4.9.103 family. Other hardware
or missing trace capabilities retain existing scheduling.

An isolated 64 KiB-per-CPU trace instance filters only `dwc3_bh_work` execution
events. It discovers actual worker PIDs instead of hard-coding the tested
u17 pool. Each candidate must be an unbound kernel worker with kernel parent,
PF_KTHREAD, and matching start time. Only SCHED_OTHER workers are promoted to
FIFO 2. No affinity, nice value, model/DM priority or timeout is changed.

Workers are shared kernel resources: while leased, a worker can execute other
work too. This is not a private DWC3 workqueue/kernel patch. Leases expire after
three seconds without observing that worker's DWC3 work. Bookkeeping is capped
at 16 workers. The helper runs SCHED_OTHER/nice 10 on cores 0-3, polls every
200 ms, and never runs in the frame exchange path.

The owner-lifetime pipe closes on normal session cleanup or unexpected daemon
exit. The helper then restores surviving identities and removes its own trace
instance. SIGTERM/INT/HUP use the same cleanup. A root-only boot-ID journal is
written before mutation so the next helper can recover after an uncatchable
helper termination. SIGKILL cannot execute immediate cleanup; reboot also
clears scheduler changes. PID reuse and another owner's scheduling changes are
not overwritten. A failed worker restore does not prevent restoring the other
workers; uncertain restoration blocks session reuse and retains the journal.

## Validation

Desktop tests cover lease renewal/expiry, worker replacement, PID reuse,
another scheduling owner, journal ordering/recovery, partial restore failure,
actual captured trace syntax, fragmented records, transport scoping and owner
cleanup. Existing power, timing, startup, AIO and hotplug checks are included.
The direct daemon-import test requires Linux (`fcntl`) and is run by CI.

An actual parked automatic-helper trial applied to discovered workers, restored
them at pipe EOF, then applied again in a second session. The 60-second interval
observed 1201 consecutive model messages, mean 44.51 ms, maximum 50.94 ms, one
above 50 ms; no model/DM/camera/odometry/pose validity failure was observed.
The ten-second reconnect interval had two calls above 50 ms, maximum 50.85 ms.
Both cleanups left an empty journal and no private trace instance. These native
observations used route_position-invalid NAV input. They verify the automatic
lifecycle, not full NAV/road stability or a guaranteed 50 ms deadline.

The original N16 model, serial transport, userspace CPU placement, 150 ms IPC
deadline, validity gates and locationd are preserved. Recurring locationd
warnings and all sporadic delays are not claimed solved. Raw device logs,
trace data and vehicle identifiers remain private.
