# Automatic process-exit tmux diagnostics

The Kona incident showed `carrot_man` exiting with SIGABRT, but the uploaded
tmux snapshots missed the failure window. The existing `CarrotException` /
`tmux_send` consumer lives inside `carrot_man`; its own failure can prevent
that upload. See [incident analysis](kona_manager_restart_20261014.md).

## Behavior

The Python process launcher enables `faulthandler` before importing the worker.
Fatal signals such as SIGABRT and SIGSEGV can then print available Python thread
stacks to stderr. Ordinary uncaught exceptions explicitly print their traceback
before Sentry handling. Native executables retain their existing stderr/core
behavior; this does not add native C-stack unwinding.

When manager reaps an unexpected nonzero exit, it sends the process name, PID,
exit code, incident wall time and launch commit/branch to a local nonblocking
Unix datagram socket. Requested stops and successful exits are excluded. The
manager performs no tmux command, filesystem write or network request, and adds
no diagnostic thread to its own interpreter. A bounded 32-event memory queue
retries on supervision ticks if the collector is unavailable.

The existing always-running `tombstoned` service hosts the receiver on devices,
independently of its upstream Sentry reporting policy. It captures the manager's
tmux pane (last 1,000 lines, wrapped lines joined) before reading Params. Each
snapshot contains a `PROCESS_EXIT` JSON header including the decoded signal.
Reports are stored separately in `/data/carrot/process_crashes/`: at most 20
reports, with at most 256 KiB of captured text each. Oldest reports are removed
at that bound, including unsent reports. Repeated upload attempts use the saved
snapshot, not the current pane. The shared `/data/media/tmux.log` is untouched.

A separate collector thread uses the existing DSM and Carrot Logs tmux upload
helpers with `tmux_why=process_crash`. Success is tracked per destination;
failures retry on a 60-second schedule, subject to preceding bounded HTTP calls.
The persisted reports and acknowledgements survive collector/device restart.
Capture proceeds while HTTP is blocked. `FAKEUPLOAD=1` prevents these uploads.
No test sends anything to the real receivers.

## Validation and limits

Focused tests cover notification buffering, diagnostic failure isolation,
retention, malformed events, missing Params/tmux, duplicate reports, independent
destination retries, recovery from persisted state and the actual upload helper
contract. The existing real-process lifecycle probe now verifies exactly one
crash notification and no requested-stop notification.

Linux CI additionally checks real Unix datagrams while the uploader is blocked
and runs a disposable Python child under an isolated tmux server. That child
aborts with core dumps disabled; its fatal Python stack must be present in the
saved tmux report. These tests are part of startup-recovery on both maintained
branches. Windows runs the portable tests and skips the Linux-only checks.

This is best-effort diagnosis, not guaranteed crash preservation. SIGKILL/OOM
cannot run a signal handler; an exit report may contain no stack. Manager death,
power loss, unavailable tmux, collector failure, storage failure, full bounded
queues or enough subsequent terminal output can prevent or truncate evidence.
Datagram delivery is not a durable acknowledgement. The original Kona SIGABRT
cause remains undiagnosed; this change supplies evidence for a future occurrence.
Branch publication is not device installation or road validation.
