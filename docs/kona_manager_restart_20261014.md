# Kona manager restart stall, 2026-10-14

The reported "장치 프로세스 동작오류" was a `commIssue` for missing
`managerState`, following a failed attempt to restart `carrot_man`. It was not
a model frame-rate failure. Use a fresh interpreter (`spawn=True`) for
`carrot_man`, including crash recovery, rather than forking the live manager.
The existing launcher, restart predicate, supervision and control fault gates
remain in effect. No vehicle installation is included.

## Recorded sequence

Full cereal decoding of Kona route `00000315--4742c01c35`, segments 131–133,
identifies `carrot-wip` commit `b089169876`. Source/local SHA256 hashes match.
The device reports `dirty=true`, so the commit is not proof of every installed
file's contents. Times below are approximate KST from logged wall/monotonic time.

| Time | Evidence |
| --- | --- |
| 10:33:07.475 | Last `carrotMan` / `navInstructionCarrot` publication in segment 131. |
| 10:33:08.979 onward | `carrot_man` PID 48695 repeatedly appears in D state. |
| 10:33:20.446 | Last `managerState` publication. |
| 10:33:20.971 | Manager logs `carrot_man is dead with -6`, then `starting python openpilot.selfdrive.carrot.carrot_man`. |
| 10:33:25.455 | `commIssue/softDisable`: managerState receipt age exceeds five seconds. |
| 10:33:28.455 | selfdriveState becomes disabled; longitudinal control deactivates. |
| 10:33:32.894 onward | Reengagement attempts show `managerState` / process-operation-error text. |
| Through 10:35:12 | No further managerState or carrotMan publication; commIssue persists. |

The manager PID remains present and runnable. From segment 132 +8.268 s to
segment 133 +58.308 s, its system CPU time increases by about 73 s, user CPU
by about 9.3 s; no replacement carrot_man child appears in the process samples.
The last operation logged is synchronous PythonProcess.start. This is the same
observable pattern as the [Xiaoge deferred-start failure](xiaoge_onroad_startup.md#deferred-process-launch-correction-2026-10-07).
The precise blocked/spinning call inside process start is not recorded.

The model continues producing 1,200 messages/minute with zero reported frame
drops in both follow-up segments. CAN remains valid. `AlwaysLateral=1` means
longitudinal disengagement did not immediately disable lateral control;
carControl.latActive follows separate driver button actions. Do not describe
this incident as simultaneous loss of every control process.

## Correction and limits

Python's [multiprocessing documentation](https://docs.python.org/3.12/library/multiprocessing.html#contexts-and-start-methods)
warns about forking a multithreaded process. The manager has active threads and
IPC by the time recovery is needed. The existing spawn option, already used by
Xiaoge and the signal observer, avoids copying that live interpreter state.
Only carrot_man's launch configuration changes; other process policies and
the five-second manager health check are unchanged.

Exit code -6 establishes SIGABRT on Linux. Neither an abort stack nor a kernel
wait stack is present, so the original carrot_man abort is not diagnosed or
claimed fixed. Repeated D states before the abort do not prove failed storage.
This correction addresses the restart path that propagates the failure into
manager supervision. Updated affected-device logs are needed to establish
recovery on that hardware.

## Validation

Tests exercise production ManagerProcess/PythonProcess/ensure_running code
with real disposable child processes and three active parent threads. A
harmless worker substitutes for vehicle services; device imports, logging and
message allocation are isolated. Coverage includes nonzero child exit,
reaping/restart, no duplicate child during startup, process-state reporting,
and not-run shutdown. A bounded outer subprocess catches launch hangs.

On POSIX, the worker is aborted with SIGABRT and core dumps disabled. A paired
legacy-fork case verifies that parent fork hooks and parent-only state are
inherited, while the configured spawn case avoids both. Windows uses process
termination and a stop-signal adapter; its fork comparison is skipped. The
existing startup-recovery workflow runs the POSIX comparison on both maintained
branches. These tests do not run actual navigation or simulate vehicle response.

Raw logs, source hashes, decoded process samples and the original one-minute
analysis remain private in `.analysis/scratch/2026-10-14-kona-process/`.
