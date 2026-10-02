# Startup update recovery — 2026-09-28

The user requested a visible Git pull/reboot action after a failed build or
manager startup, then authorized automatic polling until connectivity and a fix
are available. The launcher now routes failed runtime dependency preparation,
Params builds, SCons/model builds, Params compatibility checks and manager startup
to one recovery screen. Each command's output is retained in
`/tmp/carrot_startup_failure.log` (bounded to 64 KiB); the original exit status
survives the output-capture pipeline. Build spinners close even on exceptions.

The recovery screen shows Korean first, English underneath, the recovery address
on port 6999, a short error excerpt and a large **Git pull 후 재부팅 / Git pull &
reboot** action. It uses Raylib and selected TTF glyphs without importing Params,
cereal, the normal application framework or vehicle models. C3/C3X and C4 use
their respective display dimensions. The display is awakened when available.

The first automatic check begins after five seconds. Completed failures and
no-change results retry after thirty seconds; an in-progress network operation
has its own bounded timeout. The updater fetches the current branch's configured
upstream and fast-forwards to the verified commit under the shared repository
lock. It never selects a new branch or hard-resets local files. Network failure,
local modifications, divergence, a busy lock or failed update leaves recovery
active without rebooting. Restoring connectivity permits the next attempt.

Automatic reboot requires a newly applied commit, preventing a loop on the same
broken revision. A failed reboot request after an update can be retried while
the recovery UI remains running. The manual button can explicitly reboot after a
successful up-to-date check. Only one worker runs at a time; the UI stays responsive
during Git operations. A new commit is not evidence that the next build will pass:
if that revision fails too, recovery waits for another revision.

The failed startup has ended before the launcher's inherited build lock is
released. Normal driving services are not started from the recovery path.
Manager/build handlers defer to the launcher, so import failures also reach it.
For standalone invocations, TextWindow now uses the current Python interpreter
and stops waiting when its child exits with any code. This fixes a dead-child
wait; it does not prove the cause of the user's logo-only display.

If the recovery display itself fails, a standard-library-only update attempt
runs before waiting thirty seconds and retrying the display. Existing web
recovery stays available. A broken Python interpreter, Git installation, display
driver or OS can still prevent parts of recovery; this is not an OS installer or
an automatic Wi-Fi credentials setup screen. The AGNOS updater keeps its existing
separate verification, network retry and installation policy.

Validation uses temporary local Git remotes for successful updates, unchanged
revisions, network loss/restoration, divergence, dirty files, repository locks,
reboot failure and repeated taps. Shell checks cover preserved failure status and
boot ordering. Hidden desktop renders verify Korean/English layouts at 536x240
and 2160x1080. These do not validate physical C3/C4 touch, DRM/display takeover,
actual device reboot or a complete on-device startup failure/recovery cycle.

This change adds no setting. Existing installed devices need this launcher/UI
revision installed once before they can use the new automatic recovery path.

The subsequent user-supplied log completed SCons successfully and started the
manager, but `dm2d` crashed because its SubMaster call supplied both `poll` and
`frequency`. Commit `fd37eb15d0` removes the latter, matching stock DM in the
locally available comma/master `a86351c3ef`. The 50 ms update timeout and 20 Hz
Ratekeeper remain, allowing absent-camera fallback to continue. Enforcing the
actual constructor constraint in the simulated dispatcher reproduced ten
failures before the fix; all 107 adapted policy/parser/dispatcher tests passed
afterwards. These adapters do not exercise native IPC or a vehicle boot.
A child-process crash while the manager remains running does not enter the
startup recovery path and must not cause automatic Git/reboot during driving.

The same log's camera opcode 266 is `CAM_SENSOR_PROBE_CMD`. Both stock and this
branch try alternate sensor models during discovery, so errno 19 at that point
alone does not establish a failed camera stream. The excerpt ends before it can
establish camera health; no camera fix or on-device recovery is claimed.

Startup recovery validation passed 46 focused recovery, launcher, bootstrap UI,
spinner and model-build-cache tests. The initial broader run lacked SCons on the
Windows host; installing SCons only in the disposable test dependency directory
allowed that test to pass. Lint found no new diagnostics; five existing build.py
diagnostics are outside these changes.

### 2026-09-29: delayed tmux output after successful startup

On Ioniq 5 C4 `07b62e389ed26c81` at `ef6f56d3`, manager and its
services were running while tmux output arrived in bursts. The manager's
`unblock_stdout` relay now writes into the startup-capture pipe instead of
directly into a terminal. Its Python stdout therefore uses block buffering;
the capture process's own flush cannot release bytes still held upstream.
Flush each relayed chunk inside the existing nonblocking/error-handling path.
This preserves scheduling, process startup, and recovery behavior.

An isolated Python/PTY probe on the same device extracted the existing relay
function and compared it with the flush added. With stdout piped and a child
printing one flushed marker before sleeping for one second, the original
delivered no marker within 0.6 seconds (nor at exit, because the relay uses
`os._exit`). The corrected relay delivered the marker within that window;
both exited successfully without stderr. The running vehicle manager was
not changed or restarted. Full launcher behavior after updating remains to
be checked at the next startup.

The user then reproduced delayed manager status lines after restarting on
`20e0775e`, with the relay flush present. The first probe explicitly flushed
the child print, so it missed a second buffer: Python configures the manager's
stdout while fd 1 is still the capture pipe. `forkpty()` changes fd 1 to a
terminal but does not update that existing Python stream's buffering policy.
Reconfigure the child stdout for line buffering after `forkpty`, retaining
the parent relay flush. A second isolated probe on the same device used an
ordinary unflushed `print`: the existing fix delivered it only at child exit,
while child line buffering delivered it within 0.6 seconds. The regression
test exercises the actual relay function with ordinary print and piped output.
This corrects the incomplete first fix; the live manager is not hot-patched.

### 2026-10-02: preserve startup errors and offer an offline rebuild

The update result previously replaced the startup diagnostic (`detail or reason`).
After the first automatic check, the screen therefore showed only "No new commit"
even though the launcher had captured the failure. The display now keeps the
startup error in its own four-line area in every recovery state, with the update
or cleanup status underneath. It reads the bounded 64 KiB failure log, strips ANSI
color escapes and starts at the first recognized compiler/Python error. Remaining
lines, including preceding context, stay available by tapping the error area to
cycle pages. Long lines wrap rather than losing their suffix. If no log is
available, the launcher's failed-stage reason remains visible.

The new **리빌드 · 재부팅 / Clean build & reboot** button runs `scons -c`
(`--minimal` on AGNOS), then removes the `prebuilt` marker and requests reboot
through the existing sound-aware helper. SCons owns the generated-target/cache
cleanup; the next normal startup builds the current checkout. This action needs
no network, does not fetch or reset Git, and preserves source edits. It is offered
for all startup failures, since a stale native build can also cause manager import
failures; its presence is not a diagnosis that rebuilding will fix the error.

Cleanup shares the repository lock with Git and startup, has a 180-second command
timeout, and blocks repeated rebuild/update requests while active. Failed cleanup
does not request reboot or remove `prebuilt`; partial SCons cleanup can already
have occurred. Cleanup/lock/reboot failures leave a retryable recovery screen.
The original startup error is preserved separately from those action failures.
The existing automatic update policy and same-revision reboot-loop protection
remain unchanged. There is no automatic repeated clean/rebuild loop.

A stale generated build can be repaired by rebuilding; a source defect needs a
fixed revision, and missing dependencies need their own repair. Git hard reset is
not a general build repair and is deliberately absent from this screen. A failing
SCons configuration can also prevent its clean action; this remains visible and
does not trigger broader file deletion as a fallback.

Validation passed 75 focused recovery, bootstrap UI, launcher, model-cache and
reboot-sound tests, plus Ruff and whitespace checks. Coverage includes diagnostic
retention and wrapping, all display states, offline
cleanup ordering, source preservation, cleanup failure/timeout, lock contention,
reboot failure, and mutual exclusion between rebuild and update. C4 536x240 and
C3 2160x1080 desktop previews use a synthetic compiler error after the "No new
commit" result. These checks do not establish physical touch/readability, actual
device SCons cleanup/rebuild, or a successful vehicle reboot/recovery cycle.
