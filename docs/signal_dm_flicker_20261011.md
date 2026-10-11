# Signal GPU / driver-monitoring regression and display continuity

The user's next drive exposed a regression in the previous GPU trial: checking
the driving model alone missed overload of the driver-monitoring (DM) GPU model.
This report supersedes the eGPU-implies-free-internal-GPU assumption in
[the GPU experiment](signal_gpu_dual_model_20261011.md).

## Reproduced cause

Parked C4, engine on, Park, zero raw speed, disengaged, no Panda controls allowed;
the external driving model remained selected throughout. DM was enabled, mode 1,
with no session disable. The driver camera continued near 20 Hz. Repeated live
measurements isolated the QCOM signal worker:

| Signal helper | DM output | DM inference mean | DM camera unavailable |
|---|---:|---:|---:|
| QCOM, first measurement | 10.84 Hz | 80.84 ms | 230/230 |
| QCOM stopped | 19.99 Hz | 28.77 ms | 0/300 |
| QCOM restored | 10.57 Hz | 82.93 ms | 244/288, including transition |
| CPU comparison restored | 20.03 Hz | 28.82 ms | 0/300 |

The driving model stayed near 20 Hz in that A/B/A test. A separate private QCOM
priority-15 trial still produced only 9.80 Hz DM, 86.12 ms inference and 337/381
camera-unavailable reports. Restoring CPU gave 20.00 Hz DM and 0/299 unavailable.
Priority 15 is rejected. DM's service-frequency gate correctly switched to the
existing interaction fallback; the camera/model had not been deliberately disabled.
No DM timing, alert, camera-health or interaction policy is relaxed by this fix.

## Runtime correction

Both `neural_backend=auto` and explicit `qcom` selection pass through the backend
supervisor. QCOM is eligible only with an external driving model AND no requested
DM model or Driver View. Otherwise use the existing lightweight CPU comparison
observer. Poll settings every 100 ms and stop/join the old process before starting
the next one. A failed/unavailable DM camera does not grant GPU eligibility.
This change never disables DM to obtain GPU capacity. DM's startup transient when
independently enabled is not claimed to have zero GPU contention.

The CPU observer is the earlier tracking/color algorithm, not CPU execution of
YOLOX. Its detection limitations remain. No driving or DM model artifact is
changed. The new MDM remains trial-branch-only; carrot-wip keeps Cinque v3.

Fix `start_guard_transport` telemetry to read the flag as a file, rather than
appending `/enabled` to its path. The actual observation publisher already read
the correct flag; the old false log value did not mean transport was absent.

## Latest-drive flicker

Preserved complete segments 0–16 of route 109c privately. Moving segments 10–14
contain 783 neural observations. Segments 11–14 run at 2.33–2.67 Hz; median result
ages are 344–374 ms, maximum 619 ms. Every within-segment observation interval
exceeds the neural sequence's 250 ms reset gate. Its 424 object samples have 424
distinct identities. It therefore reacquires instead of maintaining identity.
DM remains camera-unavailable in every sampled report in these drive segments.

The old 800 ms UI hold is measured from capture, so processing has already spent
much of that budget before a result arrives. A later empty result plus expired
memory hides the box. An overlapping replacement ID can also erase old color.
These mechanisms establish display flicker, not the ground-truth accuracy of the
underlying red/green classifications. No new visual labels are claimed here.

Display revision 4 remembers the last result for at most two seconds from capture.
Only a unique reciprocal IoU >= 0.5 housing match can transfer old color to a new
unknown ID. Original color time is retained: unknown/repeated records cannot
renew it. New red or green replaces it immediately; ambiguous matches do not
inherit color. Previous results remain dim and marked `~`; dead/frozen camera,
invalid transport, session change and backwards frame/time reset memory.
The neural tracker's control-facing reset policy is unchanged.

At a reconstructed 10 Hz display cadence across ten phase offsets, old display
on-to-off transitions are 146–162 versus 14 with revision 4. This counts any-box
visibility, not identity stability or recognition accuracy. Camera capture time
was estimated per segment from road-camera message time minus the median matched
frame publication offset (21–24 ms; individual offsets 12–33 ms). This is an
offline display reconstruction, not a captured physical panel measurement.
Holding screen coordinates while the vehicle moves can leave a temporarily
misaligned old box; the dim/previous marking matters.

The display endpoint is never read by control. The stopped-car red veto keeps its
650 ms observation age, fresh green confirmation, manual gas override and warning
sound. Moving assist stays OFF. A two-second old green on screen is not a fresh
green departure permission, nor a guarantee against missed red detection.

## Validation

Desktop: 163 passed, one Linux-only skip, including DM/preview/external-model
backend combinations, live switch cleanup, explicit QCOM gating, unknown/new-ID
continuity, ambiguity, expiry, camera failure, immediate color replacement, worker
transport and existing camera/performance checks. Windows Params is substituted
for these tests; GPU timing above comes from the native parked device.

Private reproductions and telemetry are retained in
`.analysis/archive/2026-10-11-dm-flicker/`.

Native device: 115 focused display/worker/start-guard tests pass. Pytest reports
three unrecognized optional configuration keys, without test failures.
Installed runtime commit `0bfcb01d13` on `carrot-signal-shadow`; corresponding
shared-code commit on `carrot-wip` is `1042cdd954`. A process-only USB display
restart retained revision 3 through manager preimports, so a guarded normal
manager reboot was required. Post-reboot display status confirms revision 4.

The final 30-second parked measurement has 600/600 valid driver-camera frames,
600/600 valid DM outputs at 20.004 Hz, 27.26 ms mean DM inference, and zero missing
frame IDs. All 599 DM-state reports have cameraUnavailable=false. The driving
model has 600/600 valid frames at 19.998 Hz, 36.80 ms mean inference, zero skipped
IDs. No QCOM signal observation occurs; live CPU comparison observations and
fresh start-guard transport are present. DM remains enabled, mode 1, no session
disable; UsbGpuActive=true. Signal auto/display/start guard are ON, moving assist
OFF. Parked camera availability does not demonstrate actual distracted-driver
warning timing, physical chime loudness or signal recognition on the next drive.
