# HUD reconnection and camera startup stalls

## Findings

The October 9 live C4 inspection found the TURZX USB device enumerated and
authorized while the HUD process retained a deleted handle for an earlier USB
device number. Its main thread was in hardware H.264 encoder shutdown:
`close -> drain(250) -> process_ready_events -> poll(/dev/video33)`.
Repeated syscall samples, a native stack and a two-second syscall trace confirmed
that messaging SIGUSR2 notifications interrupted each wait before 250 ms elapsed.
The EINTR retry restarted the full relative timeout indefinitely. The process
stayed alive and ClusterHudConnected remained true, despite no working output.
The initiating lifecycle transition and reason for the USB disconnects are not
established by these observations.

A separate startup log recorded persistent driver-camera IFE fence failures
starting about one second after camera startup. No driver frames were published.
Each failed wait took roughly 91–104 ms and recurred about every 200 ms.
All cameras shared one event-dispatch thread, so waiting for the driver frame
also delayed road/wide publication. Their SOF intervals stayed below 60 ms, but
publication gaps reached 133/137 ms; model output fell to about 15.4 Hz while
eGPU worker computation averaged about 37 ms. The next startup and live sample
returned to about 20 Hz. The initial sensor/ISP failure remains unexplained.

The reported early-enable “Gear not D” alert was not present in the three
examined startup minutes. The code can produce it during eGPU loading because
selfdrived's initialization timeout expires before the first model output.
There is no evidence here of Drive being decoded as Park or of successful
engagement during initialization.

Raw routes, settings, tmux, kernel journal and process captures remain in the
private local analysis archive. These incidents are separate from the earlier
route whose autorun log reported that the USB HUD was not discoverable.

## Changes

- Hardware encoder polling retains an absolute monotonic deadline across EINTR.
  Zero-time polling, indefinite waits and error reporting retain poll semantics.
  HUD connection state is cleared before renderer/encoder teardown starts.
- Only the driver-camera IFE/BPS wait runs on one persistent worker. Its eventfd
  wakes the original camera thread to finish processing. Request validation,
  buffer ownership, frame publication, exposure, startup timestamp alignment
  and failure recovery all remain on that original thread. Wait limits remain
  100 ms for IFE and 50 ms for BPS. The worker inherits camera scheduling.
- Driver events stay in order while a wait is pending. A bounded 32-event queue
  discards its oldest entry on overflow and logs it; existing request/raw-ID gap
  validation handles the discontinuity. Shutdown joins the in-flight wait
  before releasing its camera resources.
- During disabled startup, an existing selfdriveInitializing event suppresses
  only the wrongGear/noEntry presentation, including a lingering copy in the
  alert manager. The wrongGear event, engagement blocks, USER_DISABLE alerts,
  fault alerts and actual gear decoding are unchanged.

This isolates a failing driver-camera wait; it does not claim to repair the
sensor/ISP failure, guarantee recovery of that camera, or alter DM fallback.
Normal model loading and initial accumulated skip counts are distinct from the
persistent post-startup publication stalls in this incident.

## Validation

Desktop focused tests exercise initialization/gear blocking and alert selection,
HUD teardown ordering, USB settings, autorun retry and existing performance
safeguards: 85 passed and one existing platform-dependent test skipped.
Seven native regression cases passed on C4. They exercise continuous SIGUSR2 interruption of a
bounded poll; ready, invalid and indefinite poll cases; an unresolved driver
fence while the owner remains responsive; repeated success/failure completion;
worker exception propagation and shutdown resource lifetime.

The modified camera dispatcher, Spectra and hardware H.264 encoder compile with
the running C4's native compiler flags in an isolated temporary directory.
This is compilation and synthetic regression validation, not installation or
physical HUD/camera validation. The running vehicle programs were not replaced
or restarted. Hardware reconnection, ignition-cycle behavior and normal C3/C4
camera delivery still require validation after the update.

Docs-Not-Needed: Internal recovery and startup alert corrections; no setting
values, menu choices or user configuration workflow changed.
