# DM2 implementation and validation - 2026-09-28

## Revised user contract

The user's follow-up supersedes the initial implementation's 5/15/25 fallback,
2x-to-2.4x wheel timing, two-second camera input credit, protected distraction
debt exclusion, 1.5x forward recovery and ten-second traffic window. The user
explicitly selected delaying camera warnings for the unavailable-camera mode-1
allowance after input, rather than immediately resuming camera evaluation.

- Camera mode 0: stock comma monitoring criteria and internal face-loss fallback,
  with the explicitly requested confirmed-parking reset below.
- Unavailable-camera mode 0: 15/30/45-second interaction alerts.
- Unavailable-camera mode 1: same timing, doubled only with verified empty-road
  conditions; eligible input resets the entire allowance before terminal alert.
- Camera mode 1: 10/16/26-second vision alerts; 20/32/52 with empty-road conditions.
  Head-pose tolerance is 20% wider before orange. Sleep, blink and phone detection
  thresholds are unchanged, but warning timing for those detections is extended.
- Fresh driver/BT input in camera mode 1 resets monitoring and starts 45 seconds
  of camera-warning grace, or 90 seconds with empty-road conditions. Detections
  continue during grace, including sleep/eye/phone, but the warning budget remains
  full. After grace, the camera clock starts. Under sustained distraction, this
  can place the first warning around 55/110 seconds and terminal around 71/142
  seconds after input. These are requested experimental choices, not legal limits.
- Confident forward attention for two seconds resets camera mode 1 without
  renewing the interaction grace. While monitoring remains enabled, terminal
  alerts and lockout cannot be reset by ordinary input, forward attention,
  camera recovery or empty-road expansion.
- DriverMonitoringEnabled is persistent and defaults on. It is exposed only
  through Carrot Web search for absent or failed DM-camera cases; leaving it on
  is recommended. Turning DM off may violate applicable laws or driving
  requirements, which depend on the jurisdiction and use conditions. Regardless
  of gear or speed, including at standstill, three distinct physical vehicle CANCEL
  presses, each separated by a release, within three seconds turn it off.
  Automatic-control CANCEL echoes and BT CANCEL do not count. Another received
  non-CANCEL button event, invalid/stale state, input-stream gap or timeout resets
  the sequence. The
  2026-09-29 follow-up removed the gear and speed restrictions; gear and speed
  changes, including stopping, do not reset it by themselves.
  Ambiguous stock-ACC speed-button echoes conservatively reset it because their
  source cannot be distinguished. The three-CANCEL session marker stops the
  model during normal onroad operation and blocks alerts, monitoring-triggered
  forceDecel and lockout until the next ignition-on or manager/device restart.
  That boundary clears only the session marker; monitoring resumes only if the
  saved DriverMonitoringEnabled setting is on. A Web OFF remains off until
  manually enabled again. Driver View retains face preview with neutral
  enforcement.

## Stock boundary and state transitions

### Confirmed parking reset (2026-09-28 follow-up)

The user authorized a reset after parking. All four camera/mode combinations
reset accumulated terminal/no-response counts, lockout, alert level and awareness
after one continuous second of Park, reported standstill, raw speed exactly zero,
finite filtered speed below 0.01 m/s in magnitude, and both enabled/active false.
The tiny filtered-speed tolerance only accommodates Kalman settling; it cannot
replace the raw-zero/standstill/Park checks. Both carState and selfdriveState
must pass validity, liveness and frequency checks and have ages in [0, 0.25) s;
CAN must be valid. Demo mode is excluded. Failed checks, backwards time and loop
gaps over 0.25 s restart confirmation. No speed-only or engage-cycle reset is
added, and no automatic engagement occurs.

One reset is emitted per confirmed parking interval. Stored DriverTooDistracted
is synchronized by its existing selfdrived owner from fresh healthy DM packets
on both transitions; restarting DM cannot resurrect a cleared stored flag once
the release has been received. A crash before that acknowledgement remains
conservative. Renewed lockout can be persisted again. Existing thirty-minute
recovery also benefits from this symmetric persistence instead of keeping a
stale true flag. The policy.py implementation and running warning thresholds
remain unchanged; mode-0 equivalence excludes this parking reset.

Tests cover all modes/camera availability with and without AlwaysOnDM, moving or
non-P gears, invalid/stale/future data, partial parking/gaps, saved-state recovery
and repeated locking, and daemon publication. 146 adapted tests pass; real cereal
types are used, with native IPC/Params/hardware adapted on Windows. This does not
establish physical vehicle parking or on-device restart behavior.

Stock monitoring/policy.py and monitoring/dmonitoringd.py remain unchanged. The
manager runs the separate dm2d dispatcher at the existing scheduling placement.
DriverMonitoring2 modifies its own settings instance, retaining normal camera
mode-0 equivalence outside the parking reset. Models, artifacts and inference behavior do not change.

Camera absence, failure, malformed probabilities/vectors and stale output select
automatic interaction fallback. Two continuous seconds of healthy samples restore
camera monitoring. Source transitions preserve fractional monitoring progress and
strong alerts; they do not preserve identical absolute seconds across different
source budgets. No manual camera-installed setting is required. Road/wide camera,
CAN, process and other control health checks remain independent. A shared camerad
failure is not rendered harmless, and this does not add hardware hotplug recovery.

Within one source, changing traffic context preserves elapsed attention seconds.
Expansion cannot erase orange/red. A shortening that crosses terminal counts the
terminal transition once. Fresh input may reset before terminal; once terminal
is reached the stock disengagement/lockout behavior remains while monitoring is
enabled. The persistent switch and separate session disable sequence gate the
complete monitoring path rather than resetting policy awareness. A grace that expires cannot reopen
merely because the road clears later. Mode 0 camera inputs stay stock; mode 1
uses fresh edges rather than a continuously held steering/gas state.

## Traffic and input evidence

DM only consumes existing radarState leads and lists. No radar detection, selection,
shared replay adapter or radar model is changed, so this task does not deploy a new
NAS radar replay algorithm. Appended fields describe DM only.

Moving observations use absolute ground speed >=2 m/s, longitudinal position
-10..150 m and path lateral distance <=6 m. Equal-speed lead traffic counts.
Stationary/slow observations are intentionally excluded; this is not proof of an
obstacle-free road. A candidate immediately removes the empty-road bonus; 0.2 s
of continuous confirmation starts a 20-second hold. Position association, relative
velocity projection, deduplication and two-second dropout retention prevent each
frame of one car from retriggering. A single spike cannot start the full hold.
These gates reduce some noise; they do not establish physical target identity.

Empty-road doubling additionally requires healthy radar/model, supported enabled
Hyundai/Kia corner coverage and ten seconds of stable straight-road conditions.
A continuously present moving object blocks this condition after the hold ends.
Unknown coverage, stale input, curves or invalid observations cannot earn it.

CarState is drained without conflation to retain short control edges. Stock-ACC
speed-button injection configurations exclude ambiguous vehicle speed buttons
from interaction credit. They still reset a pending disable sequence because an
automatic echo cannot be distinguished from a physical button that breaks three
consecutive CANCEL presses. Other eligible controls and the existing independent
BT attention journal remain. Learning/test, stale/startup/cancelled events and
long-press automatic repeats do not count. Automatic ego/set-speed changes are
never driver interactions.

The monitoring-disable sequence consumes fresh physical CarState button press
edges independently of the BT attention journal. A held CANCEL, repeated CAN
packets and BT CANCEL cannot advance it. Received CANCEL presses immediately
following openpilot's own control cancellation request are treated as automatic
gateway echoes and excluded. This is a 150 ms correlation against the request,
not confirmation that the controller transmitted CAN; a physical CANCEL that
overlaps the request window can therefore be conservatively ignored and must be
pressed again. Hyundai/Kia/Genesis openpilot-long bypasses this correlation:
its controller does not transmit a CANCEL button from the longitudinal-control
path, although `carControl.cruiseControl.cancel` can remain high while engaged.
This prevents that internal level from hiding the first physical wheel press.
Stock-long Hyundai and other platforms retain the echo filter. Suppression of
the paired automatic release expires after 0.5 seconds. A physical press/release
interleaved with a filtered echo can therefore make the following press require
one extra release and press, while still preventing an automatic echo from
contributing to the disable sequence. The sequence is
eligible with valid, fresh vehicle state regardless of gear or speed, including
at standstill. Gear and speed changes do not clear the pending count. Any other
received non-CANCEL vehicle-button event, whether a press or release, clears the
pending count. Stale or invalid state, loss of the input stream, or expiry of the
three-second window also clears it. Each counted CANCEL requires a release before
the next press.

## Configuration, diagnostics and documentation

### Persistent enable and session disable switches (revised 2026-09-29)

`DriverMonitoringEnabled` is a persistent boolean that defaults to true. Carrot
Web exposes it through setting search only, rather than the ordinary menu. This
switch is intended for an absent or failed DM camera; the recommendation is to
leave the default ON unchanged. With the setting on, an unavailable camera still
automatically selects interaction monitoring. Turning monitoring off removes
that fallback too and may violate applicable laws or driving requirements.

The three-CANCEL gesture sets the separate internal
`DriverMonitoringSessionDisabled` marker without changing the persistent setting.
Manager start and ignition-on clear this session marker only. Monitoring is
enabled when `DriverMonitoringEnabled` is true and the session marker is false.
A saved Web OFF survives later drives and restarts until the user manually turns
the setting on again. Both switches are independent of the live
`DriverMonitoringMode` and `CarrotVisionEnabled`; neither rewrites those values.

During normal onroad operation, the disabled state stops driver-model execution
and publishes a valid neutral monitoring heartbeat, so no driver-monitoring
alert, force-deceleration request or lockout is applied. Driver View is the sole
model-execution exception: after camera health recovery it may populate face and
pose data for preview, while the published alert, awareness, force-deceleration
and lockout fields remain neutral. Restoration constructs new runtime
awareness state under the stored mode rather than treating restart as attention
evidence. Any existing ignition-cycle `DriverTooDistracted` lockout remains
persistent and is reloaded across a manager restart in the same ignition cycle.
The CANCEL sequence is therefore a session monitoring switch, not another
awareness reset and not a way to select mode 0.

### Original steering touch input (2026-09-28)

The later user request removes the initial Ioniq 5 PE-only whitelist. All
Hyundai/Kia/Genesis CAN-FD vehicle configurations now admit the same received
`STEER_TOUCH_2AF` profile. `CarState.steeringTouch`
records availability, validity, contact, original CAN timestamp and raw status /
TOUCH1 / TOUCH2. It reads the existing ECAN parser's raw bytes, never the mutable
forwarding cache, CAM input or Panda transmit receipts. Existing ADAS transmit
code, safety rules and global torque-based steeringPressed remain unchanged.
After an original frame is seen, an unregistered named message is registered
with optional frequency so late arrivals work after startup fingerprinting.
Absent hardware or later dropout cannot create a new CAN-liveness requirement.
The decoder does not populate the ADAS forwarding cache. The DBC message name,
address and size must match; numeric address 0x2AF alone cannot enable touch.
This bus distinction does not authenticate a sensor against
another device injecting frames onto ECAN.

Six one-minute historical segments yielded 3,597 original 10 Hz frames. All fit
the receive checksum profile (CRC polynomial 0x1D, initial register zero, xor
residual 0x32 across bytes 1..7) and a high-nibble counter 0..14. Three newer
segments established the profile and three older segments independently checked
it. This is empirical compatibility evidence, not an OEM protocol specification.
The existing transmit checksum function differs and was not changed.

Status 0 had raw TOUCH1 12..17 / TOUCH2 14..18; contact status 1 already included
TOUCH1 15 and TOUCH2 14 on different samples. Raw ranges overlap, so a baseline
plus one threshold is not established by these logs. The user confirmed that
zero means released and a rising value represents touch. The decoder accepts
the lowest reported contact status 1, through observed status 4. All 313 samples
with logged steeringPressed were in nonzero status, including turns; this is
corroboration, not hand-labeled physical-contact ground truth.

The decoder requires the observed eight-byte layout, checksum, known status,
successive counters and sample age <=250 ms. Startup/recovery requires two
consecutive valid frames; malformed, repeated-counter, unknown-layout or stale
input grants no contact. Replay accepted 3,591 frames after the six initial
counter baselines, including 1,015 contacts. No new mandatory CAN checks are added.

The profile-based follow-up passes 184 adapted policy/parser/dispatcher tests,
including real CAN parsers for all 37 configured CAN-FD platforms, discovery
after startup, wrong-bus/TX receipt rejection, an unrelated DBC and optional
message dropout. These are synthetic platform checks. Physical/log-derived
touch evidence remains the six Ioniq 5 PE segments above, not a fleet-wide
verification that every vehicle uses the same profile.

Without a usable camera, fresh continuous contact maintains wheel awareness
before terminal alert in both modes. In camera mode 1 only a valid release-to-
contact transition grants the existing interaction grace. Held contact and
recovery while already held cannot renew camera grace. Camera mode 0 ignores
this added signal, and touch does not change terminal/lockout handling. No claim
of gaze, sleep detection, legal certification or new-vehicle validation follows
from capacitive contact or these desktop/log checks.

DriverMonitoringMode is read live every 0.5 seconds: only value 1 is experimental. Old
DisableDM never opts users into mode 1; only its old video choice migrates once to
independent CarrotVisionEnabled. The experimental-use confirmation remains.

New dm2VisionTimeoutFactor and dm2InteractionGraceRemaining fields complement the
existing wheel factor and traffic hold. dm2ForwardRecovery now identifies a full
forward-attention reset, and dm2InteractionCredit reports the full seconds restored
by an input, not a fixed two-second credit. Historical packets retain their earlier
semantics. While monitoring is enabled, camera-unavailable notices and terminal
forceDecel connections remain; this is not guaranteed emergency stopping or
equivalent stock-ACC deceleration.

Korean/English guides explain the search-only persistent switch, the session
switch, physical CANCEL sequence, conditional restoration, all four monitoring cases,
grace composition, delayed sleep warnings, moving-target exclusions and terminal
limits. Wiki MANUAL explanations are updated separately through the existing
generator.

## Validation and limits

- Focused disable-sequence coverage checks the inclusive three-second boundary,
  release-to-press edges, held packets, lost releases, timeout restart, other
  button reset, invalid/stale state and automatic CANCEL echo rejection.
  Dispatcher coverage checks that the third fresh physical CANCEL press writes
  the session marker, retries until observed and publishes a valid neutral
  disabled state. Manager coverage clears that marker at ignition-on or manager
  start and keeps dm2d alive while
  stopping ordinary onroad model execution. Disabled Driver View coverage keeps
  face preview while driver-state consumers gate alerts, lockout, forceDecel and
  the onroad driver-monitor indicator from the published disabled state.
- Stock/DM2 policy, daemon-adapter and Bluetooth tests cover
  all four modes, exact timing stages, sustained sleep/eye/phone detection, actual
  dispatcher BT grace, reset/expiry, context shortening, source recovery, terminal
  counters, lockout and stock camera mode-0 equivalence.
- Traffic/context/config and settings-schema tests cover noise
  persistence, twenty-second entry hold, dropout association and empty-road gates.
- Focused Python lint passed. Stock policy and daemon content remain unchanged.
- User-docs and Wiki validation accompany publication; generator tests are run.

The Windows adapter uses real cereal schemas, policies and numpy, but substitutes
native messaging, Params, hardware, process-title and flock interfaces. It does not
validate Linux IPC or exclusive evdev ownership. No device native build, hardware
camera-loss trial, C3/C4 timing, physical input provenance or driving validation was
performed. Neither mode is legal certification. Reproduction scripts and reports
are retained locally under .analysis/archive/2026-09-28/dm2-revision/.

Public guides: [Korean](user/ko/driver-monitoring.md),
[English](user/en/driver-monitoring.md).


## Live mode switching (2026-09-28)

The running dispatcher polls the typed mode parameter every 0.5 seconds before
configuring the current camera/traffic context. The retired CARROT_DM_MODE
environment latch is ignored and removed at manager initialization. The existing
monitor, calibration, interaction-edge history, traffic hold, elapsed awareness,
terminal counters and lockout are retained. The published dm2Experimental reports
the applied mode, not the initial startup choice.

Actual mode changes expire old interaction grace and the forward-attention streak;
re-reading an unchanged setting leaves these intact. configure_context remaps the
active budget by elapsed seconds, so shortening a budget can cause a warning
immediately. Existing orange/terminal alerts cannot be erased by increasing the
budget. Repeated toggles neither reset accumulated time nor resurrect old inputs.
This does not change either mode's thresholds, existing attention recovery,
confirmed-parking reset, or the experimental-use confirmation.

Validation uses real policy/cereal with desktop adapters for native IPC/Params;
device polling latency and physical driving behavior remain unvalidated.
All 208 focused monitoring/touch tests and 25 Wiki generator/validator tests
passed. Four native typed-Params migration cases were skipped on Windows; the
strict fake-Params migration tests passed. Local evidence is retained under
`.analysis/archive/2026-09-28/dm-live/`.
