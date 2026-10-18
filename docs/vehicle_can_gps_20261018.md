# Vehicle CAN position and GPS source priority — 2026-10-18

The requested position priority is external navigation, connected phone GPS,
verified vehicle CAN position, then device GPS. Every source retains its own fix
and original receipt time. A lower-priority packet cannot overwrite a selected
higher-priority fix. Expired or invalid fixes fall through automatically; source
changes print `[GPS] source=...` without recording coordinates in that message.

## Evidence and limits

Two retained Ioniq 5 PE drives, 20 segments, contain 1,195 original receive frames
on bus 0, address 0x374, length 8. Bytes 0–3 and 4–7 are little-endian positive
latitude/longitude integers scaled by 1e-6 degrees. Within-segment message gaps
are 0.9813–1.0198 seconds. The initial seven-segment check found 420 receive
frames, 420 corresponding bus-130 forwarding/transmit receipts, and zero
application `sendcan` frames at this address. Forwarded receipts are not
independent position samples. Coordinates change even in segments without a
working device GPS or external-navigation position.

Four of those segments overlap external-navigation coordinates: unshifted
position differences have segment medians around 7–12 m and p95 around 23–28 m.
These are agreement measurements, not surveyed accuracy; update latency,
map-matching and receiver identity are not established. This is a position
broadcast, not proof of a particular source ECU or raw satellite solution.

Nearby 0x371/0x372/0x373 byte-aligned 16-bit heading candidates, in both endian
orders and several common scales, were compared against moving coordinate
courses. None was convincing (best median circular error about 72 degrees).
This limited search does not prove there is no heading elsewhere. No OEM fix
quality, satellite count, accuracy, altitude, or heading field is claimed.
Southern/western hemisphere encoding has not been established. The older
Hyundai 0x570 GPS definition is not applicable to the observed 32-byte message.

## Implementation

- Following the user's scope correction, eligibility is based on the actual DBC,
  not a car-name allowlist. All Hyundai/Kia/Genesis platforms using the matching
  definition register the optional profile on their existing powertrain/ECAN
  parser. Both CAN-FD and legacy parser paths check the DBC; the current legacy
  DBC lacks this profile and stays inactive. Require the named message, address
  0x374, length 8, and both unsigned little-endian 32-bit coordinates at bits
  0/32 with scale 1e-6 and zero offset. An unrelated numeric ID is insufficient.
  Missing traffic never creates a mandatory CAN fault. No CAN transmission,
  Panda safety, locationd, vehicle control or model changes. Real driving
  evidence remains Ioniq 5 PE only; shared DBC eligibility does not prove every
  vehicle actually transmits valid coordinates.
- Require three consecutive plausible samples, 0.5–2.5 seconds apart, valid
  8-byte positive-coordinate layout, and motion consistent with integrated wheel
  speed. Step distance must not exceed either 90 m/s or 15 m + twice integrated
  travel. Reject stale (>2.5 s), malformed, nonfinite-speed and implausible data.
- Newly repeated coordinates after more than 15 m of wheel-speed travel are
  invalid until coordinates change. Normal motion between 1 Hz samples is not
  a freeze. This is a plausibility policy, not an OEM GNSS quality flag; plausible
  incorrect coordinates and small continuous drift cannot always be detected.
- Course is **derived**, only after at least 5 m between accepted positions and
  speed >=2 m/s. It is not IMU yaw or a decoded vehicle heading. Reverse disables
  it. Its age is bounded to 2.5 s while moving and 30 s below 0.5 m/s. Course can
  lag turns; this work does not solve high-rate orientation or GPS latency.
- Unknown direction permits raw position but no guessed extrapolation. On
  carrot-nav, the model-map consumer requires a valid direction, otherwise uses
  its existing usable-device fallback or waits. It never treats unknown as north.
- Phone position requires finite reported accuracy <=30 m and a fresh packet;
  connection is inferred from GPS packet receipt, not Wi-Fi association alone.
  Legacy phone speed units remain km/h. External/phone/device source age limit is
  3 s; vehicle limit is 2.5 s. Existing nav model-map freshness remains stricter.
- The wip port includes the required original external-navigation receipt and
  sequence handling already present on nav. It preserves wip's navigation/model
  architecture; nav-only rendering helpers are not introduced into wip.

## Validation

Windows desktop tests use real Cap'n Proto schemas and the repository CAN parser;
Params/hardware/messaging OS bindings are doubled. Nav: 188 passed, 9 native
rasterizer tests skipped because the native renderer is not built. Wip: 130
passed, with its existing model architecture. Tests include
priority/handover, receipt ageing, malformed coordinates, wrong buses, missing
optional messages, acquisition, reverse, unknown course, high-speed inter-sample
motion, long stationary periods and moving frozen coordinates.

Recorded-input replay uses all 119,551 CAN events in the 20 segments and recorded
wheel speed, maintaining state across segments within each drive. Of 1,195
position samples, 1,166 pass: six are acquiring/reacquiring and 23 repeat frozen
coordinates during motion. Derived heading is usable at 829 samples. There are
116,544 valid-position and 81,985 valid-heading CAN-update observations. This is
an offline receiver replay, not a closed-loop driving or nav-model validation.

Measured desktop mean monitor-call cost per segment is 2.39–3.79 microseconds,
excluding parser, serialization and IPC. This does not establish vehicle CPU
load or 20 Hz inference stability. Vehicle installation/restart and native
performance tests were not performed in this task. Raw logs and coordinates
remain in the private analysis archive.

## DBC eligibility follow-up

The user explicitly removed the Ioniq-only restriction. The source-priority,
coordinate/heading validity and freshness policies above remain unchanged.
All current CAN-FD platform parser configurations acquire the profile with
synthetic receive traffic on their configured bus, including arrival after
startup. Legacy DBC absence and nine incompatible-layout cases remain inactive.
Follow-up desktop checks: nav 155 passed; wip 134 passed (same focused source/
parser suite, without nav-only position consumer tests). Ruff and diff checks
pass. Existing 20-segment evidence is unchanged; no additional vehicle install,
other-model drive validation or native CPU measurement was performed.
