# Native ATC right-lane request and road-dependent distance

The first ATC right-lane request in route 1209, segment 10, lit the physical
indicator but did not enter lane-change preparation. Native Kick recorded that
lamp as isolated display output, so DesireHelper suppressed the feedback needed
by the existing TBT lane-change path. A later front-gap failure then cancelled
the request. Extending the flash duration would not correct that classification.

## Recorded evidence

The route runs commit `77dd33357a73c78745f7c27c9656e50da3c8ec40`, with
AutoBlinkerControl=2, AutoTurnControl=1, AutoTurnControlRightLaneChange=400,
CanfdBlinkerKick=1, CanfdBlinkerKickMode=1, LaneChangeDelay=0 and
LaneChangeNeedTorque=0. Times below are relative to the first carState message
in the segment, rather than the boot-time initData timestamp.

| Time | Observation |
|---|---|
| +6.119s | Native right request starts. |
| +6.577s | First SPAS submission recorded. |
| +6.669s | Model-time replay sees actual right lamp, isolation mask 2, valid neutral independent lever and available right lane. |
| +6.970s | Native request ends with healthy inputs and source continuation false. |
| +6.971s | Latest right-front target is 16.904m away at -4m/s relative speed; its three-second projection is 4.904m, below the existing 5m threshold. |

The full rlog and qcamera were retrieved read-only and their SHA-256 hashes
matched the originals. Rlog:
`e57bb67d9028ea4da39102d3640c22d6178a76cee18f7ae643e9750a6ef0a1c3`.
Qcamera:
`e4d83f0dae18f06d7dfb6d4456102f4dd7e2b0a705cc4720d1f9324acc8ca04d`.
Private recordings and analysis scripts are retained outside the repository.

## Correction

The source identifies only an admitted TBT right-lane request started at or above
30km/h. Kick binds a short permission to that accepted hold ID and renews it in
the existing isolation journal while ownership, request and inputs remain valid.
It still waits for the model's isolation acknowledgement before sending lamps.
Another indication's mask cannot be promoted to lane-change intent. Cancellation
or handover removes the permission while preserving isolation of remaining
flashes; absence of renewal expires permission after 200ms.

DesireHelper accepts the real right lamp only with that permission, valid input,
a valid neutral independent driver lever, no manual signal or steering input and
speed at least 30km/h. Ordinary lane-change admission remains responsible for
starting. Manual/model display signals and requests started below 30km/h retain
their existing isolation. Four-second start wait, 15-second overall limit,
front-gap/BSD and driver cancellation are retained. End diagnostics now include
the TBT phase and distinguish front-gap cancellation from start timeout.

The setting maximum changes from 500m to 2000m. The existing navigation road
classifier requires a connected, fresh schema-v1 session, explicit valid road
category 0 or 1 and no off-route indication. Its message timeout is 2s and lane
road-class age limit is 10s. Only then can the selected distance exceed 500m.
Other roads and missing/expired classification use min(setting, 500m). Lane
count alone is insufficient. Default off, legacy ON-to-250m migration and
existing stored distances are preserved.

## Verification and limits

The production TBT policy reconstructed its first request from +6.063s to
+6.971s and cancelled for the recorded front-gap failure. Its start differs
from the recorded native request by about 56ms because replay runs on model
publications rather than the original source scheduler.

With the same recorded car/model/navigation inputs, production DesireHelper
from the baseline does not initiate that first request. The candidate, given
the request-bound permission during the reconstructed TBT window, enters
preLaneChange at +6.669s and laneChangeStarting at +6.720s. This comparison uses
recorded movement and lamps, latest inputs at model publication, and a simulated
permission journal. It verifies admission, not closed-loop vehicle motion,
physical completion, future radar predictions or road acceptance. Native
permission production/renewal/revocation is separately exercised in tests.

Local Windows validation: 1562 distinct related checks pass; two platform skips
and 26 Linux native-safety cases are excluded. The host wrapper supplies isolated
Params/IPC and Windows clock/hardware compatibility, not driving-policy changes.
Tests cover native and external source loops, actual DesireHelper admission,
BSD/inactive/driver/low-speed/stale/disabled rejection, ownership acknowledgement,
request expiry/cancellation/handover, display-only isolation, road-class freshness,
distance limits and existing indicator/voice paths. Wiki generator tests: 25 pass;
user guides and localized setting explanations are synchronized. Linux native CI
runs after publication. No vehicle software/settings writes or actuation were
performed.

Published-code validation on `bd6e537450`: Linux signal integration, Jetson
integration, user-doc validation and Wiki publication pass. Linux signal jobs
report 2403/131/931/32/406 passing checks, with 40 skips and 365 subtests across
those jobs; these counts overlap the local runs. The web settings test initially
retained the old maximum of 500m. Updating that expected catalog limit to 2000m
passes all 31 web-control checks locally. No production change is needed for
that test correction; the final full build/web workflow runs on its follow-up.
