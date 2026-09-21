# VW MEB distance alignment

On September 21, 2026, an ID.4 approach log showed a switch from the close lead
to a farther CAN object before the driver braked. Adding the legacy 0.8-second
radar delay to distance projection moved the farther object's distance toward
the vision lead. These MEB CAN objects are filtered OEM camera object output,
not raw radar detections.

The user approved zero **extra distance delay** for Volkswagen MEB. The shared
`radar_motion/timing.py` policy is used by radard, the planner overlay and log
replay. Camera exposure versus liveTracks publication skew is still applied;
zero extra delay does not disable timestamp alignment or stale-input rejection.
Corner timing and other platforms' distance delay are unchanged.

`CP.radarDelay` remains 0.8 seconds. RadarInterface uses it for ego velocity and
acceleration history, which is a separate calibration. CAN decoding, the lead
filter, actuator delay and planner cadence are not changed here. This is not a
claim that the physical sensor latency is exactly zero.

## Evidence and validation

The historical corpus contains 19 ID.4 segments from August 31 to September 15.
A fixed-window screen retained 28 six-second moving-distance windows in 14
segments. Mean absolute CAN/vision distance differences were 1.535, 1.515,
1.505 and 1.860 metres for additional shifts of 0, 0.08, 0.15 and 0.8 seconds.
Vision bias confounds exact delay fitting: some windows favor 0.8, and the small
aggregate advantage of 0.15 over zero does not establish that physical delay.

On the September 21 incident's 0.4-6.7-second approach, legacy 0.8-second replay
selected farther track 619 for 77 of 126 frames. Zero extra delay retained track
613 for all 126. Current fusion replay of old inputs generally retained the
same target despite increased distance error; historical good behavior does
not validate the large projection. This is not a full replay of historical
CAN preprocessing or closed-loop braking, nor proof that process isolation
caused the incident.

Regression coverage includes the close/far candidate confusion, retained
publication/exposure skew, unchanged kinematics and CP value, MEB flag scope,
non-MEB behavior, and NAS loading of CarParams before or after model frames.
The radar/lead simulator and planner clock suites must pass, along with the
route vault tests. Real-log acceptance covers the two incident segments,
all 19 historical ID.4 segments, and two Ioniq 9 segments. Historical saved
zero-delay candidate outputs are compared where available; Hyundai fusion
outputs are compared against the unchanged CP-delay configuration.

Desktop replay validates this alignment change on recorded input. Vehicle
braking and comfort still require a new on-road log. NAS deployment follows
the Carrot Routes image workflow and scheduled updater, with SOURCE_COMMIT,
source fingerprint and recalculated incident data checked after deployment.
