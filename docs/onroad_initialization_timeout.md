# Onroad initialization grace period

The user requested increasing selfdrived's startup limit from 6 to 10 seconds.
Healthy CAN and subscribed services still finish initialization immediately.
This is the existing frame-count budget (`sm.frame * DT_CTRL`), not a new
wall-clock timer or an unconditional ten-second delay. At the first update
beyond that budget, the existing timeout path permits fault/event diagnosis;
it does not assert actual service health or engage control. Missing camera,
simulation/replay and normal control validity policies are retained.

The six-second constant already exists at the earliest available local history
boundary. There is no documented hardware-derived rationale for that exact
value in the inspected code. Before initialization, selfdrived displays the
initializing event and returns before most regular event checks, explaining
the need for a finite escape path when a service never becomes healthy.

## Additional model startup work

The integrated Jetlink path wraps the internal model before its first output,
even without an external Jetson connection. `JoiningModel.__init__` constructs
`Warp`, warms its reference path, and on C4 prepares and validates the fused
warp against the reference with 27 transforms, 393,216 output elements each.
These synchronous steps are included in the existing total model-load log.
The wrapper came with Jetlink integration; the C4 comparison was introduced
in 4c5be00b8d. Existing total timings do not isolate the duration of each step.

Startup now logs internal-model construction and Jetlink-adapter construction
durations separately, including whether the adapter succeeded. No workload,
pixel comparison, model selection, fallback or connection policy is changed.

The requested trial combines this longer grace period with the separately
committed Xiaoge readiness gate. Required publishers continue starting together
to satisfy their data dependencies; optional Xiaoge waits until healthy onroad
readiness. The ten-second timeout remains finite and therefore cannot guarantee
that every service is healthy at the time of a timeout transition.

## Validation and device comparison

12 focused tests exercise actual data_sample code with message/device stubs:
unready services at 6.01/8.70/9.99/10.00 seconds; immediate healthy readiness;
invalid CAN; timeout at 10.01 seconds; one-shot diagnostics; simulation/replay.
25 Xiaoge tests and 9 localizer startup tests also pass (46 total). The broader
system-ready suite cannot collect on this Windows environment without native
msgq; it is not counted as passed. Python syntax checks pass, selfdrived/test
lint passes, and modeld retains the same three existing lint findings as HEAD.
Desktop checks do not establish device timing or resolution of SPI errors.

Compare repeated starts using model phase durations, selfdrived.initialized
dt/timeout, safety selection, Xiaoge process start, SPI failure phases/recovery
times, raw CAN gaps and the vehicle's retained ACC fault. Reduced SPI errors
with both changes would support an effect of startup scheduling but would not
isolate which change mattered or prove an electrical/firmware root cause.

## Parked C4 measurement on 2026-10-07

The requested change was installed on a parked C4 and verified after a normal
manager reboot. Its CAN-FD vehicle profile and disabled ShareData differ from
the original classic-CAN report; this does not validate the Xiaoge gate's load
reduction or resolution of that incident. The baseline was an onroad start on
an already-running device; the comparison followed a device reboot and native
relink. Jetson stayed running during the comparison.

| Observation | Before (9fc06dcd) | After (26c6fa54) |
| --- | --- | --- |
| Initialization frame budget at timeout | 6.01 s | 10.01 s |
| First valid modelV2, from first CAN batch | 13.981 s | 15.717 s |
| Hyundai safety selection, same origin | 7.451 s | 11.509 s |
| Maximum per-bus CAN batch interval, buses 0/1/2 | 22.911 ms | 24.345 ms |
| RX/TX buffer overflow during segment | 0 / 0 | 0 / 0 |
| SPI checksum counter increase during segment | 0 | 0 |

The before/after checksum totals were 43 and 1 respectively; a reboot resets
the counter, so their difference is not an improvement measurement. After
reboot, tmux contained one all-zero invalid SPI-header diagnostic before the
first logged CAN batch. Neither segment reproduced the multi-second NACK burst.

New phase logging measured 1.693 s for the internal model and 6.861 s for the
Jetlink adapter, giving the reported 8.6 s total construction time. Before
construction, the existing absent-eGPU grace consumed about five seconds.
The adapter includes synchronous Warp preparation and verification; its log
does not individually time compilation, warmup, and the 27 probe comparisons.
The internal model was constructed at CAN-relative 6.904 s, but modelV2 did
not publish until 15.717 s. This establishes that delaying the first usable
model output for adapter setup matters on this device. It does not attribute
all adapter time to CPU contention, Xiaoge, or SPI.

The ten-second change is active but still expires before model readiness in
this case. Raising a finite timeout alone cannot guarantee readiness-ordered
startup. This task has not yet changed adapter scheduling or the existing
finite fault-diagnosis escape path. Fresh Park, zero speed, inactive controls,
valid CAN and active/ready Jetlink were verified after startup. Private captures
and the reproducible comparison remain in the local analysis archive.
