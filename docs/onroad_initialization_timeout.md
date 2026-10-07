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
