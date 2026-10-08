# Xiaoge onroad startup

The manager defers the optional `ShareData` lane/BSD service until onroad
initialization has reached healthy driving readiness. Previously, enabling
`ShareData` started the service even offroad, allowing its model loading and
inference to overlap the main onroad startup.

`XiaogeStartupGate` requires fresh, valid, alive, frequency-healthy observations
of deviceState, selfdriveState, modelV2, carState and pandaStates. Every publisher
must advance beyond the message observed at the current onroad transition.
This avoids relying on a shared monotonic clock domain or accepting messages
left over from a prior session. ControlsReady must be set; CAN must be valid
without a timeout; Panda safety models/parameters must match CarParams.
Additional Pandas must be silent or noOutput. selfdriveState must be engageable.

These conditions must persist across manager observations for at least 0.5 s.
A manager observation gap longer than one second restarts that interval.
Controls initialization timeout alone cannot release the gate. Conditions that
prevent engagement, including calibration or vehicle readiness conditions, can
also defer Xiaoge. Actual cruise engagement is not required.

After release, readiness is latched for that onroad session so later warnings
do not repeatedly restart the service. ShareData still enables/disables it live.
Offroad stops the service and clears the latch; manager restart also clears it.
The service's inference settings, scheduling and output logic are unchanged.
Because the whole process is deferred, its local server is also unavailable
while waiting and offroad. No new persistent setting is introduced.

## Validation

27 focused desktop tests pass, including the existing camera process predicates.
Coverage includes initialization timeout, missing/unhealthy services, old session
messages, publisher clock offsets, CAN and safety readiness, observation gaps,
offroad/restart reset, post-start faults, live ShareData and real cereal structs.
Python lint passes for the manager changes. Korean/English guides, localized
catalog descriptions and the ShareData Wiki explanation describe the new timing.

This removes optional inference from startup; it does not establish the cause
of SPI NACKs or validate their resolution. Physical-device process timing, CPU
load, camera BSD availability and SPI behavior still require before/after logs.

## Deferred process launch correction (2026-10-07)

A C3X upload on `b5740219` exposed a regression in the deferred launch path.
The last managerState was at monotonic 72.7696 s; manager logged the request to
start Xiaoge at 73.3110 s and produced no more managerState through the 115 s
segment end. The manager remained runnable with nearly unchanged user CPU time
and about 26.6 s additional system CPU time. No Xiaoge child appeared in procLog.
Model output continued at 20 Hz. This implicates the synchronous process-start
path; the recording does not identify the precise kernel wait/spin site.

Xiaoge alone now uses the multiprocessing `spawn` context, launching a fresh
interpreter instead of copying the already-running manager's interpreter and
IPC state with the default fork context. Its launcher, supervision, offroad stop,
ShareData predicate and readiness gate remain unchanged. Other managed processes
retain their existing launch context. This is separate from Panda SPI recovery.

30 focused desktop tests pass. A separate supervised launch on the parked C4
returned in 31.2 ms and kept its observation loop responsive for 20 seconds
(maximum iteration gap 42.0 ms, 400 model updates, managerState alive). Both Xiaoge
servers started; the test child was stopped and reaped afterward. Shutdown also
printed the existing native termination diagnostic; this test does not validate
graceful native resource teardown or inference output. The user's ShareData
setting was not changed. C3X reproduction and an updated affected-car log remain
necessary to establish resolution on that device. Raw incident data stays local.
