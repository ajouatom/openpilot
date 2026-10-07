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
