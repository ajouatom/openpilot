# Common temporary-steering warning delay

The user approved a common 120 ms presentation delay for temporary steering
faults, without vehicle-name, brand, CAN type or steering-control-type branches.
This replaces the vehicle-specific approach proposed in PR #532; it does not
merge or modify that remote PR.

## Behavior

`Events` retains the original `steerTempUnavailable` and
`steerTempUnavailableSilent` events immediately. Only their non-critical
SOFT_DISABLE/WARNING alert creation waits for 12 completed control cycles
(120 ms at the existing 100 Hz loop). No-entry alerts and critical takeover
alerts bypass this delay. Other events are unaffected.

The warning variants share a bounded counter, advanced once by `clear()` per
completed event cycle. Changing variant, duplicate event insertion or repeated
rendering cannot restart or accelerate the delay. To avoid hiding repeated
short interruptions, qualifying event cycles accumulate across short gaps.
After 100 consecutive completed cycles without either temporary-steering
event (one second), history resets. Healthy cycles do not create a warning.
Once the cumulative 12-cycle threshold has been reached, subsequent qualifying
warnings are immediate until that quiet reset. Existing warning severity,
sound, minimum display duration and priority remain unchanged.

This is an event-history presentation policy, not a filter on raw MDPS fault
bits. An upstream decision not to emit these events is not independently
overridden by the alert layer. No new Params setting was introduced.

## Preserved control behavior

- Raw `CarState.steerFaultTemporary` and permanent fault detection are unchanged.
- controlsd still immediately gates lateral actuation on the raw fault, including
  AlwaysLateral, and resumes only when the existing eligibility checks pass.
- Event serialization, no-entry decisions, soft-disabling entry and the existing
  three-second soft-disable deadline are unchanged.
- Short faults still recover through the existing state machine. No ECU reset,
  retry command, changed torque/CAN limit or automatic re-engagement was added.

120 ms and the one-second quiet reset are software policy choices, not
validated OEM diagnostic or vehicle-safety thresholds. This change reduces
brief warnings; it does not fix the underlying MDPS rejection or prove that
an unannounced interruption is harmless on every vehicle.

## Validation

- 70 desktop tests passed: the new warning tests, state-machine tests and
  controlsd lateral-gate tests. Coverage includes both warning variants,
  platform-independent behavior, 12-cycle onset, repeated short faults,
  99/100-cycle quiet reset, duplicate insertion/rendering, immediate no-entry
  and critical alerts, other faults, automatic recovery and unchanged timeout.
- 47 readiness/update/startup-event regression tests passed, plus the existing
  AlertManager duration test in an isolated process (118 total).
- Ruff and targeted diff whitespace checks passed.
- Recorded-input alert/state replay of the Elantra incident at 8.5–12 s uses
  350 carState samples with seven temporary-fault samples. Baseline creates
  seven alerts held for 201 display cycles; the new policy creates/displays
  none. Event payloads, state transitions and timer values match exactly.

Windows validation uses the existing desktop adapters for native IPC, Params,
hardware and logging imports, while executing production Events, AlertManager
and StateMachine code. Lateral-gate tests execute the production gate via AST.
This is not a full native Linux/device test or vehicle/audio/display validation.

Private replay script, result and reproduction commands are archived under
`.analysis/archive/2026-10-03/common-steering-warning/`. Related incident analysis:
`docs/elantra_29b_steering_warning_20261003.md`.

Docs-Not-Needed for settings guides/Wiki: no setting, default, menu or Params
behavior changed; this engineering document records the approved warning policy.
