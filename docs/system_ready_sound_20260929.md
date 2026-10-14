# Onroad readiness sound

The requested readiness point is when engagement is possible: onroadEvents has
no NO_ENTRY event. After selfdrived initialization, require onroad deviceState,
non-passive control, valid CAN without timeout, and the existing SubMaster health
checks as well. All conditions must stay true for 0.5 seconds. The initialization
timeout alone cannot trigger this notification, nor can temporary suppression of
startup communication errors bypass the actual health checks.

Wait until AlertManager has no current alert before issuing systemReady. It is a
lowest-priority, sound-only permanent event and never enables control or changes
engagement checks. Later warning sounds interrupt it normally. A local latch
prevents another notification after a fault/recovery or disengagement in the same
selfdrived lifetime. The onroad process restarts for the next driving session;
an unexpected selfdrived restart also resets this latch. Replay/simulation do not
produce the notification.

Reuse the existing localized prompt.wav (approximately 1.506 seconds), played
once through a dedicated systemReady sound identity. No new WAV or setting is
added. Existing language fallback and ambient/user volume apply; DM-only gain
floors do not apply. No changes to user settings or their documented behavior.

Validation: 60 focused desktop tests passed for readiness, health loss and
recovery, initialization timeout conditions, onroad lifecycle, alert deferral,
real event serialization/AlertManager delivery, warning priority, PCM completion,
existing DM volume protection and navigation audio. Ruff and diff checks passed.
The Windows runner adapts native IPC/Params/hardware imports; the native IPC
timeout test is excluded. Actual vehicle timing and speaker loudness remain
unvalidated. Local runner: `.analysis/archive/2026-09-29/system-ready/`.

## NNFF notification follow-up (2026-10-03)

The NNFF notice previously fired only at selfdrived frame 550. If initialization
was still pending at that frame, update_events returned before creating the
notice; the six-second initialization timeout could therefore make it disappear
for the entire onroad session even with NNFFModelName present.

The notice now retries every 0.5 seconds after the original 5.5-second minimum.
Require initialized, non-passive, onroad operation, healthy CAN and SubMaster
checks, and no NO_ENTRY event. Select the current frame's ordinary alerts and
systemReady first, then wait for an empty AlertManager before issuing torqueNNLoad
once per selfdrived lifetime. A late NNFFModelName write is retried too. Preserve
the existing text, sound, duration and priority; later warnings still preempt it.
This does not change NNFF selection, models, steering control or saved settings.
The NNFF/NNFFLite Params registrations remain default-off. A subsequent user
request exposes NNFF only through Carrot Web search; NNFFLite remains internal.
The new catalog entry preserves saved values and next-onroad application.
Its localized catalog description feeds the generated Wiki pages; the Korean
and English settings guides describe the same scope and application timing.
Catalog validation (45 tests), Wiki tests (25), generated Markdown validation
and the user-docs check pass. Executing the existing catalog normalization and
derived-view JavaScript against backend-produced NNFF and sibling entries in
V8 confirms normal-menu/favorite/profile hiding and case-insensitive NNFF search
in Korean, English and Chinese. This is a data/view check, not a browser render.

Validation: 96 desktop tests passed for NNFF notification, readiness, update
notices, model startup and impact handling, plus the AlertManager duration test
in a separate process (97 total). That existing randomized duration test mutates
shared event definitions, so it must be isolated from fixed-duration assertions.
Ruff and diff checks passed. Windows adapters replace native IPC/Params/hardware
imports; these checks do not validate vehicle display or physical audio timing.
Runner and reproduction notes: `.analysis/archive/2026-10-03/nnff-alert/`.
