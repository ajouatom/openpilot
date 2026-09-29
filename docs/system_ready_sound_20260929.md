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
