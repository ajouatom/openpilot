# Reboot notification sound

The user expanded the request from Carrot Web's automatic Git update/reboot to
ordinary reboot commands. Tici.reboot (shared by C3/C3X/C4), manager DoReboot,
Carrot Web tool reboot/calibration-reset/rebuild actions and launcher startup
recovery now use `openpilot.common.reboot`.

The shared helper plays the existing English prompt.wav (a nonverbal chime,
approximately 1.506 seconds), preceded by 150 ms of silence for audio wake-up,
then invokes the OS reboot. It plays once in a child process using sounddevice,
so it works offroad without soundd and after manager has cleaned up its workers.
The parent imports only the standard library, retaining startup recovery's
independence from native Params, cereal and graphics builds. Sound dependencies
and the WAV are loaded only in the child.

Volume is fixed at 0.5 times saved SoundVolumeAdjust, clamped to 0..1, since no
live ambient sample is available after cleanup. Saved mute is honored. Missing
or malformed volume defaults to 0.5. No new sound asset or setting is introduced.
Audio failures are logged and do not prevent reboot. The audio child is killed
and reaped by subprocess.run after a four-second timeout; reboot command errors
are still surfaced. Non-device hosts reject reboot before any audio or command.

Web handlers spawn the helper without blocking their event loop. Existing
calibration-reset delays and successful-clean-before-rebuild-reboot ordering are
preserved. Automatic-update eligibility, duplicate-commit protection and manager
shutdown ordering are unchanged. Startup recovery uses the same helper without
changing its update/retry/locking policy.

This is application-level handling, not an OS reboot hook. Raw terminal `sudo
reboot`, factory erase commands, and the standalone recovery web server's raw
reboot fallback remain outside this path. No shutdown sound is added.

Validation: 63 focused desktop tests passed: shared sound/PCM/volume handling,
audio errors/timeouts before reboot, desktop guard, hardware delegation, both web
tool APIs, and existing automatic-update/startup-recovery regressions. New code
has no added Ruff findings relative to HEAD; older web modules retain unrelated
lint findings. Tests mock audio output and reboot subprocesses, so actual C3/C4
speaker loudness, audio-device wake-up/contention and physical reboot timing are
not yet validated.
