# Offline build clock floor — 2026-10-05

The user requested clock correction after Git pull/reboot, before building,
when the device clock is earlier than the checked-out commit. This provides
an offline lower bound; it is not a measurement of the current time.

On AGNOS, `launch_chffrplus.sh` runs the standard-library-only
`openpilot/common/build_time.py` after successful AGNOS initialization and before
runtime dependency preparation, the first Params SCons invocation and the main
build. It runs under the existing repository lock. Desktop startup is unchanged.

The helper reads local HEAD's committer timestamp (`git show -s --format=%ct`),
compares UTC epoch seconds and, only when the clock is earlier, sets it to that
timestamp plus one second using `sudo -n date -u -s @...`. An equal or later clock
is left unchanged. A second check avoids setting if time synchronization has
already caught up. Git and date commands each have a ten-second timeout. No
network, Params, cereal or compiled extension is required by the helper.

Startup output records the device and commit times, the correction target and
the observed result. Invalid/missing Git metadata, permission/command failure,
timeout or a clock still earlier than HEAD fails the startup step and enters the
existing recovery screen before building. The helper does not disable NTP/GPS,
change file timestamps, clear caches or change Cython/SCons rebuild decisions.

This assumes the commit timestamp is trustworthy. It does not repair an already
future-dated device or artifacts, nor source timestamps written before clock
correction. External NTP can still change the clock concurrently; the checks do
not make system clock changes atomic. This is not proof that reported intermittent
native build failures were caused by time, or that this floor resolves them.

Validation: 26 focused clock/launcher tests passed on Windows, with root
conftest disabled because it imports the unavailable native Params module.
Clock-setting subprocesses were mocked; Git Bash checked launcher syntax.
Physical AGNOS clock permissions, offline reboot and native vehicle builds remain
unvalidated. Existing unrelated workspace changes were preserved.
