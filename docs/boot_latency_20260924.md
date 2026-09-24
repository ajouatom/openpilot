# Ioniq 5 C4 startup after the September 24 update

## Evidence

Device `07b62e389ed26c81`, NAS captures `tmux_send-20260924-081432-carrot-wip.txt`
and `onroad-20260924-081525-carrot-wip.txt`, source `209b3c0cfe` (previous
capture/firmware `958098fb65`). The first camera sync reports BOOT_TS
178.044 seconds. Early build messages have no wall timestamps, so the capture
does not establish individual build durations or an exact total startup time.

- Restoring `StoppingAccel` changed `params_keys.h`, triggering the Params build
  and relinking dependents. This part was a real update dependency.
- The precompiled model smoke test loaded both 1928x1208 and 1344x760 workers.
  Loading took 23.309 and 21.205 seconds; their ten inferences plus loading total
  48.784 seconds, excluding unmeasured process cleanup/launch overhead.
- After those workers exited, actual modeld startup loaded the driving models
  again, taking 23.2 seconds. Recent captures show 23–27 seconds for this phase.
- Panda flashed from 08:14:14.764 to 08:14:20.089. Panda and safety sources did
  not change between these commits; embedding the whole repository HEAD changed
  the firmware anyway. The build also rebuilt Jungle firmware.

`build.py` invokes optional model preparation after every ordinary build, and
the launcher runs it on source checkouts without `prebuilt`, including when
SCons itself finds no stale targets. It previously ran the smoke test regardless
of a previous successful validation. This explains recurring overhead, not just
the settings update's rebuild.

## Changes

The precompiled artifact and runtime archive retain their full SHA-256 checks
in `ensure_precompiled`; the runtime worker also retains its PKL checksum,
metadata, architecture, output and failure checks. After a successful smoke
test, an atomic device-local receipt permits skipping subsequent smoke tests
with the same catalog, machine ID, device family, OS/kernel, Python/NumPy,
camera size and relevant runtime/validation source contents (including extracted
runtime code). Unknown or missing
device identity is not cacheable. Corrupt receipts are misses. Any recorded
worker failure, including a transient failure, removes the receipt without
changing existing rejection/fallback rules. Failure to save the receipt leaves
the model usable and causes revalidation next time.

Vehicle smoke tests select C4/mici 1344x760 or C3/tici/tizi 1928x1208. The
standalone runner still defaults to both resolutions for artifact acceptance.
No receipt is migrated from older logs: the first updated boot validates once.

Panda's version identifies firmware source contents instead of repository HEAD.
The digest covers board sources/linker scripts/SCons files, shared safety
sources, crypto/signing code, public keys and the version generator. Generated
objects, tests, host Python tools and unrelated openpilot changes are excluded.
SCons continues tracking real compiler inputs/options, and pandad continues
comparing full signed firmware signatures before flashing. The version suffix
is now a source digest, not a Git commit. Switching version schemes requires
one rebuild/flash; subsequent unrelated updates should not repeat it.

## Validation and limits

Focused desktop tests exercise receipt persistence/invalidation, C3/C4 selection,
successful build reuse with integrity checks still called, transient/definitive
failures, and Panda source identity. A miniature SCons build verifies that an
unrelated source change reuses the target and a safety source change rebuilds it.
These tests are included in the build-release CI job.
The actual Panda/Jungle SCons build graph was also dry-run from an isolated copy
using a POSIX construction environment on Windows; compilation was not executed.

The observed 48.784 seconds is a removable repeated workload from the old
capture, not a measured post-change vehicle boot improvement. C3/C4 cold boot,
GPU readiness and actual firmware build/flash on a vehicle remain unvalidated.
The normal 23-second-class driving-model load and necessary source rebuilds
remain; no model, radar, scheduling, camera validity or safety policy changed.
