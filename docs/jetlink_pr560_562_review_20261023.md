# PR560-562 review and selective integration - 2026-10-23

## Scope and provenance

All three PRs were open without merge commits at review time. Changes below are
selective integrations, not unchanged merges of the PR branches.

- [PR560](https://github.com/ajouatom/openpilot/pull/560): `3367b39fc9d96523eb368336dc69f0bfb7262255`.
- [PR561](https://github.com/ajouatom/openpilot/pull/561): `e09c8d64d20e6dd99aa4d3440569950371b852b4`.
- [PR562](https://github.com/ajouatom/openpilot/pull/562): `52e05c0126628d570c4f5ffa2d8b64b641ab8799`.

## PR560: mostly already incorporated

Direct-map staging and the missing os import were already incorporated in
`e4779e059a`. The direct-map implementation matches the PR; retain the newer
N16 default-on and native image preparation rather than reverting to default-off.

One parser boundary correction was missing: another valid model can have a full
output length of 2066, also the N16 projected reply length. Full outputs now take
precedence when the selected model specification matches that length. The new
actual-parser regression failed on the previous code and passes after correction.
This correction is NAV-only; Wip retains its distinct model contract.

## PR561: connection recovery

Integrate the race where DHCP exits before reading its /proc identity, retaining
the existing five-second stop wait. Cleanup requires bounded no-follow regular
state snapshots, exact owner metadata, unchanged file identity/content and repeated
process absence. No process-name kill or relaxed ownership check is introduced.

Import the default-off Konik g1 ADB handoff, additionally requiring the opt-in
marker to belong to the privileged caller and disallowing group/world writes.
It validates offroad state, controller, gadget identity/functions, other gadget
bindings and a nonblocking ownership-checked lock before releasing the binding.
No marker is created by this integration or ordinary updates.

An explicitly enabled handoff can also disconnect PC ADB/NCM: USB device role
does not identify the peer. Removing the marker affects future attempts and does
not automatically restore ADB. Sysfs checks are not an atomic hardware transaction.
Physical Konik operation has not been tested here. Advanced opt-in requires an
independent recovery path; it is not part of normal user installation.

## PR562: preserve existing power conditions

Do not import unconditional attached-boot USB input cutoff, replacement of a
nonzero forced vote, or globally blocking Jetlink when protection is unavailable.
A shared installation cannot assume an independent power supply; these policies
could interrupt a device powered solely through USB.

Adapt only callback reassertion: a zero-valued active vote is not proof the PMIC
callback succeeded. Retry the zero vote under the original empty-port, debounce,
attachment and foreign-vote guards. Tests cover a failed callback followed by
retry and attachment at each recheck. The daemon's once-per-session policy and
launcher remain unchanged. This is not electrical or UVLO validation.

## Validation and delivery

Windows focused regression: 126 passed, 31 skipped for Linux/POSIX requirements.
The parser regression was also observed failing before the correction.

Linux CI passed for both published single-parent code commits:

- NAV `5d2437da6c`: [run 39960230006](https://github.com/ajouatom/openpilot/actions/runs/39960230006),
  94 updater/ownership checks, 1244 broad checks (56 skipped), 119 navigation checks,
  11 real storage checks and 11 isolated NAT checks.
- Wip `190f96241c`: [run 39960260867](https://github.com/ajouatom/openpilot/actions/runs/39960260867),
  861 broad checks (2 skipped), 119 navigation checks and 11 real storage checks.
- Both Windows installer CI jobs passed. Counts across suites can overlap.

New handoff and DHCP race tests execute on Linux using temporary filesystem/proc
fixtures and real file locking; these do not establish physical configfs/device
behavior. No live vehicle installation or power test was performed.

Common helpers are integrated on both maintained branches; the parser change is
NAV-only. Normal delivery is comma Git Pull and restart/reboot. No Jetson runtime
bundle, model replacement, USB patch or marker enrollment is required.
