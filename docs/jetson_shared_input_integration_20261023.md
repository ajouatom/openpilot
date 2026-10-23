# Shared RAM input and owned DHCP shutdown integration - 2026-10-23

The owner requested review, integration and device validation of contributor
[6914f6ce](https://github.com/tjdgus200/openpilot/commit/6914f6ce5a236b606defff5097546b009e50b47a)
and [fbe9a887](https://github.com/tjdgus200/openpilot/commit/fbe9a887d0346e0d0eb136689010be181a4e3788).

## Accepted scope

On carrot-nav, modeld and jetlinkd negotiate one anonymous, size-sealed memfd
per Unix connection. GPU readback fills that connection's image destination;
scalar context is copied to a disjoint region. The local socket carries a
56-byte token plus four-byte framing instead of the 655,596-byte N16 input
payload plus request/framing headers. USB framing, image/context values, N16,
GPU recurrence, HUD/display traffic and reply validation are unchanged.

The daemon maps read-only; the producer exclusively owns writes until an exact,
finite, correctly sized reply releases the slot. Nonce, monotonic generation,
frame/reset flags and 150 ms token age are checked. Timeout, malformed reply or
other uncertain completion retires the connection; a late consumer retains its
old mapping, and reconnecting cannot reuse that allocation. File descriptors
are closed after mapping and on error. Legacy peers and explicit
JETLINK_RAM_INPUT=0 retain the socket path. No asynchronous USB is introduced.

Review addition: an invalid/read-only warp destination must still drain the
already-submitted GPU operation before local fallback. The contributor's early
validation raise skipped that fence; corrected and covered by failure tests.
Add two scalars to existing once-per-second runtime timing aggregates:
ipc_request_bytes (including four-byte prefix) and ipc_shared_input. These
permit actual default-activation verification without per-frame payload logging.

DHCP cleanup waits up to five seconds for the already verified process to exit,
returning immediately for fast exit. Executable/arguments/PID/start-time checks,
pre-signal identity verification, SIGTERM-only behavior and live-process refusal
remain. Disappearance after validation is successful cleanup. No USB rebind or
ownership guard is bypassed. This changes recovery, not inference execution.

Carrot-wip receives the common DHCP correction and documentation only. Its
separate queued-model implementation does not receive the N16 IPC integration.
No new Jetson bundle/model pin is needed: normal comma Git Pull and reboot
activate these comma-side changes; the signed signal-display/N16 host remains.

## Verification

Windows suite: 108 passed, 52 Linux/POSIX skips including the four actual
daemon/socket integration cases delegated to Linux CI. Native SCM_RIGHTS/memfd tests cover exact bytes,
1,000 generations, old-reader isolation over 100 reconnects, FD/map lifetimes,
malformed descriptors, legacy negotiation, timeout/bad reply and frame wrap.
Additional tests run actual daemon/client framing through a fake USB endpoint,
full/projected replies and interleaved reserved/legacy payloads. Copyout/fence
checks cover shared destinations and context/destination failure. DHCP cases
exercise both pidfd and legacy /proc ownership/exit paths on Linux.

Before application, the vehicle was freshly confirmed P/zero/inactive. A
120.066-second read-only baseline had 2,401 valid consecutive model outputs,
mean 38.852 ms, P99 42.717 ms, max 45.374 ms, zero >50 ms; model/DM/cameras/pose
were healthy. No navigation route was present. This is our current baseline,
not the contributor's detached-bench ABBA result.

Contributor timings imply mean 36.941302 -> 36.350218 ms (0.591084 ms saved),
P99 37.713840 -> 37.072065 ms (0.641775 ms saved). Their detached bench lacked a
physical HUD panel. Do not describe 0.64 ms as their mean gain or our measured
gain. Their DHCP reproduction and 1,200 hashes are contributor evidence, not
our device validation. See imported [RAM](jetlink_shared_input_20261023.md) and
[DHCP](jetlink_dhcp_cleanup_20261023.md) notes for original scope and limitations.

Final CI, normal installation and physical validation are recorded below when
completed. Raw vehicle/device evidence stays private under .analysis.
