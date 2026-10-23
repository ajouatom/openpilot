# Connection-owned RAM input IPC

This change is confined to comma's local `modeld` → `jetlinkd` input path.
Jetson, USB framing, N16, recurrent GPU state, model outputs, HUD timing and
driving decisions are unchanged. It does not enable asynchronous USB writes.

## Lifetime and compatibility

- `jetlinkd` advertises `ram_input_v1`. A default-enabled client requests
  it during its non-realtime handshake. Unaware peers retain the existing IPC.
- Each accepted Unix connection gets a new anonymous memfd and random session
  identifier. There is no global file, reused filename, or persisted buffer.
- Size comes from the negotiated/validated model contract, not the module's
  default model. For the installed N16 the payload is 655,596 bytes.
- The size is sealed against grow/shrink and further seal changes. The daemon
  maps it read-only, while the sole producer maps it writable. SCM_RIGHTS file
  descriptors are CLOEXEC and closed after mapping, including error cases.
- Warp's existing tinygrad `copyout` completion fence copies pixels directly
  into this RAM. Packed context is copied into the disjoint remaining region.
  The socket carries a 56-byte token plus its ordinary four-byte length prefix,
  instead of the full 655,612-byte input packet. GPU→CPU copying still occurs.
- The token binds the session, monotonic generation, frame/reset flags and
  original source timestamp. A token older than 150 ms, future timestamp,
  repeated generation, or foreign session is rejected.
- Only an exactly matching, finite, correctly sized reply releases the slot.
  Timeout, disconnection or invalid input/reply closes and retires the client.
  New connections cannot write the old allocation, even while the old USB
  consumer still holds views. Those views retain the retired mapping until
  their last reference is released.
- The synchronous producer owns all writes; arbitrary concurrent writers are
  unsupported. A second reservation while a request is pending is rejected.
- Existing local-model fallback, commIssue reporting and a fresh recurrent
  reset after joining are retained. No cleanup/ownership safety guard is bypassed.

`JETLINK_RAM_INPUT` defaults to `1`; explicit `0` uses existing socket payloads.
Old peers without the capability continue using existing socket payloads.
The initial trials below kept the feature opt-in until the DHCP investigation.
No Jetson package is needed for this comma-only optimization.

## Detached hardware comparison

Base comma 91cb2392; installed Jetson 13542cc7. Candidate modules were loaded
only from private bench directories; installed sources stayed unchanged.
Real cameras, synthetic stationary cruise and 10 Hz map/navigation inputs;
20 Hz inference, actual IsOnroad=false, Panda noOutput, harness disconnected,
CAN TX=0. Physical HUD panel was not attached, so this is not a panel-rendering
load or road validation. Existing Jetson server/HUD processes were retained.

First run: 1,500-frame blocks A/B/B/A, 3,000 frames per mode, 300 timed seconds.

| Metric | Existing socket (A) | Connection RAM (B) |
|---|---:|---:|
| Whole JoiningModel mean | 36.924717 ms | 36.318320 ms |
| Whole JoiningModel P99 | 37.828712 ms | 37.048411 ms |
| Maximum | 39.002440 ms | 37.944623 ms |
| >50 ms | 0 / 3,000 | 0 / 3,000 |
| Socket-send mean | 0.547820 ms | 0.086870 ms |
| GPU readback mean | 2.073981 ms | 2.038865 ms |
| Server GPU mean | 24.194641 ms | 24.160377 ms |

P99 improved 0.780300 ms and whole-call mean improved 0.606397 ms in this run.
Both B blocks beat both A blocks. This does not imply a guaranteed saving on
every vehicle or reduction in actual GPU computation. All 6,000 timed inputs
had map/navigation; no camera drops, USB inference failures or extra HELLOs.
128 same-GPU-submission image comparisons and 128 post-consumption hashes
were exact. All parameters, power settings and original manager were restored.

## Reconnection assessment

Linux tests exercise actual SCM_RIGHTS/memfd mappings, 1,000 generations,
100 reconnects while an old USB consumer retains its data, 100 repeated
connections without FD/mapping leakage, malformed/extra/truncated descriptors,
size/seal/nonce/age validation, frame wrap, bad context, reply timeout/EOF,
wrong frame and nonfinite replies. Both legacy negotiation directions remain
compatible. These are protocol simulations, not physical cable tests.

First hardware fault-injection run used the actual daemon's normal error
cleanup/reopen path. The first USB loss recovered. After the second, existing
DHCP cleanup exceeded its two-second deadline and latched the link blocked.
No shared input mismatch was observed: all 80 captured receive-side input
hashes matched, two sessions had distinct buffers, and two retired input
buffers stayed unchanged. The run was stopped and the original manager/link
recovered normally. This failed run is retained; it is not counted as a pass.
Later inspection found the DHCP process had exited; its delayed exit's root
cause is not established. The DHCP helper and its safety guards are unchanged.

The second run spaced losses apart: before USB send, after USB send, then a
200 ms late local reply. All three triggered the expected local-model fallback
and rejoin. Four distinct shared sessions were observed; the first request in
each carried reset. All 1,200 captured receiver payload digests equalled their
sender payloads. Both retained old input buffers stayed unchanged. Same-GPU
image equality passed 128 times; 127 had post-success hashes (one deliberately
interrupted request had no success acknowledgement).

After recovery, a 750-frame A/B/B/A sequence gave 1,500 frames per mode:

| Metric | Existing socket (A) | Connection RAM (B) |
|---|---:|---:|
| Whole JoiningModel mean | 37.033550 ms | 36.449560 ms |
| Whole JoiningModel P99 | 37.755688 ms | 37.218239 ms |
| Maximum | 38.013915 ms | 37.877645 ms |
| >50 ms | 0 / 1,500 | 0 / 1,500 |

All timed frames succeeded, with no camera drop, USB failure or extra HELLO
inside these post-recovery blocks. The three deliberately failed warm-up calls
are not erased from the evidence or counted as successful timed frames.
P99 improvement was 0.537449 ms after recovery. Both device settings and the
original manager/link were restored; installed sources were unchanged.

These were actual daemon cleanup/reopen and software USB reconnection tests,
not manual physical cable unplugging. They establish input lifetime isolation
for the tested cases, not a road-safety guarantee or exhaustive hotplug coverage.

Decision: keep the implementation opt-in and do not deploy it as a default in
this turn. The first rapid-loss run's DHCP two-second cleanup failure is still
unresolved, even though the spaced rerun passed. Do not bypass its ownership
check, force rebind, increase real-time priorities or conceal the failed run.
Investigate helper shutdown latency separately before default deployment.

Linux regression result: 188 passed, 1 skipped (optional external reference
checkout absent). The new shared-input tests number 26. New module/client/tests
pass Ruff. Two pre-existing lint findings remain in daemon/model. A separate
unchanged-helper shell test hit Windows checkout CRLF; its LF-normalized
stream passes bash -n. No helper code was modified.

Private evidence is stored in `jetson-nvme/ipc-ram-owned-20261023`,
`ipc-ram-reconnect-20261023` (failed run), and `ipc-ram-reconnect2-20261023`.

## Subsequent DHCP investigation

See [DHCP cleanup follow-up](jetlink_dhcp_cleanup_20261023.md). An isolated
actual-device DHCP address probe reproduced the two-second deadline defect;
a five-second verified-exit grace passed three identical trials. With that
correction, six software USB losses and one late local reply all recovered.
Eight RAM sessions began with reset;1,200 payload digests and six retained old
buffers passed. Post-recovery P99 improved37.713840→37.072065ms, mean
36.941302→36.350218ms,0/1,500 over50ms in each mode. Production source/settings
were restored and RAM remains opt-in. Earlier failed trial evidence is retained;
this does not retroactively establish its exact untraced internal cause.

## Default rollout authorization

After the DHCP correction and reconnect tests, the owner requested default
activation, publication, installation on comma and a normal reboot followed by
a five-minute 20 Hz check. This revision enables capability negotiation by
default; explicit `JETLINK_RAM_INPUT=0` remains available for comparison.
Unset-environment tests cover both capable and legacy peers. Deployment and
post-reboot measurements are recorded separately after actual completion.
