# CAN-FD cluster forwarding on stock reception (2026-09-30)

The user requested stock-reception timing for five cluster messages after
intermittent Ioniq 5 PE "Check Driver Assistance system" warnings. This changes
the delivery mechanism; the existing captures do not establish that host
cadence caused the warnings, or that this change fixes them.

## Optional direct transmission (2026-10-03)

**Cluster CAN Direct Send** (`HyundaiCanfdClusterDirectTx`) is a persistent,
default-OFF user setting under Vehicle and Hardware / CANFD·HDA. No vehicle,
including EV6, enables it automatically. Reboot after changing the setting;
CarParams and Panda safety configuration select the mode at startup, without
live switching.

For CAN-FD CAMERA_SCC only, the enabled setting adds
`HyundaiFlags.CANFD_CLUSTER_DIRECT_TX` (bit 27) and
`HyundaiSafetyFlags.CANFD_CLUSTER_DIRECT_TX` (2048). Panda then bypasses both
the host cache and RX replacement for the five cluster IDs below. Allowed
host frames follow the pre-existing direct TX path, preserving their supplied
counter and CRC. Stock forwarding uses the existing per-ID 70 ms suppression
after host TX. Host generation rates, steering/SCC/MDPS/control FIFOs, reuse,
allowlists and relay protection remain unchanged. Safety initialization still
clears caches and timers when entering either mode. The setting has no effect
outside CAMERA_SCC CAN-FD; OFF retains the RX-paced behavior described below.

This is a manual compatibility comparison, not a diagnosis or an automatic
timeout fallback. RX-paced output requires the corresponding stock RX trigger;
direct TX removes that dependency but still requires host packets to reach
Panda. Neither path can promise delivery during an SPI or physical CAN failure.
Direct TX also restores the earlier host/stock counter handover behavior;
it does not introduce independent counter sequencing or repair +2 increments.
Updated Panda firmware is required. No device flashing or physical warning
resolution is established by desktop tests.

Validation for the option: 459 focused native/Python tests, 45 settings-schema
tests, 25 Wiki tests and 14 firmware identity tests pass. Native tests cover
missing RX, unchanged host counters/CRC (including +2/wrap), the exact legacy
70 ms boundary, cache reset on mode changes, allowlists/relay rejection,
non-camera behavior, and unchanged control FIFO drain/reuse/exhaustion.
Interface tests cover every CAN-FD platform with an existing torque-parameter
entry in HDA1/HDA2 configurations; the pre-existing incomplete K5 HEV entry
cannot construct CarParams. No platform has a static direct-TX flag.
ARM GCC 13.3.1 builds F4/H7 main firmware and bootstubs with `-Werror`, and
development signing passes. Windows Params is substituted only in desktop
tests. Reproduction scripts and results are retained locally under
`.analysis/archive/2026-10-03/ev6-direct-cluster/`.

## Behavior

With direct transmission OFF, in Hyundai CAN-FD CAMERA_SCC mode, host bus-0 copies of 0x161 (32 bytes),
0x162 (32), 0x1e0 (16), 0x1ea (32), and 0x200 (8) are consumed into independent
latest-value caches. Host TX does not put these copies directly onto the bus.
Every corresponding bus-2 RX frame produces one forwarding decision to bus 0:

- With a fresh valid host copy, use its body, the original RX counter, and a
  newly computed Hyundai CRC. Preserve the original frame's CAN metadata.
- Before the first host copy or at a host age of **150 ms or more**, forward
  the original frame unchanged. Expiry invalidates the cache until another
  accepted host update. The previous 70 ms direct-TX block is bypassed here.
- With no original RX, generate no independent transmission. Missing source
  data remains missing; this cannot prevent the cluster's own RX timeout.
- Malformed original length/extended format/CRC invalidates the cache and
  forwards that original unchanged. Do not turn corrupt input into a valid
  substituted packet. Invalid host CRC/format is rejected; only permitted
  host addresses, buses and lengths can populate the cache. Relay faults
  retain the existing TX/forwarding protection. Safety reinitialization clears
  all caches. Other safety modes and buses retain their previous paths.

The 150 ms bound is three nominal host display cycles, **not an OEM diagnostic
threshold** or a measured vehicle acceptance limit. It bounds the most recent
host arrival, not the original snapshot age inside that host message. If the
host continues republishing an old snapshot, this transport layer cannot
independently identify the stale application data.

## Different vehicle rates and timing tradeoffs

The wire-send trigger is stock RX, with no assumed 20 Hz vehicle period and
no period estimator. Host generation remains 20 Hz. A faster stock stream
reuses the latest body while it remains fresh; a slower stream uses the newest
copy rather than building up a FIFO of old display data. Unlike the existing
steering/SCC FIFO, these display caches have no two-packet start reserve and no
fixed two-reuse limit. Thus a rate mismatch does not continually exhaust a
packet-count budget or accumulate several hundred milliseconds of backlog.

Latest-value sampling can coalesce intermediate host updates, including brief
display states, when more than one update arrives between stock frames.
It does not guarantee delivery of every host-created transient. A changed
display or warning body may wait until the next stock RX; existing host
payload construction/suppression is unchanged, but its exact display timing
is not. During a host outage a previous body may remain until the 150 ms
limit, after which stock content returns on RX. Original counter repeats,
skips and resets are preserved rather than concealed by a synthesized counter.

All five DBC layouts use an **8-bit byte-2 counter**, including the 8-byte
0x200. The generic helper interprets 8-byte packets as button layouts with a
4-bit counter inside byte 1. Adding 0x200 directly to the old FIFO table would
therefore use the wrong layout. The new path uses the checked message-specific
layout and recalculates CRC after substituting byte 2.

Existing buffered LFA/LFA_ALT/SCC/MDPS/TCS/button paths, queue/reuse limits,
host control generation, CAN allowlists and safety limits are unchanged.
No warning suppression, host-rate change or new user setting is introduced.

## Validation

- **348 tests pass**: native compilation of the full production safety hooks,
  plus existing ALT2, MDPS/TCS feedback, CCNC display and fault-filter tests.
  New cases exercise all five lengths, independent CRC, counter wrap/repeat/
  skip, host/stock rate mismatches from 1 to 200 Hz, non-integral ratios,
  changing rates/jitter, bursts, host and stock loss, startup, 32-bit timer
  wrap, reset, wrong buses, malformed input, relay faults, and the existing
  HDA1/non-camera allowlists. Rates are synthetic stress cases, not claims
  about supported vehicles' measured frequencies.
- Full-cereal replay of six Ioniq 5 PE segments produces **36,120 forwarding
  outputs for 36,120 original cluster frames**, with no direct host output.
  Every substituted body matches the latest accepted host copy, every output
  counter matches original RX, and independently checked CRCs pass. Replay
  starts with empty caches; startup/segment-boundary stock fallbacks therefore
  include history absent from the replay, not demonstrated on-device outages.
- **285,594 comparisons** of host TX decisions and forwarding decisions/bytes
  for existing angle/LFA/SCC/MDPS/TCS paths match a separately compiled
  pre-change production baseline. These are fixed-input replays, not ECU or
  closed-loop driving simulations.
- ARM GCC 13.3.1 compiles and links F4/H7 main firmware and both bootstubs
  with the existing `-Werror` policy. Development signing succeeds. F4 main is
  58,024 bytes, H7 main 86,440 bytes. Native tests use Zig 0.16.0; unavailable
  Windows Params is stubbed only for Python test imports. Ruff and diff checks
  pass. Windows-only build adapters are private scratch files.
- All 14 existing firmware source-identity/rebuild tests pass, including
  detection of safety-source changes independently of the Git commit.

| Replay | Original frames / outputs | Median host-to-next-RX wait | Maximum wait |
| --- | ---: | ---: | ---: |
| Reporting car 5aa/46 | 6,000 | 31.34 ms | 48.82 ms |
| Reporting car 5c5/0 | 6,130 | 35.49 ms | 52.18 ms |
| Reporting car 5c5/1 | 6,000 | 39.28 ms | 50.25 ms |
| Reporting car 5ca/3 | 5,995 | 10.99 ms | 50.18 ms |
| Reporting car 5ca/32 | 6,000 | 42.38 ms | 49.50 ms |
| User car ff1/2 | 5,995 | 7.17 ms | 47.50 ms |

Wait statistics exclude superseded host copies and trailing copies without
a next RX in the segment. Across the replays, 2,227 intermediate host copies
are superseded. Maximum reused host-copy age at output is 59.81 ms. Stock RX
itself retains its recorded gaps (maximum 84.22 ms in 5c5/0); the change does
not repair source gaps. Replay uses host log timestamps for RX and host update
insertion, which include transport/batching delays. These numbers estimate
phase effects; they are not physical wire latency or a measured improvement
over prior firmware. Added CRC/copy work is bounded, but its worst-case MCU
execution time and actual bus arbitration/ACK latency remain unmeasured.

Normal device startup must build/install the updated Panda firmware for this
behavior to take effect; changing only host Python cannot enable it. Firmware
source hashing includes the new safety header. No device was flashed or driven
as part of these tests. Physical display/chime timing and warning recurrence
remain to be verified, including behavior during host fallback.

Private scripts, summaries and build results:
`.analysis/archive/2026-09-30/cluster-rx-forwarding/`.
Incident evidence: [Ioniq investigation](ioniq5_pe_cluster_warning_20260930.md).
