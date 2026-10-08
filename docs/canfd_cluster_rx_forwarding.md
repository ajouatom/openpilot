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

## Host template recovery after startup CAN interruptions (2026-10-04)

The host previously attempted to register camera-side transmit templates only
at ControlsReady counts 121/122. If the message had not been observed by that
single update, its template stayed absent for the whole session, even after
CAN reception recovered. Direct cluster TX cannot restore a message that the
host never generates. This is separate from the cause of a transport outage.

CarState now retries discovery of LFA, LFA_ALT, LFAHDA_CLUSTER, ADRV_0x161,
ADRV_0x200, ADRV_0x1ea, ADRV_0x160 and CCNC_0x162 after their original earliest
registration count. Only an observed address on the configured camera bus is
registered; absent variants add no checks. A template becomes available only
after the existing parser accepts an actual counter/checksum-validated frame,
with the expected payload length and initial age at most 150 ms. Registration's
zero-filled dictionary cannot initialize TX. Thereafter the template retains
the original live-dictionary behavior; this is not a new ongoing freshness
policy. Counter algorithms, control limits, Panda forwarding and the direct-TX
setting/default are unchanged. No Panda firmware change is required by this
host recovery correction.

Each template's first activation produces one bounded carlog entry, forwarded
by card to cloudlog, with message name, bus and ready count. Normal startup may
wait an additional received frame for validated data rather than transmitting
an initial zero template. In the healthy recorded-input replay this delayed
LFA availability by about 8 ms and the 20 Hz templates by about 45 ms; these
are host publication-time estimates, not physical CAN latency. Independently
seeded TX counters can consequently start at a different value; no bit-for-bit
initial TX equivalence is claimed.

Validation: 487 focused Hyundai tests pass, including delayed arrival beyond
the entire startup window, zero-template exclusion, bad CRC/counter/length,
wrong bus/TX echoes, absent variants and the existing cluster, MDPS, touch and
configuration tests. Desktop Params storage is substituted. Paired recorded
CAN/CarState replays reproduce permanent template omission with the old code
and restoration with the new code; 26,276 common healthy decoded-template
comparisons match. ControlsReady write completion and exact subscriber batching
are not recorded, so replay timings are reconstructed, with the first generated
MDPS used as an additional bound. Incident data and reproduction scripts remain
local only. This does not prove repair of the initial SPI failure, ECU fault
clearance, or vehicle warning resolution.

## Avoid redundant CAN restart during H7 safety handoff (2026-10-04)

Safety command 0xDC executes synchronously inside the SPI receive DMA handler.
The previous normal-ELM327 to Hyundai CAN-FD transition reapplied the CAN mux
and initialized all three controllers even though both policies used normal
routing and live CAN. Each controller's speed setup and FIFO initialization
enters/exits INIT separately. These waits share the MCU with SPI servicing;
electrically separate CAN and SPI buses do not imply independent CPU progress.
This code path is a plausible interruption trigger, not a measurement proving
that an observed multi-second SPI retry burst was spent inside CAN init.

H7 now preserves running controllers only for ELM327 with nonzero parameter to
Hyundai CAN-FD, with a present, unchanged harness, recorded normal mux routing,
power saving off, live/non-loopback configuration, and all three controllers
successfully initialized with unchanged bus mapping, bitrates and ISO mode.
Hardware checks reject INIT/sleep/monitor/test state, bus-off/error-passive/
error-warning, pending CAN error flags and any pending hardware TX request.
All other cases retain full reinitialization, including F4, SILENT transitions,
OBD mux changes, invalid safety modes and repeated Hyundai safety requests.

The eligibility decision, old software-TX cleanup, pre-transition hardware RX
FIFO cleanup, safety hook reset and relay handoff share a bounded critical
section. RX cleanup acknowledges at most the initial FIFO fill level, capped
at hardware capacity, and does not clear new-arrival interrupt flags. Host RX
history is retained. No old hardware TX is carried into the optimized path:
pending TX selects the old reset path because FIFO cancellation is not a safe
substitute. The optimized path skips mux reapplication and controller INIT;
the relay still switches normally. No new forwarding policy or early relay
activation is introduced. Slow initialization remains outside the added
critical section. Electrical relay timing and MCU worst-case duration have
not been measured on a device.

The driver also fixes its always-false initialization return value so successful
configuration can be recorded, and bounds the previously unbounded clock-stop
acknowledge wait using the existing 500-iteration nominal-ms timeout policy.
This is not a hard 500 ms response guarantee or a redesign of CAN error-ISR
recovery. SPI framing, retry rules and interrupt priorities are unchanged.

One `safety_can_transition` serial diagnostic reports mode, preservation choice
and MCU elapsed microseconds, all printed in hexadecimal. Duration is captured
before formatting. Serial log retrieval is delayed by transport and can lose
old lines; its host log timestamp is not the physical relay timestamp. A new
capture can distinguish an actual controller restart from a preserved handoff.

Validation: 17 tests pass (three native C tests covering F4/H7 transition
matrices, queue/configuration/failure conditions and bounded sleep exit, plus
14 firmware identity tests). Tests compile extracted production functions with
mock registers; they do not emulate peripheral timing or physical FIFO ACKs.
ARM GCC 13.3.1 builds Panda and Jungle F4/H7 main firmware and bootstubs, eight
targets, with `-Werror`; Panda development signing succeeds. Existing unrelated
working-tree experiments are excluded from build inputs. Physical SPI response,
relay behavior and warning resolution still require repeated device startups.
This change modifies Panda firmware and requires its normal startup rebuild/
installation, unlike the preceding host-only template-recovery correction.

## Extend the same H7 handoff to classic Hyundai CAN (2026-10-07)

The user requested the same conditional controller preservation for classic
CAN. The destination allowlist now includes SAFETY_HYUNDAI (8) and
SAFETY_HYUNDAI_LEGACY (23), alongside SAFETY_HYUNDAI_CANFD (28). This is about
vehicle safety modes on H7's FDCAN hardware: classic CAN frames also use that
controller. F4's bxCAN driver and all other manufacturers remain unchanged.

No eligibility check was relaxed. Only live normal-ELM327 handoff with the
same known-good timing/mapping, harness/mux, no hardware errors and no pending
TX may skip controller INIT and mux reapplication. Safety hooks, queue cleanup,
relay ownership and transition diagnostics still execute. Every ineligible
case retains full initialization. In particular, this does not remove initial
boot configuration, bitrate changes, fault recovery or safety-state resets.

The production-function C harness now covers each of modes 8/23/28 on H7 and
the unchanged F4 branch, the 64-pair mode transition matrix and 20 rejection
conditions for each target. SPI retry duration is not a measurement of CAN
initialization duration. Root cause of the reported SPI burst remains
unconfirmed, and this extension is not a demonstrated cure. Updated Panda
firmware and classic-CAN device startup evidence are required to establish that
`preserve=1` is actually selected and whether NACK/CAN gaps improve.

Validation: seven compiled C harness cases and 14 firmware source-identity
tests pass. ARM GCC 13.3.1 builds all eight Panda/Jungle F4/H7 main/bootstub
targets with `-Werror`, and Panda development signing passes. Build inputs
come from a clean HEAD archive with only this task's main.c change overlaid;
unrelated working-tree safety experiments are excluded.

## Independent CCNC host counter (2026-10-06)

The host's `CCNC_0x162` copy now removes the snapshot's explicit `COUNTER` and
uses the existing CANPacker per-address counter, as the other cluster messages
already do. The first generated message uses the initial RX counter plus one,
modulo 256. Later generated messages increment that stored value by one;
repeated, skipped or wrapped RX snapshots cannot reseed it. Unscheduled calls
and absent templates do not advance the sequence, and a new packer starts a new
sequence. Display fields, transmission schedule and automatic CRC calculation
are unchanged.

Direct-TX mode passes this independent host sequence through. Default RX-paced
mode still replaces it with each original vehicle RX counter and recomputes
CRC inside Panda; its final output is unchanged. This is a host-only correction,
with no Panda firmware or setting/default change.

Validation: 303 focused Hyundai tests pass, including Python and native packer
wrap/seed/repeated-RX tests, plus 183 compiled Panda cluster-hook tests. A
1,201-message recorded-payload replay produces only +1 counter steps with valid
CRCs and identical display bytes on both packer backends. Production C hooks
preserve the new direct-TX sequence and produce byte-identical old/new RX-paced
outputs for all replay inputs. Desktop replay does not establish physical ECU
acceptance or resolution of the reported cluster warning. Incident data remains
local only.

## Forwarding timer rollover (2026-10-07)

The legacy per-ID forwarding suppression table previously treated an initial
`last_tx_us=0` as a recent host transmission whenever the 32-bit microsecond
timer wrapped (about 71 minutes 35 seconds). Unreplaced stock messages could
therefore be blocked for their configured period plus 20 ms, even though no
host replacement was sent. This affects the fallback forwarding table, not
only the optional direct-cluster path.

Each entry now records whether a replacement was accepted and the existing
1 Hz safety-mode tick at that time. Forwarding expires that state at the
original microsecond deadline. A coarse age greater than two ticks also
expires it, preventing an old timestamp from becoming fresh after an entire
timer wrap with no intervening traffic on the ID. Two tick boundaries are
allowed because the longest existing deadline is 1.02 seconds. A legitimate
TX at microsecond zero remains valid, and mode initialization clears all
entries. Allowlist or relay rejection cannot arm suppression.

The original suppression durations, control FIFOs, RX-paced cluster delivery,
direct-send setting/default, counters, CRCs and relay protection remain intact.
No generic safety tick callback, CAN reset or SPI retry change is introduced.
This is a Panda firmware change and requires the rebuilt firmware on the
device; a host-only update cannot change forwarding already running in Panda.

Validation: 57 focused startup/full-wrap cases fail against the old compiled
C hooks. The corrected native forwarding, cluster and button tests plus
firmware-identity checks pass (408 tests). ARM GCC 13.3.1 builds F4/H7 main and
bootstub targets with `-Werror`; development signing passes. A local recorded
input replay of 7,774 frames, using an inferred timer phase, reproduces eleven
old-hook blocks and none with the corrected hooks, with identical payloads.
Builds and C tests exclude unrelated local safety experiments. These results
establish the forwarding correction, not physical ECU acceptance or resolution
of every intermittent cluster warning. Incident data remains local only.
