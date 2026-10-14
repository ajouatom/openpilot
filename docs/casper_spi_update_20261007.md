# Casper EV SPI regression investigation — 2026-10-07

The investigation sections below describe the original analysis-only stage.
Subsequently, at the user's request, commit `1056db04bc` defers Xiaoge until
healthy onroad readiness; it was pushed to carrot-wip. No device installation
or physical validation was performed. This is not a confirmed SPI root-cause fix.
Private incident evidence is retained locally and has not been published.

## NACK interpretation follow-up

Do not equate the dominant header NACKs with missing vehicle CAN data.
`spi_rx_done()` accepts a valid endpoint 0x81 request with zero MOSI payload,
calls `comms_can_read()` and sends DACK even when the returned CAN byte count
is zero. `spi_can_tx_ready` is consulted only for nonempty endpoint-3 writes.
Empty CAN RX cannot explain this route's first RX data NACK or the repeated
header NACKs. A data NACK observed by the host identifies the phase in which
the response was read, not necessarily the exact MCU state that produced it.

The retained `onroad-20261006-200805` MCU text has an especially useful ordered
sequence: mode-8 transition completed in hex 0x3455 = 13,397 us; then
`incorrect data checksum endpoint 00000081 len 0000`; then malformed header
`11 14 14 14 14 14 14`. This is another failing boot, not the supplied full-rlog
boot. Here len=0 is the host request's MOSI body length, not the CAN RX length.
Its only data-phase byte should be checksum 0xAB. The firmware rejects that
request before reading the CAN queue; unfortunately the debug hexdump prints
zero bytes for len=0, so it does not retain the actual bad checksum byte.
MCU print order places the rejection after transition completion in this boot;
delayed host log retrieval does not timestamp the physical event. It does not
show CAN INIT taking 2.5 seconds or prove that reinitialization cannot trigger
a subsequent timing problem.

Across seven old SPI captures and ten new SPI captures, maximum single recovery
elapsed time rises from 1,326 us to 32,562 us; maximum attempts rises from 3 to
34. Within the supplied route, the first long RX sequence's header-ACK phase is
7,053 us and recovery 31,816 us, versus 265 us and 1,148 us in the later short
RX sequence. These compare different transactions with the same RX request
shape; elapsed time includes host scheduling/driver and peripheral response,
not pure Panda execution. Captured diagnostics can overlap; their counts are
not additional independent incident counts.

Source recheck confirms byte-identical host spi.cc, firmware spi.h, H7 llspi.h
and can_comms.h between e985b31141 and eb5bbf2720. The entire Panda firmware is
not unchanged: the shared H7 llfdcan initialization adds a bounded delayed CSA
sleep-exit poll, plus the previously documented configuration tracking and
audio changes. The CSA poll adds no wait when CSA is already clear. Logs do
not retain CSA/INIT phase durations, so the extra sleep-exit path cannot be
proven active here. The normal INIT/exit-INIT loops predate this change.

Safety selection also requires careful timing interpretation: the host
`setting safety model` line occurs before an alternative-experience command
and the actual safety command; it is not a wire-level start marker. SPI holds
the shared lock while the control command, DACK and response complete. The
safety handler returns only after CAN initialization, so a slow initializer
does not by itself establish overlapping host SPI commands.

The measured update boundary changes startup ordering: old model loads finish
in 1.4–1.5 s before safety selection, whereas new loads take 8.6–8.7 s and
selfdrived's 6.01 s timeout permits the transition during loading. Old short
SPI errors already show poll/recovery bytes being misinterpreted. This supports
investigating a pre-existing transport/timing weakness under changed startup
conditions, but new clean captures with the same slow load prevent a causal
claim from timing overlap alone. Xiaoge deferral removes one contributor; it
does not remove the separate main-model initialization workload.

To distinguish the remaining mechanisms, device evidence needs the bad byte
even for a zero-length RX request, MCU SPI state/DMA status around that first
failure, and actual safety-control/host ioctl timing rather than only the
pre-command log line. No new CAN-init bypass or retry policy was applied in
this follow-up. The confirmed classic-template registration defect after SPI
recovery remains separate from the initial transport trigger.

Reproduction: `.analysis/archive/2026-10-07/spi-nack-cause/`.

**Full-rlog follow-up:** the subsequently supplied `00001e9e--3c5e7ca46c--0`
confirms the startup SPI outage and a separate persistent host-template
discovery defect. After reception recovers, the host still omits SCC11/SCC14
while sending SCC12; Panda's SCC12-based group forwarding gate consequently
blocks the stock SCC11/SCC14 copies. The vehicle reports ACC communication
fault status through segment end. See the follow-up below. The original
capture-only limitations are retained as the historical analysis stage.

## Finding

The supplied captures support a substantial worsening after the vehicle's
October 6 update. The severe observed symptom is a roughly 2.35–2.59 second
host/Panda SPI retry sequence at startup, with delayed CAN reception and
discarded stale sendcan messages. Earlier captures also contain SPI failures,
but their recorded retry windows are much shorter. The available evidence
does not identify the initial trigger or establish a physical wiring defect.

The new failures are not explained solely by more verbose diagnostics: the
host SPI implementation and capture diagnostics are unchanged between the
reported pre-update source vintage and the October 6 commits. CAN timeouts
and stale-message discards independently show operational consequences.

## Evidence and chronology

Source: `\\DS1821P\openpilot\carrot-wip\HYUNDAI_CASPER_EV d8e82c89faa755fa`.
Inspected all 58 onroad captures, 17 SPI-error captures, and 58 settings snapshots
available when copied. Capture filenames indicate capture/upload chronology,
not exact timestamps for every line. Terminal output is wrapped and logs can
overlap. Counts below are per capture; overlapping onroad/SPI captures must
not be added as independent incidents.

No vehicle directory ending in this exact dongle ID was found under the NAS
route-log root. Consequently there is no full rlog, raw bus timing, Panda CAN
health series, process scheduling trace, or authoritative on-screen alert
timeline in this analysis. The absence of a matching folder does not prove
that no log exists under another identifier.

| Capture group | Recorded observation |
| --- | --- |
| Seven SPI captures, Sep 23–Oct 1 | Maximum transfer window per capture 7.238–30.983 ms; maximum attempts 2–3; no `SPI: CAN NACK` or stale-send discard lines in those captures. |
| Oct 6 08:53 onroad | Still reports source date Sep 25 13:44:39, firmware `DEV-1de1c7b1f187-DEBU`, signature `79e68e56653259b4` matching expected. Logged CAN timeout count is zero. |
| Oct 6 09:11 SPI capture | Source HEAD time matches `383e86d17e`; includes rebuild and firmware replacement, old signature differing from new expected signature. New startup diagnostics appear after flashing. Transfer window reaches 2,589.509 ms; 66 stale sendcan messages discarded. This pre-flash mismatch is not proof that incompatible firmware remained in use. |
| Oct 6 09:13 through Oct 7 08:57 SPI captures | New firmware `DEV-ee88c7b40122-DEBU`, signature `17ffde04b01a7f37` matching expected. Repeated severe startup retry bursts. |
| Oct 6 20:32, 20:37, 20:44 onroad captures | Same new firmware/source vintage without a severe burst in the visible capture. The first two include zero CAN timeouts. Failure is not inevitable on every startup. |

Ten new SPI-error captures have ten distinct boot-log identifiers
(`40`, `41`, `43`, `45`, `47`, `48`, `49`, `4a`, `4e`, `4f`). Each contains
startup, safety selection and the severe burst. Their maximum transfer
windows are 2,357.568–2,589.509 ms; maximum attempts 31–34. Each records
55–66 stale sendcan discards and a first card diagnostic window with
88–92 CAN receive timeouts. These are receive-call timeout counts, not
88–92 distinct driver-visible CAN warning episodes.

The ten captures do not establish continuous failures throughout driving.
The user's report may cover additional periods outside these snapshots.

## Latest incident details

`spi_error-20261007-085736-carrot-wip.txt`, boot `0000004f--2b2f9a4c15`:

- CAN RX endpoint `0x81`: 31 attempts, including 30 header NACKs; final return
  1,984 bytes. Maximum recorded whole-transfer duration 2,495.137 ms.
- CAN TX endpoint `0x03`: 32 attempts, including 29 header NACKs, one data
  NACK and one data-ACK timeout; final return zero, which is successful for
  this write. Maximum recorded duration 2,503.861 ms.
- Subsequent TX and RX sequences each require 16 attempts. Firmware text
  includes repeated `SPI: CAN NACK` and malformed header
  `11 14 14 14 14 14 14`.
- Panda checksum counter increases from 1 to 20. Host-side receive-checksum
  diagnostic counts remain zero. Those are different checks at different
  ends of the link, not contradictory counters.
- 65 queued sendcan messages are rejected as over one second old; their
  logged ages range from 1,010.094 to 2,480.540 ms.
- `FCA11` (`0x38d`) and `SCC12` (`0x421`) are reported missing/timed out.
  These follow the transport stall; they do not establish separate ECU faults.
- First two card diagnostic windows report 89 and 20 CAN receive timeouts.
  A later onroad capture of this same boot reports a subsequent zero window.

## What the code establishes

1. In `panda/board/drivers/spi.h`, `SPI: CAN NACK` is emitted for a valid
   endpoint-3 SPI payload when `spi_can_tx_ready` is false. It is a Panda
   flow-control response, not a direct vehicle CAN checksum or CAN error-frame
   diagnosis. `refresh_can_tx_slots_available()` in `can_comms.h` restores
   readiness when **all three** CAN TX queues have sufficient free space.
   This identifies unavailable TX readiness, but cannot identify the blocked
   bus or distinguish backlog from a delayed readiness refresh here.
2. Host TX and RX share the SPI transport. Failed transactions execute
   resynchronization under the shared lock and retry. NACK retry termination
   is not bounded by elapsed NACK time; the timeout termination counts ACK
   timeouts. A retry sequence can therefore span seconds.
3. `0x11` is a host header-ACK poll byte and `0x14` is a host recovery probe.
   Seeing these as a Panda header is consistent with protocol state
   misalignment during recovery. The checksum counter alone cannot be used
   to diagnose electrical SPI corruption or a defective cable.
4. `pandad.cc` checks a queued sendcan message's age before calling the SPI
   send. Messages older than one second are discarded, explaining the burst
   of stale-send lines after the stalled transaction returns. This does not
   prove that the transaction already inside retry met its original deadline.
5. The vehicle selects safety model **8 (Hyundai), parameter 13**, before
   and after the update. This is the classic Hyundai path, not Hyundai CAN-FD.
   The CAN-FD cluster direct-TX switch and RX-paced cluster forwarding changes
   are not active fixes for this vehicle's observed transport failure.

These observations support a coupled SPI retry / TX-readiness failure.
They do not prove that TX backpressure initiated the first failure: initial
RX data NACK/header failures are also present, while firmware debug output
can be drained later than the underlying event.

## Update candidates and limits

The pre-update capture's source date corresponds to `e985b31141`; the first
new source HEAD date corresponds to `383e86d17e`, and later captures report
the date of `eb5bbf2720`. Dates and printed firmware identities identify the
observed change boundary but do not verify every deployed file as pristine.
Firmware source identities are content hashes, not Git commit IDs.

`git diff e985b31141 eb5bbf2720 -- openpilot/selfdrive/pandad
panda/board/drivers/spi.h panda/board/can_comms.h` is empty. Thus neither
new SPI retry code nor newly added SPI diagnostics is the direct changed
source in this interval. Existing recovery behavior can still amplify a
new timing or firmware trigger.

Two changed areas warrant controlled comparison:

- **Panda firmware / safety transition.** Firmware changed at the onset.
  The October 4 H7 change touches shared controller initialization and adds a
  preserved-controller handoff, but the preserved path is explicitly limited
  to Hyundai CAN-FD and does not apply to safety model 8. One failing Casper
  capture reports `mode=00000008 preserve=00000000 us=00003455`:
  13.397 ms for that transition, not 2.5 seconds. Blaming a 2.5-second
  controller INIT stall, or enabling this optimization for Casper without
  further evidence, is unsupported.
- **Host startup timing and work.** Old model load is about 1.5 s; severe new
  startup captures report 8.6–8.7 s and C4 Jetlink warp validation. New
  selfdrived initialization reaches its 6.01-second timeout before model
  readiness, then vehicle safety is selected. The new internal-model wrapper
  prepares and validates its Jetlink warp even when no external host is
  actively serving inference. DM also now starts. These changes provide
  plausible startup timing/overlap candidates, but the same 8.6-second load,
  initialization timeout and DM startup also occur in a capture without the
  severe SPI burst. They are not sufficient proof of causality.

No bus-off, transceiver failure, specific ECU failure, CPU saturation, or
firmware interrupt storm can be established without the missing telemetry.
Do not prescribe replacing the harness or disabling a CAN safety/validity
check on the strength of these captures.

## Next discriminating evidence

Collect segment 0 rlog for one failing boot and, preferably, one clean boot
on the same new software. Align first SPI failure with raw CAN delivery,
per-bus busOff/error/overflow counters, safety state and heartbeat, and
process/core timing. First determine whether a bus stopped draining before
the SPI failure or whether SPI/host service stalled first.

A subsequent parked comparison should isolate startup work from firmware
changes while keeping all safety checks and actuator limits. An undifferentiated
whole-version rollback can corroborate regression but cannot identify its
component. No such comparison or device change was performed here.

Private reproduction and file hashes:
`.analysis/archive/2026-10-07/casper-spi/INDEX.md`.

## Full-rlog follow-up: route 00001e9e--3c5e7ca46c--0

The user supplied this segment after the initial analysis. It was read from
the exact NAS route directory with the full openpilot cereal schema.
Metadata confirms C4/mici, clean `eb5bbf2720286f17fc55d6eab8a2f7aacecf1f3d`,
AGNOS `19.8-carrot-bt1`, Hyundai safety model 8 / parameter 13 and
openpilot longitudinal control. The SPI timing matches the previously
captured October 6 20:19 boot, rather than a new independent failure event.

All times below are relative to the first raw CAN publication, monotonic
49.656549874 s. Coverage ends around +61.33 s. This is startup **while the
vehicle is beginning to move**, not a stationary-only incident: about
18.9 km/h immediately before the gap, reaching 59.1 km/h later. Selfdrive
enabled/active remain false throughout.

### Measured failure sequence

| Time, s | Evidence |
| --- | --- |
| 7.398 | selfdrived reaches its 6.01-second initialization timeout, still waiting for model-related services. |
| 7.418 | First host sendcan publication. |
| 7.461 | Last Panda health before outage: ELM327, no buffer overflow or SPI checksum errors, all bus-off/error counters zero. |
| 7.472 | Last raw CAN delivery from each of buses 0, 1 and 2 before a shared 2.550498-second gap. |
| 7.482 | Host logs selection of Hyundai safety model 8 / parameter 13. |
| 7.520 | First logged data NACK, followed by repeated header NACKs. |
| 9.687–9.697 | Jetlink C4 warp validation completes; model load completes after 8.7 s. |
| 10.005–10.016 | RX and TX retry sequences finish with 32 attempts each; measured RX transfer 2,525.096 ms, TX 2,461.136 ms. No ACK timeout or host RX-checksum failure in these sequences. |
| 10.023 | Raw CAN delivery resumes with buffered data. |
| 10.043–10.180 | Original MDPS reports temporary ToiFlt/FailStat; the bits subsequently clear. Times are host delivery times, including backlog. |
| 10.137–10.211 | carState.canValid is briefly false during backlog recovery; canTimeout stays false throughout the segment. |
| 10.180 | First delivered original bus-0 TCS13.ACCEnable=3. It remains 3 through +61.322. |
| 10.192 | carState.accFaulted becomes true and stays true. |
| 10.213 | Displayed alert becomes `accFaulted/permanent`: `크루즈 오류 : 차량을 재가동 하세요`, remaining through segment end. |
| 10.353 | First returning Panda health: RX overflow 3,659; TX overflow 3,680; SPI checksum count 15; MCU interrupt load 85.5%. No heartbeat loss or Panda faults. |
| 10.643–10.663 | Original camera FCA11 reports FCA_Failinfo=3 and FCA_Status=3. Existing Casper suppression produces outgoing fault-free copies; this does not clear the original TCS13 fault. |

After the burst, SPI checksum and both overflow counters remain constant
through the end. There is no additional multi-second raw CAN gap. Persistent
displayed cruise error in this segment must not be described as continuous
new SPI failures throughout the rest of the drive.

All sampled busOffCnt, totalErrorCnt, receiveErrorCnt, transmitErrorCnt and
totalTxLostCnt remain zero on all three controllers. Bus 1's stored error
changes from noError to formError, without counted error accumulation or
bus-off; bus 1's core-reset count is already one at the first sample and
does not grow. This isolated stored field precludes saying every error field
is clean, but does not demonstrate a bus-off initiating this incident.

Panda health itself has a 2.891-second publication gap. Thus no health sample
exists inside the key failure interval. Bus-0/1 RX totals advance by 5,334 /
2,836 across the gap, showing that host delivery loss is not equivalent to
all ECUs ceasing to transmit. Bus-2 totals advance by only 203 across a
simultaneous relay/topology change; do not compare its pre/post traffic rate
as though the wiring topology stayed unchanged.

### Confirmed secondary defect: incomplete output survives SPI recovery

Recorded sendcan has **no host LKAS11, SCC11, SCC13 or SCC14** anywhere in
the segment. It contains 2,564 SCC12 and 2,565 FCA11 messages. This selective
omission is explained by the production classic-CAN discovery path:

1. `CANParser.seen_addresses` begins collecting only after
   `controls_ready=true`, set at ready-count 50.
2. `CarState.monitor_fingerprint()` attempts classic LKAS/FCA template
   registration only at count 104, and SCC templates only at count 105.
3. The outage spans those observation/registration ticks. Previously accessed
   parser dictionaries already exist for CLU11, FCA11 and SCC12, so those
   templates survive. LKAS11/SCC11/SCC13/SCC14 have no such dictionary and
   remain None. There is no later retry for this classic-CAN path.
4. `create_acc_commands_scc()` skips each None template independently. It
   continues generating SCC12 despite absent SCC11/SCC14.
5. `hyundai_fwd_hook()` gates **all of SCC11, SCC12 and SCC14** using the most
   recent accepted host SCC12 timestamp and a 400 ms threshold. Consequently,
   continued host SCC12 also blocks stock SCC11/SCC14, even though the host
   never produces replacements for those two messages.

The actual logger data supports both halves of this chain: original camera
SCC11/SCC14 resumes regularly, but each has only 11 Panda bus-0 returned
copies, all around +10.180–10.201 s; neither has another returned copy after
that, through +61.33 s. By contrast, SCC12 has 2,575 returned copies.
Panda returned copies are generated when enqueued to hardware, not independent
wire-acknowledgement measurements. They still establish the selective
host/Panda output omission.

LKAS11 and SCC13 have independent forwarding timers, so their missing host
templates do **not** imply complete loss of those messages on the vehicle
side. Their stock forwarded copies continue. Do not lump them together with
the SCC11/SCC14 group-blocking defect.

TCS13.ACCEnable is decoded directly from original bus-0 bytes at bit 43,
width 2. The DBC labels value 3 as `SCC permanent fault, communication issue`;
CarState maps any nonzero value to accFaulted. This is a vehicle-reported
communication-related inhibition, not just a stale openpilot alert. It does
not establish permanent physical damage. The initial 2.55-second interruption
and later missing SCC11/SCC14 are both credible contributors; this log does
not reveal the ECU's exact latching/clearance criteria.

### Recorded-input reproduction

Replayed the real CAN arrival batches at recorded CarState publication times
through the production CarState and CANParser, replacing only desktop Params
access with a local stub. ControlsReady write completion is not recorded;
three start-time assumptions spanning +7.398 to +7.482 s were compared.
All three retain missing LKAS11/SCC11/SCC13/SCC14 through segment end,
matching the observed host-message absence.

At the earliest assumption, count 50 occurs at +8.470 s, registration counts
104/105 at +9.750/+9.774 s, entirely inside the raw reception outage.
Moving the assumed ready time to the safety-selection log still leaves both
registration ticks before reception recovery. This is robust to that timing
uncertainty, although it is not an exact reconstruction of socket scheduling.

An analysis-only replay variant retries the four missing templates, requiring
an observed correct-bus address and a real accepted, correct-length frame no
older than 150 ms before exposing the template. It obtains LKAS11 at
+10.211 s, SCC11/SCC14 at +10.229 s and SCC13 at +10.355 s. This demonstrates
recovery of the missing template inputs. No production code was changed,
no output was transmitted, and it does not prove ECU fault clearance or a
fix for the first SPI outage.

### Revised investigation priorities

There is now a concrete host recovery defect to correct: retry missing classic
camera-SCC templates after an interruption and avoid an incomplete replacement
set permanently suppressing stock group members. Preserve real-frame validation,
message variants, safety allowlists, actuator limits and existing fault alerts.

The initial SPI outage remains a separate issue localized to the safety/relay
handoff. This segment has no bus-off evidence and no process restart. Model
startup overlap is real, but pandad remains on core 3 and modeld on core 7;
core load alone does not prove host starvation or Jetlink causation. Firmware
handoff/SPI-state diagnostics or a controlled parked A/B comparison are still
required before assigning that first failure to one changed component.

Reproduction scripts, compact data and source SHA256 are indexed in
`.analysis/archive/2026-10-07/casper-rlog/INDEX.md`. Original rlog stays on NAS.

## Follow-up: why the October 4 fixes do not cover this Casper

The two relevant October 4 changes are both explicitly CAN-FD scoped:
`b1485d40e3` preserves healthy H7 controllers only when the destination safety
mode is `SAFETY_HYUNDAI_CANFD`; `a32a570f17` retries CAN-FD camera templates.
Neither optimization covers this vehicle's classic Hyundai mode 8 or its
LKAS11/SCC11/SCC13/SCC14 discovery. A shared H7 MCU does not imply CAN-FD
vehicle policy. The new firmware is installed, but that is not evidence that
the optimized handoff branch ran. The captured mode-8 transition explicitly
reports preserve=0 and 0x3455 us (13.397 ms).

The mode command executes inside the SPI RX DMA handler, including relay/mux
changes and three controller initializations on the classic path. This makes
handoff a plausible disruption point. However, a 13.397 ms measured transition
in another failing boot cannot explain 2.5 seconds as time inside that function,
nor can it measure the transition duration of the supplied rlog itself.

The supplied rlog further separates protocol retries from later backpressure:
the first long RX sequence starts with a data NACK and then 30 header NACKs;
the simultaneous long TX sequence has 31 header NACKs and **zero data NACKs**.
The later shorter TX sequence has 13 data NACKs, matching the subsequently
drained `SPI: CAN NACK` prints. Therefore TX queue backpressure is observed,
but assigning it as the initiating cause of the long failure is unsupported.
Host retry/recovery work can extend a transient disruption; those transport
sources predate this update. The old/new capture duration difference supports
investigating an update regression, not dismissing the report as unchanged
historical SPI noise.

Common H7 changes in b1485d40e3 (initialization result bookkeeping, bounded
sleep-ack wait, mux-state recording and transition diagnostics) still execute
outside the optimized CAN-FD branch. Their regression potential is not ruled
out merely because preserve=0. Startup timing also changed: the recorded model
load grows from about 1.5 to 8.7 seconds, overlapping the six-second readiness
timeout and handoff. Neither candidate has a controlled same-device comparison
isolating it; the initial trigger remains unresolved. Recovery of classic
templates is a separately reproduced defect and cannot substitute for finding
that trigger.

## Historical tmux comparison: startup versus later short retries

Rechecked all 46 pre-update onroad captures (September 22 through October 6
08:53) and all seven pre-update SPI-error captures. These are terminal snapshots,
not complete trip recordings; file counts must not be used as failure rates.

| Observation | Pre-update captures | Post-update onroad captures |
| --- | --- | --- |
| Model loading | All 46: 1.4 or 1.5 s | All 12: 8.6 or 8.7 s |
| selfdrived initialization | All 46: timeout=false, dt 3.84–5.68 s | All 12: timeout=true, dt 6.01 s |
| Terminal ordering when both messages exist | Model loaded before Hyundai selection in all 45; one has no selection line | Hyundai selection before model loaded in all 12 |
| Visible old onroad SPI retry | One capture, Sep 27 12:36: two attempts, 5.745 ms diagnostic maximum, well after selection | Severe startup bursts described above; some new snapshots are clean |
| Reported pre-update onroad receive-timeout windows | All 28 printed windows have can_timeouts=0 | Severe first windows reach 88–92; clean new captures also exist |

The old SPI-error captures establish that protocol failures already happened
well into trips. The first visible failure follows these logger segments:

| Capture | Last preceding segment | Maximum printed transfer window, ms |
| --- | --- | ---: |
| Sep 23 10:00 | 68 | 30.983 |
| Sep 23 18:47 | 8 | 7.317 |
| Sep 24 13:33 | 43 | 22.628 |
| Sep 25 14:04 | 128 | 7.238 |
| Sep 27 12:48 | 15 | 12.034 |
| Sep 27 14:45 | 20 | 17.239 |
| Oct 1 10:21 | 60 | 10.123 |

Segment indices locate these after startup, not exact error timestamps; earlier
events may have scrolled out. All seven show the same `11 14 14 14 14 14 14`
header and data-checksum failures containing `13 13 ...` polling bytes. Those
signatures are therefore not newly introduced by the October update. All retry
diagnostics recover in two or three attempts, with no CAN NACK or stale-send
discard lines in these captures. Sep 23/24 each have a printed can_timeouts=1
window, so even the older period must not be described as absolutely error-free.
The Oct 1 capture has six transitions back to enabled after its first visible
SPI failure; Sep 24 has eight. This supports continued operation in the visible
windows, without proving the absence of every driver-visible transient alert.

The October 5 and October 6 08:53 captures still print source date September 25
13:44:39. The first update/error capture is October 6 09:11; the 09:16 onroad
capture prints October 6 09:02:07. Thus October 4/5 capture dates are not evidence
that the October 4 fixes had already been installed and worked on this device.

The most useful additional clue is the consistent startup-order reversal.
Historically, readiness completed after the model had loaded; now the six-second
timeout permits safety handoff while model initialization is still running.
This strengthens startup-overlap as a regression candidate, but clean new boots
with the same ordering prevent treating it as sufficient cause. The historical
signatures also support an existing SPI synchronization weakness whose duration
or exposure has changed, without proving identical root causes in every event.

Extraction and per-capture evidence: private archive `casper-spi/`
`historical_context.py` and `historical_context.json`. Source tmux lines lack
consistent monotonic timestamps; ordering is terminal order across processes,
not a reconstructed scheduler trace.

## Initial-loading CPU hypothesis

Re-read deviceState and procLog from the same full-schema rlog. Host pandad
and the kernel spi0 worker are observed on core 3. Source assigns pandad
FIFO priorities 54/55/56 (main/TX/RX), spi0 FIFO 1, and spi_geni IRQ affinity
to core 3. Modeld is assigned core 7. These assignments are unchanged across
the compared versions. ProcLog records the last processor of a process leader,
not every thread's affinity or execution history; IRQ affinity is source policy,
not independently measured IRQ routing in this rlog.

deviceState core-3 usage rises from 71–78% around +5.6 to +7.2 s to 90%, 90%,
92% at +8.590, +9.414, +10.213 s. It drops to 70% at +10.788 s. Cores 0, 2
and 7 are also near 100% during initialization. Two-second procLog deltas give:

| Interval, s | Core 3 busy excluding iowait | Core 3 system / IRQ | pandad process CPU, one-core basis |
| --- | ---: | ---: | ---: |
| 3.967–5.967 | 75.12% | 42.93% / 2.93% | 26.00% |
| 5.967–7.968 | 76.80% | 48.97% / 3.61% | 33.99% |
| 7.968–9.968 | 89.56% | 72.53% / 6.59% | 52.00% |
| 9.968–11.968 | 77.02% | 39.75% / 4.35% | 39.99% |

The central interval is wholly within the common raw-CAN delivery outage.
pandad accumulates about 1.04 CPU seconds in those two seconds (0.96 system,
0.08 user), so it was not simply unable to run throughout the outage. It was
doing substantial kernel/protocol work. spi0 accumulates about 3% CPU in the
same interval versus 0–0.5% before it. These accounting values do not establish
how much time either thread spent runnable but unscheduled, sleeping inside
an ioctl, awaiting a lock, or waiting for Panda hardware.

The long RX retry diagnostic has up to 41.851 ms lock-acquisition elapsed time,
10.505 ms header-ACK phase, 20.416 ms data-ACK phase and 31.943 ms recovery phase.
After recovery the shorter RX retry has maxima 2.784, 0.381, 0.850 and 1.212 ms
respectively. Lock timing includes contention and descheduling; none of these
is a pure scheduler-wait measurement. Substantial phase elongation supports
investigating host execution/driver latency along with MCU servicing.

An elevated core-3 percentage alone is insufficient: its +13.968–15.971 s
interval is also about 89.27% busy after the severe failure has ended.
During the outage, xiaoge_data uses about 188% process CPU, modeld 82.5%, and
soundd 87%; these are aggregate process values, not assigned core-3 shares.
They establish a busy startup environment, not which process delayed SPI.
Different CPU affinity for modeld does not eliminate shared driver/memory/IRQ
effects, but neither does this log prove those effects occurred.

CPU-related timing is therefore a concrete candidate, not excluded by unchanged
SPI source. Its causal direction remains unresolved because retry work itself
raises host and MCU load. Panda interruptLoad=85.5% after the gap is a separate
MCU metric, not the host core-3 usage. Panda firmware as a whole did change;
only the inspected SPI transport sources are unchanged. Per-thread scheduling
and ioctl/IRQ timing through handoff, ideally paired with a clean startup on
the same version, are the next discriminating measurements. No CPU placement,
priority, startup validity or production code was changed during analysis.

Private reproduction and CPU extracts are in `casper-cpu/` under the same dated
archive; the duplicate compressed input was discarded after extraction.

### What executes on host core 3

Core 3 is not exclusively reserved for Panda. Unchanged production code places
the native pandad main and inherited worker threads there (FIFO main 54, TX 55,
RX 56), normal/logging encoderd and its workers there (FIFO 52), and spi0 there
(FIFO 1). SPI IRQ affinity is also configured to core 3. pandad's main loop
handles safety selection, health/heartbeat, peripherals and serial-log reads;
its CAN workers perform RX/publication, TX and SPI retry/recovery. Its separate
diagnostic thread inherits the core rather than being moved elsewhere.
The MCU relay/controller initialization itself executes on Panda, not host
core 3; the host issues and waits for the corresponding SPI command.

The +7.968–9.968 s interval has pandad 52%, encoderd 9.5%, spi0 3% process CPU
on a one-core basis. User+kernel process time includes the kernel code invoked
on that process's behalf: do not add those numbers to core-3 system percentage
as independent work. Hardware video encoding does not remove host frame/queue,
driver/ioctl and output-buffer work from encoderd.

locationd, paramsd, lagd and torqued allow cores 0–3 with FIFO 5; carrot_man
also allows cores 0–3. Samples show some of these plus Bluetooth, micd, GPS and
kernel workers last executing on core 3 during startup. That is evidence of
shared placement, not a per-core CPU attribution for migrating processes.

Xiaoge's vision-server thread explicitly allows cores 0–3 before creating
camera/inference workers and requests two OpenCV workers. Its work includes
camera reception, lane inference and conditionally gated blindspot inference,
in addition to the process's 20 Hz data broadcast and HTTP/TCP services.
xiaoge_data consumes about 188–192% aggregate process CPU around this startup;
its leader is sampled on core 3 at +5.967/+7.968 s. This is a substantial
background workload candidate, not evidence that core 3 alone incurred 188% or
that this process caused the SPI failure. No thread-level profiler is present.
The inspected Xiaoge/encoder source files match the logged eb5bbf27 revision.
The model assets changed from Git LFS pointers to ordinary Git blobs in the
update interval; that storage change alone does not prove that old deployed
models were absent, or that inference first became active after this update.

Encoder FIFO 52 is below native pandad 54–56 but above the spi0 worker's FIFO 1.
This ordering is relevant to driver scheduling investigation; the recorded
encoder CPU rate is around 9% before and during the outage, and no direct
spi0 starvation measurement establishes it as the trigger. Model loading is
assigned core 7, while shared background work and kernel services still reach
core 3. No placement or priority was changed.

### Xiaoge as a contributing startup workload

Xiaoge CPU is already high before the SPI outage: 192.5% in +1.967–3.967 s,
191.5% in +3.967–5.967 s, and 192.0% in +5.967–7.968 s. It remains 188.5%
during +7.968–9.968 s and 189.5% in +9.968–11.968 s. Thus its substantial
workload predates the failure rather than appearing solely as a result of
SPI retries. The later segment still has substantial Xiaoge CPU without a
second multi-second SPI outage, so high aggregate use is not sufficient cause.

Manager's enable_xiaoge_data checks ShareData only. The vision service starts
camera workers after loading its models, and the lane worker does not wait
for ControlsReady, healthy Panda, modelV2 or engagement before inference.
Camera availability, a valid lane model and its configured interval are the
relevant lane-loop conditions. Blindspot inference has additional gates; do
not attribute startup CPU specifically to blindspot inference without a trace.

Both the Oct 6 08:53 old-version and 20:20 new-version settings snapshots have
ShareData=1, OnnxLaneIntervalMs=400, OnnxBsdIntervalMs=250 and identical threshold/
smoothing settings. Inspected Xiaoge sources did not change in this update
interval. There is no old rlog process-CPU measurement proving that the old
device had the same actual inference workload. Unchanged settings/source alone
neither establish nor exclude increased runtime work or a new overlap effect.

An appropriate causal comparison is repeated parked startup A/B/A on the same
software/settings, changing only Xiaoge execution (ShareData controls both its
data and vision services). Record actual process stop/start, model timing,
core-3 and per-thread SPI scheduling/driver timing, retries and CAN gaps. Merely
turning off a remote display may leave the local inference running. No such
device test or settings change was performed here, and a single clean startup
would not distinguish this from the observed intermittent clean new boots.
