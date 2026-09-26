# Ioniq 9 startup CAN/SPI failure, 2026-09-26

## Scope and evidence

Full cereal decoding of `HYUNDAI_IONIQ_9 a17c20823455b2b2`, route
`00000525--4ee4e148c3--0`, plus the matching `spi_error-20260926-093745`
and `can_error-20260926-093751` tmux captures. The log reports C4/mici,
AGNOS `19.8-carrot-bt1`, commit `094b52ca6f`, and **dirty=true**; the latter
prevents treating the recorded checkout as a verified pristine build.
The approximately 61-second segment is entirely stationary, gear P, with
selfdrive disabled. This is startup evidence, not a driving disengagement.

Times below are seconds from the first recorded `can` event, not `initData`.
Panda debug text can be drained after the underlying event; its publication
time is not an exact hardware-event timestamp. Health collection also spans
multiple SPI reads, so close events cannot be strictly ordered from health
publication timestamps alone.

## Observed sequence

| Time | Evidence |
| --- | --- |
| 6.367 | Host requests `hyundaiCanfd`, safetyParam=157, alternativeExperience=1. |
| 6.377 | First recorded SPI header NACK, before the first published bus-off health sample. |
| 6.399 | Last bus-2 raw frame before its 5.317139-second gap. |
| 6.489 | `canState2.busOff=true`, transmitErrorCnt=252, totalErrorCnt=124, stored bit0/bit1 errors, canCoreResetCnt=1. Other buses have no corresponding bus-off. |
| 6.585 | Last bus-0/1 raw frames before a shared 4.635396-second host-delivery gap. |
| 6.598 | TX overflow reaches 1,745; parser reports `0x1a0 SCC_CONTROL not valid (timeout or missing)`. |
| 6.686 | `carState.canValid=false`. |
| 7.946 | `canError/permanent` alert becomes visible. |
| 11.204–11.206 | SPI TX sequence completes after 793 attempts (791 data NACKs, 1 header NACK); RX completes after 790 attempts (789 header NACKs). Logged maximum transaction durations are 4.612519 and 4.608417 seconds. ACK timeout counts are zero. |
| 11.221 | Bus-0/1 raw delivery resumes. |
| 11.678 | Health reports `heartbeatLost=true`, safety `silent`, power saving enabled. RX overflow=19,297, TX overflow=14,902, SPI checksum counter=795 (initially 1). |
| 11.716 | Bus-2 raw delivery resumes; SCC_CONTROL gap is 5.332171 seconds. |
| 11.737 | `carState.canValid=true`; total canErrorCounter=194. |
| 11.805 | Panda safety becomes `noOutput`. |
| 12.281 | CAN alert clears. |
| 61.254 | Last Panda health still says `noOutput`; safetyTxBlocked=13,445, controlsAllowed=false. |

`canTimeout=true` only during 11.229–11.720. Its narrower interval does not
negate the earlier missing-message invalidity or raw delivery gaps. All 105
deviceState messages remain started=true. No selfdrive enable occurs.

Panda faults remain empty: this incident does not reproduce
`registerDivergent` or an interrupt-rate fault. Bus-2 `busOffCnt` reaches 71,
but this is **not proof of 71 separate bus-off incidents**: the firmware adds
the current bus-off bit every time it updates CAN health.

## Interpretation against the recorded commit's code

The observed lower-level failure is a bus-2 CAN TX/bus-off problem near the
startup transition into vehicle safety, accompanied by SPI protocol failures.
Bus 2 is the Hyundai camera-side logical bus. The bus-off counters, bit errors,
and transmit-error counter distinguish this from an isolated DBC/checksum
parser problem. The log cannot establish whether the first trigger is harness,
transceiver, relay-transition behavior, forwarded traffic, or firmware.
In particular, initial header NACKs precede the first bus-off health sample;
do not claim all SPI errors were caused by bus-off.

The subsequent amplification path is supported by both code and capture:

- `panda/board/drivers/can_common.h` requires free slots in **all three** TX
  queues before `can_tx_check_min_slots_free()` succeeds.
- `panda/board/can_comms.h` uses this condition to resume SPI CAN TX.
  `panda/board/drivers/spi.h` explicitly emits `SPI: CAN NACK` when TX is not
  ready. The matching captures contain repeated instances of this message.
- `openpilot/selfdrive/pandad/spi.cc` retries NACKs; its termination test counts
  ACK timeouts, not elapsed NACK duration. Therefore a NACK sequence can persist
  for seconds. Error recovery sends `0x14` probes, also visible in Panda header
  diagnostics. The checksum increase alone does not prove physical SPI noise.
- Bus-0/1 Panda RX totals increase across the host-delivery outage, while the
  bus-2 RX total remains 40,343 through the failure. The common host delivery
  gap must not be described as all vehicle ECUs ceasing transmission.
- `panda/board/main.c` enters SILENT after lost heartbeat. Panda debug text
  explicitly reports five seconds without heartbeat.
- `openpilot/selfdrive/pandad/pandad.cc` converts SILENT to NO_OUTPUT.
  `panda_safety.cc` retains `safety_configured_` while onroad, so this recovery
  path does not automatically reapply the vehicle safety configuration.
  The recorded persistent noOutput state matches this code behavior.

Thus the cleared CAN alert establishes resumed reception only. It does not
establish restored control output or a repaired bus under active forwarding.
NoOutput also stops the traffic that could reproduce the failure.

## Follow-up boundary

Investigate the initial bus-2 fault around relay/vehicle-safety activation and
the coupled TX backpressure/SPI retry behavior separately. Any recovery change
needs explicit validation of stale-send rejection, heartbeat servicing, and
safety reconfiguration; do not bypass safety checks or blindly restore output.
This analysis makes no runtime, Panda firmware, settings, or threshold changes.

Private reproduction scripts, compact extracted evidence, and the two matching
captures are indexed under `.analysis/archive/2026-09-26/ioniq9-can/`.
Original route data remains on the NAS; local decoding caches are disposable.
