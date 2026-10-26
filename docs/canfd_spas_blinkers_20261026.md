# CAN-FD SPAS indicators — carrot-nav-blink trial

The supplied f8448120/04dd4119 patch is adapted rather than applied verbatim.
`CanfdBlinkerControl` defaults off and takes effect after reboot. Selection stops
the comma-side Jetson USB2CAN sender before its BEGIN handshake. The existing
Bluetooth/audioTurn/ATC policy supplies intents to a separate low-priority comma
worker; it does not grant steering or acceleration authority.

Eligibility uses Hyundai/Kia/Genesis CAN-FD, the configured ECAN bus, SPAS1
0x165/24-byte and SPAS2 0x16A/32-byte contracts, valid advancing counters/checksums,
and fresh compatible physical stalk input. It is not an Ioniq-only fingerprint
list. Matching addresses alone do not prove a vehicle accepts the commands.

Only a fresh request starts an extended diagnostic session. Positive responses
to session/communication-control requests and a quiet stock sender are required
before transmitting SPAS at at most 50 Hz. No blind 50 ms delay or suppressed
disable acknowledgement is used. Direction changes, manual/hazard input, stale
inputs, counter/CRC conflicts and session limits revoke the request. Recovery
requests communication enable and default session, then checks the response and
resumed stock traffic. Failed recovery locks further requests for this process.
Panda independently restricts bus/length/payload/CRC/counter/rate, diagnostic
sequence and bounded session/keepalive. A software failure is not proof of an OEM
ECU's timeout behavior; that restoration still requires vehicle verification.

## Parked comparison

`CanfdBlinkerTest` resets to zero at startup. Each new nonzero selection is a
single maximum-three-second trial, requiring P, standstill, zero speed, fresh
vehicle/control/Panda state and inactive controls throughout. Select zero before
repeating. An invalid request is consumed, never deferred until later movement.

| Value | Trial |
| --- | --- |
| 0 | Cancel |
| 1 / 2 | Left / right SPAS directional commands continuously |
| 3 / 4 | Left / right directional commands for 0.2 seconds, then zero SPAS direction; existing 0x1EA hold only after observed lamp feedback |

The short-plus-hold comparison is limited to the known Ioniq5PE hold layout. This
does not restrict generic continuous-SPAS eligibility. It does not claim the hold
method works after a SPAS start; it is the hypothesis being tested. Both methods
must check actual illumination, holding, cancellation and OEM ECU restoration.

## Current evidence

Passive owner-vehicle capture: ECAN0 SPAS1/24 and SPAS2/32 each 200 frames in four
seconds, all 400 CRCs valid, approximately 50 Hz. Bus1 also carries 0x165 but at
16 bytes: it is deliberately excluded. Packer counter-zero vectors match the
provided left db65/right 848c frames. No SPAS CAN was sent during this capture.

Local offline tests: 25 session/contract checks; combined intent/card/old-indicator
suite 165 passes including those 25; Wiki checks 25 passes. Native Panda CI and
physical continuous/pulse comparisons remain pending at this checkpoint.

## Follow-up: no APRK response and parked coexistence comparison

The first actual acknowledged trial on the owner's Ioniq5PE received no 0x7B9
response to 0x7B1 extended-session requests. No communication-disable request or
directional SPAS frame was emitted. Stock SPAS continued at about 50 Hz; lamp
feedback remained off, Panda blocked count remained zero. Bounded enable/default
recovery attempts also received no ACK, so the session locked further requests.
The contributor subsequently reported the same missing response and irregular
illumination when stock zero-direction and injected direction frames coexist.
This is supporting evidence for a command conflict, not proof of ECU location.
The bus-130 echoes are forwarding to bus 2; they do not establish that modifying
that forwarded copy would reach the body controller.

Additional settings 5/6 compare left/right continuous coexistence, 7/8 compare a
0.2-second start plus existing confirmed hold, and 9/10 supply the pulse-only
control. These tests do not send UDS or disable stock traffic. They are limited
to the known Ioniq5PE layout in the Python integration; generic acknowledged
SPAS eligibility is unchanged. Each lasts at most three seconds with a 30-second
cooldown. Panda independently requires fresh CRC/counter-checked zero wheel
speeds and EV P/zero pedal input on ECAN0, fresh valid idle stalk, idle stock
SPAS, inactive controls, exact payload/CRC/counter/rate and a fixed direction.
Movement, manual input, stale input or expiry revokes the trial. Python also
cancels on a loop gap or increased Panda rejection count; no deferred restart.
Only actual lamp feedback can enable the existing hold. Physical comparison is
pending until the new native safety tests and ordinary Git Pull/reboot finish.

Initial enable exposed an IntFlag-to-Capnp assignment error; the selected option
was disabled to recover startup, then both assignments were fixed to integers.
Actual schema regressions and normal Git Pull/reboot verified the corrected
f8dd37ca28, CP flag and Panda safety parameter. N16 and the Jetson runtime stayed
unchanged. The old USB2CAN worker is stopped and CAN1 is down/listen-only, TX zero.

## 사용 방법과 확인 범위

캐럿웹에서 `CAN-FD SPAS 깜박이 사용(시험)`을 선택하고 재부팅합니다. 일반 설치는
기존 Git Pull·재부팅 경로를 사용하며 Jetson USB 패치는 필요하지 않습니다.
SPAS를 선택하면 기존 Jetson USB2CAN 깜박이 경로는 중지됩니다.
`자동 깜박이 제어`의 기존 단계와 audioTurn 시작·정차 유지·회전 종료 정책을
재사용합니다. 지원 여부는 ECAN 신호 규격과 실제 응답으로 판정합니다.

정차 시험은 P·정지·제어 비활성에서만 최대 3초입니다. 연속 신호와 짧은
시작 신호+유지를 따로 비교하며, 후자는 아직 아이오닉5PE 유지 신호만 대상으로
합니다. 설정 이름이나 CAN 수신 확인만으로 점등 성공 또는 다른 차종의 정상
작동을 주장하지 않습니다. 실제 점등·취소·주차 ECU 복귀 결과를 확인해야 합니다.

## Actual coexistence results

Native signal CI 40315922993 at a45792a0e2 passes 1173 checks, 40 skips and
362 subtests. Local focused checks pass 189 (one Linux-only case excluded);
Wiki checks pass 25. Counts overlap. The first native run caught a corrupt input
being discarded before the vehicle RX hook: parked admission now consults the
common checksum/counter/quality status as well. Ordinary Git Pull/reboot installed
a45792a0e2; stored Panda signature matches the newly built panda_h7 image. N16
rejoined with shared input/compact reply; the pre-join Cinque label was inactive,
not evidence that a different external model was running.

- First continuous request: two initial frames rejected by Panda, no lamps; the
  observer cancelled immediately. Initial rejection cause is not established.
- Instrumented continuous repeat: 14 SPAS1 and 14 directional SPAS2 TX echoes,
  left lamp feedback observed, then cancellation after about 0.36 seconds.
  Recorded stock SPAS2 counter jumps 0x3c to 0x3e at the cancellation. No new
  Panda rejection. This is not a completed three-second continuous comparison.
- Pulse-only control: eight directional frames over 0.175 seconds, no lamp
  feedback in the 12-second observation, no new Panda rejection.
- Pulse-plus-hold: eight directional frames over 0.181 seconds; raw left lamp
  feedback is on approximately 1.070 to 1.470 seconds after observation start.
  Existing 0x1EA left-hold bit is seen from 1.135 to 1.282 seconds, but a stock
  counter discontinuity cancels the test at about 1.263 seconds. Holding for the
  intended three seconds was therefore NOT tested to completion.

The pulse-plus-hold observation also records bus-2 totalErrorCnt 120 -> 142.
Bus-off count remains 95, TX-lost remains 1, RX-lost remains 0; bus 0/1 error
counts remain zero. The pre-existing bus-2 counts must not be attributed to this
trial. This delta is being compared with passive observation, not assumed to be
proof of the SPAS wire-error mechanism. Further active trials were stopped.
No ECU diagnostic command was sent by any coexistence trial. No right-side or
physical visual acceptance is claimed. Test setting is reset to zero in each
observer's finally block. The short-start/hold idea remains unverified; this
experiment must not be described as a reliable automatic indicator replacement.

Passive follow-up: 60.030 seconds with test=0, all P/inactive guards maintained.
Bus-2 totalErrorCnt stays 142, busOffCnt 95, TX-lost 1, RX-lost 0; bus 0/1 errors
stay zero. This strengthens the association with coexistence trials but does not
identify the exact controller/arbitration/forwarding mechanism. Final N16 model
1201 valid consecutive outputs, mean 40.675 ms, P99 44.904 ms, max 47.820 ms,
zero >50 ms; DM/both cameras/livePose 1200 valid consecutive, odometry 1201;
pose healthy and link/model/HUD/signal 60/60. NAV route is absent after reboot,
so no valid-NAV load, hard-deadline or driving claim. Snapshot hold is absent,
normal onroad state restored, AutoBlinkerControl2 preserved. Jetson host remains
0d4947499931; USB2CAN workers absent, CAN1 down/listen-only, TX=0. The initial
two Panda rejections remain in the cumulative counter (2), with no later increase.
Raw evidence remains private. Only carrot-nav-blink is published; no wip merge.

## Correction after the owner's patch-fidelity challenge

The previous conclusion that APRK shutdown must be solved before SPAS can light
an indicator was too strong. Both the contributor report and our left-lamp CAN
feedback demonstrate illumination while the stock sender continues. The existing
trials do not establish sustained holding, and do not reproduce the supplied
patch closely enough to declare its method unsuccessful.

Re-read both supplied patches f8448120 and 04dd4119 and execute their combined
`aprk_blinkers` method offline, with the real existing CANPacker/SPAS builder and
Params access blocked. With P/zero speed, menu test=left and no diagnostic replies,
300 controller ticks generate extended-session at 0.00s, `28 83 01` at 0.05s,
SPAS2 from 0.06s every 0.02s (147 frames through 2.98s), tester-present at 1/2s,
and default-session after cancelling. First SPAS2 is the supplied db65 counter-0
vector. The current acknowledged Session with fresh stock and no reply emits zero
SPAS2 commands. This is a sequence/packet comparison, not a new physical test.

Important differences introduced by our adaptation:

| Item | Supplied patch | Current adaptation / trials |
| --- | --- | --- |
| Diagnostic replies | Not checked; fixed 50ms delay | Positive replies and quiet stock required in normal path |
| Disable payload | `28 83 01` | `28 03 01`, requesting positive response |
| Stock stream | Can coexist; no counter-gap stop | Consecutive stock counter gate stops parked comparison |
| Cadence | Controller `frame % 2 == 0` | `next = now + .02`; actual observed mean TX-echo intervals about25-26ms |
| Initial counter | Packer's independent counter starts0 | Seeded from observed stock counter+1 |
| Test duration | Continues until menu0 | Maximum3s; actual comparisons ended earlier on guards |

Measured echo intervals: continuous14frames/0.3353s =25.793ms mean interval;
pulse-plus-hold8frames/0.18145s =25.921ms; pulse-only8frames/0.17497s =24.996ms.
These include host observation timing and are not exact hardware wire timestamps,
but do not support a claim of an equivalent 50Hz reproduction. Our guard-driven
cancellation also removed the hold bit, so subsequent extinction does not prove
that the existing hold mechanism cannot sustain a SPAS-started indication.

The CAN error increase remains an actual observation requiring investigation;
it is not erased by this correction and its exact cause is not established.
No actuation safeguards or runtime code were relaxed in this re-review. The
correct status is: lamp feedback confirmed, faithful original-method reproduction
and sustained hold acceptance still incomplete.
