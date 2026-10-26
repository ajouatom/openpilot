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


## Repeat observation and ten-second pulse/hold request

Owner-visible continuous repeat completed its three-second command window:
114 SPAS2 TX echoes over 2.971 seconds, with four left-lamp on/off cycles in
0x413 feedback. No UDS or 0x1EA hold was used. CAN bus2 totalErrorCnt stayed
142, busOffCnt 95 and TX-lost 1 unchanged; Panda blocked stayed 2. Test reset
to zero, P/zero/inactive remained confirmed. This does not reproduce the
contributor's original 50 Hz cadence or establish the prior error's cause.

The owner then explicitly requests a short start followed by the existing
hold for ten seconds. Only settings 7/8 now allow a ten-second total window.
Directional SPAS remains limited to the first 0.2 seconds; matching lamp
feedback is required for the existing 0x1EA hold. Other tests remain three
seconds, Panda's independent directional trial limit remains three seconds,
and the 30-second cooldown from start and all previous guards remain intact.
Offline tests exercise both directions with/without confirmation through
expiry and unchanged-selection no-restart. Live ten-second acceptance pending.


## Ten-second pulse/hold attempt: start-stage bus error, hold not reached

Published and normally installed 9cd699d29c after Linux CI40319488152 passed
1177 tests/40 skips/362 subtests (81 local focused and25 Wiki checks overlap).
Comma reboot restored onroad with snapshot false, P/zero/inactive and N16 active;
Jetson runtime and Panda safety source unchanged. Reboot reset error counters;
do not compare pre-reboot142 to post-reboot0 as a recovery measurement.

Six-second passive preflight had stable counters. Setting7 then emitted only
two SPAS1/SPAS2 pairs (TX echoes at1.126669/1.157340s after observation start).
Stock SPAS2 skipped EB (EA then EC), ending the direct trial at~1.187s; the
observer saw bus2 totalErrorCnt0->200 and cancelled to0 at~1.282s. TX error
counter177/error-passive were present; busOffCnt remained0, Panda blocked0.
No lamp feedback, no UDS and no 0x1EA directional hold bits were sent. Therefore
the requested ten-second hold was NOT reached or validated; this is a start
stage failure, not evidence that the existing hold fails.

The injected EA/EB counters are independently incremented after seeding from
stock+1. Stock EA also arrived with different payload. These guards validate
each stream, not exclusive ownership, counter uniqueness across senders or
wire-time separation. Direct tests keep the OEM transmitter and stock forwarding
active. Injection is on bus0 but the observed controller errors are on bus2;
the record does not identify the exact failed physical frame or prove a loop.
The saved data-phase Bit1Error means transmitted recessive monitored dominant
outside arbitration, per Bosch M_CAN manual3.3.1. Same-ID simultaneous differing
payloads are a candidate, not a demonstrated root cause from these echoes.

After cancellation the count settled at201. Passive60.025s:201->201, TX/RX error
counters0, warning/passive/bus-off false, all losses unchanged. Both lamps off,
test0, owner mode2 unchanged, no manual reset or extra active trial. N16 model,
DM, both cameras and odometry each1200 valid consecutive; pose1199 all healthy.
Model mean40.658/P9944.794/max47.417ms,0>50; link/HUD/signal60/60, NAV absent.
Raw evidence is private. No full ten-second physical, road or error-root-cause
acceptance claim.


## Harness mapping correction and one-packet phase comparison

Live Panda reports harnessStatus=flipped. The health array is indexed by physical
CAN controller, whereas CAN message sources use logical buses. can_set_orientation
maps physical2 to logical0 when flipped. Thus the reported canState2 errors are
on the same logical ECAN0 as the injection, not evidence of a bus0-to-bus2 fault.
Earlier references to "bus2" health must be read as physical controller2 here.
Also TX echoes are emitted when queued into the controller FIFO, not proof that
a frame was successfully transmitted on the wire (fdcan process_can).

Passive8s capture with test0 sees400 stock SPAS1/24 and400 SPAS2/32 on logical0,
and their400+400 returned echoes on logical130 (forwarded logical2); no SPAS
origin on logical2. No new errors during capture. No forwarding change justified.

New parked-only settings11/12 send each SPAS address once after a NEW stock RX
with source receipt age <=5ms, within an initial200ms opportunity. SPAS1 must
precede SPAS2; counters seed from each freshly observed stock value. Matching
lamp feedback then permits the existing hold, at most10s total. Existing P,
CRC/counter/stalk/freshness/rejection guards and Panda3s/30s limits stay. This
reduces repeated transmission and tries an earlier phase; userspace timing is
NOT a guarantee of physical wire separation. It is not the contributor's UDS
sequence, not a production automatic replacement, and not yet physically proven.


Single-packet setting11 actual trial: one queued packet per address, no lamp
feedback and no hold; all error/loss/rejection counts unchanged across16s.
The owner's ten-second hold was not reached. db85fa466a was normally installed;
CI40321469953 passed1182/40skip/362subtests. Postboot pre-existing physical0/1
error counts1/1 and physical2 zero stayed unchanged (no reset to hide results).

Follow-up changes only11/12 to a fresh-receipt phased pulse: at most one packet
per new stock source receipt, <=5ms age, >=15ms per-address separation, <=10
packets/address and only during the first200ms. First counter from fresh stock,
then independent consecutive TX counters. Existing hold still requires lamp
confirmation; other guards, modes and Panda remain unchanged. This retains the
previous no-error timing experiment while testing whether a short pulse train
rather than one packet starts illumination. Physical acceptance pending.


Phased200ms actual setting11:6 SPAS1 and5 SPAS2 queued, no CAN error/loss/rejection
increase. Raw left lamp on1.2274s/off1.6354s; existing left hold bit on1.3146s
through11.1707s (9.856s). Last SPAS2 queued1.2969s. This is an actual completed
hold-command trial without sustained illumination, not an aborted hold test.
No UDS. Follow-up11/12 permits bounded start/hold overlap: stop the phased start
100ms after first lamp confirmation, absolute500ms/25packets per address maximum.
No extra start frames after that, no restart or deadline extension by feedback.
This tests the transition ordering; physical acceptance remains pending.


## Ten-second SPAS-only hold comparison

The fresh-stock phased start/existing-hold test completed without increasing CAN
errors, but did not sustain lamp feedback. The first 200 ms variant queued five
SPAS2 packets and transmitted the existing left hold for about 9.856 seconds;
lamp feedback showed one flash. The overlap-capable variant queued 14 SPAS2 packets (last at 1.615s);
first lamp feedback arrived at 1.648s and the existing hold began at 1.715s,
after the 0.5s pulse cap. Therefore actual overlap was not achieved in that
window. Existing hold continued to 11.128s but lamp feedback ended at 2.045s. Both are completed failed hold comparisons, not guard aborts.
Queued Panda TX echoes are not independent evidence of successful wire delivery.

At the owner's request, parked tests 13/14 repeat only SPAS left/right for at most
ten seconds. They send at most one packet per fresh stock receipt (age <=5 ms),
retain the 15 ms minimum interval, independent sequential counters, 500-packet
per-address cap, P/zero/inactive/fresh-driver/stock guards and 30-second cooldown.
No existing 0x1EA hold or UDS command is sent by these test modes. Panda's parked
absolute bound is ten seconds; ordinary source tests 5/6 remain three seconds.
This is a bounded parked experiment, not a driving-mode fallback. Actual ten-second
lamp acceptance is pending; offline tests cover expiry and counter wraparound.


## SPAS-only comparison outcome and patch attribution correction

Normal Git Pull/reboot installed 49512be286, including 5e0a2572e0. Actual Panda
signature matched the newly built panda_h7.bin.signed. After fresh parked/inactive
and six-second stable CAN preflight, test13 queued 14 packets per SPAS address.
First lamp feedback was at t=1.560834s. The observer cancelled test0 at t=1.679513s
on physical-controller2 error count 0->13, data bit1Error and transmit error count
99/warning. There were no safety TX rejections, no UDS requests and no 0x1EA hold
bits. Ten-second holding was NOT reached. No right/repeat test followed.

Passive recovery60s kept total errors13 unchanged, transmit/receive error counters0,
no warning/passive/busoff/loss increase. Model/DM/road/odom/pose each1200 valid
consecutive, wide1201; pose healthy, model mean40.805/P9945.298/max47.500ms,
zero above50; link/model/HUD/signal60/60. NAV absent. This is parked observation,
not a road or hard deadline guarantee.

The owner correctly challenged attribution of this failure to the contributor.
The supplied original sends 021003, then 03288301 after five100Hz controller
frames, then SPAS on frame%2 with packer counters starting0, plus 023e80 each
100frames and 021001 on cancel. It does not wait for positive responses. Our
normal Session instead requests acknowledged03280301 and requires ACK/quiet;
the recent direct tests omit ALL UDS, seed counters from stock and use different
scheduling. Therefore these actual tests are NOT faithful original-patch vehicle
validation. The original contributor's report is neither disproven nor independently
confirmed. Retract any statement implying that the original patch failed or that
momentary illumination established successful sustained operation.

The direct trial source now independently watches all three physical-controller
health records: missing health, warning/passive/busoff, or changed error/loss counts
stop and latch the current trial off. It no longer relies solely on the external
observer to cancel on these conditions. This is a stop-condition improvement,
not a fix for coexistence contention, and does not authorize a driving fallback.


## Owner-requested supplied-patch sequence reproduction

After the owner's explicit request to run the supplied patch, tests15/16 reproduce
its wire sequence using actual CarController.frame and native CANPacker with the
existing create_spas_messages helper: 021003, five controller frames later03288301,
then frame%2 SPAS1/SPAS2 and frame%100023e80, with021001 on cancellation. The native
packer is retained across trials, matching the original counter ownership. Neither
positive ACK nor stock silence gates this sequence. An independent reference keeps
the supplied f8448120 method plus04dd4119 menu override for byte/tick comparison.
Six1100-frame left/right/start-offset comparisons match every original packet,
including cancellation and restart. Desktop focused127 checks pass.

The test wrapper, explicitly separate from the original packet algorithm, requires
fresh actual P/zero/standstill/inactive/manual-input/CAN-health before arm and keeps
those guards during the trial; maximum10s and30s cooldown. Panda admits suppressed
03288301 only after the prior extended-session command under these parked inputs,
retains exact payload/CRC/counter/rate limits, revokes on motion/stale/manual inputs
or watchdog/absolute expiry and permits exact021001 recovery. Normal driving still
uses the existing acknowledged path. This is not unrestricted original-patch road
installation. No physical result is claimed until the actual test completes.


## Actual supplied-sequence test on the owner's vehicle

5a3889e6bd was installed by normal Git Pull/reboot; the active Panda signature
matched newly built panda_h7.bin.signed. CI40329253439 passed1241/40skip/362subtests.
No Jetson runtime/model change. Fresh P/zero/inactive and six-second stable CAN
preflight preceded test15. Actual Panda queued-TX records (observer-relative time):

| Time (s) | Record |
| --- | --- |
| 1.085304 | 0x7B1 `0210030000000000` |
| 1.143987 | 0x7B1 `0328830100000000` |
| 1.163565 | SPAS1 counter0; SPAS2 exact supplied left vector `db65...60...` |
| 1.339635 | SPAS1/SPAS2 counter9 |
| 1.353963 | Source reports patch_sequence_ended, then trial0 |
| 1.361339 | 0x7B1 `0210010000000000` |

Each SPAS address queued10 packets with counters0..9, mean interval19.563ms;
maximum observed SPAS2 interval35.269ms. These are aggregated Panda RX/TX-echo
records, not independent wire-level timestamps or proof every queued frame was
successfully transmitted. There were zero safety TX rejections. Logical0 stock
SPAS1/2 remained present after03288301 (10/9 frames before restore). No0x7B9
response was observed on the captured logical0 stream. Physical controller2
(logicalECAN0 with flipped harness) totalErrorCnt rose0->22 with bit1Error;
source and observer cancelled. No lamp-on feedback was captured. No right/repeat
trial followed. The planned ten-second hold did not complete; this does not prove
what an unrestricted error-ignoring run would do or disprove another vehicle's
reported result. It does establish that this guarded original-sequence run on the
owner's car developed CAN errors, rather than being blocked for lack of ACK.
Default-session recovery was queued; actual APRK acceptance was not acknowledged.


Post-cancel passive60.033s: error counts physical0/1/2 remain1/0/22,
pre-existing physical0 RXlost1 unchanged; all transmit/receive error counters0,
no warning/passive/busoff or safety rejection. N16/link/HUD/signal60/60 active;
model1200 valid, mean40.742/max46.878ms, zero>50. One observed model/odom frame
3616->3618 gap and wide3617->3619 gap remain noted; road/DM/pose consecutive,
pose healthy. Do not claim all streams consecutive or a road/latency cure.
Test0, owner AutoBlinkerControl2, P/zero/inactive, snapshotfalse/onroad restored.
No additional active/right test or Jetson update was performed.


## Counter-aware lever comparison (prepared, physical result pending)

Review pins joongyu01/carrot-wip-dev at 712cef9a72d8fb7e2f478bce63adeea3084fa303.
Modes 17/18 reproduce c931db45 neutral-lever payload: same unrelated bytes,
COUNTER_ALT +1 modulo16, CRC8 poly1D/init0/xor65, left/right latch bits, 20Hz.
Existing USB2CAN tap/release uses the same CRC but a different sequence.
No 100Hz competing SPAS, APRK command, USB2CAN activation, or automatic driving
integration is included. The vehicle source restricts the comparison to Ioniq5PE,
fresh P/zero/inactive and unchanged CAN health; any OEM lever/light/counter change
ends the trial. Panda independently checks fresh parked inputs, exact unrelated
bytes/CRC/counter/direction, 40ms minimum interval/150ms gap, a ten-second lease
and thirty-second start cooldown. Ending stops replay as upstream does; actual
lamp cancellation and ten-second holding require physical observation.
Local 135 focused checks pass before native CI and installation. No physical
success is claimed here.


### Physical counter-aware lever result

Native CI40332262548 passes 1275 checks, 40 skips, 362 subtests. Normal
Git Pull/reboot installed 0071e533a4; actual Panda signature matches built H7.
Under fresh P/zero/inactive guards, test17 ran the ten-second left request.
178 queued 0x3C1 frames, all `85c0004000000000`, span 9.973850s. The host
50ms minimum schedule quantized to about 17.75Hz in this run, not an exact
reproduction of upstream frame%5/20Hz cadence. No APRK or SPAS test TX.
Panda rejection stays zero; physical CAN error totals 0/2/0 unchanged;
no warning/passive/bus-off. Both carState and raw 0x413 show no lamp-on
feedback. This is NOT a successful lamp/hold result, nor proof upstream
cannot work on another bus/vehicle. No right/repeated trial was attempted.
Owner requests immediate existing USB2CAN restoration on failure. Test0,
CanfdBlinkerControl0, AutoBlinkerControl2 retained; normal reboot requested.
Restoration verification is recorded below once the existing session returns.


Restoration completed through the normal settings/reboot path: SPAS flag absent,
Panda hyundaiCanfd param157 instead of4253, trial0/mode2, snapshotfalse/onroad.
Fresh P/zero/inactive feedback; Panda physical CAN totals0/0/0, TX/RX errors0,
no warning/passive/bus-off/rejection. Comma blink_source runs without spas;
Jetson's existing blink_can worker restarts on unchanged runtime0d4947499931,
PEAK can1 UP at500k LISTEN-ONLY while idle, TX/RX errors0, drops48 unchanged.
N16 active/ready with original01effc67 SHA. No further lamp injection or road
validation was done after restoration; worker/session restoration is not a new
physical acceptance claim. Both rebooted temporary hold watchdogs have exited;
startup snapshot state is restored. Raw observations remain private.
