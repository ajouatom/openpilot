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
