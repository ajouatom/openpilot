# Indicator timing review and TBT wheel return — 2026-11-02

The owner separated two inputs: previous deliberate manual-indication attempts
for response/cancellation review, and the latest lunch drive only for a long
turn indication. Only the subsequently requested TBT wheel-return behavior is
changed in production. Manual prediction/cancellation trials remain offline.

## Lunch incident

Ten closed full rlogs from the two lunch trips total127,897,928 bytes, matched
against hashes calculated on the comma. Both run4a555c75b8. Decode uses the full
cereal schema; raw logs, video, precise route IDs and detailed traces stay private.

The return trip's segment3 has one TBT audioTurn cue at19.355s, left,41m ahead.
CAN lamp feedback goes on at22.512s; the source requests left hold at23.772s.
The wheel returns below30 degrees near28.2s, after roughly90 degrees of measured
vehicle rotation, but the original distance condition keeps holding. Speed
falls to about5–8km/h, and subsequent steering corrections also prevent the old
10-degree/0.4-second completion gate from settling. The old source completes
at36.152s; carControl releases at36.123s, lamps extinguish at37.002s. These
services have separate sample/publish times. Lamp feedback lasts14.49s.

Wall-clock lamp interval:12:09:58.014–12:10:12.504 KST on November2.
This is an intentional software hold waiting on distance/centering, not evidence
of a stuck USB transmission. Initial send/retry and delayed illumination are a
separate observation and are not claimed fixed here.

## Implemented TBT change

After selected-side steering has exceeded30 degrees AND measured vehicle
rotation is at least20 degrees, a wheel return within±30 degrees at>=1km/h ends
the current turn hold in that source cycle. No0.4s settling or remaining30m
travel is required for this completion path. A straight wheel on approach or
while waiting before turn entry does not complete the turn. Paused/stale inputs,
manual cancellation and300s maximum remain enforced.

Distance remains a fallback: original turn point+30m, with the existing bounded
extra30m if continuing to turn. A fresh next-turn cue now stays separate from
the current distance anchor even for same-direction turns, so wheel-return
completion cannot discard that next cue. It hands over on current completion;
same-direction Policy ownership keeps the existing ID without another tap.
Receiver encoding, CAN, Panda, BT behavior and manual-indicator rules are unchanged.

Five20Hz sampling phases replay both lunch routes with the same source input
validity, navigation and pose gates. Outbound trip has no TBT requests in either
version. Return trip completion is36.026–36.066s before versus28.196–28.236s after:
7.80–7.85s earlier in matched phases. These are hold-request decisions, not
counterfactual measured lamp extinction. OEM one-touch flashing can outlast release.

## Offline manual review

Use the prior November1 deliberate-attempt drive:12,755frames,20 current-code
requests under fixed recorded inputs/ownership. The comparison is not a physical
closed-loop replay; earlier lamps would themselves change later feedback.

| Trial | Existing requests advanced | New candidates |
|---|---:|---:|
| Current0.3s/cap1.2m | baseline20 | 0 |
| 0.4s/cap1.3m | 4,0.046–0.204s | 1 |
| 0.5s/cap1.4m | 6,0.046–0.298s | 1 |
| 0.5s/cap1.2m | 5,0.046–0.298s | 1 |
| 0.6s/cap1.5m | 8,0.046–0.598s | 1 |

The additional right candidate occurs in segment3 near55.4s (55.7s with1.2m cap),
just before the vehicle slows below the30km/h admission threshold. Hash-matched
2,309,956B video shows roadside parked vehicles; a distinct adjacent travel lane
and deliberate lane-change intent cannot be certified from this image. Expanding
admission without this caveat would overstate validation. No manual tuning shipped.

Using the earlier measured0.206s median request-to-lamp latency as a fixed
assumption, current code has7/20 projected lamp samples before the nominal0.9m
centre-to-boundary reference;0.5s trials have10/21. This is NOT a measured tire
crossing or an illumination guarantee. Tire position, instantaneous latency,
late intent and geometry gates prevent such a guarantee from these logs alone.

Opposite driver torque alone occurs while still approaching the right boundary
in segment7 near9.511s: that can be countersteering, not an abandoned maneuver.
An offline trial requiring opposite pressed torque AND at least0.05m actual
retreat changes only one of20 hold endings,9.858->9.803s (55ms earlier), while
keeping the existing0.2m-return fallback. This is insufficient evidence to claim
reliable immediate manual cancellation; manual production logic is unchanged.

## Checks and delivery

710 local indicator/HUD checks pass;2 platform skips. New tests cover both
turn directions, exact30-degree return, pre-turn stop/straight wheel, insufficient
rotation, stopped turn, next same/opposite cue, same-ID continuation, invalid pose,
expiry and source hold release. Existing actual-source-loop test now requires
release on measured turn and wheel return, while retaining the stopped approach.

Normal carrot-wip Git Pull and comma restart deliver this comma-side change.
No Jetson bundle/engine build, vehicle writes, live CAN test or reboot in this task.
Physical driving acceptance remains unverified.


## Publication verification

Source08f99d4e41 is published on carrot-wip. Exact-source Jetson integration
run41452434830 passes2224 checks/56 skips plus232 monitoring,62 HUD,94 update,
119 navigation,11 storage and11 network checks; Windows also passes. The first
run41451133382 remained in the HUD step and was cancelled after this unchanged
source rerun succeeded; no root cause is asserted for that runner delay.
Full build/tests41451133200, eGPU compatibility41451133375 and user docs41451133407
pass. Wiki sync41451725370 passes; all three AutoBlinkerControl language pages
have matching reviewed semantic hashes at Wiki d06cf2d.
