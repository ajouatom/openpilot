# Lead-muted signal notices and October 17 drive review

## Delivered change

The user requested no signal notices when a lead is present. All Carrot planner
signal notices now use current `leadOne.status`, without a distance threshold:
traffic stopping, green/departure, traffic-sign change (including navigation,
soft hold and mode-2 preparation), and the signal guard's refusal sound. This
suppresses the corresponding event/popup as well as its sound. Raw signal
recognition display remains available. Other uses of refusal, collision and
radar obstacle warnings are unaffected. Muted notices do not consume the shared
navigation-event cooldown. The current lead is read before navigation handling.

This publication changes notification emission only. Red latch, lead exception,
stopping/acceleration targets, driver holds and mode-2 preparation are unchanged.

## Evidence collection

Read-only vehicle inspection found carrot-nav `2ef833b201`, SignalAssistMode 2.
Collected 99 completed qlogs dated October 17, 17 selected full rlogs, and three
road videos; all 119 files match vehicle SHA256. The morning 07:42 route used
`fe8e698f94`; later routes used `2ef833b201`. Keep these versions separate.
The raw files and investigation scripts remain in the private local directory
`.analysis/scratch/2026-10-17-signal-drive`. No recording is published here.

## Findings

1. **Actual unintended movement while the forward signal was red, 11:05:20.**
   Route 10e4 segment 13 has no nearby queued lead. Crossing vehicles are visible
   in the video. At 11:05:20.132 a vision-only lead appears at 32.605m,
   vLead 3.092m/s, modelProb 0.688, radar=false. The logged filtered stop distance
   is 32.216m; the newly added relative-2m exception opens. It closes again at
   11:05:20.581, about449ms later. The controller had already left stopping and
   commanded positive acceleration; red hold then returns. No driver pedal input
   occurs. Peak speed is0.627m/s (2.26km/h), with about1.53m travel integrated
   from speed over the enclosing eight-second interval. A second brief exception
   occurs at11:05:22.236 for75ms. The exact crossing vehicle assigned to the
   vision lead is not established by these logs. The causal issue is immediate
   acceptance of a transient lead, not a confirmed green transition. This is an
   observed limitation/regression of the preceding lead-bypass change and is
   **not repaired by muting its sound**.

2. **State/notice oscillation with a stationary lead, 12:01–12:02.**
   Route10e5 segment4 has a stable lead at4.15m. Filtered model stopping distance
   crosses approximately2.15m, repeatedly changing the strict relative-2m test.
   Guard records show16 transitions between hold and exception, including eight
   exception entries. One example is2.152m allowing following at12:01:11.570,
   then2.146m restoring hold56ms later. Lead detection remains present and ego
   speed is essentially zero. The previous code rearms refusal warnings after
   each bypass; this explains repeated sounds but does not establish physical
   creeping in that stationary interval. Similar state changes occur around
   11:12 and13:43. The alert fix removes the lead-present sounds, not the state
   oscillation.

3. **A separate small following movement, 13:47:13–13:47:17.**
   Route10e6 segment23 shows the real black lead car moving slightly and stopping.
   Its estimated gap grows from4.65m to about5.46m. The planner remains in lead
   following with effective guard hold=false throughout; MPC shouldStop clears
   and returns as the gap changes. Ego peaks at0.141m/s (0.51km/h), about0.20m
   integrated movement over the enclosing eight seconds, with no driver pedals.
   This is a following response, distinct from the transient lead exception.

The next control correction needs verified continuity/assignment of the same
lead before granting the exception, plus a policy for the noisy stop-distance
boundary. Simply retaining the bypass indefinitely or increasing its distance
would also retain unverified crossing-vehicle permission. Recognition accuracy,
lead association and red/green transitions require separate validation. No
claim of solved creeping or safe automatic departure follows from this review.

## Validation and publication

- 724 desktop tests pass, including all four signal notice types, both immediate
  and cooldown paths, current-lead navigation timing, no-lead notices, unrelated
  radar warnings and retained red control for a far lead. Existing tests now
  expect the requested muted signal refusal when a lead remains present.
- Paired production-method replay of20,384 recorded model updates has identical
  state, traffic, cruise/stop targets, mode and helper decisions before/after.
  It removes41 refusal notices and one green notice, all with a lead. Replay
  uses default parameter/navigation context, empty model leads and disabled
  preparation; wall cooldown is disabled equally. Counts are replay events,
  not measured audible plays, and this is not closed-loop/native validation.
- Native Params is doubled on Windows;316 existing NumPy deprecation warnings
  remain. Publish single-parent commits on carrot-nav and carrot-wip. No vehicle
  installation or control-policy change is part of this notification patch.
