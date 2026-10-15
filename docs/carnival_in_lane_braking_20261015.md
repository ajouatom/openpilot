# Carnival 225: corroborating a slowing vehicle already in the lane

The user requested a code correction and the complete maintained radar
validation list after delayed braking in `00000225--74da14534f--28`.
The vehicle ran clean carrot-wip `0e647cd2`, with radar mode 1 and corner
radar enabled. It already contained the October 14 stopped-corner correction.
No vehicle installation or parameter change is included in this work.

## Recorded failure

There are no invalid model/radar/control input messages or long producer gaps
in this segment. At video 15.50 s, corner 3776 appears at 77.38 m, but the
selected L1 is vision-only at 80.23 m and 16.47 m/s. Ego speed is 69.85 km/h;
the acceleration command is +0.11 m/s2. At video 16.00 s the corner is at
72.35 m, 12.35 m/s, with reported acceleration -5.88 m/s2, while vision-only
L1 remains at 85.99 m and 14.86 m/s. These are sensor estimates, not metric
ground truth or proof that all returns belong to the same physical surface.

Corners cannot own the configured primary L1. Ordinary auxiliary selection
requires CUT-IN evidence, including an outside-path history and range <=45 m.
This corner was born overlapping the path. Its trajectory also often receives
the lateral-jitter diagnostic, but jitter is not the sole rejection reason.
The October 14 fallback requires missing L1, stopping context and a nearly
stationary corner; it does not cover this moving-to-stop encounter.

Recorded front L1 first appears at log-relative 17.907 s, about 63.6 m and
3.2 m/s. The command reaches -4 m/s2 at 18.65 s. Driver brake input begins at
21.03 s with ego at 33.6 km/h and L1 around 13.7 m. Disabling only the October
14 fallback changes none of this segment's 1,202 reconstructed lead outputs.
This incident does not demonstrate a regression caused by that fallback.

## Correction contract

`InLaneCornerTracker` supplies an auxiliary measured L2 when L1 is absent or
vision-only. It leaves front/SCC L1, ordinary CUT-IN and stopped-corner policy
unchanged. A nearer existing L2 wins; the fallback does not create a CUT-IN
event or its sound.

- Both current measured inner lanes must satisfy the existing confident-lane
  contract (probability >=0.90, std <=0.25 m, plausible width, uncertainty
  clearance, near-path agreement, no lane change or extrapolation).
- The corner needs 350 ms of continuous measured motion, lane occupancy and
  reported braking <=-0.5 m/s2, with an actual speed decline >=0.30 m/s.
  Position/speed jumps, missing points, changed IDs and time reversal reset
  evidence. Repeated radar publications cannot accumulate confirmation.
- A rolling 500 ms range change must agree with integrated measured relative
  speed within the existing 2 m/s longitudinal error-rate allowance. This
  rejects cumulative motion disagreement that individual small steps hide.
- The model must independently predict substantial slowing (final velocity
  <=75% of ego speed). Current visual probability is >=0.50, with bounded
  position/velocity uncertainty. Both centers must be within the ego lane;
  lane-relative offset difference <=0.75 m, range difference <=10 m and speed
  difference <=4 m/s. Require three matching visual votes spanning >=100 ms.
- A second plausible measured corner immediately vetoes association, including
  one outside the 80 m output cap. Output is limited to 10–80 m, closing speed
  >1 m/s and radar modes 1/2/3. Current evidence loss withdraws output; there is
  no stale lane/vision extrapolation or unmeasured braking hold.

An initial broader in-lane association produced unwanted auxiliary roles in
cruising/adjacent-vehicle regression scenes and was rejected. The final change
is specifically corroborated braking, not unrestricted corner lead matching.

## Replay and validation

- Three new maintained Carnival cases cover early corroborated L2, no
  promotion before confirmation, and continuous front 41/36 primary after
  acquisition. The front case permits existing cross-sensor aliases and
  auxiliary handoff; forbidding corner aliases would also forbid the same
  front primary. Existing labels and deadlines were not weakened.
- All 102 logs / 502 labels were replayed in modes 1/2/3. The 499 prior result
  rows are identical to baseline in each mode. The three new cases pass.
  Existing expectation failures remain **11/10/13**, pre-deceleration failures
  **0/0/1**, and **208 unverified labels** remain. Missing logs: zero. This is
  not a clean corpus pass and the failures are not relabeled as successes.
- All-frame mode-1 A/B: 122,090 frames; L1 changes zero. Only 13 L2 samples
  differ: six in Carnival 225-28 and seven in Azera cf-15. The other 122,077
  outputs match. The Azera camera was reviewed around 48–51 s and shows
  approaching traffic in the lane; exact radar surface identity/range is not
  independently established. These are sparse auxiliary additions, not a
  measured on-road false-braking rate. No radar-only advisory fires.
- In the shared desktop/NAS replay, Carnival L2 begins at video 16.6021 s,
  66.35 m, and lasts six samples through 16.8508 s. This is 1.45 s before that
  replay's first front primary at video 18.0508 s. Reconstruction aligned to
  recorded radarState/carState also adds six samples, about 1.3 s before the
  recorded front acquisition. Consumed radar/pose timestamps are not logged
  in radarState, so these clocks and replays must not be silently conflated.
- The production lead extrapolator receives tighter obstacle input: at the
  first added sample, its 2.5 s stopped-equivalent obstacle is 88.11 m versus
  vision-only 136.82 m. This checks the input calculation only; no acados
  solve, acceleration-command replay, closed-loop stopping or road validation
  is claimed. Confidence loss still leaves a gap before front acquisition.
- Windows focused tests: 1,230 pass. Route-vault tests: 36 pass and four
  Windows symlink-permission skips. The image workflow includes the new tests
  with native Linux radar/CAN kernels before publishing the NAS image.

Raw recordings, exact-input traces, rejected-prototype results, all-frame
audits and complete validation reports remain private under
`.analysis/scratch/2026-10-15-carnival-225` and
`.analysis/scratch/2026-10-15-carnival225-fix`. Publish shared changes to both
maintained branches using single-parent commits. Verify the automatic image
workflow, NAS updater commit and the actual route page/recalculated data.
Branch/NAS publication is separate from installation in a vehicle.
