# Signal helper revalidation, 2026-10-11

The candidate improves several recorded failures, but **is not promoted to vehicle control**. Additional-latency tests still miss red acquisition, and one newly produced green release lacks visually verified green evidence. The vehicle remains on `c115ce4f424b7f7173b1b009cbd406ecf5c661dd`, assist OFF, dual-observer comparison ON. This investigation changes neither its installed code nor settings, driving ONNX, x/v, or actuators.

This follows [the two 1095 red departures](signal_1095_red_departure_20261011.md). The user requested revalidation of accumulated recordings and installation conditional on sufficient improvement. That installation condition is not met. Software tests passing do not establish signal-recognition accuracy.

## What the candidate changes

This is hand-written OpenCV detection/tracking, **not neural-network training**. It finds horizontal housings and colored lamp cores, tracks image correspondence, and tests consecutive red/green evidence. The original internal driving network remains independent; its SHA256 is `f73a9e535523d5e9acb9e642c64e33d631825dc8ba74123757d107cedd047bb5`.

- Retain the previous housing proposals inside the forward ROI and supplement them with daytime colored-core proposals. Replacing housing proposals entirely regressed earlier dusk clips.
- In the explicit robust-tracking experiment, retain red-observed object identity for at most 500 ms. The color streak still breaks after 250 ms; final consumer freshness stays 200 ms. Identity retention never makes an old observation fresh.
- Prefer template correspondence using textured surroundings with the changing lamp interior masked. Require correlation at least 0.85, sufficient pixels/texture, bounded displacement, and a bound track ID across a gap. A full template remains the ordinary-cadence fallback; textureless context cannot bridge a gap.
- After one missed observation, independently detected housing reacquisition requires close center/size/overlap and rejects ambiguous prior identities. Color confirmation restarts.
- An explicit trial consumer can reuse the producer's consistent red history, as the old consumer already did for green. Require fresh current raw/confirmed agreement, current visibility, at least three observations, and consistent history duration/count. Red needs 300 ms; green 400 ms. This avoids duplicating an already established red streak when intermediate results reached the consumer late.

Both tracker and red-history changes are opt-in. `revalidated_enabled` is the **name of an experimental file flag, not a statement that validation passed**. Missing it preserves the original tracker and consumer behavior. Assist still separately requires `assist_enabled`; comparison mode publishes neither observer to control. The new flag is not written on the vehicle. Consumer mode is selected at process startup; do not toggle the trial on a driving vehicle.

No steering, braking limits, ONNX trajectories, existing distance/turn/lead gates, driver override, or observation freshness limits are relaxed. Green removes the helper's restriction; it does not command acceleration. Forward-image position does not prove which signal applies to the ego lane.

## Offline replay

Private sources are the existing `2026-10-10-signal-night`, `2026-10-11-signal-followup`, and `2026-10-11-signal-1095` archives: 37 full video segments, 15,422 sampled images, and 43,090 planner updates. The 1093/1094 sampling and arrival times are recorded; 1095 retains the second comparison engine's actual arrivals. Older 1091 timing explicitly assumes alternating 150/200 ms sampling and 150 ms arrival age. It is not measured current-candidate latency.

The planner replay uses recorded inputs with original navigation/Params partly substituted by replay defaults. It does not recompute vehicle motion, stop position, other traffic response, or camera motion after an altered decision. These are previously reviewed/tuned recordings, not a blind test set.

| Problem window | Helper OFF: go-state updates | Candidate, nominal timing | Candidate, +20 ms arrival delay |
|---|---:|---:|---:|
| 1091 segment 12 | 49 | 0 | 49 |
| 1091 segment 16 | 59 | 0 | 0 |
| 1091 segment 20 | 4 | 0 | 4 |
| 1095 segment 65 | 51 | 0 | 0 |
| 1095 segment 66 | 28 | 0 | 0 |

These are planner update counts within five incidents, not independent driving trials. The segment-20 nominal result uses a moving red stop request rather than a stopped hold. In the failed older delayed windows, no current observation passes the unchanged 200 ms freshness check. Permitting stale imagery was not used to make the test pass.

There are no releases in 8,744 manually labeled, active red planner updates. The candidate produces ten release events. Nine have supporting visually green images in the reviewed sequences. **The tenth, 1094 segment 10 frame 853, is not accepted as a verified green release.** Track 3071 reports consecutive green on frames 840/844/847/851, but close inspection shows a dark housing and does not establish an illuminated green lamp at the selected location. Border/background color or incorrect object/box alignment needs resolution. Do not report this as ten successful releases or as proof that every red signal was safe.

For that disputed image, complete sequential default decoding, explicit passthrough decoding, and direct selection at frame 847 produce identical RGB SHA256 `9b351b217ff67c13a1d847cdcf80c8bd3cfb270ace0fada931d4a65dc88958c2`; each complete decode contains 1,200 frames. A simple decode-index mismatch did not explain it. For 1094 segment 8, direct review refines the transition to last red frame 690 / first green 691; the release at frame 703 is green.

An additional six earlier day/dusk clips contain 1,447 images. Neither observer reports green on labeled red images. However, dusk segment 81 has 113 confirmed-green images with the old observer versus 89 with the candidate, and candidate unknowns increase 33 to 55. Other reviewed dusk green counts match (112/72/64). The candidate therefore is not an across-the-board perception improvement.

## Isolated parked device test

The same car ran eight stored-image sequences at its actual camera frame timing, with current longitudinalPlan publication times and measured result arrival times. A fresh Park/zero-speed/inactive guard runs throughout. Original HEVC byte slices were hash-verified; no re-encoded or mismatched clips were accepted. Six clips were cut from the original recordings still on the car and two uploaded clips were verified against the PC manifest.

The test is a separate low-priority process on cores 0–3, with the existing 50% CPU duty policy. It copies a live camera buffer for conversion overhead, then analyzes the stored RGB frame selected by live frame progression. Results stay under `/data/signal-revalidation-20261011`; they never enter the production observation transport. The pure helper is exercised afterward with a virtual stopped/engaged context. The actual car remains inactive in Park. This is not an onroad test or complete actuator/planner simulation.

| Stored sequence | Nominal result | With 20 ms additional delay |
|---|---|---|
| 1091-12 | red hold, green release | same |
| 1091-14 | red hold, green release | same |
| 1093-6 | red hold, green release | **never acquires red hold** |
| 1093-8 | red hold, green release | same |
| 1094-4 | red hold, green release | same |
| 1094-8 | red hold, green release | same |
| 1095-65 | red hold through clip end | same |
| 1095-66 | red hold, green release | same |

Across 464 processed observations, per-clip median arrival age ranges 106–157 ms and p95 ranges 141–186 ms. In 1093-6, 15/32 observations are already older than 200 ms at the first following planner publication. This short clip starts with less than a second of reviewed red and no prior history; failure here is an acquisition vulnerability, not a demonstrated road departure. It still fails the proposed latency margin. Publication time approximates the consumer cycle; it does not measure its exact read instruction. The disputed 1094-10 release was **not** part of this hardware test.

## Distance and method limits

At 1344×760, reviewed signal candidate boxes can be only 16×8 pixels. A lit lamp is smaller still. This challenges color rules and neural detectors alike; image scale, signal association, and temporal evidence matter in addition to detector choice.

In reviewed 1093-8, the candidate first adds the moving red request at frame 591 (selected observation 589, a 20×10 box). Integrating recorded measured speed until the stop at frame 863 gives **106.3 m of travel**. In 1095-65, the corresponding request at frame 939 to stop frame 1064 gives 18.5 m. These are distances traveled from a helper intervention to the recorded stop, **not surveyed ranges to the lamp, maximum detection ranges, or guaranteed braking performance**. The helper's distance gate also affects when a request begins. Do not substitute ONNX x[-1] for ground-truth signal distance.

A trained signal detector is a reasonable next comparison for appearance robustness, but has not been trained or benchmarked by this work. It would require boxes and signal-state/direction labels, suitable image resolution, distinct evaluation routes/lighting, and the same latency/association tests. Current rule tuning should not be called ONNX retraining. Adding recordings alone does not repair a known timing or box-selection defect.

## Reproduction and disposition

- 360 focused software tests pass, including tracker identity/color-gap cases, stale history rejection, independent trial/assist opt-ins, production worker publication rules, and existing planner/transport/control regressions. Desktop Windows Params are substituted as in earlier signal tests.
- The production opt-in tracker matches the frozen v4 prototype on all 15,422 observations. The default observer matches the frozen pre-change observer on 771 independently selected images. This is implementation parity, not perceptual correctness.
- All 43,090 nominal planner decisions are byte-for-byte equal between the frozen prototype and the final opt-in production implementation.
- Private scripts, image review sheets, candidate failures, raw device results, timing assumptions, and manifests are retained in `.analysis/archive/2026-10-11-signal-revalidation/`. Raw recordings remain in their original private archives. No captures, identity/settings snapshots, or driving traces are committed.

The code is retained behind opt-ins for reproducible experiments. **Do not enable this candidate for vehicle control based on this report.** The next correction must resolve both missed fresh red acquisition and the disputed green box, then rerun the existing regression corpus and an independent evaluation set. The unchanged original driving model and assist-OFF vehicle state must not be described as a fixed red-departure system.
