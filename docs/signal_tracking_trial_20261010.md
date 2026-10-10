# Automatic signal tracking trial — 2026-10-10

The user requested a complete experiment after object-model retraining and a
manually located lamp-brightness prototype failed to establish usable automatic
recognition. Added an offline RGB observer and reusable video CLI in
`tools/signal_analysis/`. It automatically proposes horizontal signal housings,
tracks them causally, and confirms bounded red/green/unknown observations.
Six annotated videos and an interactive local report now show the actual selected
objects and all decisions. This iteration changes image-processing logic; it does
not train new ONNX weights or modify original model x/v/a, departure thresholds,
vehicle files or configuration.

## Method and experimental boundary

Dark rectangular housing proposals use geometric and surrounding-contrast checks.
Expected end-lamp locations, HSV color, component shape and brightness provide
lamp evidence. Previous-image template matching can preserve a confirmed-red
track through weaker housing detections. No manual boxes, future images, labels,
vehicle state or original model predictions enter the observer. Labels are used
after inference for scoring and video comparison only.

Red confirmation needs at least three observations over 0.10 seconds. Green needs
at least three over 0.15 seconds and a previously confirmed red on the same track.
Contradictory red removes green immediately. Unknown evidence can preserve the
previous confirmed state for at most 0.15 seconds since actual supporting evidence;
unknown does not renew that evidence. Missing observations become unknown after
0.10 seconds; tracks expire after 0.25 seconds. Nonmonotonic timestamps or an input
gap over 0.25 seconds reset tracking and arming. Conflicting confirmed visible
tracks produce unknown.

The school clip was used for development. The algorithm was then frozen before
the six-clip replay; **all clips had previously been visually reviewed**. This is
development/regression evidence, not unseen intersection or night validation.
The frozen source SHA256 is
`19493c5d8aa251ecd0fdb637b526fcd10bd819a7e45e846ff1b3048ac36242a9`.

## Recorded replay results

| Case | Frames | Red frames | Red as green | Green correctly read / green frames | Unknown | First green delay |
|---|---:|---:|---:|---:|---:|---:|
| School | 210 | 65 | 0 | 113 / 145 | 33 | 0.201 s |
| Outbound intersection | 180 | 64 | 0 | 112 / 116 | 9 | 0.205 s |
| Large intersection | 140 | 64 | 0 | 72 / 76 | 5 | 0.200 s |
| Last intersection | 100 | 33 | 0 | 64 / 67 | 6 | 0.155 s |
| Sign / false-Go interval | 627 | 627 | 0 | — | 2 | — |
| Daylight red reacceleration | 190 | 190 | 0 | — | 3 | — |

Across 1,447 frames: red-as-green 0/1,043, correct red 1,026/1,043,
correct green 361/404, and unknown 58/1,447. Two first-transition frames
retain the previous red. Samples within one encounter are correlated; the large
frame count does not establish a population error rate. First green delay is
measured against the reviewed transition frame using camera EOF timestamps.
It is not vehicle departure delay or evidence of a closed-loop stopping fix.
PC median compute spans 5.54–7.58 ms; this is not device timing validation.

Eight state-machine regression tests pass: startup green, continuous confirmation,
contradiction, nonrenewing unknown retention, track expiry, input gap reset,
location identity, and conflicting observations. The generic CLI also processed
20/20 requested raw-video frames and produced JSONL plus annotated MP4.

## Distant red and stopping

The user prioritizes stopping for red and preventing false departure while
stopped. Candidate selection must report red-as-green errors separately from
overall accuracy and green delay. Unknown or lost observations are not confirmed
green. This is an acceptance objective, not a claim that this offline observer
already prevents vehicle departures. Preserve the user's direction to improve
recognition rather than change departure thresholds.

The user asked whether recognizing distant red early enough to decelerate and
stop also needs learning. The present experiment establishes neither reliable
distant approach recognition nor stopping-distance behavior. Housing dimensions
in pixels are not a calibrated distance measurement.

A learning extension needs continuous approach sequences with small-to-large
signal boxes, visible state, track identity, visibility/uncertainty, ego-lane
relevance and stop-line geometry. Keep each encounter together when separating
training and evaluation. Day/night, backlight, partial occlusion, adjacent-lane
signals, arrows and signs need separate coverage. A visually unresolved lamp
should be labeled unknown, not inferred from a later frame as if visible now.

The image recognizer can remain camera-only. Planning a stop additionally needs
current speed, distance to the relevant stopping point and feasible deceleration;
those inputs need not enter the signal-color classifier. Existing trajectory and
control planning should not be replaced by a raw red-light flag. Approach tests
should measure first stable correct recognition before the stop line, intermittent
misses, lane-association errors and distance error, then evaluate stopping behavior
separately. These additions have not been implemented or validated in this trial.

## Scope and artifacts

This observer assumes the tested horizontal lamp arrangement and requires seeing
red before green. It does not distinguish arrow permissions, select the ego-lane
signal, estimate stop-line distance or issue movement permission. The original
driving ONNX remains SHA256
`f73a9e535523d5e9acb9e642c64e33d631825dc8ba74123757d107cedd047bb5`.
No vehicle installation was performed.

Private evidence stays under `.analysis/archive/2026-10-10-signal-tracking/`:
`report.html`, six annotated replay videos, per-frame records, frozen protocol,
aggregate result, source identities, CLI check and archive index. Only reusable
observer/CLI/tests and aggregate methodology belong in Git.
