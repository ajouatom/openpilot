# Central signal selection experiment — 2026-10-17

The user requested improving central/road-direction signal selection and checking
the accumulated logs. The revised offline selector removes some peripheral
green-first proposals while preserving every red and every red-history track.
It is **not connected to the vehicle observer, display, start guard or moving
preparation**. No ONNX, vehicle files, parameters or running processes changed.

## What was implemented

`tools/signal_analysis/signal_central_selection.py` compares three policies:

1. An explicitly unsafe offline comparator discards peripheral candidates of
   any colour after a stable central signal is found.
2. A conservative comparator excludes only green-first side candidates, retaining
   red and red-history tracks regardless of position.
3. The revised conservative policy also preserves separate, similarly sized
   current green candidates on the same image row. Such a pair only prevents
   exclusion; it does not confirm green, establish lane applicability or grant go.

Reliable lane/edge curves and calibration define an assumed overhead search
volume (15–160 m sampled every 2.5 m, 4–8 m height, 3.5 m lane-side allowance).
These are geometric assumptions, **not measured signal range**. Invalid or
unsupported height/range is unknown. Wide road centres cannot stand in for an
ego lane. On the latest 7,957 observations, a road-only trial had usable geometry
on 1,961 observations and a stable central anchor on only 49; it excluded just
one green proposal. At the reviewed stopped intersection in 10e5/4, geometry
was unavailable throughout. Relaxing the existing road confidence contract was
not justified by this result.

The explicit `image_fallback=True` experiment therefore uses a stable central
image group when trustworthy road geometry is unavailable. It is a screen
position hypothesis, not curved-road lane association. Anchors need 350 ms of
confirmation; gaps over 250 ms reset them. Current, matching unknown boxes may
retain an already established anchor for at most 500 ms from its last confirmed
colour. Missing boxes do not extend it. Raw detections are never mutated or
recoloured. Red history is never manufactured. Duplicate observations cannot
extend confirmation. The production tracker remains unchanged.

## Recorded-data comparison

Deduplicated retained full-log decodes with signal observer events: **122
segments, 56,612 observations**, covering the October 12–17 signal collections,
including the Santa Fe departure report and October 16/17 drives. Qlogs without
observer events cannot supply this comparison and are not counted. These are
all eligible decodes in the inspected signal/latest-drive/Santa Fe collections,
not a claim that every retained vehicle file has been replayed.

| Quantity | Count |
| --- | ---: |
| Current raw red observations (boxes, not encounters) | 27,307 |
| Current raw green observations (boxes, not encounters) | 12,294 |
| Observations with usable road geometry | 11,101 |
| Observations with a stable central anchor | 11,300 |
| Any-colour comparator: excluded red boxes | 1,035 |
| Initial conservative comparator: excluded green boxes | 577 |
| Revised conservative selector: excluded green boxes | 503 |
| Revised selector: excluded red/red-history boxes | 0 / 0 |

The all-colour comparator is not suitable for activation: geometry alone cannot
establish that those excluded reds are irrelevant. The 503 excluded green boxes
are **not 503 verified false positives**, unique objects, or an accuracy score.

## Image review and correction

Reviewed 48 exact encoded frames selected from excluded track groups, using
`roadEncodeIdx.frameId -> segmentId`. The deprecated `segmentIdEncode` is not a
video frame index. Yellow boxes in the private sheets mark excluded candidates;
cyan boxes mark central anchors. Compressed video is suitable for scene/object
review but is not identical to the live uncompressed colour evidence.

The initial conservative filter also excluded real overhead green signals. The
same-row preservation rule was added after inspecting these failures:

| Reviewed category | Reviewed examples | Excluded after correction |
| --- | ---: | ---: |
| Clear background: signs/text/foliage/wires/vehicle reflections | 34 | 33 |
| Visible overhead green signals | 7 | 0 |
| Roadside lights with unverified identity/applicability | 5 | 5 |
| Actual signal housing with uncertain colour | 1 | 1 |
| Blurred light with uncertain identity | 1 | 1 |

These are **development examples used to revise the rule**, not an independent
test set or a measured precision/recall result. The seven unresolved exclusions
remain unresolved. A single relevant green with no red history can still be
excluded if another central group is present. Pair protection also cannot prove
that a pair is made of traffic lights rather than similarly aligned background
objects. This limits the policy's suitability for control or a trusted display.

Of the six previously reviewed October 17 false-green examples, five are
excluded. The sixth (10e6/23, frame 579) remains because no stable central signal
is available. All three checked green frames at the reviewed 10e4/13 transition
(464, 506, 547) retain both genuine signal candidates. The near red housing in
10e5/4 still has its previously measured 18.802 s missing-red observation span;
selection cannot repair its colour evidence.

## Stopped-guard replay and limits

The public replay tool feeds the current `SignalStartGuard` the original and
revised observations at identical recorded arrival times. It preserves the
first-20 transport cap before exclusions, recorded freshness, driver/context
gates, and a separate +20 ms arrival-delay comparison. Both guards and selectors
start cold per segment; missing pre-segment state is the same on both sides.

| Additional arrival delay | Compared updates | Decision differences | Releases, before / after |
| --- | ---: | ---: | ---: |
| 0 ms | 115,920 | 0 | 18 / 18 |
| 20 ms | 115,898 | 0 | 18 / 18 |

This establishes unchanged decisions for these **paired recorded inputs**. It
does not establish faster departure, fewer red departures, a full planner/MPC
replay or closed-loop safety. Actual user mode overrides and unpublished
transport latency are not fully reconstructable here. Moving preparation was
not promoted: removing green candidates can remove its cancellation evidence,
so unchanged stopped-guard output alone is insufficient to enable the filter in
that path. Original lead-follow and driver hold policies remain untouched.

750 desktop signal/modeld/analysis/brake-lamp tests pass, using the existing
native-Params lookup double. No native performance, GPU, 20 Hz, DM timing or road
improvement claim follows from these tests. Initial dual-selector desktop
timing was about 0.11 ms median / 0.29 ms p95; it is not a vehicle benchmark or a
timing measurement of a deployed final pipeline.

## Reproduction and publication

Private evidence root: `.analysis/scratch/2026-10-17-signal-central/`.
It contains the deduplicated path manifest, original/revised selections, exact
frame review sheets and labels, and paired stopped-guard replay results. Raw
recordings and sheets are not published.

```text
python -m tools.signal_analysis.replay_signal_central <private-manifest.json> <private-output.json>
```

Publish the offline algorithm, replay tool, tests and findings to both maintained
branches using single-parent commits. The useful result is a narrower candidate
selection rule and reproduced counterexamples to indiscriminate central-only
filtering. The remaining target is whole-housing/lamp identity and temporal
association, with independently reviewed encounters and moving-preparation
verification before any control activation.
