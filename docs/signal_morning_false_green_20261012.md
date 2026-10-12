# Morning signal false-green investigation, 2026-10-12

The new recordings confirm false green candidates on sky between branches and
two examples of green assigned at a still-red signal. This is a failure of the
current CPU color/housing observer. It is not evidence that the driving ONNX was
retrained, nor simply an old green held by the display. Analysis only: no vehicle
code, settings, model, DM policy or departure rules were changed during this task.

## Collected evidence

The NAS route directory was checked first; these newer routes were still on the
vehicle. Vehicle HEAD was `12a680ee3c`, branch `carrot-signal-shadow`. Collected all
52 previously uncollected completed segments identified at connection time:

- 109c segments 17–32: 16 stationary segments from the preceding investigation.
- 109d segments 0–19: October 11, about 19:09–19:29, including startup/parking.
- 109e segments 0–4: October 11, about 21:15–21:20.
- 109f segments 0–10: October 12, about 06:45–06:56.

PC copies include rlog, qlog and qcamera for all 52 segments, and full-resolution
fcamera for all 11 morning segments plus 109d-6 and 109e-3. All 169 source files,
1,207,399,578 bytes, match device SHA256 and length. Completed source recordings
also have private preservation hard links on the vehicle. No recording is in Git.

The 36 segments from the three new sessions were decoded with the full cereal
schema: 9,726 comparison observations, including 2,727 in the morning. Matching
uses roadEncodeIdx frameId -> segmentId, not approximate wall-clock video seek.
The morning qlog clock adjusts around segment 2; all actual alignment uses
monotonic capture times. Native source metadata reports functional commit
`0bfcb01d13`; the later installed HEAD adds only its verification documentation.

All analyzed morning DM-state samples have cameraUnavailable=false. Startup DM
unavailability exists in 109d-0 before the prior final verification; do not label
every segment of this collection healthy from its first frame. All signal
observations in the three sessions use the CPU comparison worker; no neural
signal observations are recorded.

## What was misdetected

Morning has 6,792 fresh green candidate samples, with green present in 2,016 of
2,727 observations. These are repeated candidate samples, not unique signals or
a false-positive percentage. Sources:

| Green source | Samples |
|---|---:|
| day_color_core | 6,011 |
| dark housing contour (`detected`) | 419 |
| night_lamp_core | 349 |
| causal_template | 13 |

The broad green predicate permits OpenCV HSV hue 38–105, saturation >=65 and
value >=95. The daytime seed uses value >=140 plus a local bright-spot test.
In these morning images, sky through dark branches meets those conditions.
For example, 109f-1 frame1401 candidate366 has compressed-image green-region hue
92–97 and mean RGB about (91,131,138): blue/cyan sky, not a lamp. The same frame
has three other false candidates on branches/building edges. Frame2708 in 109f-2
has 19 fresh green candidates on trees/poles/sky, and 109f-4 frame4904 has green
background candidates around an actual red signal.

`day_proposals` derives a housing from the chosen color core: width=4*diameter,
height=2*diameter, putting red at 1/8 or green at 7/8 of that inferred box. A dark
branch next to sky can supply its required adjoining dark bar. This is not an
object detector establishing a complete physical traffic-light housing.

The two directly reviewed red/green failures:

1. **109f-6, frame7970 (segment 38.4 s), ID4944:** the small actual red signal is
   assigned green. Its preceding same-ID samples are green at7961, red at7966,
   then green at7970. The box is only16x6 pixels. Live red_score=0 and
   green_score=0.409. Re-evaluation of the HEVC image gives unknown, green0.140,
   rather than the exact live green score; lossy encoded frames are not raw camera
   buffers. The right-side accepted pixels are hue103–104. Do not claim exact
   pixel-identical reproduction of this event from HEVC.
2. **109f-10, frame12022 (segment1.0 s), ID6362:** the left signal is still red,
   but the candidate is green. At12018 its red box is [400,166,440,186]; at12022
   the green box is [407,163,443,181]. The actual red lamp has moved to the left
   of the new crop. The new box covers the dark housing end and adjacent sky.
   Live green_score=0.615 and HEVC re-evaluation green_score=0.532. The selected
   right-side pixels have hue97–105, mean RGB about(105,137,153). Spatial overlap
   let a color-dependent inferred box inherit the old identity without retaining
   the physical lamp/housing alignment. A second actual red signal is still
   detected red in this frame.

Thus both object selection and lamp/crop placement need correction. Merely
extending display memory cannot fix either. The two-second UI hold can make
wrong boxes remain visible longer, but these counts and examples are fresh
observations before that hold, proving it is not solely display persistence.

The only logged morning start-guard green release is 109f-8 at18.106 s. The
original images show a genuine green by17.89 s, and the same tracked signal
changes red->green around17.15 s. No start-guard green release is logged in the
two reviewed red/green error windows. This does not establish the absence of all
unsafe departures; no complete drive-by-drive closed-loop safety claim is made.

## Size, height and road-edge feasibility

The requested idea is useful as supporting evidence, not a traffic-light or
ego-lane proof. Use calibrated camera rays and a *range* of plausible sizes and
heights, rather than one distance from a guessed box. Pinhole projection relates
pixel size to focal length, physical size and depth; unknown physical size makes
depth ambiguous. [OpenCV calibration and projection documentation](https://docs.opencv.org/4.5.5/d9/d0c/group__calib3d.html)
describes that camera model. This repository supplies OS04C10 f=1141.5 pixels
at1344x760, live rpyCalib, and model roadEdges/roadEdgeStds.

An offline probe used past-only model/calibration samples, converted candidate
lamp points into calibration-frame rays, and intersected candidate depth ranges
with model road-edge width and ground height. Exploratory assumptions were
lamp diameter0.10–0.50 m for inferred core boxes, housing width0.7–2.5 m otherwise,
lamp height3–9 m, and lateral margin2 m +2*edge standard deviation. These are
broad unverified assumptions, not measured signal dimensions or installation
standards. `causal_template` boxes also have uncertain originating geometry.
Rejecting was permitted only with valid model age<=250 ms, calibrated pose
age<=1 s and both road-edge standard deviations<=1.5; otherwise keep as unknown.

Manual review labels cover 73 observed candidates in 11 deliberately selected
morning frames:62 background false candidates,2 wrong crops on red signals,
6 correctly red candidates and3 correctly green candidates. They are a small,
biased diagnostic set, with repeated physical signals, not an accuracy benchmark.

| Probe | Background removed /62 | Wrong-green-on-red removed /2 | Correct red/green retained |
|---|---:|---:|---:|
| Size + height + road edges, with quality gate | 8 | 0 | 6/6 red,3/3 green |
| Exploratory hue<=90 right-lamp pixel gate | 61 | 2 | red unchanged,3/3 green |

The color probe only asks whether at least two right-lamp-region pixels retain
the narrower hue criterion; it is not a new connected-component detector or
complete tracking replay. Its impressive small-sample rejection must not be
mistaken for validated day/night accuracy. It leaves one branch candidate and
has not been validated against cyan-looking true signals at night, exposure
changes, rain, backlighting, arrows or different intersections.

The geometry quality gate is available for only38/73 labeled samples. Both
wrong-green-on-red cases are geometrically plausible and have uncertain road
edges. Intersection widening, turns, signal overhang, pole-mounted signals,
slopes and guessed bounding boxes limit a hard road-edge mask. Road edges should
adjust search priority or confidence with an uncertain fallback, not turn a color
candidate into a confirmed green. Autoware's [map-based signal ROI detector](https://autowarefoundation.github.io/autoware_universe/pr-10075/perception/autoware_traffic_light_map_based_detector/)
also uses position to restrict image search, but it has mapped signal positions;
our model road edges do not provide that information.

## Next implementation priority

1. Reject sky/background-like green evidence while preserving real green across
   night and morning data; a hue-only patch is not ready for deployment.
2. Establish the complete housing and lamp positions. Do not treat a color-core
   inferred box as measured object geometry or let red->green crop shifts change
   the identity of the physical lamp. Ambiguous crops should become unknown.
3. Use calibrated height/size/road corridor as supporting candidate scores and
   search regions. Validate roadside, turning and far-signal cases separately.
4. Replay the observer, UI and stopped-car veto on these exact red failures,
   actual red->green transitions and retained earlier night cases. Any neural
   verifier must be profiled jointly with DM; the prior QCOM overload is still
   disallowed. Full driving ONNX or neural helper retraining was not performed.

The current collection is enough to construct a concrete correction experiment;
collecting more unlabelled driving alone does not resolve these confirmed bugs.
No production change is deployed by this investigation.

Private evidence: `.analysis/archive/2026-10-12-day-signal/`, including all verified
source files, decoded observations, original/annotated review images, manual
labels, geometry/color probes and reproducible scripts. The archive index lists
the reviewed examples and distinguishes source collection from selective labels.
