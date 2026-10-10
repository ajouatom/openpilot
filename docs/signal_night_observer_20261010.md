# Night signal observer investigation - 2026-10-10

The new night drive exposed a coverage failure in the installed housing-based
observer: all 11,061 logged results in twelve collected segments were unknown.
The source files were copied with 60/60 SHA256 matches. All 14,401 recorded
driving-model frames were valid without frame-ID gaps. Unknown observer output
does not establish correct recognition or prevent an original-model departure.

Video and exact model/plan alignment show red-signal acceleration in three
reviewed windows. In one stopped encounter, near-object lead mode was entered,
then left while the derived traffic state was still red and x[-1] remained
about 1.4 m. A subsequent model speed surge also produced a derived Go state.
Another encounter accelerated after the x/v-derived Go state despite a visible
red signal; a third had a brief red-approach acceleration pulse. The exact
`check_model_stopping` method reproduces all 318 traffic decisions in these
windows, using the logged prior xState and latest received carState. This is
recorded-input reconstruction, not full controller or vehicle-response replay.
The nearby crossing person and radar track are correlated, not proof of the
network's internal causal mechanism. Private timing, video and incident traces
remain local. Changing v[0]+2 alone would not cover all observed paths; the
existing separate terminal-speed >5 condition also matters.

## Image-only change

`tools/signal_analysis/signal_tracker.py` retains the original daytime housing
proposals and adds night lamp proposals when the upper image is dark:

- Find compact saturated bright cores, then inspect the surrounding color halo.
  An overexposed white center must not erase the red evidence around it.
- Require a roughly circular red core with red support in every quadrant.
  This rejects a reviewed one-sided brake-light reflection that an initial
  candidate incorrectly tracked instead of the real signal.
- Infer a horizontal four-lamp box and check the existing end-lamp evidence.
  The box is an inferred proposal, not measured housing or lane association.
- Keep startup green unarmed, same-track red history, bounded confirmation,
  contradictory-color handling and disagreement abstention.
- Permit 125 ms between supporting observations, accounting for two camera
  periods plus timing jitter. Longer gaps restart confirmation; 250 ms expiry
  and 150 ms ambiguous-support retention remain unchanged.

This is image-processing engineering, not new ONNX training. The driving ONNX,
x/v/a outputs, traffic/departure thresholds, planning and actuation are unchanged.
The existing opt-in worker continues diagnostic logging with control_permission
false. No vehicle state, manual labels or future images enter the observer.

## Recorded-image validation

Two encounters were used to develop the night proposals. Remaining recordings
and all transition boundaries were visually reviewed; these are reused evaluation
data, not an unseen validation set. Ambiguous red/green exposure boundary frames
are excluded. Counts are temporally correlated frames, not independent trials.
Scene-color scoring does not establish which signal governs the ego lane.

| Reviewed observations | Full 20 Hz video | Original worker's recorded camera cadence |
| --- | ---: | ---: |
| Actual red | 7,427 | 5,704 |
| Correct red | 7,408 | 5,677 |
| Red reported as green | 0 | 0 |
| Actual green | 688 | 540 |
| Correct green | 520 | 339 |
| Green reported as red | 8 | 6 |
| Unknown, both colors | 179 | 222 |

Four transitions first confirm scene green in approximately 0.15-0.41 seconds
in full video (0.19-0.41 seconds at recorded cadence). A fifth has two frontal
green tracks within about 0.15 seconds, but a side-facing red track makes the
scene result unknown. First scene green is then about 5.3 seconds later. This
remaining lane/heading ambiguity must not be concealed by majority voting or
discarding red tracks. Green continuity is also worse at skipped-frame cadence.

The prior six daytime/dusk clips retain all 1,447 previous scene decisions:
0/1,043 red-as-green, 361/404 correct green and 58 unknown. Forty-two desktop
observer/conversion/temporal tests pass, including saturated lamps, one-sided
reflections, startup green and irregular camera cadence.

## Parked device performance

An initial full-image HSV candidate was too slow: median work 105.6 ms, only
127 observations in the finite run and four stale results. It was not installed.
Restricting HSV conversion to each small halo, limiting connected components to
the eligible region with margins, and replacing a full-image partition with the
equivalent uint8 histogram percentile reduced median work to 42.7 ms (p95 62.0).
Median CPU time was 34.0 ms; 302/302 observations were fresh, result-age p95 120.1
ms. All 640 model frames remained valid at 20 Hz without gaps; mean model execution
was 24.88 ms before and 25.41 ms during the finite observation window.

The optimized code exactly reproduces all 25,459 full-video and sampled records
(including tracks/evidence, excluding elapsed work time) of the unoptimized
candidate on the twelve segments. Repeating the measured parked frame-gap
sequence over six selected encounters gives 0/2,675 red-as-green, 2,640 correct
red, 311/419 correct green, two green-as-red and 141 unknown. This single-phase
cadence stress replay still cannot produce scene green at the conflicting-side-
signal encounter; it is not an estimate of driving cadence or independent data.

The worker retains CPUs0..3, nice19, one OpenCV thread, no OpenCL, max20 Hz and
the existing half-of-one-CPU duty target. Actual observation rate is about12 Hz,
not20 Hz. The original recorded cadence in the table is not a prediction of new
driving load or a raw-NV12 recognition validation. Parked measurements cannot
establish closed-loop stopping or false-departure prevention.

Private evidence and annotated videos are indexed under
`.analysis/archive/2026-10-10-signal-night/`. No footage, vehicle settings, labels
or private model artifacts are committed. Next recognition work needs causal
ego-lane/signal association, longer green continuity and separately scored
distant-red approach/stop-line evidence before any control integration.

## Installation verified

Installed `b57b81835d` on the existing experimental branch with fresh valid Park,
standstill and inactive-control checks, tracked-dirty protection and the shared
repository lock. Only the observer was restarted; no vehicle reboot was needed.
Both local observer flags remain enabled. Canonical algorithm SHA256 is
`b0dd5cd6fa5d3eca49d065f82a22e4f60654c2e58cd0793c36f53a9f0d07b92f`.

The final automatic-worker observation confirmed 600/600 valid model frames at
19.999 Hz without gaps or reported drop. There were 395/395 fresh observer events
at11.977 Hz, median work44.31 ms and result-age p95122.29 ms. All three worker
threads retained CPUs0..3/nice19/SCHED_OTHER;52 manager observations retained one
running PID. A closed saved rlog contained98 new-worker events from this interval,
all matching recorded road frame IDs with the video file present. The original
driving ONNX hash remains unchanged. These are execution/recording checks, not
proof of onroad signal accuracy or false-departure prevention.

## Follow-up: direct stop-hold promotion was not validated

The user asked to use the improved version rather than keep driving with the
old one. A fresh vehicle read confirmed the improved observer commit is already
installed, ignition on, valid Park/zero speed and inactive control. There is no
new driving ONNX to switch to; observation and longitudinal control are separate.

An additional private decision-availability comparison used full video, the
original logged cadence and the measured parked frame-gap pattern. Each case
assumed fixed 50/125/200 ms processing delay, selected only results available at
the original plan timestamp, and expired evidence at camera age200 ms. This is
not a planner/MPC, changed-state-machine or closed-loop simulation. An illustrative
stopped-only scope used |logged vEgo|<=0.3 m/s without changing vehicle thresholds.

At50 ms delay, both reviewed near-stopped false-departure instants have available
red evidence in all three cadences. The moving red-approach case is outside a
stopped-only intervention. At125 ms delay with the parked gap pattern, one of
the two stopped instants instead has expired evidence. At200 ms all three
incident instants lack usable evidence by their plan timestamps. Merely gating
on currently available red therefore does not establish retained stopping.

Conversely, requiring scene green to release a latched stop has an observed
failure: the side-red/frontal-green encounter first reports scene green about
5.15-5.43 seconds after the first reviewed green image at50/125 ms assumed delay
in full/original-cadence replay. Repeating the parked gaps produces no scene
green anywhere in that reviewed approximately6.4-second green window. These
are observer availability times, not measured added vehicle departure delays;
manual release, vehicle motion and the changed future camera view are not modeled.

Direct promotion to vehicle control is therefore not supported by this comparison.
The improved observer remains enabled; no control hook, hold latch, new model or
departure-policy change was installed. Resolve relevant-signal selection and
define/test stale-evidence and release behavior before claiming a usable hold
controller. Private reproduction is tools/compare_hold_decisions.py and
hold_decision_comparison.json in the night investigation archive.
