# C3 pre-upload camera warp

## Reason and evidence

On 2026-09-20, the user approved moving generic eGPU camera preparation to the
internal QCOM GPU on tici/tizi, preserving the existing AMD path on mici/C4.

Full-cereal analysis of the exact NAS uploads found:

| Recording | Device | Input upload | Model execution | Mean reported frame drop |
| --- | --- | --- | --- | --- |
| Ioniq5 `00000f82--7a9d4876cb--6` | mici | 13.94 ms | 39.73 ms | 0% |
| EV9 `000002c1--2373baf2e5--2` | tizi | 25.05 ms | 51.28 ms | 16.53% |
| EV9 `000002c6--03b0cc1ec8--3` | tizi | 24.75 ms | 50.69 ms | 12.66% |

The final EV9 recording confirms xiaoge_data stopped in every manager sample.
Its 155 invalid odometry messages correspond exactly to skipped input frames.
Both current vehicles execute artifact e758b96d. Worker CPU scheduling delay is
only about 0.21 ms/frame, not the measured 11 ms cross-device upload difference.
These are host-call timings, not USB wire time or pure GPU kernel durations.

Older EV9 TG recordings (September 1-2, d58401e9/412b6644) ran at about
35.5-35.7 ms and 20 Hz without invalid odometry. That code warped on QCOM before
transferring prepared images to AMD. No EV9 Cinque v2 recording was found in the
NAS route inventory. V2 had both split/local-compile and fused/precompiled
paths; do not assume either was used without device evidence.

## Implementation

`local_gpu_warp.py` selects only device types tici/tizi. The same immutable
artifact runtime supplies `compile_warp`, so pixel sampling, clamping, YUV420
packing and camera transform interpretation are not reimplemented. The model
and its GPU-resident recurrent buffers remain unchanged.

The shared CPU buffer still holds raw NV12 inputs. Each frame is explicitly
copied to local QCOM memory for cache coherence, warped, and read into a compact
host buffer. Only that buffer is uploaded to AMD: 393,216 image bytes plus 512
control/alignment bytes for the current model (393,728 total versus 7,471,616
on EV9). This is a local memory copy, not a claim of end-to-end zero-copy.

At initialization, random NV12 data and identity, projective and border-clamped
transforms are processed on QCOM and the artifact's AMD warp. Any shape, dtype,
unexplained pixel mismatch or initialization failure retains the original AMD path and
logs the failure. No model inference occurs during this check, and shared
inputs are cleared afterward. This probe is an acceptance guard, not proof
of equivalence for every possible image/transform. Differences are checked
against the source-pixel rule below, never accepted by an image-error percentage.

The QCOM warp cache lives under the pinned artifact runtime, distinct from AMD
cache files. Changing the local compilation contract requires a cache-version
bump. C4 and non-generic artifacts retain their prior execution path.

## Validation and deployment checks

The first vehicle trial, EV9 `000002c9--15d447d91b--0` on 9a349b60, rejected
the QCOM warp with `QCOM pre-upload warp differs from artifact AMD warp`.
The worker continued on AMD with 7,471,616 USB input bytes and mean upload
24.801 ms. Thus this trial does not measure the proposed optimized path.
Locationd/communication alerts recur after initialization. Segment-zero timing
includes a first model run of about 1.73 s; do not compare its overall mean
directly to steady-state earlier segments. Model execution median is 50.417 ms.

Original diagnostics did not retain the failing probe or mismatch size. The
follow-up reports all failing probes, shapes/dtypes, mismatch counts, maximum
pixel difference, bounded coordinate/value samples, and repeat consistency.
At that stage acceptance remained exact. A parked startup log is sufficient
to capture this validation.

EV9 `000002ca--50469cb155--0` on 820f82ea then showed only 16 differences out
of 393,216 samples, exclusively in the projective probe, with repeat-stable
results. Identity/border probes passed. All eight recorded coordinate/value
pairs were reproduced against the deterministic source data: each pair selects
adjacent source pixels across a half-pixel rounding boundary. For example,
source x=867.499983 produces AMD sample x=867/value 30 and QCOM x=868/value 87.
The largest boundary distance among the logged samples is 0.000124 pixels.
Large intensity differences are expected for adjacent random source pixels;
an intensity-difference threshold would be inappropriate.

Validation now explains every differing sample from the original NV12 buffer
and float64 projection of the actual float32 transform. Only within 0.00025
source pixels of a half-pixel boundary may either adjacent sample be accepted;
both GPU values must belong to the correct camera and Y/U/V source plane, and
QCOM repeat output must match. All other differences retain the AMD fallback.
This is bounded nearest-neighbour numerical equivalence, not bit identity or
proof of driving equivalence. The eight logged samples pass regression tests;
the vehicle must still validate all differences and demonstrate actual timing.

Focused host tests cover device selection, compact transfer contents, recurrent
state identity/advancement, validation rejection and input cleanup. Existing
generic-runtime and worker-protocol tests also pass. Windows cannot execute
QCOM or the external AMD GPU; neither actual C3 speedup nor road behavior has
been validated by those tests.

Before claiming the incident fixed, obtain a C3 upload that confirms
`precompiledWarp.backend=qcom`, `usb_input_bytes=393728`, and no fallback warning.
Compare `local_prepare_ms`, `input_upload_ms`, total modeld loop, frame drops,
cameraOdometry validity and locationd/commIssue events. A smaller transfer alone
does not prove the total loop meets 50 ms. Confirm C4 remains on AMD.

For future official model updates, validate camera input shape/dtype, transforms,
output layout, recurrent state and device timing independently for C3/C4.
Compatible weight updates need not freeze C3. If hardware capacity is exceeded,
retain a known-good model/runtime pair for that device rather than weakening
validity checks. World Model-specific artifacts/control work remain local and
are not validated by Cinque tests.

Docs-Not-Needed: Internal model runtime optimization; no user setting or user-guide change.
