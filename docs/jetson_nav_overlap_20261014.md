# Stability-first preparation experiment

The user requested the recommended stable approach and asked whether one bad
eGPU output could be ignored. Scope remains carrot-nav only. Existing eGPU,
internal-model selection, deadlines, DM policy and rejection behavior remain.

## Experiment and decision

Code review found that preparing the next camera frame requires a larger change
to camera ownership, calibration/context snapshots and recurrent sequencing.
The first bounded experiment instead submits the current QCOM camera warp,
prepares independent current-frame CPU map/navigation/desire inputs, then reads
the completed image. It adds no frame of intentional delay, second request,
background camera reader or queue. A failed CPU preparation still drains the
submitted GPU operation before raising; it cannot send partial context.

`tools/jetlink/bench_input_overlap.py` runs a guarded parked synthetic A/B/A.
It imports a candidate from /tmp, uses lower CPU/QCOM priority than live modeld,
checks fresh Park/inactive state repeatedly and never publishes model/control
output. After compilation, subscriptions must be refreshed before checking
freshness; the first attempt stopped at that guard without running the comparison.

The completed run used 100 frames per variant, changing transforms, desire
and navigation context, including missing-map/instruction frames. Every image,
packed input and desire-history byte matched across all three variants. The
existing fused warp also passed its 27 native pixel probes. Timing excludes
the first ten frames in each variant.

| Preparation variant | Mean ms | p99 ms | Maximum ms |
| --- | ---: | ---: | ---: |
| Serial before | 11.589 | 25.491 | 31.491 |
| GPU submission then CPU preparation | 12.986 | 36.138 | 45.029 |
| Serial after | 12.235 | 32.798 | 34.091 |

These are isolated preparation timings with additional co-load and lower
priority, not production model inference times or a proof that all asynchronous
designs are slower. They establish no benefit for this candidate under the
tested conditions. **Do not enable it in production.** The temporary branch
integration in 50cd9043c9 was removed by ab1c24d7f7 before any vehicle installation.
Final JoiningModel and IPC client are unchanged from the pre-experiment source.
Warp's default call retains its original order; only the offline tool supplies
the experimental preparation callback. No 40 ms end-to-end result is claimed.

A separate ten-second production health sample around the isolated test saw
200 valid model/camera messages, 201 valid DM messages and no model frame-ID
skips. It does not establish uninterrupted DM performance throughout compilation
or the entire experiment. Raw probe source/results remain private under
`.analysis/scratch/2026-10-13-jetson-nav/`.

## One invalid eGPU output

The current worker aborts on the first wrong output count or any NaN/infinity.
The parent records failure, closes the worker and uses its internal fallback.
The previously observed NAV artifact rejection remains; no retry is enabled.

Ignoring an invalid tensor and publishing it would pass invalid predictions to
downstream consumers. Discarding that frame is different, but is not sufficient
to prove recovery: GenericModelRuntime aliases next-state outputs onto the
existing recurrent-state input buffers. The graph updates those buffers before
the worker validates the returned driving output. A rejected driving output
does not prove state corruption, but the code has no verified last-good state
rollback for a one-error continuation. Repeating an old result as a new frame
would also misrepresent its camera timestamp and conceal a publication gap.

The change only improves failure diagnosis. `output_validation.py` keeps the
same first-error rejection and reports either actual/expected element counts
or NaN/+infinity/-infinity counts plus at most eight bad indices. It does not
log camera/map/tensor contents, replace values, clear rejection or retry. Valid
outputs remain byte-identical. The early worker progress path stays stdlib-only;
the helper is imported after the existing NumPy/runtime setup.

106 focused desktop checks passed; four Linux-only IPC cases were deselected
after the wider Windows run failed solely on unavailable AF_UNIX. Linux CI
checks the socket, missing/late/truncated/nonfinite response and fallback paths.
Tests cover exact input/desire parity, draining after invalid context, GPU read
failure, every nonfinite type, count mismatch and bounded diagnostics.

Final ab1c24d7f7 CI passed 761 integration tests (11 skipped), 119 navigation/
cluster tests and 11 protected-storage tests. eGPU compatibility and Windows
installer checks passed. These are not numerical reproduction of the old eGPU
failure or evidence of faster live inference.

After another fresh Park/inactive guard and confirmation that no eGPU worker
was running, only precompiled_worker.py and its output_validation.py dependency
were installed with expected-old-hash checks and a separate device backup.
Native finite/nonfinite probes retained rejection and produced the bounded
diagnostic. No process was restarted, no timeout or rejection marker changed,
and the overlap candidate was not installed. The helper is available to a
subsequent eGPU worker; the physical eGPU is absent and live reproduction
remains unperformed.

Final readback confirmed vehicle Jetlink model.py/link.py byte-for-byte equal
to its pre-experiment files and the eGPU rejection marker still present. A
ten-second post-install health sample had 200 valid model, camera and DM
messages, all 199 successive model frame IDs consecutive, and external NAV
active/ready throughout. Mean inference remained 45.005 ms (maximum 57.282 ms).
This is a short unchanged-path health check, not a new sustained 40 ms claim.
