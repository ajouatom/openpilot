# Internal signal model observer — 2026-10-10

The user requested a separate trial branch and installation on their parked Ioniq 5 PE C4.
`carrot-signal-shadow` keeps the original internal model in control and runs the trained
longitudinal head candidate in a separate CPU observer. This is not a promotion of the
candidate: the prior training trial failed green-progress preservation criteria.

## Architecture

The original QCOM artifact, inference call, parser, history feedback and control outputs
are unchanged. An opt-in observer copies the original exposed 512-value camera feature,
previous-feature history and desire pulses. It mirrors the original 96-frame feature queue
(sample every fourth value) and 100-frame desire queue (max over each group of four).
Only actual policy runs advance history; prepare-only frames do not.

Every fourth run submits a copy through a private nonblocking Unix datagram socketpair.
The separate worker runs ONNX Runtime CPU with one thread, SCHED_OTHER, nice 19 and cores
0..3. Its policy-only ONNX shares the original policy backbone and adds the candidate
99-value longitudinal head. No experimental output returns to modeld. Slow/full queues
drop comparison packets, packets older than 400 ms are skipped, and a worker failure
leaves original inference running. CPU and memory contention still require device checks.

`signalModelShadow` logMessage events contain camera frame IDs/timestamp, model hashes,
33 x/v/a values for the live baseline, separately evaluated policy baseline, and candidate,
inference time and parent queue-drop count. Compare candidate against the policy baseline
to separate head effects from runtime arithmetic differences. Logs contain predictions,
not ground-truth signal labels. Logging/IPC can lose samples under load.

## Selection and local artifacts

Default OFF. `/data/signal-model-shadow/enabled` must contain `1` at the next modeld startup.
There is no experimental-control selector. External eGPU/Jetson paths do not emit internal
comparison samples. Artifact errors leave the original internal model running.

```sh
cd /data/openpilot
/usr/local/venv/bin/python -m openpilot.selfdrive.modeld.signal_shadow status
/usr/local/venv/bin/python -m openpilot.selfdrive.modeld.signal_shadow on
/usr/local/venv/bin/python -m openpilot.selfdrive.modeld.signal_shadow off
```

Changes apply at the next normal modeld startup. Device-local `policy_shadow.onnx`,
`installed.json` (version 2 and checksums), and the isolated `runtime/` are required.
The ordinary Python environment and model files are not overwritten.
`tools/model_training/build_signal_shadow.py` verifies that only longitudinal mean-head
weights/biases differ before extracting the separate policy observer. Source models and
private footage remain local, not in Git.

Base SHA256: `f73a9e535523d5e9acb9e642c64e33d631825dc8ba74123757d107cedd047bb5`.
Candidate SHA256: `161fef282f795ecfc4680698dcaf5d217a68e5df14e9fcadea290f84267c7fd4`.

## Rejected combined-graph approach

The first implementation appended comparison outputs to the controlling QCOM graph.
PC verification on 43 recorded inputs preserved every baseline output exactly; independent
candidate discrepancy was at most 0.0078125. However, the C4 compiled combined graph changed
the original outputs substantially on identical input/history queues. Frame-zero x10 was
124.445 for the original compiled model versus 203.000 for the combined graph; original
ORT was 124.813. Original ONNX identity and runtime tree were checked. The exact compiler
cause remains unresolved. That graph was never enabled for control or live comparison.
Version 2 rejects its version-1 manifest and never loads its compiled file.

## Separate observer validation

- 43 recorded inputs: separate-policy versus recorded original longitudinal maximum
  difference x 0.125 m, v 0.015625 m/s, a 0.001953125 m/s².
- 22 native QCOM original outputs, with matching history: maximum difference x 0.2655 m,
  v 0.03192 m/s, a 0.00537 m/s². Original QCOM remains controlling; these are observer
  arithmetic differences, not changes to vehicle targets.
- C4 CPU benchmark, 100 repetitions on cores 0..3/nice19: median 21.70 ms, p95 26.34 ms.
  Compared with PC CPU inference on the same feed, maximum baseline difference 0.0001221
  and candidate difference 0.00006104. Full live contention validation is recorded below
  when installation is complete.
- Seven desktop tests cover default-off, checksum rejection, rejection of old combined
  graphs, exact history sampling/pulse pooling, input immutability and IPC failure isolation.
  Ruff passes for the helper, builder and tests.

## Further data

Retain both original road/wide HEVC streams and rlog across approach, red waiting, green
transition and normal departure. Include day/night, different intersections, with/without
leading vehicles, correct detections and failures. Label the applicable lane's real signal
and transition times. Split train/validation by drive or intersection, not adjacent frames.
More diverse labelled data can improve training and reveal regressions; quantity alone
does not establish improvement. Closed-loop behavior and signal recognition gains remain
unvalidated by this comparison-only installation.
