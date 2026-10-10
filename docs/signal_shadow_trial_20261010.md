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

## Installed device verification

Installed and normally rebooted the connected C4 at 192.168.0.178 while fresh valid
carState/selfdriveState/carControl confirmed Park, standstill and inactive control.
eGPU was physically absent and UsbGpuActive remained false. Runtime code commit
`cb45d99b61` uses policy artifact SHA256
`8b0941e9e36b3b869918dccde613a1365e36ee1ec160ba884c228f9594f871a2`.

- Device tests: 39 passed (observer, existing helpers and eGPU startup retry).
  Three warnings concern absent optional pytest plugins in the isolated test environment.
- 40-second live observation: 800/800 valid model frames, 20.0106 Hz, zero frame gaps,
  zero model frame-drop percentage; mean execution 24.656 ms, max 30.805 ms.
- 200 comparison events (5 Hz); 199 frame IDs matched the model subscription window.
  Live baseline values agree with published modelV2 within the five-decimal logging
  rounding bound (maximum 0.000005).
- Worker mean 28.542 ms, p95 38.645 ms, max 51.267 ms under live load. Eight comparison
  packets were dropped during worker startup; the counter stayed at eight throughout
  the observation window. Worker affinity 0..3, SCHED_OTHER, nice19 were verified.
- Live separate-policy versus QCOM baseline maximum differences: x 0.03311 m,
  v 0.01028 m/s, a 0.00097 m/s?. These remain logged separately.
- Completed segment `0000108c--e6597f4447--0` contains 1,053 modelV2 messages and
  247 signalModelShadow events (frame IDs 217..1201); both fcamera.hevc and ecamera.hevc
  exist. This verifies storage in rlog, not just display/subscription.

The vehicle remains on the comparison branch with observation enabled. These are parked
runtime and recording checks, not proof of improved traffic-signal recognition or driving.
Private reproduction and validation evidence is indexed in the local analysis archive.
