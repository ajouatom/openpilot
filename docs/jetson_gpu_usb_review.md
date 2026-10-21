# Jetson device-state and C3 USB priority review — 2026-10-21

This records the initial default-OFF review. The owner subsequently requested
activation and real-link validation; see [the follow-up](jetson_gpu_state_activation_20261021.md).

Reviewed and integrated the relevant source from
[7cbedfe](https://github.com/tjdgus200/openpilot/commit/7cbedfe03757c52482d9088dda49d89a94263ca4)
and [8f0cd37](https://github.com/tjdgus200/openpilot/commit/8f0cd3706604f95b35188f85899fc64a3d2bb39b).
The first adds optional GPU-resident recurrent state; the second extends the
existing bounded USB completion worker priority helper to C3/C3X.

## Integration scope

`carrot-nav` receives the GPU implementation, attribution and regression tests.
`CARROT_TRT_DEVICE_STATE` defaults to OFF; only the exact value `1` enables it
when a stateful TensorRT model is configured. It does not change model weights,
USB framing, asynchronous USB writes, deadline handling or controls.

Every recurrent float16/float32 output is checked on the GPU before the reply
event, with one four-byte status transfer. Driving-output checks remain on the
CPU. Trailing device-to-device feedback and the next reset/run share the same
CUDA stream, so old feedback cannot overwrite the new reset. Close waits for
trailing work. CUDA/NVRTC/capture failures refuse this opt-in instead of silently
skipping validation. The normal host-state path remains the default.

`carrot-wip` does not have this N16 stateful backend/queue contract. It receives
the common USB eligibility change and this review record, not NAV's model or
runtime architecture. Its existing model choices remain intact.

The USB helper now accepts exact device-tree models `comma tici`, `comma tizi`
and `comma mici`, still only on aarch64 kernel `4.9.103`. Dynamic DWC3 worker
identification, kernel-thread/starttime checks, FIFO priority 2, worker bound,
three-second lease and restoration journal remain unchanged. No C3/C3X hardware
measurement was performed; C4 results do not establish C3 latency improvement.

## Native Jetson comparison

The user powered the Jetson and allowed parked testing. A fresh Park/zero-speed/
inactive check preceded an actual offroad transition through `IsTakingSnapshot`.
A separate offroad heartbeat guarded a bounded test supervisor, which stopped
only the inference service and restored it in `finally`. Test outputs never
entered the vehicle control transport. The original parameter was restored
afterward and actual onroad transition confirmed.

The first isolated child lacked the account's supplementary GPU groups and
failed at CUDA stream creation (`/dev/nvmap` permission denied). The supervisor
restored the service. A new trial used the existing jetlink account groups
(including video/render), without changing device permissions or production
services. That trial passed.

Hardware: Orin sm87, TensorRT 10.3.0, existing cached N16 plan for model SHA256
`01effc679eb46d6036ce0302673a87fe2dcfcc95a679daeeb929679e53c3647a`.
Candidate modules were loaded from an isolated temporary directory over the
existing `691566f0fa9a47d9e7907567c48da962975b1a4c` runtime. No model rebuild or
operational source/environment replacement was used.

Four arms reused identical seeded camera/scalar/NAV inputs: OFF, ON, ON, OFF,
250 frames each, resetting at frames 0, 100 and 175. All driving-output arrays
were exactly equal across all arms, including reset boundaries; maximum
absolute difference was zero. Their output SHA256 was
`5271d93055cef7fb8c224a94a6fd7ae18b2eda10798d7c7bc5246ec62a9f1193`.

Wall times cover queue input preparation, engine run and feedback; statistics
use frames 50–249 of each arm, including the later resets:

| State location | Mean ms | p99 ms | Max ms |
| --- | ---: | ---: | ---: |
| Host, first | 26.828 | 26.953 | 26.975 |
| GPU, first | 24.633 | 24.934 | 25.013 |
| GPU, second | 24.631 | 24.877 | 24.947 |
| Host, second | 26.743 | 26.947 | 26.971 |

The average reduction is approximately 2.153 ms (8.0%) in this isolated test.
The event can return before trailing GPU feedback; subsequent frames still
order behind it. `last_gpu_us` is host wall time around the engine run, not a
CUDA kernel-only timing. These numbers exclude USB, camera capture, DM and
live navigation processing, and do not establish a 20 Hz full-system deadline,
road behavior, long-duration stability or a locationd/link-failure cure.

Actual CUDA/NVRTC finite scanning passed all 65,536 binary16 patterns and a
binary32 set containing signed zero, largest finite values, both infinities and
NaN payloads, separately checking finite-only and nonfinite-containing inputs.

## Regression and deployment status

112 focused desktop cases pass: device recurrence/graph/reset ordering,
nonfinite rejection/recovery, close after trailing work, transfer exclusion,
capture failure, stateful feedback and daemon timing, plus USB eligibility and
priority restoration. Windows cannot run the existing socket.sendmsg protocol
cases or import the unadapted fcntl daemon-compat module; Linux integration CI
is the appropriate environment for those checks.

This integration preserves the reviewed default OFF policy. The production
Jetson service was restored to runtime `691566f0fa`; GPU recurrence is not
enabled on the vehicle. The signed host-runtime pin remains unchanged. A
vehicle Git Pull/reboot picks up the common comma-side USB eligibility change;
it does not activate this optional GPU path. Publishing and selecting a new
signed host runtime, plus deliberate opt-in and full-link validation, are
separate activation work. No SD-card reflash or manual USB patch is required
for the common change. Private logs and diagnostic scripts are not published.

After restoration, the original N16 connection reported ready, the C4 DWC3
worker retained FIFO2, and Park/zero speed/inactive plus all observed service
valid/alive flags were confirmed. A subsequent 60-second read-only capture
received 1,200 valid consecutive frames each from model, DM, road/wide cameras
and camera odometry. Rates were respectively 20.001716, 20.000842, 20.000292,
20.000272 and 20.001718 Hz. All 1,200 livePose records were valid with inputsOK
and posenetOK true (20.004022 Hz). This confirms bounded restoration health,
not GPU-on live driving or long-term stability. The first observer script
mistakenly compared poller wrapper identities and counted zero messages; its
corrected version drained the subscribed sockets and produced these results.

Both source commits passed the Jetson integration workflow, including the
Windows installer check. NAV [run 39584970894](https://github.com/ajouatom/openpilot/actions/runs/39584970894)
on `a51dcb02d5` passed the Linux privileged receiver/rollback stage (94 cases),
main integration stage (987 passed, 56 skipped), navigation/scene stage (119),
real loop-filesystem stage (11), and isolated Wi-Fi forwarding stage (11).
Wip [run 39584970899](https://github.com/ajouatom/openpilot/actions/runs/39584970899)
on `e925a3cce4` passed its main integration stage (780 passed, 2 skipped),
navigation/scene stage (119), and loop-filesystem stage (11). Stage counts can
overlap and are not a count of unique tests. The final follow-up commits only
record restoration and these completed checks; runtime sources are unchanged.
