# Cheddar internal-model review, 2026-11-18

## Decision

Hold the internal-model replacement. The current compilation entry point rejects the Cheddar input contract. An isolated adapter reaches the installed QCOM backend, but the first native attempt fails during kernel preparation and trial execution. Cheddar has no valid steady-state latency measurement from this review.

## Artifacts

The reviewed upstream PR is [commaai/openpilot #39255](https://github.com/commaai/openpilot/pull/39255), head `10548c12dee3e6b5507b2d105ec4203360aa65cd`, merged as `b3ab22efe1be5d6c0ad46503f0985b26f8b7a36f`.

| Item | Installed CD210 | Cheddar |
| --- | --- | --- |
| ONNX bytes | 60,792,584 | 69,671,018 |
| SHA-256 | `f73a9e535523d5e9acb9e642c64e33d631825dc8ba74123757d107cedd047bb5` | `350a47ea36e554fb71634c7361b2c2a2a3da07ecb211a4c6cff3d2c3013d1d82` |
| Prediction output | `[1, 2576]` | `[1, 2580]` |
| Direct action output | Absent | Four values, existing action consumer reads the first two |

The Cheddar file was fetched from the official model repository and matched the PR's exact byte count and hash. Its hosted filename is `big_driving_supercombo.onnx`; the PR installs those same bytes as the internal `driving_supercombo.onnx`. The checkpoint metadata is `c8198a2a-674c-4c0b-af1d-e7c4bb28cbf8/40960/26ada517-d522-4c97-b358-c91c34bf40b1/16384`. [Verified model source](https://huggingface.co/commaai/openpilot_driving_models/blob/main/26ada517-d522-4c97-b358-c91c34bf40b1/16384/big_driving_supercombo.onnx)

## Compatibility

CD210 receives separate `img` and `big_img` inputs, `desire_pulse`, and `features_buffer`. Carrot's compiler builds and samples these queues outside the ONNX graph.

Cheddar receives `new_img`, a current `desire`, and three recurrent state inputs. It returns updated states for the following frame.

| State input | Shape | Type |
| --- | --- | --- |
| `state_img_q` | `[2, 5, 6, 128, 256]` | uint8 |
| `state_desire_q` | `[132, 1, 8]` | float32 |
| `state_feat_q` | `[128, 1, 512]` | float32 |

Calling the current production `get_policy_npy_shapes()` with Cheddar metadata reproduces `KeyError: 'desire_pulse'`. The current warp queue builder also requires `img`. Runtime vision-stream naming, state feedback, reset, standby warmup, and recovery must accommodate the new contract before installation.

Both files pass the ONNX checker. An isolated ONNX Runtime CPU sequence runs 140 frames per model with recurrent feedback and finite outputs. Frames 0, 4, 39, and 139 pass the production `Parser`, `fill_model_msg`, and real cereal serialization. This verifies prediction layout compatibility. The native Cheddar execution failed before numerical comparison against the CPU reference was possible.

Cheddar includes direct lateral-acceleration and acceleration outputs. The current action consumer already supports that path for other models. CD210 derives actions from the predicted plan, so changing the internal model also changes the action source.

## Native timing and failure

The owner's TICI device was reachable and reported offroad throughout testing. Tests used its installed code `b5df7d4d3` and tinygrad tree `2c7501785916e365318c31976d4aef5b6e44a332`, which matches the reviewed workspace runtime tree. Online CPUs were 0–3. The observed GPU clock was 675 MHz, with a reported ceiling of 710 MHz.

Temporary model, script and cache files were placed in a dedicated device scratch directory. A 200 ms offroad observer terminates the benchmark if ignition/onroad state changes or becomes unavailable.

The benchmark uses fixed prewarped images, recurrent updates, and synchronous prediction readback. It excludes camera warp, live camera delivery, and the concurrent onroad workload. Five warmup calls and 40 settling frames precede 140 measured calls. These are offroad policy measurements; a full onroad deadline assessment needs the remaining pipeline and load.

| Run | Mean | Median | p95 | Maximum | Above 50 ms |
| --- | --- | --- | --- | --- | --- |
| CD210 before Cheddar | 25.58 ms | 25.12 ms | 29.62 ms | 34.50 ms | 0 / 140 |
| Cheddar | Unavailable | Unavailable | Unavailable | Unavailable | Unavailable |
| CD210 after Cheddar failure | 25.25 ms | 25.15 ms | 29.03 ms | 35.02 ms | 0 / 140 |

The temporary Cheddar adapter executes the official ONNX graph and feeds its next-state outputs back into device buffers. It fails on its first call while realizing the graph, before recurrent-state assignment and before a completed warmup call. The retained traceback reaches `compile_linear`, then `optimize_local_size`, and reports `AssertionError: all optimize_local_size exec failed`. QCOM synchronization reports a 30,000 ms wait timeout, with expected signal 7181 and observed 7180. This timeout is a kernel-preparation failure and provides no steady-state inference-time value. The underlying kernel/driver cause remains unresolved.

A bounded diagnostic retry retained a log and reached its 150-second process limit during preparation (exit 124). It did not produce a completed inference or a second independently reproduced kernel failure. The first attempt's tool output truncated repetitive cleanup exceptions, so the private evidence explicitly identifies its retained traceback as a summary.

The failure-after-CD210 sequence and successful subsequent CD210 run establish that this candidate does not pass the installed execution path. They do not establish that Cheddar fails with every tinygrad version or on every comma device. The upstream snapshot pins tinygrad `ec91b74b5b6b0fc924b72d3f27689679ca2438b9` and uses a different generic compilation and state-buffer integration.

## Evidence

Private artifacts, metadata, reference outputs, native scripts, timings and diagnostics are under `.analysis/scratch/2026-11-18-cheddar-review`. The test adapter is an evaluation artifact. Installation remains blocked on stateful runtime integration, successful native execution and numerical comparison, followed by representative end-to-end latency validation.
