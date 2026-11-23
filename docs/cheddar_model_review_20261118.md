# Cheddar internal-model review, 2026-11-18

## Decision

On November 20 the owner requested a return to CD210 after disliking Cheddar's driving behavior. Restore the exact previous CD210 ONNX and its vendored tinygrad compiler/ModelState path together, including the fixed model name in Carrot Web. External eGPU/Jetson choices and runtime pins stay unchanged. The isolated Cheddar implementation remains available for explicit experiments but is no longer the internal default. Normal update and restart rebuild the internal artifact; this repository change does not establish installation on a particular vehicle.

The November 19 promotion and validation below are historical. Matching the official CPU configuration removed the observed synthetic deadline overruns, and worker/reset checks passed; these checks did not establish preferred driving behavior. Returning to CD210 also restores its existing optional signal-observer path. No new observer setting is enabled by this change.

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

Both files pass the ONNX checker. An isolated ONNX Runtime CPU sequence runs 140 frames per model with recurrent feedback and finite outputs. Frames 0, 4, 39, and 139 pass the production `Parser`, `fill_model_msg`, and real cereal serialization. This verifies prediction layout compatibility. The initial native attempt failed before numerical comparison; the follow-up below supplies successful native results.

Cheddar includes direct lateral-acceleration and acceleration outputs. The current action consumer already supports that path for other models. CD210 derives actions from the predicted plan, so changing the internal model also changes the action source.

## Initial native timing and failure

The owner's TICI device was reachable and reported offroad throughout testing. Tests used its installed code `b5df7d4d3` and tinygrad tree `2c7501785916e365318c31976d4aef5b6e44a332`, which matches the reviewed workspace runtime tree. Online CPUs were 0–3. The observed GPU clock was 675 MHz, with a reported ceiling of 710 MHz.

Temporary model, script and cache files were placed in a dedicated device scratch directory. A 200 ms offroad observer terminates the benchmark if ignition/onroad state changes or becomes unavailable.

The benchmark uses fixed prewarped images, recurrent updates, and synchronous prediction readback. It excludes camera warp, live camera delivery, and the concurrent onroad workload. Five warmup calls and 40 settling frames precede 140 measured calls. These are offroad policy measurements; a full onroad deadline assessment needs the remaining pipeline and load.

| Run | Mean | Median | p95 | Maximum | Above 50 ms |
| --- | --- | --- | --- | --- | --- |
| CD210 before Cheddar | 25.58 ms | 25.12 ms | 29.62 ms | 34.50 ms | 0 / 140 |
| Cheddar | Unavailable | Unavailable | Unavailable | Unavailable | Unavailable |
| CD210 after Cheddar failure | 25.25 ms | 25.15 ms | 29.03 ms | 35.02 ms | 0 / 140 |

The temporary Cheddar adapter executes the official ONNX graph and feeds its next-state outputs back into device buffers. It fails on its first call while realizing the graph, before recurrent-state assignment and before a completed warmup call. The retained traceback reaches `compile_linear`, then `optimize_local_size`, and reports `AssertionError: all optimize_local_size exec failed`. QCOM synchronization reports a 30,000 ms wait timeout, with expected signal 7181 and observed 7180. This timeout is a kernel-preparation failure and provides no steady-state inference-time value. The underlying cause was unresolved at this stage; see the follow-up.

A bounded diagnostic retry retained a log and reached its 150-second process limit during preparation (exit 124). It did not produce a completed inference or a second independently reproduced kernel failure. The first attempt's tool output truncated repetitive cleanup exceptions, so the private evidence explicitly identifies its retained traceback as a summary.

The failure-after-CD210 sequence and successful subsequent CD210 run establish that this candidate does not pass the installed execution path. They do not establish that Cheddar fails with every tinygrad version or on every comma device. The upstream snapshot pins tinygrad `ec91b74b5b6b0fc924b72d3f27689679ca2438b9` and uses a different generic compilation and state-buffer integration.

## Follow-up: first failure and correction

The diagnostic retry records the first exception before the optimizer can suppress it. Cheddar's `node_index_select` gathers nine overlapping 25-row windows from `[1, 33, 8]` desire history: 225 selections. The old constant-Gather implementation expands these into 225 individual slices and padded concatenations. The resulting `E_9_25_2_4` shader has 388,368 bytes.

QCOM encodes its shader load length in 128-byte units. This shader needs 3,035 units, but `CP_LOAD_STATE6_0_NUM_UNIT` is a 10-bit field with a maximum of 1,023. The generated command word is `0x2f6f60000`, exceeding uint32. `array.array('I', self._q)` raises `OverflowError` before submission. HCQ has already advanced its timeline; after the optimizer swallows the exception, subsequent trials wait for a signal that the rejected command never submitted. This explains the later timeout. Replaying the retained program alone reproduces the same first exception.

Coalesce consecutive constant indices into contiguous slices, preserving ordering, repeats, negative-index normalization and output shape. For this node, nine slices represent the same 225 selections. Its generated source shrinks from 26,822 to 1,549 characters, and its shader from 388,368 to 3,064 bytes (24 load units). No command truncation, timeout relaxation, omitted model input or changed history is used. CD210's only constant Gather is scalar, so its selection expression is unchanged.

The corrected installed-runtime adapter completes warmup and 180 recurrent frames, all finite. Tests compare selection against NumPy on all positive/negative axes, overlapping windows, repeated/reversed/negative indices, scalar selection, noncontiguous input and changed inputs after JIT capture. All 32 pass using the desktop Python backend; the compatibility workflow also runs them on native CPU. This fixes the reproduced history-selection failure, not every possible oversized shader.

## Follow-up: matching runtime and execution

The exact upstream tinygrad is `ec91b74b5b6b0fc924b72d3f27689679ca2438b9`. Its `examples/openpilot/compile_onnx.py` and compiler flags from the reviewed openpilot snapshot compile Cheddar successfully. Initial execution, JIT replay, changed-input checking and the saved-artifact round trip pass. This is an isolated runtime; the production tinygrad is not replaced wholesale.

The upstream contract supplies named output buffers and aliases `next_state_*` outputs to the corresponding state inputs. A 540-frame sequence (40 settling, 500 measured) with separate state copies and one with those aliases produce exactly equal prediction captures at frames 0, 4, 39, 139 and 539. Separate copies cost 56.06 ms on average; aliases cost 32.97 ms. Compiling output transfer once, as upstream does, also preserves those captures exactly; that run averages 33.39 ms, p95 37.54 ms, maximum 42.00 ms, with 0/500 over 50 ms.

The corrected old runtime's captured predictions differ from matching-upstream predictions by at most 0.123 across the four common capture frames, with maximum frame mean absolute difference 0.0011. Against ONNX Runtime CPU, matching-upstream recurrent outputs have maximum absolute difference 4.472 and mean 0.0549 at the settled frame; these are mixed prediction fields, not one physical quantity. Supplying the same GPU state to a single CPU inference reduces these to 0.998 and 0.0226; the first two action differences are approximately 0.00038 and 0.00101. This characterizes precision and recurrence differences; it does not establish a general numerical acceptance threshold or driving equivalence.

## Follow-up: image preparation comparison

Both following measurements use the same offroad TICI, online CPUs 0–3, two synthetic 1928×1208 NV12 images, warp, recurrent policy and synchronous prediction readback. Each has five warmup calls, 40 settling calls and 500 measurements. CD210 uses its resident camera-buffer path. The upstream Cheddar candidate packs/copies both full NV12 images into its shared input buffer each frame, then performs its compiled upload, warp, policy and download. These are the respective execution paths, so the difference includes input handling as well as model computation.

| Path | Mean | Median | p95 | Maximum | Above 50 ms |
| --- | --- | --- | --- | --- | --- |
| Installed CD210 | 28.70 ms | 28.38 ms | 34.61 ms | 51.90 ms | 1 / 500 |
| Cheddar, matching upstream runtime | 41.26 ms | 40.43 ms | 49.60 ms | 65.74 ms | 22 / 500 |

Cheddar is about 44% slower in this comparison. The CD210 repeat starts after the corrected Cheddar process has exited. Neither measurement includes real camera synchronization, publication, competing onroad processes or sustained thermal testing. The upstream full-image packing cost is an additional optimization target before production integration; policy-only timing must not be presented as whole-frame timing.

Model integration still needs explicit state-buffer, reset, standby warmup and recovery handling, plus representative end-to-end deadline validation. This review does not change the internal default model or install a candidate runtime on the vehicle.

## Evidence

Initial private artifacts are under `.analysis/scratch/2026-11-18-cheddar-review`; follow-up programs, first-error records, generated shader sources, reference comparisons and full timing samples are under `.analysis/scratch/2026-11-18-cheddar-runtime`. Tests used temporary models/scripts rather than installed model artifacts. Diagnostic imports also populated the existing content-addressed compiler cache; no Params, vehicle control or installed source/model files were changed.

## November 19: matching the official scheduling and integrating execution

Repeat on the same offroad device with all CPU cores online, the inference thread pinned to core 7 with `SCHED_FIFO` priority 54, CPU governors set to performance and the normal 1,689,600 kHz cap, and the GPU interrupt assigned to core 7. Snapshot and restore the previous power/IRQ settings around each isolated command. A separate supervisor checks offroad state every 200 ms and terminates the entire test process group if it changes. The integrated worker reports affinity `[7]` and priority `54` in its startup handshake.

These settings match the relevant upstream modeld/CPU configuration. They do not reproduce simultaneous camera capture, driver monitoring, other onroad processes, vehicle motion or a sustained thermal workload. All rows use synthetic full-resolution NV12 input, image warp, recurrent inference and synchronous readback, with 40 settling frames and 500 timed calls. The integrated row also includes IPC, validation and the production output parser. CD210 uses its existing resident image-buffer path; Cheddar uses the official full-image upload path.

| Execution path | Mean | p95 | Maximum | Above 50 ms |
| --- | --- | --- | --- | --- |
| Installed CD210 | 21.62 ms | 21.96 ms | 22.19 ms | 0 / 500 |
| Cheddar, official runtime/transfer path | 31.12 ms | 31.33 ms | 31.63 ms | 0 / 500 |
| Cheddar, integrated worker and parser | 32.59 ms | 32.80 ms | 32.91 ms | 0 / 500 |

The integrated worker loads and warms in 2.98 seconds in this cached-artifact test. After warmup/reset, the first output matches a fresh reset exactly. Two 150-frame sequences with changing images, action inputs, desire edges and catch-up calls also match exactly after reset. Power restoration is read back after the timing tests; online cores return to 0–3.

The new compiler uses the unmodified `ec91b74` source archive, verified by SHA-256, and the upstream generic ONNX compiler. The worker selects that runtime before importing tinygrad, so it can coexist with the legacy runtime in modeld and driver monitoring. GPU state stays resident: next-state outputs alias the three input-state buffers. Host uploads and prediction readback are compiled once. Reset clears the same GPU allocations and waits for acknowledgement before clearing the parent's desire-edge history. Synthetic standby warmup clears history afterward; external-model fallback uses the same reset interface. The self-contained ONNX graph produces a complete policy on catch-up calls as well.

Combining policy and warp object graphs in one pickle memo produced an HCQ parameter-index failure during the first integration attempt. Serialize each warp independently, as upstream's separate files do, then include its bytes in the chunkable bundle. The saved/reloaded integrated QCOM run passes after this correction. Native CPU tests also cover an empty buffer arena and require materializing a synthetic next-state value before writing an alias; arbitrary graphs are not assumed to be alias-safe.

Linux CI compiles the actual Cheddar ONNX with the official Linux `CPU:LLVM` backend, compares 40 recurrent frames with aliased versus separate state destinations exactly, and exercises the production worker at 1928×1208 and 1344×760, including warmup, changed input, catch-up and reset. The alternative C renderer triggered a clang 18 frontend crash; it is not the production Linux backend. Related desktop checks cover failure containment, reset acknowledgements, model selection, external fallback and the existing action conversion.

The new code does not alter Carrot's action conversion, stopping policy or smoothing. Cheddar supplies the existing direct-action branch instead of CD210's plan-derived action branch; the model replacement itself can therefore change driving behavior. Road behavior, concurrent onroad deadline margin and thermal endurance remain unmeasured. The full release build and broader CI run after publication.

Private follow-up evidence is under `.analysis/scratch/2026-11-19-cheddar-integration`. The device disconnected during a subsequent fresh production-compiler run; its completion and scratch cleanup require reconnection. Successful timing/reset results above were observed before that disconnection. No installed vehicle source/model, Params or control commands were changed.

## November 19 evening: installed-device validation and cleanup

The owner restored device access and explicitly requested installation. With
`IsOnroad=0` and no tracked device edits, fast-forwarded the installed checkout
from `a9811adce075a81f6a719e611f28ae444193f750` to
`93c5c6afafa5016f9c94347da6982c967d9dd39b`, then used the normal reboot path.
The production SCons compiler completed a fresh QCOM Cheddar bundle and the
full device build. The installed ONNX SHA-256 matches `350a47ea...d1d82`.
The production worker passes warmup, changed-input, catch-up and exact reset
checks at both 1928x1208 and 1344x760. No scratch bundle was substituted for the
installed build. The earlier interrupted scratch compiler had not produced a
completed artifact.

After the owner enabled vehicle READY, the real cameras were 1344x760. Modeld
and its Cheddar worker ran on core 7, FIFO54. An initial 70-second live window
contained 1,398 consecutive valid model frames, mean 34.30 ms, maximum
44.97 ms, none above 50 ms, with valid camera odometry and livePose. All samples
were P, zero speed, lateral and longitudinal controls inactive. Because Jetson
became ready during the overall startup observation, the subsequent test
explicitly tracked backend status rather than treating this first window as
an isolated backend benchmark.

A bounded test suspended only the Jetlink daemon for 45 seconds, continuously
requiring fresh P/zero/inactive state. It retained the exact process identity,
restored the daemon in `finally`, and used an independent 50-second resume
watchdog. No model selection or control Params were changed.

| Observed stable backend | Frames | Mean | Maximum | Above 50 ms |
| --- | ---: | ---: | ---: | ---: |
| Cheddar internal fallback, seconds 3–44 | 820 | 34.32 ms | 45.21 ms | 0 |
| N16 after external recovery | 1,406 | 41.43 ms | 50.03 ms | 1 |

All samples in these stable windows had valid model, camera odometry and pose.
The deliberate link interruption itself cost 207.41 ms before fallback,
followed by a three-frame ID gap and one invalid camera-odometry observation.
Pose remained valid and no selfdrive alert appeared in the sampled test. This
is successful fault recovery, not seamless zero-gap switching. N16 resumed
about ten seconds after the daemon was restored. The device was left with its
existing N16 selection active and Cheddar available as the internal fallback.
These are parked live-camera checks, not a road trial or thermal-endurance test.

Earlier raw timing arrays, prediction captures, scripts and logs were recovered.
The retained scratch bundle and warp were copied to private evidence with
matching device/local SHA-256. After confirming no process used the exact
scratch directory, removed its 5,319 files and verified the directory absent.
The live worker's shared mmap file and installed model/runtime were retained.
Evidence is under `.analysis/scratch/2026-11-19-cheddar-install`.

The published code's final full build and actual SCons-artifact execution
([tests 44456161179](https://github.com/ajouatom/openpilot/actions/runs/44456161179)),
[model compatibility](https://github.com/ajouatom/openpilot/actions/runs/44456161337),
[Jetson integration](https://github.com/ajouatom/openpilot/actions/runs/44456161249)
and documentation-policy checks all passed before this installation.
