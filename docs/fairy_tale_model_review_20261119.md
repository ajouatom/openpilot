# Fairy Tale model review, 2026-11-19

## Decision

Keep Cheddar as the internal default and retain the existing external catalog.
[Upstream PR #39415](https://github.com/commaai/openpilot/pull/39415), reviewed at
`57a0a395cad83c026405fa460e5e542791d1ffd1`, changes only the external
`big_driving_tinygrad.pkl`. It does not change the internal model or execution
code. The PR was open at review time.

It reapplies `d739e474a3fd3df31b4e1d0db19c60d8d835d57c`, previously reverted by
`03fd773f295fa69697094c7aefac7a9e0fc9c08b`. Those commit messages do not explain
the revert's cause; do not infer a defect or a validated cure from that history.

## Provenance and availability

The new official compiled artifact is 764,592,225 bytes, SHA-256
`16882f1695767f3727e042cdeb73a3329ea956bcee9be2e721fadff262266a3b`, replacing
Light speed 2's `b4dead3e...` artifact. Upstream pins tinygrad
`ec91b74b5b6b0fc924b72d3f27689679ca2438b9`, the revision already isolated for
Cheddar and Light speed 2. Matching a revision alone does not verify a new PKL.

The PR's configured public Hugging Face LFS batch endpoint returned
`404 Object does not exist` for the exact new artifact identity. Consequently,
its complete hash, serialized metadata and actual eGPU execution could not be
verified. No unverified PKL or catalog entry was published.

The [official ONNX source](https://huggingface.co/commaai/openpilot_driving_models/blob/main/e0623b24-06b0-46e7-9f2b-79d38cfea203/32768/big_driving_supercombo.onnx)
was available: 766,137,036 bytes, SHA-256
`bc125a9af33033ded89ac1886e27dbbe296da285275b2eb8fc0b5edb35b56c41`.
The downloaded file matches the hosting repository's LFS identity and passes
the ONNX checker. Its checkpoint is
`f22e06e5-3b3f-48d2-a6b9-2f32dc20e707/56320/e0623b24-06b0-46e7-9f2b-79d38cfea203/32768`.
This associates the source with the upstream commit's checkpoint label; the
unavailable PKL still needs independent source/contract verification.

## Compatibility and execution

Input/output shapes and every output slice match the existing Light speed 2
contract. Inputs are the packed two-camera image, desire, traffic convention,
action times and three recurrent states. `state_feat_q` has shape
`[128, 1, 16384]`, and prediction output is `[1, 18452]`. There are no `nav_map`
or `navigation` inputs; this is not an N16 navigation replacement.

ONNX Runtime CPU completes 40 changing-image/desire frames with recurrent
feedback and finite outputs. Frames 0, 5, 20 and 39 pass the production parser,
`fill_model_msg` and real cereal serialization. This establishes source-level
contract compatibility, not numerical equivalence to the compiled GPU artifact,
Jetson TensorRT compatibility, or driving quality.

The [official published model replay](https://github.com/commaai/openpilot/pull/39415#issuecomment-7000994106)
contains 60 Chestnut model samples: mean 39.388 ms, p95 39.776 ms, maximum
39.922 ms, none above 50 ms. The small-model replay is unchanged by this PR:
its first call is 388.615 ms, and the remaining 59 average 29.854 ms. These are
upstream replay measurements, not measurements on the owner's eGPU or Jetson.
The process replay's 80 passing cases do not establish new-model road behavior.

Before promotion, obtain and verify the exact official PKL, inspect its contract
and execute it through the pinned eGPU runtime. Jetson separately needs this
ONNX's model specification, matching signed runtime delivery, TensorRT engine
validation and device timing. No current model selection or external runtime
pin was changed by this review.

Private source/hash records, replay decoding and CPU checks are retained under
`.analysis/scratch/2026-11-19-fairy-tale`.

## Independent Jetson attempt, November 19 evening

The owner explicitly requested Jetson-first validation from the available ONNX.
This does not require the AMD PKL. A successful Jetson validation can support a
Jetson-only catalog entry, provided delivery and backend availability handling
are implemented; an absent official PKL must not be advertised as eGPU support.
The exact official PKL still returned 404 on the subsequent retry.

The owner's Orin Nano 8 GB was reachable in MAXN_SUPER, with TensorRT 10.3 and
the existing signed N16 host `0cb67d46cc4ac448326cc9b272ebfebdc6fde797`.
Following explicit authorization, rebooted comma after verifying fresh
P/zero/inactive state, then temporarily stopped only the Jetson inference
service. The user imposed a departure deadline of approximately five minutes.
An independent vehicle-state reader and bounded host supervisor required fresh
P/zero/disabled samples and restored the original service on completion or
loss of the heartbeat. No selection or installed runtime pin changed.

The hash-verified ONNX reached TensorRT parsing, tactic generation, timing-cache
serialization and the staged plan write. The build printed
`NvMapMemAllocInternalTagged: 1075072515 error 12`. However, this warning did not
stop the builder from returning a serialized plan: the timing cache existed
and the staged `engine.plan` had 534,724,608 bytes when interrupted. A repeated
SSH connection used for heartbeat renewal timed out during banner exchange;
the host supervisor terminated the benchmark process group and restarted the
production service. There was no completed atomic plan, deserialization check
or inference sample. Do not characterize this as proven model incompatibility,
a completed engine build, or a measured inference deadline failure.

The production service restarted successfully, reloaded the original N16 engine,
and comma subsequently reported N16 `active=true`, `ready=true`, empty error.
Cheddar remained available internally. The incomplete temporary build directory
was removed after checking the test processes were gone (1,300,859,716 bytes).
Only the verified source, model specification, scripts, log and advisory timing
cache remain in `/opt/carrot-jetlink/fairy-test-20261119` for a later attempt.
No further device testing was performed after the departure cutoff.

The offline harness now sends heartbeat messages through one persistent SSH
session instead of repeatedly negotiating new SSH sessions. The four-second
remote expiry and fresh vehicle-state requirements are retained. This harness
revision has not yet been exercised on the Jetson. N16/Light speed 2/Fairy Tale
timing comparison, device reset/output validation and any catalog promotion
remain outstanding.
