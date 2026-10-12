# Experimental Jetson ResAction branch

This branch adds client and host code for the **specific ONNX export** in
[commaai/openpilot commit 2fb4ac4](https://github.com/commaai/openpilot/commit/2fb4ac4aa0b2e6438ae2b8330295aa71df9a184a).
It is a source-code integration, not an installable or signed Jetson release.
Pulling this branch onto a comma alone does not install the new model on Jetson.

## Pinned model

| Field | Value |
| --- | --- |
| Upstream path | `openpilot/selfdrive/modeld/models/big_driving_supercombo.onnx` |
| SHA-256 (upstream LFS OID) | `1563b85f6bd00d9e2edf50bdb69646f0d95d5ac0f218d5abe3fe54a13427d71b` |
| File size | 792485215 bytes |
| Checkpoint | `b9facbcc-4d47-410e-b3ce-dfcbad12ba92/56320/7a933c1c-b3ba-4b31-be45-d2a03ca9fa9d/12864` |
| Client/host contract | `openpilot/selfdrive/modeld/jetlink/jetson_model.json` |

The pin was checked against the upstream LFS pointer and ONNX metadata.
Only bounded ranges of the public ONNX were fetched for code review; the full
weight file was not downloaded or executed during this change. Deployment must
verify the entire file's size and SHA-256. Do not substitute a later PR revision
or a model sharing the same display name.

## What changes

- Recognized Jetson peers request this exact ResAction contract. Internal driving
  models, driver monitoring, control code, Mac/phone model selection and
  `cinque_v2.json` remain unchanged.
- The client sends two warped images plus 12 float32 scalars. `state_img_q`,
  `state_desire_q` and `state_feat_q` stay on Jetson and advance from their
  corresponding `next_state_*` outputs. Reset clears all three histories.
- The host selects the driving output by name and validates all next states
  before committing feedback. Non-finite state invalidates the frame and clears
  history. Existing inference error/fallback handling remains in place.
- TensorRT 10.3's UINT8 input workaround covers the image queue. Only
  byte-preserving Slice/Concat/Gather/Reshape operations and related shape
  operations are promoted to FP16; unknown integer operations are rejected.
  Pixels 0..255 remain exactly representable. The original ONNX stays unchanged.
- Release tooling reads the Jetson pin and retains support for older bundles
  that contain only `cinque_v2.json`.

## Deployment prerequisites (not performed)

1. Prepare the exact ONNX and build a committed host bundle with
   `python tools/jetlink/build_host_bundle.py <output.tar.gz>`.
2. On the target Orin, build and probe that model using the matching host code,
   L4T/TensorRT environment and existing provisioning workflow. The probe now
   advances recurrent state across frames. Measure end-to-end latency and test
   output parity on recorded inputs before using the branch on a vehicle.
3. Publish the model and bundle through an authorized release location and
   generate a matching signed manifest using `tools/jetlink/sign_release.py`.
   That tool still requires the private key corresponding to the installed
   public trust root and the existing download-host policy. No signing key,
   trust-root replacement, hosted weights or new signed manifest is included.
4. Older installed standalone updaters read only `cinque_v2.json`. They need the
   updated bootstrap/updater from this source before accepting a bundle with
   the separate ResAction pin. This migration requires its own device install;
   a new `host_release.json` alone is insufficient for such installations.
5. Commit the matching signed `host_release.json` to the client branch when that
   deployment is ready. Until then the old Cinque manifest is retained as a
   reference but is not sent over the boot, Wi-Fi or HUD provisioning channels.
   A Jetson boot gate reports the missing matching release explicitly.

For manual model provisioning, `download_model.py --output <path>
--manifest-url <https-url>` accepts a manifest naming
`big_driving_supercombo.onnx` in the same directory. It verifies the pinned file
size and SHA-256. There is no default ResAction hosting URL in this branch.

## Validation scope

The focused tests cover host history/feedback/reset, non-finite rejection,
named output selection, v2/v3 framing, exact Jetson selection, unchanged App
selection, release mismatch rejection and ONNX ReferenceEvaluator parity of
the image-queue transformation. The parity fixture follows the exact export's
image-queue operations and exercises all byte pixel values across frames and
resets; it is not a full-model numerical parity result.

Run on Linux from the repository root with numpy, pytest, onnx and pycryptodome:

```sh
PYTHONPATH=.:third_party/jetlink:tools/jetlink python3 -m pytest \
  --confcutdir=tools/jetlink -o addopts='' \
  tools/jetlink/test_resaction.py tools/jetlink/test_stateful_protocol.py \
  tools/jetlink/test_v3.py tools/jetlink/test_app_models.py \
  tools/jetlink/test_download_model.py tools/jetlink/test_boot_update.py \
  tools/jetlink/test_update_host.py tools/jetlink/test_bundle.py
```

The initial host implementation copies recurrent states through pinned CPU
buffers each frame. Actual Orin TensorRT compatibility, full-model parity,
memory use and sustained 20 Hz performance of **this integration** still need
measurement; reports about another Orin implementation do not measure this
branch's transport and state-copy overhead. No hardware installation was done.
