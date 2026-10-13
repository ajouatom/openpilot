# Experimental Jetson ResAction branch

This document records the earlier `codex/jetson-resaction` experiment. The
current `codex/jetson-nav-rl` branch selects a different model and package;
use [NAV_RL.md](NAV_RL.md) for its contract, preparation and validation.

This branch adds client and host code for the **specific ONNX export** in
[commaai/openpilot commit 2fb4ac4](https://github.com/commaai/openpilot/commit/2fb4ac4aa0b2e6438ae2b8330295aa71df9a184a).
The experiment can be preinstalled once from a PC. The comma branch then selects
it over USB; restoring the official comma branch selects the official Internet
release again at the next Jetson boot. No QSPI, JetPack or OS replacement is part
of this installation. Pulling this branch onto a comma alone is insufficient
until the PC package has been registered on Jetson.

## Pinned model

| Field | Value |
| --- | --- |
| Upstream path | `openpilot/selfdrive/modeld/models/big_driving_supercombo.onnx` |
| SHA-256 (upstream LFS OID) | `1563b85f6bd00d9e2edf50bdb69646f0d95d5ac0f218d5abe3fe54a13427d71b` |
| File size | 792485215 bytes |
| Checkpoint | `b9facbcc-4d47-410e-b3ce-dfcbad12ba92/56320/7a933c1c-b3ba-4b31-be45-d2a03ca9fa9d/12864` |
| Client/host contract | `openpilot/selfdrive/modeld/jetlink/jetson_model.json` |

The pin was checked against the upstream LFS pointer and ONNX metadata. The
complete public weight file was downloaded for the PC package and its size and
SHA-256 were verified. Package preparation and installation repeat those checks.
PC validation executes the full model; target Orin execution remains pending.
Do not substitute a later PR revision or a model sharing the same display name.

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
- The v2 host explicitly advertises `stateful_v2`. The client accepts stateful
  inference over v2 only after this capability is confirmed by HELLO. Older v2
  peers remain rejected; v3 behavior and queued v2 models are unchanged.

## PC package and branch switching

| Comma selection | Jetson at its next boot |
| --- | --- |
| `codex/jetson-resaction` | Verify and activate the root-registered local ResAction bundle/model; no model download server needed |
| Official Carrot branch | Verify its signed manifest, download the official bundle/model if needed, probe and activate them using the existing Internet path |

Switch the comma branch first, then restart Jetson with the USB connection
attached. Switching the comma alone while Jetson stays powered does not rerun
the boot selector. Internet must be available for uncached official downloads.
The official model cache and local experiment are retained, allowing later
switches without installing from the PC again.

Build the preparation package from committed sources on the PC:

```sh
python tools/jetlink/download_experiment.py --output /path/to/package/model.onnx
python tools/jetlink/build_host_bundle.py /path/to/host.tar.gz
python tools/jetlink/prepare_local_experiment.py \
  --bundle /path/to/host.tar.gz --model /path/to/package/model.onnx \
  --output /path/to/package
```

The package includes the original hash-verified ONNX, the exact committed host
bundle, an installer and a checksum manifest. No model weights are committed to
Git. The host runtime must already provide L4T 36.4.7 and TensorRT 10.3.0, just as
the official update policy requires.

For installation, access the Jetson in recovery mode and identify/mount its
existing protected DATA partition. Use a root Linux Python process against the
**DATA `runtime` directory**, not the immutable APP recovery runtime:

```sh
sudo python3 /path/to/package/install_local_experiment.py \
  --package /path/to/package --runtime /mounted/DATA/runtime
```

This command requires an existing protected runtime and boot-update marker.
It verifies every package file, registers a fixed local profile, backs up the
previous small updater scripts, and extends the updater's selection handling.
It does not switch `current`, replace the official public signing key, change
Wi-Fi/accounts, disable updates, write firmware, or repartition storage. The
original `boot-update-required` policy is retained. A package with the same
profile but different contents is rejected; use a new profile for a new host
revision. The installer is also usable on a live host only with the model, HUD,
boot gate and staging services stopped; recovery mode is preferred.

The USB request `local_host_release.json` can select only that root-owned local
record. It cannot register a new package, provide paths or URLs, or grant trust
to arbitrary unsigned network content. Official selections still pass the
original signature/host checks. Local and official activation share the same
probe, transaction, crash recovery and rollback handling. A failed experimental
probe retains the prior release and withholds readiness for the failed selection.

The first experimental boot builds the engine on Jetson and runs the existing
finite-inference probe with recurrent feedback before activation. The experiment
has not yet been installed or executed on the user's hardware. Measure latency
and full-model parity on the target before treating this integration as verified.

## Validation scope

The focused tests cover host history/feedback/reset, non-finite rejection,
named output selection, v2/v3 framing, exact Jetson selection, unchanged App
selection, release mismatch rejection and ONNX ReferenceEvaluator parity of
the image-queue transformation. The parity fixture follows the exact export's
image-queue operations and exercises all byte pixel values across frames and
resets; it is not a full-model numerical parity result.

The downloaded full export also passed input/output shape agreement and the
ONNX checker with `full_check=True` after applying the same layout-node removal
and UINT8 promotion used by the TensorRT backend. This checks graph structure
and inferred types, not TensorRT execution or prediction parity.

A complete client/host session test caught an integration error missed by the
earlier isolated tests: the deployed host speaks v2, while the client rejected
all stateful models on v2 before sending the first frame. Explicit capability
negotiation fixes that path. The `resaction-2fb4ac4-v2` PC package supersedes v1;
use the updated comma branch and package together.

On 2026-10-12, ONNX Runtime 1.31.0's CPU provider executed the full original
export (only its layout hint removed) against the promoted export through the
production Jetlink client, v2 host Session, StatefulQueues and output parser.
All 18,452 driving values and all three next-state tensors were exactly equal
over 140 continuous synthetic frames plus eight frames replayed after reset.
Inputs covered black/white images, all byte values, seeded noise, desire pulses,
both traffic conventions and changing action delays. A NaN injected into the
next feature state was rejected, abandoned the client link and cleared queues.
The focused Linux suite passed 304 tests with one external-reference test
skipped. These are synthetic PC results, not road-data replay or a vehicle
safety result. CPU loopback latency is not an Orin performance measurement.

Run on Linux from the repository root with numpy, pytest, onnx and pycryptodome:

```sh
PYTHONPATH=.:third_party/jetlink:tools/jetlink python3 -m pytest \
  --confcutdir=tools/jetlink -o addopts='' \
  tools/jetlink/test_resaction.py tools/jetlink/test_stateful_protocol.py \
  tools/jetlink/test_v3.py tools/jetlink/test_app_models.py \
  tools/jetlink/test_download_model.py tools/jetlink/test_boot_update.py \
  tools/jetlink/test_update_host.py tools/jetlink/test_bundle.py \
  tools/jetlink/test_local_experiment.py tools/jetlink/test_resaction_session.py
```

The initial host implementation copies recurrent states through pinned CPU
buffers each frame. Actual Orin TensorRT compatibility, TensorRT output parity,
memory use and sustained 20 Hz performance of **this integration** still need
measurement; reports about another Orin implementation do not measure this
branch's transport and state-copy overhead. The local catalog/installer
integration tests use a Linux root test environment and temporary files; they
exercise local → official Internet → local switching without touching hardware.
