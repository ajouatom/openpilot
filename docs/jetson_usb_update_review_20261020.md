# Jetson USB update review, 2026-10-20

The user requested reviewing contributor commit
[52a8ffee9fdce860137b512303bc45cee6d53bca](https://github.com/tjdgus200/openpilot/commit/52a8ffee9fdce860137b512303bc45cee6d53bca)
and its [receiver release](https://github.com/tjdgus200/openpilot/releases/tag/jetson-usb-sha256-bootstrap-20261020-52a8ffee).
The user subsequently explicitly deferred the newer Jetson model and physical
device work, requesting only this USB update feature.

## Scope and integration

Imported the contributor's single commit on its exact carrot-nav base
c4568c30346bc6412237f3fc1e1cfcf74c9ceb9d, then added three interruption tests
and a Linux root test step. All integration commits have one parent.
This is the carrot-nav update path: carrot-wip lacks the prerequisite local
release catalog and separate Jetson pin. Importing that different runtime or
changing its model selection is outside this request.

The optional transport downloads branch-pinned HTTPS assets on the comma and
sends them over USB. Both ends verify SHA-256 and byte length. Registration is
separate from activation, which retains existing ABI, model, inference-probe,
protected-storage and rollback checks. SHA-256 provides integrity, not author
authentication: the explicitly enrolled receiver trusts the attached comma.
The signed NAS path and official public key remain intact.

No usb_host_sha256_release.json is selected. Existing host_release.json,
jetson_model.json and the eGPU model configuration are unchanged. Installing
source alone does not enroll existing Jetsons, select a package, or update a
device. The receiver requires one-time local enrollment; future package URLs,
hashes and sizes must be pinned by the maintainer after package verification.
See [deployment instructions](../tools/jetlink/USB_SHA256_KO.md).

For the user's existing Jetson, no action is needed now. A comma Git Pull alone
cannot enroll the new USB receiver. When USB delivery is actually chosen, the
receiver needs one local installation; later selected packages can transfer
through the comma while offroad. The existing signed NAS update route does not
require USB enrollment and can still deliver a future signed model/runtime.
No device was contacted to inspect or change its current installation.

## Artifact and regression verification

- Release ZIP: 37,494 bytes, SHA-256
  86c759f5620f8ce29fd91e8a1d9266986e46fa5c4c2dc50be6629bef995d5723.
  All 11 included Python/document/public-key files match the original Git blobs
  exactly. The ZIP was inspected without running its installer.
- Desktop regression: 64 passed, 57 skipped (Linux/root-only facilities).
  Changed-source Ruff and whitespace checks passed.
- Linux root ownership/update subset: 94 passed, no skips. This exercises real
  temporary files, ownership and symlinks with an in-memory USB transport.
  System services, ABI checks and inference probes are test doubles.
- Added regressions cover disconnect cleanup, local-policy revocation before
  registration, and loss of offroad permission after upload before activation.
- The first CI run passed the 94 root checks but failed subsequent compilation
  because root-created bytecode directories were not writable by the runner.
  The root step now disables bytecode and pytest caches. This was a CI setup
  defect, not a device result; final workflow results are recorded below.
- The next broad run exposed two existing HUD regressions: the exported bundle
  omitted lead_brake_lights.py, now imported by the renderer, and a renderer
  fixture created with object.__new__ lacked the new brake display reader.
  Included the existing module and initialized the fixture's real reader.
  Existing exported-entrypoint and remote-camera tests verify those fixes;
  no display/recognition policy was changed. Desktop subset: 6 passed, one
  Linux-only exported-entrypoint skip.
- The following layout-test process also exposed three pre-existing native
  Params dependencies in geometry tests. Explicitly select model-indicator
  display for those tests (119 desktop checks pass). The production Jetson HUD
  already installs its read-only DisplayParams adapter; its runtime was not
  changed to accommodate a test-only environment.

No physical USB transfer, Jetson TensorRT build, power-loss test, vehicle
installation, driving or 20 Hz result is claimed for this change.

## Deferred model question

The current carrot-nav Jetson pin is 80eedcdd/12864 (814,591,106 bytes).
The eGPU PKL uses NAV n16 checkpoint 96244b1f/12800; see the existing
[n16 investigation](egpu_n16_stability_20261015.md).

The official Hugging Face API now lists
[96244b1f/12800/big_driving_supercombo.onnx](https://huggingface.co/commaai/openpilot_driving_models/blob/main/96244b1f-6c4e-4adc-848d-eccd0b44b182/12800/big_driving_supercombo.onnx),
814,591,107 bytes, published LFS SHA-256
01effc679eb46d6036ce0302673a87fe2dcfcc95a679daeeb929679e53c3647a.
This is new availability relative to the earlier PKL-only integration record;
the matching path is not independent TensorRT or numeric-equivalence evidence.
Model adaptation, model-pin changes, runtime/model publication and on-device
validation were deferred at the user's request.

## Final publication checks

Code commit a863f4b7631b8b57d4555ae75a9053b8c63fdeaa passed:

- [Jetson integration CI](https://github.com/ajouatom/openpilot/actions/runs/39357706187):
  94 root ownership/update tests; 835 broad tests with 55 skips; 119 navigation/
  cluster tests; 11 real loop-filesystem protected-storage tests. Windows
  installer disk-selection checks also passed. The root subset executes the
  ownership cases that skip in the non-root broad run; counts overlap.
- [eGPU compatibility CI](https://github.com/ajouatom/openpilot/actions/runs/39357641029):
  success. These are compatibility checks, not physical GPU inference evidence.

Published to carrot-nav. Final documentation records the tested code without
changing its runtime. Existing unrelated generated Web edits and log.log were
preserved. No NAS package, receiver enrollment, live process or device setting
was changed.
