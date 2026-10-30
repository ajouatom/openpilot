# Light speed external model integration, 2026-10-30

The selectable external catalog adds Light speed to N16 Nav and Flip Flop.
Selection remains frozen for the entire comma boot and is shared by eGPU and
Jetson. N16 remains the default; internal CD210, DM, navigation admission,
indicator behavior and vehicle settings are unchanged. Light speed has no
navigation inputs, so it is labelled NOO not supported.

## Reviewed upstream changes

- [PR39173](https://github.com/commaai/openpilot/pull/39173), reviewed head
  `c7cbe5de6a9dae4bab032bc5faeb24a248c41f8f`, is still open at review time.
  Its current base is `0152aab4a4db0971494847406bd875f938aee598`.
- [0152aab4](https://github.com/commaai/openpilot/commit/0152aab4a4db0971494847406bd875f938aee598)
  updates tinygrad to `d39227beebd0c3a340dc42b2cdc1e3db1c36aed9` and recompiles
  the existing Chestnut driving PKL plus two camera warp PKLs. Its diff changes
  artifact pointers and the runtime submodule, with no vehicle-control source
  changes. A shared checkpoint label alone does not prove numerical equivalence.
- Apply that runtime only to the new Light speed package. Existing N16 and
  Flip Flop PKLs keep their matched runtimes. Do not replace the repository-wide
  tinygrad or internal/DM artifacts merely to add this external model.
- The PR's intermediate separate-history-buffer change was reverted by its
  final recompilation. Final upstream uses aliased input/next-state buffers,
  matching the existing generic eGPU adapter. No state-feedback change is needed.
- Upstream warp PKLs are not substituted into Carrot: its existing camera-layout
  path compiles the camera warp under the selected isolated tinygrad. The driving
  model PKL is downloaded verbatim, never locally recompiled.

## Artifact identity

| Artifact | Size (bytes) | SHA-256 |
| --- | ---: | --- |
| Official Light speed eGPU PKL | 790445184 | `33ba092837e85c20ead465050673d11fbac1dd17dbaebc6c9fd77e64186019a3` |
| Official Light speed ONNX | 792305602 | `6274eb4a3f3eb7ad1ab341d222bbba2ba709b06645dbbd15ac4971d6cb22ab21` |
| tinygrad source archive | 12196809 | `09ef44a6765e000e0f884a787ec38410664b8a4eabbb58cd13f7c7d863176384` |
| Isolated tinygrad runtime | 2960556 | `cb9bf2cfa14746485f75b1dd89d8b7ab65011c10c8fc6c4aea4d41b45a47dc51` |

Checkpoint: `f22e06e5-3b3f-48d2-a6b9-2f32dc20e707/56320/f22b8b34-0983-4185-a453-a87b73798d61/16384`.
Official ONNX source is comma's Hugging Face `openpilot_driving_models` repository,
`f22b8b34-0983-4185-a453-a87b73798d61/16384/big_driving_supercombo.onnx`.
Official PKL is resolved by the PR's Git LFS SHA through comma's `openpilot-lfs`.
Both full downloads match the official sizes and SHA-256. The ONNX checker passes.
Independently extracted ONNX and inert-unpickled PKL metadata agree on checkpoint,
all input/output shapes and parser slices. This is not numerical GPU equivalence.

Vehicles download the new immutable package from the NAS
`models/comma4-big-light-speed-f22b8b34/`. The package builder verifies both
model and source archive hashes and produces a deterministic isolated runtime.
The USB wait/error hooks are enabled only for its exact runtime hash after source
review: USBIface.sleep remains a no-op; USB error_state still references the link
error word and usb_reap still reports failed transfers. Existing deadlines remain.

## Delivery and validation scope

Normal delivery is comma Git Pull and reboot, then selecting Light speed in Carrot
Web and rebooting comma and Jetson while parked with ignition off and internet.
A new signed Jetson bundle pins the exact ONNX under both bootstrap filenames;
`build_host_bundle.py --model-spec` reads the committed catalog spec, leaving the
repository's default N16 pin unchanged. First Jetson use requires engine preparation.

Source editing, artifact preparation and publication do not require a powered
Jetson. Actual TensorRT build, loading and inference checks require it powered on.
No vehicle installation, model selection, restart or CAN action is part of this
change. Physical eGPU/Jetson inference and road behavior remain unvalidated.
Windows cannot execute the new tinygrad's POSIX native CPU backend; Linux CI runs
the real pinned warp, persistent-arena and recurrent recovery checks.

This is a web-only catalog addition, not a new Params setting. Korean/English
Carrot Web guides and localized help are updated; no generated setting Wiki page
is introduced.

Release prepared from source `d66d445b05c99ab27b4a96aa1f904cc3dfe2834c`:
64,838,820-byte signed host bundle. All 1,152 payload files match the committed
source, with only the two explicit bootstrap model-pin substitutions. Signature,
model pairing, NAS copy and full HTTPS bundle hash pass. Both large model files
and the isolated tinygrad runtime also pass full HTTPS hash/size verification.

Desktop checks: 54 focused catalog/contract/bundle/artifact/retry tests pass, two
Linux host-entrypoint tests skipped; the expanded pass found 259 successful tests
and nine Windows `sendmsg` failures in Linux-only protocol tests. These protocol
checks and `fcntl`-dependent app tests are covered by Jetson Linux CI, not claimed
as desktop passes. Web model/status/selection tests: 17 pass; production build,
focused lint and Korean/English user-doc validation pass. Counts overlap.


Final focused desktop run: 260 passed, two Linux-only host-entrypoint skips.
The prior 54-test run overlaps this coverage. GitHub Actions on published
`7e03c7637f4307c9f4e541e9a653a977892575a9`:

- [eGPU compatibility 40996730817](https://github.com/ajouatom/openpilot/actions/runs/40996730817)
  passes: 401 contract/recovery checks, 57 navigation checks, three native CPU
  runtime checks each for N16 and Light speed, and web tests/build.
- [Jetson integration 40996730849](https://github.com/ajouatom/openpilot/actions/runs/40996730849)
  passes: 2028 broad checks/56 skips, supporting suites 232/62/94/119/11/11,
  and the Windows installer check. Counts overlap earlier focused tests.
- [User docs 40996730666](https://github.com/ajouatom/openpilot/actions/runs/40996730666)
  passes. No physical eGPU or Jetson inference was run.
