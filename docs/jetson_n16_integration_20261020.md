# N16 ONNX selection for the owner's Jetson

The owner requests reviewing and applying the model from
[tjdgus200's branch](https://github.com/tjdgus200/openpilot/commits/codex/jetson-nav-runtime-port/).
The reviewed head is `cf036fa5783793cfa39d9efd2869039a35337d42`.
The scoped model change is
[a1d723a6b4](https://github.com/tjdgus200/openpilot/commit/a1d723a6b4db051070c0eab5ffceb59c982998e4).
This request supersedes the earlier decision to defer the latest Jetson model.

## Identity and scope

The current carrot-nav eGPU PKL and the official Jetson ONNX identify the same
N16 `96244b1f/12800` checkpoint/export family. They are different artifacts and
backends; byte equality, learned-weight equality and output equivalence between
the PKL and TensorRT execution have not been independently established.

| Artifact | Bytes | SHA-256 |
| --- | ---: | --- |
| Existing eGPU PKL | 822,465,506 | `52205044e136efb64a6a841c9b016cf35b1db81186dd0f941b62eee5ebdcde24` |
| New Jetson ONNX | 814,591,107 | `01effc679eb46d6036ce0302673a87fe2dcfcc95a679daeeb929679e53c3647a` |

The ONNX was downloaded and hashed from the
[official fixed HF revision](https://huggingface.co/commaai/openpilot_driving_models/tree/eed9e515dccd5aa9be969a2d6c57eb469174a97d/96244b1f-6c4e-4adc-848d-eccd0b44b182/12800).
Its actual embedded checkpoint ends in `96244b1f-6c4e-4adc-848d-eccd0b44b182`,
without the directory's `/12800` suffix. The client spec preserves that value.
Only the hash, byte count and checkpoint differ from the old Jetson spec.
All 11 inputs, six outputs, five state pairs, output slices and frame skip match.

The integration selects that ONNX and updates its displayed name and regression
expectations. It retains our existing host recurrence, TensorRT graph patch,
USB inference, signed NAS updater and Wi-Fi-to-Jetson/local-USB-panel topology.
The contributor's GPU-state/AIO experiments, alternate USB release/trust policy,
battery display and other unrelated branch changes are not part of this port.
eGPU/internal/DM models are unchanged. carrot-wip retains its separate runtime;
the model selection applies to carrot-nav.

## Independent checks and signed package

Full ONNX checking passes on the original and our patched graph. ONNX Runtime
1.23.2 CPU execution through our actual state queues compares 140 continuous
synthetic frames plus eight reset/replay frames. All six outputs are finite and
exactly equal between graphs, and the reset replay matches. Images include
black, white, gray and seeded random inputs; map/navigation presence alternates.
This is graph-transform/queue evidence, not PKL/TensorRT or road equivalence.
The contributor's separate CPU/TensorRT comparison reports nonzero precision
differences and no road acceptance threshold; this integration does not convert
that report into a perception or closed-loop validation claim.

Desktop checks: 91 passed, one skip. Linux
[CI39366620777](https://github.com/ajouatom/openpilot/actions/runs/39366620777)
passes 94 root updater cases, 845 broad cases (55 skips), 119 navigation/cluster
cases and 11 real protected-storage cases; counts overlap. Windows installer
checks and eGPU compatibility CI39366620916 pass.

Our signed runtime is `abac23f644877876285a57631674ade5e901f77e`,
64,670,484 bytes, SHA-256
`b4e1a7bde4b7f6850821668f051f269d1333b9209deb83bb36dcfee83e6f4f10`.
[Manifest](https://upload.shind0.synology.me/models/jetlink-host-abac23f644877876285a57631674ade5e901f77e/manifest.json).
The official ONNX is mirrored alongside the existing n16 PKL on NAS; the PKL
manifest is unchanged. Existing signing trust and rollback checks are retained.
## Owner-device activation

Fresh valid/live Park, standstill, zero speed, disabled/inactive selfdrive and
carControl, and all pandas disallowing controls were checked before each
temporary offroad transition. The existing startup hold/onroad-cycle mechanism
was used with separate bounded restoration watchdogs; onroad flags were not
forged. The comma tracked tree was clean, existing model-selection files were
backed up, and source was advanced to `5a0e8158c2`.

Direct Jetson-to-NAS download timed out. The already verified public artifacts
were transferred through the dedicated WLAN SSH path to the normal cache,
hashed again on Jetson, and staged by the unchanged signature-verifying updater.
The normal Jetson reboot gate built a new hash-specific TensorRT plan and its
actual import/three-frame finite-inference probe printed
`CANDIDATE_PROBE_OK 01effc679eb46d6036ce0302673a87fe2dcfcc95a679daeeb929679e53c3647a`.
Runtime `abac23f644` became active; the OS remained in protected read-only mode.
No firmware, JetPack or trust-root changes were made.

The comma manager was restarted to clear its old preimported model contract,
preserving its interpreter, environment and CPU affinity. Returning onroad
before the fresh client finished model preparation correctly deferred setup.
A second guarded offroad interval completed it. During the repeated discovery,
the existing optional cable DHCP cleanup encountered a disappearing
`/proc/<pid>/stat` and latched a cleanup block. An offroad daemon restart
revalidated cleanup and recovered. This operation does not claim to fix that
existing teardown race or to validate the contributor's broader recovery port.

N16 then reported active/ready with its exact new ONNX hash, and HUD connected.
Both temporary snapshot holds were restored to their original absent values;
ordinary onroad operation and ClusterHud=1 were restored. The prior release
and old model cache remain available. Final observation results follow below.


## Final parked observation

After warmup, 300.004 seconds of actual-camera observation recorded:

| Stream | Valid frames | Missing/nonconsecutive IDs | Producer rate | Largest publication interval |
| --- | ---: | ---: | ---: | ---: |
| modelV2 | 6,000/6,000 | 0 | 19.99946879 Hz | 69.448 ms |
| driverStateV2 | 6,000/6,000 | 0 | 19.99926818 Hz | 65.601 ms |

N16 remained active in all 61 model samples. The direct AP remained
active; sender connections were present in 61/61 samples and
Jetson receiver connections in 60/60 samples over its separate
300-second observation. The final USB renderer log reports
10.0--10.0 Hz and actual Park/vehicle data; this is renderer
reporting, not optical panel-frame measurement. Sampled Jetson temperature
reached 64.718 C. Final onroad, N16 active and HUD-connected states were
confirmed, with both temporary holds absent.

These are approximately 20 Hz streams; decimal values below 20 must not be
reported as a strict >=20 Hz pass, and individual intervals exceed 50 ms.
No actual map-media frames were received in this observation. This is bounded
Park/inactive validation, not road accuracy, PKL/TensorRT numerical equivalence,
closed-loop response, RF fault recovery or long-term thermal qualification.
The earlier deployment retries remain documented above. Raw captures and
connection details remain private. No full SD image or Windows package was
rebuilt; the signed runtime/model and the owner's installed system are updated.
