# Cinque v3 integration and upstream review, 2026-09-19

The user requested full integration of `carrot-cinque_v3` (9653269dcd) into
`carrot-wip` (1130b07462), plus the new official driver-monitoring model and
applicable recent upstream changes. The merge preserves both histories.
The subsequent explicit retirement request removes the local and remote v3
branches after validation; `carrot-wip` is the sole maintained top-level branch.

## Included

- Pinned Cinque v3 eGPU artifact e758b96d and its matching isolated tinygrad
  d5e17c93 runtime. NAS download/verification, generic state queues, camera warp
  caching, fallback and per-stage call timing are included unchanged from v3.
- AGNOS 19.6.3-carrot -> 19.8-carrot-bt1 for both C3 and C3X/C4 manifests.
  Boot/system change; the existing other firmware entries and Carrot patches
  are retained. OS update selection is independent of whether an eGPU exists.
- Native Bluetooth support, web pairing/device mappings, per-device short,
  double/long gestures, cruise/lane actions and their existing interlocks.
- Official [Super Leicht DM #38942](https://github.com/commaai/openpilot/pull/38942),
  merged as 5ae0da0e465a21cb32b07eac0a987c24b2b7945e. Checkpoint
  `a9462a65-1886-462a-8847-ad4624d9abfc/200`; ONNX size 7,844,499 bytes;
  SHA-256 `dee5a294e8afaacc9295ac5d100e00733ecac278e79264b96331e40a3ede1b04`.
  The real ONNX blob remains in Git as in the existing fork, with no LFS
  pointer installation step. Official monitoring policy adds sleep probability
  above 0.75 as a distraction input, retaining confidence gates and alert
  timing. The additive cereal field, replay plot and debug layout are included.

The internal-GPU driving model, repository tinygrad and existing common radar,
camera cadence, vTurn and UI optimizations are unchanged. DM input/output shapes
and slices match the previous model. Existing startup model-tree invalidation
and SCons ONNX dependencies trigger recompilation of the changed DM artifact.

## Recent upstream reviewed

Review head: 5ae0da0e46 (2026-09-19 00:35:52 UTC). The official Git fetch/API was
used because the public web commit page returned an older cached history.

- #38932 is now merged upstream. The existing verified v3 artifact/runtime pair
  remains pinned; newer serialization is not substituted underneath it.
- #38933/#38956 change tinygrad and retargetable/disk-backed artifact loading.
  They require a coordinated rebuild/migration of internal models and runtime
  APIs. They are not required for the new DM ONNX contract and are not pulled
  into this integration.
- #38941 removes upstream model chunking; Carrot's current packaging and model
  loader still use chunks, so this is not an independent compatible fix.
- #38656 expands compile-time CPU affinity; it concerns the newer upstream
  compiler path and does not fix per-frame latency. Existing affinity remains.
- Other recent commits concern Cabana, Prime/pairing UI, replay tools and CI;
  no additional runtime fix required by this integration was identified.

## Validation and limits

The official DM download hash matches the committed upstream LFS identity.
ONNX checker, existing tinygrad metadata/parser and two CPU ONNX Runtime
inferences passed with finite (1, 553) outputs. Focused tests cover the v3
artifact/worker, Bluetooth input, OS manifests, and sleep-only alert/recovery
with both driver positions and confidence gates. The Linux CI builds native
modules and models before running monitoring/cruise/Bluetooth regressions.

The existing C4 OS/model/Bluetooth trial evidence remains in the linked trial
document. PC/CI validation is not a new C3/C3X/C4 on-road or GPU timing result.
This change does not claim to repair the separately diagnosed 9842 vehicle's
MPC extension import failure, nor establish the cause of camera/SPI stalls.

Final integration commit: `a13bba3ca4a306c1c3bdc1ccc0bb5e402e31bee9`.
[Linux CI](https://github.com/ajouatom/openpilot/actions/runs/35433080120)
passed the full build, 221 model/monitoring/cruise/Bluetooth tests, web controls
and startup/update lifecycle tests. User-doc validation passed. The NAS updater
deployed that commit and verified 1,196 replay frames; an independent public
health/page/data check matched the updater's replay hash.

The previously discussed SPI protocol PR #38868 remains open and unmerged at
review (head f0568611). It changes both Panda firmware and host protocol code;
it is not included as a proven SPI-error fix in this integration.

Docs-Not-Needed: No global setting definition or setting semantics change;
Bluetooth is an existing web-only feature with localized in-dialog guidance.

## Automatic AGNOS updates (2026-09-24)

The user requested unattended OS updates on `carrot-wip`. Both C3/tici and
C4/mici startup updater screens now start the update immediately, without an
Install button or a saved approval. The updater downloads and flashes the
inactive slot, verifies it, activates it and reboots automatically on success.
The existing device-specific manifests and required OS version are unchanged.

The startup UI passes `--retry-network`: disconnects, timeouts, interrupted
responses and temporary HTTP failures (408, 429 and 5xx) wait ten seconds
between attempts and resume without a Retry tap, including after prolonged
loss of connectivity. Existing cached-download and HTTP Range handling remain
in use. This is not Wi-Fi-only enforcement; any working internet connection
can supply the update. Both progress screens retain Wi-Fi configuration access
while the update worker runs, and returning to progress does not spawn another
worker. Connectivity HEAD probes never gate the actual download.

Permanent server/TLS errors retain bounded retries and an error screen;
partition-write and integrity failures do not enter the network retry loop.
The existing image verification, bounded flash/swap attempts, updater lock and
startup OS-version gate remain in force. An incomplete required update still
blocks normal openpilot startup. Already verified inactive slots retain the
offline activation/reboot fast path. The offroad background updater retains
its existing metered-network and bounded retry policies.

Validation uses mocked UI/hardware, simulated connectivity recovery after
60 and 600 seconds, cached download/resume tests and CLI verification/swap
tests. All 59 focused updater/startup tests passed, as did Ruff and shell syntax
checks. It does not establish a new on-device flash or C3/C4 reboot result.
