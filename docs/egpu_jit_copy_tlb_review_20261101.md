# eGPU captured copies and Flip Flop TLB load failure, 2026-11-01

## Upstream review and selected integration

[880b40e1926c7ec49ccbfe96e65f291d60211b15](https://github.com/commaai/openpilot/commit/880b40e1926c7ec49ccbfe96e65f291d60211b15)
(PR39187) compiles input and output copy schedules once with CapturedJit and
replays them each frame. It also reinstates the d39227 tinygrad runtime and
898b5cfd Chestnut PKL previously reverted by PR39184, plus its two warp PKLs.
[PR39187](https://github.com/commaai/openpilot/pull/39187) explicitly reports
non-JIT copies becoming 35% slower in the new eager tinygrad and says this change
fixes that regression. This is an upstream measurement, not a local hardware
result. The copy change directly addresses repeated host scheduling costs around the
transfers measured in the preceding Light speed investigation. It does not
establish that all transfer time disappears or resolve a GPU initialization hang.

Apply the exact upstream CapturedJit/compile_linear construction in the generic
external-model adapter, scoped by the verified d39227 runtime archive hash:
`cb9bf2cfa14746485f75b1dd89d8b7ab65011c10c8fc6c4aea4d41b45a47dc51`.
This is currently Light speed. Input views bind to the captured upload buffer;
output copy reads the existing model output storage. Both paths are prepared
before live inference, without running the driving model or advancing history.
The same transfer binding supports the QCOM pre-upload warp adapter and its
existing AMD fallback. Existing recurrent aliases, checkpoint/rollback, numerical
validation and worker deadlines remain intact.

This is a selective adaptation, not a wholesale cherry-pick. Existing N16 and
Flip Flop retain their exact PKLs/runtimes and legacy transfer behavior. The
repository-wide runtime, internal CD210, driver monitoring and Jetson TensorRT
are unchanged. Updating Flip Flop artifacts is not required to apply the copy
optimization to Light speed, whose package already contains d39227. No replacement
weights or incompatible runtime-only substitution is introduced.

## Attached failure

The attachment identifies **Flip Flop f0667cf6**, runtime directory c7c5862265223135
(the matched 02e698 runtime), phase load, rejected=true. Its traceback ends with:

`TimeoutError: flush_tlb timeout. Timed out after 10000 ms, condition not met: 0 != 1`

The loader is transferring the persistent model arena from disk to AMD. USB
controller buffer allocation maps GPU addresses, then the driver requests GPU
translation-cache invalidation and polls its acknowledgement bit. The bit remains
zero for ten seconds. Inference has not started. This is distinct from Light
speed's 50 ms loop-budget problem and its later inference-response timeout.

The traceback establishes a GPU mapping/initialization acknowledgement failure,
not corrupt weights or a model-runtime mismatch. Power, link state, GPU state and
driver behavior remain possible causes; the attachment does not distinguish them.
It contains no device identity or contemporaneous USB/kernel evidence sufficient
to assign this failure to either previously analyzed vehicle.

There is also a concrete Carrot classification defect: a worker TimeoutError
crosses the control pipe as RuntimeError(traceback). The existing classifier
recognizes several terminal driver failures but missed flush_tlb, so it created
a persistent model-hash rejection. The copy optimization occurs later and cannot
repair this load-stage hardware/driver acknowledgement by itself.

## Classification and recovery correction

Recognize only the exact terminal flush_tlb TimeoutError format during load or
boot validation. Preserve the failure record, invalidate the startup receipt,
and classify the error as a transient initialization failure instead of bad
model weights. Existing bounded background preparation retry accepts it; no
retry count, delay, internal-health gate, GPU-owner lock or deadline is relaxed.

For an existing rejected hash, a matching saved load/boot-validation failure
from a different nonempty boot ID may re-enter the normal verified installation
and startup-smoke path. Same-boot rejection, mismatched hashes, missing evidence,
checksum/contract failures, appended terminal errors and unknown failures remain
blocked. This permits a new attempt; it does not fake a successful load or prove
the TLB problem is fixed. No failure file or rejection marker was manually deleted
on a vehicle, and no automatic reboot or power cycle was added.

## Verification and delivery

New regression expectations fail on the original classifier (8 failures, 14
passes in the focused selection), including the serialized error, old-boot
migration and bounded preparation retry. Revised desktop coverage passes 287
checks across transfer dispatch, artifact validation, retry, recovery and both
warp adapters. The native Linux test additionally exercises the pinned d39227
runtime with repeated mutable 3,244,544-byte inputs and 18,452-float outputs,
verifying current data, stable input storage and propagation of non-finite data.
The older pinned runtime's existing native tests remain required.

Normal comma Git Pull and reboot delivers these source changes. No new model
package, NAS artifact or Jetson host bundle is needed. Physical eGPU latency,
USB transfer behavior, TLB recovery and road acceptance remain unvalidated; do
not describe the previous Light speed warning as cured without hardware results.
Unrelated ongoing indicator changes in the worktree are excluded from this task.


Published source: `9bdc257cd07709e47a0b04d3f62270f9fc73d291`.
[eGPU CI41265046121](https://github.com/ajouatom/openpilot/actions/runs/41265046121)
passes 427 contract/delivery/recovery checks, 57 navigation checks, three native
old-runtime checks (one new-runtime-only test skipped), all four Light speed
native checks including captured transfers, and the Web tests/build. The exact
attached traceback also passes local classification with its original text
preserved and no rejection marker. Focused lint and user-doc mapping checks pass;
counts overlap the earlier desktop coverage. These are not AMD USB hardware tests.

[Jetson integration CI41265046147](https://github.com/ajouatom/openpilot/actions/runs/41265046147)
also passes: broad Linux 2137 passed/56 skipped, supporting suites
232/62/94/119/11/11, and the Windows installer check. No Jetson inference or
host bundle deployment is implied by these compatibility checks.
