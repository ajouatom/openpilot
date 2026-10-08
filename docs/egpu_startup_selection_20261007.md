# eGPU model selection before startup, October 7, 2026

The user clarified that the fallback model means the comma internal model,
as used while waiting for Jetson, not a previous external model. They selected
waiting for the new external model's download and verification during reboot
before starting normal operation. Apply this common lifecycle fix to carrot-wip
and carrot-mdm while preserving each branch's pinned model/runtime.

## Cause and behavior

The prior launcher consulted persistent active state before building, then
started the downloader in the background alongside manager. Changing a branch
did not invalidate that persisted external-model selection. In the observed
MDM transition, the Cinque v3 worker started at20:17:22 and MDM installation
finished at20:18:56. That worker kept its original loaded model. A subsequent
restart did execute MDM; installing files was not proof of active inference.

With an eGPU detected during startup, the launcher now synchronously delivers
the selected model and runtime under the existing delivery lock, before model
build invalidation, build-time validation and manager. Existing verified files
are reused. Transient network/clock/server failures retain the existing30-second
retry loop; startup waits and the spinner shows progress or network waiting.
Permanent delivery failures select the internal model for that boot via an
inherited startup-failure gate. A later background install cannot lift that gate.

Execution/build selection independently checks the source-pinned model hash,
size and filename against the installed state. A different cached external
model is rejected without a network request. Explicit custom catalog overrides
require matching recorded catalog provenance; delivery establishes provenance
even when the bytes were already cached. The old ONNX previous-model fallback
is removed. Prior files can remain stored without being eligible for execution.

When the expected package is ready, normal existing GPU validation/loading
and its failure policy still apply. No onroad model hot swap or automatic
reboot is added. An absent eGPU does not block startup; background delivery
for remembered hardware remains, and modeld still requires the selected model
on a later start. Jetson's internal-model waiting/handover policy is unchanged.
This does not supply the unavailable MDM ONNX or change the Jetson checkpoint.

## Validation

144 focused desktop tests passed, including stale-cache rejection, explicit
catalog changes, source/runtime retry and resume, permanent/rejected failures,
foreground CLI completion, no-eGPU startup, shell call ordering, cache/build
validation and existing eGPU initialization fallback. Windows runs used the
bundled tinygrad tree, existing libusb DLL and Git Bash. Ruff and bash syntax
checks passed. A dedicated Linux CI job covers selection/delivery/startup order;
the existing full build suite also includes the new model-selection tests.

These are software lifecycle tests, not physical reboot, download interruption,
GPU inference or driving validation of this change. No vehicle deployment or
reboot is performed by this task. Earlier parked MDM execution predates this fix.

Docs-Not-Needed: Internal model delivery and selection lifecycle correction;
no setting or user-operated workflow is added or changed.
