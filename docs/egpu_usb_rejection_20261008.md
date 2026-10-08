# eGPU USB transfer failure incorrectly rejected a model

The October 8 live inspection found carrot-mdm2 at be76615d with the selected
e20cde17 model and runtime installed. The preceding boot's load failure ended
in `bulk OUT 0x02 failed: Input/Output Error`, during AMD firmware upload over
the USB bridge, before trained-model inference. The failure classifier did not
recognize this transport error and persisted a `rejected` marker. The next boot
therefore refused the same artifact and ran the internal model. Branch selection
and completed download alone could not clear the rejection. An initial DNS
failure was retried; the persistent rejection was the later startup blocker.

Recognize this exact USB error in the existing transient-initialization helper.
This preserves the existing bounded loader attempts and internal fallback after
failure, while retaining diagnostics without permanently rejecting the model.
It does not classify arbitrary file I/O errors or model/shape/checkpoint failures
as retryable and does not establish the physical cause of the USB error.

For already affected installations, delivery may retry a rejected artifact only
with a matching pickle hash, rejected load/boot-validation diagnostic, this exact
USB error and a nonempty boot ID different from the current boot. Missing or
malformed evidence, same-boot failure and inference rejection remain blocked.
Recovery invalidates any validation receipt, performs ordinary catalog and file
integrity checks, and clears the marker only after successful installation.
The original failure report stays intact; normal device validation/loading still
must succeed. No onroad hot swap or automatic reboot is introduced.

114 focused desktop tests pass, including bounded failure/recovery attempts,
serialized worker errors, previous-boot migration, corruption repair, retained
permanent rejection, delivery, startup selection and validation receipt behavior.
Ruff passes. Local diagnostic captures remain private under `.analysis/`.
Device recovery and the underlying USB issue require separate live verification.

Docs-Not-Needed: Internal artifact rejection/retry correction; no setting change
or user guide work requested.
