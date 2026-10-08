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

After explicit approval, the parked/disengaged C4 was fast-forwarded from
be76615d to 699f9424 and rebooted through manager. Startup recovered the matching
previous-boot rejection, reverified artifacts and passed the GPU smoke test.
Changing the shared helper also triggered internal-model recompilation; normal
model output resumed roughly five minutes after reboot. The active eGPU worker
used the e20cde17 PKL and reported checkpoint 870a4823/12864. The retained failure
report belongs to the old boot; no rejected marker or new failure appeared.

A 30.04-second parked observation received 602 model messages (20.04 Hz), zero
invalid model/pose/DM updates and zero model frame-ID gaps or reported drops.
Model execution time averaged 40.02 ms, maximum 42.70 ms; aggregate core7 usage
was 67.2%. These are current parked measurements, not a matched before/after CPU
comparison or driving validation. The underlying physical USB-error cause is
still unresolved. Linux MDM2 CI passed 163 contract/retry tests, one native CPU
runtime test and 11 Web tests with generated assets unchanged.

Docs-Not-Needed: Internal artifact rejection/retry correction; no setting change
or user guide work requested.
