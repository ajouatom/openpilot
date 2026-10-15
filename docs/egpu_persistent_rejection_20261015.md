# eGPU remains blocked after reboot

## Findings

The reported Ioniq 5 PE's private NAS evidence now establishes the cause of the
persistent block. Ten diagnostics were copied and SHA256-verified. October 14
22:09/22:12 captures show eGPU active. October 15 09:42 captures show an inference
failure from code 734634c983; subsequent 09:48/09:51/10:05 captures on 5a33dc40ed
retain the identical rejected failure, with no successful new load.

The terminal error is `RuntimeError: AMD failed with 1` during input upload in
`GenericModelRuntime.run`. The selected runtime directory is 90970e01f2ba70d5.
Inspection of the SHA256-verified runtime archive
90970e01f2ba70d553294c0f9d5c6fe1678827f750705cab679a7b19b7bcb0b2
shows `tinygrad/runtime/support/usb.py:457` maps AMD.error_state to the USB link
error word; `usb_reap` at lines 312-317 latches the asynchronous transfer status.
The same archive's `autogen/libusb.py:204` defines status 1 as
LIBUSB_TRANSFER_ERROR. The existing classifier accepted only -1, so this verified
USB error was incorrectly treated as an artifact failure and persisted forever.
The exact recorded traceback fails the old classifier and passes the correction.

The initial captured device wall clock was stale: the October 15 09:42 log has
an embedded October 14 21:05 clock. Do not use that stored time as the actual
incident time. Within that snapshot, the error precedes the later logged USB
unplug; the unplug is not proof of the initial cause. Cable, power, bridge and
host causes are not distinguished by this evidence. The failing code predates
the October 15 response-deadline/wait-yield changes.

The numerical-rejection issue below was found independently during the initial
code review; it is not the cause of this reporter's USB +1 block.

Comparison of current remote branches found a concrete recovery gap:
`carrot-wip` 5a33dc40ed lacks `retry_output_rejection`, which carrot-nav already
has from dabe4f1d67 (October 14). Wip's worker combines shape/non-finite checks
into `ValueError: invalid precompiled model output`. Its persisted rejection
survives a reboot because delivery only migrates recognized USB/PSP failures.
Four positive replay cases stay rejected on the unmodified wip checkout; five
negative cases remain blocked as intended. The migrated helper passes all nine.

Both branches also offer Restart & compile on a persisted rejection. That API
only writes a waiting status and requests a reboot; it neither repairs the
failure nor clears the marker. This can repeatedly show the same card.

The October 15 response-deadline change classifies its TimeoutError as transient;
that path does not itself write a persistent rejection. This does not establish which physical or runtime condition caused USB status 1. The prior web
map-preview change does not modify the eGPU loader or failure policy.

## Changes and limits

- Both branches now recognize exactly -1 and +1 for the verified runtime hash.
  Other runtimes/codes and quoted/nonterminal errors remain rejected. The existing
  new-boot migration revalidates old +1 rejections. New +1 faults stop the failed
  worker without permanently rejecting the artifact; existing bounded recovery,
  internal-model fallback and handover checks remain in force. No runtime binary,
  inference deadline, GPU model or USB error suppression is changed.
- Port only nav's numerical-rejection migration into wip. Keep the separate
  branch models, runtimes and per-frame output policies; do not port nav's
  single-frame rollback implementation with this repair.
- Require matching artifact identity and previous, nonempty boot identity.
  Retry recognized inference-output failures only on a later boot. Rehash/repair
  artifacts and discard the old smoke-test receipt before eligibility returns.
- Explicit count/checkpoint/checksum errors, boot-validation failures, unknown
  failures and missing evidence remain blocked. The old combined error cannot
  distinguish shape from numbers, so its legacy migration still requires fresh
  artifact checks and a new smoke test; it does not certify the model as safe.
- Both web branches hide/deny the ineffective compile-restart action for a
  blocked artifact. Add Download diagnostics for error cards. The local endpoint
  exports only the selected installed artifact's saved failure fields, including
  the original earlier-boot error, with a bounded read and no cache. It excludes
  unrelated JSON fields and does not publish/upload the report anywhere.

## Verification

Nav: 169 model/progress/rejection/output-recovery tests and 32 API tests pass.
The Windows API runner bypasses only the unrelated eager feature initializer
that imports native msgq; the tested API code and services load normally.
13 web tests pass and the web bundle builds. Wip migration alone: 116 tests pass,
including the four cases that failed before the fix. Final wip verification: 134 model/progress/rejection tests,
32 API tests and 13 web tests pass. Both shipped tools bundles match their asset
manifest hashes. These are desktop checks, not physical GPU or
road validation. No user's vehicle was restarted, installed or unblocked.

After updating wip, a recognized earlier-boot numerical rejection can be retried
on boot. A remaining block needs the downloaded diagnostic report; repeatedly
rebooting or deleting every rejected marker is not a diagnosis or repair.

## Reporter-specific follow-up verification

175 focused tests pass for USB rejection classification, file revalidation and
bounded background recovery. The exact privately saved traceback also passes a
matching-runtime/later-boot migration probe, rejects a same-boot attempt and
preserves the original failure record. This proves recovery eligibility, not
physical USB stability or successful driving. No NAS raw logs are committed.

## Tmux recovery lifecycle visibility

The user requested explicit error and retry output so future analysis does not
need to infer the recovery path from a generic card. All added events go to
stderr with `[eGPU]`, process ID and monotonic time; they never enter the worker
stdout control protocol. Failures include phase, model/runtime identity,
terminal error, saved diagnostic path and persistent/transient classification.
The full earlier traceback remains in the existing diagnostic report.

Recovery prints fallback, scheduled delay and attempt/limit, actual attempt
start, failure/cancellation, probe READY, handover, and terminal stop reason.
Waiting explains absent USB, internal health or backoff. Unchanged waits are
not repeated; changed waiting reasons are limited to one per five seconds.
The modeld first-output publication path separately prints live_output_resumed:
a successful synthetic probe or handover is not claimed as a live result.
Boot-time artifact rejection/migration also prints the retained cause and next
step. Startup without a recovery manager explicitly reports no retry.

No retry limits, validation gates, inference timeouts, model selection or USB
runtime are changed. A closed diagnostic stream is swallowed, not propagated
into inference.179 focused tests pass on nav before publication; final wip uses
the same tests. Tests check three-attempt exhaustion, delayed handover, absent
USB log volume, old-block cause output and failure of the logging stream.
Syntax checks pass. Targeted lint passes; modeld's existing ISC002/E501/F841
findings are outside these changes. No physical vehicle/reboot test or install.
