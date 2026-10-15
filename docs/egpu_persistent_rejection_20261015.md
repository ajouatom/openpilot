# eGPU remains blocked after reboot

## Findings

The supplied Mountain Dew card explicitly reports `rejected`, not a fresh USB
or timeout diagnosis. The reporter says it worked yesterday and restarting or
rebooting did not help. Their device identity, exact commit and saved failure
have not been supplied, so the first failure cause remains unknown.

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
that path does not itself write a persistent rejection. This does not rule out
other recent changes or identify this user's original failure. The prior web
map-preview change does not modify the eGPU loader or failure policy.

## Changes and limits

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
