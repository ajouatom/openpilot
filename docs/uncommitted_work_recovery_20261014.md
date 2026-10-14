# Recovery of uncommitted completed work, 2026-10-14

The primary checkout was on carrot-nav at 7dda85fbfd, with no unpublished
commits on that branch but with completed changes still outside Git history.
The remote carrot-wip tip was 0cd29a0d1c. Its local branch checkout was five
commits behind the remote, rather than five commits awaiting publication.

## Recovered changes and conversations

- October 3, "Check onroad nnff loaded message": retry the one-shot NNFF
  startup notice after delayed initialization or alert contention; subsequently
  expose the existing default-OFF NNFF option through Carrot Web search only.
  Preserve NNFFLite, existing model precedence and next-onroad application.
- October 3, "Analyze 9초 steering warning": delay only non-critical
  temporary-steering SOFT_DISABLE/WARNING presentation for 12 completed
  100 Hz control cycles. Accumulate repeated faults across short clear gaps;
  reset after 100 clear cycles. Keep raw lateral inhibition, event transport,
  no-entry, critical alerts and state-machine deadlines unchanged.
- September 30 through October 10: 24 untracked engineering reports and one
  tracked steering-report follow-up, covering vehicle incidents, radar/native
  CPU investigations, Niro stopping experiments and signal-model analysis.
  The steering-warning report belongs with its implementation commit. These
  reports describe their historical scope and limits; publication does not
  mean the experimental approaches became production changes.
- Pending repository-memory updates preserve the steering analysis, existing
  core7 UI decision and previously authorized Wiki publication policy.

The user was shown the three functional changes before committing or pushing.
Task commits are separated into NNFF, temporary-steering presentation, and
engineering-record recovery, then applied to both maintained branches without
merge commits or changes to their distinct model/navigation selections.

## Deleted failed experiment

At the user's explicit request, delete the unpublished parked-blinker safety
extension, host tool, dedicated tests and all five integration blocks. The
normal Hyundai CAN-FD safety header matches the pre-cleanup committed version.
Since the experiment was never tracked, its deletion creates no source-file
deletion in the new commit. The retained October 2 report is explicitly marked
historical and retired. No vehicle action is performed.

## Current verification

- 171 focused tests pass using the existing Windows native-import adapters:
  NNFF/readiness/update notices, startup, impact handling, temporary-steering
  presentation, state-machine behavior and lateral control gates.
- The AlertManager duration test passes in its own process, keeping its shared
  event mutation separate from fixed-duration tests.
- 47 settings-schema tests and 25 Wiki-generator tests pass (244 total).
- Generated Wiki Markdown/content validation and the Korean/English user-docs
  mapping check pass. The existing workflow synchronizes Wiki after a
  carrot-wip catalog push.
- Ruff passes for the changed selfdrived code/tests. The two modified Wiki
  tests have the same 22 existing lint findings as their committed versions.
- Existing NumPy shape deprecation warnings remain. No native vehicle build,
  physical warning/audio test or road validation is claimed.

Raw logs, recordings, local conversation archives and validation artifacts
remain private and excluded from publication.
