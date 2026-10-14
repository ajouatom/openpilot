# Elantra radar memory investigation, 2026-10-14

Confirmed a drive-long Cap'n Proto allocation leak in `DPathRadarD.update()`.
The correction gives each radar update a fresh message arena. Radar algorithms,
controller history, fields, validity, polling and control decisions are unchanged.
This fixes a real contributor, not the entire observed system-memory increase.
No vehicle installation or on-device post-fix measurement was performed.

## Uploaded evidence

Full rlogs from route `00000311--3630b19323`, segments 3 and 10, confirm
`HYUNDAI_ELANTRA`, device type `tizi`, and commit
`0cd29a0d1cf491494b1b33a8a6d53d98451b769e`. Both segments have 112 deviceState,
30 procLog, 1,199 radarState and 1,200 modelV2 records. Only these two minutes
are uploaded for this route; the intervening six minutes are not observed.

- Segment 3 SHA256: `3dd17cc9d3920214db2e84f5f8328479a45ec55274c4107cf378dcb8185e8fc0`.
- Segment 10 SHA256: `ae85ca7fbed2eb7cae21003e72d7bdbd2ae093fef78cd0c5e2b52bca981cfe5f`.

The first segment reports 86% memory usage throughout; segment 10 reports
87% once and 88% in 111 samples. MemAvailable falls from 505.84 to 430.97 MiB
between the first segment-3 and final segment-10 process samples, 478.07 seconds
apart. Total memory is 3,607.10 MiB. This is not simply a rising file cache:
Cached falls from 507.04 to 442.08 MiB over these endpoints.

Same-PID/start-time processes, RSS in MiB at those endpoints:

| Process | First | Last | Change |
| --- | ---: | ---: | ---: |
| loggerd | 133.465 | 170.328 | +36.863 |
| dmonitoringmodeld | 227.875 | 237.375 | +9.500 |
| encoderd | 40.832 | 46.344 | +5.512 |
| radard_dpath | 106.227 | 109.535 | +3.309 |
| modeld | 268.441 | 269.727 | +1.285 |
| cluster_autorun | 253.895 | 254.812 | +0.918 |
| UI | 343.613 | 344.512 | +0.898 |

RSS includes shared pages; adding these changes does not account exactly for
MemAvailable. Radar PSS increases 13.952 -> 17.478 MiB, but PSS samples are
cached/asynchronous. Pss_Anon/Pss_Shmem are zero in these logs and cannot identify
private versus shared growth. This is the same limitation noted in the October 2
Sonata investigation; its one-minute radar growth was a clue, not then proof.

## Confirmed radar cause

Commit `70ea8c8893` removed the per-update `RadarState.new_message()` on July 28
as a CPU optimization. A single builder subsequently lived for the whole drive.
Each replacement of variable-length lead lists, and assignment of the incoming
radar-error struct, allocates within its arena. Overwritten storage is not
reclaimed until the arena is destroyed. Even empty lead lists reproduce growth.

`publish()` creates a new Event and copies only the reachable RadarState into it.
Consequently the published messages stay small and functionally correct while
the source arena grows. `total_size` also traverses only reachable objects and
misses the garbage: this regression must inspect all allocated `to_segments()`
bytes or native process memory, not just Python object counts/message size.

The fix restores a fresh builder at the start of each update. It preserves the
existing controller and fills every existing field through the same assignments.
It does not periodically restart radar, erase tracking, change garbage collection,
or impose new limits on visible vehicles.

## Validation

- Regression tests before the fix: all three bounded-arena cases fail (0, 1,
  16 entries per list); field/validity replacement test passes. After the fix,
  all four pass. Real cereal/Cap'n Proto handles serialization; IPC and controller
  outputs are substituted in these focused publisher tests.
- 532 focused publisher, radar-policy and motion-predictor tests pass on Windows.
- Actual recorded-input controller replay: 1,200 + 1,199 frames, with logged
  radar mode 1, corner radar off and zero radar distance delay. Old/fixed
  serialized Events match byte-for-byte on all 2,399 frames. Input arrival is
  reconstructed from log order, not a reproduction of native scheduling.
  Each segment starts fresh because the intervening data is absent.
- Old arena after each replay: 385,816 / 411,696 bytes. Fixed arena ends at
  656 bytes, with maximum 1,448 / 1,376 bytes respectively.
- Separate-process accelerated soak uses 72,000 updates of the actual uploaded
  radar payload sequence, equivalent in frame count to one hour at 20 Hz.
  This is about 5.4 seconds of desktop execution, not an hour-long vehicle test.
  Old arena reaches 24,094,864 bytes while reachable data is only 648 bytes;
  RSS grows 23.02 MiB and private committed memory grows 31.97 MiB.
  Fixed arena ends at 656 bytes; RSS grows 0.086 MiB during warm-up then is flat
  at all six 12,000-frame checkpoints, with zero private-commit growth.
  Both final published Events are 848 bytes. Old/fixed loop totals were
  5.392 / 5.480 seconds; these desktop timings are not a vehicle CPU guarantee.

## Remaining system-memory question

Radar RSS grows only 3.31 MiB while system MemAvailable falls 74.86 MiB.
The largest observed process increase is loggerd. Segment 10 alone adds
23.20 MiB RSS, including a rapid ~20 MiB increase near its beginning, shortly
after log/video segment rotation. Its virtual size also increases 24.49 MiB
during that minute. This is a separate allocation pattern, not explained by
the small reachable radar messages.

Source review finds bounded encoder queues, per-segment writer replacement,
Zstd stream destruction, and audio/video resource cleanup. These are not proof
of leak freedom. The largest decoded event in segment 10 is a 95,856-byte
procLog, so there is no recorded 20 MiB ordinary event explaining that jump;
video packet payloads are not retained in rlog. Shared IPC residency, heap
retention/fragmentation, compression and video allocations cannot be separated
with this procLog alone. No loggerd fix is claimed or applied without a
reproducer. No OOM-specific application message was found; kernel OOM status
is not established by that absence.

The next discriminating evidence is an uninterrupted longer process series plus
per-mapping smaps/private-heap snapshots around loggerd segment rotation, followed
by an allocation profile if private memory continues growing. This task did not
access or modify the vehicle. A post-fix drive is still needed to quantify system
usage; do not describe the 88% issue as completely resolved by the radar fix.

Private scripts, reduced process series, source rlogs and replay/soak results:
`.analysis/scratch/2026-10-14-elantra-memory/`. Raw evidence is not committed.
