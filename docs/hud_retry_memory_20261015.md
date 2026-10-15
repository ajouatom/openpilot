# HUD retry memory retention, 2026-10-15

The user's C4 was still on `93160a93`, 13 commits behind the published NAV
branch. Carrot Web's actual failed Git Pull jobs named two manually installed,
untracked eGPU helpers. Both matched their incoming Git blobs exactly. They
were backed up under `/data/local/tmp/git-pull-repair-20261015` before repair;
unrelated untracked build/model files must not be cleaned.

The failed pull also reset tracked trial changes. Its running HUD logged
`S3 display is closed` followed by `asynchronous USB JPEG pipeline failed`.
The published renderer-ready Wi-Fi connection fix was absent from this checkout.

## Memory evidence

Private previous-session qlogs show memory usage reaching 99%. Between segment
5 and 27, the same HUD PID 208998 grew from 168.62 to 528.55 MiB RSS, with a
181.89 MiB PSS increase. Loggerd RSS rose 25.55 MiB but PSS only 3.19 MiB over
those selected endpoints. RSS contains shared pages and cannot be summed to
explain all system consumption. Available RAM was as low as 65 MiB at a sampled
segment-20 endpoint. These data establish HUD growth as a major contributor,
not a complete attribution of all memory or a proven cause of reboot.

After reconnection, a new boot initially reported 84% with approximately 533 MiB
available. P, zero speed and disabled controls were verified. Pstore later
confirmed a previous-boot kernel BUG in `dump_throttled_rt_tasks`, with the
interrupted stack in `shrink_slab`/`try_to_free_pages`/`filemap_fault`. The task
named by RT throttling was radarcan. No OOM-killer entry was found; this does not
prove HUD caused the panic or establish a radarcan allocation leak.
The existing radar arena fix was already in the old vehicle commit.

## Reproduced retention and correction

`freeze_gc_after_init()` permanently excluded the HUD session's cyclic objects
from collection. Autorun reuses the process after failure. A JPEG worker retains
its failure traceback, which references its frozen pipeline and frame data.
Ordinary `gc.collect()` cannot reclaim this frozen cycle.

Using the actual AsyncJpegUsbPipeline with a disconnected transport, four old
sessions remained alive after close plus collection. Unfreeze plus collection
released all four. The fix pairs each successful freeze with an unfreeze and
restores prior GC thresholds at the very start of `run_demo`'s finally block,
including errors and normal exit. Active-frame freeze/threshold behavior stays
unchanged; no new collection is inserted in the rendering loop. Cleanup failures
cannot skip the unfreeze. The next initialization collection can reclaim the
previous session after its stack unwinds.

Validation: 27 focused desktop Wi-Fi/lifecycle tests pass. The new test repeats
12 actual JPEG-worker failure sessions and asserts all weak references are
released and thresholds restored. The same 12-cycle test passes in an isolated
C4 Python process using the candidate helper, without changing driving services.
Changed main source adds no lint findings; existing unrelated findings remain.
This is a reproduced Python retention fix, not proof that every native graphics
allocation is bounded.

## Installed verification and remaining problems

Actual Carrot Web Git Pull job `849abc7c4050` completed successfully after a
fresh parked/inactive guard and a bounded stop of manager-owned processes.
Vehicle advanced from `93160a93` to NAV `0cf297c3`, then rebooted. Web status
reported zero ahead/behind and a clean tracked tree. Model and DM each published
1,200 consecutive valid frames in 60 seconds, approximately 20 Hz, with eGPU
active. A subsequent three-minute sample measured 82--86% system usage. HUD PSS
repeatedly returned to approximately 110--126 MiB after failures rather than
retaining every previous Python session. This short test is not a long-term
memory clearance or proof that the system is back to its earlier baseline.

S3 still authenticates but times out awaiting frame responses. Known S3
transport errors now report one bounded line with the underlying exception and
the existing five-second retry. Unexpected encoder/programming failures keep
their traceback. Authentication, transport timeouts, queue bounds and retry
intervals are unchanged. Eight diagnostic regression cases plus the existing
27 Wi-Fi/lifecycle tests pass.

Further native inspection found a separate remaining allocation path: the
HUD PID's ION orphan-labelled buffers rose from 216 objects / 59,719,680 bytes
to 222 / 61,378,560 bytes over a retry interval. The label alone does not prove
an unreachable allocation; repeated growth and actual release must be checked.
This is outside ordinary Python RSS/PSS accounting. Current system usage still
reaches 87%; historical comparison and native-buffer cleanup remain under
investigation. Do not report the overall memory problem or S3 timeout as solved.

Raw qlogs, process snapshots, tmux excerpts and diagnostic scripts remain private
in `.analysis/scratch/2026-10-15-egpu-s3/` and are not included in firmware releases.
