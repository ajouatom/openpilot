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

## Historical comparison requested by the user

Read 925 distinct locally retained compressed logs; 916 contain memory samples
from the user's device identity (908 also identify Ioniq 5 PE, eight omit
carParams). Combine them with surviving on-device qlogs and deduplicate matching
route/segment observations. This is retained evidence, not every historical
drive. Many old on-device files are truncated; keep their parse errors and do
not infer an entire minute from a fragment. Dates below come from initData and
the corresponding archived collection, not from an inferred panic timestamp.

The earliest **observed** >80% sample is October 10 route 108e segment 4 on
`cb45d99b61`. It is not the date of a proven code regression: the same revision's
earlier 108c run stayed around 74--76%, and a later October 11 run remained
73--77% with both HUD and signal processes running.

Representative complete-session observations, excluding segments 0--4:

| Date / route suffix | Observed segments | Memory range | Median of segment medians |
| --- | --- | --- | --- |
| Oct 10 / 108e | 5--25 | 79--81% | 80% |
| Oct 11 / 1095 | 5--70 | 73--77% | 75% |
| Oct 12 / 109f | 5--10 | 81--83% | 82% |
| Oct 14 / 10bc | 5--72 | 73--83% | 78% |
| Oct 15 / 10ce | 22--115 available | 82--87% | 86% |
| Oct 15 / 10d3, before fix | 5--27 | 88--99% | 95% |
| Oct 15 / 10d4, before fix | 5--20 | 83--94% | 87% |
| Oct 15 / 10d5, GC fix | 5--22 captured | 83--89% | 86% |

September 25--27 retained samples were generally in the 60s, but HUD PSS was
only approximately 11 MiB (waiting, not rendering), with different model/backend
configuration. They are not a like-for-like target for the current active HUD.
October 11's 1095 is a stronger demonstration that active HUD and signal support
can coexist below 80%; it does not isolate which later configuration accounts
for every additional byte.

In long earlier sessions, DM PSS rises during the first approximately 20 minutes
then flattens near 156 MiB. Loggerd repeatedly moves between approximately
86--113 MiB rather than accumulating without return. Current values near
151 MiB DM and 87--113 MiB loggerd fit those historical patterns; a pair of
short endpoints alone should not have been interpreted as proof of a leak.
UI remains near 230 MiB, and the fixed radar is near 9 MiB. The previous
runaway HUD is the clearly demonstrated large same-process change today.

An additional live native check is not normal: HUD PID 53363 accumulated
324 ION orphan-labelled objects / 89,579,520 bytes. They remained after that
process was killed and a new manager-owned HUD PID appeared. The new PID then
accumulated 60 objects / 16,588,800 bytes. Thus process recycling alone is **not**
a demonstrated fix for these buffers. The installed raylib wheel reports
`comma-deps-raylib 6.0.0.1.post98`; do not assume the older checked-in build.sh
source pin describes that binary. Native buffer ownership/cleanup is unresolved.

After the logging update's actual Web Git Pull, the existing manager continued
forking the already imported autorun code. A HUD-only restart therefore did not
activate its new exception reporter, despite the correct on-disk HEAD. Do not
claim the new log output verified until the manager is freshly loaded.

Another live 60-second check during this investigation has 1,200 consecutive
valid model and 1,200 DM frames, 20.004/20.000 Hz respectively, unchanged model/DM
PIDs and eGPU active. S3 is disconnected in that final snapshot. Model timing
is currently healthy; retained native buffers and repeated S3 failure mean the
overall state is **not** certified normal or restored to its former memory level.

### Final activation check

After preserving the preceding evidence, Web job `4732670e92a9` installed
`cfbc50fb9a`. Another 554 fresh parked/inactive checks preceded an authorized
reboot. The fresh manager now emits the actual expected-failure line
`S3 Wi-Fi disconnected (TimeoutError: timed out); retrying in 5s`, without the
three chained tracebacks. Both maintained branches contain the reporter fix;
its 35 focused checks passed on each branch.

At uptime 253 seconds, memory was 81%, MemAvailable 701,008 KiB (684.6 MiB).
The old PID's retained allocations disappeared across reboot, but the new HUD
PID already had 24 orphan-labelled objects / 6,635,520 bytes after four failed
sessions. This confirms recurrence; reboot is recovery, not a native-leak fix.
The postboot 60-second model/DM observation had 1,199 consecutive valid frames
each, 19.974 Hz, with no missing frame IDs or eGPU fallback. Report that measured
rate exactly rather than claiming a strictly >=20 Hz postboot pass. S3 still
times out, so healthy end-to-end display operation is not established.
