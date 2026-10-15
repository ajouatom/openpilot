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
available. P, zero speed and disabled controls were verified. Current kernel
output had no OOM entry; previous-boot OOM/panic attribution remains unverified.
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
allocation is bounded. Full updated live memory/rate checks follow installation.

Raw qlogs, process snapshots, tmux excerpts and diagnostic scripts remain private
in `.analysis/scratch/2026-10-15-egpu-s3/` and are not included in firmware releases.
