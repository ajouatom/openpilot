# C4 memory growth: process logger retention and shared-page copying

## Result and scope

The user's 90% reading is real loss of available memory, not a claim that all
used memory is harmless cache. The completed two-hour capture identifies a
large shared-to-private transition in `proclogd`. Controlled native fork tests
reproduce GC-induced copying and verify a narrowly scoped startup protection.
A separate process-metadata cache defect retains exited PIDs indefinitely and
returns stale arguments after same-name PID reuse; that defect is also fixed.

This does **not** account for the entire roughly 80% after reboot to 90% later
change (about 361 MiB on this device). In particular, the live transition has
no GC callback trace, so GC is a supported mechanism, not a directly recorded
trigger at that exact instant. No vehicle production file, process, setting,
model, or control code was changed during this investigation. Publication is
separate from vehicle activation and long-duration verification.

## Completed uninterrupted capture

The previously pending capture in [the scope follow-up](memory_scope_followup_20261018.md)
completed normally on the same boot and base `087f862d81`, with earlier selective
HUD fixes. It spans 2026-10-18 09:12:55 to 11:12:53 KST (7,197.755 seconds),
1,412 system samples and 118 detailed inventories. The private file is
5,868,405 bytes, SHA-256
`7460a6f4c5f57169a7758dcfc93407dea036d0dbf72ec9817cfe3fff41ad4ea1`.

| Metric | Observation |
| --- | --- |
| Available memory, first / last | 421.45 / 375.51 MiB |
| Minimum available memory | 326.65 MiB |
| Derived usage, first / last / maximum | 88.32 / 89.59 / 90.94% |
| System anonymous pages change | +59.23 MiB |
| Shared memory change | +14.68 MiB |
| Unreclaimable slab change | +1.09 MiB |
| `proclogd` PSS, first / last | 14.64 / 52.81 MiB |
| HUD last-10-min minus first-10-min median PSS | +0.59 MiB |
| Model / DM / eGPU-worker PSS change | -3.69 / -3.92 / -4.13 MiB |
| Observer PSS, first / last | 10.74 / 14.54 MiB |

Model, DM and main UI GPU allocation inventories are identical at both ends.
HUD allocations vary with its active/closing/retry phases, so endpoints are not
comparable: the last HUD PSS is 199.97 MiB in a transition, versus 185.56 MiB at
entry. It repeatedly returns to about 84 MiB when inactive and roughly
185-187 MiB in ordinary active samples. The historical orphan baseline remains
4.75 MiB, with the current active-session inventory adding about 1.58 MiB.
These overlapping GPU/ION inventories must not be added to PSS as disjoint RAM.

The observer itself adds memory and takes approximately 4-6 seconds to gather a
detailed inventory; snapshots are not atomic. It stopped after the time limit.
The HUD-connected flag is false in 260 samples (18.4%), so this is explicitly
not a Wi-Fi stability pass. No production process restart occurred in the capture.

## The live transition

Same `proclogd` PID and start time, adjacent detailed samples:

| KiB | 10:20:17 KST | 10:21:18 KST | Change |
| --- | ---: | ---: | ---: |
| PSS | 15,494 | 53,863 | +38,369 |
| Private dirty | 7,956 | 47,500 | +39,544 |
| RSS | 103,416 | 103,780 | +364 |
| Anonymous mappings | 91,172 | 91,180 | +8 |
| VmData | 151,560 | 151,560 | 0 |

No resident process disappears or appears between these inventories. The
logger's PSS then stays approximately 52.6-52.8 MiB until the capture ends.
This is principally existing mapped memory becoming private, not allocation
of 38 MiB of new process metadata. PSS also depends on sharing; do not equate
its delta to an exact device-wide recovery prediction. Concurrent system
anonymous pages rise 14.39 MiB while available memory falls 13.85 MiB between
these two non-atomic samples, with other allocations changing as well.

The manager preimports Python modules and then forks this daemon. Python GC
can dirty inherited object bookkeeping even when application payloads are not
changing. The [Python GC documentation](https://docs.python.org/3.12/library/gc.html#gc.freeze)
describes this copy-on-write mechanism. This change protects only the child at
entry to `proclogd.main`; it is not the broader parent-before-fork recipe and
cannot undo copies already incurred before entry.

## Fixes and validation

1. Prune metadata to current `(PID, starttime)` identities at each publication.
   Include start time in cache identity even if the process name is unchanged.
   Keep existing process fields, cadence, and paced PSS sampling.
2. Freeze the inherited GC heap at the first line of `proclogd.main`, before
   PubMaster, sampler, message builders or runtime cycles exist. Newly created
   objects remain collectible; automatic GC is not disabled. The parent and
   other managed children are untouched. Freezing later after runtime resources
   are created, or repeatedly freezing the live daemon, is not supported.

Desktop: **20 tests pass, 2 Linux-only tests skip**. Tests cover churn, PID
reuse, actual Cap'n Proto publication, freeze ordering and existing paced
sampling behavior. The public builder PID-reuse regression fails against the
previous source and passes after the correction. A Linux-only fork regression
checks new-cycle collection and preservation of the parent's GC state.

Native C4 isolated validation uses low priority, cores 0-3, a 45-second alarm,
and a minimum 320 MiB available-memory guard. It neither publishes `procLog`
nor changes managed services:

- 2,000 real Cap'n Proto updates are byte-for-byte equal for unchanged process
  identities. Under simulated helper churn, old metadata grows to 2,001 entries;
  fixed metadata stays at the two current processes.
- Reusing the same PID and name with a new start time keeps the old command in
  the old implementation, and refreshes it in the fixed implementation.
- A forked comparison runs the exact fixed `main()` with a diagnostic PubMaster
  callback instead of a real publisher. One full GC increases private dirty
  memory by 13,600 KiB without protection and 1,700 KiB with startup protection.
  Both children collect a newly allocated self-cycle and retain enabled GC.
  Parent freeze count and enabled state are unchanged, including a pre-existing
  permanent generation. This is an isolated heap, not the full manager heap or
  a prediction of an exact 11.6 MiB production saving.
- A separate preloaded real-procfs fork comparison reproduces the same pattern:
  normal GC adds 13,384 KiB private dirty; child-entry freeze plus GC adds
  1,280 KiB. Measurement allocations are included in both.

The C4 does not have `pytest`; attempting the desktop suite in a temporary
folder stops at import without running tests. No package was installed. The
native assertions above passed independently, but are not a full native pytest
suite or an hours-long production verification.

The unbounded metadata behavior is already present at the retained shallow
history boundary `de874f96a8` (2026-07-12). That proves it predates the S3 Wi-Fi
feature in the available history, not its true introduction date. PSS sampling
changed on October 3; this investigation does not establish which change or
workload caused a full GC at the live transition.

## Vehicle state and remaining work

At 11:37:25 KST the device still reports 90%, with 363.09 MiB available and no
active alert in that sample. Original model, DM, eGPU worker, HUD and `proclogd`
PIDs remain. S3 is disconnected at that instant. `proclogd` is still the old
source SHA-256
`9ac21a1b2ebe124ca6c912b18a22a1c325946378506c6ca41f84f2356c344129`.

The fix is published on both maintained branches, not activated in the vehicle.
The existing manager has already imported the old module: writing its file or
merely recycling the child is insufficient. A normal update and fresh manager
start are needed, followed by another matched long-duration observation. Do
not infer success from memory falling immediately after a restart. There is
no new model-rate, road, total-memory clearance or S3 connectivity claim.

The remaining attribution requires capturing the full post-restart warm-up and
later growth with the same configuration, measuring system headroom alongside
process private/PSS memory and GPU/kernel inventories. The overnight 80-to-90%
change is not fully reconstructed by this later two-hour window.
