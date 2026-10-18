# Memory and Wi-Fi HUD isolation, 2026-10-19

The user's C4 again reached approximately 90% with ignition on and Park. The
user also reports 92% the previous night, 88% after driving without rebooting,
and about 90% on arrival. Those reports remain valid observations; the retained
qlogs do not cover every instant, especially offroad time.

This investigation finds an unnecessary **legacy signal-policy comparison
worker** in the eGPU standby path. Removing it returns substantial headroom.
It does **not** establish that this worker leaks, explain every historical
80-to-90% increase, or fix S3 connectivity.

## Controlled exclusions on the current boot

The vehicle runs `2d71d318` on one boot throughout these measurements. Fresh
Park, standstill, zero speed and inactive controls were checked before each
setting change. Model PID 354496, DM 354500 and eGPU worker 354577 were retained.
There was no vehicle reboot, model restart, router change or firmware flash.

| Step | Available memory | Interpretation |
| --- | --- | --- |
| Before stopping HUD | 352.90 MiB / 90.22% used | Active HUD PSS 181.79 MiB |
| Normal HUD exit | 572.64 MiB / 84.12% used | Approximately 220 MiB returned; includes normal working memory |
| Later HUD-off snapshot | 604.80 MiB / 83.23% used | Initial 20-minute observer had exited, stderr empty |
| Before `SignalAssistMode=0` | 582.83 MiB | Actual setting was mode 2 |
| Recognition process exits | 625.76 MiB | Its PSS was 58.18 MiB; separate comparison worker remains |
| Before stopping legacy comparison | 635.71 MiB | Legacy `/data/signal-model-shadow/enabled` was `1` |
| Legacy comparison terminates | 753.00 MiB / 79.12% used | Approximately 117 MiB returned; worker PSS was 104.34 MiB |

These snapshots include different transient diagnostic/allocator states. Do
not sum each difference as an exact leak attribution. Stopping HUD removed
its current six ION buffers (1,658,880 bytes) and GPU owner. Remaining orphan
ION was 7,413,760 bytes, unchanged when signal processes stopped.

The HUD-off observer collected 236 samples over 1,195.62 seconds. Settled early
and later windows had median available memory 606.06 and 602.25 MiB. This is
not an untouched 20-minute workload: marked windows include log transfer,
read-only probes and a temporary pre-encoded sender. It does not prove an
hours-long plateau. A separate bounded observation follows with HUD and both
signal processes disabled and with RAM-backed file allocation included.

## Confirmed unnecessary signal worker

`ModelState` constructed `ShadowClient.optional(...)` for every internal model,
including the ready eGPU's standby model. A previously enabled experimental
policy comparison therefore loaded a separate ONNX runtime even though the
standby received no live inference. Temporarily suppressing its capture during
warmup did not undo the earlier child creation. `SignalAssistMode=0` controls
the current recognition service, not this independent legacy flag.

The live worker was PID 354780, parent modeld 354496. Its identity and parent
were checked before termination. Its original enable file was backed up under
`/data/carrot/diagnostics`; the flag was set to zero to prevent respawn at the
next normal startup. Only this experimental child was terminated. It briefly
remains a zombie owned by modeld, without its former address space; modeld was
not restarted just to reap that process.

The code correction adds an explicit constructor gate: a standby built for an
already loaded eGPU does not create the comparison worker. The internal model,
warmup, recurrent reset and fallback remain intact. Optional comparison for an
internal model that starts active retains its previous opt-in behavior. There
is no new artifact validation, process launch or observer startup on the live
fallback path. Current signal recognition/control is independent of this gate.

This is a footprint correction for devices with the legacy comparison flag
and matching artifact. It is **not** an explanation for all C3/eGPU reports:
those devices may never have enabled or installed this comparison.

## Historical attribution and its limits

Re-examined the existing 1,279 deduplicated retained route/segment summaries
and the original October 11 high-memory qlog, rather than inferring dates from
the user's recollection. The earliest retained >80% sample remains October 10
route 108e on `cb45d99b61`. October 11 route 1095 nevertheless remains 73-77%
with HUD and signal processes. A single introduction date is not established.

October 11 route 109c, segment 17, contains 89-91% usage on `64b2a63c`.
Its largest process is a multiprocessing child **of signalcolord**, PID 54119,
with PSS 377.4 MiB. The separate legacy policy worker is another 102.7 MiB.
The parent/manager relationship is present in the original procLog and
managerState. This is before the repository's S3 Wi-Fi bridge commit
`81d2c83c6a` dated October 14. The old cluster source also lacks the Wi-Fi
transport. That rules out the later committed Wi-Fi service as the sole cause
of every historical high-memory event; it does not prove which old experiment
or signal backend caused that child's footprint or an increasing leak.

The current mode-2 recognition service is approximately 58-60 MiB, not the
old 377 MiB child. The current boot's early route has both recognition and
legacy comparison already near their later sizes. They raise baseline usage,
but their current measured footprints do not explain all overnight growth.

For the latest boot, 150 retained closed qlogs were collected/read across
routes ec through f0 (28 selected initially, then 122 additional files).
The recorded maxima are 84%, 86%, 90%, 91%, and 90% respectively. This does not
invalidate the user's 92% reading outside the retained samples.

The full retained early window (ec segments 20-79) and late window (f0 segments
5-15, before this investigation's HUD stop) have median available memory
613.00 and 413.04 MiB: approximately 200 MiB less headroom. Model/DM/eGPU RSS is broadly stable; HUD active RSS
medians are approximately 203.46 and 204.65 MiB. Current proclogd PSS stays
around 12.4 MiB, without the earlier approximately 38 MiB shared-to-private
jump. PSS medians rise 18.44 MiB for navi, 10.17 for athena, 9.27 for web,
6.59 for HUD and 5.09 for UI. System shared memory increases 127.38 to
206.86 MiB. PSS is
dated and sharing changes across process lifetimes; RSS/PSS deltas cannot be
blindly added together.

## RAM-backed logs and recent common-code audit

On this device `/dev/shm`, `/run`, `/var/log`, and `/tmp` use tmpfs. A final
inventory found 109.21 MiB in `/dev/shm`, 60.42 MiB in `/run/log/journal`,
20.38 MiB in `/var/log`, and 10.34 MiB in `/tmp`. These are allocated blocks,
not just apparent file lengths. Message queues account for much of `/dev/shm`
and have fixed-size rings. The current log allocation is not proof that all
approximately 79 MiB of historical Shmem growth came from logs: no matching
early per-directory inventory exists.

Retained current-boot journal records include 41,363 Bluetooth unknown-ACL
messages, 13,726 hardware heartbeat messages, 7,765 privileged Wi-Fi profile
reads and 15,911 sudo session records. Repeated profile reads and the heartbeat
predate the recent S3 feature in retained history. The heartbeat may participate
in the external watchdog contract: it was not removed. No journal/syslog was
deleted or vacuumed and no OS logging limit was changed to conceal growth.

The bounded diagnostic observer now also records allocations for those four
fixed roots, plus both signal enable states. It is an explicitly run diagnostic
tool, not a new always-on production service. Its existing 2-hour/32-MiB cap,
low scheduling priority, single-observer lock and low-headroom exit remain.

Reviewed October changes in proclogd metadata/PSS collection, eGPU warmup,
recovery and progress records, native UI geometry/text caches, camera fence
waits, navigation rasterization and web media queues. The inspected queues and
caches have bounds; eGPU progress is a fixed mmap record and standby warmup is
three startup inferences. Source review is not proof that all native libraries
or every lifecycle is leak-free.

The installed Cython text and ribbon routines were exercised in an isolated
low-priority native process: 60,000 unique labels plus 60,000 ribbon clips,
using validated ABI structures and no-op draw callbacks. RSS reached 22,132 KiB
at 40,000 iterations and stayed there through 60,000; cache stayed at 256
entries/1,280 glyphs and cleared to zero entries. This covers CPU-side
allocation and eviction, **not actual GPU rendering or long road operation**.

## Same-payload transport comparisons

The actual renderer was stopped. Pre-encoded four-second H.264 loops were sent
at a 10fps target, four 15-second phases each. S3 acknowledged every phase with
no transport exception. The detailed clip has a maximum access unit of 84,776
bytes. These are sender/ack rates, not optical panel FPS.

| Sender / feature state | Simple 64KiB | Detailed 64KiB | Detailed 8KiB | Simple 64KiB repeat |
| --- | ---: | ---: | ---: | ---: |
| Wired PC | 10.00 | 9.97 | 9.91 | 10.00 |
| C4, HUD off, signals on | 9.09 | 9.71 | 8.17 | 8.82 |
| C4, HUD and signals off | 9.93 | 9.81 | 7.82 | 9.62 |

S3-reported USB chunk times stayed around 11-12 ms average and at most about
63 ms in the 64KiB phases. Smaller chunks did not improve the C4 path. No chunk
size, bitrate or timeout policy was changed on this evidence. Radio conditions
and CPU load differ between consecutive phases; this is not a controlled proof
that freeing memory improved Wi-Fi. Earlier connection failures occurred even
before video transmission. Their original cause remains unresolved.

## Validation and deployment boundaries

Desktop checks: 20 proclog tests pass with two Linux-only skips; selected
signal/eGPU lifecycle tests pass (42 NAV, 43 wip). NAV includes a repaired test harness
that had omitted the newly added `log_event` callback; production event logging
was not removed. Tests execute actual constructor/gating AST with fake model
resources, verify no comparison artifact I/O or child construction for standby,
and preserve active internal opt-in behavior. They do not reload the live GPU.

Before the signal exclusions, a clean 60-second reader observed 1,200 valid,
consecutive model and DM publications each, approximately 19.9966 Hz, eGPU
active. That is approximately 20 Hz, not a strict >=20 Hz timing guarantee.
The separate all-off observation completed 120 samples over 599.19 seconds.
The largest positive PSS change in a surviving production process was about
0.1 MiB between its first/last inventories. Journal allocation rose 3,704 KiB and
syslog-directory allocation 188 KiB; total Shmem rose 4.52 MiB. A temporary
35 MiB restore/wait helper and marked stream/probe windows affect total
headroom, so the raw endpoint difference is not a production leak measurement.

After that observation, fresh Park/inactive guards allowed restoring the
original `SignalAssistMode=2`, then `ClusterHud=1`. The legacy comparison flag
stays zero. Recognition alone returned to 81.40% used; with HUD active the
60/90-second snapshots were 87.20/87.22% and connected. The snapshot helper's
own footprint is included. HUD PSS was 177.9/178.7 MiB, with six current ION
buffers and no increase in the historical orphan pool. A later light status
read saw the S3 connection flag false again: the link is not declared fixed.

A following 180-second clean publication reader saw 3,600 valid consecutive
model/DM frames each, at 20.00088/20.00138 Hz, same model/DM PIDs and eGPU
active. Available memory ranged 414.93-573.47 MiB (about 84.1-88.5% used).
Only one of 36 five-second samples had the S3 connected flag true, with
discovery failures and peer-closed TCP retries in tmux. Thus these disconnects
also occur below the user's 90% memory-warning condition. This does not exclude
memory pressure aggravating other incidents. The current comparison uses the
shared router network, not a controlled phone-hotspot trial.

Two further bounded, isolated actual-HUD traces used the installed renderer and
native H.264 encoder, stopping/restoring only the managed HUD under the same
Park guard. The first ended early with a peer disconnect; its 300-event trace
filled before the exception and cannot establish the exact closing trigger.
The second retained errors and slow calls and reached its 90-second configured
limit without an exception. Responses nevertheless took up to 1.43 seconds.
Graphics setup preceded TCP open; initial GC took about 0.21 seconds and the
largest startup inter-command gap was about 1.19 seconds. These samples do not
establish a two-second startup-idle failure or justify changing firmware
timeouts. Temporary instrumentation changed no production files; HUD was
restored to 1 after both traces. Intermittent success is not a connectivity fix.

A new low-priority observer (PID 408329) then started on the same boot with
HUD=1, recognition mode=2 and legacy comparison disabled. Its private output
is `memory-restored-20261019-1792364242067270330.jsonl` under
`/data/carrot/diagnostics`. It is capped at two hours/32 MiB, has the existing
single-observer lock and low-headroom exit, and includes RAM-backed allocation
inventories. This capture is **pending**, not a completed stability result.
The new model constructor gate is published for normal update/startup; it was
not installed into or used to restart live modeld. The vehicle mitigation in
this session is the disabled legacy flag and terminated experimental worker.

Raw qlogs, command output, process/allocator inventories and pairing material
remain private in `.analysis/scratch/2026-10-19-hud-isolation/`. No private
credentials, firmware NVS or full recordings are part of these changes.
