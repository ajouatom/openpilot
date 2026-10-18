# Memory follow-up on the user's C4

The user reports continued low-memory warnings and asks whether the newly added
Wi-Fi HUD service or contemporaneous eGPU changes also affect devices without
an S3. After an initial comparison search, the user explicitly redirects the
investigation to their own C4. **The earlier 92% event is still not completely
attributed. No new memory-leak fix is claimed here.**

The vehicle remains on boot `85ba6f34-a1d6-48ea-a773-d61519263a19`, base
`087f862d81`, with the selective HUD recovery-delay change documented in
[the connection follow-up](s3_disconnect_followup_20261018.md). Model, driver
monitor and eGPU worker PIDs remain 246277, 246285 and 246357. No model, control,
router or firmware change, or vehicle reboot, is part of this memory analysis.

## Does unused Wi-Fi HUD run on every device?

`ClusterHud` defaults to zero. Manager's `enable_cluster_hud` starts the HUD
process only when it equals one. Manager does preimport Python process modules,
but `cluster_autorun` defers Wi-Fi transport and graphics imports to function
execution. There is no independent always-running S3 transport daemon in the
reviewed code. Enabled wired HUD with no pairing configuration takes its wired
path; this is distinct from having HUD disabled entirely.

Nine isolated native import trials compare the source before Wi-Fi support,
the initial Wi-Fi commit `81d2c83c6a`, and current source (three trials each).
Dependencies are preloaded, source is compiled/executed in a retained namespace,
and audit hooks record socket/subprocess activity. Before versus initial Wi-Fi
PSS increments are approximately 1.75 versus 1.74 MiB; current source is about
2.07 MiB. All trials create zero sockets, descriptors, threads or new modules.
These numbers include diagnostic source compilation and allocator effects;
they are not exact manager bytecode-import costs. They support the absence of
an automatically running Wi-Fi sender, not a proof about every unrelated change.

The October 15 eGPU change `fec410988b` additionally executes the already-created
internal standby model during startup, preserving rapid fallback. This may
affect initialized memory headroom. Its incremental native allocation has not
been measured in an isolated before/after test and it is **not an established
cause of continuing growth**. It was not disabled speculatively.

## The user's recorded increase is real

Reparsed 129 retained qlogs across six user routes, with source hashes stored
privately. On the current boot/base, the warmed evening route `10ea` reaches
82–83%, while the next morning route `10eb` reaches 88–89%. Selected late
system samples show approximately 648 MiB available in the evening and 437 MiB
in the morning. This is not a matched-input experiment, but it is also not
evidence that the user's warning was imaginary.

The evening and morning eGPU worker RSS values are about 143 and 144 MiB;
modeld about 271 and 273 MiB. Those processes restart with ignition. Such small
resident differences cannot by themselves explain the entire system difference.
Process PSS in qlogs is sampled less often than RSS; zero or stale PSS must not
be interpreted as a simultaneous physical-memory total.

Separately, the older `10e6` route contains 93–95% samples while the subsequently
withdrawn locationd experiment was being compiled: a `clang++` process reaches
about 201 MiB RSS / 212 MiB sampled PSS. This diagnostic workload is a confounder
for those particular peaks. It does **not** explain away the user's earlier
92% report. The withdrawn implementation remains withdrawn.

## Same-boot live observations

Two whole-system snapshots 1,429 seconds apart show modeld PSS +0.074 MiB and
eGPU worker +0.036 MiB; web server +6.49 MiB and navigation receiver +4.93 MiB.
HUD was restarted between these snapshots during the separate connection fix,
and diagnostic/download workloads also changed. The increase in available
memory between these endpoints is therefore not a clean leak comparison.

A subsequent 600-second observation makes no service changes. It records 21
samples, including real S3 disconnect/retry activity in the same HUD PID 263721.

| Measurement | Result |
| --- | --- |
| Available memory, first / last | 392.77 / 392.74 MiB |
| Available memory range | 356.31–430.43 MiB |
| Derived usage range | 88.07–90.12% |
| Modeld PSS change | +0.096 MiB |
| eGPU worker PSS change | +0.121 MiB |
| Driver monitor PSS change | +0.008 MiB |
| Web server PSS change | −0.018 MiB |
| Navigation receiver PSS change | +0.011 MiB |
| HUD inactive retry PSS | 83.75–84.01 MiB; 34 descriptors |
| HUD ordinary active samples | approximately 182–186 MiB; about 104 descriptors |
| HUD transition maximum | 194.27 MiB |
| Logger PSS range | 60.89–113.08 MiB, repeatedly falls again |

HUD's first and last samples are different lifecycle phases. Subtracting them
would incorrectly label about 94 MiB as retained growth. Inactive samples at
0/150/300/540 seconds return to about 84 MiB; the current renderer's six ION
objects also disappear again. Orphan ION alternates between the old 4,980,736
bytes and 6,639,616 bytes while the current session is active. The original
old-buffer total does not accumulate with these observed retries.

An overlapping five-minute GPU inventory has unchanged model (609 allocations,
187.953 MiB), DM (402, 167.730 MiB) and main UI (436, 336.945 MiB) inventories.
These sizes also match the previous day's recorded inventories. HUD allocations
appear/disappear with retries and reach about 151 MiB while active. DMA buffers
vary with session state. GPU/imported buffers overlap process mappings: do not
add these inventories to PSS as disjoint physical bytes.

These observations do not reproduce cumulative leakage in this ten-minute
interval. They also leave little headroom and do not justify calling the
overall condition normal, fixed, or safe for indefinite operation. The missing
evidence is an uninterrupted allocation history through the next warning and
its recovery, including kernel and GPU state.

## Bounded event capture

`tools/memory_analysis/capture_device_memory.py` is a diagnostic tool, not an
automatically enabled production service. It records system memory every five
seconds, process PSS/RSS and GPU/ION inventories normally once a minute, and
every fifteen seconds below 9% headroom. It uses cores 0–3 and low scheduling
priority, never changes vehicle settings, and stops after at most two hours,
32 MiB of output, or less than 128 MiB available memory. A lock prevents two
instances in the same output directory; exclusive creation protects old records.
Output files are mode 0600. Command arguments and pairing/network credentials
are not recorded.

Native validation reads 82–85 resident processes and all five GPU owners. A
snapshot takes approximately 4–6 seconds wall time, so this diagnostic is not
free of overhead; its own memory and timing are captured. A bounded observer
was started at about 09:13 KST, PID 273288, with private output under
`/data/carrot/diagnostics/`. Initial resident footprint is about 14 MiB. The
launcher initially checked the file before the first detailed record finished;
it reported an assertion failure. A follow-up verified the **existing single
observer**, its growing records and empty error file; no duplicate was started.

Update: the two-hour observation completed normally. The measured growth and
subsequent fixes are recorded in [the process logger follow-up](proclog_memory_20261018.md).
Completion is not a stability pass. Raw logs
and observer output remain private. The initial NAS sample did not identify the
specific C3/eGPU reporter, and cannot establish fleet-wide causation.

With that observer active, a separate guarded 60-second check receives 1,200
valid consecutive model frames and 1,199 valid consecutive DM frames, with no
missing IDs. Measured rates are 19.99856 and 19.99491 Hz respectively: about
20 Hz, **not** a strict minimum-20-Hz pass. Original model/DM/eGPU PIDs remain
unchanged and HUD/eGPU flags are active at the end. Syntax compilation and the
native snapshot/observer checks passed; there is no claim of a full test suite
or a completed two-hour result.
