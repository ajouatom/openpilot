# Follow-up: memory while the vehicle remains powered

The user asked to keep investigating after a reported value above 92%, rather
than treating the earlier short retry check as sufficient. The vehicle stayed
on the same boot (`c399d61d-a5bf-437e-9ddb-c5068f5394fd`) and base commit
`2ef833b201`, with the two selective HUD fixes documented in
[the retry investigation](hud_memory_retry_20261017.md). No new production code,
firmware, router setting, model or control change was installed in this follow-up.

## Read-only observation before intervention

A 15-minute recorder sampled total memory and every resident process every ten
seconds, with process PSS and ION details once per minute. Only its first
580 seconds (59 samples) precede the first diagnostic HUD stop. The later rows
include isolated tests and must not be described as an uninterrupted 15-minute
production run.

In the untouched interval, the same HUD PID repeatedly tried to reach S3.
None of the ten-second samples reported a connected display, so this phase
tests retry reclamation, not continuous network video.

| Observation | Result |
| --- | --- |
| HUD RSS, including initialization | 87.15--194.10 MiB |
| HUD PSS after failed sessions | 75.55--75.74 MiB |
| First versus last comparable inactive HUD PSS | +0.082 MiB |
| Inactive HUD descriptors | 34--35 |
| Available system memory | 354.70--472.08 MiB |
| Historical orphan ION allocation | 4,980,736 bytes, unchanged after each retry |

One sample during renderer initialization includes six additional GBM objects
(1,658,880 bytes); subsequent samples return to the original orphan total.
This is the distinction between seeing active graphics allocations and proving
that they remain after close. No surviving process has a hundreds-of-MiB PSS
increase in this interval. The largest positive comparable change is logmessaged
at about 0.50 MiB; that small observation alone is not a leak diagnosis.

Separate GPU inventory snapshots about 66.7 minutes apart show unchanged size
and count for the main UI, model and driver-monitoring allocations. The main
UI GPU inventory is about 336.95 MiB, model 187.95 MiB and DM 167.73 MiB.
These GPU inventories overlap mapped/imported memory and must not be added
blindly to process PSS as if they were disjoint physical allocations.

## Continuous rendering and encoding, isolated from S3 loss

Fresh Park, zero-speed, inactive controls and panda permission checks precede
each HUD stop. The diagnostic stops only managed HUD, runs the actual live-input
renderer, road camera and native H.264 encoder at the configured 10 Hz, and
discards encoded packets at a test display endpoint. It preserves the real
bounded encoder queue and memory policy. S3 network and physical display
performance are explicitly **not** exercised by this test. Diagnostic callbacks
check the parked condition throughout and stop if available memory falls below
300 MiB. They do not change the production warning threshold.

Two setup attempts ended early (a diagnostic class-name error and a stale guard
subscription after initialization), both restoring the HUD setting. They are
not counted as continuous passes. After correcting only the private diagnostic,
the completed run produced 5,985 H.264 access-unit diagnostic counts. At the
601-second active sample it had observed 5,943 encoder packets and 6,528 chunks.
Collection itself adds occasional HUD pauses; these are not an optical FPS or
hard real-time benchmark.

| Time / condition | HUD PSS |
| --- | ---: |
| First rendered frame | 167.24 MiB |
| 30 seconds | 179.98 MiB |
| 120 seconds | 182.47 MiB |
| 300 seconds | 184.66 MiB |
| 601 seconds | 184.26 MiB |
| Last five minutes, sampled range | 183.81--184.98 MiB |
| After normal close | 106.61 MiB |
| After post-session collection and heap return | 83.66 MiB |

The text measurement cache reaches its 1,024-entry limit and is cleared:
992 entries at 420 seconds, then 88 at 450 seconds. Python object counts also
fall across that event. Stroked-text textures remain at nine entries (113,488
pixel bytes). GPU inventories 140 seconds apart retain exactly 240 allocations
and 158,851,072 bytes (151.49 MiB). Open descriptors fluctuate around 104--106
with a transient 110, rather than increasing each frame. After close, the test
display weak reference is gone and no ION rows remain owned by the test PID.

The early rise is real. These observations support bounded initialization/cache
growth followed by a short plateau; they do not justify calling every earlier
rise a cache, or proving that no slower or input-dependent leak exists. In this
run, continuing growth to 92% was not reproduced, and there is no newly
demonstrated production leak to patch speculatively.

## Historical comparison and limits

Older retained logs do contain a genuine large HUD increase: route `10d3` on
`93160a931c` spans roughly 154--465 MiB PSS with a 95% median system usage. Those
old retention paths were the subject of the earlier fixes; this is not evidence
that the user's warnings were imaginary. Today's pre-fix HUD reaches about
220 MiB, while current observed retries return to about 75.6 MiB PSS.

The older route with a 75% median has a different process configuration (no
precompiled eGPU worker in its recorded process snapshots). That comparison
cannot by itself assign the entire difference to a new leak or to eGPU. Total
headroom remains limited. The user's 92% event is not fully attributed, and
neither moving/driving inputs nor hours of uninterrupted real S3 streaming were
validated here. Raw diagnostics and logs remain private.

After the diagnostic process exited, HUD was restored to 1 and the original
model/DM PIDs were unchanged. A separate 60.0018-second observation, without
concurrent heavy memory scans, received 1,200 valid consecutive model frames at
20.0012 Hz and 1,200 valid consecutive DM frames at 20.0069 Hz. No frame IDs
were missing and eGPU remained active. Final available memory was 453.42 MiB
(87.43% used), HUD RSS 80.96 MiB, and S3 was disconnected. These are bounded
observations, not a guarantee of a future rate or a successful S3 streaming pass.
