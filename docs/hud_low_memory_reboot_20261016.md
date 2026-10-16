# Low-memory warning after the long-running HUD investigation

The user reported a live low-memory warning after leaving the vehicle running.
The same C4 boot had approximately 14.5 hours of uptime. Fresh Park, standstill
and inactive-control checks passed before any intervention. No full branch pull,
model replacement or control-code installation was performed.

## Captured state and mitigation

| State | Memory used (MemAvailable basis) | Available |
| --- | ---: | ---: |
| Warning, HUD running | 93.29% | 241.89 MiB |
| HUD stopped, first sample | 90.04% | 359.32 MiB |
| HUD stopped, later sample | 88.68% | 408.40 MiB |
| After C4 reboot, HUD still stopped, uptime 173s | 74.75% | 910.88 MiB |

The device reported 75% after reboot. The production low-memory event is emitted
when the device's rounded memory usage exceeds 90%; its threshold was not changed.
No kernel OOM kill was found in the captured kernel log.

The active HUD had 216.88 MiB PSS before stopping. Its six current graphics
allocations were reclaimed on stop. The historical 510 allocations / 134.47 MiB
from the earlier HUD PID were still present, together with earlier probe and
other allocations. They disappeared on reboot. This confirms that installing
the cleanup correction did not retroactively reclaim already-orphaned buffers.

The comparable HUD-off snapshot from 9,837.6 seconds earlier had 432.70 MiB
available: a 24.29 MiB decrease over approximately 2h44m. The largest positive
PSS differences among surviving PIDs were 1.88 MiB for one system Python
process, 1.66 MiB for loggerd and 1.30 MiB for proclogd. This comparison does not
show a new hundreds-of-MiB increase in those surviving processes, but does not
cover every lifetime or prove all allocation paths leak-free.

Reboot recovered more than the known 134.47 MiB of old HUD allocations. Between
the HUD-off pre/post snapshots, anonymous pages fell 193.26 MiB, shared memory
fell 97.87 MiB and unreclaimable slab fell 13.67 MiB; cache state also changed.
These categories overlap other accounting and cannot be summed as independent
leaks. Do not attribute the full 502.48 MiB availability recovery solely to GBM.

## Restart safeguards and follow-up

Diagnostics were saved locally and privately on C4 before reboot. The selectively
installed HUD adapter, entry and shutdown changes were hash-verified. Automatic
Git Pull was disabled already, no finalized staged update was present, and fresh
Park/control checks were repeated immediately before reboot. C4 returned with a
new boot ID and unchanged `e9b33329e4` HEAD; eGPU was active without startup failure.

The first postboot 60-second model/DM sample contained 1199/1200 consecutive valid
frames with no missing IDs. Arrival-derived rates were 19.9836/19.9852Hz; do not
call this a strict >=20Hz pass. The original HUD setting is restored only with
adequate available memory and a bounded observation that stops HUD if headroom
drops below 11%.

The 180-second HUD-restoration observation completed without invoking that stop.
Available memory fell during initialization from 901.12 MiB to 667.87 MiB at
40 seconds, then ranged from 635.28 to 661.82 MiB over seconds 50–180, ending at
639.38 MiB (about 82.3% used). Every sampled connected flag from second 40 onward
was true, with the same HUD PID and six current ION allocations throughout.
Typical descriptor count was 73 (one sample 75). HUD PSS rose from 188.00 MiB at
50 seconds to 191.19 MiB at 180 seconds, so this is not proof of a long-duration
flat memory plateau. No historical HUD orphan PIDs reappeared.

The final live log reports host refresh 9.4–10.0Hz, not optical panel FPS.
The simultaneous 60-second model/DM measurement has 1202/1202 valid consecutive
frames, no missing IDs, arrival-derived 20.0340/20.0323Hz, eGPU active and S3
connected. `ClusterHud=1` is restored at completion. This is a bounded recovered
session, not a resolution of every earlier Wi-Fi fault.

The immediate warning recovery is distinct from long-duration validation and
from the unresolved intermittent S3 Wi-Fi transport problem. Raw process,
network, kernel and vehicle records remain private.
