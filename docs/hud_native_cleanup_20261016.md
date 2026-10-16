# HUD native graphics retention, 2026-10-16

The remaining retry-related native allocation leak is reproducible without S3,
JPEG encoding or vehicle data. Opening and closing a hidden comma-raylib window
leaves six ION allocations (1,658,880 bytes, about 1.58 MiB) each time on the
investigated C4. Two cycles leave twelve allocations / 3,317,760 bytes. DRM and
descriptor-passing socket handles also accumulate. This is separate from the
Python frozen-object retention corrected on October 15.

## Ownership defect and correction

The installed `comma-deps-raylib 6.0.0.1.post98` binary's `ClosePlatform`
disassembly confirms EGL termination and `gbm_device_destroy`, but no
`gbm_surface_destroy` or DRM descriptor close. Its DRM descriptor handoff also
leaves the one-shot socket open. Stopping the HUD does not reclaim the already
orphaned allocations on this device. The earlier stopped-HUD snapshot retained
510 objects / 141,004,800 bytes (134.47 MiB).

The HUD entry now loads a small native compatibility library before importing
raylib. It tracks raylib-created GBM devices, surfaces and locked front buffers;
after EGL termination it releases buffers and destroys surfaces before the
device. It closes only an owned DRM descriptor and the unique, verified
`/tmp/drmfd.sock` handoff socket. An explicitly supplied `DRM_FD` remains borrowed.
Other GBM callers are not tracked. Tracking storage is bounded and no per-frame
allocation is added. The adapter is local to the HUD process, not the manager or
main UI. The manager preimports raylib for other processes; a forked HUD therefore
inherits an already-bound library. On this condition only, the HUD replaces its
own Python process image once (same PID, explicit HUD module, bounded environment
guard), then loads cleanup before renderer/encoder imports. The manager, UI and
model processes are not reexecuted. Native testing caught this integration issue;
the loader correctly refused the earlier, late installation attempts.

Actual managed shutdown also exposed a second cleanup gap: one of two stops
still retained six buffers despite the adapter. The old code marked the window
owned only after font/model loading, and sequential cleanup could skip window
close when interrupted or when an earlier close failed. Mark ownership as soon
as the context is ready; always close the native window in the renderer's
`finally`. The run-level `finally` now guarantees renderer close and performs it
before potentially blocking USB/network display shutdown. Fault injection at
GC, network, pipeline, renderer and display cleanup verifies this order,
including `KeyboardInterrupt`. The precise interrupt site of the observed
second stop was not recorded; do not present that timing as proven.

Only the SHA256-verified comma binaries below are accepted. A changed or unknown
binary fails before graphics initialization, rather than applying an ownership
workaround to a potentially fixed library. Desktop/non-comma rendering is
unchanged. Native builds include `libcarrot_gbm_cleanup.so` through SCons.

| Package | Native binary SHA256 |
| --- | --- |
| 6.0.0.1.post98 | `b1cc6eb0241081f4fe826f14d80f8a6e6f2b1953440ce6cf5f34c4af5e3660fb` |
| 6.0.0.1.post103 | `ebb0b6c4112a39d700e06fd40947023485d3c14c0d0ac36abef8c6a9551d7502` |

## Validation

All native experiments used fresh Park, standstill, inactive control and panda
permission checks. The managed HUD was stopped during isolated graphics tests.
The native source compiles on C4 with `-Wall -Wextra -Werror`.

| Native experiment | Result after every close |
| --- | --- |
| Unmodified post98, two minimal windows | Six then twelve retained ION objects; handles grow |
| Corrected post98, twelve minimal windows | Zero test-owned ION objects; device/socket count stable |
| Corrected post98, twelve actual HUD render/readback/close cycles | Zero test-owned ION objects; device/socket count stable |
| Corrected post103, twelve minimal windows using an isolated package overlay | Zero test-owned ION objects; device/socket count stable |
| Corrected post98, twelve cycles with caller-owned `DRM_FD` | Zero test-owned ION objects; handles stable; caller descriptor still valid |

The post103 package was downloaded from official PyPI, verified against its
published wheel digest, and tested outside the installed environment. The
vehicle's installed raylib package was not replaced. Counts concern each test
process's allocations, not all historical allocations remaining on the device.

The cleanup loader, existing GC lifecycle, retry logging and Wi-Fi transport
tests pass together: **54 tests**, including inherited-library restart bounds and
cleanup-before-encoder import order. Ruff passes for the new loader/tests and
`cluster_run.py`; the existing autorun file retains its preexisting lint findings.
The Windows environment cannot collect the native-Params autorun test without
`params_pyx`; no complete native autorun-suite pass is claimed.

## Recent increase versus introduction date

October 15 logs show one HUD PID growing from 168.62 to 528.55 MiB RSS and
181.89 MiB PSS growth over segments 5–27. The Python retention and native GBM
retention are demonstrated separate paths. Frequent S3 transport retries expose
both through repeated renderer sessions. S3 support arrived October 14, but
that does not establish when the installed raylib defect first appeared.

Earlier logs already exceed 80% on October 10, while October 11 includes
73–77% runs with HUD and signal processing. The old freeze policy also predates
October. Do not identify a single proven regression date from those facts.

## Vehicle application and remaining limits

The investigated vehicle remains on `e9b33329e4`; no full branch pull or unrelated
model/control installation was performed. The compiled adapter and identical
Python loader were placed in a private directory outside the checkout, with a
backup and hashes. The tracked HUD entry loads that directory, and only the
renderer and run-loop shutdown hunks were applied to the existing HUD sources.
This avoids new untracked repository files obstructing a subsequent Git Pull.
A normal branch update/build uses the committed loader and SCons library instead.

After the shutdown correction, **three actual managed HUD stop/reconnect cycles
pass**. PIDs 322188, 322999 and 324076 each hold six objects / 1,658,880 bytes
while rendering and leave zero of their objects after termination. The final
HUD PID 325071 has the adapter mapped and S3 connected, with `ClusterHud=1`.
All prior orphan allocations remain unchanged across these three stops. Active
HUD PSS snapshots are 185.0, 187.4 and 190.1 MiB; the final new process is
185.7 MiB. These short warmup snapshots are not a long-duration plateau test.
The output is configured for 10fps; actual Wi-Fi refresh fluctuates, and a
separate earlier session still logged a transport disconnect. No stable 10fps
or Wi-Fi root-cause resolution is claimed here.

During the earlier managed-cycle measurement, model/DM produced 1200/1201
consecutive valid frames in 60 seconds, with no missing IDs. Arrival-derived
rates were 19.993/19.996Hz; this is not a strict >=20Hz arrival-rate pass.

The final 60-second measurement after the successful three-cycle sequence has
1200/1200 consecutive valid model/DM frames, no missing IDs, and arrival-derived
rates 20.000010/20.000090Hz. Model and DM PIDs are unchanged. The S3 connected
flag is false at the end of that measurement, so this does not establish an
uninterrupted S3 connection throughout the interval.

Existing orphaned buffers are not reclaimed by installing this fix; they require
a separate C4 reboot, which was not performed. Repeated process
cycling alone is not a memory fix. This correction establishes reclamation of
the demonstrated HUD graphics leak, not clearance of all system memory growth,
the logger's earlier growth, or the physical cause of S3 Wi-Fi disconnections.

Raw process, ION, network and vehicle evidence remains private.
