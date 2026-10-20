# Initial Jetson connection status and locationd follow-up

The owner identified the red **upper-right comma Jetson badge**, rather than
the USB panel or a central driving alert. Before the first working session,
an unanswered HELLO and the intentional boot-gate-to-runtime handoff reached
the generic connection exception handler. It published `retrying`, which the
badge presented as `ERROR`. This is a real connection attempt that has not yet
succeeded, but does not by itself establish a failed model or broken device.
The available drive logs do not preserve every initial badge transition, so
not every red indication reported by the owner is attributed to this path.

## Change and delivery

The normal boot handoff now has a distinct exception and handler: it reports
`connecting` and logs progress without a failure traceback. A physical
disconnect during release checking remains a connection error.

Before the daemon has ever reached a warmed, ready session, an initial
timeout or FunctionFS's explicit “no reader” write timeout reports connecting
for at most 120 seconds per initial attachment. A timeout after that budget,
after HELLO/model preparation begins, or after a ready session remains an
error. Signature/model/protocol validation, unsafe cleanup, other USB errors
and runtime failures are not reclassified. The transport still abandons and
cleans up failed attempts; its deadlines, protocol fallback and retry cadence
are unchanged. Error detail remains in diagnostics/logs.

The existing badge reads **연결 대기** while connecting and **모델 준비 중**
while preparing, using its existing yellow progress style. READY and active
green indications are unchanged. Stale reports, blocked cleanup, model faults,
overheating and other actual errors retain their existing warning/error path.
No selfdrived alerts, pose validity, engagement permission or model output
changes are included.

This is comma-side Git Pull plus restart/reboot. Neither a new Jetson image
nor another signed host runtime is needed for this badge correction. The
previous NAV-preview runtime selection remains in carrot-nav; if that update
has not been activated yet, follow its offroad/Internet procedure and reboot
both comma and Jetson. The owner vehicle was read only and still ran
`5a0e8158c269` during this review. No live files, Params, services or caches
were patched.

## locationd: further evidence, not a timing fix

The previously identified invalid odometry is still caused by model input
frame IDs jumping 18030 to 18032. Both road and wide camera-state records
contain frame 18031. Their SOF timestamps differ by only 11,302 ns; the
existing 20 ms pair limit accepts them. Replaying those recorded metadata
through the unchanged selection rule retains all three frames. Camerad's
`sendState` sends the VisionIPC frame before publishing its camera-state
record. This narrows the problem to delivery/consumption/timing after camera
production; it does not identify the exact reader delay or prove a hardware
fault. Do not relax camera pairing or pose validity to make the alert vanish.

One-second timing summaries cannot identify which camera receive/resync or
preceding inference cycle caused this single gap. A bounded `modelCameraGap`
diagnostic now retains only the previous and current cycle, the ordered
main/wide receive frame IDs, SOF timestamps, receive durations, and major cycle
timings. It emits at most once per second, only on gaps; logging failure is
contained. It changes no camera selection, buffer policy, recurrence, model,
deadlines or health flags. New evidence requires ordinary update activation
and another occurrence. **The locationd timing failure is not fixed or
road-validated by this diagnostic.**

## Validation

Nav: 122 desktop focused tests passed. Wip: 120 passed, preserving its separate
model and signed-manifest selection. These include initial timeout bounds,
real-error precedence, actual boot-gate handoff, existing startup alert
contracts, recorded camera metadata, trace bounds and logger failure.
Linux daemon/transport coverage runs in the existing integration workflow.

The actual C4 badge draw method rendered all five states with the production
Korean font in a hidden desktop Raylib context; text fit and color were
visually checked. This is a synthetic desktop capture, not a vehicle screen
verification. In a separate low-priority native C4 process, 10,000 fake camera
pairs cost 4.11 us/pair without tracing versus 15.14 us/pair including trace
and cycle retention. This measures helper CPU overhead only, not real camera,
GPU/USB latency or road performance. Raw drive/device evidence stays private.


Completed integration checks on the published source:

- [Nav CI 39389864670](https://github.com/ajouatom/openpilot/actions/runs/39389864670)
  on `b85a5f1101`: 94 root updater cases, 923 broad cases (56 skips),
  119 navigation/cluster cases, 11 real storage and 11 NAT cases passed.
- [Wip CI 39389916455](https://github.com/ajouatom/openpilot/actions/runs/39389916455)
  on `10d48e66a4`: 735 broad cases (2 skips), 119 navigation/cluster and
  11 real storage cases passed. Both Windows installer jobs passed.

Counts overlap. Physical initial-connect appearance and the next camera-gap
trace remain unverified until the user's normal Git Pull/restart activation.
