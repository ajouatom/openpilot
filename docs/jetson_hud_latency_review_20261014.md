# Jetson HUD and Cap'n Proto latency review

Scope: carrot-nav only. The Jetson is disconnected; this review uses existing
source, archived deployment files and previously captured runtime timings.
No vehicle/Jetson installation or new hardware performance result is implied.

## Findings

Inference requests and replies use the Jetlink binary header and tensor arrays,
not Cap'n Proto. The client checks sequence, frame, status, output size and
finite values. Cereal messages are constructed on the comma after parsing the
model output. The previously investigated locationd events followed delayed
camera-frame consumption and explicit cameraOdometry invalidation on a dropped
frame, rather than an established Cap'n Proto decoding failure.

HUD snapshots contain Cap'n Proto events wrapped in Base64/JSON. Encoding and
camera-preview construction already run in a separate low-priority publisher
process, outside the inference USB owner's Python interpreter. Comparing the
seven archived bundle/local Cap'n Proto files finds identical contents after
normalizing line endings; this is not a current live Jetson filesystem audit.

The negotiated Jetson navigation path still sends the prepared HUD snapshot
and a normal media fragment synchronously after submitting inference and
before waiting for its response. A pump-capable Jetson may receive one extra
fragment if admitted within 4 ms. That is an admission budget, not a 4 ms
deadline for an already-started write. The existing transport watchdog applies.
The receiver thread can already be reading the response while the owner sends
display messages; owner handoff can nevertheless wait for these sends.

## Existing measurements

The following are separate previously recorded parked runs, not a controlled
HUD-off/on experiment. All times below are weighted means in milliseconds.

| Run | HUD/media scope | Subsequent response scope | Sum of both scopes | Server total |
| --- | ---: | ---: | ---: | ---: |
| First post-correction, 120 s | 1.625 | 30.115 | 31.739 | 27.931 |
| Longer run, 300 s | 2.573 | 29.228 | 31.801 | 27.916 |
| Navigation run, 120 s | 2.957 | 28.926 | 31.883 | 27.914 |

The navigation run's HUD scope reaches 15.323 ms, but the displayed maximum
and response maximum are not necessarily from the same frame. Larger HUD work
can overlap server execution and reduce the following wait. Therefore neither
subtracting the whole HUD duration from model latency nor adding independent
maxima predicts a HUD-off result. The first/navigation mean HUD difference is
1.332 ms while the combined mean differs by only 0.143 ms; other conditions
also differ, so this is evidence against simplistic accounting, not causal A/B.

The existing USB traces also identify delayed completion-worker scheduling;
see [the locationd/USB investigation](jetson_locationd_latency_20261014.md).
HUD transmission is a candidate contributor, not the established sole cause.
Moving all display writes after the reply can delay the next inference request.

## Diagnostic correction

Record separate snapshot-read, HUD-write, media-read and normal-media-write
durations and payload bytes. Record the extra early-fragment call separately.
New same-frame `usb_exchange_ms` and `submit_to_response_ms` totals preserve
combined latency without adding unrelated aggregate maxima. These scopes
include CPU scheduling and server waits; they are not pure USB bus timings.
Media byte counts cover the normal fragment, not the optional extra fragment.
When the transport exposes reader timings, also record its per-response chunk
preparation, read-wait and handoff maxima. Handoff measures time from a chunk's
arrival until the owner consumes it; it includes owner scheduling and other
work and does not by itself identify HUD as the cause. Transports without
these counters omit them instead of reporting a fabricated zero.

Diagnostics retain the existing once-per-second bounded aggregation. Request,
reply and HUD ordering, payloads, sequence numbers, deadlines, failure handling,
model/state validation, eGPU/internal-model behavior and display settings stay
unchanged. No new sender thread or buffer reuse is introduced. Extra timestamp
and aggregation overhead has not been measured on the disconnected hardware;
this commit does not claim reduced latency or attainment of the 40 ms target.

Focused desktop validation: 23 tests pass, two Linux SEQPACKET tests skip.
Injected clocks verify all navigation/pump combinations, direct combined
durations, empty/nonempty HUD/media messages, unchanged send order/sequence/
bytes and propagation of USB write failure before consuming the next fragment.
Use the Linux integration workflow for the platform-specific checks.

Next native comparison must retain the same model, route, map, camera and
power/clocks, then compare HUD/media transmission off/on/off. Measure output
age, continuous frame IDs, DM/pose health, server GPU/total, combined request/
response, separate display writes and the next request's admission delay.
Separating those variables is necessary before changing production scheduling.
Raw logs and positions remain private.
