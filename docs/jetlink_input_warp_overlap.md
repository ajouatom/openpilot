# Candidate 4: prepare same-frame context while comma warp is in flight

`JoiningModel.run` now submits its existing QCOM image warp and fills the CPU
context through the existing `Warp.__call__(prepare=...)` hook before reading
the completed image. Previously context filling finished before warp submission.

This is independent of candidate 3's reply format: it works with full replies
and does not require a new Jetson package. It does require the existing Warp
prepare hook in the base branch; when porting to carrot-nav, retain that hook's
`finally: result.numpy()` drain behavior as well as the call-site change.

Only `fill_inputs` overlaps GPU work. Both operations use the same frame; the
image, transforms, context and desire history are neither replaced nor prepared
on a background thread. USB submission still waits for both operations. This
does not introduce asynchronous USB writes, cross-frame pipelining, changes to
model execution, control outputs, reset policy, timeouts or CPU priorities.

If context preparation raises, Warp drains the outstanding GPU result before
the existing local-model fallback can use QCOM. A failed GPU read cannot send
an old image. A link failure still closes the client, records the external-model
fault when active, resets the local model and follows the existing fallback.

## Evidence and limits

Earlier bench prototype, 2,400 selected frames/condition, real cameras and
simulated map/cruise inputs, vehicle detached:

| Condition | Mean call ms | P99 ms | Maximum ms | Calls >50 ms |
|---|---:|---:|---:|---:|
| Baseline | 41.480 | 42.297 | 45.174 | 0/2400 |
| Input/warp overlap only | 41.223 | 41.995 | 44.374 | 0/2400 |
| Reply projection + overlap | 38.898 | 39.669 | 42.549 | 0/2400 |

The overlap-only improvement was about **0.26 ms mean / 0.30 ms P99** in this
setup. This is a small host preparation improvement, not faster GPU inference.
These are previous prototype measurements; this commit's production call-site
has been checked on PC and has not been freshly installed or timed on devices.
It is not proof of unchanged road-driving behavior. Representative full-load
and longer device acceptance remain necessary before operational rollout.

PC validation: 136 passed across input-overlap, hotplug, warp startup, navigation,
app-model and output-projection suites. Tests compare input/desire/image bytes, route disappearance, exact production
call ordering, and context/GPU/USB failure paths before local fallback. Existing
legacy and stateful app-model, navigation and hotplug checks remain required.
The intermittent 100+ ms USB write tail is not claimed to be fixed by this change.
