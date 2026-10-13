# Next parked performance investigation

Follow-up: `jetson_nav_retest_20261014.md` records the completed native A/B/A,
the small state-copy gain, the stale manager-import discovery and the actual
LZ4 regression. Default map-only packing sustained 20 Hz for two minutes with
camera/HUD/DM and no model frame-ID skips; active-route rendering is pending.

User request: investigate faster approaches for tomorrow. Scope remains
carrot-nav only. This preparation changes no production inference path, model,
vehicle files, settings, services or installed release. No device was contacted
during this follow-up. The last departure state is in
`jetson_nav_performance_20261013.md`: internal model restored at 20 Hz after
the external path still measured 19.8 Hz.

## What the timing actually establishes

The measured roughly 25 ms `server_gpu_ms` is **not kernel-only GPU time**.
`TrtEngine.run()` measures graph launch through stream synchronization; its
graph includes host-to-device input copies, model execution and device-to-host
output copies. The host CPU's roughly 11% total load does not establish that
these serial stages have spare time. The complete model loop was 50.56 ms,
so a small average gain is insufficient evidence for sustained 20 Hz. Aim for
several milliseconds of margin and examine p99/frame loss as well as averages.

`state_img_q` occupies 3,932,160 bytes as float16. Feature history occupies
8,388,608 bytes as float32, desire 4,224 bytes, and map/navigation feature
history 32,768 bytes each. Total recurrent state is **12,390,528 bytes**.
Each frame copies this to CPU, validates it and copies it back to GPU on the
next invocation: 24,781,056 bytes/frame, or 495,621,120 bytes/second at 20 Hz.
These are explicit copy sizes calculated from the selected graph and observed
runtime dtypes, not measured DRAM traffic and not USB payload sizes.

The previous shared pinned staging correction removed a CPU-to-CPU copy but
retained both GPU/CPU directions. This leaves a concrete, independent target
after the approximately 5.6 ms host improvement already measured.

## Priorities

| Priority | Candidate | What must remain equivalent | Status |
| --- | --- | --- | --- |
| 1 | Copy previous state directly GPU-to-GPU for the next frame; retain all CPU output validation | Every output bit, history resets, nonfinite rejection and recovery | Isolated A/B/A tool prepared; native timing pending |
| 2 | Validate recurrent outputs on GPU, returning only actual driving output and a validity result | Check every element including NaN/infinity, clear all history on failure, preserve diagnostic output contracts | Design candidate only; would remove state readback and CPU scan, but needs independent kernel validation |
| 3 | Measure actual request bytes and encoding cost before further compression changes | Exact pixels/context and bounded decoding | LZ4 negotiated, but departure measurement did not establish a speed gain; final live byte statistics not collected |
| 4 | Overlap next camera warp with current Jetson inference using bounded buffers | Camera identity, transform identity, no stale/reordered frames and unchanged DM cadence | Design candidate only; do not add a frame of control latency just to improve throughput |

Do not count compression of a repeated ramp as a realistic camera result.
Integer-map packing saves 196,608 bytes only when the map values are canonical
integer pixels; LZ4 camera savings depend on image entropy and CPU cost.
Compression checks may consume the byte savings on a busy comma. Keep exact
raw fallback and measure a representative real route before selecting it.

Batching consecutive driving frames is inappropriate for this latency target.
Do not reduce DM cadence, relax output freshness, hide skipped frames, or change
to INT8/model weights merely to report 20 Hz. TensorRT graphs and fixed clocks
are already enabled; re-enabling those is not a new optimization.

## Prepared comparison

`tools/jetlink/bench_state_feedback.py` is an explicit offline tool. It does not
open USB, start a service, alter an engine or join modeld. It refuses to run if
the installed inference service is active or its state cannot be confirmed.
Use it only during a parked session with the production service already
stopped. The tool uses the existing plan and records its SHA256.

It compares original host staging, GPU-to-GPU state copying, then original host
staging again. This A/B/A order helps expose thermal/clock drift. All GPU input
and output allocations remain distinct. A serial stream consumes previous
outputs before model execution overwrites them. Every output is still copied
back and validated using the installed `StatefulQueues`; this first experiment
does not remove any finiteness checks. All six tensor hashes must match for
every synthetic frame, including resets every 25 frames. Deliberate nonfinite
injection in each output must clear history, and the next reset inference must
match the original zero-history frame exactly.

The GPU reset explicitly clears the previous-output device buffers, not just
CPU staging. Forgetting that would silently reuse old history after reconnect
or invalid output. Unit checks cover copy direction/order, all-output readback,
failed enqueue, device-source reset and refusal on uncertain service state.

Example after making these **tool files** available in an isolated checkout on
the parked Jetson (do not change the signed current release):

```sh
PYTHONPATH=/opt/carrot-jetlink/current/third_party/jetlink \
  /opt/carrot-jetlink/venv/bin/python /path/to/bench_state_feedback.py \
  --engine /opt/carrot-jetlink/cache/engines/1d18c42ea4cd95c8.trt10.3.0.Orin-sm87.plan \
  --spec /opt/carrot-jetlink/current/openpilot/selfdrive/modeld/jetlink/jetson_model.json \
  --frames 150 --output /tmp/nav-state-feedback-new.json
```

The script can use the installed jetlink package via PYTHONPATH when copied
outside a full checkout. A passing synthetic comparison is not native camera,
USB, HUD, route-map or driving validation. If the measured gain is worthwhile,
integrate with explicit reset hooks and test the real full pipeline for at least
two minutes, then representative route imagery and warm-device conditions.

Preparation checks: 20 focused desktop tests pass; Ruff and CLI import/help
checks pass. No CUDA performance or 20 Hz improvement is claimed for this
candidate until the Jetson comparison is run.

## Documentation supporting the investigation

NVIDIA's [CUDA 12.6 Tegra memory guidance](https://docs.nvidia.com/cuda/archive/12.6.3/cuda-for-tegra-appnote/index.html#memory-management)
describes shared physical DRAM but different caching/access behavior and the
need to measure memory choices. Shared DRAM does not make explicit copies free.
The [TensorRT 10.x performance guidance](https://docs.nvidia.com/deeplearning/tensorrt/10.x.x/performance/best-practices.html)
separates transfers, inference and end-to-end measurements. These sources
support profiling and copy reduction, not a speedup estimate for our hardware.
