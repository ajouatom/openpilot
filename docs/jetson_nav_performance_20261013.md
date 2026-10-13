# RL Nav: recovering the 50 ms frame budget

Scope: carrot-nav only. This follows the live 17.8--17.9 Hz failure documented
in `jetson_nav_device_20261013.md`; it does not establish usable 20 Hz Jetson
operation or road behavior.

## Device disposition

After fresh valid Park, standstill and inactive-control checks, stopped
`carrot-jetlink` for this boot. A subsequent 30-second sample received 600 valid
modelV2 outputs, 600 valid driverStateV2 outputs and 599 road-camera messages.
Mean internal model execution was 24.54 ms, maximum 35.8 ms. Jetlink was inactive
throughout. The HUD remains stopped and the boot diagnostic display running.
Neither service is persistently disabled: reboot can restart the old release.

The installed signed host remains `0b027229ba`, using the cached RL Nav
80eedcdd engine. No model weights, TensorRT plan or validity gates changed.
Automatic approval review restricted prior publication approval to that payload;
follow-up signing/publication/install approval remains pending. These candidate
changes were tested separately from the stopped production inference service.

## Candidate changes

* Replace scalar AArch64 NumPy float16 finiteness checks with reusable binary16
  exponent-mask buffers. All 65,536 representations agree with `np.isfinite`,
  including both infinities and all NaN payloads. Validate every output; invalid
  output clears all recurrent state. Use the existing exact uint8-to-float16
  lookup for camera input conversion.
* TensorRT can reuse each prior state output's pinned host buffer as the next
  input staging source. GPU input/output buffers remain distinct. The serial
  CUDA stream finishes H2D before execution and D2H; synchronisation and output
  checks finish before another frame. Validate shape/dtype before graph capture,
  and preserve all reset/error behavior. Other backends retain copied staging.
* Explicitly negotiate `carrot_nav_u8_v1` for stateful protocol 2. Integer map
  pixels travel as uint8 and reconstruct identical float32 bits. Fractional,
  negative-zero, out-of-range and nonfinite maps retain the original payload.
  Other context floats remain untouched. Decode uses exact model-derived size,
  rejects malformed frames before inference and preserves legacy/v3 framing.
  Context shrinks from 262,380 to 65,772 bytes, bringing the complete request
  below the existing 512 KiB FunctionFS write limit. No lossy quantization or
  larger kernel contiguous-buffer allocation is introduced.

## Native isolated measurements

Jetson was already MAXN_SUPER with fixed CPU/GPU/EMC clocks. Existing cached
TensorRT 10.3.0 Orin engine was reused, with changing synthetic camera/navigation
inputs and recurrent-state resets every 25 frames. Each comparison ran 100
frames per variant; all six output tensors matched bit-for-bit on every frame.
Timing summaries exclude the first ten frames.

| Variant | Input ms | Engine ms | Feedback ms | Total mean ms | Total p99 ms |
| --- | ---: | ---: | ---: | ---: | ---: |
| Installed queues | 2.465 | 24.855 | 6.056 | 33.376 | 34.074 |
| Exact conversion/finiteness | 0.963 | 24.856 | 3.558 | 29.377 | 29.717 |
| Repeated copy baseline | 0.951 | 24.827 | 3.533 | 29.311 | 29.470 |
| Shared pinned state staging | 0.926 | 24.781 | 2.050 | 27.756 | 27.909 |

The shared-staging probe also injected a nonfinite driving output, verified
clearing every state buffer and then ran another finite frame successfully.
This reduces measured host work by about 5.6 ms, not the whole USB/model loop.

The actual codec, tested with blank/ramp/random integer maps, restored every bit.
Jetson decode averaged 0.106 ms. On the comma running its internal model,
encoding averaged 0.86--0.88 ms with scheduling outliers up to 14.4 ms; this
standalone process is not the production model thread. Do not infer worst-case
50 ms compliance from these averages or the isolated host benchmark.

Focused desktop checks: 74 passed. The wider Windows run had 568 passes,
30 skips and 76 failures involving Linux-only socket/process/system interfaces;
it is not a passing integration run. Linux integration CI must verify the final
commit before deployment. Ruff checks pass, excluding two pre-existing vendor
rules (TID251/ISC002). Native results/scripts remain private in
`.analysis/scratch/2026-10-13-jetson-nav/`.

Required next validation: signed candidate with matching client, actual camera
and USB, repaired HUD rendering, sustained valid model/DM 20 Hz, frame loss,
latency tails and per-core load. Route imagery is not validated while the
external route is absent. No closed-loop or traffic-light performance claim.
