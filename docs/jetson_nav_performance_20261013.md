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

## First deployed candidate and follow-up

The user explicitly approved signing, publication and parked installation of
`174dc5a145` and the follow-ups needed for 20 Hz validation. Its immutable bundle
was signature/hash verified over HTTPS and installed by the existing reboot
gate. Engine cache reuse and `CANDIDATE_PROBE_OK` were observed. The vehicle
retains HEAD `5e85695197`, with backed-up, hash-checked client/codec/manifest
overlays. The earlier startup overlay remains. Linux CI passed 732 integration,
119 navigation/cluster and 11 protected-storage tests, plus Windows installer
and eGPU compatibility checks.

HUD now starts successfully in screen mode 7/camera mode 2. Its service and
inference remain alive without restarts. The boot-local display override uses
the signed implementation. One boot handoff unknown-message/reconnect remains;
current health subsequently returned to OK. Physical panel appearance and map
delivery remain distinct from renderer process health.

Actual 60-second camera/USB/HUD co-load: 1,186 valid model outputs, 1,200 valid
DM outputs, 1,201 camera messages. Model mean interval 50.60 ms, p99 56.40 ms,
maximum 72.43 ms; 14 skipped camera frames. This is **19.77 Hz, still failing
the requested sustained 20 Hz criterion**. Mean full loop 50.58 ms, inference
46.27 ms, host total 28.25 ms and USB send 7.04 ms. Per-core comma CPU was
80.5/79.1/83.3/83.3/53.7/74.4/70.8/50.4%, aggregate 71.8%; Jetson aggregate
10.9%, busiest core 24.6%, including the working HUD. Jetson temperature was
about 67.7 C with thermal health OK.

The follow-up negotiates optional system LZ4 compression of the exact camera
and context bytes. Both devices already provide liblz4. No image resampling or
lossy compression is used. Incompressible data keeps raw framing, unavailable
libraries never negotiate it, and decoding requires exact bounded output size
before inference/reset. Compression can compose with integer-map packing.
Add request byte counts to existing timing diagnostics to measure actual
benefit; sustained live performance must be retested after deployment.


## Departure checkpoint

Signed follow-up `f1838e2c54` was published, verified and installed through the
same reboot gate; current host source and both running services were confirmed.
The cached engine was reused. Vehicle client/block codec/diagnostic byte count/
manifest overlays were backed up separately; previous manifest comparison
normalizes CRLF only, without changing signed content. Linux CI passed 736
integration tests, 119 navigation/cluster tests and 11 protected-storage tests;
eGPU compatibility and Windows installer checks also passed.

The user needed to depart immediately, so prolonged parked validation stopped.
A 10-second check still yielded 198 valid model outputs, 199 valid DM outputs,
200 camera messages and two skipped camera frames. Model interval averaged
50.56 ms, inference 46.17 ms; LZ4 capability was negotiated, but these data do
not prove transfer reduction or sustained 20 Hz. Both services had zero
restarts and thermal health was OK at about 69.4 C. **20 Hz remains unresolved.**

After another fresh Park/inactive guard, stopped Jetson inference for this boot
to restore the internal model. The new release remains installed; this is not
a persistent disable, and reboot can start it again. Do not call this release
20 Hz validated or road validated. Follow-up performance work should resume
while parked; no driving-time restart or further model swap was requested.
