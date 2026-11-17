# N16 replay

Runs part of a recorded drive through N16 on the PC, with the inputs the comma sent
(camera frames, calibration, desire, navigation vector, route map, `action_t`), and
puts the result next to what the tablet returned in the rlog. Use it to tell whether
a bad drive comes from the model, from our inputs, or from the tablet's execution.

Needs a Python with `numpy av pycapnp zstandard onnxruntime onnx`, the N16 ONNX
(`model.onnx` from the Jetlink model index), and the drive's `rlog.zst` plus video.

```sh
# 1. once: fp32 copy of N16 (ORT CPU runs the fp16 graph at minutes per frame)
python to_fp32.py model.onnx n16_fp32.onnx

# 2. optional: the route map the comma should have drawn (mapsd's renderer, Mapbox tiles)
python positions.py 'ROUTE--{seg}/rlog.zst' pos.json 10:45 11:25 [--full]
MAPBOX_TOKEN=pk... cargo run --release --features native -p openpilot-mapsd \
  --example replay_maps -- pos.json maps.bin /tmp/tiles

# 3. replay; --qcam for the uploaded qcamera, or --fcam/--ecam for the full video
python replay.py --rlog 'ROUTE--{seg}/rlog.zst' --start 10:45 --end 11:25 \
  --model n16_fp32.onnx --qcam 'ROUTE--{seg}/qcamera.ts' [--maps maps.bin] out.npz

# 4. side by side with the tablet
python compare.py out.npz [other.npz] --from 664 --to 684
```

- `{seg}` in a pattern is the segment number. Times are seconds from the route start.
- With `--qcam` there is no wide camera: the wide input is the road camera warped
  into the wide frame, black outside. Straight road matches the tablet to about
  0.13 m (lanes); turns need `--fcam`/`--ecam` (copy `fcamera.hevc`/`ecamera.hevc`
  from `/data/media/0/realdata/<route>--<seg>/` on the device). Use `--full` for
  `positions.py` then.
- The first 6–10 s of a window fill the model's recurrent state; start 20 s early.
- Drive `00000cba` (2026-11-15), missed left turn at 673–678 s: with the black map
  the comma actually sent, the PC plan barely turns (turn-left desire ≤ 0.07); with
  the rendered map it turns from 47 m out (desire 0.4–0.8).
