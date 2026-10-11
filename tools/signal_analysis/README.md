# Offline traffic-signal observer

Find horizontal signal housings, track them through consecutive RGB frames, and
write observed red/green/unknown states with an annotated video. This is a
classical image-processing experiment, not a newly trained ONNX model.

```sh
python -m pip install -r tools/signal_analysis/requirements.txt
python tools/signal_analysis/replay_signal.py input.hevc output --start-frame 400 --end-frame 609
python -m unittest discover -s tools/signal_analysis -p test_signal_tracker.py -v
```

Choose a new output directory. Outputs are `observer.mp4`, `observer.jsonl`, and
`summary.json`. Frame bounds are inclusive. The default frame rate is 20 Hz;
override with `--fps`. For recorded camera timing, provide `--timestamps times.json`
containing `{ "400": 123456789000 }` mappings from frame index to EOF nanoseconds.
Without that file, temporal decisions use nominal frame index / fps.

The observer consumes current/previous images and timestamps only. It receives no
labels, manual boxes, future frames, vehicle state or driving-model outputs.
Green confirmation requires a previously confirmed red on the same track;
startup green remains unknown. Agreeing visible tracks do not identify the ego
lane, distinguish turn arrows, measure stop-line distance or authorize movement.
No vehicle messaging, Params or control outputs are implemented.

See [the trial report](../../docs/signal_tracking_trial_20261010.md) for the six
previously reviewed clip results and their limitations. Private recordings,
annotations and generated videos are intentionally not included.
## Offline color-correction experiment

`replay_color_experiment.py ROUTES OUTPUT` compares the existing tracker and
`conservative_green=True` on local `ROUTES/<segment>/fcamera.hevc` and
`decoded.json.gz` investigation data. It joins exact frame IDs across segment
boundaries, retains recorded cadence and writes paired compressed traces and
summary counts. Install this directory's OpenCV/NumPy requirements first.

The option is default OFF and has no live-worker selector. It is not ONNX
retraining. See `docs/signal_color_experiment_20261012.md` for the measured sky
rejection, remaining LED/reflection false positives and motion-blur regression.

Optional `--motion-dir DIRECTORY` adds a third tracker that rejects green during
moving turns and requires fresh straight/stopped context before resuming. Each
`DIRECTORY/<segment>.json.gz` is a list of carState records containing `timestamp`
(monotonic seconds), `valid`, `angle` (degrees), `rate` (degrees/s) and `speed` (m/s).
This is also offline-only; it preserves red evidence and does not use old green
as current permission. Steering thresholds are provisional, not a yaw estimator.

`--pose-dir DIRECTORY` adds the preferred rotation experiment independently of
the steering comparison. Each compressed segment list contains `timestamp`
(arrival seconds), `sample_time`, calibrated `yaw`/`yaw_std` (degrees/s), and
`valid` including filter/sensor/measurement/calibration checks. A stale or invalid
pose never falls back to steering-based green permission. This is offline-only.
