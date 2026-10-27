# Nav model integration — carrot-nav, 2026-10-12

## Scope and model identity

The current eGPU selection is superseded by the [2026-10-13 RL nav update](nav_rl_model_20261013.md), including its archived ONNX. The deployment and timing results below describe the original model.

This branch starts at carrot-wip `0e4ee1bdbef3ad7284b136fddf31592e3f8a785f` and ports [comma nav-model-integration](https://github.com/commaai/openpilot/commits/nav-model-integration/) through `df7fb1843a0affc3d9b50e6d7f5ccdd98c58ebf4` into the existing Carrot navigation paths.

This upstream revision is a new **eGPU driving model** with `nav_map` uint8 `[1,1,256,256]`, `navigation` float32 `[47]`, and additional recurrent map/navigation state. It is not the old separate navModel neural process. Internal driving, driver monitoring and Jetson model selections stay unchanged; they do not gain model navigation inputs from this change. Existing signal/DM protection is retained.

- Checkpoint: `cf8f6120-49ee-48f8-b2d7-d663a4708271/56320`.
- Model SHA256: `17c06f9208ab9d1529b52589662af85734425a482e5972d342f676e5d6597f56` (806,704,311 bytes).
- Runtime: tinygrad `d3f09c9bbd542fbfbe68c8569a1173550c408969`, tar SHA256 `90970e01f2ba70d553294c0f9d5c6fe1678827f750705cab679a7b19b7bcb0b2`.
- Delivery: `https://upload.shind0.synology.me/models/comma4-big-nav-cf8f6120/manifest.json`; isolated package and runtime, normal startup delivery gate. No matching public ONNX export was available during this work.

`tools/modeld/build_nav_package.py` constructs the hash-pinned package. Public artifacts were copied to the user's separate NAS model directory and verified. Empty-cache HTTPS installation and network-disabled cache reuse passed. Installation alone is not GPU execution validation.

## Data flow and source priority

Existing 7713/legacy TCP routes, 7714 CarrotNavi control data, CarrotServ TBT conversion and Mapbox navd are reused. CarrotMan is the single navRoute publisher. A connected external source owns both guidance and geometry, including explicit empty route/off-route state. Saved Mapbox geometry never fills a missing external route. External disconnect clears legacy geometry; the saved Mapbox destination can resume with a new route request.

Mapbox destinations are independent persistent Params. Receiving an external route does not overwrite or remove them. A stable `NavRoute.carrotRouteId` changes on source/geometry replacement; periodic heartbeats serve restarted consumers without restarting route tracking. Route and instruction publishers have bounded liveness checks. A slow response for a changed destination, key or waypoint list is discarded. HTTP requests run in one bounded navd worker rather than delaying the instruction publisher. Geometry is bounded to 4096 published points.

Carrot's existing phone/external GPS fusion remains in use. Added CarrotMan fix and prediction timestamps distinguish a fresh prediction from an old fix rebroadcast at 20 Hz. Consumers reject fused fixes older than two seconds, stale publications/predictions, invalid coordinates and stale native GPS fallback. These schema/Params changes require a native rebuild.

`mapsd` renders one CPU grayscale image using comma's 1 m/pixel style and route-snapped pose. Invalid or unavailable map data produces black. A route mismatch or map older than 500 ms is excluded from model input; missing inputs are explicitly zeroed every inference. Nonblocking VisionIPC reads keep network/render waits out of modeld. The old unused Qt map renderer and orphan navModel service are removed; historical log schema remains readable.

## Mapbox, Web and USB

Register the existing Mapbox Public key in Carrot Web Settings → Network → Mapbox. No Secret key is required. Search uses [Geocoding v6](https://docs.mapbox.com/api/search/geocoding/) with permanent results so a selected destination may be persisted; the account must permit Permanent Geocoding. Route requests use Directions driving-traffic. Keys stay on the server and are omitted from errors.

With a configured key, basemap roads come from [Mapbox Streets v8](https://docs.mapbox.com/data/tilesets/reference/mapbox-streets-v8/) through the [Vector Tiles API](https://docs.mapbox.com/api/maps/vector-tiles/). The adapter preserves comma's raster widths/intensities and bridge/tunnel ordering while mapping Mapbox road classes. Mapbox and Protomaps geometry/classification are not guaranteed pixel-identical or model-equivalent. Without a key, comma's authenticated Protomaps PMTiles endpoint is used; availability/authorization of that service is required. There is no fabricated road geometry fallback.

Carrot Web Tools → **Nav Map / 내비 지도**, `/navigation`, provides the rendered image, destination search/selection and clear action. The page pauses image requests when hidden. Network and provider account requirements apply; live paid API requests are not exercised by desktop tests.

`ClusterHudScreenMode=7` displays the shared rendered image on the right of the external USB HUD, retaining the camera/3D driving view and complete driving HUD on the left. See the [layout correction and missed-turn investigation](nav_model_followup_20261012.md). It does not select a driving model. The PNG is produced at most 2 Hz in tmpfs; USB texture updates only when its timestamp changes. Display expiry is two seconds. Default screen mode stays 0. Korean, English and Chinese setting descriptions include modes 6 and 7. Korean/English user guides are updated. The experimental branch's Wiki preview was generated and 559 Markdown pages validated; the shared production Wiki remains sourced only from carrot-wip.

## CPU policy and verification

- Renderer: SCHED_OTHER, nice 15, little-core mask 0–3. It does not use the QCOM GPU or change DM/control/camera/USB affinity.
- Render work starts only for an active eGPU, USB map mode, or a recently requesting Web viewer. At most 10 Hz, with cooldown of at least 10 times measured render CPU work (about 9–10% of one core for rendering). This is a duty policy, not an OS CPU quota; other work and first-load costs are separate.
- Current-view tile fetches only; no full-route prefetch. Decoded tiles use a bounded cache, PMTiles disk cache is 64 MiB, Mapbox response is capped at 1 MiB. Token changes invalidate the tile client. Tile errors back off 30 seconds.
- A render blocked for 500 ms is discarded. Excessive renderer cost can reduce map availability rather than delay driving inference.
- Native Cython analytic strokes preserve the Python reference's caps, compositing and rounding. Fast math and contraction are disabled; expanded row bounds skip pixels outside a stroke. The Python implementation remains the fallback when the extension is unavailable. Native build is required for the CPU reduction.
- Dense desktop benchmark, 120 roads and 4096 route points: mean wall 27.47 ms, p95 46.54 ms, mean CPU 27.08 ms. Fixed 20 Hz would consume approximately 54.17% of one desktop core; the adaptive estimate is 9.08%. These are synthetic desktop results, not target timing or road validation.

Desktop regression suite: 413 passed, nine platform/native-dependent skips. It uses real Cap'n Proto messages with Params/platform/IPC stubs where Windows cannot run native interfaces. Coverage includes artifact delivery/startup, mixed byte/float layout and recurrent state metadata, renderer/tiles, external priority, stale position/input, asynchronous destinations, Web API and USB texture lifecycle. Focused checks after final fixes passed. Web bundles build and the existing 13 model/status JS tests pass. Linux CI includes native pinned-runtime CPU warp tests and navigation tests; a physical GPU and USB require target verification.

Use `python tools/modeld/profile_navigation.py --seconds 60 --output /tmp/nav-cpu.json` to record every CPU core and relevant process affinity/load. `--benchmark` runs the reproducible synthetic renderer workload. For target comparison, verify Park/inactive, retain model/DM periods and validity, compare map idle, map active, and shared USB/Web output, and inspect each core separately. If constrained, reduce render cadence/preview frequency or tile decoding/cache churn before changing protected process placement.

Vehicle 192.168.0.178 installed `59f6081712` on carrot-nav through the normal startup download/build/boot-validation path. Actual gfx1200 execution uses the pinned new model. The public Mapbox key is stored only in vehicle Params; no secret key is required or stored. Comma's tile endpoint returned HTTP 403, while Mapbox tiles, Korean address search, destination save/clear and Directions returned usable results. Saving a test Mapbox destination preserved the active external route ID and geometry; the test destination was cleared afterward.

With real Mapbox tiles and external guidance, the initial Python renderer averaged 9.29% of one CPU core. Model and DM each published 601 valid results over 30 seconds, approximately 20 Hz. Core means 0–7 were 82.99, 82.03, 92.19, 85.65, 69.82, 75.76, 76.11, 72.25%; core 2 has limited headroom. This motivates native rasterization without changing protected affinities.

Parked target synthetic dense-road benchmark (20 paced frames, 120 roads, 4096 route points): Python mean CPU 118.09 ms / wall 154.17 ms / wall p95 212.29 ms; native mean CPU 33.96 ms / wall 46.98 ms / wall p95 57.20 ms. CPU work fell about 71%. Nine native/reference parity cases pass on aarch64, including 2880 randomized strokes and complete road/bridge/tunnel images. These are synthetic timing and pixel parity results, not driving validation. Private device evidence stays under `.analysis`.

## Final vehicle validation

Runtime revision `fa69187f9b` built through the native SCons C++ path and passed all nine installed-extension parity cases. Restarting only mapsd retained the manager's preimported Python module, so the vehicle was rebooted in Park/inactive to activate the new renderer. The `ready` status file and Web header confirm the new process code. Model delivery/boot validation succeeds and the actual worker uses the pinned `17c06f92…` artifact.

With 162 external route points, Mapbox basemap, Web preview and USB screen mode 7, a 30-second sample recorded:

| Measurement | Result |
| --- | --- |
| mapsd CPU, one-core scale | 5.23% mean, little cores 0–3 |
| USB display CPU, one-core scale | 9.52% mean, core 4 |
| Driving model | 601/601 valid, 19.999 Hz |
| Driver monitoring | 601/601 valid, 19.998 Hz |
| CPU core means 0–7 | 81.29, 81.01, 91.04, 84.80, 66.74, 73.77, 74.67, 69.76% |
| CPU core maxima 0–7 | 91.7, 92.0, 95.1, 90.9, 92.1, 87.8, 78.1, 72.6% |

A separate read-only inspection of the live model's shared input buffer found nonzero `nav_map` and `navigation` in all 197 samples. Native map publication was 4.995 Hz with receive-age p95 62.54 ms; this age is measured from render start to the observing client, not camera-to-actuation latency. USB connected state and approximately 5 Hz H264 output were observed. Physical panel appearance was requested from the user but was not independently captured. The temporary screen mode 7 test was restored to the prior camera mode 2.

The live before/after samples have different route lengths and a reboot between them; they are operational observations, not an isolated CPU speedup experiment. The paired synthetic workload supports the 71% raster CPU reduction. Core 2 still has limited headroom despite normal model/DM cadence. Keep the bounded duty/caches and lower preview/cadence before expanding auxiliary work; no protected process affinities or DM policy were changed. These parked checks do not establish moving-route availability, physical USB appearance, or driving behavior.

After restoring camera mode 2, another 30-second sample recorded mapsd 5.13% and USB display 16.92% of one core. Model and DM each had 601/601 valid messages at 20.007 Hz. Core means 0–7 were 80.98, 82.39, 90.79, 87.23, 72.07, 72.96, 78.26, 70.27%; the largest one-second sample was core 2 at 95.9%. USB remained connected. Model/DM/worker PIDs were unchanged between the two final display-mode samples. The vehicle retains mode 2, SignalAssistMode 2 and the new eGPU nav model.

[Linux CI 38232319180](https://github.com/ajouatom/openpilot/actions/runs/38232319180) passes: 251 model/delivery/startup checks, 34 navigation checks including native pixel parity, two pinned-runtime CPU warp/buffer checks and 13 Web JS checks. Final focused Windows tests pass 39 checks with nine native-extension skips; the installed aarch64 tests and Linux CI cover those native cases.
