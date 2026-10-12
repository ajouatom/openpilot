# Nav model integration — carrot-nav, 2026-10-12

## Scope and model identity

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

`ClusterHudScreenMode=7` displays the shared rendered image on the external USB HUD, retaining speed and alerts. It does not select a driving model. The PNG is produced at most 2 Hz in tmpfs; USB texture updates only when its timestamp changes. Display expiry is two seconds. Default screen mode stays 0. Korean, English and Chinese setting descriptions include modes 6 and 7. Korean/English user guides are updated. The experimental branch's Wiki preview was generated and 559 Markdown pages validated; the shared production Wiki remains sourced only from carrot-wip.

## CPU policy and verification

- Renderer: SCHED_OTHER, nice 15, little-core mask 0–3. It does not use the QCOM GPU or change DM/control/camera/USB affinity.
- Render work starts only for an active eGPU, USB map mode, or a recently requesting Web viewer. At most 10 Hz, with cooldown of at least 10 times measured render CPU work (about 9–10% of one core for rendering). This is a duty policy, not an OS CPU quota; other work and first-load costs are separate.
- Current-view tile fetches only; no full-route prefetch. Decoded tiles use a bounded cache, PMTiles disk cache is 64 MiB, Mapbox response is capped at 1 MiB. Token changes invalidate the tile client. Tile errors back off 30 seconds.
- A render blocked for 500 ms is discarded. Excessive renderer cost can reduce map availability rather than delay driving inference.
- Dense desktop benchmark, 120 roads and 4096 route points: mean wall 27.47 ms, p95 46.54 ms, mean CPU 27.08 ms. Fixed 20 Hz would consume approximately 54.17% of one desktop core; the adaptive estimate is 9.08%. These are synthetic desktop results, not target timing or road validation.

Desktop regression suite: 413 passed, nine platform/native-dependent skips. It uses real Cap'n Proto messages with Params/platform/IPC stubs where Windows cannot run native interfaces. Coverage includes artifact delivery/startup, mixed byte/float layout and recurrent state metadata, renderer/tiles, external priority, stale position/input, asynchronous destinations, Web API and USB texture lifecycle. Focused checks after final fixes passed. Web bundles build and the existing 13 model/status JS tests pass. Linux CI includes native pinned-runtime CPU warp tests and navigation tests; a physical GPU and USB require target verification.

Use `python tools/modeld/profile_navigation.py --seconds 60 --output /tmp/nav-cpu.json` to record every CPU core and relevant process affinity/load. `--benchmark` runs the reproducible synthetic renderer workload. For target comparison, verify Park/inactive, retain model/DM periods and validity, compare map idle, map active, and shared USB/Web output, and inspect each core separately. If constrained, reduce render cadence/preview frequency or tile decoding/cache churn before changing protected process placement.

Vehicle 192.168.0.178 deployment and measured comparison are in progress at the user's explicit request. Final target results will be appended after native rebuild and execution checks. Private device evidence stays under `.analysis`.
