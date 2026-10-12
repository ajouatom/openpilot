# Nav model follow-up: USB layout and missed navigation turns

For the subsequent drive's external GPS/pose timing analysis and repeated-fix correction, see [nav_gps_timing_20261012.md](nav_gps_timing_20261012.md).

The user requested the nav map beside the driving view, on the right. USB screen mode 7 now retains the complete driving camera/3D view and HUD in the existing 1124-pixel left region and draws the shared map only in the 792-pixel right region. Alerts still draw last. This mode keeps the requested left/right order even when the general panel-swap setting is enabled. No driving-model or control policy changes accompany this layout correction.

The production renderer was exercised with synthetic 3D and camera scenes and its PNG outputs visually inspected. All 138 focused display, map-texture, driving-report and scene tests pass. The map texture retains its two-Hz cache and expiry; the UI does not start another map renderer.

## What the drive establishes

At the user's request, nine completed full logs covering the recent drive were copied privately alongside the low-rate route logs. No raw recordings, locations, credentials or vehicle identifiers belong in this document or Git.

The eGPU worker was already running the new checkpoint `cf8f6120/56320`, artifact `17c06f92…`. Its host inputs include a uint8 256×256 `nav_map` and 47-float navigation vector. The adapter uploads the packed host buffer and binds both inputs to the model's named device views. The prior parked inspection found both inputs nonzero in all 197 sampled frames; that does not prove their contents at every past driving frame.

In the reviewed drive, lane-line substitution was inactive. In the final right-turn approach (approximately 19:16:33–19:16:38 KST), external instructions remained valid and the upcoming right-turn distance decreased from about 25 m to 4 m. GPS fix age stayed below half a second in the sampled rows. The model's requested curvature remained near straight and then slightly left; the control curvature closely followed it. For example, at approximately 19:16:36 the model/control curvature was 0.00045/0.00054 1/m, and at 19:16:38 it was -0.00370/-0.00379 1/m. Steering was active without a reported driver steering override in those samples. Earlier right-turn approaches did produce right-turn predictions, so this is not evidence that navigation never reaches the model.

There are 105 mapsd timing summaries across the nine segments: 100 end with a valid map, five do not. The invalid summaries occur earlier with absent/short routes. The summaries surrounding the final missed turn report valid maps at approximately 5 Hz, with maximum rendering work below 42 ms. A five-second summary's final state is not a per-model-frame input trace. These observations do not establish whether the learned model ignored valid guidance, whether a route/map interpretation was unsuitable, or whether a transient model-input problem occurred between summaries.

The evidence supports the user's missed-turn report and rules out lane-line replacement in this drive. It does **not** establish successful route-following or a verified model/control fix. Do not force steering from the map, increase curvature/torque limits, or describe the parked input checks as navigation-driving validation.

## Final red-signal approach

The user also reported insufficient stopping at the last red intersection. Sequentially decoded road-camera frames from the final approach show two forward red lamps, including the approach and the near stop-line view. Random seeking in the raw HEVC stream produced reference-frame corruption and was discarded; use the sequentially decoded evidence only. The private recordings remain outside Git.

From approximately 19:15:20 to 19:15:38 KST, the CPU observer produced 118 observations: 78 aggregate red and 40 unknown. Its tracks included 96 confirmed-red lamp-core observations and 26 confirmed-red template observations, but no measured-housing (`detected`) observations. Result age at publication ranged from 95 to 170 ms. These are observation counts, not independently labelled detection accuracy. The visible red signal was therefore not simply absent from every observer result.

The moving preparation explicitly requires measured housing evidence before estimating range. Lamp-core boxes are synthesized around a bright point, so their width cannot be substituted as a measured housing width. All confirmed-red observations in this approach fail that source requirement. The preparation log alternates between `no_red`, `stale`, and the broad `ineligible` reason before the existing model stop starts; publication age below 200 ms does not guarantee freshness at a later planner read. Across all nine reviewed segments, all 288 preparation decision changes are inactive (144 ineligible, 80 stale, 64 no_red). There is no evidence of supplementary red preparation in this drive, despite mode 2 being selected.

The existing model-derived stopping state begins near 19:15:31.9 at 52.4 km/h. Requested acceleration is initially about -0.05 m/s2, reaches about -0.73 at 19:15:34.3, and fluctuates before strengthening. At 19:15:37.6 the driver brake is pressed at about 42 km/h. Earlier approach samples have no gas/brake override, blinkers or radar leads. This establishes a late/insufficient approach response requiring driver intervention, not a successful autonomous stop. `trafficState=green` in earlier planner logs is an inference from the driving model's motion trajectory; it is not proof that the separate color observer classified the visible red lamps as green.

The existing stopped-red departure guard cannot supply moving-approach braking. The current preparation is also only a conditional gentle deceleration cap, not a complete signal-stop controller. No range gate, detector threshold, acceleration limit or model selection was changed to mask this failure. A correction still needs a validated way to associate the real lamps with the forward movement and obtain usable geometry at night, plus recorded-input timing/stop-distance validation; route following and red stopping remain unresolved.

## USB deployment and final parked timing

Revision `f9379d64de` was installed while Park/inactive was verified. Only the USB cluster child was restarted; the driving-model, eGPU worker and driver-monitoring PIDs remained unchanged. Mode 7 remains selected, and the USB connection recovered successfully. The physical panel layout has not been independently photographed; layout verification used production-renderer images and runtime checks.

A subsequent 30-second parked check received 601 valid driving-model and 601 valid driver-monitoring messages at 19.997 and 20.006 Hz. Mean core utilization (cores 0 through 7) was 75.08, 73.63, 86.35, 79.83, 60.19, 78.25, 76.66 and 66.62 percent; core 2 peaked at 93.8 percent. USB used 8.45 percent of one core. External navigation had disconnected and there was no route during this final check, so its 1.20 percent mapsd utilization is an idle measurement, not a replacement for the earlier active-route measurements. CPU timing and valid message delivery do not establish signal detection or vehicle-response correctness.
