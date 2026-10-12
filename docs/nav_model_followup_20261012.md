# Nav model follow-up: USB layout and missed navigation turns

The user requested the nav map beside the driving view, on the right. USB screen mode 7 now retains the complete driving camera/3D view and HUD in the existing 1124-pixel left region and draws the shared map only in the 792-pixel right region. Alerts still draw last. This mode keeps the requested left/right order even when the general panel-swap setting is enabled. No driving-model or control policy changes accompany this layout correction.

The production renderer was exercised with synthetic 3D and camera scenes and its PNG outputs visually inspected. All 138 focused display, map-texture, driving-report and scene tests pass. The map texture retains its two-Hz cache and expiry; the UI does not start another map renderer.

## What the drive establishes

At the user's request, nine completed full logs covering the recent drive were copied privately alongside the low-rate route logs. No raw recordings, locations, credentials or vehicle identifiers belong in this document or Git.

The eGPU worker was already running the new checkpoint `cf8f6120/56320`, artifact `17c06f92…`. Its host inputs include a uint8 256×256 `nav_map` and 47-float navigation vector. The adapter uploads the packed host buffer and binds both inputs to the model's named device views. The prior parked inspection found both inputs nonzero in all 197 sampled frames; that does not prove their contents at every past driving frame.

In the reviewed drive, lane-line substitution was inactive. In the final right-turn approach (approximately 19:16:33–19:16:38 KST), external instructions remained valid and the upcoming right-turn distance decreased from about 25 m to 4 m. GPS fix age stayed below half a second in the sampled rows. The model's requested curvature remained near straight and then slightly left; the control curvature closely followed it. For example, at approximately 19:16:36 the model/control curvature was 0.00045/0.00054 1/m, and at 19:16:38 it was -0.00370/-0.00379 1/m. Steering was active without a reported driver steering override in those samples. Earlier right-turn approaches did produce right-turn predictions, so this is not evidence that navigation never reaches the model.

There are 105 mapsd timing summaries across the nine segments: 100 end with a valid map, five do not. The invalid summaries occur earlier with absent/short routes. The summaries surrounding the final missed turn report valid maps at approximately 5 Hz, with maximum rendering work below 42 ms. A five-second summary's final state is not a per-model-frame input trace. These observations do not establish whether the learned model ignored valid guidance, whether a route/map interpretation was unsuitable, or whether a transient model-input problem occurred between summaries.

The evidence supports the user's missed-turn report and rules out lane-line replacement in this drive. It does **not** establish successful route-following or a verified model/control fix. Do not force steering from the map, increase curvature/torque limits, or describe the parked input checks as navigation-driving validation.
