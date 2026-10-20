# Jetson USB NAV preview and first N16 drive review

The owner clarified that the missing map is on the USB display attached to
Jetson. Read-only inspection found screen mode 7, the **NAV model preview**.
This is different from the phone navigation video.

## Display failure and correction

On comma, mapsd and navd were running, the route had 144 coordinates, and the
256 x 256 PNG preview was 1826 bytes, 0.193 seconds old, with status `ready`.
Jetson received mode 7 but had neither that PNG nor a preview in its display
snapshot. Its separate 960 x 540 phone navigation H.264 stream was fresh and
decoding without drops. That stream does not populate the NAV model panel.
The earlier successful phone-stream observation did not validate mode 7.

The Wi-Fi snapshot publisher now explicitly enables forwarding of the existing
NAV preview in mode 7. It reads at most twice per second, limits PNGs to 32 KiB
and 256 x 256, and carries the source age. The Jetson-only display adapter checks
the digest, full PNG decode, source age and conservative transport/local age
before atomically making the image available to the existing renderer.
Missing, expired, malformed or oversized input hides the map. Unchanged pixels
reuse the texture. Preview data cannot increase the existing 96 KiB snapshot
limit; optional images are discarded before route events if necessary.

Only the Wi-Fi publisher opts in. There is no new comma-to-Jetson USB display
fallback, ESP32 path, driving-model input change or locationd change. Jetson
continues driving its own USB panel. `nav_model_map` in the host HUD status
reports preview freshness separately from the phone-map metrics.

Desktop validation: 55 tests passed, two Linux/native cases skipped, including
the actual renderer draw method fed through the new remote PNG adapter,
disconnect expiry, unchanged-image reuse, source expiry, malformed data,
transport age, mode switching, snapshot budgets and the existing Wi-Fi/media
contracts. Renderer calls use a graphics double: this is not a physical panel
or road validation. The signed runtime and branch pin are delivered through
Git Pull and **both comma and Jetson reboot**, with actual offroad and Internet
throughout preparation. See [the delivery policy](jetson_update_policy_20261020.md).
No operational vehicle files, settings, services or model caches were changed
during this diagnosis. Physical activation and appearance remain to be checked
after the ordinary update.

The published host source is `c8a95308b0e6ec98e2da0a8f60f34043d8c64f5d`.
[CI 39386479268](https://github.com/ajouatom/openpilot/actions/runs/39386479268)
passed 94 root updater tests, 868 broad tests (56 skips), 119 navigation/cluster
tests, 11 real protected-storage tests and 11 NAT tests including isolated
packets; Windows installer checks passed too. Counts overlap.
The 64,675,738-byte runtime bundle has SHA-256
`1781c3830bac3d0d006263812ab39d9c2afc48b0c7a7d9f5e45f27a0d9896beb`.
Its signature and all 1084 bundled files were checked against the public source
export, including the existing legacy model-pin filename mapping. The bundle
and manifest were published on the existing signed NAS path and the full bundle
was fetched again over HTTPS to verify size/hash. The N16 model hash is unchanged.
This verifies distribution contents, not physical Git Pull/reboot activation.

## Actual DATA capacity

The owner's original R2 write record and current partition table agree: the
119.375 GiB card contains a fixed 40 GiB protected image, with approximately
16 GiB allocated to CARROT_DATA and 79.375 GiB outside the image. Automatic DATA
growth was disabled by that image builder. This layout explains why a nominal
128 GB card does not expose all its space as DATA.

Current mounted DATA has 16,795,955,200 bytes of filesystem capacity,
6,546,927,616 bytes used and **10,232,250,368 bytes (9.53 GiB) available**:
40% used, roughly 2% of inodes used. The queried current-boot server/updater
logs contain no storage exhaustion errors. N16 is loaded successfully.
Models occupy about 2.23 GiB and engines 2.20 GiB; retained runtime releases
and update bundles account for about 1.28 GiB together. The current model
plus engine need about 1.50 GiB. A fresh N16 preparation estimate including
patched ONNX and bundle/extraction is about 2.40 GiB; the previous release's
conservative USB admission requirement is 5.64 GiB. Current free space exceeds
both. Capacity is not the observed failure, and repartitioning is unnecessary
for this update. Old ONNX/release retention can still consume space over future
updates; this result is not a claim of unlimited retention.

## locationd alert from the reported drive

Twenty-two closed qlogs and the three full segments surrounding the alert
were read privately. In the three-minute full window, 3599/3600 livePose
messages have all health flags true; one has `inputsOK=False` while
`sensorsOK` and `posenetOK` remain true. The locationd process stays running.

The exact preceding cameraOdometry message is the only invalid one in that
window. Model frame IDs jump from 18030 to 18032; frame 18032's odometry is
marked invalid by the existing `vipc_dropped_frames < 1` rule in
`fill_pose_msg`. livePose becomes input-invalid at monotonic 13130.190661 and
recovers at 13130.248367, about 57.7 ms later. Vehicle speed is approximately
39 km/h. The `locationdTemporaryError/softDisable` alert is displayed from
13130.199585 through 13132.184096, about 1.985 seconds. Alert display duration
must not be described as two seconds of continuously invalid pose.

The encompassing model timing window records one dropped frame, maximum loop
83.801 ms, maximum camera wait 32.466 ms and maximum inference 51.701 ms.
The overlapping Jetlink window has maximum USB send 16.977 ms, exchange
44.526 ms and server GPU 27.273 ms. These are window maxima, not a per-frame
causal trace; they do not uniquely attribute the drop to USB, GPU, Wi-Fi,
the model checkpoint or CPU contention. No storage failure or locationd crash
was found. Do not weaken validity/alerts or claim that the display correction
fixes the remaining frame-timing problem.

Raw logs, location data, original image evidence and device identifiers remain
private. The NAV display/runtime change belongs to carrot-nav; carrot-wip
receives this record only and retains its separate model/runtime.
