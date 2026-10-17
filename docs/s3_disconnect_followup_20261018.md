# S3 disconnect follow-up, 2026-10-18

Actual Wi-Fi HUD TCP failures remain reproducible. This change removes only the
ten-second shared-USB settling delay from the Wi-Fi path. Both paths still wait
for eGPU startup; the directly connected USB display retains its existing delay.
S3 USB traffic does not traverse C4's USB bus. No encoder, bitrate, queue,
authentication, firmware, router, model, or control policy is changed.

## Live evidence

The vehicle runs base `087f862d81` with the previous selective HUD memory fixes.
Three-minute simultaneous C4 ping sampling initially gives mean RTTs of
232.12 ms to S3, 226.16 ms to the gateway and 222.37 ms to the wired PC. Received
counts are 176/180, 177/180 and 176/180. Two TCP disconnect episodes occur.
This first interval overlaps another investigation's bulk vehicle-log/video
downloads. An SSH connection has 1,705,680 bytes queued; these observations
cannot assign the common delay exclusively to an AP or S3 defect.

After those downloads finish, a second three-minute real HUD observation still
has two disconnect episodes. Mean RTTs fall to 81.11/67.73/69.87 ms and all three
destinations return 177/180 packets. Authenticated S3 discovery usually succeeds
even while the TCP display connection is closed. Bulk transfer therefore
aggravates the measurements but does not explain away the remaining failures.

A guarded HUD-only off/on comparison also leaves delayed gateway/PC replies
while the HUD is stopped. Mean gateway RTT is 441.45 ms before, 117.70 ms with
HUD off, and 92.13 ms after restoration. Because background downloads change
during these phases, their difference is not a clean causal estimate of HUD load.
MemAvailable at the phase snapshots is 287.80/413.85/390.16 MiB. Freeing active
renderer memory is not by itself proof of a continuing leak.

Omada shows both clients on the same EAP610-Outdoor: C4 on 5 GHz/channel 36 at
-67 dBm, S3 on 2.4 GHz/channel 1 at -71 dBm, both with power save disabled.
Observed radio utilization is 2%/26% respectively. Events show both clients
online at 08:25 and no later association loss or roam through inspection around
08:41, despite TCP failures. This does not prove the RF path is healthy or the
mesh configuration is wrong. C4's background scan configuration is
`simple:30:-70:86400`; its involvement has not been demonstrated. No network
settings were changed. S3 UART was unavailable, so the internal close/reset/USB
history is still missing.

## Validation and deployment

- 45 focused autorun, authenticated Wi-Fi and H.264 tests pass on Windows with
  native Params/platform imports doubled. The new simulated-clock test checks
  that Wi-Fi still waits for model loading and wired USB keeps the extra 10 s.
- Selectively installed only `cluster_autorun.py`, SHA256
  `c1531ce98bb919775565cfb2116ba9fb74bb3f179e4b8cd562605c0d919f63e2`.
  A verified original was backed up; fresh Park/standstill/inactive guards
  surrounded a HUD-only restart, with HUD=1 restored in `finally`.
- Native logs confirm the Wi-Fi path starts without the settling delay. The
  actual HUD reconnects and reports 9.9-10.0 Hz host refresh; optical output
  was not independently checked. Model/DM PIDs remain unchanged, eGPU active.
- The final separate 60 s observation receives 1,199 consecutive valid model
  and DM frames each, at 19.999392 and 20.000896 Hz. No frame IDs are missing.
  This is approximately 20 Hz, not a strict minimum-frequency guarantee.
  An earlier pre-install sample missed one DM observation; it is not a clean
  DM cadence pass and is not silently substituted for the final measurement.

The latest sampled connection is active. The initial TCP failure, overall
low-memory warning, hours of operation and driving stability remain unresolved.
Raw network, memory and router observations remain private. Other C3/eGPU
memory reports require separate analysis of HUD-disabled execution and the
contemporaneous eGPU changes; this recovery-delay fix is not a memory fix.
