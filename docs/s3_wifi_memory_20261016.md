# S3 status LED and Wi-Fi / HUD memory investigation — 2026-10-16

## Scope

The user requested stopping the Wi-Fi HUD while investigating memory growth and
link loss, then requested an onboard LED for Wi-Fi status. C4 networking remains
enabled. Only the S3 application firmware is installed in this follow-up; no C4
control/model update or full repository deployment is included.

## Firmware 1.1.1

Adds a dim addressable RGB status LED on configurable GPIO48 (some board revisions
use GPIO38). Blue blinks at initial connection, red after Wi-Fi/IP loss, green
indicates IP readiness with a USB display, and recent successful USB image writes
add a brief dark pulse to green. USB absence/failure uses a red double blink when
Wi-Fi is ready. Purple phone-setup indication takes priority. LED failure stops
only its task. It uses a fixed stack and fixed RMT allocation, without per-frame
heap allocation.

Adds the read-only UART `S3STATUS` command: uptime/reset reason, association/IP,
RSSI/channel, disconnect reason/count and internal free/minimum heap. No password
or pairing key is included. Wi-Fi disconnect and lost-IP events are logged. This
release does not change the reconnection policy or claim to repair link loss.

Validation:

- ESP-IDF 5.4 builds the application (891744 bytes); LED source compiles without
  the initial empty-struct initializer warning.
- Standalone C assertions pass with `-Wall -Wextra -Werror`, covering state
  precedence, blink boundaries, stale/future frame times and bounded brightness.
- UART device identity and pairing are checked before flashing. Private NVS is
  backed up; only the app at `0x10000` is written; pairing remains identical.
- Installed boot log reports 1.1.1, GPIO48 and confirmed panel idle-off.
- Three PC-driven phases each acknowledge 50 JPEG frames in 5 seconds: normal
  close, abrupt connection loss, and reconnect. Idle-off after the abrupt loss
  is confirmed by panel command response. These are USB acknowledgements, not
  an optical panel FPS measurement or C4 rendering test.
- Actual LED color/pin confirmation from the user remains pending at publication.

The subsequent continuous PC test acknowledged all 1,200 JPEG frames in 120.029
seconds (9.998 Hz). Fourteen UART snapshots remained associated on channel 6 at
-40 to -43 dBm with zero disconnects. Internal free heap varied with traffic
(lowest sampled 112,496 bytes; lifetime minimum 106,236), then returned to exactly
182,104 bytes after close, matching the earlier idle snapshot. No Wi-Fi-loss,
USB-timeout, LED-driver-stop or panic line appeared. This is a two-minute synthetic
bench check under the improved association, not long-duration/vehicle validation
or proof that the earlier Wi-Fi failure is fixed.

## Memory findings

With fresh valid Park/standstill/inactive samples, `ClusterHud` was changed from
1 to 0. HUD termination and absence of restart were checked. NetworkManager was
not stopped. Available memory rose from 276.4 MiB in the detailed before snapshot
to 432.7 MiB in the later after snapshot, and subsequently about 499.8 MiB.

The stopped HUD process had approximately 140 MiB PSS / 187 MiB RSS. Its 510
orphan native ION objects totaling 141,004,800 bytes remained after termination.
Thus stopping the process recovers process memory but does not reclaim those
native buffers. This is not a memory fix or evidence that the system is normal.
The current captured kernel output contained no OOM-killer report; this does not
exclude an earlier event or identify the user's reported memory error.

Installed comma raylib imports GBM device destruction/release but no GBM surface
destruction. Official current platform source also lacks several matching cleanup
operations. That warrants a matching-build native cleanup investigation; it is
not yet a verified allocation ownership explanation or deployed correction.
An isolated headless-backend trial exited with SIGSEGV during initialization,
before readiness. It was not enabled in production and is not a workaround.

## Link findings

Before the S3 reflash, C4 could initially discover the S3 but even 512-byte
brightness commands became slow and timed out without rendering or image traffic.
The later small/large-image comparisons failed before sending images. PC-to-S3
authentication also failed, excluding an exclusively C4-sender failure. C4-to-AP
gateway ping had 12.5% loss and 69 ms mean in one capture, while S3 lost all 40
pings. Company Wi-Fi therefore cannot be excluded.

UART showed the S3 software and periodic panel idle-off remained alive. Following
the separately authorized S3 reset, channel 6 association was temporarily refused;
the S3 then selected a channel 1 AP at -72 dBm and obtained an IP. PC discovery and
authentication still timed out. Omada displayed different AP associations and a
mesh uplink. No Omada settings were changed in this follow-up. The association
refusal and weak alternate AP are clues, not proof of the complete root cause.

After flashing 1.1.1, the same saved SSID connected on channel 6 at -38 dBm.
All 24 small USB commands succeeded across three authenticated connections,
followed by the 150-frame test above. AP selection and signal changed along with
the reboot, so the improvement cannot be attributed to LED/diagnostic code.
Long-duration stability and a controlled repeat under the failing conditions
remain unverified. Raw UART, pairing, NVS, vehicle and network captures stay private.

The C4 HUD remains disabled to avoid further native buffer accumulation. Do not
call process cycling a memory fix, enable the failed headless backend, or reinstall
unrelated control changes as part of this S3 firmware work.
