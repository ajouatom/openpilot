# ESP32-S3 Wi-Fi HUD bridge

The C4 renders the existing complete HUD, including camera/map overlays, and sends
H.264/JPEG TURZX command packets over Wi-Fi. The S3 only authenticates and forwards USB
transactions. UI changes do not require a new S3 firmware. There is no OTA updater.
Future bridge bug fixes can still require a UART flash; this is not a promise that
firmware will never need maintenance.

Supported hardware: ESP32-S3 N16R8 (8 MB octal PSRAM), a separately powered USB hub,
and TURZX **9.2-inch 1cbe:0092**, 1920×462 landscape. Connect the hub upstream to the
native **USB** connector beside RESET. The **UART** connector beside BOOT is for
power/initial registration. A bare C-to-C connection does not supply host VBUS on
the tested board. This is USB data forwarding, not DisplayPort/USB-C video output.

When several access points advertise the registered SSID, firmware 1.2.0 scans
all channels and selects by signal strength when connecting. It does not pin an
AP or continuously roam after association. UART `S3STATUS` includes the connected
BSSID, channel and RSSI for diagnosis.

Firmware 1.2.1 bounds association attempts to 30 seconds and IP acquisition to
20 seconds before rejoining, with up to five seconds between failed attempts.
These are phase deadlines, not an end-to-end reconnection guarantee. It also
protects the initial sync/setup window from idle-off and invalidates the TCP
stream when idle-off resets the panel decoder. The two-second visual idle/off
policy is retained. `S3STATUS` reports `wifi_recoveries`; UART logs distinguish
RX/TX termination, Wi-Fi recovery, and local decoder reset without credentials.
Local UART `S3RECONNECT` retries the saved network without rebooting or rewriting
credentials. It interrupts the display briefly; it is not a network command.

## One-time firmware installation

For ordinary users, use the [NAS browser installer and Korean setup guide](https://upload.shind0.synology.me/models/carrot-s3/index.html).
It includes the N16R8 board checklist, wiring diagram, initial setup and a firmware
ZIP download. Chrome/Edge on a PC can install without Python or ESP-IDF. The update
option validates the existing partition table, writes only the app, and verifies
that NVS is unchanged. Version 1.1.0 fixes confirmed-off/retry handling.
See also [INSTALL_KO.md](INSTALL_KO.md). The commands below are for source builds.

Use ESP-IDF **5.4.0**, target esp32s3, from `firmware/`:

```sh
idf.py set-target esp32s3
idf.py build
idf.py -p /dev/ttyUSB0 flash
```

The supplied partition table has a single factory app and NVS; no OTA partitions.
Do not erase NVS during later UART reflashing if registrations must be retained.
Hold BOOT for three seconds **after startup** to reopen phone setup for ten minutes.
Holding BOOT during power-on instead enters the chip's ROM download mode.

## Register from C4 over USB

Connect C4 to the intended **2.4 GHz** hotspot first, then connect S3's UART USB
connector to C4. Run from the openpilot repository:

```sh
python3 tools/esp32s3_display/provision.py --auto
```

The utility reads the active NetworkManager Wi-Fi profile and password, verifies
the UART bridge identity, writes credentials to S3 NVS, and saves the device ID and
pairing key in `/data/carrot/cluster_wifi.json`. It does not save the Wi-Fi password
in that file or print it. Reading the active profile's secrets may require root.
Auto selects only when exactly one supported USB UART is present, then verifies
the S3 identity before writing. Otherwise select the actual UART with `--port`;
the utility never probes or resets other serial devices.
This works while C4 is offroad. Plugging in the cable alone does not run registration:
run the command again after changing C4's active Wi-Fi to copy the new credentials.

For manual setup on a PC or a C4 without readable NetworkManager credentials:

```sh
python3 tools/esp32s3_display/provision.py --port COM14 --ssid "My hotspot" --output carrot-s3.json
```

The password is prompted without echo. Import that registration file on C4 with
the command below. Never commit or share a registration file.

## Register from a phone

1. On first startup, or after holding BOOT for three seconds, join setup Wi-Fi
   **`CARROT-HUD`**, password **`12345678`**, also shown on the panel. Configure one
   board at a time when several are nearby. These deliberately simple credentials
   apply only during temporary physical setup; normal C4 pairing uses a random key.
2. Join that network and open **http://192.168.4.1**, or use the network sign-in
   prompt if shown. No app or Internet connection is required. If the phone routes
   this page over mobile data, select “stay connected without Internet” or briefly
   turn mobile data off. Use another phone if yours cannot run its hotspot while joining Wi-Fi.
3. Enter the target 2.4 GHz Wi-Fi name/password and select **Wi-Fi 저장**. Saving
   does not require a file download. For an already registered C4, keep its existing
   registration and simply re-enable the target hotspot. Setup closes 15 seconds later.
4. For the first C4 registration, select **처음 C4 연결: 등록 파일 받기** after saving.
   This downloads `carrot-s3-<timestamp>.json` with the device ID/pairing key, not the
   hotspot password. Every click uses a new filename so existing downloads need not
   be overwritten; the downloaded blob remains available after setup closes.
5. Transfer the downloaded registration file to C4 and import it once:

```sh
python3 tools/esp32s3_display/provision.py --import-pairing /path/to/carrot-s3.json
```

Both registration methods preserve the existing pairing key when changing SSID.
Updating only the Wi-Fi name/password therefore does not require re-pairing C4.
The phone page is accessible only through the temporary S3 access point. It is
closed on the normal hotspot/LAN, and does not reopen automatically on link loss.

## Run and reconnect

Enable the existing 9.2-inch `ClusterHud` setting. With a registration file present,
the existing HUD service discovers that S3 using authenticated UDP on port 3333,
then sends packets over authenticated TCP on the same port. C4 and S3 must share
a network that allows communication between clients; some hotspots isolate clients.
No fixed IP, Internet server, Bluetooth, or PC relay is needed in the vehicle.
Normal HUD output retains the existing onroad gate; use `ClusterHudDebug` for
offroad display testing. Wi-Fi registration and S3 reconnection do not require onroad.

The S3 retries Wi-Fi with a bounded 1–30 second backoff. C4's existing supervisor
restarts failed display connections and discovers the current IP again. S3 stores
credentials across power cycles, discards prior-session/queued-old packets, and
turns the backlight off after two seconds without a successful image. A stalled
USB transaction can add its bounded timeout to that interval. USB reconnect is
handled by re-enumeration. Reopening setup temporarily replaces the driving display.

C4 also detects registration saved while it is waiting for a USB display. The
Wi-Fi connection starts after graphics initialization so that startup does not
exceed the bridge's two-second idle timeout. These host fixes require updated C4
code, not a new S3 firmware.

Updated C4 code automatically uses H.264 when the paired S3 authenticates its
firmware 1.2.0 capability. C4 encodes the video; the TURZX display decodes it.
S3 does not decode or re-encode video. The default H.264 target is 1 Mbps with
a keyframe every 10 frames. Missing capability replies or older firmware retain
JPEG (default quality 50). A lost capability query can therefore select JPEG
until the next HUD session. `ClusterHudEncoder` selects the H.264 hardware/software
backend; it does not override the Wi-Fi codec selection.
Auto encoder initialization fallback can still reach JPEG when neither H.264
backend starts; an explicitly selected `codec: h264` does not allow that fallback.

The registration JSON optionally accepts `codec`: `auto` (default), `jpeg`, or
`h264`. Explicit H.264 requires a verified capability reply. Integer `fps` (1–20)
and JPEG-only `quality` (20–90) apply on the next HUD start. The default target
is 15 FPS, capped at 10 FPS with eGPU active; direct USB retains its 5 FPS cap.
Lower configured rates still apply. Brightness, layout, camera, mirror and
orientation use the existing renderer. H.264 reference frames stay ordered in
bounded queues; sustained congestion restarts the stream instead of dropping
dependent frames. Logs summarize H.264 bitrate and send delays every 10 seconds.
Remove the registration file and restart HUD to return to direct USB behavior.
`CARROT_S3_CONFIG` can select an isolated file for desktop tests.

## Board status LED and UART diagnostics (1.1.1)

The addressable RGB LED uses a low brightness of 3/255 per channel. Blue blinks
while initially connecting, red blinks after Wi-Fi/IP loss, and green stays on
after receiving an IP with a USB display present. Recent successful image
transactions add a short dark pulse to green once per second. With Wi-Fi ready,
missing/failed USB uses a red double blink. Phone setup overrides these states
with purple blinking. Green means IP readiness, not proven C4/Internet reachability;
image activity means USB transfer success, not optical panel verification.

`CONFIG_S3_STATUS_LED_GPIO` defaults to GPIO48; some DevKitC-style revisions use
GPIO38. Select the actual board's RGB LED pin through `idf.py menuconfig` before
building. The separate power LED is not controlled. LED driver initialization or
transmission failure logs a warning and stops only the LED task.

On the UART port at 115200 baud, send `S3STATUS` followed by LF. The `S3JSON` reply
contains uptime, reset reason, association/IP state, RSSI/channel, disconnect
count/reason, and current/minimum internal heap. It contains no Wi-Fi password or
pairing key. Disconnect/IP-loss events also appear in the normal UART log. These
are diagnostics, not a new network recovery policy.

The pure LED policy can be checked without ESP-IDF:

```sh
cc -std=c11 -Wall -Wextra -Werror tools/esp32s3_display/tests/test_status_led.c -o /tmp/test_status_led
/tmp/test_status_led
```

## Protocol and boundaries

- Device ID: six-byte Wi-Fi MAC. Pairing key: random 32 bytes in S3 NVS and the C4
  registration file (created with mode 0600 on Linux).
- UDP discovery: `S3D1` / `S3R1`, device ID, 16-byte nonce, HMAC-SHA256.
- TCP hello: `S3H1`, device ID, random 32-byte challenge. Mutual HMAC proofs use
  a fresh client nonce; a derived session key authenticates each packet and ACK.
- Packet header: six network-order uint32 values (magic, sequence, length,
  expect-reply, zero, zero), HMAC, then 512..1048576 bytes of TURZX data.
- ACK header: magic, sequence, USB response length (0..512), status, USB microseconds,
  zero; HMAC then raw USB response. Auth failure closes the session.
- Three PSRAM packet slots plus the existing latest-image renderer queue bound
  memory/backlog. No vehicle control messages, discovery credentials, or firmware
  upload commands exist in this transport.

HMAC authenticates but does not encrypt image contents. Use the private WPA2 hotspot;
the firmware does not enable flash encryption and physical flash access can expose
stored credentials. The phone registration file authorizes access to that display.

## Validation

Version 1.1.0 initializes the panel before local shutdown, waits 200 ms, and only
marks it dark after the brightness command succeeds. Failed local shutdowns retry
after 1/2/4/8/16 seconds; idle panels are reasserted off every 30 seconds to recover
a powered-panel reboot. This is backlight off, not removal of USB power. Setup
screens remain visible. C4 also includes registered S3 displays in offroad dimming
and confirms shutdown on connection close. First image wakes the panel again.

The October 15 native C4 test corrected a missing registration, an AP channel
latency problem, and the graphics-startup idle disconnect. The existing camera HUD
then rendered at typically 9–13 Hz, with model and driver monitoring at 20 Hz in
the initial parked check. This is renderer cadence, not an optical panel FPS
measurement or road validation. See the
[field investigation](../../docs/s3_wifi_egpu_investigation_20261015.md) for evidence
and remaining C4 network latency. The following numbers are the earlier PC bench.

The final authenticated bridge was exercised on the physical S3, powered hub and
panel. Phone setup/save/download, importing that file, UART credential replacement
with the same pairing key, and reset-to-saved-hotspot reconnection passed. Real UDP
discovery signatures were verified; the normal STA HTTP endpoint returned 403 and
an incorrect TCP pairing key was rejected. A subsequent valid connection recovered.
At a 15 FPS target, 181 synthetic JPEG frames were acknowledged in 12.10 seconds
(14.96 FPS; median USB transaction 44.90 ms). At a 20 FPS target, 216 frames took
12.25 seconds (17.63 FPS; median USB transaction 47.22 ms), so 20 FPS is not guaranteed.
The existing production HUD renderer
also ran with desktop simulated inputs at roughly 13–15 Hz. These are bench results,
not a C4 installation, native C4 performance measurement, or vehicle-camera test.
Host transport/display/rate checks passed 38 tests; existing autorun/performance
checks passed another 38 with one Linux-only skip under desktop Params/hardware
stubs. ESP-IDF 5.4.0 built the installed 873472-byte application successfully.
The subsequent setup-page correction separates Wi-Fi save from optional download
and assigns a timestamp filename on each download click. Its 873584-byte firmware
was built, flashed and checked for preserved pairing; repeat-download phone UI
confirmation is separate from the earlier phone save/download check.

The earlier hardware bench achieved about 15 FPS at JPEG quality 75 and 20 FPS at
quality 50 with a simple synthetic HUD. Detailed synthetic content reached about
16 FPS. These are USB acknowledgement rates, not optical refresh measurements or
actual C4 camera rendering/vehicle validation. The bench's PC used an ADB phone
relay because it had no Wi-Fi adapter; that relay is not part of this implementation.

Run focused host tests with:

```sh
pytest openpilot/selfdrive/carrot/tests/test_cluster_wifi.py
```

Official APIs used: [ESP-IDF HTTP server](https://docs.espressif.com/projects/esp-idf/en/v5.4/esp32s3/api-reference/protocols/esp_http_server.html).
