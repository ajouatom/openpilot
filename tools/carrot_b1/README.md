# Carrot B1 BLE indicator controller

ESP32-S3 N16R8, Arduino-ESP32 3.3.5, one 500 kbit/s classic CAN bus.
Build and upload from the repository root with Arduino CLI:

    arduino-cli compile --fqbn esp32:esp32:esp32s3:FlashSize=16M,PartitionScheme=app3M_fat9M_16MB,PSRAM=opi --output-dir build-b1 tools/carrot_b1/CarrotB1
    arduino-cli upload -p PORT --fqbn esp32:esp32:esp32s3:FlashSize=16M,PartitionScheme=app3M_fat9M_16MB,PSRAM=opi --input-dir build-b1 tools/carrot_b1/CarrotB1

## Wiring

S3 GPIO4 is TWAI TX, GPIO5 is TWAI RX. Power a TJA1050 from a regulated 5 V
supply and connect logic ground. Use an SN74LVC1G17 at 3.3 V between module
RX and GPIO5. Use an SN74LVC1G07 at 3.3 V between GPIO4 and module TX, with
10 kΩ from GPIO4 to 3.3 V and 2.2 kΩ from its open-drain output to 5 V.
Decouple each buffer with 100 nF. Connect H and L to the matching bus wires.
Only terminate a bus at its two ends. GPIO38 and GPIO48 are both reserved for
the two common onboard RGB LED variants.

## Enroll a board

With Bluetooth enabled on a stationary comma, power up a new board. For three
minutes after startup an unclaimed board accepts a one-time seed over encrypted
GATT. Comma discovers the name Carrot-B1 and the service UUID, pairs the unique
candidate, then proves its key using an authenticated STOP before saving it.
No address or key needs to be typed. If multiple matching units are visible,
power just the intended unit or select it in Carrot Web. A claimed board refuses
replacement keys. The physical UART at 115200 accepts PAIR for recovery and OTA.

The registration selects BLE instead of the Jetson PEAK sender. The separate
native CAN-FD SPAS setting must be off to use the BLE actuator.

Comma uses the existing remote, lane-change and turn policy. The board owns
classic CAN encoding, one-shot transmission, counter reservation and timing. A single shot lost to arbitration can retry the
identical first frame up to 16 times within 40 ms. After its first successful
retry the board times the rest of the tap from that transmission. Later frames
may retry up to eight times within 18 ms and at least 5 ms before the next slot. Other
CAN errors stop the sequence. If a confirmed press already reached the bus,
a fresh, matching lamp-on within the 350 ms observation window may acknowledge
the original request after arbitration exhaustion. The board records
lamp_confirmed_partial and the actual confirmed-frame count separately. It never
starts the whole tap again to manufacture a completion.
A matching car name is not required for B1. Two fresh standard 8-byte streams,
valid 0x3C1 CRC/counter, and neutral lever/idle 0x413 lamp feedback are required
to begin a tap. Address collisions alone cannot establish vehicle compatibility.
Observe stock left and right feedback before a parked first actuation on a new
vehicle. On cars without the existing Ioniq 5 PE continuous hold implementation,
a sustained request can renew the measured OEM one-touch after the last
confirmed flash has been dark for 1.2 seconds. Cancelling the hold stops renewal.
An already accepted OEM one-touch can finish its normal flashes.

Amber indicates power with no matching CAN. Purple means BLE connected but
CAN unmatched. Blue breathing means CAN present, waiting for an operating
lease. Green means matched CAN with a valid control lease. Teal is a tap or
firmware update, red is a CAN fault.

## BLE maintenance

Power the module with the CAN harness disconnected and wait at least five
seconds. Disconnect the normal BLE client. On a BLE-capable PC with Python
and bleak installed, run:

    python ble_update.py BOARD_BLE_ADDRESS build-b1/CarrotB1.ino.bin

Use the application image, not the merged flash dump. The updater prompts for
the board key. The S3 authenticates the image manifest using HMAC-SHA256,
verifies the SHA-256 of received bytes before selecting its inactive 3 MiB
slot, and reboots. For the PC updater, a quiet CAN bus, the same BLE session and sequential offsets
are required throughout the transfer. USB can be used for recovery.

C3 and C4 show an S3 badge near the upper right of the home and driving screens
for an enrolled module. A missing or stale report shows connection waiting.
Purple means BLE connected and authenticated, waiting for matching CAN. Green
means matching CAN is present. The onboard green LED additionally requires an
active control lease.

For an installed board running the parked-update firmware, use the enrolled
comma as the BLE central. Run from /data/openpilot:

    /usr/local/venv/bin/python -m tools.carrot_b1.comma_update /path/to/CarrotB1.ino.bin

The comma checks fresh Park, standstill and disengagement throughout an onroad
transfer, or confirms offroad. The board accepts this separately authenticated
manifest only outside an active CAN transmission and ignores control commands
until the update finishes or aborts. Add --quiet-bus when upgrading an earlier
board that does not yet support the parked manifest.
## Automatic updates after Git Pull

The comma checkout includes CarrotB1.bin. After the usual Git Pull and reboot,
Carrot Web checks the authenticated, registered board even while the car is
moving. Automatic updates wait for the indicator to be idle for four seconds,
with no queued request or hold, then recheck the board state and lamps before
starting. During transfer the S3 automatic indicator is unavailable and the
C3/C4 badge shows an update in progress. Only the S3 reboots after verification.
The manual command above retains its Park guard.
A new release installs over BLE and the running ELF build ID is read after
reconnection. The current build is skipped, newer board releases are preserved
and conflicting builds with the same release number are reported. Failed
transfers back off. A legacy board without the diagnostic characteristic
requires a quiet CAN bus for its first update.

For a release, increment RELEASE_TAG in the sketch, build the application with
the board settings above, and commit its CarrotB1.ino.bin as tools/carrot_b1/CarrotB1.bin
in the same change. The release number and ELF identity are extracted from the
bundled app image. Never package a device flash dump or its enrollment data.
