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

On its physical UART at 115200, enter `PAIR` followed by newline to read its
per-device enrollment key. Treat this value as a secret. With the vehicle
stationary, open Carrot Web → Tools → Bluetooth remotes, enable Bluetooth, scan,
select Carrot-B1 by its address and choose B1 auto-connect. Enter the UART key.
The registration selects BLE instead of the Jetson PEAK sender. The separate
native CAN-FD SPAS setting must be off to use the BLE actuator.

Comma uses the existing remote, lane-change and turn policy. The board owns
classic CAN encoding, one-shot transmission, counter reservation and timing.
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
slot, and reboots. A quiet CAN bus, the same BLE session and sequential offsets
are required throughout the transfer. USB can be used for recovery.
