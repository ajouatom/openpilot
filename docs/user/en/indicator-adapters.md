# Physical indicator adapters: support and installation

Choose a device to open its photographs, wiring, firmware installer and checks. For a first build without soldering, start with **CANable 2.0 and the Windows installer**. USB CAN adapters connect to Jetson; the S3 connects to comma over BLE.

| Device and guide | Connection | Supported scope |
|---|---|---|
| **[Install CANable 2.0](canable.md)** | Jetson USB | MKS V2.0 / V2.0 S standard STM32G431 boards with the verified DFU layout. Carrot firmware required; parked left/right lamp test completed. |
| **[Build an S3 BLE adapter](s3-can.md)** | comma BLE | ESP32-S3 DevKitC-1 N16R8 plus SN65HVD230 3.3 V. Four soldered interconnects. |
| **[Connect PEAK PCAN-USB](peak-can.md)** | Jetson USB | Original classic PCAN-USB, USB `0c72:000c`. Keep its manufacturer firmware. |

## When several adapters are connected

The first S3, PEAK or CANable that establishes its connection and receives matching CAN data owns output. An existing S3 enrollment no longer excludes USB adapters. A newly connected adapter does not preempt a working owner.

When the selected adapter disconnects or loses matching CAN, commands stop and another ready adapter may take over. A transition delay lets the previous receiver's command lease expire. In-flight commands are not copied and replayed on the replacement. Switching is not seamless. The separate vehicle **CAN-FD SPAS** option retains its existing vehicle output path when enabled.

## Vehicle compatibility is a separate requirement

The current PEAK/CANable profile is the **Ioniq 5 PE, classic CAN at 500 kbit/s, 0x3C1 and 0x413** profile. S3 also checks message layout, state and CRC; matching numerical IDs alone do not establish compatibility with another vehicle.

Use a verified vehicle CAN H/L connection point. An OBD connector or a bus name such as E-CAN/C-CAN does not establish pin assignments. Check the stock stalk first, then test both directions while parked in P.

CANable V1.0, different-MCU clones, CANable Pro, PCAN-USB FD/Pro and other USB CAN products have not been validated for this installer. The lower Pro board in the CANable photograph is a location comparison, not an installation approval.

[Open the CANable PC/Jetson guide](canable.md) · [한국어](../ko/indicator-adapters.md)

## Compatibility selected by CAN protocol

There is no vehicle-model allowlist. USB, BLE and Panda/SPAS outputs use the same manual-steering, model-intent and ATC indication settings, retaining CANCEL, speed, lane and driver-priority conditions. Only a matching output protocol may transmit.

- **USB/BLE:** the existing B1 classic CAN profile requires 500 kbit/s, eight-byte `0x3C1` and `0x413`, matching signal meanings, CRC and fresh reception. An adapter's CAN-FD hardware support does not extend this control profile.
- **Kicker/Panda:** camera-SCC wiring must expose ECAN SPAS `0x165` (24 bytes), `0x16A` (32 bytes) and stalk `0x3C1` (8 bytes). A new request requires ten CRC-valid advancing stock SPAS frames and a fresh neutral stalk. Selecting Kicker continues to suspend external USB/BLE output.
- **Kicker without a lever (test):** with `CanfdBlinkerKickNoLever` also on, the Kicker runs on cars with the same SPAS messages but no `0x3C1` lever (Tucson HDA1). It checks the own-lamp record and lamp isolation instead of a lever.
- **Holding:** comma maintains an indication when camera-side `0x1EA` (32 bytes) is observed and its existing transmit path is enabled. Otherwise BLE uses S3 holding; USB retains one-touch and relight behavior.

The reported EV4 `0x3E3` CAN-FD+BRS, 16-byte format is a separate protocol. Removing model-name restrictions does not add transmission support for it. Verify physical illumination, holding and release separately on a newly supported vehicle. Apply the comma update and restart; this change requires no S3 or CANable firmware reinstall.
