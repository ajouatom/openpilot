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
