# Connect PEAK PCAN-USB

[Back to supported adapters](indicator-adapters.md)

Connect the **original classic PCAN-USB, USB 0c72:000c**, to Jetson. PCAN-USB FD and Pro are different devices. **Never flash CANable firmware onto PEAK.**

1. Git Pull the supported carrot-wip release and restart comma and Jetson. The supported Jetson image prepares the existing PEAK driver.
2. With power off, connect the verified vehicle CAN H/L and reference ground. Preserve the same connector and bus if reusing a working PCAN harness. Consult the [official PEAK PCAN-USB documentation](https://www.peak-system.com/PCAN-USB.199.0.html?&L=1) for product-specific connector assignments.
3. Attach PCAN-USB to Jetson USB. It waits if another S3 or USB CAN adapter already owns output; the first ready adapter is selected.
4. While parked in P, check remote left/right commands against the actual vehicle lamps.

The current USB profile is Ioniq 5 PE classic CAN at 500 kbit/s using 0x3C1 and 0x413. Receiving CAN from another vehicle does not establish indicator-control compatibility.

Keep the manufacturer's PEAK firmware. Neither the CANable nor the S3 firmware installer is needed.

[한국어](../ko/peak-can.md)
