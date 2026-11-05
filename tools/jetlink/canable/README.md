# CANable 2.0 Carrot SLCAN firmware

This transport requires the pinned `carrot-b1-slcan-2` image; an unmodified
CANable is detected but is not opened for B1 control. It shares the existing
PEAK controller, profile and input guards. Shared first-ready ownership selects
one S3/USB output source, retaining a working owner and delaying failover until
the previous receiver lease has expired. It does not add
support for arbitrary vehicles or arbitrary CAN identifiers.

The source is [normaldotcom/canable2-fw](https://github.com/normaldotcom/canable2-fw)
at the commit in `firmware.json`. The patch preserves silent mode across open,
defaults to no retransmission, clears pending transmissions on close, and adds
`Q` status. The host checks the actual controller mode and waits at most 15 ms
for the individual controller transmission result before accepting a send.
Only confirmed cancellation without protocol errors can retry the same frame,
at most eight attempts sharing that 15 ms deadline with a short random backoff.
Acknowledged, ambiguous, timed-out and protocol-error outcomes never retry.
CAN acknowledgement alone does not prove the vehicle illuminated its lamp.

`Q` returns `Q<mode><result><sequence:4 hex><PSR:8 hex><IR:8 hex><ECR:8 hex>\r`.
Mode is C (closed), L (listen),
T (normal with retransmission disabled), or E (unexpected mode/bus-off).
Result is 0 (idle), 1 (pending), 2 (TXBTO success), 3 (failed/protocol error),
or 4 (TXBCF cancellation without PEA/PED/BO errors). The protocol-error flags
are cleared before each request and remain latched throughout that attempt.
Sequence increments on a submitted transmit command; the host sends only one
frame at a time and validates its sequence. Parent loss closes the CAN channel
through a separate host watchdog. No automatic stock firmware flashing occurs.

`prepare.py NEW_DIRECTORY` prepares hash-checked source and the upstream
MIT-licensed `mpaland/printf` dependency. Source and dependency licenses remain
in that tree. Build with GNU Arm Embedded GCC/newlib. The shipped binary was
built with Ubuntu Jammy arm-none-eabi-gcc; its hash is in `firmware.json`.

For a verified STM32G431 CANable 2.0, disconnect CAN and enter the ROM DFU
bootloader using the board's BOOT button or jumper while reconnecting USB.
Match the serial and internal flash descriptor first. Back up internal flash
before writing. Program only internal flash at `0x08000000`, read the same number
of bytes back and compare. Do not change option bytes, OTP or protection.
Remove USB, restore BOOT to its normal state, then reconnect USB before CAN.
Other CANable revisions/clones require hardware verification first. Stock
`16d0:117e` and this upstream source's `ad50:60c4` USB identities are recognized;
both still require the exact firmware version before any CAN channel opens.
