# CAN-FD camera feedback counters

## Change (2026-09-27)

Camera-SCC MDPS and TCS feedback now remove `COUNTER` from the copied input
values and pass it as `rx_counter` to the existing CANPacker. The first emitted
counter is the received counter plus one; subsequent transmissions advance the
per-message counter modulo 256, independently of repeated or skipped input
counter values. Missing snapshots produce no message and do not advance it.
The packer recalculates the checksum after assigning the outgoing counter.

This preserves the existing transmission cadence, buses, feedback field
modifications, and original receive snapshots. It does not implement a queue
that forwards every received frame exactly once: feedback still uses the latest
snapshot at each scheduled transmission. MDPS and TCS have separate sequences.
Button-message forwarding is outside this change.

## Evidence and limits

A 60-second GV70 capture on c04fa456 had consecutive original MDPS counters,
but camera-bound MDPS contained 254 repeated counters and 253 increments of two.
TCS counters were consecutive in that capture; its identical explicit-counter
copying pattern is corrected as well. A camera-side warning and LFA state change
were recorded, but causality between the warning and counter behavior is not
established. This change is not a demonstrated vehicle warning fix.

`test_canfd_feedback_counters.py` checks repeated/skipped input counters,
wraparound, absent snapshots, independent MDPS/TCS sequences, source immutability,
unchanged payload behavior, and CRC with an independent CRC implementation.
All three focused tests passed on Windows with only the unavailable native Params
module stubbed. The combined packer/parser run passed 19 tests and failed the
existing `test_parser_can_valid` initial-validity assertion; that failure also
reproduced using the unchanged HEAD Hyundai message implementation.
Vehicle acceptance and warning recurrence require on-device validation.
