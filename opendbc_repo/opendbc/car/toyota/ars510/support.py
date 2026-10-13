"""Support messages: the radar's ACC target (0x235 / 0x237 / 0x239 / 0x23B), the cycle header (0x190), the
selected-target summaries (0x191-0x194), the single-object stream (0x680), and car-bus target 0x366; docs/05 and 13.

0x192 / 0x194 (4 bytes, ~radar cycle) each carry the position of one target of the radar's internal tracker in the
object list's encoding: word 0 distance (code - 160) / 16 m, word 1 lateral (code - 2048) * LATERAL_M_PER_CODE m, left
positive. The whole-frame sentinel is 00 FF 00 FF. Use a parsed position together with the ACC target's availability
and association (docs/05).
"""
from __future__ import annotations

from dataclasses import dataclass

from .objects import LATERAL_M_PER_CODE

A192_SENTINEL = bytes.fromhex("00FF00FF")
A680_IDLE = bytes.fromhex("000008008000800A")


@dataclass(frozen=True)
class Target192:
    range_code13: int
    field1_code13: int

    @property
    def d_rel(self) -> float:
        """Distance in m: the object list's encoding, (code - 160) / 16."""
        return (self.range_code13 - 160) / 16

    @property
    def y_rel(self) -> float:
        """Lateral position in m, left positive: (code - 2048) * 0.015, the object list's encoding."""
        return (self.field1_code13 - 2048) * LATERAL_M_PER_CODE


def parse_0x192(data: bytes) -> Target192 | None:
    """Raw 13-bit summary words; Target192.d_rel / .y_rel give metres in the object list's encoding."""
    if len(data) < 4 or bytes(data[:4]) == A192_SENTINEL:
        return None
    return Target192(int.from_bytes(data[0:2], "big") & 0x1FFF,
                     int.from_bytes(data[2:4], "big") & 0x1FFF)


A191_SENTINEL = bytes.fromhex("FEFEFEFCFCFFFEFF")


@dataclass(frozen=True)
class CycleHeader:
    active_summaries: int  # 0-2: how many of the 0x191/0x192 and 0x193/0x194 pairs carry a target (0x191 fills first)
    timestamp_us: int      # radar cycle clock, microseconds (32 bits)
    counter: int           # mod-16 counter, +3 per cycle


def parse_0x190(data: bytes) -> CycleHeader | None:
    """0x190 (7 bytes, once per radar cycle): summary count, microsecond timestamp, cycle counter (docs/05).

    Byte 0 = count << 2 | 2; the count equals the number of populated 0x191 / 0x193 frames in every logged cycle after start-up
    (681,864 of 681,879; the rest are startup frames).
    Returns None for a short frame or the all-ones start-up frame."""
    if len(data) < 7 or data[0] == 0xFF:
        return None
    return CycleHeader(data[0] >> 2, int.from_bytes(data[2:6], "big"), data[6] >> 4)


@dataclass(frozen=True)
class Target191:
    score: int       # 91-100 while a target is present
    age: int         # radar cycles, saturates at 126
    track_code: int  # pair-local target code 0-63
    width_m: float   # class template: 1.8 car, 2.2 truck
    height_m: float  # class template: 1.5 car, 2.3 truck
    length_m: float  # class template: 4.5 car, 12.0 truck
    quality: int     # counter that grows with age and saturates at 30


def parse_0x191(data: bytes) -> Target191 | None:
    """0x191 / 0x193 (8 bytes): companion of the 0x192 / 0x194 target summary, or None for the sentinel (docs/05).

    The three size fields are a class template in 0.1 m (18:15:45 for cars, 22:23:120 for trucks on 99.97 % of
    active frames, 607,999 of 608,186), the same class the ACC target reports on 0x239 when it is this target."""
    if len(data) != 8 or bytes(data) == A191_SENTINEL or data[0] >> 1 == 127:
        return None
    return Target191(data[0] >> 1, data[1] >> 1, data[3] >> 2, (data[4] >> 2) * 0.1, (data[6] >> 1) * 0.1, data[7] * 0.1,
                     data[5] >> 3)


@dataclass(frozen=True)
class Object680:
    selector: int        # byte 0: 0 / 1 mostly stationary objects, 3 mostly same-direction vehicles
    d_rel: float         # m, longitudinal distance, 1/32 m per code
    y_rel: float         # m, left positive, 0.015 m per code (the object list's lateral unit)
    v_ground: float      # m/s over ground, nominal 0.15 m/s per code (0 for a stationary object)
    flags: int           # motion flags: bit 0 stationary, bit 1 oncoming, bit 3 seen moving; bits 2, 4, 5 raw
    lat_speed_code: int  # centred lateral speed over ground, about 0.13-0.18 m/s per code (0 on stationary objects)

    @property
    def stationary(self) -> bool:
        """Flag bit 0: set on 99.2 % of objects slower than 0.5 m/s over ground and on 0.7 % of moving ones."""
        return bool(self.flags & 0x01)

    @property
    def oncoming(self) -> bool:
        """Flag bit 1: set on 98.5 % of objects faster than 3 m/s toward ego and on 0.7 % of the others."""
        return bool(self.flags & 0x02)

    @property
    def seen_moving(self) -> bool:
        """Flag bit 3: a vehicle that moves or has moved (86 % of moving objects; with bit 0 a stopped vehicle)."""
        return bool(self.flags & 0x08)


def parse_0x680(data: bytes) -> Object680 | None:
    """0x680 (8 bytes, 2 Hz): one object from the radar's function-level tracker, or None when idle (docs/05).

    Mostly a stationary roadside object, which the 0x80 object list mostly drops while driving; sometimes a vehicle.
    Distance scale is fixed by stationary objects closing at ego speed; the speed unit agrees with the ACC target
    (0.09 m/s median absolute difference on shared targets). An analysis helper; the driving interface reads 0x235 / 0x237."""
    if len(data) != 8:
        return None
    rng, lat = _be_field(data, 43, 13), _be_field(data, 32, 11)
    if rng <= 1 and lat == 0:
        return None
    return Object680(data[0], rng / 32, (lat - 2048 if lat >= 1024 else lat) * LATERAL_M_PER_CODE,
                     (_be_field(data, 22, 10) - 512) * 0.15, _be_field(data, 16, 6), _be_field(data, 8, 8) - 128)


def _be_field(data: bytes, start: int, length: int) -> int:
    """Bit field of an 8-byte frame read as one big-endian integer (bit 0 = LSB of the last byte)."""
    return (int.from_bytes(bytes(data[:8]).ljust(8, b"\x00"), "big") >> start) & ((1 << length) - 1)


def parse_acc_target_vrel(data: bytes) -> float | None:
    """0x235: closing speed of the radar's ACC target, m/s (negative = closing).

    Bits 29..39, offset 1024, 0.125 m/s per code. The unit is fixed by stopped lead vehicles (true closing speed =
    ego speed) and by the slope of the target's own range code: speed / range rate = 0.99 on 1,862 windows (docs/05).
    """
    if len(data) < 8:
        return None
    return (_be_field(data, 29, 11) - 1024) * 0.125


def parse_acc_target_arel(data: bytes) -> float | None:
    """0x235 byte 2: relative acceleration of the radar's ACC target, m/s^2 (positive = opening).

    (byte 2 - 100) * 0.125; the idle payload's 0x64 decodes to 0. The unit follows from the frame's own closing
    speed: over 2 s windows the speed code changes by 1.00 times the integral of this code (0.997 on 222,050 windows,
    1.002 on other drives), so one code is one speed code (0.125 m/s) per second. Meaningful only while the target
    is active (docs/05).
    """
    if len(data) < 8:
        return None
    return (data[2] - 100) * 0.125


def parse_acc_target_position(data: bytes) -> tuple[float, float] | None:
    """0x237: (distance m, lateral m left positive) of the radar's ACC target, as the track association uses them.

    Distance: bits 39..51 (13 bits), code * 0.025 m (parse_acc_target_range_code returns the raw code).
    Lateral: bits 27..38 (12 bits), (code - 2000) * 0.01 m; one object-list lateral code is 1.5 of these codes (1.499-1.502).
    """
    if len(data) < 8:
        return None
    return _be_field(data, 39, 13) * 0.025, (_be_field(data, 27, 12) - 2000) * 0.01


def parse_acc_target_range_code(data: bytes) -> int | None:
    """0x237 BE39|13 distance code of the radar's ACC target: distance = 0.025 m x code, zero offset.

    Against the 0x680 object range the fit is 0.02500 m per code, +0.01 m, 0.02 m median residual (docs/05). Read it
    after checking target availability (0x235 state 5); `parse_acc_target` returns the full target.
    """
    if len(data) < 8:
        return None
    return _be_field(data, 39, 13)


@dataclass(frozen=True)
class AccTarget:
    target_id: int        # 0x235 bits 0-4: constant while the radar follows one target
    d_rel: float          # m, 0.025 m per code
    y_rel: float          # m, left positive, 12-bit (code - 2000) * 0.01
    v_rel: float          # m/s, negative = closing
    a_rel: float          # m/s^2, positive = opening
    v_lat: float          # m/s, left positive, about 0.0125 m/s per code
    a_lat: float          # m/s^2, relative lateral acceleration, (code - 100) * 0.125: mostly minus ego's own
    in_path: bool         # in-path confirmed (93.6 % of target frames)
    in_path_level: int    # 5 confirmed, then 4, 3, 2 as the target moves off the path
    tracker_codes: tuple[int, int]  # two raw tracker bytes that rise while the target accelerates or brakes


def parse_acc_target(data235: bytes, data237: bytes) -> AccTarget | None:
    """Every field of the radar's ACC target from one 0x235 / 0x237 pair, or None while the target is idle (docs/05).

    Field boundaries come from the carry structure of 2 M frames; while the target is idle every numeric field sits at its zero
    code. The driving interface keeps using the single-field helpers below."""
    if len(data235) != 8 or len(data237) != 8 or data235[1] & 0x0F != 5 or not _be_field(data237, 10, 1):
        return None
    return AccTarget(target_id=_be_field(data235, 0, 5), d_rel=_be_field(data237, 39, 13) * 0.025,
                     y_rel=(_be_field(data237, 27, 12) - 2000) * 0.01, v_rel=(_be_field(data235, 29, 11) - 1024) * 0.125,
                     a_rel=(data235[2] - 100) * 0.125, v_lat=(_be_field(data235, 8, 11) - 1024) * 0.0125,
                     a_lat=(_be_field(data235, 19, 8) - 100) * 0.125, in_path=bool(_be_field(data237, 8, 1)),
                     in_path_level=_be_field(data237, 4, 3), tracker_codes=(_be_field(data237, 19, 8), _be_field(data237, 11, 8)))


@dataclass(frozen=True)
class AccTargetInfo:
    target_class: int     # 1 car, 2 truck (the 0x191 template class); 0, 4, 5, 6 other; 7 idle
    present: bool
    in_object_list: bool  # the target also has an object in the 0x80 list
    timestamp_us: int     # radar cycle clock of this target update, same clock as 0x190


def parse_0x239(data: bytes) -> AccTargetInfo | None:
    """0x239 (8 bytes, 50 Hz): class, object-list flag and cycle timestamp of the radar's ACC target (docs/05).

    The timestamp is 32 bits starting 3 bits into byte 3 and equals the 0x190 timestamp of the cycle the 0x235 / 0x237
    content belongs to. Returns None for a short frame or the all-ones start-up frame."""
    if len(data) != 8 or data[1] & 0x0F == 0x0F:
        return None
    return AccTargetInfo(data[1] & 0x0F, bool(data[3] & 0x08), bool(data[3] & 0x80), _be_field(data, 3, 32))


# ---------------------------------------------------------------- 0x23B: target width, counter, CRC-8
A23B_CRC_XOROUT = 0x59


def crc8_0x1d_bits(bits) -> int:
    """CRC-8, polynomial 0x1D (x^8+x^4+x^3+x^2+1), MSB first, init 0, direct bit order in and out, over an iterable of bits."""
    crc = 0
    for bit in bits:
        crc = ((crc << 1) & 0xFF) ^ ((((crc >> 7) & 1) ^ bit) * 0x1D)
    return crc


def crc_0x23b(value: int, counter_and_low: int) -> int:
    """Check byte of 0x23B: CRC-8/0x1D over [counter 4 bits][0000][low nibble of byte 1][value byte 2 8 bits], xor 0x59."""
    bits = [(counter_and_low >> (7 - i)) & 1 for i in range(4)] + [0] * 4 + [(counter_and_low >> (3 - i)) & 1 for i in range(4)] \
        + [(value >> (7 - i)) & 1 for i in range(8)]
    return crc8_0x1d_bits(bits) ^ A23B_CRC_XOROUT


def parse_0x23b(data: bytes) -> tuple[int, int, bool] | None:
    """0x23B (3 bytes, 50 Hz), in frame order as received: (ACC target width in cm, rolling counter 0-15, check byte valid).

    Byte 0 = CRC-8, byte 1 = counter << 4 | width high nibble, byte 2 = width low byte. The width is 0 exactly while
    the target is idle, 140-220 for the car class (median 171) and 200-280 for the truck class (docs/05). The startup
    frame 00 0f ff is the one frame with a mismatching check byte; every other frame carries a valid one."""
    if len(data) != 3:
        return None
    return (data[1] & 0x0F) << 8 | data[2], data[1] >> 4, crc_0x23b(data[2], data[1]) == data[0]


@dataclass(frozen=True)
class Target366:
    """Car-bus target report: matches ACC in some regimes and also reports other states (docs/13)."""
    speed_code: int
    distance_code: int
    header_raw: int
    tail_raw: int
    lateral_code: int = 15  # signed five bits at MSB-first 40|5; 15 is the no-target value

    @property
    def y_rel(self) -> float | None:
        """Lateral offset in metres, left positive, -0.34 m per code; None for 15 (idle) and -16 (invalid, mostly beyond +-5 m)."""
        return None if self.lateral_code in (15, -16) else -0.34 * self.lateral_code

    @property
    def v_rel(self) -> float | None:
        """Nominal relative speed for codes below 256 (0.5 km/h per code, zero 155); None from 256 (docs/13)."""
        return (self.speed_code - 155) * 5 / 36 if self.speed_code < 256 else None

    @property
    def d_rel(self) -> float:
        """Approximate range in metres at the fitted 0.8 m per code (fit 0.79, docs/13)."""
        return self.distance_code * .8


def parse_0x366(data: bytes) -> Target366 | None:
    """Decode a seven-byte car-bus report, or None for the 7FFF idle word.

    0x366 arrives about 90 ms after 0x365. It agrees with the ACC target in matched mature-target comparisons and
    also reports while the ACC target is idle (docs/13). An analysis helper; the profiles read the ACC target frames.
    """
    if len(data) != 7 or data[2:4] == b"\x7f\xff":
        return None
    lateral = data[5] >> 3
    return Target366(data[2] * 2 + (data[3] >> 7), data[3] & 127,
                     int.from_bytes(data[:2], "big"), int.from_bytes(data[4:], "big"), lateral - 32 if lateral >= 16 else lateral)
