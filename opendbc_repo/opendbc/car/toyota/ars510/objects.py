"""Native decode of one 36-byte 0x80 object slot.

Each slot is read as ONE little-endian bit field: bit 0 is the LSB of slot byte 0, bit 8 the LSB of byte 1,
and so on. `start|length` below uses that numbering (it is also DBC `@1+` Intel numbering on the slot bytes).

Field boundaries come from carry chains between consecutive cycles of the same slot (a higher bit flips
only together with the bit below it, in nearly every case). Nominal scales and zero points use vehicle-motion consistency,
host-standstill object-code peaks and camera/vision references; exact physical calibration is bounded in docs/06.
docs/03_slot_fields.md describes every field; docs/06_accuracy.md the measured accuracy.

Output convention matches openpilot's RadarPoint: dRel forward (m), yRel LEFT positive (m).
"""
from __future__ import annotations

from dataclasses import dataclass

SLOT_BYTES = 36


@dataclass(frozen=True)
class NativeField:
    name: str
    bit_start: int
    bit_len: int
    zero_code: float
    scale: float
    unit: str
    status: str  # how well the field is established; see docs


# Longitudinal distance, forward. Nominal code/16 - 10 m; code 160 decodes to zero.
# Scale 1/16 m; zero code 160 fitted against vision, consistent with camera ground contact within about 0.2 m (docs/06).
LONG_DIST = NativeField("long_dist", 32, 12, 160.0, 1.0 / 16.0, "m", "validated")
# Lateral distance, LEFT positive. 12-bit offset binary around 2048, 0.015 m per code: 1.5 codes (1.499-1.502) of the radar's
# ACC-target lateral (cm), ten codes per lateral-velocity code-second, 69 codes/m [65, 74.5] under ego rotation (docs/06).
LATERAL_M_PER_CODE = 0.015
LAT_DIST = NativeField("lat_dist_left", 44, 12, 2048.0, LATERAL_M_PER_CODE, "m", "validated_sign_likely_scale")
# Longitudinal velocity OVER GROUND. Nominal 0.15 m/s/code and zero 510.5; the driving profiles read 0.149 against
# Toyota 0xB4 ego speed (docs/06).
# vRel = this - ego speed.
LONG_VEL_GROUND = NativeField("long_vel_over_ground", 64, 10, 510.5, 0.15, "m/s", "validated_with_caveats")
# Lateral velocity over ground, left positive. With the rotating-frame correction (vy = dy/dt + yaw_rate * x) it fits
# 0.142-0.155 m/s per code (0.142-0.145 on three drive groups, 0.147-0.155 against the gyro in turns); 0.15 is used
# here (docs/03). Published as yvRel (minus yaw rate x range) to forks that
# still carry that RadarPoint field (docs/08).
LAT_VEL = NativeField("lat_vel_over_ground", 74, 10, 510.5, 0.15, "m/s", "likely")
# Longitudinal acceleration over ground, filtered: zero code 511, about 0.04 m/s^2 per code, follows the velocity by about
# 0.5 s. Kept in centred codes (docs/03).
ACCEL_LIKE = NativeField("accel_like_84", 84, 10, 511.0, 1.0, "code", "likely")
# Track age in radar cycles (~60 ms): 1 at birth, saturates at 126, 0 = slot being retired.
AGE = NativeField("age_cycles", 24, 7, 0.0, 1.0, "cycles", "structure")
# Motion code 109|3: 0 moving forward, 1 slow or standing, 2 oncoming, 3 moving right, 4 moving left, 5 initializing,
# 7 stopped after moving (docs/03). move_state is the historical two-bit view, kept for compatibility.
MOVE_STATE = NativeField("move_state", 109, 2, 0.0, 1.0, "enum", "legacy_two_bit_view")
MOVEMENT_CODE = NativeField("movement_code", 109, 3, 0.0, 1.0, "code", "likely")
# Startup code 8|5: min(30, floor(31 * (2/3)**max(age - 4, 0))) while the motion code is 5 (initializing).
# Bit 13 is a separate raw flag (docs/03).
RAW8_LOW5 = NativeField("raw8_low5", 8, 5, 0.0, 1.0, "code", "confirmed")
RAW13_BIT = NativeField("raw13_bit", 13, 1, 0.0, 1.0, "bit", "raw")
# Lane state 128|3: 3 ego lane, 2 right lane, 4 left lane; 1 / 5 / 7 = all-zero lane weights (docs/03).
RAW_WEIGHT_STATE128 = NativeField("raw_weight_state128", 128, 3, 0.0, 1.0, "code", "confirmed_structure")
# Lane weights in 1/15 steps: 148|4 right lane, 152|4 left lane, 156|4 ego lane; nonzero triplets sum to 15 or 16
# (docs/03). Raw wire values are kept as they are.
RAW_WEIGHT_148 = NativeField("raw_weight148", 148, 4, 0.0, 1.0, "code", "likely")
RAW_WEIGHT_152 = NativeField("raw_weight152", 152, 4, 0.0, 1.0, "code", "likely")
RAW_WEIGHT_156 = NativeField("raw_weight156", 156, 4, 0.0, 1.0, "code", "likely")
# Oncoming-like motion state (passed the original motion test). It can persist after slowing and reset before
# the native allocation ends (docs/03).
ONCOMING_FLAG = NativeField("oncoming_flag", 14, 1, 0.0, 1.0, "flag", "tested_semantics")

# Existence probability in percent (10-100); 20|3 is its bits 4-6, score // 16 (docs/03). Reaches 100 % at about age 21.
EXISTENCE = NativeField("existence_pct", 16, 8, 0.0, 1.0, "%", "likely")
# Predicted record, the Continental "Meas = 0" state: common in a track's last records (docs/03).
PREDICTED = NativeField("predicted", 107, 1, 0.0, 1.0, "flag", "likely")

# Camera-association state 112|3: 0 radar only, 1 (rarely 2-4) while the camera has the vehicle. By day it is set inside
# about 45 m, at night (camera light-source mode) also far out (docs/03).
CAMERA_ASSOC = NativeField("camera_assoc", 112, 3, 0.0, 1.0, "state", "likely")
# Confidence of the assigned class in 1/20 steps (mostly 0 while unclassified, 20 = 100 %); steps by one per cycle on
# 95-96 % of its changes, and a large
# vehicle is re-classified as a car when it has fallen to 5 (docs/03).
CLASS_CONFIDENCE = NativeField("class_confidence", 115, 5, 0.0, 5.0, "%", "likely")

# Velocity standard deviation (sigma vx): grows with range and during velocity excursions below 40 m, higher when the velocity
# disagrees with the camera by > 2 m/s; about 0.043-0.045 m/s per count against the radar's ACC target (docs/03).
VEL_UNC_240 = NativeField("vel_uncertainty_candidate", 240, 7, 0.0, 1.0, "code", "likely")

# Historical labels only; full code 3 is rightward-like while full code 7 is stopped-like.
MOVE_STATE_NAMES = {0: "moving_away", 1: "not_clearly_moving", 2: "moving_toward", 3: "not_clearly_moving_3"}

# Object length and width (docs/03). Both read 0 only in the age-1 initialization template, whose position
# codes are placeholders (range code 160 = 0 m, lateral code 2047).
LENGTH = NativeField("length", 56, 7, 0.0, 0.1, "m", "likely")
WIDTH = NativeField("width_minus_one", 216, 6, 0.0, 0.1, "m", "likely")

# Remaining uncertainty, class, size and score codes, passed through raw (docs/03). UNCERTAINTY_PER_COUNT: scales against the radar's own
# ACC target (docs/03, uncertainty section), relative to that reference.
UNC_RANGE_224 = NativeField("range_uncertainty_code", 224, 7, 0.0, 1.0, "code", "candidate")
UNC_LATERAL_232 = NativeField("lateral_uncertainty_code", 232, 7, 0.0, 1.0, "code", "likely")
UNC_VLAT_248 = NativeField("lateral_speed_uncertainty_code", 248, 7, 0.0, 1.0, "code", "candidate")
UNC_AX_256 = NativeField("accel_uncertainty_code", 256, 8, 0.0, 1.0, "code", "candidate")
UNC_AY_264 = NativeField("lateral_accel_uncertainty_code", 264, 8, 0.0, 1.0, "code", "candidate")
UNC_ORIENT_200 = NativeField("orientation_uncertainty_code", 200, 7, 0.0, 1.0, "code", "likely")
SECONDARY_SCORE_184 = NativeField("secondary_score_pct", 184, 8, 0.0, 1.0, "%", "obstacle_probability_candidate")
OBJECT_CLASS = NativeField("object_class", 163, 3, 0.0, 1.0, "code", "likely")
HEIGHT_LIKE_272 = NativeField("height_like_code", 272, 5, 0.0, 1.0, "code", "candidate")
# m or m/s per count; range_uncertainty_code is the overall 0.23 (0.24 below 40 m).
UNCERTAINTY_PER_COUNT = {"vel_uncertainty_candidate": 0.043, "lateral_uncertainty_code": 0.10, "range_uncertainty_code": 0.23,
                         "lateral_speed_uncertainty_code": 0.37}

AGE_SATURATION = 126
# |lateral code - 2048| >= this is a sentinel.
LAT_INVALID_ABS_CODE = 2000

NAMED_FIELDS = (AGE, LONG_DIST, LAT_DIST, LONG_VEL_GROUND, LAT_VEL, ACCEL_LIKE, MOVE_STATE, MOVEMENT_CODE, RAW8_LOW5, RAW13_BIT, ONCOMING_FLAG, VEL_UNC_240, EXISTENCE, PREDICTED, RAW_WEIGHT_STATE128, RAW_WEIGHT_148, RAW_WEIGHT_152, RAW_WEIGHT_156,
                CAMERA_ASSOC, CLASS_CONFIDENCE, LENGTH, WIDTH, UNC_RANGE_224, UNC_LATERAL_232, UNC_VLAT_248, UNC_AX_256, UNC_AY_264, UNC_ORIENT_200,
                SECONDARY_SCORE_184, OBJECT_CLASS, HEIGHT_LIKE_272)


def slot_bits(slot: bytes, start: int, length: int) -> int:
    return (int.from_bytes(slot, "little") >> start) & ((1 << length) - 1)


def field_code(slot: bytes, field: NativeField) -> int:
    return slot_bits(slot, field.bit_start, field.bit_len)


def field_value(slot: bytes, field: NativeField) -> float:
    return (field_code(slot, field) - field.zero_code) * field.scale


def encode_slot(**codes: int) -> bytes:
    """Build a synthetic slot from raw codes, e.g. encode_slot(long_dist=640, age=40). Unset bits are 0."""
    if "move_state" in codes and "movement_code" in codes:
        if (int(codes["move_state"]) & 3) != (int(codes["movement_code"]) & 3):
            raise ValueError("move_state must equal the low two bits of movement_code")
    by_name = {f.name: f for f in NAMED_FIELDS}
    value = 0
    for name, code in codes.items():
        f = by_name[name]
        value |= (int(code) & ((1 << f.bit_len) - 1)) << f.bit_start
    return value.to_bytes(SLOT_BYTES, "little")


@dataclass(frozen=True)
class NativeObject:
    slot: int
    age: int
    d_rel: float  # m, forward of the radar
    y_rel: float  # m, left positive
    v_long_ground: float  # m/s over ground (minus ego speed = vRel)
    v_lat_ground: float  # m/s, provisional
    accel_like_code: int  # centred code, unscaled
    move_state: int  # legacy low two bits; see movement_code for distinct full states
    oncoming_flag: bool  # raw oncoming-like state; can reset within the allocation
    vel_unc_code: int  # velocity-uncertainty code (240|7), about 0.045 m/s per count
    geometry_valid: bool  # age >= 1 (age 0 carries stale geometry) and past the age-1 initialization template
    lateral_valid: bool  # lateral code differs from the sentinel
    movement_code: int | None = None  # full 109|3; None for legacy manually constructed objects
    raw8_low5: int | None = None  # 8|5 startup countdown while the motion code is 5
    raw13_bit: int | None = None  # 13|1 raw flag next to the startup code
    raw_weights148: tuple[int, int, int] | None = None  # 148/152/156|4 lane weights (right, left, ego) in 1/15 steps
    raw_weight_state128: int | None = None  # 128|3 lane state: 3 ego, 2 right, 4 left lane; 1/5/7 all-zero lane weights
    vel_code: int = -1  # raw 64|10 code; 1023 is a saturated (invalid) reading
    v_lat_code: int = -1  # raw 74|10 code; 0 and 1023 are sentinels
    existence_pct: int | None = None  # 16|8 existence probability, percent
    predicted: bool = False  # 107|1: the record is the radar's prediction
    camera_assoc: int | None = None  # 112|3: 0 radar only, non-zero while the camera has the vehicle
    class_confidence_pct: int | None = None  # 115|5 x 5: confidence of the assigned class, percent
    object_class: int | None = None  # 163|3: 1 unclassified, 2 car, 3 large vehicle, 4 pedestrian, 5 provisional, 6 two-wheeler
    length_m: float | None = None  # 56|7 x 0.1 m (0 only in the age-1 template)
    width_m: float | None = None  # (216|6 + 1) x 0.1 m
    height_code: int | None = None  # 272|5, height-like size code (raw)
    range_unc_code: int | None = None  # 224|7 (about 0.23 m per count overall, 0.24 below 40 m)
    lateral_unc_code: int | None = None  # 232|7 (about 0.10 m per count)
    vlat_unc_code: int | None = None  # 248|7 (about 0.37 m/s per count, unstable between cars)
    accel_unc_code: int | None = None  # 256|8, follows the frame scatter of 84|10 (0.0062 m/s^2 per count)
    lateral_accel_unc_code: int | None = None  # 264|8, follows the lateral speed scatter
    orientation_unc_code: int | None = None  # 200|7, 63 for stopped objects, 127 sentinel
    secondary_score_pct: int | None = None  # 184|8, percent-like, 100 on 98 % of rows (obstacle-probability candidate)


def is_init_template(age: int, slot: bytes) -> bool:
    """Age-1 first output with zero size codes: its range / lateral codes are placeholders."""
    return age == 1 and field_code(slot, LENGTH) == 0 and field_code(slot, WIDTH) == 0


def decode_native_slot(slot_index: int, slot: bytes) -> NativeObject:
    if len(slot) != SLOT_BYTES:
        raise ValueError(f"0x80 slot must be {SLOT_BYTES} bytes, got {len(slot)}")
    age = field_code(slot, AGE)
    lat_code = field_code(slot, LAT_DIST) - LAT_DIST.zero_code
    return NativeObject(
        slot=int(slot_index),
        age=int(age),
        d_rel=field_value(slot, LONG_DIST),
        y_rel=lat_code * LAT_DIST.scale,
        v_long_ground=field_value(slot, LONG_VEL_GROUND),
        v_lat_ground=field_value(slot, LAT_VEL),
        accel_like_code=int(field_code(slot, ACCEL_LIKE) - ACCEL_LIKE.zero_code),
        move_state=int(field_code(slot, MOVE_STATE)),
        movement_code=int(field_code(slot, MOVEMENT_CODE)),
        raw8_low5=int(field_code(slot, RAW8_LOW5)),
        raw13_bit=int(field_code(slot, RAW13_BIT)),
        raw_weights148=tuple(field_code(slot, f) for f in (RAW_WEIGHT_148, RAW_WEIGHT_152, RAW_WEIGHT_156)),
        raw_weight_state128=int(field_code(slot, RAW_WEIGHT_STATE128)),
        oncoming_flag=bool(field_code(slot, ONCOMING_FLAG)),
        vel_unc_code=int(field_code(slot, VEL_UNC_240)),
        geometry_valid=age >= 1 and not is_init_template(age, slot),
        lateral_valid=abs(lat_code) < LAT_INVALID_ABS_CODE,
        vel_code=int(field_code(slot, LONG_VEL_GROUND)),
        v_lat_code=int(field_code(slot, LAT_VEL)),
        existence_pct=int(field_code(slot, EXISTENCE)),
        predicted=bool(field_code(slot, PREDICTED)),
        camera_assoc=int(field_code(slot, CAMERA_ASSOC)),
        class_confidence_pct=int(field_code(slot, CLASS_CONFIDENCE)) * 5,
        object_class=int(field_code(slot, OBJECT_CLASS)),
        length_m=field_value(slot, LENGTH),
        width_m=(field_code(slot, WIDTH) + 1) * WIDTH.scale,
        height_code=int(field_code(slot, HEIGHT_LIKE_272)),
        range_unc_code=int(field_code(slot, UNC_RANGE_224)),
        lateral_unc_code=int(field_code(slot, UNC_LATERAL_232)),
        vlat_unc_code=int(field_code(slot, UNC_VLAT_248)),
        accel_unc_code=int(field_code(slot, UNC_AX_256)),
        lateral_accel_unc_code=int(field_code(slot, UNC_AY_264)),
        orientation_unc_code=int(field_code(slot, UNC_ORIENT_200)),
        secondary_score_pct=int(field_code(slot, SECONDARY_SCORE_184)),
    )


def relative_velocity(obj: NativeObject, v_ego: float) -> float:
    return obj.v_long_ground - float(v_ego)
