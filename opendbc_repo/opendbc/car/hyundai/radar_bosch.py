import bisect
import copy
import itertools
import math
import struct
from collections import Counter, deque
from dataclasses import dataclass, field
from functools import lru_cache
from numbers import Integral, Real
from typing import Sequence

import numpy as np

from opendbc.car import structs
from opendbc.car.radar_lead_filter import RadarLeadFilter

# Bosch MRRevo14F passive radar
# Raw return IDs and physical object IDs are distinct. Only physical
# observations are emitted; 0x601 supplies selected-member metadata.

BOSCH_INACTIVE_WORD = 0x40100000
BOSCH_MAX_RAW_TRACK_ID = 2**31 - 1
BOSCH_TRACK_ADDRESSES = frozenset(range(0x602, 0x612))
BOSCH_WINDOW_NS = 20_000_000
# A model path is only a corridor where it still reaches well past the object.
BOSCH_STALE_NS = 300_000_000
BOSCH_OUTPUT_INTERVAL_NS = 100_000_000
BOSCH_SAMPLE_HOLD_NS = 150_000_000  # one 10 Hz observation period plus one SCC publication period
BOSCH_ALEAD_SAMPLE_PERIOD_S = .10
BOSCH_ALEAD_STATE_MAX = 128

# Candidate A B5: the frozen birth signature may defer only the newborn's
# first publication. Tracking, grouping, physical IDs, qualification, aliases
# and every existing filter continue to observe the complete object tuple.
BOSCH_B5_OFF = 0
BOSCH_B5_ACTIVE = 1
BOSCH_B5_MODE = BOSCH_B5_ACTIVE
BOSCH_B5_PARENT_STABILITY_SCANS = 250
BOSCH_B5_COUNTER_MAX = 255
BOSCH_B5_CONTEXT_MAX_AGE_NS = 200_000_000
BOSCH_B5_HISTORY_NS = 120_000_000
BOSCH_B5_HISTORY_MIN_SPAN_NS = 40_000_000
BOSCH_B5_MOTION_HISTORY_NS = 400_000_000
BOSCH_B5_RADAR_TO_DEVICE_X_M = 1.52
BOSCH_B5_OUTSIDE_MARGIN_M = 1.0
BOSCH_B5_PARENT_MIN_AGE_SCANS = 20
BOSCH_B5_MIN_WORLD_SPEED_MPS = 1.0
BOSCH_B5_MAX_D_M = 12.0
BOSCH_B5_MAX_Y_M = 10.0
BOSCH_B5_MAX_DV_MPS = 1.5
BOSCH_B5_MAX_DWORLD_MPS = 1.5
BOSCH_B5_MIN_VEGO_MPS = 3.0
BOSCH_B5_MAX_CURVATURE_1PM = 0.02
BOSCH_B5_EDGE_STD_MAX_M = 0.5
BOSCH_B5_EDGE_WIDTH_MIN_M = 2.0
BOSCH_B5_EDGE_WIDTH_MAX_M = 20.0
BOSCH_B5_LANE_PROB_MIN = 0.5
BOSCH_B5_LANE_STD_MAX_M = 0.5
BOSCH_B5_LANE_STABILITY_MAX_M = 0.5
BOSCH_B5_CORRIDOR_WIDTH_MIN_M = 2.0
BOSCH_B5_CORRIDOR_WIDTH_MAX_M = 5.5
BOSCH_B5_EDGE_LANE_BAND_MAX_M = 7.0
BOSCH_B5_DIVERGENCE_MIN_M = 0.75

# M1 mirror-birth hold. The mirror geometry is an exact port of the frozen
# FR-WALLHN shadow; only newly born physical IDs can start a publication hold.
BOSCH_MIRROR_BIRTH_OFF = 0
BOSCH_MIRROR_BIRTH_SHADOW = 1
BOSCH_MIRROR_BIRTH_ACTIVE = 2
BOSCH_MIRROR_BIRTH_MODE = BOSCH_MIRROR_BIRTH_ACTIVE
BOSCH_MIRROR_BIRTH_STAT_WORLD_MAX_MPS = 0.5
BOSCH_MIRROR_BIRTH_STAT_Y_MIN_M = 1.0
BOSCH_MIRROR_BIRTH_HIST_SCANS = 20
BOSCH_MIRROR_BIRTH_GAP_NS = 300_000_000
BOSCH_MIRROR_BIRTH_BAND_HALF_M = 0.5
BOSCH_MIRROR_BIRTH_SUPPORT_MIN = 6
BOSCH_MIRROR_BIRTH_COVER_HALF_M = 6.0
BOSCH_MIRROR_BIRTH_COVER_MIN = 0.5
BOSCH_MIRROR_BIRTH_DD_MAX_M = 1.5
BOSCH_MIRROR_BIRTH_DV_MAX_MPS = 0.5
BOSCH_MIRROR_BIRTH_RESID_MAX_M = 2.0
BOSCH_MIRROR_BIRTH_BEYOND_MIN_M = 1.0
BOSCH_MIRROR_BIRTH_INSIDE_MIN_M = 1.0
BOSCH_MIRROR_BIRTH_PARENT_AGE_SCANS = 10
BOSCH_MIRROR_BIRTH_MOVE_MIN_MPS = 2.0
BOSCH_MIRROR_BIRTH_D_MAX_M = 80.0
BOSCH_MIRROR_BIRTH_CURV_MAX_1PM = 0.01
BOSCH_MIRROR_BIRTH_VEGO_MIN_MPS = 3.0
BOSCH_MIRROR_BIRTH_CONFIRM_SCANS = 5
BOSCH_MIRROR_BIRTH_HOLD_SCANS = 10
BOSCH_MIRROR_BIRTH_Y_MIN_M = 4.5
BOSCH_MIRROR_BIRTH_INSIDE_MARGIN_M = 0.5
BOSCH_MIRROR_BIRTH_PARENT_MISS_SCANS = 2
BOSCH_MIRROR_BIRTH_DIVERGE_DV_MPS = 1.5
BOSCH_MIRROR_BIRTH_DIVERGE_DD_M = 5.0
BOSCH_MIRROR_BIRTH_DIVERGE_SCANS = 3

# M3 is a read-only SHADOW extension of M1: left road-edge birth fallback and
# bounded coupled extension. Its state is deliberately separate from M1.
BOSCH_MIRROR_M3_OFF = 0
BOSCH_MIRROR_M3_SHADOW = 1
BOSCH_MIRROR_M3_ACTIVE = 2
BOSCH_MIRROR_M3_MODE = BOSCH_MIRROR_M3_ACTIVE
BOSCH_MIRROR_M3_HOLD_SCANS = 10
BOSCH_MIRROR_M3_Y_MIN_M = 4.5
BOSCH_MIRROR_M3_INSIDE_MARGIN_M = 0.5
BOSCH_MIRROR_M3_PARENT_MISS_SCANS = 2
BOSCH_MIRROR_M3_DIVERGE_DV_MPS = 1.5
BOSCH_MIRROR_M3_DIVERGE_DD_M = 5.0
BOSCH_MIRROR_M3_DIVERGE_SCANS = 3
BOSCH_MIRROR_M3_EXT_MAX_SCANS = 50
BOSCH_MIRROR_M3_EXT_DD_M = 3.0
BOSCH_MIRROR_M3_EXT_DV_MPS = 1.0
BOSCH_MIRROR_M3_EXT_RESID_M = 3.0
BOSCH_MIRROR_M3_EDGE = 1
BOSCH_MIRROR_M3_EDGE_LEFT_ONLY = 1
BOSCH_MIRROR_M3_VEGO_MIN_MPS = 10.0
BOSCH_MIRROR_M3_EDGE_FRESH_NS = 200_000_000
BOSCH_MIRROR_M3_EDGE_SIDE_MIN_M = 0.5
BOSCH_MIRROR_M3_BEYOND_MIN_M = 1.0
BOSCH_MIRROR_M3_PARENT_AGE_SCANS = 10
BOSCH_MIRROR_M3_PARENT_VABS_MIN_MPS = 2.0
BOSCH_MIRROR_M3_PARENT_INSIDE_MIN_M = 1.0
BOSCH_MIRROR_M3_PARENT_DD_MAX_M = 1.5
BOSCH_MIRROR_M3_PARENT_DV_MAX_MPS = 0.5
BOSCH_MIRROR_M3_RESID_MAX_M = 2.0
BOSCH_MIRROR_M3_D_MAX_M = 80.0
BOSCH_MIRROR_M3_RADAR_TO_CAMERA_M = 1.52
BOSCH_MIRROR_M3_GAP_NS = 300_000_000

# Candidate P91 changes only the final Bosch publication view; raw detections,
# grouping, physical IDs, qualification and alias bindings remain. SHADOW
# diagnostics stay available for replay and on-device comparison.
BOSCH_P91_OFF = 0
BOSCH_P91_SHADOW = 1
BOSCH_P91_ACTIVE = 2
BOSCH_P91_ACTIVE_TEST = BOSCH_P91_ACTIVE  # compatibility name for existing synthetic tests
BOSCH_P91_MODE = BOSCH_P91_ACTIVE
BOSCH_P91_SUPPORT_HOLD_NS = 500_000_000

# S37 B1: two completed Bosch scans may establish that a singleton newborn is
# another surface of a mature physical object.  This state is deliberately
# separate from the S32 close-birth bundle above: it never changes raw/physical
# tracking and only withholds the newborn from the final public view.
BOSCH_FAMILY_COMPANION_OFF = 0
BOSCH_FAMILY_COMPANION_SHADOW = 1
BOSCH_FAMILY_COMPANION_ACTIVE = 2
BOSCH_FAMILY_COMPANION_MODE = BOSCH_FAMILY_COMPANION_ACTIVE
BOSCH_FAMILY_MIN_ANCHOR_AGE_SCANS = 25
BOSCH_FAMILY_MIN_D_M = 3.0
BOSCH_FAMILY_MAX_D_M = 12.0
BOSCH_FAMILY_MAX_Y_M = 2.5
BOSCH_FAMILY_MAX_DV_MPS = .75
BOSCH_FAMILY_MAX_WORLD_SPEED_MPS = .35
BOSCH_FAMILY_MAX_D_STEP_M = 1.5
BOSCH_FAMILY_MAX_Y_STEP_M = .75
BOSCH_FAMILY_MIN_WORLD_SPEED_MPS = 4.0
BOSCH_FAMILY_MAX_GAP_NS = 160_000_000
# Reuse the established Bosch provider cut-in evidence (three monotone scans,
# 0.90 m/s inward); keep literal values here because the OEM constants below
# are declared later in this module.
BOSCH_FAMILY_INWARD_SCANS = 3
BOSCH_FAMILY_INWARD_MPS = .90

# S33 burst multi-return: one physical body can split into several radar
# surfaces inside a single scan.  The signature is a burst -- three or more
# singletons born in the same scan at one world speed, packed into a thin range
# slab but fanned out laterally, beside a mature object already moving at that
# same world speed.  Confirmed on video for a cargo truck (route26d S11,
# route26a S26, route247 S18) and for a distant passenger car (route283 S14),
# so the bounds below describe the return geometry, not a vehicle class.
# Like B1 above this only withholds the newborn from the final public view.
BOSCH_BURST_MULTIRETURN_OFF = 0
BOSCH_BURST_MULTIRETURN_SHADOW = 1
BOSCH_BURST_MULTIRETURN_ACTIVE = 2
BOSCH_BURST_MULTIRETURN_MODE = BOSCH_BURST_MULTIRETURN_ACTIVE
BOSCH_BURST_MIN_MEMBERS = 3
BOSCH_BURST_MAX_WORLD_SPAN_MPS = .35
BOSCH_BURST_MAX_D_SPAN_M = 3.0
BOSCH_BURST_MIN_Y_SPAN_M = 8.0
BOSCH_BURST_MIN_WORLD_SPEED_MPS = 4.0
BOSCH_BURST_MIN_VEGO_MPS = 8.0
BOSCH_BURST_NEIGHBOUR_AGE_SCANS = 25
BOSCH_BURST_NEIGHBOUR_D_M = 30.0
BOSCH_BURST_HOLD_MAX_SCANS = 5
BOSCH_BURST_MAX_MISSING_SCANS = 1
BOSCH_BURST_MAX_GAP_NS = 320_000_000
BOSCH_BURST_STATE_MAX = 16

# Side-pass extended-body lateral estimate (publication only).  A Bosch raw
# return is one scattering centre.  On a fixed point of a body its range changes
# with the reported longitudinal relative velocity.  While ego overtakes a long
# vehicle the dominant scatterer instead moves forward along that body: the
# range closes more slowly than vRel ("slide") and the reported lateral drifts
# toward the body's near side (route2bc S21/S23 trucks, whose LDWS camera
# lateral stayed constant).  While that rigid-point model is violated the
# lateral change is not evidence of vehicle motion, so the published lateral
# keeps its physical anchor.  Raw tracks, members, PIDs and aliases are untouched.
BOSCH_SIDEPASS_LATERAL_OFF = 0
BOSCH_SIDEPASS_LATERAL_SHADOW = 1
BOSCH_SIDEPASS_LATERAL_ACTIVE = 2
BOSCH_SIDEPASS_LATERAL_MODE = BOSCH_SIDEPASS_LATERAL_ACTIVE
BOSCH_SIDEPASS_WINDOW_NS = 800_000_000
BOSCH_SIDEPASS_MIN_SCANS = 5
BOSCH_SIDEPASS_MIN_SPAN_NS = 400_000_000
BOSCH_SIDEPASS_SLIDE_ARM_MPS = 1.5
BOSCH_SIDEPASS_SLIDE_RELEASE_MPS = .5
BOSCH_SIDEPASS_RELEASE_SCANS = 3
BOSCH_SIDEPASS_CLOSING_MPS = 1.5
BOSCH_SIDEPASS_MIN_LEAD_SPEED_MPS = 3.0  # same-direction mover; cut-in needs vLead > 0.5
BOSCH_SIDEPASS_CORRIDOR_M = 1.9          # downstream body-overlap half width (0.9 + 1.0)
BOSCH_SIDEPASS_ZONE_M = 5.4              # downstream lateral motion scope
BOSCH_SIDEPASS_MAX_D_M = 45.0            # downstream cut-in range
BOSCH_SIDEPASS_MAX_OFFSET_M = 1.0        # beyond surface migration on one body
BOSCH_SIDEPASS_REALIGN_MPS = .10         # below the downstream lateral motion floor
BOSCH_SIDEPASS_MAX_GAP_NS = 320_000_000
BOSCH_SIDEPASS_STATE_MAX = 64

# Behavioural naming for the two 0x601 records. No proprietary signal name is
# claimed. word1 (bytes 4..7) is bit-identical to exactly one raw record in
# 63,819 of 63,822 active scans across 146 segments, and its activity equals
# SCC11.ObjValid in 99 % of frames; word0 (bytes 0..3) matches no raw record in
# 96 % of active scans and is gated by lateral offset and by range near 100 m.
# The pair is therefore read as a state, not as one boolean support flag.
BOSCH_OEM_STATE_NONE = 0
BOSCH_OEM_STATE_TENTATIVE = 1   # word0 only: in-path candidate, no selected member
BOSCH_OEM_STATE_SELECTED = 2    # word1 only: selected member outside the word0 gate
BOSCH_OEM_STATE_VALIDATED = 3   # word0 and word1

BOSCH_OEM_GATE_OFF = 0
BOSCH_OEM_GATE_SHADOW = 1
BOSCH_OEM_GATE_ACTIVE = 2
BOSCH_OEM_GATE_MODE = BOSCH_OEM_GATE_ACTIVE
# A validated target that drops word1 for one or two scans must not become
# gate-eligible. Measured bracketed word1 dropouts on 146 segments are 58 in
# total and only 12 % are shorter than five scans, so the hold is cheap.
BOSCH_OEM_EVIDENCE_HOLD_NS = 500_000_000
# SCC12 keeps computing a longitudinal request while openpilot drives. It is an
# independent OEM intent observation, never an actuator command: it only helps
# decide whether an unvalidated in-path candidate may reach the published view.
# Raw field 0 decodes to the -10.23 floor and means "not populated" here, so a
# sane band is required before the sample counts at all.
BOSCH_OEM_INTENT_MIN_MPS2 = -9.0
BOSCH_OEM_INTENT_MAX_MPS2 = 3.0
BOSCH_OEM_INTENT_FRESH_NS = 150_000_000
BOSCH_OEM_INTENT_WINDOW_NS = 300_000_000
BOSCH_OEM_INTENT_MIN_SAMPLES = 3
# Gate thresholds. Every value is set from the route273/26d/274 sweep in
# analysis/bosch_oem_validation; none is a guessed round number.
BOSCH_OEM_GATE_NEUTRAL_MPS2 = -0.25
BOSCH_OEM_GATE_PERSIST_SCANS = 3
BOSCH_OEM_GATE_MIN_SPEED_MPS = 8.0
BOSCH_OEM_GATE_MIN_RANGE_M = 30.0
BOSCH_OEM_GATE_MIN_TTC_S = 4.0
BOSCH_OEM_GATE_CUTIN_MPS = 0.90
BOSCH_OEM_GATE_CUTIN_SCANS = 3
BOSCH_OEM_GATE_LATERAL_WINDOW_NS = 600_000_000
# Only a lead candidate can be withheld, and only while it is new to the path.
# Widening the corridor past 1.0 m or letting a settled object stay eligible
# both reintroduced real-lead losses in the route273/26d/274 sweep.
BOSCH_OEM_GATE_IN_PATH_M = 1.0
BOSCH_OEM_GATE_SETTLED_SCANS = 10
# A large vehicle answers as several returns. When one of them is the OEM's
# validated target the others are companions of a confirmed object, not
# unvalidated candidates; without this the second return of a followed box
# truck was withheld for 0.6 s on route26d segment 87.
BOSCH_OEM_GATE_SIBLING_D_M = 12.0
BOSCH_OEM_GATE_SIBLING_Y_M = 3.0
BOSCH_OEM_GATE_SIBLING_V_MPS = 1.5
# Stock SCC observation, read on the Bosch provider's own CAN stream so the
# samples carry receive timestamps and no shared CANParser changes. Offsets are
# the hyundai_kia_generic SCC11/SCC12 little-endian start bits.
BOSCH_SCC_BUS = 2
BOSCH_SCC11_ADDR = 0x420
BOSCH_SCC12_ADDR = 0x421
BOSCH_SCC11_OBJ_VALID_BIT = 16      # SCC11.ObjValid
BOSCH_SCC12_AREQ_RAW_BIT = 24       # SCC12.aReqRaw, 11 bits, (0.01, -10.23)
BOSCH_SCC_ADDRESSES = frozenset((BOSCH_SCC11_ADDR, BOSCH_SCC12_ADDR))
BOSCH_SCC_STALE_NS = 300_000_000

# Bosch <-> OEM-camera extended grouping. General distribution was authorized
# on 2026-09-29. Preserve the existing mode values and publication behavior.
BOSCH_CAMERA_EXTENDED_OFF = 0
BOSCH_CAMERA_EXTENDED_SHADOW = 1
BOSCH_CAMERA_EXTENDED_ACTIVE = 2  # Legacy observation-preserving mode
BOSCH_CAMERA_EXTENDED_PUBLICATION = 3
BOSCH_CAMERA_EXTENDED_ACTIVE_TEST = BOSCH_CAMERA_EXTENDED_PUBLICATION  # Legacy replay compatibility
# PUBLICATION changes actual RadarData, including downstream lead-selection input.
# OFF/SHADOW preserve baseline publication; mode changes require restarting radarcan.
BOSCH_CAMERA_EXTENDED_TEST_INTERVALS = 2
# A scan that keeps an extended group alive through the E2 coast but has no
# strict camera edge used to throw the accumulated maturity away, so a single
# missing camera confirmation restarted the two-interval warm-up. Hold it while
# the member motion still tracks the prediction, bounded by the last strict
# confirmation. A coasting group may keep collapsing publication only if it was
# already mature on the previous scan and every member was measured this scan;
# it never becomes mature during a coast. Otherwise both contacts stay published.
BOSCH_CAMERA_MATURITY_HOLD_NS = 200_000_000
# Motion-continuity bounds for maturity. Range noise of the far surface and the
# bearing noise of a distant body (lateral error grows with range) used to reset
# confirmed large-vehicle groups (P1 drives: 190 of 812 confirmations; lateral
# residual p50 0.34 m at 66 m against a fixed 0.26 m bound).
BOSCH_CAMERA_MATURITY_D_TOL_M = 1.0
BOSCH_CAMERA_MATURITY_Y_RANGE_K = 0.005
# The OEM selected the same member of this exact set on the previous scan too,
# which is independent evidence that the set is one vehicle. Shorten the
# interval requirement for that case only; the geometry checks are unchanged.
BOSCH_CAMERA_MATURITY_WORD1_INTERVALS = 1
# A camera class-1 (large vehicle) verdict on both members is itself independent
# evidence that the set is one body, so one stable interval suffices there too.
# The motion-continuity checks are unchanged.
BOSCH_CAMERA_MATURITY_CLASS1_INTERVALS = 1
BOSCH_CAMERA_EXTENDED_MODE = BOSCH_CAMERA_EXTENDED_PUBLICATION
BOSCH_CAMERA_HEADER = 0x738
BOSCH_CAMERA_FIRST_OBJECT = 0x739
BOSCH_CAMERA_LAST_OBJECT = 0x756
BOSCH_CAMERA_LAST_FAMILY = 0x760
BOSCH_CAMERA_SLOTS = 10
BOSCH_CAMERA_PERIOD_NS = 40_000_000
BOSCH_CAMERA_FRAME_TOLERANCE_NS = 25_000_000
BOSCH_CAMERA_OBSERVATION_GAP_NS = 160_000_000
BOSCH_CAMERA_E2_HOLD_NS = 250_000_000
BOSCH_CAMERA_SNAPSHOT_COUNT = 4
BOSCH_CAMERA_WIDTH_REF_M = 1.70
# 카메라의 longitudinal / lateral / relative speed LSB는 0.0625가 아니라 0.05다.
# 독립 근거 6개가 일치하고 반례는 0건이다
# (analysis/bosch_radar/20260912_ldws_mobileye_verified_decode).
# 이전 상수는 세 필드를 모두 +25 % 과대 스케일로 읽고 있었다.
BOSCH_CAMERA_RANGE_LSB = 0.05
# continuity anchor 대비 association 세로 창. 서로 무관한 두 오차를 흡수해야 하고
# 둘의 스케일 법칙이 다르다.
#   * 단안 거리 오차는 거리에 비례한다 -> BASE + RANGE_K * d_rel
#   * 차량 자신의 return 집합은 수 m 깊고 카메라는 후면을 보고하므로 레이더는
#     차체 더 깊은 곳의 return을 유지한다 -> 폭 항, 그리고 가까운 쪽이 훨씬 크다
# 기존의 비대칭 상수 [0.0, 5.0]은 +25 % 과대 스케일을 상쇄하기 위한 경험값이었다.
# 올바른 스케일에서 그 창은 "카메라가 레이더보다 가깝다"는 참 쌍의 1/3을 전부 기각한다.
BOSCH_CAMERA_LONG_BASE_M = 1.75
BOSCH_CAMERA_LONG_RANGE_K = 0.12
BOSCH_CAMERA_LONG_POS_WIDTH_K = 4.0
BOSCH_CAMERA_LONG_NEG_WIDTH_K = 16.0
BOSCH_CAMERA_LONG_MAX_M = 20.0
BOSCH_CAMERA_LAT_MAX_M = 1.75
BOSCH_CAMERA_ANGLE_LSB = 1.0 / 4496.3
BOSCH_CAMERA_ASSOC_UNRESOLVED = 0
BOSCH_CAMERA_ASSOC_ASSIGNED = 1
BOSCH_CAMERA_ASSOC_AMBIGUOUS = 2
# Bosch-only causal fallback for a short curve-induced lateral/bearing miss.
# It never changes a baseline ASSIGNED/AMBIGUOUS verdict and never refreshes
# itself: only a normal A0 ASSIGNED pair may seed or advance the history.
BOSCH_CAMERA_CURVE_REACQUIRE_OFF = 0
BOSCH_CAMERA_CURVE_REACQUIRE_SHADOW = 1
BOSCH_CAMERA_CURVE_REACQUIRE_ACTIVE = 2
BOSCH_CAMERA_CURVE_REACQUIRE_MODE = BOSCH_CAMERA_CURVE_REACQUIRE_ACTIVE
BOSCH_CAMERA_CURVE_REACQUIRE_HOLD_NS = 800_000_000
BOSCH_CAMERA_CURVE_REACQUIRE_MIN_D_M = 70.0
BOSCH_CAMERA_CURVE_REACQUIRE_MAX_D_M = 100.0
BOSCH_CAMERA_CURVE_REACQUIRE_MIN_YAW_RATE = 0.015
BOSCH_CAMERA_CURVE_REACQUIRE_LAT_MAX_M = 2.35
BOSCH_CAMERA_CURVE_REACQUIRE_BEAR_MAX_RAD = 0.026
BOSCH_CAMERA_CURVE_REACQUIRE_STATE_MAX = 64
# camera class 6은 경험적으로 car-family 상태이지 truck enum이 아니다.
# 아래의 독립적인 폭/rigid-pair 증거를 모두 만족할 때만 대형차 P2를 보조하며,
# 기존 class-1 경로는 별도 분기로 그대로 유지한다.
BOSCH_TRUCK_P2_CLASS = 6
# Three continuous confirmations (0.2 s) of the same wide class-6 camera object and
# truck geometry; five delayed most truck merges by another 0.2 s (P1 drives).
BOSCH_TRUCK_P2_CONFIRMATIONS = 3
BOSCH_TRUCK_P2_STATE_MAX = 16
# Buses, box trucks and car carriers also report 2.25-2.40 m (2026-09-11 study:
# a car carrier at 2.30-2.35 m never formed a pair). The 5.5 m spacing floor stays:
# a 5.0-5.5 m pair can be a parked car and the object just past it.
BOSCH_TRUCK_P2_WIDTH_MIN_M = 2.25
BOSCH_TRUCK_P2_DD_MIN_M = 5.50
# Long trucks/buses put their two strongest returns 9-11 m apart (P1 drives: 105 of
# 162 unmerged class-6 scans); 12 m matches the geometry candidate and class-1 limit.
BOSCH_TRUCK_P2_DD_MAX_M = 12.00
BOSCH_TRUCK_P2_DY_MAX_M = 0.875
BOSCH_TRUCK_P2_DV_MAX_MPS = 0.50
BOSCH_TRUCK_A0_RECOVERY_HOLD_SCANS = 2
BOSCH_TRUCK_A0_RECOVERY_BEARING_EXCESS_RAD = 0.010
BOSCH_TRUCK_A0_RECOVERY_COST_MARGIN = 0.15
# Footprint association for the far member of a camera-confirmed large vehicle.
# The camera bearing span describes the rear face; the front of a long body in
# an adjacent lane sits farther away, so its bearing moves toward the centre and
# fails A0 by 0.016-0.11 rad while range/lateral gates pass (P1 drives: 151 of
# 153 unassociated far members). Used for strict pairs only, never exported.
BOSCH_LARGE_FOOTPRINT_LENGTH_M = 12.0
BOSCH_LARGE_FOOTPRINT_LAT_MARGIN_M = 0.5
# Shared camera episode와 OEM selection은 physical identity proof가 아니다.
# 기존 rigid-pair opportunity 안에서도 prior raw/group split과 representative handoff가
# 모두 확인된 경우에만 먼 표면의 publication을 미룬다. 근거가 없으면 fail open한다.
BOSCH_COMPANION_DEFER_OFF = 0
BOSCH_COMPANION_DEFER_ACTIVE = 1
BOSCH_COMPANION_DEFER_MODE = BOSCH_COMPANION_DEFER_ACTIVE
BOSCH_COMPANION_DEFER_CONFIRMATIONS = 1
BOSCH_COMPANION_DEFER_HOLD_NS = 200_000_000
BOSCH_COMPANION_DEFER_STATE_MAX = 16
BOSCH_COMPANION_DEFER_DD_MIN_M = 3.0
BOSCH_COMPANION_DEFER_DD_MAX_M = 12.0
BOSCH_COMPANION_DEFER_DY_MAX_M = 1.5
BOSCH_COMPANION_DEFER_DV_MAX_MPS = 1.0
BOSCH_COMPANION_DEFER_Y_MAX_M = 2.5
BOSCH_COMMON_ANCESTRY_MAX_AGE_NS = 40_000_000_000
BOSCH_COMMON_ANCESTRY_RAW_PAIR_MAX = 32 * 31 // 2
# 좁은 camera object는 여러 m 떨어진 두 return을 가질 수 없다.
BOSCH_COMPANION_DEFER_WIDTH_DD_M = ((2.00, 5.0), (2.30, 7.0))
# 물리 객체는 대표 member의 거리를 발행한다. 대표는 연속성을 첫 키로 고르므로 한 번
# 더 먼 member가 대표가 되면 객체가 사는 동안 유지되고, 정차한 앞차를 자기 최근접
# 표면보다 1~3 m 뒤로 발행한다(cluster 지름 상한 3.0 m). 같은 scan에서 OEM이 자기
# 제어 대상으로 고른 0x601 word1 slot이 그 객체의 member이고 대표보다 가까우면, 발행
# 좌표만 그 member로 바꾼다. publication_view의 마지막 단계이므로 tracker/grouping/
# qualifier/P91/OEM gate는 전혀 영향받지 않고, 발행 거리는 가까워지는 방향으로만 바뀐다.
BOSCH_OEM_NEARER_PUBLICATION_OFF = 0
BOSCH_OEM_NEARER_PUBLICATION_ACTIVE = 1
BOSCH_OEM_NEARER_PUBLICATION_MODE = BOSCH_OEM_NEARER_PUBLICATION_ACTIVE
# 미지원 첫 singleton 발행 억제(birth guard)와 native surface 코너 gate. 2026-10-03
# 전체 코퍼스 검증(카메라 확인 차량 8,047 episode)에서 둘 다 실제 차량을 지웠다
# (birth 42, corner 22 REAL_CONFIRMED; birth는 실제 끼어들기 경계 0.75 s 지연과 정상
# fixture 3개 손실). 따라서 birth는 기본으로 끄고, 판정 코드와 시험은 그대로 둔다.
BOSCH_UNSUPPORTED_BIRTH_PUBLICATION = False
# 코너 gate가 지운 영상 확인 차량 22대 중 21대는 92.5 m 이내의 분기·출구·갓길·병행
# 차로 차량이었고(model road edge 밖), 사용자 확인 코너 고스트 11개는 모두 109.7 m
# 이상이었다. 그래서 gate는 품질을 보장하지 않는 원거리 발행점에만 작용한다. 하한은
# 발행 좌표(native dRel) 기준이고 다른 판정 조건은 그대로다.
BOSCH_SURFACE_CORNER_GATE = True
BOSCH_SURFACE_CORNER_MIN_RANGE_M = 100.0


def _bosch_camera_signed(value, bits):
  sign = 1 << (bits - 1)
  return value - (1 << bits) if value & sign else value


def _bosch_camera_range_span(d_rel):
  """카메라 객체와 무관한, 거리에만 의존하는 창 성분."""
  return BOSCH_CAMERA_LONG_BASE_M + BOSCH_CAMERA_LONG_RANGE_K * d_rel


def _bosch_camera_long_window(span, width_m):
  extra = max(width_m - BOSCH_CAMERA_WIDTH_REF_M, 0.0)
  return (min(span + BOSCH_CAMERA_LONG_POS_WIDTH_K * extra, BOSCH_CAMERA_LONG_MAX_M),
          min(span + BOSCH_CAMERA_LONG_NEG_WIDTH_K * extra, BOSCH_CAMERA_LONG_MAX_M))


@dataclass(slots=True)
class BoschCameraObject:
  obj_id: int = 0
  episode: int = 0
  long_m: float = 0.0
  lat_m: float = 0.0
  vrel_mps: float = 0.0
  width_m: float = 0.0
  class_code: int = -1
  angle_left: float = 0.0
  angle_right: float = 0.0


class BoschCameraCycleCache:
  """Bounded, allocation-free-per-frame cache for the 25 Hz camera family.

  A/B/C are accepted only with the current header counter.  A tiny fixed
  pre-header bank handles arbitrary ordering inside one CAN receive batch.
  Four preallocated snapshots retain the latest causal cycle when a newer
  camera cycle is already later than the Bosch scan being closed.
  """
  def __init__(self):
    self._counter = -1
    self._cycle_index = -1
    self._header_ns = -1
    self._n_objects = 0
    self._a = [0] * BOSCH_CAMERA_SLOTS
    self._b = [0] * BOSCH_CAMERA_SLOTS
    self._c = [0] * BOSCH_CAMERA_SLOTS
    self._a_ns = [-1] * BOSCH_CAMERA_SLOTS
    self._b_ns = [-1] * BOSCH_CAMERA_SLOTS
    self._c_ns = [-1] * BOSCH_CAMERA_SLOTS
    self._masks = [0, 0, 0]
    pending_size = 16 * BOSCH_CAMERA_SLOTS
    self._pending = [[0] * pending_size for _ in range(3)]
    self._pending_ns = [[-1] * pending_size for _ in range(3)]
    self._buffers = [[BoschCameraObject() for _ in range(BOSCH_CAMERA_SLOTS)]
                     for _ in range(BOSCH_CAMERA_SNAPSHOT_COUNT)]
    self._snapshot_cycle = [-1] * BOSCH_CAMERA_SNAPSHOT_COUNT
    self._snapshot_header_ns = [-1] * BOSCH_CAMERA_SNAPSHOT_COUNT
    self._snapshot_complete_ns = [-1] * BOSCH_CAMERA_SNAPSHOT_COUNT
    self._snapshot_count = [0] * BOSCH_CAMERA_SNAPSHOT_COUNT
    self._snapshot_next = 0
    self._last_seen_cycle = [-1000] * 256
    self._episode = [0] * 256
    self._next_episode = 0
    self.fault_ns = -1
    self.completed_cycles = 0
    self.rejected_cycles = 0

  @staticmethod
  def _counter_of(word):
    return (word >> 52) & 0xf

  def _unwrap(self, counter, timestamp_ns):
    if self._counter < 0:
      return 0
    elapsed = max(0, round((timestamp_ns - self._header_ns) / BOSCH_CAMERA_PERIOD_NS))
    wanted = (counter - self._counter) & 0xf
    step = elapsed + ((wanted - elapsed) & 0xf)
    if step - elapsed > 8:
      step -= 16
    return self._cycle_index + max(step, wanted)

  def _store(self, kind, slot, word, timestamp_ns):
    arrays = (self._a, self._b, self._c)
    times = (self._a_ns, self._b_ns, self._c_ns)
    arrays[kind][slot] = word
    times[kind][slot] = timestamp_ns
    self._masks[kind] |= 1 << slot

  def _pending_store(self, counter, kind, slot, word, timestamp_ns):
    index = counter * BOSCH_CAMERA_SLOTS + slot
    if timestamp_ns >= self._pending_ns[kind][index]:
      self._pending[kind][index] = word
      self._pending_ns[kind][index] = timestamp_ns

  def _start_cycle(self, counter, n_objects, timestamp_ns):
    if self._header_ns >= 0 and timestamp_ns <= self._header_ns:
      return False
    if self._counter >= 0 and self._n_objects and not self._is_complete():
      self.fault_ns = timestamp_ns
      self.rejected_cycles += 1
    cycle_index = self._unwrap(counter, timestamp_ns)
    self._counter = counter
    self._cycle_index = cycle_index
    self._header_ns = timestamp_ns
    self._n_objects = n_objects
    self._masks[:] = (0, 0, 0)
    self._a_ns[:] = [-1] * BOSCH_CAMERA_SLOTS
    self._b_ns[:] = [-1] * BOSCH_CAMERA_SLOTS
    self._c_ns[:] = [-1] * BOSCH_CAMERA_SLOTS
    for kind in range(3):
      for slot in range(BOSCH_CAMERA_SLOTS):
        index = counter * BOSCH_CAMERA_SLOTS + slot
        pending_ns = self._pending_ns[kind][index]
        if pending_ns >= 0 and abs(pending_ns - timestamp_ns) <= BOSCH_CAMERA_FRAME_TOLERANCE_NS:
          self._store(kind, slot, self._pending[kind][index], pending_ns)
        self._pending_ns[kind][index] = -1
    self._try_complete()
    return True

  def _is_complete(self):
    required = (1 << self._n_objects) - 1
    return all(mask & required == required for mask in self._masks)

  def _try_complete(self):
    if self._counter < 0 or not self._is_complete():
      return
    if any(cycle == self._cycle_index for cycle in self._snapshot_cycle):
      return
    seen_ids = 0
    for slot in range(self._n_objects):
      a = self._a[slot]
      if not (a & ((1 << 48) - 1)):
        self.fault_ns = max(self._header_ns, self._a_ns[slot])
        self.rejected_cycles += 1
        return
      obj_id = a & 0xff
      bit = 1 << obj_id
      if seen_ids & bit:
        self.fault_ns = max(self._header_ns, self._a_ns[slot])
        self.rejected_cycles += 1
        return
      seen_ids |= bit
    target = self._snapshot_next
    objects = self._buffers[target]
    complete_ns = self._header_ns
    for slot in range(self._n_objects):
      a, b, c = self._a[slot], self._b[slot], self._c[slot]
      complete_ns = max(complete_ns, self._a_ns[slot], self._b_ns[slot], self._c_ns[slot])
      obj_id = a & 0xff
      if self._cycle_index - self._last_seen_cycle[obj_id] > 2:
        self._next_episode += 1
        self._episode[obj_id] = self._next_episode
      self._last_seen_cycle[obj_id] = self._cycle_index
      obj = objects[slot]
      obj.obj_id = obj_id
      obj.episode = self._episode[obj_id]
      obj.long_m = ((a >> 8) & 0xfff) * BOSCH_CAMERA_RANGE_LSB
      obj.lat_m = _bosch_camera_signed((a >> 20) & 0xfff, 12) * BOSCH_CAMERA_RANGE_LSB
      obj.vrel_mps = _bosch_camera_signed((a >> 40) & 0xfff, 12) * BOSCH_CAMERA_RANGE_LSB
      obj.width_m = (b & 0x3f) * 0.05
      obj.class_code = int((b >> 48) & 0x3) + 4 * int((b >> 50) & 0x1)
      obj.angle_right = _bosch_camera_signed((c >> 18) & 0x1fff, 13) * BOSCH_CAMERA_ANGLE_LSB
      obj.angle_left = _bosch_camera_signed((c >> 31) & 0x1fff, 13) * BOSCH_CAMERA_ANGLE_LSB
    self._snapshot_cycle[target] = self._cycle_index
    self._snapshot_header_ns[target] = self._header_ns
    self._snapshot_complete_ns[target] = complete_ns
    self._snapshot_count[target] = self._n_objects
    self._snapshot_next = (target + 1) % BOSCH_CAMERA_SNAPSHOT_COUNT
    self.completed_cycles += 1

  def ingest(self, timestamp_ns, address, payload):
    # About a thousand camera frames a second arrive here, so the common paths
    # avoid helper calls. Each shortcut below only skips work whose result the
    # original helper would have discarded.
    if len(payload) != 8:
      self.fault_ns = timestamp_ns
      return False
    if address == BOSCH_CAMERA_HEADER:
      word = int.from_bytes(payload, 'little')
      n_objects = word & 0xf
      if n_objects > BOSCH_CAMERA_SLOTS:
        self.fault_ns = timestamp_ns
        self.rejected_cycles += 1
        return False
      return self._start_cycle((word >> 52) & 0xf, n_objects, timestamp_ns)
    if not BOSCH_CAMERA_FIRST_OBJECT <= address <= BOSCH_CAMERA_LAST_OBJECT:
      return False
    word = int.from_bytes(payload, 'little')
    slot, kind = divmod(address - BOSCH_CAMERA_FIRST_OBJECT, 3)
    counter = (word >> 52) & 0xf
    header_ns = self._header_ns
    if (counter == self._counter and header_ns >= 0 and
        abs(timestamp_ns - header_ns) <= BOSCH_CAMERA_FRAME_TOLERANCE_NS):
      if kind == 0:
        self._a[slot] = word
        self._a_ns[slot] = timestamp_ns
      elif kind == 1:
        self._b[slot] = word
        self._b_ns[slot] = timestamp_ns
      else:
        self._c[slot] = word
        self._c_ns[slot] = timestamp_ns
      masks = self._masks
      mask = masks[kind] | (1 << slot)
      masks[kind] = mask
      # _try_complete acts only on a complete cycle that has no snapshot yet.
      required = (1 << self._n_objects) - 1
      if (mask & required == required and masks[0] & required == required and
          masks[1] & required == required and masks[2] & required == required and
          self._cycle_index not in self._snapshot_cycle):
        self._try_complete()
    else:
      self._pending_store(counter, kind, slot, word, timestamp_ns)
    return True

  def snapshot(self, timestamp_ns):
    best = -1
    best_cycle = -1
    for index in range(BOSCH_CAMERA_SNAPSHOT_COUNT):
      complete_ns = self._snapshot_complete_ns[index]
      if (complete_ns <= timestamp_ns and complete_ns > self.fault_ns and
          timestamp_ns - complete_ns <= BOSCH_CAMERA_OBSERVATION_GAP_NS and
          self._snapshot_cycle[index] > best_cycle):
        best, best_cycle = index, self._snapshot_cycle[index]
    if best < 0:
      return None
    return self._buffers[best], self._snapshot_count[best], self._snapshot_cycle[best], self._snapshot_complete_ns[best]


@dataclass(slots=True)
class _BoschExtendedHistory:
  members: tuple[int, ...]
  cam_key: int
  class_code: int
  last_confirm_ns: int
  # 현재 exact tuple의 직전 관측만 보관한다(최대 8 member). 과거 이력 원장이 아니다.
  stable_intervals: int = 0
  sample_ns: int = 0
  observations: tuple[tuple[float, float, float], ...] = ()
  # physical ID the OEM word1 selected inside this member set, or -1.
  oem_anchor: int = -1


@dataclass(slots=True)
class _BoschCurveReacquireHistory:
  camera_id: int
  episode: int
  last_assigned_ns: int


@dataclass(slots=True)
class _BoschTruckPairHistory:
  members: tuple[int, int]
  camera_id: int
  episode: int
  last_ns: int
  confirmations: int
  dd: float
  dy: float
  dv: float
  camera_d: float
  camera_y: float
  camera_v: float
  camera_width: float
  recovery_age: int = 0


@dataclass(slots=True)
class _BoschExtendedRepresentative:
  members: frozenset[int]
  representative_pid: int
  timestamp_ns: int
  d_rel: float
  y_rel: float
  v_rel: float
  age: int


class BoschCameraExtendedGrouping:
  """Compute A0/P2/G0/E2 groups and M2 publication eligibility.

  OFF/SHADOW/legacy ACTIVE preserve baseline observations. PUBLICATION reduces
  the output after alias allocation. Each representative retains its own PID
  history; histories from other members are not transferred to it.
  """
  def __init__(self, mode=BOSCH_CAMERA_EXTENDED_MODE,
               curve_reacquire_mode=BOSCH_CAMERA_CURVE_REACQUIRE_MODE):
    if mode not in (BOSCH_CAMERA_EXTENDED_OFF, BOSCH_CAMERA_EXTENDED_SHADOW,
                    BOSCH_CAMERA_EXTENDED_ACTIVE, BOSCH_CAMERA_EXTENDED_PUBLICATION):
      raise ValueError('invalid Bosch camera extended-grouping mode')
    if curve_reacquire_mode not in (BOSCH_CAMERA_CURVE_REACQUIRE_OFF,
                                    BOSCH_CAMERA_CURVE_REACQUIRE_SHADOW,
                                    BOSCH_CAMERA_CURVE_REACQUIRE_ACTIVE):
      raise ValueError('invalid Bosch camera curve-reacquire mode')
    self.mode = mode
    self.curve_reacquire_mode = curve_reacquire_mode
    self.camera = BoschCameraCycleCache() if mode != BOSCH_CAMERA_EXTENDED_OFF else None
    self.histories: dict[tuple[int, ...], _BoschExtendedHistory] = {}
    self.truck_pair_histories: dict[tuple[int, int], _BoschTruckPairHistory] = {}
    self.representatives: list[_BoschExtendedRepresentative] = []
    self.last_ns = None
    self.last_groups: tuple[tuple[int, ...], ...] = ()
    self.last_association_count = 0
    self.last_associations = {}
    self.last_footprint_associations = ()
    self.last_candidate_count = 0
    self.last_coast_count = 0
    self.max_state_count = 0
    self.representative_switches = 0
    self.max_representative_jump = [0.0, 0.0, 0.0]
    self.mature_groups = ()
    self.maturity_resets = 0
    self.last_maturity_resets = 0
    self.last_camera_ns = None
    self.last_truck_edge_count = 0
    self.max_truck_pair_state = 0
    self.last_truck_recovery_count = 0
    self.last_camera_by_episode = {}
    self.last_v_ego = math.nan
    self.last_yaw_rate = None
    self.curve_reacquire_histories: dict[int, _BoschCurveReacquireHistory] = {}
    self.curve_reacquire_state_peak = 0
    self.curve_reacquire_attempts = 0
    self.curve_reacquire_successes = 0
    self.curve_reacquire_rejects = Counter()
    self.last_baseline_associations = {}
    self.last_curve_reacquire = ()
    self.last_curve_reacquire_would = ()

  @staticmethod
  def _geometry(a, b, v_ego, yaw_rate):
    dd = abs(a.d_rel - b.d_rel)
    dy = abs(a.y_rel - b.y_rel)
    dv = abs(a.v_rel - b.v_rel)
    conflict = False
    if math.isfinite(v_ego):
      yaw = yaw_rate if yaw_rate is not None and math.isfinite(yaw_rate) else 0.0
      wa = abs(a.v_rel + v_ego - yaw * a.y_rel)
      wb = abs(b.v_rel + v_ego - yaw * b.y_rel)
      conflict = min(wa, wb) <= 0.6 and max(wa, wb) >= 1.4
    return dd, dy, dv, 3.0 < dd <= 12.0 and dy <= 1.5 and dv <= 1.5 and not conflict

  @staticmethod
  def _associate(obj, camera_objects, count):
    bearing = math.atan2(-obj.y_rel, max(obj.d_rel, 0.5))
    span = _bosch_camera_range_span(obj.d_rel)
    best = second = math.inf
    best_obj = None
    passed = 0
    for index in range(count):
      cam = camera_objects[index]
      l_pos, l_neg = _bosch_camera_long_window(span, cam.width_m)
      d_long = cam.long_m - obj.d_rel
      d_lat = cam.lat_m + obj.y_rel
      lo, hi = min(cam.angle_left, cam.angle_right), max(cam.angle_left, cam.angle_right)
      bear_out = max(lo - bearing, bearing - hi)
      if (d_long > l_pos or d_long < -l_neg or bear_out > 0.020 or
          abs(d_lat) > BOSCH_CAMERA_LAT_MAX_M):
        continue
      passed += 1
      half = (hi - lo) * 0.5
      bear_n = (half + bear_out) / max(half + 0.020, 1e-6)
      cost = (4.0 * bear_n + 2.0 * abs(cam.vrel_mps - obj.v_rel) / 3.0 +
              abs(d_lat) / BOSCH_CAMERA_LAT_MAX_M +
              0.5 * abs(d_long) / max(l_pos + l_neg, 1e-6))
      if cost < best:
        second, best, best_obj = best, cost, cam
      elif cost < second:
        second = cost
    if not passed:
      return BOSCH_CAMERA_ASSOC_UNRESOLVED, -1, -1
    if passed >= 2 and second - best < 0.15:
      return BOSCH_CAMERA_ASSOC_AMBIGUOUS, -1, -1
    return BOSCH_CAMERA_ASSOC_ASSIGNED, best_obj.episode, best_obj.class_code

  def _curve_reacquire(self, obj, camera_objects, count, timestamp_ns, yaw_rate):
    """Return Candidate C's verdict without mutating its A0-only history."""
    self.curve_reacquire_attempts += 1
    history = self.curve_reacquire_histories.get(obj.physical_track_id)
    if history is None or not 0 <= timestamp_ns - history.last_assigned_ns <= BOSCH_CAMERA_CURVE_REACQUIRE_HOLD_NS:
      self.curve_reacquire_rejects['history'] += 1
      return BOSCH_CAMERA_ASSOC_UNRESOLVED, -1, -1
    if not obj.oem_selected:
      self.curve_reacquire_rejects['word1'] += 1
      return BOSCH_CAMERA_ASSOC_UNRESOLVED, -1, -1
    if not (BOSCH_CAMERA_CURVE_REACQUIRE_MIN_D_M <= obj.d_rel < BOSCH_CAMERA_CURVE_REACQUIRE_MAX_D_M and
            yaw_rate is not None and math.isfinite(yaw_rate) and
            abs(yaw_rate) >= BOSCH_CAMERA_CURVE_REACQUIRE_MIN_YAW_RATE):
      self.curve_reacquire_rejects['context'] += 1
      return BOSCH_CAMERA_ASSOC_UNRESOLVED, -1, -1

    bearing = math.atan2(-obj.y_rel, max(obj.d_rel, 0.5))
    span = _bosch_camera_range_span(obj.d_rel)
    matches = []
    for index in range(count):
      camera = camera_objects[index]
      if camera.obj_id != history.camera_id or camera.episode != history.episode:
        continue
      l_pos, l_neg = _bosch_camera_long_window(span, camera.width_m)
      d_long = camera.long_m - obj.d_rel
      d_lat = camera.lat_m + obj.y_rel
      lo, hi = min(camera.angle_left, camera.angle_right), max(camera.angle_left, camera.angle_right)
      bear_out = max(lo - bearing, bearing - hi)
      if (-l_neg <= d_long <= l_pos and
          abs(d_lat) <= BOSCH_CAMERA_CURVE_REACQUIRE_LAT_MAX_M and
          bear_out <= BOSCH_CAMERA_CURVE_REACQUIRE_BEAR_MAX_RAD):
        matches.append(camera)
    if len(matches) != 1:
      self.curve_reacquire_rejects['ambiguous' if len(matches) > 1 else 'geometry'] += 1
      return BOSCH_CAMERA_ASSOC_UNRESOLVED, -1, -1
    camera = matches[0]
    return BOSCH_CAMERA_ASSOC_ASSIGNED, camera.episode, camera.class_code

  def _curve_history_prepare(self, timestamp_ns, by_pid, camera_objects, camera_count):
    if self.curve_reacquire_mode == BOSCH_CAMERA_CURVE_REACQUIRE_OFF:
      self.curve_reacquire_histories = {}
      return
    camera_keys = {(camera_objects[index].obj_id, camera_objects[index].episode)
                   for index in range(camera_count)}
    self.curve_reacquire_histories = {
      pid: history for pid, history in self.curve_reacquire_histories.items()
      if (pid in by_pid and (history.camera_id, history.episode) in camera_keys and
          0 <= timestamp_ns - history.last_assigned_ns <= BOSCH_CAMERA_CURVE_REACQUIRE_HOLD_NS)
    }

  def _curve_history_update(self, timestamp_ns, baseline, camera_by_episode):
    if self.curve_reacquire_mode == BOSCH_CAMERA_CURVE_REACQUIRE_OFF:
      return
    for pid, verdict in baseline.items():
      if verdict[0] == BOSCH_CAMERA_ASSOC_ASSIGNED:
        camera = camera_by_episode.get(verdict[1])
        if camera is not None:
          self.curve_reacquire_histories[pid] = _BoschCurveReacquireHistory(
            camera.obj_id, camera.episode, timestamp_ns)
      elif verdict[0] == BOSCH_CAMERA_ASSOC_AMBIGUOUS:
        self.curve_reacquire_histories.pop(pid, None)
    if len(self.curve_reacquire_histories) > BOSCH_CAMERA_CURVE_REACQUIRE_STATE_MAX:
      keep = sorted(self.curve_reacquire_histories.items(),
                    key=lambda item: (-item[1].last_assigned_ns, item[0]))[:BOSCH_CAMERA_CURVE_REACQUIRE_STATE_MAX]
      self.curve_reacquire_histories = dict(keep)
    self.curve_reacquire_state_peak = max(self.curve_reacquire_state_peak,
                                          len(self.curve_reacquire_histories))

  @staticmethod
  def _complete_link(objects, edges, previous):
    ids = sorted(obj.physical_track_id for obj in objects)
    # With no confirmed/coasting camera edge, complete-link can only return
    # these singleton groups.  Avoid the quadratic failed pair search while
    # preserving all association/history maintenance that precedes this call.
    if not edges:
      return [{pid} for pid in ids]
    active = {pid for pair in edges for pid in pair}
    groups = [{pid} for pid in ids if pid in active]
    groups += [{pid} for pid in ids if pid not in active]
    previous = [set(group) for group in previous]
    while True:
      choices = []
      for i, a in enumerate(groups):
        for j in range(i + 1, len(groups)):
          b = groups[j]
          if len(a) + len(b) > 8:
            continue
          cross = [tuple(sorted((x, y))) for x in a for y in b]
          if all(pair in edges for pair in cross):
            overlap = any(old & a and old & b for old in previous)
            choices.append((not overlap, max(edges[pair] for pair in cross), min(a), min(b), i, j))
      if not choices:
        break
      *_, i, j = min(choices)
      groups[i] |= groups.pop(j)
    return groups

  def _choose_representatives(self, groups, by_pid, timestamp_ns, yaw_rate):
    multi = [set(group) for group in groups if len(group) > 1]
    matches = {}
    claims = []
    for i, group in enumerate(multi):
      for j, prior in enumerate(self.representatives):
        overlap = len(group & prior.members)
        if overlap:
          score = 10 * overlap + 3 * (prior.representative_pid in group) + min(prior.age, 1000) * 1e-5
          claims.append((-score, min(group), prior.representative_pid, i, j))
    used_current, used_prior = set(), set()
    for _, _, _, i, j in sorted(claims):
      if i not in used_current and j not in used_prior:
        matches[i] = self.representatives[j]
        used_current.add(i)
        used_prior.add(j)
    new_states = []
    representatives = {}
    for i, group in enumerate(multi):
      members = [by_pid[pid] for pid in group]
      prior = matches.get(i)
      if prior is not None:
        dt = (timestamp_ns - prior.timestamp_ns) / 1e9
        angle = -(yaw_rate if yaw_rate is not None and math.isfinite(yaw_rate) else 0.0) * dt
        dx = prior.d_rel + prior.v_rel * dt
        px = dx * math.cos(angle) - prior.y_rel * math.sin(angle)
        py = dx * math.sin(angle) + prior.y_rel * math.cos(angle)
        def cost(obj):
          continuity = abs(obj.d_rel - px) + .5 * abs(obj.y_rel - py) + .5 * abs(obj.v_rel - prior.v_rel)
          return (continuity, obj.physical_track_id != prior.representative_pid,
                  not obj.vision_supported, not obj.oem_selected, -obj.age_scans, obj.physical_track_id)
      else:
        ordered = sorted(obj.d_rel for obj in members)
        middle = len(ordered) // 2
        median = ordered[middle] if len(ordered) & 1 else (ordered[middle - 1] + ordered[middle]) * .5
        def cost(obj):
          return (abs(obj.d_rel - median), False, not obj.vision_supported,
                  not obj.oem_selected, -obj.age_scans, obj.physical_track_id)
      # publication_view hides every member of a mature group except this one,
      # so this choice decides which surface of one vehicle downstream keeps.
      # Every pair inside a group passed 3.0 < dd <= 12.0, so range order is
      # unambiguous by at least 3 m and cannot chatter on measurement noise.
      # Prefer the member the OEM selected; otherwise the nearest, because
      # hiding the nearest surface of a vehicle overstates the lead range.
      rep = min(members, key=lambda obj: (not obj.oem_selected, obj.d_rel, cost(obj)))
      if prior is not None and rep.physical_track_id != prior.representative_pid:
        self.representative_switches += 1
        self.max_representative_jump[0] = max(self.max_representative_jump[0], abs(rep.d_rel - px))
        self.max_representative_jump[1] = max(self.max_representative_jump[1], abs(rep.y_rel - py))
        self.max_representative_jump[2] = max(self.max_representative_jump[2], abs(rep.v_rel - prior.v_rel))
      representatives[tuple(sorted(group))] = rep
      new_states.append(_BoschExtendedRepresentative(
        frozenset(group), rep.physical_track_id, timestamp_ns, rep.d_rel, rep.y_rel, rep.v_rel,
        prior.age + 1 if prior is not None else 1))
    self.representatives = new_states
    return representatives

  def _maturity(self, members, episode, class_code, timestamp_ns, by_pid, yaw_rate,
                *, confirmed=True, confirm_ns=None):
    # 연구 M2 정의 그대로: 첫 confirmed scan은 0, 두 이전 안정 간격 후 2.
    # camera ID/episode만으로 연속성을 인정하지 않고 모든 member의 d/y/v를 확인한다.
    prior = self.histories.get(members)
    age = 0
    if prior is not None and prior.cam_key == episode and prior.observations:
      dt = (timestamp_ns - prior.sample_ns) * 1e-9
      if 0 < dt <= .160:
        angle = -(yaw_rate if yaw_rate is not None and math.isfinite(yaw_rate) else 0.) * dt
        co, si = math.cos(angle), math.sin(angle)
        for pid, (d, y, v) in zip(members, prior.observations):
          obj = by_pid[pid]
          x = d + v * dt
          if not (abs(obj.d_rel - (x * co - y * si)) <= BOSCH_CAMERA_MATURITY_D_TOL_M + 2.5 * dt * dt and
                  abs(obj.y_rel - (x * si + y * co)) <= (.0625 + 2 * dt +
                                                         BOSCH_CAMERA_MATURITY_Y_RANGE_K * abs(obj.d_rel)) and
                  abs(obj.v_rel - v) <= .25 + 5 * dt):
            break
        else:
          # confirmed=False is a coast: this scan has no strict camera edge. The
          # same motion-continuity test above is still required; only the
          # accumulated interval survives instead of being discarded.
          age = (min(prior.stable_intervals + 1, BOSCH_CAMERA_EXTENDED_TEST_INTERVALS)
                 if confirmed else prior.stable_intervals)
    anchor = -1
    for pid in members:
      if by_pid[pid].oem_selected:
        anchor = pid
        break
    return _BoschExtendedHistory(members, episode, class_code,
                                 timestamp_ns if confirm_ns is None else confirm_ns,
                                 age, timestamp_ns,
                                 tuple((by_pid[p].d_rel, by_pid[p].y_rel, by_pid[p].v_rel) for p in members),
                                 anchor)

  @staticmethod
  def _large_footprint_associations(geometry, associations, by_pid, camera_by_episode):
    """Far member of a pair whose near member A0 assigned to a large camera object.

    Only a far member no camera object explains (A0 UNRESOLVED) qualifies, and
    only inside that object's body footprint: behind its rear face by at most
    BOSCH_LARGE_FOOTPRINT_LENGTH_M, laterally within half its width plus a
    margin, at the near member's speed. The bearing gate is the one A0 test it
    is allowed to miss.
    """
    out = {}
    for key, (dd, dy, dv) in sorted(geometry.items()):
      a, b = by_pid[key[0]], by_pid[key[1]]
      near, far = (a, b) if (a.d_rel, a.physical_track_id) <= (b.d_rel, b.physical_track_id) else (b, a)
      verdict = associations.get(near.physical_track_id)
      far_verdict = associations.get(far.physical_track_id)
      if (verdict is None or verdict[0] != BOSCH_CAMERA_ASSOC_ASSIGNED or far_verdict is None or
          far_verdict[0] != BOSCH_CAMERA_ASSOC_UNRESOLVED or far.physical_track_id in out or dv > BOSCH_TRUCK_P2_DV_MAX_MPS):
        continue
      camera = camera_by_episode.get(verdict[1])
      if camera is None or not (verdict[2] == 1 or (verdict[2] == BOSCH_TRUCK_P2_CLASS and
                                                     camera.width_m >= BOSCH_TRUCK_P2_WIDTH_MIN_M)):
        continue
      _, l_neg = _bosch_camera_long_window(_bosch_camera_range_span(far.d_rel), camera.width_m)
      d_long = camera.long_m - far.d_rel
      d_lat = camera.lat_m + far.y_rel
      if not (-l_neg <= d_long < 0. and -d_long <= BOSCH_LARGE_FOOTPRINT_LENGTH_M and
              abs(d_lat) <= min(BOSCH_CAMERA_LAT_MAX_M, camera.width_m * .5 + BOSCH_LARGE_FOOTPRINT_LAT_MARGIN_M)):
        continue
      out[far.physical_track_id] = (BOSCH_CAMERA_ASSOC_ASSIGNED, verdict[1], verdict[2])
    return out

  @staticmethod
  def _truck_pair_continuous(prior, timestamp_ns, dd, dy, dv, camera):
    dt = (timestamp_ns - prior.last_ns) * 1e-9
    return (
      0 < dt <= BOSCH_CAMERA_OBSERVATION_GAP_NS * 1e-9 and
      prior.episode == camera.episode and prior.camera_id == camera.obj_id and
      abs(dd - prior.dd) <= .50 and abs(dy - prior.dy) <= .375 and abs(dv - prior.dv) <= .25 and
      abs(camera.long_m - (prior.camera_d + prior.camera_v * dt)) <= .75 and
      abs(camera.lat_m - prior.camera_y) <= .125 and
      abs(camera.vrel_mps - prior.camera_v) <= .50 and
      abs(camera.width_m - prior.camera_width) <= .10)

  @staticmethod
  def _truck_a0_bearing_near_miss(obj, anchor, camera_objects, count):
    """Frozen A0 바로 바깥의 명확한 bearing miss인지 확인한다.

    A0 verdict 자체는 바꾸지 않는다. 이미 seed된 대형차 pair의 빠진 한 node가
    같은 camera object에 대해 다른 frozen gate를 모두 통과하고, raw cost도
    경쟁 object보다 명확히 작을 때만 truck P2용 보강 evidence로 사용한다.
    """
    bearing = math.atan2(-obj.y_rel, max(obj.d_rel, .5))
    span = _bosch_camera_range_span(obj.d_rel)
    anchor_cost = math.inf
    other_cost = math.inf
    anchor_gate = False
    for index in range(count):
      camera = camera_objects[index]
      l_pos, l_neg = _bosch_camera_long_window(span, camera.width_m)
      d_long = camera.long_m - obj.d_rel
      d_lat = camera.lat_m + obj.y_rel
      lo, hi = min(camera.angle_left, camera.angle_right), max(camera.angle_left, camera.angle_right)
      bear_out = max(lo - bearing, bearing - hi)
      half = (hi - lo) * .5
      bear_n = (half + bear_out) / max(half + .020, 1e-6)
      cost = (4.0 * bear_n + 2.0 * abs(camera.vrel_mps - obj.v_rel) / 3.0 +
              abs(d_lat) / BOSCH_CAMERA_LAT_MAX_M +
              .5 * abs(d_long) / max(l_pos + l_neg, 1e-6))
      if camera.obj_id == anchor.obj_id and camera.episode == anchor.episode:
        anchor_cost = cost
        anchor_gate = (-l_neg <= d_long <= l_pos and abs(d_lat) <= BOSCH_CAMERA_LAT_MAX_M and
                       .020 < bear_out <= .020 + BOSCH_TRUCK_A0_RECOVERY_BEARING_EXCESS_RAD)
      else:
        other_cost = min(other_cost, cost)
    return anchor_gate and anchor_cost + BOSCH_TRUCK_A0_RECOVERY_COST_MARGIN <= other_cost

  def _truck_strict(self, timestamp_ns, geometry, associations, by_pid, camera_by_episode,
                    camera_objects=(), camera_count=0):
    candidates = []
    for key, (dd, dy, dv) in geometry.items():
      aa = associations.get(key[0], (BOSCH_CAMERA_ASSOC_UNRESOLVED, -1, -1))
      ab = associations.get(key[1], (BOSCH_CAMERA_ASSOC_UNRESOLVED, -1, -1))
      if not (aa[0] == BOSCH_CAMERA_ASSOC_ASSIGNED and ab[0] == BOSCH_CAMERA_ASSOC_ASSIGNED and
              aa[1] == ab[1] and aa[1] >= 0 and aa[2] == ab[2] == BOSCH_TRUCK_P2_CLASS):
        continue
      camera = camera_by_episode.get(aa[1])
      fresh = (camera is not None and self.last_camera_ns is not None and
               0 <= timestamp_ns - self.last_camera_ns <= BOSCH_CAMERA_OBSERVATION_GAP_NS and
               by_pid[key[0]].timestamp_ns == by_pid[key[1]].timestamp_ns == timestamp_ns)
      if not (fresh and camera.width_m >= BOSCH_TRUCK_P2_WIDTH_MIN_M and
              BOSCH_TRUCK_P2_DD_MIN_M <= dd <= BOSCH_TRUCK_P2_DD_MAX_M and
              dy <= BOSCH_TRUCK_P2_DY_MAX_M and dv <= BOSCH_TRUCK_P2_DV_MAX_MPS):
        continue
      score = dd / 12.0 + dy / 1.5 + dv / 1.5
      candidates.append((score, key, dd, dy, dv, camera))

    # 비정상적으로 조밀한 return에서도 fail-open으로 state 상한을 지킨다.
    candidates.sort(key=lambda row: (row[0], row[1]))
    next_history = {}
    strict = {}
    for score, key, dd, dy, dv, camera in candidates[:BOSCH_TRUCK_P2_STATE_MAX]:
      prior = self.truck_pair_histories.get(key)
      confirmations = prior.confirmations + 1 if prior is not None and self._truck_pair_continuous(
        prior, timestamp_ns, dd, dy, dv, camera) else 1
      state = _BoschTruckPairHistory(key, camera.obj_id, camera.episode, timestamp_ns, confirmations,
                                     dd, dy, dv, camera.long_m, camera.lat_m, camera.vrel_mps, camera.width_m)
      next_history[key] = state
      if confirmations >= BOSCH_TRUCK_P2_CONFIRMATIONS:
        strict[key] = score

    recovered = 0
    recovery_candidates = []
    if len(next_history) < BOSCH_TRUCK_P2_STATE_MAX:
      for key, prior in self.truck_pair_histories.items():
        if key in next_history or key not in geometry or prior.confirmations < BOSCH_TRUCK_P2_CONFIRMATIONS:
          continue
        prior_rep = next((rep for rep in self.representatives
                          if len(rep.members) == len(key) and all(pid in rep.members for pid in key)), None)
        if prior_rep is None or prior_rep.representative_pid not in by_pid:
          continue
        aa = associations.get(key[0], (BOSCH_CAMERA_ASSOC_UNRESOLVED, -1, -1))
        ab = associations.get(key[1], (BOSCH_CAMERA_ASSOC_UNRESOLVED, -1, -1))
        verdicts = (aa, ab)
        assigned = [index for index, value in enumerate(verdicts) if value[0] == BOSCH_CAMERA_ASSOC_ASSIGNED]
        if len(assigned) != 1:
          continue
        assigned_index = assigned[0]
        missing_index = 1 - assigned_index
        anchor_verdict, missing_verdict = verdicts[assigned_index], verdicts[missing_index]
        if (missing_verdict[0] != BOSCH_CAMERA_ASSOC_UNRESOLVED or
            anchor_verdict[1] != prior.episode or anchor_verdict[2] != BOSCH_TRUCK_P2_CLASS):
          continue
        camera = camera_by_episode.get(anchor_verdict[1])
        dd, dy, dv = geometry[key]
        fresh = (camera is not None and camera.obj_id == prior.camera_id and self.last_camera_ns is not None and
                 0 <= timestamp_ns - self.last_camera_ns <= BOSCH_CAMERA_OBSERVATION_GAP_NS and
                 by_pid[key[0]].timestamp_ns == by_pid[key[1]].timestamp_ns == timestamp_ns)
        absolute = (fresh and camera.width_m >= BOSCH_TRUCK_P2_WIDTH_MIN_M and
                    BOSCH_TRUCK_P2_DD_MIN_M <= dd <= BOSCH_TRUCK_P2_DD_MAX_M and
                    dy <= BOSCH_TRUCK_P2_DY_MAX_M and dv <= BOSCH_TRUCK_P2_DV_MAX_MPS)
        recovery_age = prior.recovery_age + 1
        if (not absolute or recovery_age > BOSCH_TRUCK_A0_RECOVERY_HOLD_SCANS or
            not self._truck_pair_continuous(prior, timestamp_ns, dd, dy, dv, camera) or
            not self._truck_a0_bearing_near_miss(by_pid[key[missing_index]], camera, camera_objects, camera_count)):
          continue
        score = dd / 12.0 + dy / 1.5 + dv / 1.5
        recovery_candidates.append((score, key, dd, dy, dv, camera, prior.confirmations, recovery_age))

    recovery_candidates.sort(key=lambda row: (row[0], row[1]))
    capacity = BOSCH_TRUCK_P2_STATE_MAX - len(next_history)
    for score, key, dd, dy, dv, camera, confirmations, recovery_age in recovery_candidates[:capacity]:
      next_history[key] = _BoschTruckPairHistory(
        key, camera.obj_id, camera.episode, timestamp_ns, confirmations, dd, dy, dv,
        camera.long_m, camera.lat_m, camera.vrel_mps, camera.width_m, recovery_age)
      strict[key] = score
      recovered += 1
    self.truck_pair_histories = next_history
    self.max_truck_pair_state = max(self.max_truck_pair_state, len(next_history))
    self.last_truck_edge_count = len(strict)
    self.last_truck_recovery_count = recovered
    return strict

  def update(self, timestamp_ns, objects, v_ego, yaw_rate=None):
    if self.mode == BOSCH_CAMERA_EXTENDED_OFF:
      return objects
    prior_maturity = self.histories
    prior_mature = set(self.mature_groups)
    self.mature_groups = ()
    curve_clock_reset = self.last_ns is not None and timestamp_ns <= self.last_ns
    if self.last_ns is not None and timestamp_ns - self.last_ns > BOSCH_CAMERA_OBSERVATION_GAP_NS:
      self.histories = {}
      self.truck_pair_histories = {}
      self.representatives.clear()
      self.curve_reacquire_histories = {}
    elif curve_clock_reset:
      self.curve_reacquire_histories = {}
    self.last_ns = timestamp_ns
    by_pid = {obj.physical_track_id: obj for obj in objects}
    ordered = sorted(objects, key=lambda obj: (obj.d_rel, obj.physical_track_id))
    geometry = {}
    candidate_nodes = set()
    for i, a in enumerate(ordered):
      for b in itertools.islice(ordered, i + 1, None):
        dd = b.d_rel - a.d_rel
        if dd > 12.0:
          break
        if dd <= 3.0:
          continue
        dd, dy, dv, okay = self._geometry(a, b, v_ego, yaw_rate)
        if not okay:
          continue
        key = tuple(sorted((a.physical_track_id, b.physical_track_id)))
        geometry[key] = (dd, dy, dv)
        candidate_nodes.update(key)
    self.last_candidate_count = len(geometry)

    snapshot = self.camera.snapshot(timestamp_ns)
    self.last_camera_ns = snapshot[3] if snapshot is not None else None
    associations = {}
    camera_by_episode = {}
    camera_objects = ()
    camera_count = 0
    if snapshot is not None:
      camera_objects, camera_count, _, _ = snapshot
      camera_by_episode = {camera_objects[index].episode: copy.copy(camera_objects[index]) for index in range(camera_count)}
      self._curve_history_prepare(timestamp_ns, by_pid, camera_objects, camera_count)
      # The frozen shadow definition runs the normal A0 gate for every live
      # physical PID. Extra baseline verdicts seed history only; they are not
      # exposed through last_associations unless C actually reacquires.
      baseline_nodes = (by_pid if self.curve_reacquire_mode != BOSCH_CAMERA_CURVE_REACQUIRE_OFF
                        else {pid: by_pid[pid] for pid in candidate_nodes})
      baseline_associations = {}
      reacquired = []
      would_reacquire = []
      for pid, obj in baseline_nodes.items():
        baseline = self._associate(obj, camera_objects, camera_count)
        baseline_associations[pid] = baseline
        if pid in candidate_nodes:
          associations[pid] = baseline
        if baseline[0] == BOSCH_CAMERA_ASSOC_UNRESOLVED and self.curve_reacquire_mode != BOSCH_CAMERA_CURVE_REACQUIRE_OFF:
          candidate = self._curve_reacquire(obj, camera_objects, camera_count,
                                            timestamp_ns, yaw_rate)
          if candidate[0] == BOSCH_CAMERA_ASSOC_ASSIGNED:
            camera = camera_by_episode[candidate[1]]
            would_reacquire.append((pid, camera.obj_id, camera.episode))
            if self.curve_reacquire_mode == BOSCH_CAMERA_CURVE_REACQUIRE_ACTIVE:
              associations[pid] = candidate
              reacquired.append((pid, camera.obj_id, camera.episode))
              self.curve_reacquire_successes += 1
      self._curve_history_update(timestamp_ns, baseline_associations, camera_by_episode)
      self.last_baseline_associations = baseline_associations
      self.last_curve_reacquire = tuple(sorted(reacquired))
      self.last_curve_reacquire_would = tuple(sorted(would_reacquire))
    else:
      self.curve_reacquire_histories = {}
      self.last_baseline_associations = {}
      self.last_curve_reacquire = ()
      self.last_curve_reacquire_would = ()
    self.last_association_count = len(associations)
    self.last_associations = associations
    # publication_view의 companion deferral이 같은 scan의 A0 판정과 pair gate를
    # 다시 계산하지 않고 그대로 쓰도록 보관한다.
    self.last_camera_by_episode = camera_by_episode
    self.last_v_ego = v_ego
    self.last_yaw_rate = yaw_rate
    # Strict pairs may also use footprint verdicts; every other consumer keeps
    # the frozen A0 verdicts stored above.
    footprint = self._large_footprint_associations(geometry, associations, by_pid, camera_by_episode)
    self.last_footprint_associations = tuple(sorted(footprint))
    if footprint:
      associations = {**associations, **footprint}

    strict = {}
    strict_classes = {}
    strict_episodes = {}
    for key, (dd, dy, dv) in geometry.items():
      aa = associations.get(key[0], (BOSCH_CAMERA_ASSOC_UNRESOLVED, -1, -1))
      ab = associations.get(key[1], (BOSCH_CAMERA_ASSOC_UNRESOLVED, -1, -1))
      if (aa[0] == BOSCH_CAMERA_ASSOC_ASSIGNED and ab[0] == BOSCH_CAMERA_ASSOC_ASSIGNED and
          aa[1] == ab[1] and aa[1] >= 0 and aa[2] == 1 and ab[2] == 1):
        strict[key] = dd / 12.0 + dy / 1.5 + dv / 1.5
        strict_classes[key] = 1
        strict_episodes[key] = aa[1]
    truck_strict = self._truck_strict(timestamp_ns, geometry, associations, by_pid, camera_by_episode,
                                      camera_objects, camera_count)
    strict.update(truck_strict)
    strict_classes.update((key, BOSCH_TRUCK_P2_CLASS) for key in truck_strict)
    strict_episodes.update((key, self.truck_pair_histories[key].episode) for key in truck_strict)

    edges = dict(strict)
    next_history = {}
    coast_groups = []
    for members, history in list(self.histories.items()):
      pairs = [tuple(sorted((members[i], members[j]))) for i in range(len(members)) for j in range(i + 1, len(members))]
      verdicts = [associations.get(pid, (BOSCH_CAMERA_ASSOC_UNRESOLVED, -1, -1)) for pid in members]
      assigned = [value for value in verdicts if value[0] == BOSCH_CAMERA_ASSOC_ASSIGNED]
      okay = (all(pid in by_pid for pid in members) and all(pair in geometry for pair in pairs) and
              timestamp_ns - history.last_confirm_ns <= BOSCH_CAMERA_E2_HOLD_NS and
              not any(value[0] == BOSCH_CAMERA_ASSOC_AMBIGUOUS for value in verdicts) and
              not any(value[1] != history.cam_key or value[2] != history.class_code for value in assigned) and
              any(value[1] == history.cam_key for value in assigned))
      if not okay:
        continue
      if not all(pair in strict for pair in pairs):
        member_set = set(members)
        # A coast may preserve exactly this confirmed set, never recruit a new
        # node through a currently-confirmed cross edge.
        for pair in [pair for pair in edges if bool(member_set & set(pair)) and not set(pair) <= member_set]:
          del edges[pair]
        for pair in pairs:
          dd, dy, dv = geometry[pair]
          edges[pair] = dd / 12.0 + dy / 1.5 + dv / 1.5
        coast_groups.append(members)
        next_history[members] = history
    self.last_coast_count = len(coast_groups)

    previous = [state.members for state in self.representatives]
    groups = self._complete_link(objects, edges, previous)

    representatives = self._choose_representatives(groups, by_pid, timestamp_ns, yaw_rate)
    mature = []
    for group in groups:
      if len(group) < 2:
        continue
      members = tuple(sorted(group))
      pairs = [tuple(sorted((members[i], members[j]))) for i in range(len(members)) for j in range(i + 1, len(members))]
      if all(pair in strict for pair in pairs):
        episode = strict_episodes[pairs[0]]
        class_code = strict_classes[pairs[0]]
        fresh = (self.last_camera_ns is not None and
                 0 <= timestamp_ns - self.last_camera_ns <= BOSCH_CAMERA_OBSERVATION_GAP_NS and
                 all(by_pid[p].timestamp_ns == timestamp_ns for p in members))
        if self.mode == BOSCH_CAMERA_EXTENDED_PUBLICATION and fresh:
          state = self._maturity(members, episode, class_code, timestamp_ns, by_pid, yaw_rate)
          next_history[members] = state
          prior = self.histories.get(members)
          required = BOSCH_CAMERA_EXTENDED_TEST_INTERVALS
          if prior is not None and state.oem_anchor >= 0 and prior.oem_anchor == state.oem_anchor:
            required = BOSCH_CAMERA_MATURITY_WORD1_INTERVALS
          if class_code == 1:
            # Only when the set collapses onto its nearest member: a moved OEM
            # anchor on a farther member must still wait the full requirement.
            rep = representatives.get(members)
            if rep is not None and rep.d_rel <= min(by_pid[p].d_rel for p in members):
              required = min(required, BOSCH_CAMERA_MATURITY_CLASS1_INTERVALS)
          if state.stable_intervals >= required:
            mature.append(members)
        else:
          next_history[members] = _BoschExtendedHistory(members, episode, class_code, timestamp_ns)
      elif members not in next_history:
        raise AssertionError('E2 coast created an unconfirmed extended member set')
      else:
        # E2 association은 유지되지만 이 scan에 strict edge가 없는 coast다. 마지막
        # strict 확인 이후 bounded window 안이고 모든 member의 운동 연속성이 계속
        # 성립하면 누적 interval을 보존한다. coast 중에는 mature에 넣지 않으므로
        # publication은 축소되지 않는다(camera 재확인 전에는 두 contact 유지).
        state = next_history[members]
        if (self.mode == BOSCH_CAMERA_EXTENDED_PUBLICATION and state.observations and
            state.stable_intervals > 0 and
            timestamp_ns - state.last_confirm_ns <= BOSCH_CAMERA_MATURITY_HOLD_NS):
          coasted = self._maturity(
            members, state.cam_key, state.class_code, timestamp_ns, by_pid, yaw_rate,
            confirmed=False, confirm_ns=state.last_confirm_ns)
          next_history[members] = coasted
          required = (BOSCH_CAMERA_MATURITY_WORD1_INTERVALS
                      if coasted.oem_anchor >= 0 and state.oem_anchor == coasted.oem_anchor
                      else BOSCH_CAMERA_EXTENDED_TEST_INTERVALS)
          # Without a camera edge this scan, collapse only onto the nearest
          # member: a coast must never hide a nearer surface of the set.
          coast_rep = representatives.get(members)
          if (members in prior_mature and coasted.stable_intervals >= required and
              all(by_pid[p].timestamp_ns == timestamp_ns for p in members) and
              coast_rep is not None and coast_rep.d_rel <= min(by_pid[p].d_rel for p in members)):
            mature.append(members)
        else:
          next_history[members] = _BoschExtendedHistory(members, state.cam_key, state.class_code,
                                                        state.last_confirm_ns)
    self.last_maturity_resets = sum(bool(old.observations) and
      (key not in next_history or not next_history[key].observations or next_history[key].stable_intervals == 0)
      for key, old in prior_maturity.items())
    self.maturity_resets += self.last_maturity_resets
    self.mature_groups = tuple(mature)
    self.histories = next_history
    self.max_state_count = max(self.max_state_count, len(self.histories))
    self.last_groups = tuple(sorted(tuple(sorted(group)) for group in groups if len(group) > 1))
    # qualifier와 allocator 입력은 모든 모드에서 baseline 그대로 보존한다.
    return objects


def bosch_numpy_linear_sum_assignment(cost_matrix, *, potentials=False):
  """Jonker-Volgenant assignment. potentials also returns the dual row/column
  potentials, which satisfy cost - u - v >= 0 with equality on every match."""
  cost = np.asarray(cost_matrix, dtype=float)
  if cost.ndim != 2:
    raise ValueError('expected a matrix')
  if np.isnan(cost).any() or np.isneginf(cost).any():
    raise ValueError('matrix contains invalid numeric entries')
  transposed = cost.shape[0] > cost.shape[1]
  if transposed:
    cost = cost.T
  n, m = cost.shape
  if not n:
    empty = np.empty(0, dtype=int)
    return (empty, empty, np.empty(0), np.empty(0)) if potentials else (empty, empty)
  u, v = np.zeros(n + 1), np.zeros(m + 1)
  p, way = np.zeros(m + 1, dtype=int), np.zeros(m + 1, dtype=int)
  # Scratch reused across rows. The search only ever touches columns 1..m, so
  # every vector stays full width and is masked in place: the arithmetic per
  # column is the same as the gather/scatter form, without its temporaries.
  reduced = np.empty(m)
  improve = np.empty(m, dtype=bool)
  minimum = np.empty(m + 1)
  used = np.empty(m + 1, dtype=bool)
  unused = np.empty(m + 1, dtype=bool)
  path_rows = np.empty(m + 1, dtype=int)
  v_tail, minimum_tail, way_tail = v[1:], minimum[1:], way[1:]
  for row in range(1, n + 1):
    p[0] = row
    minimum.fill(np.inf)
    used.fill(False)
    unused.fill(True)
    column = 0
    depth = 0
    while True:
      used[column] = True
      unused[column] = False
      # A used column is never revisited, so parking it at +inf lets the plain
      # argmin below select exactly the column the masked argmin selected.
      minimum[column] = np.inf
      current_row = p[column]
      path_rows[depth] = current_row
      depth += 1
      np.subtract(cost[current_row - 1], u[current_row], out=reduced)
      np.subtract(reduced, v_tail, out=reduced)
      np.less(reduced, minimum_tail, out=improve)
      np.logical_and(improve, unused[1:], out=improve)
      np.copyto(minimum_tail, reduced, where=improve)
      way_tail[improve] = column
      next_column = int(np.argmin(minimum))
      delta = minimum[next_column]
      if not np.isfinite(delta):
        raise ValueError('cost matrix is infeasible')
      u[path_rows[:depth]] += delta
      np.subtract(v, delta, out=v, where=used)
      np.subtract(minimum, delta, out=minimum, where=unused)
      column = next_column
      if p[column] == 0:
        break
    while column:
      previous_column = way[column]
      p[column] = p[previous_column]
      column = previous_column
  columns = np.flatnonzero(p[1:])
  rows = p[columns + 1] - 1
  if transposed:
    rows, columns = columns, rows
  order = np.argsort(rows)
  if potentials:
    # Restate the duals in the caller's orientation, dropping the sentinel slot.
    row_potential, column_potential = (v[1:], u[1:]) if transposed else (u[1:], v[1:])
    return rows[order], columns[order], row_potential, column_potential
  return rows[order], columns[order]


try:
  from scipy.optimize import linear_sum_assignment as bosch_linear_sum_assignment
except ImportError:
  bosch_linear_sum_assignment = bosch_numpy_linear_sum_assignment


def _bosch_integer(value: object, label: str, minimum: int, maximum: int | None = None) -> None:
  # An exact int type already excludes bool and every non-Integral, so the scan
  # path never pays for the abstract-base-class lookup. Anything else keeps the
  # original predicate, including its short-circuit before the comparisons.
  if type(value) is int:
    if minimum <= value and (maximum is None or value <= maximum):
      return
  elif not (isinstance(value, bool) or not isinstance(value, Integral) or value < minimum
            or (maximum is not None and value > maximum)):
    return
  raise ValueError(f"{label} must be an integer in [{minimum}, {maximum}]")


def _bosch_finite(value: object, label: str) -> None:
  if type(value) is float:
    if math.isfinite(value):
      return
  elif not (isinstance(value, bool) or not isinstance(value, Real) or not math.isfinite(value)):
    return
  raise ValueError(f"{label} must be finite")


@dataclass(frozen=True)
class BoschRawDetection:
  timestamp_ns: int
  slot: int
  d_rel: float
  y_rel: float
  v_rel: float
  raw_word: int = 0

  def __post_init__(self) -> None:
    # Around twenty of these are built per scan and the decoder always presents
    # exact int/float, so recognise that shape inline and only enter the shared
    # predicate for anything else. Accepted values and messages are unchanged.
    timestamp_ns, slot, raw_word = self.timestamp_ns, self.slot, self.raw_word
    d_rel, y_rel, v_rel = self.d_rel, self.y_rel, self.v_rel
    if type(timestamp_ns) is not int or timestamp_ns < 0:
      _bosch_integer(timestamp_ns, "timestamp_ns", 0)
    if type(slot) is not int or not 0 <= slot <= 31:
      _bosch_integer(slot, "slot", 0, 31)
    if type(raw_word) is not int or not 0 <= raw_word <= 2**32 - 1:
      _bosch_integer(raw_word, "raw_word", 0, 2**32 - 1)
    # Every coordinate is checked for finiteness before any range is judged:
    # a value that fails both must still report the finiteness failure.
    if type(d_rel) is not float or not math.isfinite(d_rel):
      _bosch_finite(d_rel, "d_rel")
    if type(y_rel) is not float or not math.isfinite(y_rel):
      _bosch_finite(y_rel, "y_rel")
    if type(v_rel) is not float or not math.isfinite(v_rel):
      _bosch_finite(v_rel, "v_rel")
    if not 0 <= d_rel <= 255.75:
      raise ValueError("d_rel outside the supported Bosch decoded range")
    if not -32 <= y_rel <= 31.96875:
      raise ValueError("y_rel outside the supported Bosch decoded range")
    if not -128 <= v_rel <= 127.75:
      raise ValueError("v_rel outside the supported Bosch decoded range")
    if raw_word == BOSCH_INACTIVE_WORD or raw_word & (1 << 31):
      raise ValueError("empty or unsupported bit31 record is not a RawDetection")

  @property
  def address(self) -> int:
    return 0x602 + self.slot // 2

  @property
  def half(self) -> int:
    return self.slot % 2


@dataclass(frozen=True)
class BoschRawTrackingConfig:
  distance_gate_m: float = 3.5
  speed_gate_mps: float = 2.0
  bearing_near_deg: float = 12.0
  bearing_mid_deg: float = 5.0
  bearing_far_deg: float = 2.5
  bearing_distant_deg: float = 2.0
  coast_s: float = 0.3
  same_slot_bonus: float = 0.03
  unmatched_cost: float = 0.65

  def __post_init__(self) -> None:
    for name in self.__dataclass_fields__:
      value = getattr(self, name)
      _bosch_finite(value, name)
      if name == "same_slot_bonus":
        if not 0 <= value <= 0.1:
          raise ValueError("same_slot_bonus must remain a weak preference in [0, 0.1]")
      elif value <= 0:
        raise ValueError(f"{name} must be positive")
    for name in ("bearing_near_deg", "bearing_mid_deg", "bearing_far_deg", "bearing_distant_deg"):
      if getattr(self, name) > 180:
        raise ValueError(f"{name} must be <= 180 degrees")

  def bearing_gate_rad(self, distance_m: float) -> float:
    if distance_m < 15:
      degrees = self.bearing_near_deg
    elif distance_m < 30:
      degrees = self.bearing_mid_deg
    elif distance_m < 60:
      degrees = self.bearing_far_deg
    else:
      degrees = self.bearing_distant_deg
    return math.radians(degrees)


@dataclass(frozen=True)
class BoschRawTrack:
  raw_track_id: int
  detection: BoschRawDetection
  age_scans: int
  recovered: bool = False
  previous_slot: int | None = None

  @property
  def timestamp_ns(self) -> int:
    return self.detection.timestamp_ns

  @property
  def slot(self) -> int:
    return self.detection.slot

  @property
  def address(self) -> int:
    return self.detection.address

  @property
  def half(self) -> int:
    return self.detection.half

  @property
  def d_rel(self) -> float:
    return self.detection.d_rel

  @property
  def y_rel(self) -> float:
    return self.detection.y_rel

  @property
  def v_rel(self) -> float:
    return self.detection.v_rel


@dataclass(frozen=True)
class BoschRawAssociationDecision:
  """One raw-return association, recorded for shadow diagnostics only.

  Positions are in the scan-time rotated frame the association actually uses,
  so residual_d_m and residual_bearing_rad are the quantities the distance and
  bearing gates were applied to. best_alternative_cost is the cheapest edge
  this detection did not take, which is what makes a swap visible offline.
  identity_ambiguous marks a detection in a globally tied component; it is
  diagnostic only and does not change the historical deterministic assignment.
  """
  timestamp_ns: int
  raw_track_id: int
  slot: int
  previous_slot: int | None
  created: bool
  recovered: bool
  age_scans: int
  d_rel: float
  y_rel: float
  v_rel: float
  predicted_d_rel: float | None
  predicted_y_rel: float | None
  residual_d_m: float | None
  residual_bearing_rad: float | None
  bearing_gate_rad: float | None
  chosen_cost: float | None
  best_alternative_cost: float | None
  candidate_count: int
  identity_ambiguous: bool


@dataclass
class _BoschRawState:
  track: BoschRawTrack
  # Association-only state aligned to the last update's scan timestamp.
  x: float
  y: float
  last_seen_scan_ns: int
  last_seen_update: int


def _bosch_advance(x: float, y: float, velocity: float, dt: float, yaw_rate: float | None) -> tuple[float, float]:
  x += velocity * dt
  angle = 0.0 if yaw_rate is None else yaw_rate * dt
  c, s = math.cos(angle), math.sin(angle)
  return c * x + s * y, -s * x + c * y


def _bosch_admissible_cycle(size, adjacency, real):
  """Iterative Tarjan: True when a nontrivial component contains a real vertex.

  Every vertex of a nontrivial strongly connected component lies on a cycle, so
  such a component holding a real row or column means an alternating cycle of
  zero reduced cost re-matches that vertex: a second optimum. Cycles confined to
  the dummy block only permute which unmatched row pairs with which unmatched
  column, which no caller can observe.
  """
  index = [0] * size
  low = [0] * size
  seen = bytearray(size)
  on_stack = bytearray(size)
  stack = []
  counter = 1
  for root in range(size):
    if seen[root]:
      continue
    work = [(root, 0)]
    while work:
      vertex, position = work[-1]
      if not position:
        seen[vertex] = 1
        index[vertex] = low[vertex] = counter
        counter += 1
        stack.append(vertex)
        on_stack[vertex] = 1
      neighbours = adjacency[vertex]
      descended = False
      while position < len(neighbours):
        other = neighbours[position]
        position += 1
        if not seen[other]:
          work[-1] = (vertex, position)
          work.append((other, 0))
          descended = True
          break
        if on_stack[other] and index[other] < low[vertex]:
          low[vertex] = index[other]
      if descended:
        continue
      work.pop()
      if work:
        parent = work[-1][0]
        if low[vertex] < low[parent]:
          low[parent] = low[vertex]
      if low[vertex] == index[vertex]:
        members = []
        while True:
          other = stack.pop()
          on_stack[other] = 0
          members.append(other)
          if other == vertex:
            break
        if len(members) > 1 and any(real[member] for member in members):
          return True
  return False


def _bosch_rectangular_unique(rows, columns, row_edges, unmatched_cost, margin):
  """Certify an optional real matching without solving the square dummy block.

  For k real matches the square cost is sum(edges) + (a+b-2*k)*U.
  Keeping only real rows and charging 2*U for each private miss gives
  sum(edges) + (a-k)*2*U: the difference is the constant (b-a)*U.
  This preserves the optimum, but not the solver's choice among tied optima.
  Return only a strictly unique matching; every refusal keeps the square path.
  """
  a, b = len(rows), len(columns)
  tolerance = max(margin, 1e-12)
  # A square admissible cycle has at most a+b non-matching edges. Widen the
  # rectangular graph conservatively so a near tie refused by that certificate
  # cannot become a fast success merely because the solver found other duals.
  threshold = 4 * tolerance * (a + b + 1)
  if not math.isfinite(threshold):
    return None
  costs = np.full((a, a + b), np.inf)
  column_at = {vertex: j for j, vertex in enumerate(columns)}
  for i, vertex in enumerate(rows):
    costs[i, b + i] = 2 * unmatched_cost
    for other, cost in row_edges[vertex]:
      j = column_at.get(other)
      if j is None:
        return None
      costs[i, j] = cost
  try:
    solved_rows, solved_columns, u, v = bosch_numpy_linear_sum_assignment(costs, potentials=True)
  except ValueError:
    return None
  reduced = costs - u[:, None] - v[None, :]
  occupied = np.zeros(a + b, dtype=bool)
  occupied[solved_columns] = True
  if (len(solved_rows) != a or not np.isfinite(u).all() or not np.isfinite(v).all() or
      np.any(reduced < -tolerance) or np.any(np.abs(reduced[solved_rows, solved_columns]) > tolerance) or
      np.any(v > tolerance) or np.any(np.abs(v[~occupied]) > tolerance)):
    return None

  # Rectangular alternatives are alternating cycles OR alternating paths from
  # an occupied zero-potential column to a free column. A reservoir closes
  # exactly these paths into cycles; omitting it would falsely certify ties
  # that move a birth or miss. Each private dummy column belongs to only one
  # row, so any cycle containing a row changes an observable real assignment.
  reservoir = 2 * a + b
  adjacency = [[] for _ in range(reservoir + 1)]
  partner = dict(zip(solved_rows.tolist(), solved_columns.tolist()))
  for i, matched in partner.items():
    adjacency[a + matched].append(i)
    adjacency[i].extend(a + j for j in np.flatnonzero(reduced[i] <= threshold).tolist() if j != matched)
  for j in range(a + b):
    if not occupied[j]:
      adjacency[a + j].append(reservoir)
    elif v[j] >= -threshold:
      adjacency[reservoir].append(a + j)
  real = bytearray(reservoir + 1)
  real[:a] = b'\x01' * a
  if _bosch_admissible_cycle(reservoir + 1, adjacency, real):
    return None
  return {columns[j]: rows[i] for i, j in partner.items() if j < b}


def _bosch_component_matching(rows, columns, row_edges, unmatched_cost, margin=None):
  """Solve one component's augmented assignment, optionally certifying it.

  The augmented objective is a sum of independent per-component terms, so a
  component with a strictly unique optimum carries that same matching inside
  every global optimum. Certifying it therefore replaces the whole-scan solver
  without moving a single association.

  Returns (matching, unique), or None when the component is not self-contained
  or the solve fails. Passing no margin skips the certificate and reports
  unique False; the matching is still an optimum of this component, and with
  rows and columns in ascending order it is the one the whole-scan matrix
  reaches whenever that matrix's own tie choice is a property of the component.
  """
  if margin is not None:
    matching = _bosch_rectangular_unique(rows, columns, row_edges, unmatched_cost, margin)
    if matching is not None:
      return matching, True
  a, b = len(rows), len(columns)
  size = a + b
  costs = np.full((size, size), np.inf)
  column_at = {vertex: j for j, vertex in enumerate(columns)}
  for i, vertex in enumerate(rows):
    costs[i, b + i] = unmatched_cost
    for other, cost in row_edges[vertex]:
      j = column_at.get(other)
      if j is None:
        return None  # Not a closed component; leave it to the global solver.
      costs[i, j] = cost
  for j in range(b):
    costs[a + j, j] = unmatched_cost
  costs[a:, b:] = 0.
  try:
    if margin is None:
      solved_rows, solved_columns = bosch_numpy_linear_sum_assignment(costs)
    else:
      solved_rows, solved_columns, u, v = bosch_numpy_linear_sum_assignment(costs, potentials=True)
  except ValueError:
    return None
  partner = dict(zip(solved_rows.tolist(), solved_columns.tolist()))
  if len(partner) != size:
    return None
  matching = {columns[j]: rows[i] for i, j in partner.items() if i < a and j < b}
  if margin is None:
    return matching, False
  admissible = (costs - u[:, None] - v[None, :]) <= max(margin, 1e-12)
  adjacency = [[] for _ in range(2 * size)]
  for i in range(size):
    matched = partner[i]
    # Matching edges point column to row, admissible non-matching edges point
    # row to column, so every directed cycle is an alternating swap.
    adjacency[size + matched].append(i)
    outgoing = adjacency[i]
    for j in np.flatnonzero(admissible[i]).tolist():
      if j != matched:
        outgoing.append(size + j)
  real = bytearray(2 * size)
  for i in range(a):
    real[i] = 1
  for j in range(b):
    real[size + j] = 1
  return matching, not _bosch_admissible_cycle(2 * size, adjacency, real)


def _bosch_raw_unique_component(rows, columns, row_edges, column_edges, unmatched_cost):
  # Each real match replaces one birth and one miss, so maximizing the sum of
  # (2*unmatched_cost - edge_cost) is exactly the augmented objective minus a
  # constant. A unique real matching is independent of all dummy assignments.
  reward = 2 * unmatched_cost
  margin = 1e-12 * max(1.0, abs(reward)) * (len(rows) + len(columns) + 1)
  if not math.isfinite(margin):
    return None
  if len(rows) == 1 and len(columns) == 1:
    # The lone pair, which is most of them. It carries exactly one edge, so the
    # certificate is a single comparison; below that strictness the general
    # paths refuse as well, and the caller solves the pair itself.
    saving = reward - row_edges[rows[0]][0][1]
    if saving > margin:
      return {columns[0]: rows[0]}
    return {} if -saving > margin else None

  # Distinct strict vertex-best choices attain the sum of independent upper
  # bounds. This certifies the unique optimum, even in a large component.
  for vertices, adjacency, transposed in ((rows, row_edges, False), (columns, column_edges, True)):
    matching = {}
    used = set()
    for vertex in vertices:
      best, second, partner = 0.0, -math.inf, None
      for other, cost in adjacency[vertex]:
        saving = reward - cost
        if saving > best:
          best, second, partner = saving, best, other
        elif saving > second:
          second = saving
      if best - second <= margin or (partner is not None and partner in used):
        break
      if partner is not None:
        used.add(partner)
        if transposed:
          matching[vertex] = partner
        else:
          matching[partner] = vertex
    else:
      return matching

  # Exact bounded optional matching DP. Keep the runner-up, including equal
  # optima; ambiguous or numerically indistinguishable results are left to the
  # caller's tie path, which solves the component itself.
  if min(len(rows), len(columns)) > 6 or max(len(rows), len(columns)) > 12:
    # Too wide for the subset DP. Certify the component's own optimum instead;
    # only a genuinely tied component reaches the caller's tie path.
    solved = _bosch_component_matching(rows, columns, row_edges, unmatched_cost, margin)
    return solved[0] if solved is not None and solved[1] else None
  transposed = len(rows) < len(columns)
  small, large = (rows, columns) if transposed else (columns, rows)
  bits = {vertex: 1 << i for i, vertex in enumerate(small)}
  adjacency = column_edges if transposed else row_edges
  options = [[(bits[other], reward - cost,
               (vertex, other) if transposed else (other, vertex))
              for other, cost in adjacency[vertex]] for vertex in large]
  memo = {}

  def solve(index, occupied):
    if index == len(options):
      return 0.0, -math.inf, ()
    key = (index, occupied)
    cached = memo.get(key)
    if cached is not None:
      return cached
    best, second, matching = solve(index + 1, occupied)
    for bit, saving, pair in options[index]:
      if occupied & bit:
        continue
      child_best, child_second, child_matching = solve(index + 1, occupied | bit)
      candidate = saving + child_best
      if candidate > best:
        best, second, matching = candidate, best, (pair,) + child_matching
      elif candidate > second:
        second = candidate
      runner_up = saving + child_second
      if runner_up > second:
        second = runner_up
    result = (best, second, matching)
    memo[key] = result
    return result

  try:
    best, second, matching = solve(0, 0)
  finally:
    # Break the recursive closure cycle even when process-wide GC is disabled.
    solve = None
  return dict(matching) if best - second > margin else None


def _bosch_raw_assignment(n, m, row_edges, column_edges, unmatched_cost):
  components = []
  visited_rows, visited_columns = set(), set()
  for first in range(n):
    edges = row_edges[first]
    if not edges or first in visited_rows:
      continue
    if len(edges) == 1:
      column = edges[0][0]
      if len(column_edges[column]) == 1:
        # Two thirds of the components are one state and one return that
        # nothing else can reach; that is the component, so skip the walk.
        visited_rows.add(first)
        visited_columns.add(column)
        components.append(([first], [column]))
        continue
    rows, columns, pending = [], [], [first]
    while pending:
      vertex = pending.pop()
      if vertex >= 0:
        if vertex in visited_rows:
          continue
        visited_rows.add(vertex)
        rows.append(vertex)
        pending.extend(~column for column, _ in row_edges[vertex] if column not in visited_columns)
      else:
        column = ~vertex
        if column in visited_columns:
          continue
        visited_columns.add(column)
        columns.append(column)
        pending.extend(row for row, _ in column_edges[column] if row not in visited_rows)
    components.append((rows, columns))
  largest = max((len(rows) + len(columns) for rows, columns in components), default=0)
  assignment = {}
  certified = 0
  ambiguous_columns = None
  for rows, columns in components:
    result = _bosch_raw_unique_component(rows, columns, row_edges, column_edges, unmatched_cost)
    if result is not None:
      certified += 1
    else:
      # Record exact identity ambiguity, but retain the historical deterministic
      # tie-break until replay plus video GT can show that a generation break is
      # safer. Refusing here can create more coasted states and cascade ties.
      solved = _bosch_component_matching(sorted(rows), sorted(columns), row_edges, unmatched_cost)
      if solved is not None:
        result, unique = solved
        if not unique:
          if ambiguous_columns is None:
            ambiguous_columns = set(columns)
          else:
            ambiguous_columns.update(columns)
    if result is None:
      # Not self-contained, or the component solver refused it. Rebuild the
      # exact old matrix so this path stays a faithful last resort.
      costs = np.full((n + m, n + m), np.inf)
      for row, edges in enumerate(row_edges):
        costs[row, m + row] = unmatched_cost
        for column, cost in edges:
          costs[row, column] = cost
      for column in range(m):
        costs[n + column, column] = unmatched_cost
      costs[n:, m:] = 0.0
      ri, ci = bosch_linear_sum_assignment(costs)
      assignment = {int(column): int(row) for row, column in zip(ri, ci) if row < n and column < m}
      return assignment, len(components), largest, 0, True, frozenset()
    assignment.update(result)
  return assignment, len(components), largest, certified, False, (frozenset() if ambiguous_columns is None else frozenset(ambiguous_columns))


class BoschRawTrackManager:
  """Optimal one-to-one association across all slots with internal coasting.

  update() requires strictly increasing scan timestamps. Each detection may be
  slightly older than its scan availability timestamp, but must be newer than
  the preceding scan. Its state is projected to scan time only for association;
  the returned detection always contains the original, unsmoothed observation.
  age_scans counts matched observations (not elapsed or coasted scans).

  Missing tracks remain eligible through coast_s inclusive; they are never
  emitted. Slots influence cost weakly and cannot override any hard gate.
  Rejected arguments and ID exhaustion leave manager state unchanged.
  """

  def __init__(self, config: BoschRawTrackingConfig | None = None, *, first_id: int = 1) -> None:
    self.config = config if config is not None else BoschRawTrackingConfig()
    if not isinstance(self.config, BoschRawTrackingConfig):
      raise TypeError("config must be RawTrackingConfig")
    _bosch_integer(first_id, "first_id", 1, BOSCH_MAX_RAW_TRACK_ID)
    self.next_id = int(first_id)
    self.last_timestamp_ns: int | None = None
    self._update_count = 0
    self._states: dict[int, _BoschRawState] = {}
    self.last_pair_possible = self.last_pair_candidates = 0
    self.last_component_count = self.last_largest_component = 0
    self.last_fast_components = self.last_tied_components = 0
    self.last_ambiguous_detections = 0
    self.last_solver_fallback = False
    self.stats = {name: 0 for name in (
      "updates", "detections", "created", "deleted", "assignments",
      "cross_slot", "recovered", "coasted_track_scans", "max_active",
      "gated_pairs", "distance_rejections", "speed_rejections", "bearing_rejections",
      "yaw_compensated_updates", "yaw_unavailable_updates",
      "ambiguous_identity_components", "ambiguous_identity_detections",
    )}
    # Shadow diagnostic switch, matching the group manager. Off leaves the scan
    # path unchanged; on only fills last_decisions. Neither setting gates a match.
    self.trace_decisions = False
    self.last_decisions: tuple[BoschRawAssociationDecision, ...] = ()
    # Only current-scan ambiguity evidence; no expensive association trace.
    self.last_identity_evidence: dict[int, tuple[int, int, bool]] = {}

  @property
  def active_count(self) -> int:
    """Internal live plus coasted return hypotheses; not an output count."""
    return len(self._states)

  def update(self, timestamp_ns: int, detections: Sequence[BoschRawDetection], yaw_rate: float | None = None) -> tuple[BoschRawTrack, ...]:
    _bosch_integer(timestamp_ns, "timestamp_ns", 0)
    if self.last_timestamp_ns is not None and timestamp_ns <= self.last_timestamp_ns:
      raise ValueError("scan timestamps must be strictly increasing")
    if yaw_rate is not None:
      _bosch_finite(yaw_rate, "yaw_rate")
    current = tuple(detections)
    if len(current) > 32:
      raise ValueError("a Bosch scan cannot contain more than 32 detections")
    slots = set()
    for detection in current:
      if not isinstance(detection, BoschRawDetection):
        raise TypeError("detections must contain RawDetection instances")
      if detection.timestamp_ns > timestamp_ns:
        raise ValueError("detection timestamp is later than scan availability")
      if self.last_timestamp_ns is not None and detection.timestamp_ns <= self.last_timestamp_ns:
        raise ValueError("stale/repeated detection cannot be tracked as fresh")
      if detection.slot in slots:
        raise ValueError("a scan must contain at most one detection per slot")
      slots.add(detection.slot)
    current = tuple(sorted(current, key=lambda point: point.slot))
    config = self.config
    coast_ns = round(config.coast_s * 1e9)
    states = self._states
    retained = []
    for key in sorted(states):
      state = states[key]
      if timestamp_ns - state.last_seen_scan_ns <= coast_ns:
        retained.append(state)
    expired_count = len(states) - len(retained)
    dt = 0.0 if self.last_timestamp_ns is None else (timestamp_ns - self.last_timestamp_ns) / 1e9
    angle = 0.0 if yaw_rate is None else yaw_rate * dt
    cosine, sine = math.cos(angle), math.sin(angle)
    predicted = []
    # Retained velocities and slots are read again by the gate loop; keep them
    # off the RawTrack/RawDetection property chain there.
    retained_speeds = []
    retained_slots = []
    for state in retained:
      detection = state.track.detection
      speed = detection.v_rel
      retained_speeds.append(speed)
      retained_slots.append(detection.slot)
      x = state.x + speed * dt
      predicted.append((cosine * x + sine * state.y, -sine * x + cosine * state.y))
    observed = []
    rotations = {}
    for point in current:
      rotation = rotations.get(point.timestamp_ns)
      if rotation is None:
        sample_dt = (timestamp_ns - point.timestamp_ns) / 1e9
        angle = 0.0 if yaw_rate is None else yaw_rate * sample_dt
        rotation = (sample_dt, math.cos(angle), math.sin(angle))
        rotations[point.timestamp_ns] = rotation
      sample_dt, cosine, sine = rotation
      x = point.d_rel + point.v_rel * sample_dt
      observed.append((cosine * x + sine * point.y_rel, -sine * x + cosine * point.y_rel))
    n, m = len(retained), len(current)
    predicted_bearings = [math.atan2(y, x) for x, y in predicted]
    observed_bearings = [math.atan2(y, x) for x, y in observed]
    predicted_x = [point[0] for point in predicted]
    observed_x = [point[0] for point in observed]
    row_edges, column_edges = [[] for _ in range(n)], [[] for _ in range(m)]
    distance_pairs = speed_rejections = bearing_rejections = gated_pairs = 0
    # Same keys the equivalent lambdas produced, read at C speed.
    order = sorted(range(m), key=observed_x.__getitem__)
    lower = upper = 0
    # Per-scan constants. bearing_gate_rad() is inlined below with exactly the
    # same thresholds and math.radians() values it would have returned.
    distance_gate = config.distance_gate_m
    speed_gate = config.speed_gate_mps
    same_slot_bonus = config.same_slot_bonus
    gate_near = math.radians(config.bearing_near_deg)
    gate_mid = math.radians(config.bearing_mid_deg)
    gate_far = math.radians(config.bearing_far_deg)
    gate_distant = math.radians(config.bearing_distant_deg)
    observed_speeds = [point.v_rel for point in current]
    observed_slots = [point.slot for point in current]
    window_gate = math.nextafter(distance_gate, math.inf)
    for row in sorted(range(n), key=predicted_x.__getitem__):
      pred_x = predicted_x[row]
      state_speed = retained_speeds[row]
      state_slot = retained_slots[row]
      predicted_bearing = predicted_bearings[row]
      row_edge = row_edges[row]
      # Cover values whose subtraction rounds onto the gate, including when
      # pred_x - gate cancels to zero. The actual gate below stays unchanged.
      minimum = math.nextafter(pred_x - window_gate, -math.inf)
      maximum = math.nextafter(pred_x + window_gate, math.inf)
      while lower < m and observed_x[order[lower]] < minimum:
        lower += 1
      upper = max(upper, lower)
      while upper < m and observed_x[order[upper]] <= maximum:
        upper += 1
      for index in range(lower, upper):
        col = order[index]
        obs_x = observed_x[col]
        delta_d = abs(obs_x - pred_x)
        if delta_d > distance_gate:
          continue
        distance_pairs += 1
        delta_v = abs(observed_speeds[col] - state_speed)
        if delta_v > speed_gate:
          speed_rejections += 1
          continue
        angle = observed_bearings[col] - predicted_bearing
        delta_bearing = abs(math.atan2(math.sin(angle), math.cos(angle)))
        nearest = pred_x if pred_x < obs_x else obs_x
        if nearest < 0.0:
          nearest = 0.0
        if nearest < 15:
          gate = gate_near
        elif nearest < 30:
          gate = gate_mid
        elif nearest < 60:
          gate = gate_far
        else:
          gate = gate_distant
        if delta_bearing > gate:
          bearing_rejections += 1
        else:
          gated_pairs += 1
          cost = ((delta_d / distance_gate)**2 + (delta_v / speed_gate)**2 + (delta_bearing / gate)**2) / 3
          if state_slot == observed_slots[col]:
            cost = max(0.0, cost - same_slot_bonus)
          row_edge.append((col, cost))
          column_edges[col].append((row, cost))
    assignment, components, largest, fast_components, fallback, ambiguous_columns = _bosch_raw_assignment(
      n, m, row_edges, column_edges, config.unmatched_cost,
    )
    pending_stats = {"gated_pairs": gated_pairs, "distance_rejections": n * m - distance_pairs,
                     "speed_rejections": speed_rejections, "bearing_rejections": bearing_rejections}
    births = m - len(assignment)
    if self.next_id + births - 1 > BOSCH_MAX_RAW_TRACK_ID:
      raise OverflowError("raw return Int32 ID space exhausted; IDs must not wrap or be reused")

    # Commit only after all validation and assignment have succeeded.
    self.last_pair_possible, self.last_pair_candidates = n * m, distance_pairs
    self.last_component_count, self.last_largest_component = components, largest
    self.last_fast_components, self.last_solver_fallback = fast_components, fallback
    self.last_tied_components = 0 if fallback else components - fast_components
    self.last_ambiguous_detections = len(ambiguous_columns)
    self._update_count += 1
    update_count = self._update_count
    # A coasted state only advances its projection, so it is re-seated in place
    # rather than rebuilt. Rows that a detection claims are overwritten below,
    # so they are not carried over at all.
    claimed_rows = set(assignment.values())
    states = {}
    self._states = states
    for row, (state, (x, y)) in enumerate(zip(retained, predicted)):
      if row in claimed_rows:
        continue
      state.x = x
      state.y = y
      states[state.track.raw_track_id] = state
    output = []
    decisions = []
    identity_evidence = {}
    assignments = cross_slot = recovered_count = created = 0
    for col, (point, (x, y)) in enumerate(zip(current, observed)):
      row = assignment.get(col)
      if row is not None:
        old = retained[row]
        old_track = old.track
        recovered = old.last_seen_update != update_count - 1
        track = BoschRawTrack(old_track.raw_track_id, point, old_track.age_scans + 1, recovered, old_track.slot)
        assignments += 1
        cross_slot += point.slot != old_track.slot
        recovered_count += recovered
      else:
        track = BoschRawTrack(self.next_id, point, 1)
        self.next_id += 1
        created += 1
      output.append(track)
      states[track.raw_track_id] = _BoschRawState(track, x, y, timestamp_ns, update_count)
      identity_evidence[track.raw_track_id] = (point.timestamp_ns, len(column_edges[col]), col in ambiguous_columns)
      if self.trace_decisions:
        chosen = next((cost for other, cost in column_edges[col] if other == row), None)
        alternatives = sorted(cost for other, cost in column_edges[col] if other != row)
        bearing = None
        if row is not None:
          angle = observed_bearings[col] - predicted_bearings[row]
          bearing = math.atan2(math.sin(angle), math.cos(angle))
        decisions.append(BoschRawAssociationDecision(
          timestamp_ns=timestamp_ns, raw_track_id=track.raw_track_id, slot=point.slot,
          previous_slot=track.previous_slot, created=col not in assignment,
          recovered=track.recovered, age_scans=track.age_scans,
          d_rel=point.d_rel, y_rel=point.y_rel, v_rel=point.v_rel,
          predicted_d_rel=predicted[row][0] if row is not None else None,
          predicted_y_rel=predicted[row][1] if row is not None else None,
          residual_d_m=x-predicted[row][0] if row is not None else None,
          residual_bearing_rad=bearing,
          bearing_gate_rad=(config.bearing_gate_rad(max(0.0, min(predicted[row][0], x)))
                            if row is not None else None),
          chosen_cost=chosen, best_alternative_cost=alternatives[0] if alternatives else None,
          candidate_count=len(column_edges[col]), identity_ambiguous=col in ambiguous_columns))
    self.last_decisions = tuple(decisions)
    self.last_identity_evidence = identity_evidence
    self.last_timestamp_ns = int(timestamp_ns)
    stats = self.stats
    for key, value in pending_stats.items():
      stats[key] += value
    stats["assignments"] += assignments
    stats["cross_slot"] += cross_slot
    stats["recovered"] += recovered_count
    stats["created"] += created
    stats["updates"] += 1
    stats["detections"] += m
    stats["deleted"] += expired_count
    stats["coasted_track_scans"] += n - len(assignment)
    stats["ambiguous_identity_components"] += self.last_tied_components
    stats["ambiguous_identity_detections"] += len(ambiguous_columns)
    stats["max_active"] = max(stats["max_active"], len(states))
    stats["yaw_compensated_updates" if yaw_rate is not None else "yaw_unavailable_updates"] += 1
    return tuple(output)


@dataclass(frozen=True)
class BoschGroupingConfig:
  distance_diameter_m: float = 3.0
  lateral_diameter_m: float = 1.5
  velocity_diameter_mps: float = 1.5
  min_pair_observations: int = 3
  min_pair_span_s: float = .18
  pair_max_gap_s: float = .16
  evidence_window_s: float = .8
  # Reject expanding separation, not a return settling toward an existing one.
  max_relative_distance_growth_m: float = 1.0
  max_relative_lateral_growth_m: float = .75
  stationary_speed_mps: float = .6
  moving_speed_mps: float = 1.4
  max_members: int = 8
  coast_s: float = .3
  first_physical_id: int = 1_000_000
  provisional_enabled: bool = True
  # SHAPE_MEMORY_2000 retention with the world-robust surface guard. Recorded
  # morphology is not identity evidence: a retained family may only hide
  # members the published surface covers (see _BoschMatureGroupRetention).
  mature_retention_enabled: bool = False
  provisional_min_distance_m: float = 1.0
  provisional_max_distance_m: float = 2.0
  provisional_max_lateral_m: float = .25
  provisional_max_bearing_deg: float = .20
  provisional_max_velocity_mps: float = .25
  provisional_max_world_speed_mps: float = .05
  provisional_min_range_m: float = 40.
  provisional_min_world_speed_mps: float = 4.

  def __post_init__(self):
    for name, value in vars(self).items():
      if name in ('provisional_enabled', 'mature_retention_enabled'):
        if not isinstance(value, bool):
          raise ValueError(f'{name} must be boolean')
        continue
      if isinstance(value, bool) or not math.isfinite(value) or value <= 0:
        raise ValueError(f'{name} must be positive and finite')
    if self.min_pair_observations < 2 or int(self.min_pair_observations) != self.min_pair_observations:
      raise ValueError('at least two integer observations are required')
    if not 2 <= self.max_members <= 32 or int(self.max_members) != self.max_members:
      raise ValueError('max_members must be an integer in [2,32]')
    if self.stationary_speed_mps >= self.moving_speed_mps:
      raise ValueError('kinematic stationary and moving bands must be disjoint')
    if self.min_pair_span_s > self.evidence_window_s:
      raise ValueError('evidence window must contain the minimum pair span')
    if not isinstance(self.first_physical_id, int) or self.first_physical_id >= 2**31:
      raise ValueError('physical IDs must fit positive downstream Int32')
    if self.provisional_min_distance_m >= self.provisional_max_distance_m:
      raise ValueError('provisional distance band must be nonempty')
    if self.provisional_max_distance_m > self.distance_diameter_m:
      raise ValueError('provisional pairs must remain inside normal grouping geometry')


@dataclass(frozen=True)
class BoschVisionCue:
  """Association support only. No model horizon is treated as a new object ID."""
  d_rel: float
  y_rel: float
  probability: float
  distance_tolerance_m: float = 8.
  lateral_tolerance_m: float = 1.5


def _bosch_sample_ns(row):
  return row[0]


class _BoschModelLine:
  __slots__ = ('x', 'y')

  def __init__(self, line):
    self.x = tuple(line.x)
    self.y = tuple(line.y)


class BoschModelSnapshot:
  """Plain copy of the modelV2 fields Bosch reads, taken once per message.

  The context is set at 100 Hz while modelV2 changes at 20 Hz, and three
  consumers read the same road edges. Reading the capnp lists once leaves the
  values unchanged; only the repeated capnp access goes away. Never modified
  after construction; consumers may rely on that for the same object.
  """
  __slots__ = ('timestampEof', 'roadEdges', 'laneLines', 'position', 'roadEdgeStds',
               'laneLineProbs', 'laneLineStds', 'cues')

  def __init__(self, model):
    self.timestampEof = getattr(model, 'timestampEof', 0)
    self.roadEdges = tuple(_BoschModelLine(edge) for edge in getattr(model, 'roadEdges', ()))
    self.laneLines = tuple(_BoschModelLine(lane) for lane in getattr(model, 'laneLines', ()))
    position = getattr(model, 'position', None)
    self.position = _BoschModelLine(position) if position is not None else None
    self.roadEdgeStds = tuple(getattr(model, 'roadEdgeStds', ()))
    self.laneLineProbs = tuple(getattr(model, 'laneLineProbs', ()))
    self.laneLineStds = tuple(getattr(model, 'laneLineStds', ()))
    cues = ()
    if model.leadsV3:
      lead = model.leadsV3[0]
      if lead.x and lead.y:
        # Same model-to-radar coordinates as the existing primary matcher.
        cues = (BoschVisionCue(float(lead.x[0]) - 1.52, -float(lead.y[0]), float(lead.prob)),)
    self.cues = cues


@dataclass(frozen=True)
class BoschPublishedSurface:
  """The one member whose geometry downstream receives for a physical object.

  A physical object carries two roles that used to share one set of fields.

  CONTINUITY ANCHOR -- `d_rel`/`y_rel`/`v_rel` and `representative_raw_track_id`
  on `BoschPhysicalObject`. Internal state: the next scan projects it forward,
  the representative-continuity term scores against that projection, and the
  physical-ID assignment score rewards a cluster that still holds it. Changing
  it changes which physical ID a cluster inherits.

  PUBLISHED SURFACE -- what the car is asked to follow longitudinally. Normally
  the same member, but it does not have to be: the most continuous return of a
  group can sit behind the vehicle's nearest surface for the life of the
  object, and the anchor has to stay there for identity while the published
  range does not.

  One member's whole tuple, never a mix of two members' coordinates.  The only
  exception is `lateral_estimate`: the member's own range and speed with the
  side-pass physical lateral estimate in place of its sliding lateral.
  """
  raw_track_id: int
  d_rel: float
  y_rel: float
  v_rel: float
  lateral_estimate: bool = False


@dataclass(frozen=True)
class BoschPhysicalObject:
  physical_track_id: int
  timestamp_ns: int
  members: tuple[BoschRawTrack, ...]
  representative_raw_track_id: int
  d_rel: float
  y_rel: float
  v_rel: float
  oem_selected: bool
  vision_supported: bool
  age_scans: int
  grouping_evidence: str
  # None means the continuity anchor above is also the published surface, which
  # is what every object the tracker itself emits carries. Only the final
  # publication stage may set one, and nothing inside the provider reads it.
  published_surface: BoschPublishedSurface | None = None

  @property
  def member_slots(self):
    return tuple(m.slot for m in self.members)


@dataclass(frozen=True)
class BoschAssociationDecision:
  """One physical-ID assignment, recorded for shadow diagnostics only.

  The tracker never reads these records back, so enabling the trace cannot
  change a published object, ID, coordinate or timestamp. It exists to answer
  one question offline: how far is the state that inherits a physical ID from
  the state that ID last published, projected forward over the scan gap?

  member_overlap counts the raw members the assignment actually scored, which
  includes members retained only for coasting. observed_member_overlap counts
  the members of the previously published object. A carry with
  observed_member_overlap == 0 kept an ID through a coasted member alone.
  """
  timestamp_ns: int
  physical_track_id: int
  previous_physical_track_id: int | None
  assignment_mode: str
  dt_s: float
  member_raw_track_ids: tuple[int, ...]
  member_slots: tuple[int, ...]
  previous_member_raw_track_ids: tuple[int, ...]
  member_overlap: int
  observed_member_overlap: int
  representative_raw_track_id: int
  previous_representative_raw_track_id: int | None
  representative_changed: bool
  representative_still_a_member: bool
  d_rel: float
  y_rel: float
  v_rel: float
  predicted_d_rel: float | None
  predicted_y_rel: float | None
  predicted_v_rel: float | None
  residual_d_m: float | None
  residual_y_m: float | None
  residual_v_mps: float | None
  best_score: float | None
  second_score: float | None
  age_scans: int
  grouping_evidence: str


@dataclass
class _BoschPairEvidence:
  samples: deque = field(default_factory=deque)


@dataclass(frozen=True, slots=True)
class _BoschCommonGroupAncestry:
  timestamp_ns: int
  physical_track_id: int


@dataclass
class _BoschPhysicalState:
  observation: BoschPhysicalObject
  # Includes temporarily missing raw members, for internal coasting only.
  member_last_seen: dict[int, int]


@dataclass
class _BoschProvisionalBundleState:
  raw_track_ids: tuple[int, int]
  timestamp_ns: int
  last_ages: tuple[int, int]
  representative_raw_track_id: int
  d_rel: float
  y_rel: float
  v_rel: float


@dataclass(frozen=True)
class BoschProvisionalBundleDecision:
  timestamp_ns: int
  action: str
  release_reason: str
  raw_track_ids: tuple[int, int]
  physical_track_ids: tuple[int, ...]
  representative_raw_track_id: int | None
  representative_physical_track_id: int | None
  delta_d_m: float | None
  delta_y_m: float | None
  delta_bearing_deg: float | None
  delta_vrel_mps: float | None
  delta_world_speed_mps: float | None


def _bosch_group_representative(candidates, raw_index, vision_supported, oem_slot, *,
                                prior=None, predicted_xy=None):
  """Select one whole raw surface using the physical group's existing cost."""
  if len(candidates) == 1:
    return candidates[0]
  if prior is not None:
    px, py = predicted_xy

    def cost(member):
      continuity = (abs(member.d_rel-px) + .5*abs(member.y_rel-py) +
                    .5*abs(member.v_rel-prior.v_rel))
      return (continuity, member.raw_track_id != prior.representative_raw_track_id,
              not vision_supported[raw_index[member.raw_track_id]], member.slot != oem_slot,
              -member.age_scans, member.raw_track_id)
  else:
    ordered = sorted(member.d_rel for member in candidates)
    middle = len(ordered)//2
    median = ordered[middle] if len(ordered) % 2 else (ordered[middle-1]+ordered[middle])/2

    def cost(member):
      return (abs(member.d_rel-median), False,
              not vision_supported[raw_index[member.raw_track_id]], member.slot != oem_slot,
              -member.age_scans, member.raw_track_id)
  return min(candidates, key=cost)


def _bosch_physical_component(rows, columns, row_edges, column_edges, margin):
  """One component's maximum-weight matching, or None when it is not unique.

  Every score is strictly positive and an unassigned cluster costs nothing, so
  the augmented square matrix the whole-scan solver builds is minimised exactly
  by a maximum-weight matching of this graph. A component whose maximum is
  strictly unique therefore holds the same pairs inside every global optimum,
  whichever way that solver happened to break its own ties.
  """
  if len(rows) == 1:
    row = rows[0]
    best = second = 0.
    partner = None
    for column, score in row_edges[row]:
      if score > best:
        best, second, partner = score, best, column
      elif score > second:
        second = score
    return {row: partner} if best-second > margin else None
  if len(columns) == 1:
    column = columns[0]
    best = second = 0.
    partner = None
    for row, score in column_edges[column]:
      if score > best:
        best, second, partner = score, best, row
      elif score > second:
        second = score
    return {partner: column} if best-second > margin else None
  if len(rows) > 8 or len(columns) > 12:
    return None
  bits = {column: 1 << i for i, column in enumerate(columns)}
  options = [[(bits[column], score, (row, column)) for column, score in row_edges[row]] for row in rows]
  memo = {}

  def solve(index, occupied):
    # Keep the runner-up as well: equal optima are what the caller must not guess.
    if index == len(options):
      return 0., -math.inf, ()
    key = (index, occupied)
    cached = memo.get(key)
    if cached is not None:
      return cached
    best, second, matching = solve(index+1, occupied)
    for bit, score, pair in options[index]:
      if occupied & bit:
        continue
      child_best, child_second, child_matching = solve(index+1, occupied | bit)
      candidate = score+child_best
      if candidate > best:
        best, second, matching = candidate, best, (pair,)+child_matching
      elif candidate > second:
        second = candidate
      runner_up = score+child_second
      if runner_up > second:
        second = runner_up
    result = (best, second, matching)
    memo[key] = result
    return result

  try:
    best, second, matching = solve(0, 0)
  finally:
    # Break the recursive closure cycle even when process-wide GC is disabled.
    solve = None
  return dict(matching) if best-second > margin else None


def _bosch_physical_assignment(row_edges, column_edges):
  """Split the physical score graph into independent parts and solve each one.

  Returns the matching, or None when some part carries more than one optimum:
  that is the only case in which the whole-scan solver's own tie order is
  observable, and the caller then rebuilds exactly the matrix it always built.
  """
  assignment = {}
  seen_rows, seen_columns = set(), set()
  for first, edges in enumerate(row_edges):
    if not edges or first in seen_rows:
      continue
    rows, columns, pending, peak = [], [], [first], 0.
    while pending:
      vertex = pending.pop()
      if vertex >= 0:
        if vertex in seen_rows:
          continue
        seen_rows.add(vertex)
        rows.append(vertex)
        for column, score in row_edges[vertex]:
          if score > peak:
            peak = score
          if column not in seen_columns:
            pending.append(~column)
      else:
        column = ~vertex
        if column in seen_columns:
          continue
        seen_columns.add(column)
        columns.append(column)
        pending.extend(row for row, _ in column_edges[column] if row not in seen_rows)
    result = _bosch_physical_component(rows, columns, row_edges, column_edges,
                                       1e-12*peak*(len(rows)+len(columns)+1))
    if result is None:
      return None
    assignment.update(result)
  return assignment


@dataclass
class _BoschMatureGroupCertificate:
  start_ns: int
  last_strict_ns: int
  ages: tuple[int, ...]
  history: list[tuple[int, dict[tuple[int, int], tuple[float, float]]]] = field(default_factory=list)
  ready: bool = False

  def clone(self) -> '_BoschMatureGroupCertificate':
    # Copy mutable containers, preserving repeated history references.
    offsets_memo = {}
    entries_memo = {}
    history = []
    for entry in self.history:
      if id(entry) not in entries_memo:
        ns, offsets = entry
        if id(offsets) not in offsets_memo:
          offsets_memo[id(offsets)] = offsets.copy()
        entries_memo[id(entry)] = (ns, offsets_memo[id(offsets)])
      history.append(entries_memo[id(entry)])
    return _BoschMatureGroupCertificate(self.start_ns, self.last_strict_ns, self.ages, history, self.ready)


class _BoschMatureGroupRetention:
  """Bounded morphology memory for exact raw families, never PID ownership.

  Only an exact group present in the previous scan may receive extra edges.
  Archived certificates can resume only after a natural strict rejoin. They
  contain no physical-object state or history and do not alter strict evidence.
  This policy reproduces SHAPE_MEMORY_2000, not physical identity.

  World-robust surface guard: whether a family is one vehicle or two cannot be
  observed, so a retained scan must never be worse than the strict split in
  either case. The retained group may only hide members that its published
  surface covers -- nearer (within GUARD_NEAR_TOL_M), not faster, and at least
  as close to the driven path or within GUARD_COVER_PATH_M of it. Every DPath
  lateral gate is at least that wide, so any gate a hidden member passes the
  surface passes too. Without a recent path the scan falls back to the split.
  """
  MATURITY_NS = 2_000_000_000
  LEASE_NS = 2_000_000_000
  MEMORY_NS = 3_000_000_000
  LIVE_MAX = 16
  ARCHIVE_MAX = 64
  HISTORY_MAX = 21
  GUARD_NEAR_TOL_M = .25
  GUARD_COVER_PATH_M = .5
  GUARD_COVER_SPEED_MPS = .25
  GUARD_PATH_HOLD_NS = 500_000_000

  def __init__(self):
    self.families: dict[tuple[int, ...], _BoschMatureGroupCertificate] = {}
    self.suspended: dict[tuple[int, ...], _BoschMatureGroupCertificate] = {}
    self.last_raw_ages: dict[int, int] = {}
    self.now_ns: int | None = None
    self.stats = Counter()
    self.events: list[dict] = []
    self.peak = self.archive_peak = 0
    # Scan context for the surface guard, supplied by the group manager/provider.
    self.context = (None, None, ())
    self.path: tuple = ()
    self.path_ns: int | None = None

  def ingest_path(self, path, timestamp_ns: int) -> None:
    """Latest scan-gated model path; held for at most GUARD_PATH_HOLD_NS."""
    if len(path) >= 2:
      self.path = tuple(path)
      self.path_ns = timestamp_ns

  def _surface_guard(self, manager, family, timestamp_ns, raw_tracks, raw_index, live):
    prior = None
    for state in manager.states.values():
      if tuple(sorted(m.raw_track_id for m in state.observation.members)) == family:
        prior = state.observation
        break
    if prior is None:
      return 'guard_prior_missing_veto'
    yaw_rate, oem_slot, vision = self.context
    cues = [(cue.d_rel, cue.y_rel, cue.distance_tolerance_m, cue.lateral_tolerance_m) for cue in vision
            if math.isfinite(cue.d_rel) and math.isfinite(cue.y_rel) and cue.probability >= .7]
    vision_supported = [any(abs(r.detection.d_rel-cd) <= dt and abs(r.detection.y_rel-cy) <= lt
                            for cd, cy, dt, lt in cues) for r in raw_tracks]
    # The same representative the group update will choose for this exact family.
    dt = (timestamp_ns-prior.timestamp_ns)/1e9
    angle = -(yaw_rate or 0.)*dt
    dx = prior.d_rel + prior.v_rel*dt
    ca, sa = math.cos(angle), math.sin(angle)
    candidates = tuple(live[rid] for rid in family)
    rep = _bosch_group_representative(candidates, raw_index, vision_supported, oem_slot, prior=prior,
                                      predicted_xy=(dx*ca-prior.y_rel*sa, dx*sa+prior.y_rel*ca))
    # Published surface: the anchor, unless the final OEM-nearer stage moves it to a nearer word1 member.
    surface = rep
    if BOSCH_OEM_NEARER_PUBLICATION_MODE == BOSCH_OEM_NEARER_PUBLICATION_ACTIVE and oem_slot is not None:
      owned = next((m for m in candidates if m.slot == oem_slot), None)
      if owned is not None and owned.d_rel < rep.d_rel:
        surface = owned
    if surface.d_rel - min(m.d_rel for m in candidates) > self.GUARD_NEAR_TOL_M:
      return 'guard_surface_not_nearest_veto'
    if len(self.path) < 2 or self.path_ns is None or not 0 <= timestamp_ns - self.path_ns <= self.GUARD_PATH_HOLD_NS:
      return 'guard_path_missing_veto'
    offset = _BoschFamilyCompanionFilter._path_offset
    surface_path = abs(offset(surface, self.path))
    for member in candidates:
      if member is surface:
        continue
      if surface.v_rel > member.v_rel + self.GUARD_COVER_SPEED_MPS:
        return 'guard_cover_speed_veto'
      if surface_path > max(abs(offset(member, self.path)), self.GUARD_COVER_PATH_M):
        return 'guard_cover_lateral_veto'
    return None

  @staticmethod
  def _pairs(family: tuple[int, ...]) -> tuple[tuple[int, int], ...]:
    return tuple((a, b) for index, a in enumerate(family) for b in family[index+1:])

  def _veto(self, certificate: _BoschMatureGroupCertificate, pairs: tuple[tuple[int, int], ...],
            offsets: dict[tuple[int, int], tuple[float, float]], live: dict[int, BoschRawTrack],
            config: BoschGroupingConfig, v_ego: float) -> str | None:
    for a, b in pairs:
      ra, rb = live[a], live[b]
      dd, dy = map(abs, offsets[(a, b)])
      speeds = (abs(ra.v_rel+v_ego), abs(rb.v_rel+v_ego))
      if dd > config.distance_diameter_m+.25 or dy > config.lateral_diameter_m+.03125:
        return 'extent_veto'
      if (abs(ra.v_rel-rb.v_rel) > .5 or
          (math.isfinite(v_ego) and min(speeds) <= config.stationary_speed_mps and
           max(speeds) >= config.moving_speed_mps)):
        return 'motion_veto'
      if certificate.history:
        historical = [entry[1][(a, b)] for entry in certificate.history]
        recent = [entry[1][(a, b)] for entry in certificate.history
                  if entry[0] >= certificate.last_strict_ns-round(config.evidence_window_s*1e9)]
        component_unstable = (
          dd-min(abs(p[0]) for p in historical) > config.max_relative_distance_growth_m+.25 or
          dy-min(abs(p[1]) for p in historical) > config.max_relative_lateral_growth_m+.1875)
        if not recent:
          return 'separation_reference_missing_veto'
        if component_unstable and math.hypot(dd, dy) > max(math.hypot(*p) for p in recent)+.25:
          return 'shape_growth_veto'
    return None

  def before(self, manager: 'BoschObjectGroupManager', timestamp_ns: int,
             raw_tracks: tuple[BoschRawTrack, ...], compatible: list[int],
             pair_cost: list[float], raw_index: dict[int, int], v_ego: float) -> None:
    config = manager.config
    live = {raw.raw_track_id: raw for raw in raw_tracks}
    raw_ids = set(live)
    previous_groups = [{member.raw_track_id for member in state.observation.members}
                       for state in manager.states.values()]
    self.events = []
    if self.now_ns is not None and timestamp_ns-self.now_ns > round(config.pair_max_gap_s*1e9):
      self.suspended.clear()
      self.last_raw_ages.clear()
      self.families.clear()
      self.stats['gap_resets'] += 1
    self.now_ns = timestamp_ns
    broken = {rid for rid, raw in live.items()
              if rid in self.last_raw_ages and raw.age_scans <= self.last_raw_ages[rid]}
    for family in tuple(self.suspended):
      certificate = self.suspended[family]
      if (not set(family).issubset(raw_ids) or broken.intersection(family) or
          timestamp_ns-certificate.last_strict_ns > self.MEMORY_NS or
          any(group.intersection(family) and not group.issubset(family) for group in previous_groups)):
        del self.suspended[family]
        self.stats['suspended_certificate_pruned'] += 1
    previous = {tuple(sorted(group)) for group in previous_groups if len(group) >= 2}
    for family in previous:
      if family not in self.families and family in self.suspended:
        self.families[family] = self.suspended[family].clone()
        self.stats['natural_rejoin_certificate_reused'] += 1
    self.families = {family: cert for family, cert in self.families.items() if family in previous}
    n = len(raw_tracks)
    for family in sorted(previous):
      if not all(rid in live for rid in family):
        self.families.pop(family, None)
        self.stats['missing_member_veto'] += 1
        continue
      indices = tuple(raw_index[rid] for rid in family)
      pairs = self._pairs(family)
      strict = all(compatible[raw_index[a]] & (1 << raw_index[b]) for a, b in pairs)
      certificate = self.families.get(family)
      ages = tuple(live[rid].age_scans for rid in family)
      if certificate and any(age <= old for age, old in zip(ages, certificate.ages, strict=False)):
        certificate = None
        self.families.pop(family, None)
        self.stats['age_rollback_veto'] += 1
      offsets = {pair: (live[pair[0]].d_rel-live[pair[1]].d_rel,
                        live[pair[0]].y_rel-live[pair[1]].y_rel) for pair in pairs}
      if strict:
        if certificate is None:
          certificate = _BoschMatureGroupCertificate(timestamp_ns, timestamp_ns, ages)
          self.families[family] = certificate
        certificate.last_strict_ns = timestamp_ns
        certificate.ready = certificate.ready or timestamp_ns-certificate.start_ns >= self.MATURITY_NS
        certificate.ages = ages
        certificate.history.append((timestamp_ns, offsets))
        certificate.history = certificate.history[-self.HISTORY_MAX:]
        continue
      if not certificate or not certificate.ready:
        self.stats['maturity_veto'] += 1
        self.families.pop(family, None)
        continue
      certificate.ages = ages
      elapsed = timestamp_ns-certificate.last_strict_ns
      if elapsed > self.LEASE_NS:
        self.stats['lease_expiry_veto'] += 1
        self.families.pop(family, None)
        continue
      reason = self._veto(certificate, pairs, offsets, live, config, v_ego)
      if reason is None:
        for outsider in raw_tracks:
          if outsider.raw_track_id in family:
            continue
          outsider_index = raw_index[outsider.raw_track_id]
          if any(compatible[index] & (1 << outsider_index) for index in indices):
            reason = 'strict_competitor_veto'
            break
          if any(abs(outsider.d_rel-live[rid].d_rel) <= 5.0 and
                 abs(outsider.y_rel-live[rid].y_rel) <= 2.0 and
                 abs(outsider.v_rel-live[rid].v_rel) <= 1.0 for rid in family):
            reason = 'near_competitor_veto'
            break
      if reason is None:
        reason = self._surface_guard(manager, family, timestamp_ns, raw_tracks, raw_index, live)
      if reason:
        self.stats[reason] += 1
        self.families.pop(family, None)
        continue
      added = []
      for a, b in pairs:
        i, j = raw_index[a], raw_index[b]
        if not compatible[i] & (1 << j):
          compatible[i] |= 1 << j
          compatible[j] |= 1 << i
          pair_cost[i*n+j] = pair_cost[j*n+i] = (
            abs(live[a].d_rel-live[b].d_rel)/config.distance_diameter_m +
            abs(live[a].y_rel-live[b].y_rel)/config.lateral_diameter_m +
            abs(live[a].v_rel-live[b].v_rel)/config.velocity_diameter_mps)
          added.append((a, b))
      self.events.append({'ns': timestamp_ns, 'family': family, 'added': added, 'lease_ms': elapsed/1e6})
      self.stats['retained_group_scans'] += 1
      self.stats['retained_edges'] += len(added)
    self.peak = max(self.peak, len(self.families))
    if len(self.families) > self.LIVE_MAX:
      raise AssertionError('nonexclusive family certificate')
    for family, certificate in self.families.items():
      if certificate.ready:
        self.suspended[family] = certificate.clone()
    if len(self.suspended) > self.ARCHIVE_MAX:
      self.suspended = dict(sorted(self.suspended.items(),
                                   key=lambda item: (-item[1].last_strict_ns, item[0]))[:self.ARCHIVE_MAX])
    self.last_raw_ages = {rid: raw.age_scans for rid, raw in live.items()}
    self.archive_peak = max(self.archive_peak, len(self.suspended))
    self.peak = max(self.peak, len(self.families)+len(self.suspended))


class BoschObjectGroupManager:
  """Temporal complete-link grouping followed by global physical-ID assignment.

  Formation requires strict geometry, motion-band and temporal pair evidence.
  An opt-in mature exact-family policy can retain bounded morphology edges;
  complete-link still applies. There is no transitive single-link merge.
  Existing raw membership and representative continuity preserve the physical
  ID. OEM selection never establishes identity or creates an extra point.
  """
  def __init__(self, config: BoschGroupingConfig | None = None):
    self.config = config or BoschGroupingConfig()
    self.next_id = self.config.first_physical_id
    self.now_ns = None
    self.pairs: dict[tuple[int, int], _BoschPairEvidence] = {}
    self.states: dict[int, _BoschPhysicalState] = {}
    self.stats = Counter()
    self.last_pair_possible = self.last_pair_candidates = 0
    self.last_conflicts = self.last_direct_carries = self.last_multi_count = 0
    self.provisional_bundles: dict[tuple[int, int], _BoschProvisionalBundleState] = {}
    self.mature_retention = _BoschMatureGroupRetention() if self.config.mature_retention_enabled else None
    self.provisional_hidden: dict[int, int] = {}
    self.provisional_pid_pairs: dict[tuple[int, int], tuple[int, int]] = {}
    self.last_provisional_decisions: tuple[BoschProvisionalBundleDecision, ...] = ()
    self.provisional_created = self.provisional_coherent = 0
    self.provisional_handoffs = self.provisional_releases = 0
    # Bounded Bosch-only lineage used by the final CAMERA_COMPANION publication
    # gate. raw_track_id is provider association state, not a native OBJECT_ID.
    self.common_group_ancestry: dict[tuple[int, int], _BoschCommonGroupAncestry] = {}
    self.common_representative_owners: dict[int, dict[int, int]] = {}
    # Shadow diagnostic switch. Off leaves the scan path byte-for-byte as it
    # was; on only fills last_decisions. Neither setting gates an assignment.
    self.trace_decisions = False
    self.last_decisions: tuple[BoschAssociationDecision, ...] = ()

  @property
  def last_diagnostics(self):
    # Research/debug consumers can request the old representation. Production
    # needs neither per-pair dictionaries nor rejection strings each scan.
    c = self.config
    result = []
    for key, evidence in sorted(self.pairs.items()):
      samples = evidence.samples
      if samples[-1][0] != self.now_ns:
        continue
      stable = (abs(samples[-1][1])-min(abs(s[1]) for s in samples) <= c.max_relative_distance_growth_m and
                abs(samples[-1][2])-min(abs(s[2]) for s in samples) <= c.max_relative_lateral_growth_m)
      mature = (len(samples) >= c.min_pair_observations and
                (self.now_ns-samples[0][0])/1e9 + 1e-6 >= c.min_pair_span_s)
      reason = 'relative_motion_drift' if not stable else ('geometry' if mature else 'insufficient_temporal_evidence')
      result.append(dict(a=key[0], b=key[1], compatible=stable and mature, reason=reason))
    return result

  def _geometry(self, a, b, v_ego):
    c = self.config
    if abs(a.d_rel - b.d_rel) > c.distance_diameter_m:
      return False, 'distance_diameter'
    if abs(a.y_rel - b.y_rel) > c.lateral_diameter_m:
      return False, 'lateral_diameter'
    if abs(a.v_rel - b.v_rel) > c.velocity_diameter_mps:
      return False, 'velocity_diameter'
    if math.isfinite(v_ego):
      speeds = sorted((abs(a.v_rel + v_ego), abs(b.v_rel + v_ego)))
      if speeds[0] <= c.stationary_speed_mps and speeds[1] >= c.moving_speed_mps:
        return False, 'stationary_moving_conflict'
    return True, 'geometry'

  @staticmethod
  def _vision_support(member, vision):
    return any(math.isfinite(c.d_rel) and math.isfinite(c.y_rel) and c.probability >= .7
         and abs(member.d_rel - c.d_rel) <= c.distance_tolerance_m
         and abs(member.y_rel - c.y_rel) <= c.lateral_tolerance_m for c in vision)

  def reset_common_ancestry(self):
    self.common_group_ancestry = {}
    self.common_representative_owners = {}

  def _update_common_ancestry(self, timestamp_ns, objects):
    """Keep only live, recent raw/group and representative ownership history."""
    live_pids = self.states.keys()
    active_raw = {raw_id for state in self.states.values() for raw_id in state.member_last_seen}
    max_age = BOSCH_COMMON_ANCESTRY_MAX_AGE_NS
    ancestry = self.common_group_ancestry
    for raw_pair in tuple(ancestry):
      evidence = ancestry[raw_pair]
      if (not 0 <= timestamp_ns - evidence.timestamp_ns <= max_age or
          raw_pair[0] not in active_raw or raw_pair[1] not in active_raw):
        del ancestry[raw_pair]
    owners_by_raw = self.common_representative_owners
    for raw_id in tuple(owners_by_raw):
      if raw_id not in active_raw:
        del owners_by_raw[raw_id]
        continue
      owners = owners_by_raw[raw_id]
      for pid in tuple(owners):
        ns = owners[pid]
        if pid not in live_pids or not 0 <= timestamp_ns - ns <= max_age:
          del owners[pid]
      if not owners:
        del owners_by_raw[raw_id]

    for obj in objects:
      members = obj.members
      for index, member_a in enumerate(members):
        raw_a = member_a.raw_track_id
        for other_index in range(index + 1, len(members)):
          raw_b = members[other_index].raw_track_id
          key = (raw_a, raw_b) if raw_a < raw_b else (raw_b, raw_a)
          self.common_group_ancestry.setdefault(
            key, _BoschCommonGroupAncestry(timestamp_ns, obj.physical_track_id))
      owners = self.common_representative_owners.setdefault(
        obj.representative_raw_track_id, {})
      owners[obj.physical_track_id] = timestamp_ns

    if len(self.common_group_ancestry) > BOSCH_COMMON_ANCESTRY_RAW_PAIR_MAX:
      keep = sorted(self.common_group_ancestry.items(),
                    key=lambda item: (-item[1].timestamp_ns, item[0]))[
                      :BOSCH_COMMON_ANCESTRY_RAW_PAIR_MAX]
      self.common_group_ancestry = dict(keep)

  def common_ancestry_evidence(self, first, second, shared_start_ns, timestamp_ns):
    """Return prior group split and representative handoff for this PID pair."""
    first_raw = {member.raw_track_id for member in first.members}
    second_raw = {member.raw_track_id for member in second.members}
    split = any(
      (evidence := self.common_group_ancestry.get(tuple(sorted((raw_a, raw_b))))) is not None and
      evidence.timestamp_ns < shared_start_ns and
      0 <= timestamp_ns - evidence.timestamp_ns <= BOSCH_COMMON_ANCESTRY_MAX_AGE_NS
      for raw_a in first_raw for raw_b in second_raw if raw_a != raw_b)
    pair = {first.physical_track_id, second.physical_track_id}
    handoff = any(
      pair.issubset(owners) and
      all(0 <= timestamp_ns - owners[pid] <= BOSCH_COMMON_ANCESTRY_MAX_AGE_NS for pid in pair)
      for owners in self.common_representative_owners.values())
    return split, handoff

  def update(self, timestamp_ns: int, raw_tracks: Sequence[BoschRawTrack], *,
       yaw_rate: float | None = None, v_ego: float = math.nan,
       oem_slot: int | None = None, vision: Sequence[BoschVisionCue] = ()) -> tuple[BoschPhysicalObject, ...]:
    if isinstance(timestamp_ns, bool) or not isinstance(timestamp_ns, int) or timestamp_ns < 0:
      raise ValueError('timestamp must be nonnegative integer nanoseconds')
    if self.now_ns is not None and timestamp_ns <= self.now_ns:
      raise ValueError('group updates require strictly increasing scan timestamps')
    if yaw_rate is not None and (isinstance(yaw_rate, bool) or not math.isfinite(yaw_rate)):
      raise ValueError('yaw_rate must be finite or None')
    if isinstance(v_ego, bool) or not isinstance(v_ego, (int, float)) or math.isinf(v_ego):
      raise ValueError('v_ego must be finite or NaN unknown')
    raw_tracks = tuple(raw_tracks)
    if len(raw_tracks) > 32 or not all(isinstance(r, BoschRawTrack) for r in raw_tracks):
      raise ValueError('at most 32 fresh RawTrack observations are required')
    if len({r.slot for r in raw_tracks}) != len(raw_tracks):
      raise ValueError('duplicate raw slot')
    for r in raw_tracks:
      if (r.timestamp_ns > timestamp_ns or
        timestamp_ns-r.timestamp_ns > round(self.config.coast_s*1e9) or
        (self.now_ns is not None and r.timestamp_ns <= self.now_ns)):
        raise ValueError('stale, repeated or future raw observation')
    if len({r.raw_track_id for r in raw_tracks}) != len(raw_tracks):
      raise ValueError('duplicate raw track ID')
    # ID exhaustion is exceptional. Stage that rare update separately so an
    # overflow cannot partially alter history, ownership, timestamps or stats.
    if self.next_id+len(raw_tracks) > 2**31:
      staged = copy.deepcopy(self)
      result = staged._update(timestamp_ns, raw_tracks, yaw_rate=yaw_rate, v_ego=v_ego,
                  oem_slot=oem_slot, vision=vision)
      self.__dict__.update(staged.__dict__)
      return result
    return self._update(timestamp_ns, raw_tracks, yaw_rate=yaw_rate, v_ego=v_ego,
              oem_slot=oem_slot, vision=vision)

  def _update(self, timestamp_ns, raw_tracks, *, yaw_rate=None, v_ego=math.nan, oem_slot=None, vision=()):
    raw_tracks = tuple(sorted(raw_tracks, key=lambda r: r.raw_track_id))
    self.now_ns = timestamp_ns
    c = self.config
    coast_ns = round(c.coast_s * 1e9)
    for pid in list(self.states):
      state = self.states[pid]
      if timestamp_ns - state.observation.timestamp_ns > coast_ns:
        del self.states[pid]
        self.stats['deleted'] += 1
      else:
        # Drop in place: rebuilding every surviving member map each scan was
        # pure churn, since almost no scan expires a member.
        last_seen = state.member_last_seen
        expired = [rid for rid, ns in last_seen.items() if timestamp_ns - ns > coast_ns]
        for rid in expired:
          del last_seen[rid]

    n = len(raw_tracks)
    ids = [r.raw_track_id for r in raw_tracks]
    distances = [r.d_rel for r in raw_tracks]
    lateral = [r.y_rel for r in raw_tracks]
    velocities = [r.v_rel for r in raw_tracks]
    raw_index = {rid: i for i, rid in enumerate(ids)}
    for pair in list(self.pairs):
      i, j = raw_index.get(pair[0]), raw_index.get(pair[1])
      # The old all-pairs loop cleared evidence immediately when two observed
      # members failed distance. Preserve this even outside the new window.
      if (timestamp_ns - self.pairs[pair].samples[-1][0] > c.evidence_window_s * 1e9 or
          (i is not None and j is not None and abs(distances[i]-distances[j]) > c.distance_diameter_m)):
        del self.pairs[pair]

    compatible = [0] * n
    pair_cost = [0.] * (n*n)
    distance_order = sorted(range(n), key=distances.__getitem__)
    ground_speeds = [abs(v+v_ego) for v in velocities] if math.isfinite(v_ego) else None
    pair_candidates = lateral_rejections = velocity_rejections = motion_rejections = 0
    provisional_matches = {}
    provisional_handoffs = set()
    for position, a in enumerate(distance_order):
      for following in range(position+1, n):
        b = distance_order[following]
        if distances[b]-distances[a] > c.distance_diameter_m:
          break
        pair_candidates += 1
        i, j = (a, b) if a < b else (b, a)
        key = (ids[i], ids[j])
        delta_d, delta_y = distances[i]-distances[j], lateral[i]-lateral[j]
        delta_v = abs(velocities[i]-velocities[j])
        if c.provisional_enabled and math.isfinite(v_ego):
          prior_bundle = self.provisional_bundles.get(key)
          fresh_pair = raw_tracks[i].age_scans == raw_tracks[j].age_scans == 1
          if fresh_pair or prior_bundle is not None:
            yaw = yaw_rate or 0.
            world_i = velocities[i] + v_ego - yaw*lateral[i]
            world_j = velocities[j] + v_ego - yaw*lateral[j]
            bearing_i = math.degrees(math.atan2(lateral[i], max(distances[i], .5)))
            bearing_j = math.degrees(math.atan2(lateral[j], max(distances[j], .5)))
            metrics = (abs(delta_d), abs(delta_y), abs(bearing_i-bearing_j), delta_v,
                       abs(world_i-world_j))
            gap_ok = (prior_bundle is None or
                      timestamp_ns-prior_bundle.timestamp_ns <= c.pair_max_gap_s*1e9)
            ages_ok = (prior_bundle is None or
                       (raw_tracks[i].age_scans > prior_bundle.last_ages[0] and
                        raw_tracks[j].age_scans > prior_bundle.last_ages[1]))
            supported = oem_slot is not None and oem_slot in (raw_tracks[i].slot, raw_tracks[j].slot)
            if (gap_ok and ages_ok and not supported and
                c.provisional_min_distance_m <= metrics[0] <= c.provisional_max_distance_m and
                metrics[1] <= c.provisional_max_lateral_m and
                metrics[2] <= c.provisional_max_bearing_deg and
                metrics[3] <= c.provisional_max_velocity_mps and
                metrics[4] <= c.provisional_max_world_speed_mps and
                min(distances[i], distances[j]) >= c.provisional_min_range_m and
                min(abs(world_i), abs(world_j)) >= c.provisional_min_world_speed_mps and
                abs(raw_tracks[i].slot-raw_tracks[j].slot) == 1):
              provisional_matches[key] = (i, j, metrics)
        if abs(delta_y) > c.lateral_diameter_m:
          lateral_rejections += 1
          self.pairs.pop(key, None)
          continue
        if delta_v > c.velocity_diameter_mps:
          velocity_rejections += 1
          self.pairs.pop(key, None)
          continue
        if (ground_speeds is not None and min(ground_speeds[i], ground_speeds[j]) <= c.stationary_speed_mps
            and max(ground_speeds[i], ground_speeds[j]) >= c.moving_speed_mps):
          motion_rejections += 1
          self.pairs.pop(key, None)
          continue
        evidence = self.pairs.get(key)
        if evidence is None:
          evidence = self.pairs[key] = _BoschPairEvidence()
        samples = evidence.samples
        if samples and timestamp_ns - samples[-1][0] > c.pair_max_gap_s * 1e9:
          samples.clear()
        # Only the magnitudes are ever read back, and abs() is idempotent, so
        # store them once here instead of re-taking them over the whole window.
        abs_delta_d, abs_delta_y = abs(delta_d), abs(delta_y)
        samples.append((timestamp_ns, abs_delta_d, abs_delta_y))
        while timestamp_ns - samples[0][0] > c.evidence_window_s * 1e9:
          samples.popleft()
        # Widening from a recent minimum, not old larger converging separation.
        stable = (abs_delta_d-min(s[1] for s in samples) <= c.max_relative_distance_growth_m and
                  abs_delta_y-min(s[2] for s in samples) <= c.max_relative_lateral_growth_m)
        mature = (len(samples) >= c.min_pair_observations and
                  (timestamp_ns-samples[0][0])/1e9 + 1e-6 >= c.min_pair_span_s)
        if stable and mature:
          if key in self.provisional_bundles:
            provisional_matches.pop(key, None)
            provisional_handoffs.add(key)
          compatible[i] |= 1 << j
          compatible[j] |= 1 << i
          cost = abs_delta_d/c.distance_diameter_m + abs_delta_y/c.lateral_diameter_m + delta_v/c.velocity_diameter_mps
          pair_cost[i*n+j] = pair_cost[j*n+i] = cost
    self.last_pair_possible = n*(n-1)//2
    self.last_pair_candidates = pair_candidates
    self.stats['pair_rejected_distance_diameter'] += self.last_pair_possible-pair_candidates
    self.stats['pair_rejected_lateral_diameter'] += lateral_rejections
    self.stats['pair_rejected_velocity_diameter'] += velocity_rejections
    self.stats['pair_rejected_stationary_moving_conflict'] += motion_rejections

    if self.mature_retention is not None:
      self.mature_retention.context = (yaw_rate, oem_slot, vision)
      self.mature_retention.before(self, timestamp_ns, raw_tracks, compatible, pair_cost, raw_index, v_ego)
    owner = {rid: pid for pid, state in self.states.items() for rid in state.member_last_seen}
    previous = sorted(self.states)
    owner_bits = {pid: 1 << i for i, pid in enumerate(previous)}
    clusters = [[i] for i in range(n)]
    cluster_masks = [1 << i for i in range(n)]
    cluster_compatible = [compatible[i] | cluster_masks[i] for i in range(n)]
    cluster_owners = [owner_bits.get(owner.get(rid), 0) for rid in ids]
    member_cluster = list(range(n))
    max_members = c.max_members
    while True:
      best = None
      for i, a in enumerate(clusters):
        reach = cluster_compatible[i] & ~cluster_masks[i]
        if not reach:
          continue  # No compatible neighbor: singleton/group cannot merge.
        # Every cluster that could merge with i has all of its members here, so
        # walking these bits reaches exactly the candidates the full index scan
        # did. The minimum below compares whole (owner, cost, i, j) tuples, so
        # the order they are visited in cannot change which pair wins.
        limit = max_members-len(a)
        seen = 0
        while reach:
          bit = reach & -reach
          reach ^= bit
          j = member_cluster[bit.bit_length()-1]
          mark = 1 << j
          if j <= i or seen & mark:
            continue
          seen |= mark
          b = clusters[j]
          if len(b) > limit:
            continue
          if cluster_compatible[i] & cluster_masks[j] != cluster_masks[j]:
            continue
          # Same complete-link cost and original cluster-index tie order.
          cost = max(pair_cost[x*n+y] for x in a for y in b)
          choice = (not (cluster_owners[i] & cluster_owners[j]), cost, i, j)
          if best is None or choice < best:
            best = choice
      if best is None:
        break
      _, _, i, j = best
      clusters[i] += clusters.pop(j)
      cluster_masks[i] |= cluster_masks.pop(j)
      cluster_compatible[i] &= cluster_compatible.pop(j)
      cluster_owners[i] |= cluster_owners.pop(j)
      for index in range(j, len(clusters)):
        for member in clusters[index]:
          member_cluster[member] = index
      for member in clusters[i]:
        member_cluster[member] = i

    assigned = {}
    carry_mode = None
    solver_scores = None
    self.last_conflicts = self.last_direct_carries = 0
    if previous and clusters:
      # In a conflict-free graph every positive overlap edge must be selected;
      # zero-score dummy assignments cannot affect an output identity. Keep
      # the original whole solver on any merge/split, including its tie order.
      claims = {}
      ambiguous = set()
      for i, mask in enumerate(cluster_owners):
        if not mask:
          continue
        if mask & (mask-1):
          ambiguous.add(i)
        remaining = mask
        while remaining:
          bit = remaining & -remaining
          if bit in claims:
            ambiguous.update((i, claims[bit]))
          else:
            claims[bit] = i
          remaining ^= bit
      self.last_conflicts = len(ambiguous)
      if not ambiguous:
        assigned = {i: previous[mask.bit_length()-1] for i, mask in enumerate(cluster_owners) if mask}
        self.last_direct_carries = len(assigned)
        carry_mode = 'direct_carry'
      else:
        carry_mode = 'assignment_solver'
        # Invert membership once instead of intersecting every cluster against
        # every previous state; only the states a cluster actually touches can
        # score, and the score itself is unchanged.
        holders = {}
        for j, pid in enumerate(previous):
          for rid in self.states[pid].member_last_seen:
            entry = holders.get(rid)
            if entry is None:
              holders[rid] = [j]
            else:
              entry.append(j)
        # Fewer than five in a hundred of the matrix cells the old solver walked
        # could ever score. Collect the edges instead and let the decomposition
        # below reach the same matching without materialising the matrix.
        row_edges, column_edges = [], {}
        for i, cluster in enumerate(clusters):
          members = {ids[k] for k in cluster}
          overlaps = {}
          for rid in members:
            for j in holders.get(rid, ()):
              overlaps[j] = overlaps.get(j, 0) + 1
          edges = []
          for j, overlap in overlaps.items():
            pid = previous[j]
            observation = self.states[pid].observation
            score = (10*overlap +
                    3*(observation.representative_raw_track_id in members) +
                    min(observation.age_scans, 1000)*1e-5 + 1/(pid+1))
            edges.append((j, score))
            column = column_edges.get(j)
            if column is None:
              column_edges[j] = [(i, score)]
            else:
              column.append((i, score))
          row_edges.append(edges)
        # The shadow trace reports the whole score matrix, so a trace run keeps
        # the original path; production never needs the matrix at all.
        matched = None if self.trace_decisions else _bosch_physical_assignment(row_edges, column_edges)
        if matched is None:
          scores = np.zeros((len(clusters), len(previous)+len(clusters)))
          for i, edges in enumerate(row_edges):
            for j, score in edges:
              scores[i, j] = score
          ri, ci = bosch_linear_sum_assignment(-scores)
          assigned = {int(i): previous[int(j)] for i, j in zip(ri, ci) if j < len(previous) and scores[i, j] > 0}
          if self.trace_decisions:
            solver_scores = scores
        else:
          assigned = {i: previous[j] for i, j in matched.items()}

    result = []
    decisions = []
    current_ids = set(ids)
    # Whether a cue is usable at all depends on the cue alone. Test that once
    # per scan instead of once per return; the two tolerance comparisons that
    # actually decide support are untouched, and an unusable cue could never
    # reach them anyway.
    cues = [(cue.d_rel, cue.y_rel, cue.distance_tolerance_m, cue.lateral_tolerance_m) for cue in vision
            if math.isfinite(cue.d_rel) and math.isfinite(cue.y_rel) and cue.probability >= .7]
    if cues:
      vision_supported = []
      for track in raw_tracks:
        point = track.detection
        d_rel, y_rel = point.d_rel, point.y_rel
        vision_supported.append(any(abs(d_rel-cd) <= dt and abs(y_rel-cy) <= lt
                                    for cd, cy, dt, lt in cues))
    else:
      vision_supported = [False] * n
    # Nearly nine in ten clusters hold one return. Split that case out and hoist
    # the loop-invariant lookups; the emitted object is byte-for-byte the same.
    stats = self.stats
    states = self.states
    trace = self.trace_decisions
    v_ego_known = math.isfinite(v_ego)
    stationary_speed = c.stationary_speed_mps
    largest_group = stats['max_group_size']
    for i, cluster in enumerate(clusters):
      pid = assigned.get(i)
      old = states.get(pid)
      prior = old.observation if old else None
      if pid is None:
        if self.next_id >= 2**31:
          raise OverflowError('physical Int32 ID space exhausted; no reuse')
        pid, self.next_id = self.next_id, self.next_id+1
        stats['created'] += 1
      size = len(cluster)
      # Build the member sequence as the tuple the emitted object keeps, so the
      # object below stores it without a second copy.
      if size == 1:
        candidates = (raw_tracks[cluster[0]],)
        member_ids = {candidates[0].raw_track_id}
      else:
        candidates = tuple(raw_tracks[k] for k in sorted(cluster))
        member_ids = {m.raw_track_id for m in candidates}
      predicted = None
      # A single-member cluster picks its only member, so production skips the
      # projection there. The trace still needs it: that is exactly the case in
      # which no continuity term reaches the published state at all.
      if prior and (size > 1 or trace):
        dt = (timestamp_ns-prior.timestamp_ns)/1e9
        angle = -(yaw_rate or 0.)*dt
        dx = prior.d_rel + prior.v_rel*dt
        ca, sa = math.cos(angle), math.sin(angle)
        px = dx*ca-prior.y_rel*sa
        py = dx*sa+prior.y_rel*ca
        predicted = (dt, px, py)
      if size == 1:
        rep = candidates[0]
        oem_selected = rep.detection.slot == oem_slot
        supported = vision_supported[cluster[0]]
        evidence = 'single_return'
      else:
        rep = _bosch_group_representative(
          candidates, raw_index, vision_supported, oem_slot, prior=prior,
          predicted_xy=(px, py) if prior else None)
        oem_selected = any(m.slot == oem_slot for m in candidates)
        supported = any(vision_supported[k] for k in cluster)
        evidence = 'temporal_complete_link'
      point = rep.detection
      v_rel = point.v_rel
      obj = BoschPhysicalObject(pid, timestamp_ns, candidates, rep.raw_track_id,
                point.d_rel, point.y_rel, v_rel, oem_selected, supported,
                (prior.age_scans+1 if prior else 1), evidence)
      if trace:
        decisions.append(self._decision(obj, old, prior, carry_mode, predicted,
                                        solver_scores, i, previous))
      # A raw member currently assigned elsewhere cannot belong to this ID.
      # Dropping those in place leaves the surviving entries in the order the
      # rebuilt map had, and this state is the one states[pid] already holds.
      if old is None:
        last_seen = {}
        states[pid] = _BoschPhysicalState(obj, last_seen)
      else:
        last_seen = old.member_last_seen
        for rid in [rid for rid in last_seen if rid in current_ids and rid not in member_ids]:
          del last_seen[rid]
        old.observation = obj
      for m in candidates:
        last_seen[m.raw_track_id] = timestamp_ns
      if prior and prior.representative_raw_track_id != rep.raw_track_id:
        stats['representative_changes'] += 1
      if size == 1:
        # A lone member has at most one previous owner, so it can never record
        # a membership merge.
        recovered_all = old is not None and rep.recovered
      else:
        old_owners = set()
        recovered_all = old is not None
        for m in candidates:
          holder = owner.get(m.raw_track_id)
          if holder is not None:
            old_owners.add(holder)
          if not m.recovered:
            recovered_all = False
        if len(old_owners) > 1:
          stats['membership_merge_events'] += len(old_owners)-1
        stats['multi_member_groups'] += 1
      if recovered_all:
        stats['coasting_recoveries'] += 1
      if supported:
        stats['vision_supported_groups'] += 1
      if v_ego_known and abs(v_rel+v_ego) <= stationary_speed:
        stats['stationary_groups'] += 1
      if size > largest_group:
        largest_group = size
      result.append(obj)
    if largest_group != stats['max_group_size']:
      stats['max_group_size'] = largest_group

    # Provisional bundles never alter clusters, physical IDs, raw histories or
    # alias ownership.  They only describe which one of two independently
    # tracked singleton objects the final publication view should expose while
    # normal temporal pair evidence matures.
    provisional_decisions = []
    next_bundles = {}
    provisional_hidden = {}
    provisional_pid_pairs = {}
    raw_to_object = {member.raw_track_id: obj for obj in result for member in obj.members}
    candidate_degree = Counter(rid for key in provisional_matches for rid in key)
    ambiguous_raw = {rid for rid, count in candidate_degree.items() if count > 1}
    handled = set()
    for key, prior_bundle in sorted(self.provisional_bundles.items()):
      if key in provisional_matches and not ambiguous_raw.intersection(key):
        continue
      handled.add(key)
      if key in provisional_handoffs:
        reason = 'normal_group_handoff'
        self.provisional_handoffs += 1
      elif ambiguous_raw.intersection(key):
        reason = 'ambiguous_candidate'
        self.provisional_releases += 1
      elif not all(rid in raw_index for rid in key):
        reason = 'member_missing'
        self.provisional_releases += 1
      elif timestamp_ns-prior_bundle.timestamp_ns > c.pair_max_gap_s*1e9:
        reason = 'scan_gap'
        self.provisional_releases += 1
      elif any(raw_tracks[raw_index[rid]].age_scans <= age for rid, age in zip(key, prior_bundle.last_ages)):
        reason = 'age_rollback'
        self.provisional_releases += 1
      elif oem_slot is not None and any(raw_tracks[raw_index[rid]].slot == oem_slot for rid in key):
        reason = 'oem_member_support'
        self.provisional_releases += 1
      else:
        reason = 'criteria_diverged'
        self.provisional_releases += 1
      pids = tuple(sorted({raw_to_object[rid].physical_track_id for rid in key if rid in raw_to_object}))
      provisional_decisions.append(BoschProvisionalBundleDecision(
        timestamp_ns, 'HANDOFF' if reason == 'normal_group_handoff' else 'RELEASE', reason,
        key, pids, prior_bundle.representative_raw_track_id,
        raw_to_object[prior_bundle.representative_raw_track_id].physical_track_id
          if prior_bundle.representative_raw_track_id in raw_to_object else None,
        None, None, None, None, None))

    for key, (i, j, metrics) in sorted(provisional_matches.items()):
      if key in handled:
        continue
      if ambiguous_raw.intersection(key):
        provisional_decisions.append(BoschProvisionalBundleDecision(
          timestamp_ns, 'REJECT', 'ambiguous_candidate', key, (), None, None, *metrics))
        continue
      first, second = raw_to_object.get(key[0]), raw_to_object.get(key[1])
      if first is None or second is None or first is second:
        # A normal mature group owns the pair; publication already has one
        # physical object and no provisional suppression is necessary.
        pids = () if first is None and second is None else tuple(sorted({obj.physical_track_id for obj in (first, second) if obj}))
        provisional_decisions.append(BoschProvisionalBundleDecision(
          timestamp_ns, 'HANDOFF', 'normal_group_handoff', key, pids, None, None, *metrics))
        self.provisional_handoffs += 1
        continue
      if len(first.members) != 1 or len(second.members) != 1:
        provisional_decisions.append(BoschProvisionalBundleDecision(
          timestamp_ns, 'REJECT', 'existing_group_continuity', key,
          tuple(sorted((first.physical_track_id, second.physical_track_id))), None, None, *metrics))
        if key in self.provisional_bundles:
          self.provisional_releases += 1
        continue
      candidates = (raw_tracks[i], raw_tracks[j])
      prior_bundle = self.provisional_bundles.get(key)
      predicted_xy = None
      if prior_bundle is not None:
        dt = (timestamp_ns-prior_bundle.timestamp_ns)/1e9
        angle = -(yaw_rate or 0.)*dt
        dx = prior_bundle.d_rel + prior_bundle.v_rel*dt
        ca, sa = math.cos(angle), math.sin(angle)
        predicted_xy = (dx*ca-prior_bundle.y_rel*sa, dx*sa+prior_bundle.y_rel*ca)
      rep = _bosch_group_representative(
        candidates, raw_index, vision_supported, oem_slot,
        prior=prior_bundle, predicted_xy=predicted_xy)
      rep_obj = raw_to_object[rep.raw_track_id]
      other_obj = second if rep_obj is first else first
      next_bundles[key] = _BoschProvisionalBundleState(
        key, timestamp_ns, (raw_tracks[i].age_scans, raw_tracks[j].age_scans),
        rep.raw_track_id, rep.d_rel, rep.y_rel, rep.v_rel)
      provisional_hidden[other_obj.physical_track_id] = rep_obj.physical_track_id
      provisional_pid_pairs[key] = (rep_obj.physical_track_id, other_obj.physical_track_id)
      action = 'CREATE' if prior_bundle is None else 'COHERENT'
      if prior_bundle is None:
        self.provisional_created += 1
      else:
        self.provisional_coherent += 1
      provisional_decisions.append(BoschProvisionalBundleDecision(
        timestamp_ns, action, '', key,
        tuple(sorted((first.physical_track_id, second.physical_track_id))), rep.raw_track_id,
        rep_obj.physical_track_id, *metrics))

    self.provisional_bundles = next_bundles
    self.provisional_hidden = provisional_hidden
    self.provisional_pid_pairs = provisional_pid_pairs
    self.last_provisional_decisions = tuple(provisional_decisions)

    # Retire absorbed physical IDs immediately, retaining only missing members
    # for genuine coasting. Prevent two IDs from owning the same raw member.
    live_owners = {m.raw_track_id: obj.physical_track_id for obj in result for m in obj.members}
    output_ids = {obj.physical_track_id for obj in result}
    for pid in list(states):
      state = states[pid]
      last_seen = state.member_last_seen
      stolen = [rid for rid in last_seen if live_owners.get(rid, pid) != pid]
      for rid in stolen:
        del last_seen[rid]
      if not last_seen and pid not in output_ids:
        del states[pid]
        stats['absorbed'] += 1
    self._update_common_ancestry(timestamp_ns, result)
    stats['scans'] += 1
    stats['output_objects'] += len(result)
    multi = 0
    for obj in result:
      if len(obj.members) > 1:
        multi += 1
    self.last_multi_count = multi
    self.last_decisions = tuple(decisions)
    return tuple(sorted(result, key=lambda obj: obj.physical_track_id))

  def provisional_publication_view(self, objects, *, strict_associations=None, oem_pids=()):
    """Expose one surface per active bundle, or fail open on identity support."""
    if not self.provisional_hidden or not objects:
      return objects
    strict_associations = strict_associations or {}
    present = {obj.physical_track_id: obj for obj in objects}
    oem_pids = set(oem_pids)
    hidden = dict(self.provisional_hidden)
    decisions = list(self.last_provisional_decisions)
    for raw_key, (representative_pid, hidden_pid) in tuple(self.provisional_pid_pairs.items()):
      pair_pids = (representative_pid, hidden_pid)
      assigned = [(pid, strict_associations[pid]) for pid in pair_pids if pid in strict_associations]
      independent_camera = (len(assigned) == 1 or
                            (len(assigned) == 2 and assigned[0][1] != assigned[1][1]))
      oem_supported = any(pid in oem_pids or (pid in present and present[pid].oem_selected) for pid in pair_pids)
      if not all(pid in present for pid in pair_pids):
        reason = 'publication_member_missing'
      elif independent_camera:
        reason = 'strict_camera_identity'
      elif oem_supported:
        reason = 'oem_identity'
      else:
        continue
      state = self.provisional_bundles.pop(raw_key, None)
      self.provisional_pid_pairs.pop(raw_key, None)
      hidden.pop(hidden_pid, None)
      self.provisional_releases += 1
      decisions.append(BoschProvisionalBundleDecision(
        objects[0].timestamp_ns, 'RELEASE', reason, raw_key, tuple(sorted(pair_pids)),
        state.representative_raw_track_id if state else None, representative_pid,
        None, None, None, None, None))
    self.provisional_hidden = hidden
    self.last_provisional_decisions = tuple(decisions)
    if not hidden:
      return objects
    return tuple(obj for obj in objects if obj.physical_track_id not in hidden)

  def reset_provisional(self, timestamp_ns, reason):
    if not self.provisional_bundles:
      self.provisional_hidden = {}
      self.provisional_pid_pairs = {}
      return
    self.last_provisional_decisions = tuple(
      BoschProvisionalBundleDecision(
        timestamp_ns, 'RELEASE', reason, key, tuple(sorted(self.provisional_pid_pairs.get(key, ()))),
        state.representative_raw_track_id, self.provisional_pid_pairs.get(key, (None,))[0],
        None, None, None, None, None)
      for key, state in sorted(self.provisional_bundles.items()))
    self.provisional_releases += len(self.provisional_bundles)
    self.provisional_bundles = {}
    self.provisional_hidden = {}
    self.provisional_pid_pairs = {}

  def _decision(self, obj, old, prior, carry_mode, predicted, solver_scores, cluster_index, previous):
    """Build one shadow record. Called only while trace_decisions is set."""
    member_ids = {m.raw_track_id for m in obj.members}
    best = second = None
    if solver_scores is not None:
      ranked = sorted((float(v) for v in solver_scores[cluster_index][:len(previous)] if v > 0), reverse=True)
      best = ranked[0] if ranked else None
      second = ranked[1] if len(ranked) > 1 else None
    dt_s = px = py = residual_d = residual_y = residual_v = None
    if predicted is not None:
      dt_s, px, py = predicted
      residual_d, residual_y = obj.d_rel-px, obj.y_rel-py
      residual_v = obj.v_rel-prior.v_rel
    elif prior is not None:
      dt_s = (obj.timestamp_ns-prior.timestamp_ns)/1e9
    return BoschAssociationDecision(
      timestamp_ns=obj.timestamp_ns,
      physical_track_id=obj.physical_track_id,
      previous_physical_track_id=prior.physical_track_id if prior else None,
      assignment_mode='created' if prior is None else carry_mode,
      dt_s=dt_s if dt_s is not None else 0.,
      member_raw_track_ids=tuple(m.raw_track_id for m in obj.members),
      member_slots=obj.member_slots,
      previous_member_raw_track_ids=tuple(m.raw_track_id for m in prior.members) if prior else (),
      member_overlap=len(member_ids.intersection(old.member_last_seen)) if old else 0,
      observed_member_overlap=len(member_ids.intersection(m.raw_track_id for m in prior.members)) if prior else 0,
      representative_raw_track_id=obj.representative_raw_track_id,
      previous_representative_raw_track_id=prior.representative_raw_track_id if prior else None,
      representative_changed=bool(prior and prior.representative_raw_track_id != obj.representative_raw_track_id),
      representative_still_a_member=bool(prior and prior.representative_raw_track_id in member_ids),
      d_rel=obj.d_rel, y_rel=obj.y_rel, v_rel=obj.v_rel,
      predicted_d_rel=px, predicted_y_rel=py,
      predicted_v_rel=prior.v_rel if prior else None,
      residual_d_m=residual_d, residual_y_m=residual_y, residual_v_mps=residual_v,
      best_score=best, second_score=second,
      age_scans=obj.age_scans, grouping_evidence=obj.grouping_evidence)


class BoschPhysicalTracker:
  def __init__(self, raw_config: BoschRawTrackingConfig | None = None, group_config: BoschGroupingConfig | None = None):
    self.raw_manager = BoschRawTrackManager(raw_config)
    self.group_manager = BoschObjectGroupManager(group_config)
    self.last_raw_tracks: tuple[BoschRawTrack, ...] = ()

  def set_decision_trace(self, enabled: bool) -> None:
    """Turn the shadow association traces on both layers on or off.

    Diagnostics only: neither layer reads its trace back, so this never changes
    a raw match, a physical ID, a representative or a published coordinate.
    """
    self.raw_manager.trace_decisions = bool(enabled)
    self.group_manager.trace_decisions = bool(enabled)

  def update(self, timestamp_ns, detections, *, yaw_rate=None, v_ego=math.nan, oem_slot=None, vision=()):
    raw = self.raw_manager.update(timestamp_ns, detections, yaw_rate=yaw_rate)
    self.last_raw_tracks = raw
    result = self.group_manager.update(timestamp_ns, raw, yaw_rate=yaw_rate, v_ego=v_ego,
                                       oem_slot=oem_slot, vision=vision)
    return result


class _BoschPublicationPassThrough:
  """Compatibility hook for publication-stage replay instrumentation."""

  @staticmethod
  def update(objects, *_args, **_kwargs):
    return objects


@dataclass
class _BoschFamilyCompanionState:
  newborn_pid: int
  anchor_pid: int
  newborn_raw_id: int
  anchor_raw_ids: tuple[int, ...]
  anchor_representative_raw_id: int
  first_ns: int
  last_ns: int
  newborn_age: int
  anchor_age: int
  delta_d_m: float
  delta_y_m: float
  delta_bearing_deg: float
  delta_vrel_mps: float
  delta_world_speed_mps: float
  d_path_m: float
  inward_scans: int
  inward_from_abs_path_m: float
  inward_since_ns: int
  active: bool


@dataclass(frozen=True)
class BoschFamilyCompanionDecision:
  timestamp_ns: int
  action: str
  release_reason: str
  newborn_pid: int
  newborn_raw_id: int
  anchor_pid: int
  anchor_raw_ids: tuple[int, ...]
  delta_d_scan1_m: float | None
  delta_d_scan2_m: float | None
  delta_y_scan1_m: float | None
  delta_y_scan2_m: float | None
  delta_vrel_mps: float | None
  delta_world_speed_mps: float | None
  delta_bearing_deg: float | None
  newborn_age: int | None
  anchor_age: int | None
  anchor_member_count: int | None
  camera_coarse: bool
  camera_strict: bool
  oem_identity: bool
  d_path_m: float | None
  public_suppressed: bool


class _BoschFamilyCompanionFilter:
  """S37 B1 two-scan mature-family proof and publication-only hold.

  The fixed bounds are copied from the offline B1 study.  Scan one records a
  singleton birth and a deterministic mature anchor.  Scan two activates the
  hold only when both physical identities, their raw membership, and their
  relative geometry remain continuous.  There is no timeout: the relation is
  kept only while the same evidence continues and is released immediately on
  independent identity, motion divergence, anchor loss, or reset.
  """

  def __init__(self, mode=BOSCH_FAMILY_COMPANION_MODE):
    if mode not in (BOSCH_FAMILY_COMPANION_OFF, BOSCH_FAMILY_COMPANION_SHADOW,
                    BOSCH_FAMILY_COMPANION_ACTIVE):
      raise ValueError('invalid Bosch family companion mode')
    self.mode = mode
    self._states: dict[int, _BoschFamilyCompanionState] = {}
    self.last_ns = None
    self.would_suppress = frozenset()
    self.last_decisions: tuple[BoschFamilyCompanionDecision, ...] = ()
    self.pair_evaluations_last = self.pair_evaluations_total = self.pair_evaluations_peak = 0
    self.holds = self.releases = self.handoffs = self.publication_suppressed = 0
    self.state_peak = 0

  @staticmethod
  def _bearing(obj):
    return math.degrees(math.atan2(obj.y_rel, max(obj.d_rel, .5)))

  @staticmethod
  def _path_offset(obj, path):
    if not path:
      return obj.y_rel
    before = None
    for x, y in path:
      if not math.isfinite(x) or not math.isfinite(y):
        continue
      if x >= obj.d_rel:
        if before is None or x == before[0]:
          path_y = y
        else:
          ratio = (obj.d_rel-before[0])/(x-before[0])
          path_y = before[1] + ratio*(y-before[1])
        return obj.y_rel + path_y  # model y is right-positive; radar y is left-positive
      before = (x, y)
    return obj.y_rel + before[1] if before is not None else obj.y_rel

  @staticmethod
  def _strict_independent(newborn_pid, anchor_pid, strict):
    if newborn_pid not in strict:
      return False
    return anchor_pid not in strict or strict[newborn_pid] != strict[anchor_pid]

  @staticmethod
  def _world_speed(obj, v_ego, yaw):
    return obj.v_rel + v_ego - yaw*obj.y_rel

  def _metrics(self, newborn, anchor, v_ego, yaw):
    return (abs(newborn.d_rel-anchor.d_rel), abs(newborn.y_rel-anchor.y_rel),
            abs(self._bearing(newborn)-self._bearing(anchor)),
            abs(newborn.v_rel-anchor.v_rel),
            abs(self._world_speed(newborn, v_ego, yaw)-self._world_speed(anchor, v_ego, yaw)))

  @staticmethod
  def _geometry_ok(metrics, newborn_world_speed):
    dd, dy, _db, dv, dw = metrics
    return (BOSCH_FAMILY_MIN_D_M < dd <= BOSCH_FAMILY_MAX_D_M and
            dy <= BOSCH_FAMILY_MAX_Y_M and dv <= BOSCH_FAMILY_MAX_DV_MPS and
            dw <= BOSCH_FAMILY_MAX_WORLD_SPEED_MPS and
            abs(newborn_world_speed) >= BOSCH_FAMILY_MIN_WORLD_SPEED_MPS)

  def _decision(self, timestamp_ns, action, reason, state, newborn=None, anchor=None,
                metrics=None, strict=(), oem=(), d_path=None, suppressed=False):
    strict_map = strict if isinstance(strict, dict) else {}
    strict_identity = self._strict_independent(state.newborn_pid, state.anchor_pid, strict_map)
    oem_identity = state.newborn_pid in oem
    return BoschFamilyCompanionDecision(
      timestamp_ns, action, reason, state.newborn_pid, state.newborn_raw_id,
      state.anchor_pid, state.anchor_raw_ids, state.delta_d_m,
      metrics[0] if metrics is not None else None, state.delta_y_m,
      metrics[1] if metrics is not None else None,
      metrics[3] if metrics is not None else None,
      metrics[4] if metrics is not None else None,
      metrics[2] if metrics is not None else None,
      newborn.age_scans if newborn is not None else None,
      anchor.age_scans if anchor is not None else None,
      len(anchor.members) if anchor is not None else None,
      bool(newborn and newborn.vision_supported), strict_identity, oem_identity,
      d_path, suppressed)

  def reset(self, timestamp_ns, reason='STATE_RESET'):
    decisions = []
    for state in self._states.values():
      if state.active:
        decisions.append(self._decision(timestamp_ns, 'RELEASE', reason, state))
        self.releases += 1
    self._states = {}
    self.would_suppress = frozenset()
    self.last_decisions = tuple(decisions)
    self.last_ns = timestamp_ns

  def update(self, objects, timestamp_ns, v_ego, *, yaw_rate=None,
             strict_associations=None, oem_pids=(), excluded_pids=(), path=()):
    self.would_suppress = frozenset()
    self.last_decisions = ()
    self.pair_evaluations_last = 0
    if self.mode == BOSCH_FAMILY_COMPANION_OFF:
      self._states = {}
      self.last_ns = timestamp_ns
      return self.would_suppress
    if (self.last_ns is not None and
        (timestamp_ns <= self.last_ns or timestamp_ns-self.last_ns > BOSCH_FAMILY_MAX_GAP_NS)):
      self.reset(timestamp_ns, 'STATE_RESET' if timestamp_ns <= self.last_ns else 'SCAN_GAP')
      # A gap/reset scan cannot be the first half of a proof.
      return self.would_suppress
    self.last_ns = timestamp_ns
    strict = strict_associations or {}
    oem = set(oem_pids)
    excluded = set(excluded_pids)
    oem.update(obj.physical_track_id for obj in objects if obj.oem_selected)
    yaw = yaw_rate if yaw_rate is not None and math.isfinite(yaw_rate) else 0.
    if not math.isfinite(v_ego):
      self.reset(timestamp_ns, 'SPEED_DIVERGENCE')
      return self.would_suppress

    present = {obj.physical_track_id: obj for obj in objects}
    next_states = {}
    suppress = set()
    decisions = list(self.last_decisions)
    handled = set()
    for pid, prior in self._states.items():
      handled.add(pid)
      newborn, anchor = present.get(pid), present.get(prior.anchor_pid)
      if pid in excluded:
        reason = 'CONTINUITY_LOSS'
      elif newborn is None:
        anchor_members = {member.raw_track_id for member in anchor.members} if anchor is not None else set()
        reason = ('NORMAL_HANDOFF' if prior.newborn_raw_id in anchor_members else
                  ('ANCHOR_LOST' if anchor is None else 'CONTINUITY_LOSS'))
      elif anchor is None:
        reason = 'ANCHOR_LOST'
      elif (newborn.age_scans <= prior.newborn_age or anchor.age_scans <= prior.anchor_age):
        reason = 'STATE_RESET'
      elif not prior.active and (prior.newborn_age != 1 or newborn.age_scans != 2):
        reason = 'STATE_RESET'
      elif (len(newborn.members) != 1 or newborn.members[0].raw_track_id != prior.newborn_raw_id or
            tuple(member.raw_track_id for member in anchor.members) != prior.anchor_raw_ids or
            anchor.representative_raw_track_id != prior.anchor_representative_raw_id):
        reason = 'CONTINUITY_LOSS'
      elif self._strict_independent(pid, prior.anchor_pid, strict):
        reason = 'STRICT_CAMERA'
      elif pid in oem:
        reason = 'OEM_IDENTITY'
      else:
        metrics = self._metrics(newborn, anchor, v_ego, yaw)
        newborn_world = self._world_speed(newborn, v_ego, yaw)
        if (metrics[3] > BOSCH_FAMILY_MAX_DV_MPS or
            metrics[4] > BOSCH_FAMILY_MAX_WORLD_SPEED_MPS or
            abs(newborn_world) < BOSCH_FAMILY_MIN_WORLD_SPEED_MPS):
          reason = 'SPEED_DIVERGENCE'
        elif (not self._geometry_ok(metrics, newborn_world) or
              abs(metrics[0]-prior.delta_d_m) > BOSCH_FAMILY_MAX_D_STEP_M or
              abs(metrics[1]-prior.delta_y_m) > BOSCH_FAMILY_MAX_Y_STEP_M):
          reason = 'GEOMETRY_DIVERGENCE'
        else:
          d_path = self._path_offset(newborn, path)
          abs_path = abs(d_path)
          if abs_path < abs(prior.d_path_m):
            inward_scans = prior.inward_scans + 1
            inward_from = prior.inward_from_abs_path_m if prior.inward_scans else abs(prior.d_path_m)
            inward_since = prior.inward_since_ns if prior.inward_scans else prior.last_ns
          else:
            inward_scans, inward_from, inward_since = 0, abs_path, timestamp_ns
          inward_rate = ((inward_from-abs_path) / max(1e-3, (timestamp_ns-inward_since)*1e-9)
                         if inward_scans >= BOSCH_FAMILY_INWARD_SCANS else 0.)
          if inward_rate >= BOSCH_FAMILY_INWARD_MPS:
            reason = 'INWARD_MOTION'
          else:
            active = prior.active or newborn.age_scans == 2
            state = _BoschFamilyCompanionState(
              pid, prior.anchor_pid, prior.newborn_raw_id, prior.anchor_raw_ids,
              prior.anchor_representative_raw_id, prior.first_ns, timestamp_ns,
              newborn.age_scans, anchor.age_scans, metrics[0], metrics[1], metrics[2],
              metrics[3], metrics[4], d_path, inward_scans, inward_from, inward_since, active)
            next_states[pid] = state
            if active:
              suppress.add(pid)
            if active and not prior.active:
              self.holds += 1
              decisions.append(self._decision(timestamp_ns, 'HOLD', '', prior, newborn, anchor,
                                                metrics, strict, oem, d_path, True))
            continue
      if prior.active:
        action = 'HANDOFF' if reason == 'NORMAL_HANDOFF' else 'RELEASE'
        decisions.append(self._decision(timestamp_ns, action, reason, prior, newborn, anchor,
                                          strict=strict, oem=oem,
                                          d_path=self._path_offset(newborn, path) if newborn else None))
        if action == 'HANDOFF':
          self.handoffs += 1
        else:
          self.releases += 1

    # New scan-one singleton births.  Distance ordering bounds the search to
    # the fixed 12 m B1 window instead of adding an all-object pair pass.
    anchors = sorted((obj for obj in objects
                      if obj.age_scans >= BOSCH_FAMILY_MIN_ANCHOR_AGE_SCANS and
                      obj.physical_track_id not in excluded),
                     key=lambda obj: obj.d_rel)
    for newborn in objects:
      pid = newborn.physical_track_id
      if (pid in handled or pid in excluded or newborn.age_scans != 1 or
          len(newborn.members) != 1 or pid in oem):
        continue
      plausible = []
      for anchor in anchors:
        if anchor.physical_track_id == pid:
          continue
        if anchor.d_rel < newborn.d_rel-BOSCH_FAMILY_MAX_D_M:
          continue
        if anchor.d_rel > newborn.d_rel+BOSCH_FAMILY_MAX_D_M:
          break
        self.pair_evaluations_last += 1
        metrics = self._metrics(newborn, anchor, v_ego, yaw)
        if (not self._strict_independent(pid, anchor.physical_track_id, strict) and
            self._geometry_ok(metrics, self._world_speed(newborn, v_ego, yaw))):
          plausible.append((anchor, metrics))
      if not plausible:
        continue
      # B1 is existential: a stable mature family anchor completes the proof.
      # Prefer the most mature qualifying anchor, then the closest fixed-band
      # geometry, so scan-one candidate ordering cannot change the result.
      anchor, metrics = min(plausible, key=lambda item: (
        -item[0].age_scans, item[1][0], item[1][1], item[0].physical_track_id))
      d_path = self._path_offset(newborn, path)
      next_states[pid] = _BoschFamilyCompanionState(
        pid, anchor.physical_track_id, newborn.members[0].raw_track_id,
        tuple(member.raw_track_id for member in anchor.members), anchor.representative_raw_track_id,
        timestamp_ns, timestamp_ns, newborn.age_scans, anchor.age_scans,
        metrics[0], metrics[1], metrics[2], metrics[3], metrics[4], d_path,
        0, abs(d_path), timestamp_ns, False)

    self._states = next_states
    self.would_suppress = frozenset(suppress)
    self.last_decisions = tuple(decisions)
    self.pair_evaluations_total += self.pair_evaluations_last
    self.pair_evaluations_peak = max(self.pair_evaluations_peak, self.pair_evaluations_last)
    self.state_peak = max(self.state_peak, len(next_states))
    return self.would_suppress

  def publication_view(self, objects):
    if self.mode != BOSCH_FAMILY_COMPANION_ACTIVE or not self.would_suppress or not objects:
      return objects
    present = {obj.physical_track_id for obj in objects}
    suppressed = {pid for pid in self.would_suppress
                  if pid in present and self._states[pid].anchor_pid in present}
    if not suppressed:
      return objects
    self.publication_suppressed += len(suppressed)
    return tuple(obj for obj in objects if obj.physical_track_id not in suppressed)


@dataclass
class _BoschBurstMultiReturnState:
  child_pid: int
  anchor_pid: int
  arm_ns: int
  last_ns: int
  burst_members: int
  burst_d_span_m: float
  burst_y_span_m: float
  burst_world_mean_mps: float
  child_d_rel_m: float
  child_y_rel_m: float
  anchor_age: int
  held_scans: int
  missing_scans: int


@dataclass(frozen=True)
class BoschBurstMultiReturnDecision:
  timestamp_ns: int
  action: str
  release_reason: str
  child_pid: int
  anchor_pid: int
  burst_members: int
  burst_d_span_m: float
  burst_y_span_m: float
  burst_world_mean_mps: float
  child_d_rel_m: float
  child_y_rel_m: float
  anchor_age: int
  held_scans: int


class _BoschBurstMultiReturnDefer:
  """S33 burst multi-return proof and publication-only defer.

  One physical body can break into several radar surfaces inside a single
  scan.  The signature this reads is entirely present-scan physics: three or
  more singletons born together, their world speeds inside one narrow band,
  their ranges inside a thin slab, their lateral positions fanned wide apart,
  and a mature object already tracked at that same world speed nearby.  A
  burst that wide cannot be three vehicles that all became visible in the same
  100 ms while driving abreast inside an 8 m lateral window at one speed.

  Absence of camera or word1 support is never the reason a hold starts: the
  burst geometry is.  Independent support only decides which members of an
  already-proven burst are deferred, and regaining any of it releases the hold
  immediately.  Nothing here touches raw tracks, physical identity, grouping,
  representatives, camera or OEM history, or aliases -- a deferred object keeps
  every bit of its state and can be published on the very next scan.
  """

  def __init__(self, mode=BOSCH_BURST_MULTIRETURN_MODE):
    if mode not in (BOSCH_BURST_MULTIRETURN_OFF, BOSCH_BURST_MULTIRETURN_SHADOW,
                    BOSCH_BURST_MULTIRETURN_ACTIVE):
      raise ValueError('invalid Bosch burst multi-return mode')
    self.mode = mode
    self._states: dict[int, _BoschBurstMultiReturnState] = {}
    self.last_ns = None
    self.would_suppress = frozenset()
    self.last_decisions: tuple[BoschBurstMultiReturnDecision, ...] = ()
    self.pair_evaluations_last = self.pair_evaluations_total = self.pair_evaluations_peak = 0
    self.holds = self.releases = self.publication_suppressed = 0
    self.state_peak = 0

  @staticmethod
  def _world_speed(obj, v_ego, yaw):
    return obj.v_rel + v_ego - yaw*obj.y_rel

  def _decision(self, timestamp_ns, action, reason, state):
    return BoschBurstMultiReturnDecision(
      timestamp_ns, action, reason, state.child_pid, state.anchor_pid, state.burst_members,
      state.burst_d_span_m, state.burst_y_span_m, state.burst_world_mean_mps,
      state.child_d_rel_m, state.child_y_rel_m, state.anchor_age, state.held_scans)

  def reset(self, timestamp_ns, reason='STATE_RESET'):
    decisions = [self._decision(timestamp_ns, 'RELEASE', reason, state)
                 for state in self._states.values()]
    self.releases += len(decisions)
    self._states = {}
    self.would_suppress = frozenset()
    self.last_decisions = tuple(decisions)
    self.last_ns = timestamp_ns

  def update(self, objects, timestamp_ns, v_ego, *, yaw_rate=None, word0_validated_pids=()):
    self.would_suppress = frozenset()
    self.last_decisions = ()
    self.pair_evaluations_last = 0
    if self.mode == BOSCH_BURST_MULTIRETURN_OFF:
      self._states = {}
      self.last_ns = timestamp_ns
      return self.would_suppress
    if not math.isfinite(v_ego):
      self.reset(timestamp_ns, 'SPEED_DIVERGENCE')
      return self.would_suppress
    if (self.last_ns is not None and
        (timestamp_ns <= self.last_ns or timestamp_ns-self.last_ns > BOSCH_BURST_MAX_GAP_NS)):
      # The proof is a one-scan observation; a discontinuity cannot carry a hold
      # across it, and cannot be the scan that starts one either.
      self.reset(timestamp_ns, 'STATE_RESET' if timestamp_ns <= self.last_ns else 'SCAN_GAP')
      return self.would_suppress
    self.last_ns = timestamp_ns
    present = {obj.physical_track_id: obj for obj in objects}
    validated = set(word0_validated_pids)
    decisions = []
    suppress = set()
    alive = {}
    for pid, state in self._states.items():
      child = present.get(pid)
      if child is None:
        if state.missing_scans < BOSCH_BURST_MAX_MISSING_SCANS:
          # One dropped scan is ordinary radar behaviour, not a new object.
          state.missing_scans += 1
          alive[pid] = state
          suppress.add(pid)
          continue
        reason = 'CHILD_LOST'
      elif child.oem_selected or child.vision_supported or pid in validated:
        reason = 'CHILD_EVIDENCE'
      elif len(child.members) != 1:
        reason = 'CHILD_GREW'
      elif state.held_scans >= BOSCH_BURST_HOLD_MAX_SCANS:
        reason = 'HOLD_BUDGET'
      else:
        state.held_scans += 1
        state.missing_scans = 0
        state.last_ns = timestamp_ns
        alive[pid] = state
        suppress.add(pid)
        continue
      decisions.append(self._decision(timestamp_ns, 'RELEASE', reason, state))
      self.releases += 1
    self._states = alive

    if v_ego >= BOSCH_BURST_MIN_VEGO_MPS:
      yaw = yaw_rate if yaw_rate is not None and math.isfinite(yaw_rate) else 0.
      newborn = [(self._world_speed(obj, v_ego, yaw), obj) for obj in objects
                 if obj.age_scans == 1 and len(obj.members) == 1]
      if len(newborn) >= BOSCH_BURST_MIN_MEMBERS:
        # World speed orders the scan; range, offset and identity break ties so
        # the banding cannot depend on the order objects arrived in.
        newborn.sort(key=lambda item: (item[0], item[1].d_rel, item[1].y_rel,
                                       item[1].physical_track_id))
        others = None
        index = 0
        while index < len(newborn):
          end = index
          while (end+1 < len(newborn) and
                 newborn[end+1][0]-newborn[index][0] <= BOSCH_BURST_MAX_WORLD_SPAN_MPS):
            end += 1
          group, index = newborn[index:end+1], end+1
          if len(group) < BOSCH_BURST_MIN_MEMBERS:
            continue
          ranges = [obj.d_rel for _speed, obj in group]
          offsets = [obj.y_rel for _speed, obj in group]
          if (max(ranges)-min(ranges) > BOSCH_BURST_MAX_D_SPAN_M or
              max(offsets)-min(offsets) < BOSCH_BURST_MIN_Y_SPAN_M):
            continue
          world_mean = sum(speed for speed, _obj in group)/len(group)
          if abs(world_mean) < BOSCH_BURST_MIN_WORLD_SPEED_MPS:
            continue
          if others is None:
            others = [(self._world_speed(obj, v_ego, yaw), obj) for obj in objects
                      if obj.age_scans >= BOSCH_BURST_NEIGHBOUR_AGE_SCANS]
          members = {obj.physical_track_id for _speed, obj in group}
          neighbours = []
          for speed, obj in others:
            if obj.physical_track_id in members:
              continue
            self.pair_evaluations_last += 1
            if (abs(speed-world_mean) <= BOSCH_BURST_MAX_WORLD_SPAN_MPS and
                min(abs(obj.d_rel-value) for value in ranges) <= BOSCH_BURST_NEIGHBOUR_D_M):
              neighbours.append(obj)
          if not neighbours:
            continue
          # The burst is proven; the anchor is recorded, never decided on.
          anchor = min(neighbours, key=lambda obj: (-obj.age_scans, obj.d_rel,
                                                    obj.physical_track_id))
          d_span = max(ranges)-min(ranges)
          y_span = max(offsets)-min(offsets)
          for _speed, child in group:
            pid = child.physical_track_id
            if (pid in self._states or child.oem_selected or child.vision_supported or
                pid in validated):
              continue
            if len(self._states) >= BOSCH_BURST_STATE_MAX:
              break
            state = _BoschBurstMultiReturnState(
              pid, anchor.physical_track_id, timestamp_ns, timestamp_ns, len(group),
              d_span, y_span, world_mean, child.d_rel, child.y_rel, anchor.age_scans, 1, 0)
            self._states[pid] = state
            suppress.add(pid)
            self.holds += 1
            decisions.append(self._decision(timestamp_ns, 'HOLD', '', state))

    self.would_suppress = frozenset(suppress)
    self.last_decisions = tuple(decisions)
    self.pair_evaluations_total += self.pair_evaluations_last
    self.pair_evaluations_peak = max(self.pair_evaluations_peak, self.pair_evaluations_last)
    self.state_peak = max(self.state_peak, len(self._states))
    return self.would_suppress

  def publication_view(self, objects):
    if self.mode != BOSCH_BURST_MULTIRETURN_ACTIVE or not self.would_suppress or not objects:
      return objects
    suppressed = {obj.physical_track_id for obj in objects
                  if obj.physical_track_id in self.would_suppress}
    if not suppressed:
      return objects
    self.publication_suppressed += len(suppressed)
    return tuple(obj for obj in objects if obj.physical_track_id not in suppressed)


@dataclass(slots=True)
class _BoschSidePassLateralState:
  raw_track_id: int
  samples: deque
  armed: bool = False
  anchor_y_m: float = 0.
  offset_m: float = 0.
  quiet_scans: int = 0
  last_ns: int = 0


@dataclass(frozen=True)
class BoschSidePassLateralDecision:
  timestamp_ns: int
  physical_track_id: int
  action: str
  reason: str
  slide_mps: float
  raw_y_m: float
  published_y_m: float


class _BoschSidePassLateralEstimator:
  """Measurement gating of a sliding scatterer's lateral position.

  Arming reads present physics only: a singleton, same-direction object being
  overtaken, outside the body-overlap corridor and inside the downstream cut-in
  scope, whose representative raw track closed its range at least 1.5 m/s more
  slowly than its own vRel over the last 0.4-0.8 s.  A rigid point cannot do
  that; a scattering centre moving along a long body does.  Missing camera or
  OEM support never arms it.

  While armed the published lateral keeps the value already published at arm
  time.  Every lane-entry signal releases it on the same scan: the raw return
  inside the corridor, OEM selection (word1 member or validated word0), or a
  gated displacement larger than migration on one body.  When the slide ends
  the published lateral keeps following every later raw movement one-to-one and
  only its residual offset re-aligns, at a rate below the downstream
  lateral-motion floor, so re-alignment itself never looks like an entry.
  State is keyed by physical ID and representative
  raw track; a new ID, a representative change or a scan gap never inherits an
  anchor, and only the scan's own objects are ever adjusted.  An anchored ID
  that misses scans keeps its state for at most the scan-gap limit, so a
  one-scan qualification drop is not published as an anchor-to-raw step.
  """

  def __init__(self, mode=BOSCH_SIDEPASS_LATERAL_MODE):
    if mode not in (BOSCH_SIDEPASS_LATERAL_OFF, BOSCH_SIDEPASS_LATERAL_SHADOW,
                    BOSCH_SIDEPASS_LATERAL_ACTIVE):
      raise ValueError('invalid Bosch side-pass lateral mode')
    self.mode = mode
    self._states: dict[int, _BoschSidePassLateralState] = {}
    self.last_ns = None
    self.published_y: dict[int, float] = {}
    self.last_decisions: tuple[BoschSidePassLateralDecision, ...] = ()
    self.arms = self.releases = self.publication_adjusted = self.state_peak = 0

  def reset(self, timestamp_ns, reason='STATE_RESET'):
    self.last_decisions = tuple(
      BoschSidePassLateralDecision(timestamp_ns, pid, 'RESET', reason, math.nan, math.nan, math.nan)
      for pid, state in sorted(self._states.items()) if state.armed or state.offset_m)
    self._states = {}
    self.published_y = {}
    self.last_ns = timestamp_ns

  @staticmethod
  def _slide(samples):
    """Least-squares range rate minus mean vRel over the window, or None."""
    n = len(samples)
    if n < BOSCH_SIDEPASS_MIN_SCANS or samples[-1][0] - samples[0][0] < BOSCH_SIDEPASS_MIN_SPAN_NS:
      return None
    t0 = samples[0][0]
    st = sd = stt = std = sv = 0.
    for ns, d_rel, v_rel in samples:
      t = (ns - t0) * 1e-9
      st += t
      sd += d_rel
      stt += t * t
      std += t * d_rel
      sv += v_rel
    denominator = n * stt - st * st
    if denominator <= 0.:
      return None
    return (n * std - st * sd) / denominator - sv / n

  def update(self, objects, timestamp_ns, v_ego, *, path=(), oem_pids=()):
    self.last_decisions = ()
    if self.mode == BOSCH_SIDEPASS_LATERAL_OFF:
      self._states = {}
      self.published_y = {}
      self.last_ns = timestamp_ns
      return self.published_y
    decisions = []
    if (self.last_ns is not None and
        (timestamp_ns <= self.last_ns or timestamp_ns - self.last_ns > BOSCH_SIDEPASS_MAX_GAP_NS)):
      self.reset(timestamp_ns, 'STATE_RESET' if timestamp_ns <= self.last_ns else 'SCAN_GAP')
      decisions.extend(self.last_decisions)
    self.last_ns = timestamp_ns
    states = self._states
    speed_known = math.isfinite(v_ego)
    published = {}
    live = set()
    for obj in objects:
      pid = obj.physical_track_id
      state = states.get(pid)
      members = obj.members
      representative = obj.representative_raw_track_id
      # A window is only kept where arming is reachable; an anchor is kept
      # until it has been released or re-aligned.
      reachable = (obj.v_rel < -.5 and speed_known and obj.v_rel + v_ego >= 1. and
                   obj.d_rel <= BOSCH_SIDEPASS_MAX_D_M + 5. and abs(obj.y_rel) <= BOSCH_SIDEPASS_ZONE_M + 1.5)
      if state is None or state.raw_track_id != representative:
        if state is not None:
          if state.armed or state.offset_m:
            decisions.append(BoschSidePassLateralDecision(
              timestamp_ns, pid, 'RESET', 'REPRESENTATIVE_CHANGE', math.nan, obj.y_rel, obj.y_rel))
          del states[pid]
        if len(members) != 1 or not reachable:
          continue
        state = states[pid] = _BoschSidePassLateralState(representative, deque())
      elif not (reachable or state.armed or state.offset_m):
        del states[pid]
        continue
      member = members[0] if len(members) == 1 else next(
        m for m in members if m.raw_track_id == representative)
      samples = state.samples
      samples.append((member.timestamp_ns, obj.d_rel, obj.v_rel))
      while timestamp_ns - samples[0][0] > BOSCH_SIDEPASS_WINDOW_NS:
        samples.popleft()
      state.last_ns = timestamp_ns
      live.add(pid)
      slide = self._slide(samples)
      y_rel = obj.y_rel
      published_y = y_rel + state.offset_m
      closing = obj.v_rel <= -BOSCH_SIDEPASS_CLOSING_MPS
      moving = speed_known and obj.v_rel + v_ego >= BOSCH_SIDEPASS_MIN_LEAD_SPEED_MPS
      armable = (slide is not None and slide >= BOSCH_SIDEPASS_SLIDE_ARM_MPS and len(members) == 1 and
                 closing and moving and obj.d_rel <= BOSCH_SIDEPASS_MAX_D_M)
      if not (state.armed or state.offset_m or armable):
        continue
      offset = _BoschFamilyCompanionFilter._path_offset(obj, path)
      release = None
      if abs(offset) < BOSCH_SIDEPASS_CORRIDOR_M:
        release = 'CORRIDOR_ENTRY'
      elif pid in oem_pids or obj.oem_selected:
        release = 'OEM_SELECTED'
      elif abs(state.offset_m) > BOSCH_SIDEPASS_MAX_OFFSET_M:
        release = 'BEYOND_BODY'
      if release is not None:
        if state.armed or state.offset_m:
          decisions.append(BoschSidePassLateralDecision(
            timestamp_ns, pid, 'RELEASE', release, math.nan if slide is None else slide, y_rel, y_rel))
          self.releases += 1
        state.armed = False
        state.offset_m = 0.
        state.quiet_scans = 0
        continue
      in_zone = abs(offset) <= BOSCH_SIDEPASS_ZONE_M and obj.d_rel <= BOSCH_SIDEPASS_MAX_D_M
      if not state.armed:
        if armable and in_zone:
          state.armed = True
          state.anchor_y_m = published_y
          state.quiet_scans = 0
          self.arms += 1
          decisions.append(BoschSidePassLateralDecision(timestamp_ns, pid, 'ARM', 'SLIDE', slide, y_rel, published_y))
      else:
        quiet = (slide is None or slide < BOSCH_SIDEPASS_SLIDE_RELEASE_MPS or not closing or not moving or
                 not in_zone or len(members) != 1)
        state.quiet_scans = state.quiet_scans + 1 if quiet else 0
        if state.quiet_scans >= BOSCH_SIDEPASS_RELEASE_SCANS:
          state.armed = False
          self.releases += 1
          decisions.append(BoschSidePassLateralDecision(
            timestamp_ns, pid, 'RELEASE', 'EVIDENCE_END', math.nan if slide is None else slide, y_rel, published_y))
      if state.armed:
        state.offset_m = state.anchor_y_m - y_rel
        if abs(state.offset_m) > BOSCH_SIDEPASS_MAX_OFFSET_M:
          state.armed = False
          state.offset_m = 0.
          self.releases += 1
          decisions.append(BoschSidePassLateralDecision(
            timestamp_ns, pid, 'RELEASE', 'BEYOND_BODY', slide, y_rel, y_rel))
      elif state.offset_m:
        step = BOSCH_SIDEPASS_REALIGN_MPS * BOSCH_OUTPUT_INTERVAL_NS * 1e-9
        if abs(state.offset_m) <= step:
          state.offset_m = 0.
          decisions.append(BoschSidePassLateralDecision(
            timestamp_ns, pid, 'REALIGNED', '', math.nan if slide is None else slide, y_rel, y_rel))
        else:
          state.offset_m -= math.copysign(step, state.offset_m)
      if state.offset_m:
        published[pid] = y_rel + state.offset_m
    for pid in [pid for pid in states if pid not in live]:
      state = states[pid]
      # A pre-arm window only delays arming.  An anchor survives a short
      # absence of the same ID and is dropped, with a decision, after that.
      if state.armed or state.offset_m:
        if timestamp_ns - state.last_ns <= BOSCH_SIDEPASS_MAX_GAP_NS:
          continue
        decisions.append(BoschSidePassLateralDecision(
          timestamp_ns, pid, 'RESET', 'ABSENT', math.nan, math.nan, math.nan))
      del states[pid]
    if len(states) > BOSCH_SIDEPASS_STATE_MAX:
      # An evicted pre-arm window only delays arming; anchors are kept first.
      victims = sorted(states, key=lambda key: (bool(states[key].armed or states[key].offset_m),
                                                states[key].last_ns, key))
      for pid in victims[:len(states) - BOSCH_SIDEPASS_STATE_MAX]:
        del states[pid]
        published.pop(pid, None)
    self.state_peak = max(self.state_peak, len(states))
    self.published_y = published
    self.last_decisions = tuple(decisions)
    return published

  def publication_view(self, objects):
    if self.mode != BOSCH_SIDEPASS_LATERAL_ACTIVE or not self.published_y or not objects:
      return objects
    adjusted = None
    for index, obj in enumerate(objects):
      y_rel = self.published_y.get(obj.physical_track_id)
      state = self._states.get(obj.physical_track_id)
      # Only this scan's objects, and only when the tracked raw track is the
      # member actually published (never another member's surface).
      if (y_rel is None or state is None or obj.timestamp_ns != self.last_ns or
          obj.published_surface is not None or obj.representative_raw_track_id != state.raw_track_id):
        continue
      if adjusted is None:
        adjusted = list(objects)
      adjusted[index] = BoschPhysicalObject(
        obj.physical_track_id, obj.timestamp_ns, obj.members, obj.representative_raw_track_id,
        obj.d_rel, obj.y_rel, obj.v_rel, obj.oem_selected, obj.vision_supported, obj.age_scans,
        obj.grouping_evidence,
        BoschPublishedSurface(state.raw_track_id, obj.d_rel, y_rel, obj.v_rel, lateral_estimate=True))
      self.publication_adjusted += 1
    return objects if adjusted is None else tuple(adjusted)


@dataclass
class _BoschP91PairState:
  parent_pid: int
  since_ns: int
  last_ns: int
  samples: int
  candidate_raw_id: int
  parent_representative_raw_id: int
  d_offset_mean: float
  d_offset_m2: float
  y_offset_mean: float
  y_offset_m2: float
  dv_sq_sum: float
  anchor_candidate_y: float
  anchor_y_offset: float
  last_candidate_y: float
  last_d_offset: float
  last_y_offset: float


class _BoschPersistentSpatialCloneFilter:
  """Bosch-only persistent unsupported spatial-copy shadow detector.

  This is not an 0x601 authenticity filter. Supported objects form a small
  parent pool; support on the candidate is instead an immediate fail-open.
  One fixed-size state is retained per live candidate PID, never a full pair
  history. The Bosch research branch keeps SHADOW diagnostics while ACTIVE
  removes only a mature candidate from the final publication view.
  """
  _EVIDENCE_NS = 3_000_000_000
  # Bosch singleton returns may be absent for one 10 Hz scan; measured P91
  # component gaps peak just below 300 ms. Keep fixed state through that one
  # missing observation, while never suppressing an absent object.
  _GAP_NS = 320_000_000
  _MIN_SAMPLES = 25
  _MIN_EGO_SPEED_MPS = 3.0
  _MIN_WORLD_SPEED_MPS = 4.0
  _MAX_DV_MPS = .75
  _MAX_V_RMSE_MPS = .35
  _MIN_D_OFFSET_M = 6.0
  _MAX_D_OFFSET_M = 30.0
  _MIN_Y_SEPARATION_M = 3.5
  _MAX_D_OFFSET_STD_M = .75
  _MAX_Y_OFFSET_STD_M = .80
  _MAX_D_OFFSET_STEP_M = 1.5
  _MAX_Y_OFFSET_STEP_M = .75
  _MAX_CANDIDATE_Y_STEP_M = .75
  _MAX_LATERAL_EXCURSION_M = 2.5

  def __init__(self, mode=BOSCH_P91_MODE):
    if mode not in (BOSCH_P91_OFF, BOSCH_P91_SHADOW, BOSCH_P91_ACTIVE):
      raise ValueError('invalid Bosch Candidate P91 mode')
    self.mode = mode
    self._states: dict[int, _BoschP91PairState] = {}
    self._support_until: dict[int, int] = {}
    self.last_ns = None
    self.would_suppress = frozenset()
    self.decisions = {}
    self.parent_pool_last = self.parent_pool_peak = 0
    self.pair_evaluations_last = self.pair_evaluations_total = self.pair_evaluations_peak = 0
    self.state_peak = 0
    self.publication_suppressed = 0

  @staticmethod
  def _std(mean, m2, samples):
    return math.sqrt(max(0., m2 / samples)) if samples else math.inf

  @staticmethod
  def _welford(mean, m2, samples, value):
    delta = value - mean
    mean += delta / samples
    return mean, m2 + delta * (value - mean)

  def update(self, objects, timestamp_ns, v_ego, *, word0_pids=(), yaw_rate=None):
    self.last_ns = timestamp_ns
    self.would_suppress = frozenset()
    self.decisions = {}
    self.pair_evaluations_last = 0
    if self.mode == BOSCH_P91_OFF:
      self._states = {}
      self._support_until = {}
      return self.would_suppress
    if not math.isfinite(v_ego) or v_ego < self._MIN_EGO_SPEED_MPS or not objects:
      self._states = {}
      self._support_until = {}
      return self.would_suppress

    live = {obj.physical_track_id for obj in objects}
    word0 = frozenset(word0_pids)
    support_until = {pid: expiry for pid, expiry in self._support_until.items()
                     if pid in live and expiry >= timestamp_ns}
    for obj in objects:
      if obj.vision_supported or obj.oem_selected or obj.physical_track_id in word0:
        support_until[obj.physical_track_id] = timestamp_ns + BOSCH_P91_SUPPORT_HOLD_NS
    self._support_until = support_until
    supported = {pid for pid, expiry in support_until.items() if expiry >= timestamp_ns}
    parents = tuple(obj for obj in objects if obj.physical_track_id in supported)
    self.parent_pool_last = len(parents)
    self.parent_pool_peak = max(self.parent_pool_peak, len(parents))
    if not parents:
      self._states = {}
      return self.would_suppress

    rotation = yaw_rate if yaw_rate is not None and math.isfinite(yaw_rate) else 0.
    previous_states = self._states
    states = {}
    suppress = set()
    for candidate in objects:
      pid = candidate.physical_track_id
      # Any recent independent support is an immediate KEEP/reset. Multi-member
      # objects stay inside the existing large-vehicle/member-aware path.
      if pid in supported or len(candidate.members) != 1:
        continue
      world_speed = candidate.v_rel + v_ego - rotation * candidate.y_rel
      if not math.isfinite(world_speed) or world_speed < self._MIN_WORLD_SPEED_MPS:
        continue

      previous = previous_states.get(pid)
      plausible = []
      for parent in parents:
        if parent.physical_track_id == pid:
          continue
        self.pair_evaluations_last += 1
        d_offset = candidate.d_rel - parent.d_rel
        y_offset = candidate.y_rel - parent.y_rel
        dv = candidate.v_rel - parent.v_rel
        if (self._MIN_D_OFFSET_M <= d_offset <= self._MAX_D_OFFSET_M and
            abs(y_offset) >= self._MIN_Y_SEPARATION_M and abs(dv) <= self._MAX_DV_MPS):
          plausible.append((abs(dv), -parent.age_scans, parent.physical_track_id,
                            parent, d_offset, y_offset, dv))
      if not plausible:
        continue
      if previous is not None:
        chosen = next((item for item in plausible if item[3].physical_track_id == previous.parent_pid), None)
      else:
        chosen = None
      _, _, _, parent, d_offset, y_offset, dv = chosen or min(plausible)
      member_id = candidate.members[0].raw_track_id
      continuous = bool(
        previous is not None and previous.parent_pid == parent.physical_track_id and
        0 < timestamp_ns - previous.last_ns <= self._GAP_NS and
        previous.candidate_raw_id == member_id and
        previous.parent_representative_raw_id == parent.representative_raw_track_id and
        abs(d_offset - previous.last_d_offset) <= self._MAX_D_OFFSET_STEP_M and
        abs(y_offset - previous.last_y_offset) <= self._MAX_Y_OFFSET_STEP_M and
        abs(candidate.y_rel - previous.last_candidate_y) <= self._MAX_CANDIDATE_Y_STEP_M and
        abs(candidate.y_rel - previous.anchor_candidate_y) <= self._MAX_LATERAL_EXCURSION_M and
        abs(y_offset - previous.anchor_y_offset) <= self._MAX_LATERAL_EXCURSION_M)
      if not continuous:
        state = _BoschP91PairState(parent.physical_track_id, timestamp_ns, timestamp_ns, 1,
                                   member_id, parent.representative_raw_track_id,
                                   d_offset, 0., y_offset, 0., dv * dv,
                                   candidate.y_rel, y_offset, candidate.y_rel, d_offset, y_offset)
      else:
        samples = previous.samples + 1
        d_mean, d_m2 = self._welford(previous.d_offset_mean, previous.d_offset_m2, samples, d_offset)
        y_mean, y_m2 = self._welford(previous.y_offset_mean, previous.y_offset_m2, samples, y_offset)
        state = _BoschP91PairState(parent.physical_track_id, previous.since_ns, timestamp_ns, samples,
                                   member_id, parent.representative_raw_track_id,
                                   d_mean, d_m2, y_mean, y_m2, previous.dv_sq_sum + dv * dv,
                                   previous.anchor_candidate_y, previous.anchor_y_offset,
                                   candidate.y_rel, d_offset, y_offset)
      states[pid] = state
      age_ns = timestamp_ns - state.since_ns
      v_rmse = math.sqrt(state.dv_sq_sum / state.samples)
      d_std = self._std(state.d_offset_mean, state.d_offset_m2, state.samples)
      y_std = self._std(state.y_offset_mean, state.y_offset_m2, state.samples)
      established = (age_ns >= self._EVIDENCE_NS and state.samples >= self._MIN_SAMPLES and
                     v_rmse <= self._MAX_V_RMSE_MPS and d_std <= self._MAX_D_OFFSET_STD_M and
                     y_std <= self._MAX_Y_OFFSET_STD_M)
      if established:
        suppress.add(pid)
      self.decisions[pid] = {
        'would_suppress': established, 'reason': 'persistent_spatial_copy' if established else 'warming',
        'parent_pid': parent.physical_track_id, 'candidate_pid': pid,
        'evidence_age_s': age_ns * 1e-9, 'samples': state.samples,
        'v_rmse_mps': v_rmse, 'd_offset_mean_m': state.d_offset_mean,
        'd_offset_std_m': d_std, 'y_offset_mean_m': state.y_offset_mean,
        'y_offset_std_m': y_std, 'world_speed_mps': world_speed,
      }
    parent_ids = {parent.physical_track_id for parent in parents}
    for pid, state in previous_states.items():
      if (pid not in live and state.parent_pid in parent_ids and
          timestamp_ns - state.last_ns <= self._GAP_NS):
        states[pid] = state
    self._states = states
    self.would_suppress = frozenset(suppress)
    self.pair_evaluations_total += self.pair_evaluations_last
    self.pair_evaluations_peak = max(self.pair_evaluations_peak, self.pair_evaluations_last)
    self.state_peak = max(self.state_peak, len(states))
    return self.would_suppress


@dataclass
class _BoschOemGateState:
  """Bounded per-candidate probation state. No pair matrix, no history list."""
  tentative_scans: int
  since_ns: int
  last_ns: int
  evidence_ns: int
  ever_validated: bool
  in_path_scans: int
  last_abs_y: float
  inward_scans: int
  inward_from_abs_y: float
  inward_since_ns: int
  withheld: bool


class _BoschOemIntentTracker:
  """Short bounded window over the stock SCC longitudinal request.

  SCC12 runs at 50 Hz while the object list runs at 10 Hz, so a single 20 ms
  sample is never used. Freshness and a sane decoded band are required before
  the value counts; otherwise the caller fails open.
  """

  def __init__(self):
    self._samples = deque()
    self.last_ns = 0

  def reset(self):
    self._samples.clear()
    self.last_ns = 0

  def ingest(self, timestamp_ns, value):
    if value is None or not math.isfinite(value):
      return
    if not BOSCH_OEM_INTENT_MIN_MPS2 <= value <= BOSCH_OEM_INTENT_MAX_MPS2:
      return
    if self._samples and timestamp_ns < self._samples[-1][0]:
      self._samples.clear()
    self._samples.append((timestamp_ns, value))
    self.last_ns = timestamp_ns
    horizon = timestamp_ns - BOSCH_OEM_INTENT_WINDOW_NS
    while self._samples and self._samples[0][0] < horizon:
      self._samples.popleft()

  def median(self, timestamp_ns):
    """Window median, or None when the stream is stale or too sparse."""
    if not self._samples or timestamp_ns - self.last_ns > BOSCH_OEM_INTENT_FRESH_NS:
      return None
    horizon = timestamp_ns - BOSCH_OEM_INTENT_WINDOW_NS
    values = sorted(value for ns, value in self._samples if ns >= horizon)
    if len(values) < BOSCH_OEM_INTENT_MIN_SAMPLES:
      return None
    return values[len(values) // 2]


class _BoschOemValidationGate:
  """Withhold publication of a sustained unvalidated in-path candidate.

  The 0x601 pair is read as a state rather than one support flag. Only the
  single candidate the word0 record points at can be withheld, and only while
  every independent real-object cue is absent and the stock SCC is also not
  asking for deceleration. The physical track, its members, its ID and its
  alias are untouched: this is a publication-view decision that reverses on the
  first scan any evidence returns.
  """

  def __init__(self, mode=BOSCH_OEM_GATE_MODE):
    if mode not in (BOSCH_OEM_GATE_OFF, BOSCH_OEM_GATE_SHADOW, BOSCH_OEM_GATE_ACTIVE):
      raise ValueError('invalid Bosch OEM validation gate mode')
    self.mode = mode
    self.neutral_mps2 = BOSCH_OEM_GATE_NEUTRAL_MPS2
    self.persist_scans = BOSCH_OEM_GATE_PERSIST_SCANS
    self.min_speed_mps = BOSCH_OEM_GATE_MIN_SPEED_MPS
    self.min_range_m = BOSCH_OEM_GATE_MIN_RANGE_M
    self.min_ttc_s = BOSCH_OEM_GATE_MIN_TTC_S
    self.cutin_mps = BOSCH_OEM_GATE_CUTIN_MPS
    self.cutin_scans = BOSCH_OEM_GATE_CUTIN_SCANS
    self.evidence_hold_ns = BOSCH_OEM_EVIDENCE_HOLD_NS
    self.in_path_m = BOSCH_OEM_GATE_IN_PATH_M
    self.settled_scans = BOSCH_OEM_GATE_SETTLED_SCANS
    self.sibling_d_m = BOSCH_OEM_GATE_SIBLING_D_M
    self.sibling_y_m = BOSCH_OEM_GATE_SIBLING_Y_M
    self.sibling_v_mps = BOSCH_OEM_GATE_SIBLING_V_MPS
    self.intent = _BoschOemIntentTracker()
    self._states: dict[int, _BoschOemGateState] = {}
    self.last_ns = None
    self.state = BOSCH_OEM_STATE_NONE
    self.would_withhold = frozenset()
    self.reasons = {}
    self.publication_withheld = 0
    self.state_peak = 0

  @staticmethod
  def classify(word0_active, word1_active):
    if word1_active:
      return BOSCH_OEM_STATE_VALIDATED if word0_active else BOSCH_OEM_STATE_SELECTED
    return BOSCH_OEM_STATE_TENTATIVE if word0_active else BOSCH_OEM_STATE_NONE

  def update(self, objects, timestamp_ns, v_ego, *, state, word0_pids=(), oem_valid=None):
    self.last_ns = timestamp_ns
    self.state = state
    self.would_withhold = frozenset()
    self.reasons = {}
    if self.mode == BOSCH_OEM_GATE_OFF:
      self._states = {}
      return self.would_withhold

    live = {obj.physical_track_id for obj in objects}
    previous = self._states
    states = {}
    # Any independent cue refreshes the evidence clock, including on objects the
    # gate will never consider. A candidate that was validated moments ago keeps
    # its grace through a short word1 dropout.
    validated_word0 = state == BOSCH_OEM_STATE_VALIDATED and oem_valid is not False
    word0 = frozenset(word0_pids)
    for obj in objects:
      pid = obj.physical_track_id
      prior = previous.get(pid)
      evidence_ns = prior.evidence_ns if prior is not None else 0
      ever = prior.ever_validated if prior is not None else False
      if (obj.vision_supported or obj.oem_selected or len(obj.members) > 1 or
          (validated_word0 and pid in word0)):
        evidence_ns = timestamp_ns
      # Once the OEM has confirmed an object it is never a gate candidate again
      # for the rest of that track's life. The route274 S19 return was never
      # confirmed in sixty seconds; a real lead is confirmed within one.
      if obj.oem_selected or (validated_word0 and pid in word0):
        ever = True
      # A real cut-in closes on the path scan after scan. A large-vehicle ghost
      # wanders in and back out, so only an unbroken inward run counts, never a
      # single noisy step of a 0.03125 m lateral grid seen at seventy metres.
      abs_y = abs(obj.y_rel)
      in_path = (prior.in_path_scans + 1 if prior is not None and abs_y <= self.in_path_m
                 else int(abs_y <= self.in_path_m))
      if prior is None or timestamp_ns - prior.last_ns > BOSCH_OEM_GATE_LATERAL_WINDOW_NS:
        inward_scans, inward_from, inward_since = 0, abs_y, timestamp_ns
      elif abs_y < prior.last_abs_y:
        inward_scans = prior.inward_scans + 1
        inward_from = prior.inward_from_abs_y if prior.inward_scans else prior.last_abs_y
        inward_since = prior.inward_since_ns if prior.inward_scans else prior.last_ns
      else:
        inward_scans, inward_from, inward_since = 0, abs_y, timestamp_ns
      states[pid] = _BoschOemGateState(
        prior.tentative_scans if prior is not None else 0,
        prior.since_ns if prior is not None else timestamp_ns,
        timestamp_ns, evidence_ns, ever, in_path, abs_y, inward_scans, inward_from,
        inward_since, prior.withheld if prior is not None else False)

    withhold = set()
    # Exactly one unambiguous word0 candidate is eligible. An ambiguous match
    # cannot name a target and stays fail-open, as elsewhere in this provider.
    eligible = word0 & live if state == BOSCH_OEM_STATE_TENTATIVE and len(word0) == 1 else frozenset()
    intent = self.intent.median(timestamp_ns)
    confirmed = tuple(obj for obj in objects if states[obj.physical_track_id].ever_validated)
    for obj in objects:
      pid = obj.physical_track_id
      entry = states[pid]
      if pid not in eligible:
        entry.tentative_scans = 0
        entry.since_ns = timestamp_ns
        entry.withheld = False
        continue
      entry.tentative_scans += 1
      if entry.tentative_scans == 1:
        entry.since_ns = timestamp_ns
      lateral_rate = ((entry.inward_from_abs_y - entry.last_abs_y) /
                      max(1e-3, (timestamp_ns - entry.inward_since_ns) * 1e-9)
                      if entry.inward_scans >= self.cutin_scans else 0.)
      closing = -obj.v_rel
      ttc = obj.d_rel / closing if closing > .1 else math.inf
      fail_open = None
      if entry.ever_validated:
        fail_open = 'was_validated'
      elif entry.in_path_scans > self.settled_scans:
        fail_open = 'settled_in_path'
      elif abs(obj.y_rel) > self.in_path_m:
        fail_open = 'not_in_path'
      elif any(other.physical_track_id != pid and
               abs(obj.d_rel - other.d_rel) <= self.sibling_d_m and
               abs(obj.y_rel - other.y_rel) <= self.sibling_y_m and
               abs(obj.v_rel - other.v_rel) <= self.sibling_v_mps for other in confirmed):
        fail_open = 'validated_sibling'
      elif oem_valid is True:
        fail_open = 'scc_valid'
      elif not math.isfinite(v_ego) or v_ego < self.min_speed_mps:
        fail_open = 'low_speed'
      elif obj.d_rel < self.min_range_m:
        fail_open = 'close_range'
      elif ttc < self.min_ttc_s:
        fail_open = 'short_ttc'
      elif timestamp_ns - entry.evidence_ns <= self.evidence_hold_ns and entry.evidence_ns:
        fail_open = 'recent_evidence'
      elif lateral_rate >= self.cutin_mps:
        fail_open = 'lateral_cutin'
      elif intent is None:
        fail_open = 'oem_intent_unavailable'
      elif intent <= self.neutral_mps2:
        fail_open = 'oem_decelerating'
      elif entry.tentative_scans < self.persist_scans:
        fail_open = 'warming'
      if fail_open is None:
        withhold.add(pid)
      entry.withheld = fail_open is None
      self.reasons[pid] = {
        'withhold': fail_open is None, 'reason': fail_open or 'tentative_oem_disagreement',
        'tentative_scans': entry.tentative_scans, 'in_path_scans': entry.in_path_scans,
        'd_rel': obj.d_rel, 'y_rel': obj.y_rel,
        'v_rel': obj.v_rel, 'ttc_s': ttc, 'lateral_rate_mps': lateral_rate,
        'oem_intent_mps2': intent, 'scc_obj_valid': oem_valid,
        'evidence_age_s': (timestamp_ns - entry.evidence_ns) * 1e-9 if entry.evidence_ns else None,
      }
    self._states = states
    self.state_peak = max(self.state_peak, len(states))
    self.would_withhold = frozenset(withhold)
    return self.would_withhold


BOSCH_PUBLICATION_ALIAS_START = 32
BOSCH_PUBLICATION_ALIAS_COUNT = 64
BOSCH_PUBLICATION_ALIAS_GRACE_S = 0.5


class BoschPublicationAliasAllocator:
  """Physical-ID bindings for the existing Hyundai front publication namespace.

  Tracking never reads these aliases. Unused aliases, then released aliases in
  FIFO order, are assigned independently of raw slots and representatives.
  """
  def __init__(self):
    self.physical_to_alias: dict[int, int] = {}
    self.alias_to_physical: dict[int, int] = {}
    self.last_published_ns: dict[int, int] = {}
    self.free_aliases = deque(range(BOSCH_PUBLICATION_ALIAS_START,
                                   BOSCH_PUBLICATION_ALIAS_START + BOSCH_PUBLICATION_ALIAS_COUNT))
    self.peak_usage = 0
    self.denial_count = 0
    self.grace_eviction_count = 0

  @property
  def current_usage(self):
    return len(self.physical_to_alias)

  def _release(self, physical_id):
    alias = self.physical_to_alias.pop(physical_id)
    del self.alias_to_physical[alias]
    del self.last_published_ns[physical_id]
    self.free_aliases.append(alias)

  def update(self, timestamp_ns, published_ids, live_ids) -> dict[int, int]:
    published = set(published_ids)
    # Bosch has at most 32 raw returns per scan, hence at most 32 physical
    # objects. Do not silently truncate or alias two objects if that invariant
    # is ever violated by a future producer using this 64-entry allocator.
    if len(published) > BOSCH_PUBLICATION_ALIAS_COUNT:
      self.denial_count += len(published) - BOSCH_PUBLICATION_ALIAS_COUNT
      raise ValueError('Bosch publication exceeds the front radar alias pool')
    grace_ns = int(BOSCH_PUBLICATION_ALIAS_GRACE_S * 1e9)
    for physical_id in list(self.physical_to_alias):
      if physical_id not in live_ids or timestamp_ns - self.last_published_ns[physical_id] > grace_ns:
        self._release(physical_id)

    needed = len(published.difference(self.physical_to_alias)) - len(self.free_aliases)
    if needed > 0:
      # Under pressure only unpublished grace bindings may be reclaimed;
      # every currently published object keeps its binding and is emitted.
      dormant = sorted(physical_id for physical_id in self.physical_to_alias if physical_id not in published)
      dormant.sort(key=self.last_published_ns.__getitem__)
      for physical_id in dormant[:needed]:
        self._release(physical_id)
        self.grace_eviction_count += 1

    for physical_id in sorted(published):
      if physical_id not in self.physical_to_alias:
        alias = self.free_aliases.popleft()
        self.physical_to_alias[physical_id] = alias
        self.alias_to_physical[alias] = physical_id
      self.last_published_ns[physical_id] = timestamp_ns
    self.peak_usage = max(self.peak_usage, self.current_usage)
    return {physical_id: self.physical_to_alias[physical_id] for physical_id in published}


@dataclass(frozen=True, slots=True)
class BoschB5ModelContext:
  publication_ns: int
  source_ns: int
  road_edges: tuple[tuple[tuple[float, float], ...], ...]
  road_edge_stds: tuple[float, ...]
  lane_lines: tuple[tuple[tuple[float, float], ...], ...]
  lane_probs: tuple[float, ...]
  lane_stds: tuple[float, ...]
  path: tuple[tuple[float, float], ...]


@dataclass(slots=True)
class _BoschB5ParentState:
  raw_member_set: tuple[int, ...]
  representative_raw_id: int
  stable_scans: int
  last_seen_ns: int
  lifecycle_birth_ns: int


@dataclass(frozen=True, slots=True)
class BoschB5Decision:
  timestamp_ns: int
  target_pid: int
  parent_pid: int | None
  parent_stable_scans: int
  target_vrel_code: int | None
  parent_vrel_code: int | None
  new_raw_count: int
  new_pid_count: int
  singleton: bool
  n4_eligible: bool
  decision: bool
  reason: str


class BoschBirthB5Defer:
  """Frozen B5 decision plus a one-publication birth defer.

  This state is downstream of raw association and physical grouping. Nothing
  here is read by either tracker, and publication_view never feeds its result
  back into this class or any existing Bosch filter.
  """

  def __init__(self, mode=BOSCH_B5_MODE):
    self.mode = mode
    self.states: dict[int, _BoschB5ParentState] = {}
    self.raw_prior_owners: dict[int, set[int]] = {}
    self.speed_samples: list[tuple[int, float]] = []
    self.pose_samples: list[tuple[int, float, bool]] = []
    self.model_contexts: list[BoschB5ModelContext] = []
    self.last_scan_ns: int | None = None
    self.last_decisions: tuple[BoschB5Decision, ...] = ()
    self.would_suppress = frozenset()
    self.last_suppressed: tuple[int, ...] = ()
    self._counted_suppression_ns: int | None = None
    self.suppressed_points = 0
    self.suppressed_scans = 0
    self.max_state_count = 0
    self.max_raw_owner_count = 0
    self.reset_count = 0
    self.last_reset_reason = 'CONSTRUCTION'

  def reset(self, reason='STATE_RESET'):
    self.states.clear()
    self.raw_prior_owners.clear()
    self.speed_samples.clear()
    self.pose_samples.clear()
    self.model_contexts.clear()
    self.last_scan_ns = None
    self.last_decisions = ()
    self.would_suppress = frozenset()
    self.last_suppressed = ()
    self._counted_suppression_ns = None
    self.reset_count += 1
    self.last_reset_reason = reason

  @staticmethod
  def _append_sample(samples, sample, *, horizon_ns=BOSCH_B5_MOTION_HISTORY_NS, maximum=64):
    timestamp_ns = sample[0]
    if samples and samples[-1] == sample:
      return
    if samples and timestamp_ns < samples[-1][0]:
      samples.clear()
    if samples and timestamp_ns == samples[-1][0]:
      samples[-1] = sample
    else:
      samples.append(sample)
    cutoff = timestamp_ns - horizon_ns
    first = bisect.bisect_left(samples, cutoff, key=_bosch_sample_ns)
    if first:
      del samples[:first]
    if len(samples) > maximum:
      del samples[:-maximum]

  def ingest_pose(self, timestamp_ns, yaw_rate_left):
    if not isinstance(timestamp_ns, int) or timestamp_ns <= 0:
      return
    valid = yaw_rate_left is not None and math.isfinite(yaw_rate_left)
    self._append_sample(self.pose_samples, (timestamp_ns, float(yaw_rate_left or 0.), valid))

  def ingest_speed(self, timestamp_ns, v_ego):
    if not isinstance(timestamp_ns, int) or timestamp_ns <= 0 or not math.isfinite(v_ego):
      return
    self._append_sample(self.speed_samples, (timestamp_ns, float(v_ego)))

  def ingest_model(self, model, publication_ns):
    if model is None or not isinstance(publication_ns, int) or publication_ns <= 0:
      return
    source_ns = int(getattr(model, 'timestampEof', 0) or 0)
    contexts = self.model_contexts
    if (contexts and publication_ns == contexts[-1].publication_ns and
        source_ns == contexts[-1].source_ns):
      return
    edges = getattr(model, 'roadEdges', ())
    lanes = getattr(model, 'laneLines', ())
    position = getattr(model, 'position', None)
    edge_stds = tuple(float(value) for value in getattr(model, 'roadEdgeStds', ()))
    lane_probs = tuple(float(value) for value in getattr(model, 'laneLineProbs', ()))
    lane_stds = tuple(float(value) for value in getattr(model, 'laneLineStds', ()))
    if (source_ns <= 0 or len(edges) != 2 or len(lanes) != 4 or position is None or
        len(edge_stds) != 2 or len(lane_probs) != 4 or len(lane_stds) != 4):
      return

    def points(polyline):
      xs, ys = polyline.x, polyline.y
      # A non-finite member makes the sum non-finite, so this path only skips
      # a filter that would have removed nothing.
      if math.isfinite(sum(xs) + sum(ys)):
        return tuple(zip(map(float, xs), map(float, ys), strict=False))
      return tuple((float(x), float(y)) for x, y in zip(xs, ys, strict=False)
                   if math.isfinite(x) and math.isfinite(y))

    context = BoschB5ModelContext(
      publication_ns, source_ns, tuple(points(edge) for edge in edges), edge_stds,
      tuple(points(lane) for lane in lanes), lane_probs, lane_stds, points(position))
    if any(len(polyline) < 2 for polyline in context.road_edges + context.lane_lines) or len(context.path) < 2:
      return
    if contexts and publication_ns < contexts[-1].publication_ns:
      contexts.clear()
    if contexts and publication_ns == contexts[-1].publication_ns:
      contexts[-1] = context
    else:
      contexts.append(context)
    if len(contexts) > 4:
      del contexts[:-4]

  @staticmethod
  def _interpolate(samples, timestamp_ns, value_index=1, *, times=None):
    if times is None:
      times = [row[0] for row in samples]
    index = bisect.bisect_left(times, timestamp_ns)
    if index <= 0:
      return samples[0][value_index]
    if index >= len(samples):
      return samples[-1][value_index]
    before, after = samples[index - 1], samples[index]
    if after[0] == before[0]:
      return before[value_index]
    weight = (timestamp_ns - before[0]) / (after[0] - before[0])
    return before[value_index] + weight * (after[value_index] - before[value_index])

  def _integrate_motion(self, start_ns, end_ns):
    speeds, poses = self.speed_samples, self.pose_samples
    if (end_ns < start_ns or not speeds or not poses or
        speeds[0][0] > start_ns or poses[0][0] > start_ns):
      return None
    x = y = heading = 0.
    timestamp_ns = start_ns
    pose_times = [row[0] for row in poses]
    speed_times = [row[0] for row in speeds]
    while timestamp_ns < end_ns:
      next_ns = min(timestamp_ns + 10_000_000, end_ns)
      middle_ns = (timestamp_ns + next_ns) // 2
      pose_index = bisect.bisect_right(pose_times, middle_ns) - 1
      if (pose_index < 0 or middle_ns - poses[pose_index][0] > BOSCH_B5_CONTEXT_MAX_AGE_NS or
          not poses[pose_index][2]):
        return None
      speed = self._interpolate(speeds, middle_ns, times=speed_times)
      yaw = self._interpolate(poses, middle_ns, times=pose_times)
      if not math.isfinite(speed) or not math.isfinite(yaw):
        return None
      dt = (next_ns - timestamp_ns) * 1e-9
      middle_heading = heading + .5 * yaw * dt
      x += speed * math.cos(middle_heading) * dt
      y += speed * math.sin(middle_heading) * dt
      heading += yaw * dt
      timestamp_ns = next_ns
    return x, y, heading

  @staticmethod
  def _transform(points, dx, dy, heading, trig=None):
    cosine, sine = (math.cos(heading), math.sin(heading)) if trig is None else trig
    neg_sine = -sine
    return tuple(sorted([(cosine * (x := model_x - BOSCH_B5_RADAR_TO_DEVICE_X_M - dx) + sine * (y := -model_y - dy),
                          neg_sine * x + cosine * y) for model_x, model_y in points]))

  @staticmethod
  def _interp(points, x):
    if len(points) < 2 or x < points[0][0] or x > points[-1][0]:
      return math.nan
    # Search the sorted geometry directly instead of allocating its x list
    # for every edge/lane interpolation.
    index = bisect.bisect_left(points, x, key=_bosch_sample_ns)
    if index <= 0:
      return points[0][1]
    if index >= len(points):
      return points[-1][1]
    x0, y0 = points[index - 1]
    x1, y1 = points[index]
    return .5 * (y0 + y1) if x1 == x0 else y0 + (x - x0) / (x1 - x0) * (y1 - y0)

  def _geometry(self, context, scan_ns):
    if context.source_ns > scan_ns or scan_ns - context.source_ns > BOSCH_B5_CONTEXT_MAX_AGE_NS:
      return None
    motion = self._integrate_motion(context.source_ns, scan_ns)
    if motion is None:
      return None
    dx, dy, heading = motion
    trig = (math.cos(heading), math.sin(heading))
    return {
      'edges': tuple(self._transform(edge, dx, dy, heading, trig) for edge in context.road_edges),
      'lanes': tuple(self._transform(lane, dx, dy, heading, trig) for lane in context.lane_lines),
      'path': self._transform(context.path, dx, dy, heading, trig),
      'edge_stds': context.road_edge_stds, 'lane_probs': context.lane_probs,
      'lane_stds': context.lane_stds, 'source_ns': context.source_ns,
    }

  def _latest_context(self, scan_ns, now_ns):
    available = [context for context in self.model_contexts if context.publication_ns <= now_ns]
    if not available:
      return None
    latest = available[-1]
    if now_ns - latest.publication_ns > BOSCH_B5_CONTEXT_MAX_AGE_NS:
      return None
    if latest.source_ns > scan_ns or scan_ns - latest.source_ns > BOSCH_B5_CONTEXT_MAX_AGE_NS:
      return None
    return latest

  def _align(self, obj, geometry):
    d_rel, y_rel = obj.d_rel, obj.y_rel
    left = self._interp(geometry['edges'][0], d_rel)
    right = self._interp(geometry['edges'][1], d_rel)
    if not math.isfinite(left) or not math.isfinite(right) or left <= right:
      return None
    path_y = self._interp(geometry['path'], d_rel)
    is_left = y_rel >= 0.
    edge_y = left if is_left else right
    outside = y_rel - left if is_left else right - y_rel
    return {
      'outside': outside, 'left': left, 'right': right, 'width': left - right,
      'path_inside': math.isfinite(path_y) and right <= path_y <= left,
      'edge_std': geometry['edge_stds'][0 if is_left else 1], 'edge_y': edge_y,
    }

  def _instant_lane(self, obj, geometry):
    d_rel, y_rel = obj.d_rel, obj.y_rel
    left = self._interp(geometry['edges'][0], d_rel)
    right = self._interp(geometry['edges'][1], d_rel)
    lane_y = [self._interp(line, d_rel) for line in geometry['lanes']]
    if (not math.isfinite(left) or not math.isfinite(right) or left <= right or
        len(lane_y) != 4 or not all(math.isfinite(value) for value in lane_y)):
      return {'available': False}
    reliable = [geometry['lane_probs'][index] >= BOSCH_B5_LANE_PROB_MIN and
                geometry['lane_stds'][index] <= BOSCH_B5_LANE_STD_MAX_M for index in range(4)]
    sign = 1. if y_rel >= 0. else -1.
    edge_y = left if sign > 0. else right
    entries = sorted(((value, index) for index, value in enumerate(lane_y)), reverse=True)
    pair = any(
      reliable[high_i] and reliable[low_i] and low <= y_rel <= high and
      BOSCH_B5_CORRIDOR_WIDTH_MIN_M <= high - low <= BOSCH_B5_CORRIDOR_WIDTH_MAX_M and
      sign * (.5 * (high + low) - edge_y) >= 0.
      for (high, high_i), (low, low_i) in zip(entries, entries[1:], strict=False))
    band = any(
      reliable[index] and BOSCH_B5_CORRIDOR_WIDTH_MIN_M <= sign * (value - edge_y) <= BOSCH_B5_EDGE_LANE_BAND_MAX_M and
      sign * (value - y_rel) >= 0.
      for index, value in enumerate(lane_y))
    diverging = False
    inside_lines = [(sign * (edge_y - value), index) for index, value in enumerate(lane_y)
                    if reliable[index] and sign * (edge_y - value) >= 0.]
    if inside_lines:
      target_gap, lane_index = min(inside_lines)
      lane, edge = geometry['lanes'][lane_index], geometry['edges'][0 if sign > 0. else 1]
      minimum_x = max(lane[0][0], edge[0][0])
      anchor_x = max(minimum_x, d_rel - 20.)
      if d_rel - anchor_x >= 10.:
        lane_anchor, edge_anchor = self._interp(lane, anchor_x), self._interp(edge, anchor_x)
        if math.isfinite(lane_anchor) and math.isfinite(edge_anchor):
          anchor_gap = sign * (edge_anchor - lane_anchor)
          diverging = target_gap >= 1. and target_gap - anchor_gap >= BOSCH_B5_DIVERGENCE_MIN_M
    return {'available': True, 'lane_y': tuple(lane_y), 'reliable_count': sum(reliable),
            'evidence': bool(pair or band or diverging)}

  @staticmethod
  def _population_std(values):
    if len(values) < 2:
      return math.nan
    mean = sum(values) / len(values)
    return math.sqrt(sum((value - mean) ** 2 for value in values) / len(values))

  def _scan_geometry(self, context, scan_ns, scan_cache):
    if scan_cache is None:
      return self._geometry(context, scan_ns)
    geometries = scan_cache.setdefault('geometries', {})
    key = id(context)
    if key not in geometries:
      geometries[key] = self._geometry(context, scan_ns)
    return geometries[key]

  def _corridor_clear(self, obj, scan_ns, now_ns, scan_cache=None):
    latest = self._latest_context(scan_ns, now_ns)
    if latest is None:
      return False
    samples = []
    for context in self.model_contexts:
      if (context.publication_ns > now_ns or context.source_ns > scan_ns or
          context.source_ns < scan_ns - BOSCH_B5_HISTORY_NS):
        continue
      geometry = self._scan_geometry(context, scan_ns, scan_cache)
      if geometry is None:
        continue
      instant = self._instant_lane(obj, geometry)
      instant['source_ns'] = context.source_ns
      samples.append(instant)
    samples.sort(key=lambda row: row['source_ns'])
    valid = [row for row in samples if row['available']]
    if len(valid) < 2 or valid[-1]['source_ns'] - valid[0]['source_ns'] < BOSCH_B5_HISTORY_MIN_SPAN_NS:
      return False
    line_stds = []
    for index in range(4):
      values = [row['lane_y'][index] for row in valid]
      std = self._population_std(values)
      if math.isfinite(std):
        line_stds.append(std)
    if (not line_stds or max(line_stds) > BOSCH_B5_LANE_STABILITY_MAX_M or
        min(row['reliable_count'] for row in valid[-2:]) < 2):
      return False
    return not all(row['evidence'] for row in valid[-2:])

  @staticmethod
  def _representative_code(obj):
    member = next((member for member in obj.members
                   if member.raw_track_id == obj.representative_raw_track_id), None)
    return None if member is None else (member.detection.raw_word >> 21) & 0x3ff

  def _n4_parent(self, target, objects, scan_ns, now_ns, v_ego, yaw_rate_left,
                 camera_associations, oem_state, word0_pids, prior_raw_owners, scan_cache=None):
    if not math.isfinite(v_ego) or v_ego < BOSCH_B5_MIN_VEGO_MPS:
      return None, 'LOW_SPEED_FAIL_OPEN'
    if yaw_rate_left is None or not math.isfinite(yaw_rate_left):
      return None, 'POSE_UNAVAILABLE_FAIL_OPEN'
    if abs(yaw_rate_left / max(v_ego, .1)) > BOSCH_B5_MAX_CURVATURE_1PM:
      return None, 'HIGH_CURVATURE_FAIL_OPEN'
    context = self._latest_context(scan_ns, now_ns)
    if context is None:
      return None, 'MODEL_STALE_FAIL_OPEN'
    geometry = self._scan_geometry(context, scan_ns, scan_cache)
    if geometry is None:
      return None, 'POSE_UNAVAILABLE_FAIL_OPEN'
    aligned = scan_cache.get('aligned') if scan_cache is not None else None
    if aligned is None:
      aligned = {obj.physical_track_id: self._align(obj, geometry) for obj in objects}
      if scan_cache is not None:
        scan_cache['aligned'] = aligned
    target_alignment = aligned.get(target.physical_track_id)
    if (target_alignment is None or not target_alignment['path_inside'] or
        not BOSCH_B5_EDGE_WIDTH_MIN_M <= target_alignment['width'] <= BOSCH_B5_EDGE_WIDTH_MAX_M or
        not math.isfinite(target_alignment['edge_std']) or
        target_alignment['edge_std'] > BOSCH_B5_EDGE_STD_MAX_M):
      return None, 'ROAD_EDGE_UNSTABLE_FAIL_OPEN'
    if target_alignment['outside'] < BOSCH_B5_OUTSIDE_MARGIN_M:
      return None, 'OUTSIDE_MARGIN_FAIL_OPEN'
    target_world = target.v_rel + v_ego - yaw_rate_left * target.y_rel
    if target_world <= BOSCH_B5_MIN_WORLD_SPEED_MPS:
      return None, 'TARGET_DIRECTION_FAIL_OPEN'
    if not self._corridor_clear(target, scan_ns, now_ns, scan_cache):
      return None, 'CORRIDOR_UNKNOWN_OR_POSITIVE_FAIL_OPEN'

    candidates = []
    for parent in objects:
      if parent.physical_track_id == target.physical_track_id:
        continue
      parent_alignment = aligned.get(parent.physical_track_id)
      parent_world = parent.v_rel + v_ego - yaw_rate_left * parent.y_rel
      if (parent_alignment is None or parent_alignment['outside'] > 0. or
          parent.age_scans < BOSCH_B5_PARENT_MIN_AGE_SCANS or
          parent_world <= BOSCH_B5_MIN_WORLD_SPEED_MPS):
        continue
      if (abs(target.d_rel - parent.d_rel) <= BOSCH_B5_MAX_D_M and
          abs(target.y_rel - parent.y_rel) <= BOSCH_B5_MAX_Y_M and
          abs(target.v_rel - parent.v_rel) <= BOSCH_B5_MAX_DV_MPS and
          abs(target_world - parent_world) <= BOSCH_B5_MAX_DWORLD_MPS):
        score = (abs(target_world - parent_world) + abs(target.d_rel - parent.d_rel) / BOSCH_B5_MAX_D_M +
                 abs(target.y_rel - parent.y_rel) / BOSCH_B5_MAX_Y_M)
        candidates.append((score, parent.physical_track_id, parent))
    if len(candidates) != 1:
      return None, 'AMBIGUOUS_PARENT_FAIL_OPEN' if candidates else 'NO_PARENT_FAIL_OPEN'
    parent = candidates[0][2]
    target_assoc = camera_associations.get(target.physical_track_id)
    parent_assoc = camera_associations.get(parent.physical_track_id)
    if (target_assoc is not None and target_assoc[0] == BOSCH_CAMERA_ASSOC_ASSIGNED and target_assoc[1] >= 0 and
        (parent_assoc is None or parent_assoc[0] != BOSCH_CAMERA_ASSOC_ASSIGNED or
         parent_assoc[1] != target_assoc[1])):
      return None, 'INDEPENDENT_CAMERA_FAIL_OPEN'
    if oem_state == BOSCH_OEM_STATE_VALIDATED and target.physical_track_id in word0_pids:
      return None, 'INDEPENDENT_OEM_FAIL_OPEN'
    target_raws = {member.raw_track_id for member in target.members}
    if any(owner != parent.physical_track_id for raw_id in target_raws
           for owner in prior_raw_owners.get(raw_id, ())):
      return None, 'INDEPENDENT_RAW_FAIL_OPEN'
    return parent, 'N4_FROZEN'

  def update(self, objects, raw_tracks, timestamp_ns, now_ns, v_ego, yaw_rate_left,
             *, camera_associations=(), oem_state=BOSCH_OEM_STATE_NONE,
             word0_pids=(), live_pids=(), live_raw_ids=()):
    if self.mode == BOSCH_B5_OFF:
      self.last_decisions = ()
      self.would_suppress = frozenset()
      return self.would_suppress
    if self.last_scan_ns is not None:
      if timestamp_ns <= self.last_scan_ns:
        self.reset('CLOCK_RESET')
      elif timestamp_ns - self.last_scan_ns >= BOSCH_STALE_NS:
        self.reset('RADAR_INPUT_GAP')
    self.last_scan_ns = timestamp_ns
    live_pids = set(live_pids)
    live_raw_ids = set(live_raw_ids)
    for pid in tuple(self.states):
      if pid not in live_pids:
        del self.states[pid]
    for raw_id in tuple(self.raw_prior_owners):
      if raw_id not in live_raw_ids:
        del self.raw_prior_owners[raw_id]
    prior_raw_owners = {raw_id: frozenset(owners) for raw_id, owners in self.raw_prior_owners.items()}
    for obj in objects:
      pid = obj.physical_track_id
      structure = tuple(sorted(member.raw_track_id for member in obj.members))
      previous = self.states.get(pid)
      if (previous is None or obj.age_scans == 1 or previous.raw_member_set != structure or
          previous.representative_raw_id != obj.representative_raw_track_id):
        birth_ns = timestamp_ns if previous is None or obj.age_scans == 1 else previous.lifecycle_birth_ns
        self.states[pid] = _BoschB5ParentState(
          structure, obj.representative_raw_track_id, 1, timestamp_ns, birth_ns)
      else:
        previous.stable_scans = min(BOSCH_B5_COUNTER_MAX, previous.stable_scans + 1)
        previous.last_seen_ns = timestamp_ns
    self.max_state_count = max(self.max_state_count, len(self.states))

    new_raw_count = sum(track.age_scans == 1 for track in raw_tracks)
    newborns = [obj for obj in objects if obj.age_scans == 1]
    new_pid_count = len(newborns)
    camera_associations = dict(camera_associations)
    word0_pids = frozenset(word0_pids)
    decisions = []
    suppressed = set()
    # Contexts, motion samples, and objects are fixed throughout this update.
    # Keep reuse local: even a repeated timestamp must read fresh input state.
    scan_cache = {}
    for target in newborns:
      singleton = len(target.members) == 1
      parent, reason = self._n4_parent(
        target, objects, timestamp_ns, now_ns, v_ego, yaw_rate_left,
        camera_associations, oem_state, word0_pids, prior_raw_owners, scan_cache)
      n4 = parent is not None
      parent_state = self.states.get(parent.physical_track_id) if parent is not None else None
      stable_scans = parent_state.stable_scans if parent_state is not None else 0
      target_code = self._representative_code(target)
      parent_code = self._representative_code(parent) if parent is not None else None
      if not n4:
        decision = False
      elif not singleton:
        decision, reason = False, 'NON_SINGLETON_FAIL_OPEN'
      elif target_code is None or parent_code is None or target_code != parent_code:
        decision, reason = False, 'VREL_NOT_EXACT_FAIL_OPEN'
      elif new_raw_count != 1:
        decision, reason = False, 'NEW_RAW_COUNT_FAIL_OPEN'
      elif new_pid_count != 1:
        decision, reason = False, 'NEW_PID_COUNT_FAIL_OPEN'
      elif stable_scans < BOSCH_B5_PARENT_STABILITY_SCANS:
        decision, reason = False, 'PARENT_NOT_STABLE_FAIL_OPEN'
      else:
        decision, reason = True, 'BIRTH_B5_ACTIVE_DEFER'
        suppressed.add(target.physical_track_id)
      decisions.append(BoschB5Decision(
        timestamp_ns, target.physical_track_id,
        parent.physical_track_id if parent is not None else None,
        stable_scans, target_code, parent_code, new_raw_count, new_pid_count,
        singleton, n4, decision, reason))

    for obj in objects:
      for member in obj.members:
        self.raw_prior_owners.setdefault(member.raw_track_id, set()).add(obj.physical_track_id)
    self.max_raw_owner_count = max(self.max_raw_owner_count, len(self.raw_prior_owners))
    self.last_decisions = tuple(decisions)
    self.would_suppress = frozenset(suppressed)
    self.last_suppressed = ()
    return self.would_suppress

  def publication_view(self, objects):
    if (self.mode != BOSCH_B5_ACTIVE or not objects or not self.would_suppress or
        not all(obj.timestamp_ns == self.last_scan_ns for obj in objects)):
      self.last_suppressed = ()
      return objects
    suppressed = tuple(sorted(obj.physical_track_id for obj in objects
                              if obj.physical_track_id in self.would_suppress))
    self.last_suppressed = suppressed
    if not suppressed:
      return objects
    if self._counted_suppression_ns != self.last_scan_ns:
      self.suppressed_points += len(suppressed)
      self.suppressed_scans += 1
      self._counted_suppression_ns = self.last_scan_ns
    hidden = set(suppressed)
    return tuple(obj for obj in objects if obj.physical_track_id not in hidden)


def bosch_published_surface(obj):
  """(raw_track_id, d_rel, y_rel, v_rel) that reaches RadarData for `obj`.

  Without a surface the continuity anchor is published, which is the state of
  every object the tracker emits.
  """
  surface = obj.published_surface
  if surface is None:
    return obj.representative_raw_track_id, obj.d_rel, obj.y_rel, obj.v_rel
  return surface.raw_track_id, surface.d_rel, surface.y_rel, surface.v_rel


@dataclass(slots=True)
class BoschLeadAccelerationState:
  v_lead_filtered: float
  lead_filter: RadarLeadFilter
  last_scan_ns: int
  update_count: int
  a_lead: float

  def update(self, v_lead):
    self.v_lead_filtered = .5 * self.v_lead_filtered + .5 * v_lead
    stationary = abs(self.v_lead_filtered) < .3 and abs(v_lead - self.v_lead_filtered) < .05
    return self.lead_filter.update(v_lead, stationary=stationary)


class BoschLeadAccelerationEstimator:
  """Physical-PID-owned software causal lead acceleration estimate.

  This is not Bosch-native decoded acceleration. State updates happen only on
  fresh completed Bosch scans; held publications reuse the stored estimate.
  """
  def __init__(self):
    self.states = {}
    self._a_lead_by_pid = {}
    self.last_scan_ns = None
    self.reset_count = 0
    self.last_reset_reason = 'INITIAL'
    self.last_publication_kind = 'INITIAL'
    self.expired_count = 0
    self.last_expiry_reason = 'NONE'
    self.update_count = 0
    self.peak_state_count = 0
    self.allocation_count = 0
    self.held_publication_count = 0
    self.held_update_count = 0
    self.invalid_input_count = 0
    self.corrupt_state_count = 0
    self.clock_reset_count = 0
    self.gap_reset_count = 0

  def reset(self, reason='STATE_RESET'):
    self.states.clear()
    self._a_lead_by_pid.clear()
    self.last_scan_ns = None
    self.reset_count += 1
    self.last_reset_reason = reason
    self.last_publication_kind = 'RESET'

  @staticmethod
  def _v_lead(obj, v_ego):
    surface = obj.published_surface
    return v_ego + (obj.v_rel if surface is None else surface.v_rel)

  def update(self, objects, scan_ns, v_ego, live_pids):
    if scan_ns is None:
      self.last_publication_kind = 'NO_SCAN'
      return {}
    if isinstance(scan_ns, bool) or not isinstance(scan_ns, Integral) or scan_ns < 0:
      self.reset('INVALID_SCAN_TIMESTAMP')
      self.invalid_input_count += len(objects)
      return {}
    publication_kind = 'HELD' if scan_ns == self.last_scan_ns else 'FRESH'
    if publication_kind == 'HELD':
      self.last_publication_kind = publication_kind
      self.held_publication_count += 1
      return self._a_lead_by_pid
    if self.last_scan_ns is not None:
      if scan_ns < self.last_scan_ns:
        self.clock_reset_count += 1
        self.reset('CLOCK_RESET')
      elif scan_ns > self.last_scan_ns and scan_ns - self.last_scan_ns >= BOSCH_STALE_NS:
        self.gap_reset_count += 1
        self.reset('SCAN_GAP')
    current = objects
    result = self._a_lead_by_pid
    result.clear()
    for obj in current:
      pid = obj.physical_track_id
      state = self.states.get(pid)
      if state is not None and not isinstance(state, BoschLeadAccelerationState):
        del self.states[pid]
        state = None
        result[pid] = math.nan
        self.corrupt_state_count += 1
        continue
      if state is None or state.last_scan_ns != scan_ns:
        v_lead = self._v_lead(obj, v_ego)
        if not math.isfinite(v_lead):
          if state is not None:
            del self.states[pid]
          result[pid] = math.nan
          self.invalid_input_count += 1
          continue
        if state is None:
          state = BoschLeadAccelerationState(
            float(v_lead), RadarLeadFilter(float(v_lead), BOSCH_ALEAD_SAMPLE_PERIOD_S),
            scan_ns, 1, 0.0)
          self.allocation_count += 1
        else:
          filtered = state.update(v_lead)
          if not math.isfinite(filtered):
            del self.states[pid]
            result[pid] = math.nan
            self.corrupt_state_count += 1
            continue
          state.last_scan_ns = scan_ns
          state.update_count += 1
          state.a_lead = float(filtered) if state.update_count >= 6 else 0.0
        self.states[pid] = state
        self.update_count += 1
      result[pid] = state.a_lead
    for pid in tuple(self.states):
      if pid not in live_pids and pid not in result:
        del self.states[pid]
        self.expired_count += 1
        self.last_expiry_reason = 'PID_DEATH'
    if len(self.states) > BOSCH_ALEAD_STATE_MAX:
      victims = sorted((pid in result, state.last_scan_ns, pid) for pid, state in self.states.items())
      for _, _, pid in victims[:len(self.states) - BOSCH_ALEAD_STATE_MAX]:
        del self.states[pid]
        if pid in result:
          result[pid] = math.nan
        self.expired_count += 1
        self.last_expiry_reason = 'STATE_CAP'
    self.last_scan_ns = scan_ns
    self.last_publication_kind = publication_kind
    self.peak_state_count = max(self.peak_state_count, len(self.states))
    return result

  def debug_snapshot(self, now_ns):
    return {
      'last_scan_ns': self.last_scan_ns, 'reset_count': self.reset_count,
      'last_reset_reason': self.last_reset_reason,
      'last_publication_kind': self.last_publication_kind,
      'expired_count': self.expired_count, 'last_expiry_reason': self.last_expiry_reason,
      'update_count': self.update_count, 'state_count': len(self.states),
      'peak_state_count': self.peak_state_count,
      'allocation_count': self.allocation_count,
      'held_publication_count': self.held_publication_count,
      'held_update_count': self.held_update_count,
      'invalid_input_count': self.invalid_input_count,
      'corrupt_state_count': self.corrupt_state_count,
      'clock_reset_count': self.clock_reset_count, 'gap_reset_count': self.gap_reset_count,
      'states': [{
        'physical_pid': pid, 'source_scan_timestamp_ns': state.last_scan_ns,
        'estimator_update_count': state.update_count,
        'estimated_aLead': state.a_lead,
        'state_age_ns': max(0, now_ns - state.last_scan_ns),
      } for pid, state in sorted(self.states.items())],
    }


_BOSCH_POINT_FIELD_NAMES = ('trackId', 'dRel', 'yRel', 'vRel', 'aRel', 'yvRel', 'jLead', 'aLead', 'vLead',
                            'radarSource', 'trackState', 'measured')
# (schema, fields) of the last RadarPoint schema written. Every `.schema` access
# builds a new wrapper whose `fields` dict refers back to it; radarcan runs with
# gc disabled, so reading `fields` per publication leaked that cycle (2.5 kB, 185 MB/h
# on device). Fields are read once per loaded schema; a field of another loaded
# schema would be rejected by the setter, hence the schema comparison.
_bosch_point_fields_cache = None


def _bosch_point_fields(point):
  global _bosch_point_fields_cache
  schema = point.schema
  cache = _bosch_point_fields_cache
  if cache is None or not cache[0] == schema:
    fields = schema.fields
    cache = _bosch_point_fields_cache = (schema, tuple(fields[name] for name in _BOSCH_POINT_FIELD_NAMES))
  return cache[1]


def bosch_fill_point(point, obj, v_ego, alias=None, a_lead=math.nan):
  # bosch_append_points repeats these writes through cached fields; keep both equal.
  point.trackId = obj.physical_track_id if alias is None else alias[obj.physical_track_id]
  _, d_rel, y_rel, v_rel = bosch_published_surface(obj)
  point.dRel, point.yRel, point.vRel = d_rel, y_rel, v_rel
  point.aRel = point.yvRel = point.jLead = math.nan
  point.aLead = a_lead
  point.vLead = v_ego + v_rel
  point.radarSource = 'frontRadar'
  point.trackState = 0
  point.measured = True


_BOSCH_FLOAT32 = struct.Struct('<f')


def _bosch_float32(value):
  return _BOSCH_FLOAT32.unpack(_BOSCH_FLOAT32.pack(value))[0]


def bosch_native_surface_coordinates(obj, now_ns):
  """Coordinates after the same Float32 writes used by bosch_append_points."""
  raw_id, d_rel, y_rel, v_rel = bosch_published_surface(obj)
  representative = obj.members[0] if len(obj.members) == 1 else next(
    member for member in obj.members if member.raw_track_id == raw_id)
  f32 = _bosch_float32
  d_rel, y_rel, v_rel = f32(d_rel), f32(y_rel), f32(v_rel)
  return f32(d_rel + v_rel * ((now_ns - representative.timestamp_ns) * 1e-9)), y_rel, v_rel


def bosch_append_points(radar, objects, v_ego, now_ns, alias=None, a_lead_by_pid=None):
  """Append directly to the final native list, retaining SCC aliasing safety."""
  if not objects:
    return
  # Independent struct copies survive re-initializing the list, like the dicts did.
  previous = [point.copy() for point in radar.points]
  offset = len(previous)
  points = radar.init('points', offset + len(objects))
  for index, values in enumerate(previous):
    points[index] = values
  # The same writes as bosch_fill_point, through cached schema fields instead of
  # an attribute lookup per field and point.
  (f_track, f_d, f_y, f_v, f_a_rel, f_yv, f_jerk, f_a_lead, f_v_lead, f_source, f_state,
   f_measured) = _bosch_point_fields(points[offset])
  nan = math.nan
  for index, obj in enumerate(objects, offset):
    point = points[index]
    put, get = point._set_by_field, point._get_by_field
    a_lead = nan if a_lead_by_pid is None else a_lead_by_pid.get(obj.physical_track_id, nan)
    published_id, d_rel, y_rel, v_rel = bosch_published_surface(obj)
    put(f_track, obj.physical_track_id if alias is None else alias[obj.physical_track_id])
    put(f_d, d_rel)
    put(f_y, y_rel)
    put(f_v, v_rel)
    put(f_a_rel, nan)
    put(f_yv, nan)
    put(f_jerk, nan)
    put(f_a_lead, a_lead)
    put(f_v_lead, v_ego + v_rel)
    put(f_source, 'frontRadar')
    put(f_state, 0)
    put(f_measured, True)
    members = obj.members
    if len(members) == 1:
      representative = members[0]
    else:
      # The age the published range is extrapolated over is the surface's own
      # measurement age, not the anchor's.
      representative = next(member for member in members
                            if member.raw_track_id == published_id)
    # Read the native Float32 fields before projection, exactly as the original
    # temporary RadarData path did. This also preserves rounding for test inputs.
    age_s = (now_ns - representative.timestamp_ns) * 1e-9
    put(f_d, get(f_d) + get(f_v) * age_s)


def bosch_to_native_radar_data(objects: Sequence[BoschPhysicalObject], timestamp_ns: int, *, v_ego=math.nan,
            data_type=None, complete=True, unsupported=False):
  """Serialize fresh physical hypotheses. Unknown native fields remain unknown.

  trackState is an integer in the existing schema, so its default 0 is used
  as UNKNOWN (not Bosch moving/confirmed state). measured is receipt only.
  No coasting state is ever serialized. No message is published by this API.
  """
  if data_type is None:
    from opendbc.car import structs
    data_type = structs.RadarData
  if any(obj.timestamp_ns != timestamp_ns for obj in objects):
    raise ValueError('stale physical observation cannot be serialized as fresh')
  if len({obj.physical_track_id for obj in objects}) != len(objects):
    raise ValueError('duplicate physical ID')
  radar = data_type.new_message()
  radar.errors.canError = not complete
  radar.errors.wrongConfig = unsupported
  for p, obj in zip(radar.init('points', len(objects)), objects):
    bosch_fill_point(p, obj, v_ego)
  return radar


@dataclass(frozen=True)
class _BoschCanFrame:
  timestamp_ns: int
  address: int
  payload: bytes
  order: int


def bosch_decode_frame(timestamp_ns: int, address: int, payload: bytes):
  """Decode two independent 32-bit records; 0x601 is never a detection."""
  if address not in BOSCH_TRACK_ADDRESSES or len(payload) != 8:
    raise ValueError('expected an eight-byte Bosch raw track frame')
  # One little-endian read covers both records: in that order the first word is
  # the low half of the payload, so no slice is copied to reach it.
  pair = int.from_bytes(payload, 'little')
  slot = (address - 0x602) * 2
  detections = []
  unsupported = False
  for half in range(2):
    word = (pair >> (half * 32)) & 0xFFFFFFFF
    if word == BOSCH_INACTIVE_WORD:
      continue
    if word & (1 << 31):
      unsupported = True
      continue
    detections.append(BoschRawDetection(timestamp_ns, slot + half,
                                   (word & 0x3FF) * .25,
                                   ((word >> 10) & 0x7FF) * .03125 - 32,
                                   ((word >> 21) & 0x3FF) * .25 - 128, word))
  return tuple(detections), unsupported


def _bosch_match_processed_target_word(objects, word):
  """Conservatively map 0x601 bytes0..3 decoded geometry to physical PIDs.

  The record is not assumed to be a raw-track copy or assigned a protocol
  name. Ambiguous near matches all count as fail-open support.
  """
  if word is None or word == BOSCH_INACTIVE_WORD or word & (1 << 31):
    return frozenset()
  d_rel = (word & 0x3FF) * .25
  y_rel = ((word >> 10) & 0x7FF) * .03125 - 32
  v_rel = ((word >> 21) & 0x3FF) * .25 - 128
  return frozenset(obj.physical_track_id for obj in objects
                   if abs(obj.d_rel - d_rel) <= 1.0 and abs(obj.y_rel - y_rel) <= .5 and
                   abs(obj.v_rel - v_rel) <= .5)


def bosch_make_points(objects, v_ego=math.nan):
  """Native physical IDs and original observations; unknown fields stay NaN."""
  if not objects:
    return []
  data = bosch_to_native_radar_data(objects, objects[0].timestamp_ns, v_ego=v_ego)
  return list(data.points)


@dataclass(frozen=True)
class BoschMirrorBirthDecision:
  physical_track_id: int
  action: str
  reason: str
  parent_pid: int | None = None
  wall_y_m: float | None = None
  residual_m: float | None = None
  count: int = 0


@dataclass
class _BoschMirrorBirthHoldState:
  physical_track_id: int
  parent_pid: int
  wall_y_m: float
  residual_m: float
  d_rel_m: float
  y_rel_m: float
  v_rel_mps: float
  birth_ns: int
  held_scans: int = 1
  parent_miss_scans: int = 0
  diverge_scans: int = 0


class BoschMirrorBirthHold:
  """Frozen wall-mirror birth decision with a bounded publication-only hold.

  The v1 shadow runs on the same physical objects and raw observations as the
  provider scan. Its result can start a hold only on the first qualified scan
  for a PID. Holds never feed the tracker, grouping, camera/OEM, aliases or
  any other provider filter.
  """

  def __init__(self, mode=BOSCH_MIRROR_BIRTH_MODE):
    if mode not in (BOSCH_MIRROR_BIRTH_OFF, BOSCH_MIRROR_BIRTH_SHADOW,
                    BOSCH_MIRROR_BIRTH_ACTIVE):
      raise ValueError('invalid Bosch mirror-birth mode')
    self.mode = mode
    self.hist: list[tuple[int, list[tuple[float, float, int]]]] = []
    self.holds: dict[int, _BoschMirrorBirthHoldState] = {}
    self.seen_births: set[int] = set()
    self.last_ns: int | None = None
    self.last_decisions: tuple[BoschMirrorBirthDecision, ...] = ()
    self.last_mirror_objects: dict[int, tuple] = {}
    self.last_decisions_by_pid: dict[int, BoschMirrorBirthDecision] = {}
    self.last_events: tuple[dict, ...] = ()
    self.would_suppress = frozenset()
    self.reset_count = 0
    self.hold_count = 0
    self.release_count = 0
    self.publication_suppressed = 0
    self.peak_history_points = 0
    self.peak_hold_count = 0

  def _reset_shadow(self):
    self.hist = []
    self.would_suppress = frozenset()
    self.reset_count += 1

  def _record_event(self, state, action, reason, timestamp_ns):
    fields = dict(
      action=action, ns=timestamp_ns, pid=state.physical_track_id,
      parent=state.parent_pid, wall_y=state.wall_y_m, resid=state.residual_m,
      d=state.d_rel_m, y=state.y_rel_m, v=state.v_rel_mps,
      reason=reason, held_scans=state.held_scans)
    self._pending_events.append(fields)

  def reset(self, timestamp_ns=None, reason='STATE_RESET'):
    """Release all holds and clear both hold and v1 history state."""
    self._pending_events = []
    release_ns = self.last_ns if timestamp_ns is None else timestamp_ns
    for state in tuple(self.holds.values()):
      self._record_event(state, 'RELEASE', reason, release_ns)
      self.release_count += 1
    self.holds.clear()
    self.seen_births.clear()
    self._reset_shadow()
    self.last_ns = None
    self.last_decisions = ()
    self.last_mirror_objects = {}
    self.last_decisions_by_pid = {}
    self.last_events = tuple(self._pending_events)
    self.would_suppress = frozenset()
    del self._pending_events

  def _wall(self, side, y_lo, y_hi, x_r_of, excluded):
    pts = [(x, y) for _timestamp_ns, scan_points in self.hist
           for x, y, raw_id in scan_points
           if raw_id not in excluded and y_lo < side * y < y_hi]
    if not pts:
      return None, 'NO_WALL_SUPPORT', 0, 0.0
    pts.sort(key=lambda point: side * point[1])
    best_fail = ('NO_WALL_SUPPORT', 0, 0.0)
    tried = set()
    for x0, y0 in pts:
      center = side * y0 + BOSCH_MIRROR_BIRTH_BAND_HALF_M
      key = round(center, 1)
      if key in tried:
        continue
      tried.add(key)
      band = [(x, y) for x, y in pts
              if abs(side * y - center) <= BOSCH_MIRROR_BIRTH_BAND_HALF_M + 1e-9]
      if not band:
        continue
      wall_y = sorted(y for _x, y in band)[len(band) // 2]
      x_r = x_r_of(wall_y)
      if not math.isfinite(x_r):
        continue
      window = [x for x, _y in band if abs(x - x_r) <= BOSCH_MIRROR_BIRTH_COVER_HALF_M]
      bin_count = int(2 * BOSCH_MIRROR_BIRTH_COVER_HALF_M / 2.0)
      bins = {min(bin_count - 1, max(0, int(
        (x - (x_r - BOSCH_MIRROR_BIRTH_COVER_HALF_M)) / 2.0))) for x in window}
      cover = len(bins) / bin_count
      if (len(window) >= BOSCH_MIRROR_BIRTH_SUPPORT_MIN and
          cover >= BOSCH_MIRROR_BIRTH_COVER_MIN):
        return wall_y, 'OK', len(window), cover
      if len(window) > best_fail[1]:
        best_fail = ('WALL_NOT_AT_SPECULAR_POINT', len(window), cover)
    return None, best_fail[0], best_fail[1], best_fail[2]

  def _mirror_decisions(self, timestamp_ns, v_ego, yaw_rate, raw_tracks, objects,
                        camera_associations, word0_pids):
    mirror = {}
    for obj in objects:
      mirror[obj.physical_track_id] = (
        obj.d_rel, obj.y_rel, obj.v_rel, obj.age_scans,
        tuple(member.raw_track_id for member in obj.members),
        obj.representative_raw_track_id, obj.oem_selected, obj.vision_supported)
    fresh_raw = {raw.raw_track_id for raw in raw_tracks}
    if yaw_rate is not None and math.isfinite(yaw_rate):
      stationary = [(raw.d_rel, raw.y_rel, raw.raw_track_id) for raw in raw_tracks
                    if (abs(raw.v_rel + v_ego - yaw_rate * raw.y_rel) <=
                        BOSCH_MIRROR_BIRTH_STAT_WORLD_MAX_MPS and
                        abs(raw.y_rel) >= BOSCH_MIRROR_BIRTH_STAT_Y_MIN_M)]
      self.hist.append((timestamp_ns, stationary))
    else:
      self.hist.append((timestamp_ns, []))
    self.peak_history_points = max(self.peak_history_points,
                                   sum(len(points) for _ns, points in self.hist))
    context_ok = (yaw_rate is not None and math.isfinite(yaw_rate) and
                  v_ego >= BOSCH_MIRROR_BIRTH_VEGO_MIN_MPS and
                  abs(yaw_rate / max(v_ego, .1)) <= BOSCH_MIRROR_BIRTH_CURV_MAX_1PM)
    yaw_for_world = yaw_rate if yaw_rate is not None and math.isfinite(yaw_rate) else 0.0
    camera_associations = camera_associations or {}
    word0_pids = set(word0_pids)

    def world(obj):
      return obj[2] + v_ego - yaw_for_world * obj[1]

    def fresh(obj):
      return any(member_id in fresh_raw for member_id in obj[4])

    decisions = []
    for pid, obj in mirror.items():
      d_rel, y_rel, _v_rel, age_scans, _members, _rep, _oem, _vision = obj
      if age_scans != 1 or pid in self.seen_births:
        continue
      reason = None
      parent = wall_y = residual = None
      support, cover = 0, 0.0
      if (abs(y_rel) < BOSCH_MIRROR_BIRTH_BEYOND_MIN_M + BOSCH_MIRROR_BIRTH_INSIDE_MIN_M or
          d_rel > BOSCH_MIRROR_BIRTH_D_MAX_M or world(obj) < BOSCH_MIRROR_BIRTH_MOVE_MIN_MPS):
        reason = 'NOT_APPLICABLE'
      elif not context_ok:
        reason = 'CONTEXT_FAIL_OPEN'
      elif not fresh(obj):
        reason = 'X_NOT_FRESH'
      elif (obj[7] or obj[6] or pid in word0_pids or
            (camera_associations.get(pid, (0, -1))[0] == BOSCH_CAMERA_ASSOC_ASSIGNED and
             camera_associations.get(pid, (0, -1))[1] >= 0)):
        reason = 'INDEPENDENT_IDENTITY_FAIL_OPEN'
      else:
        side = 1.0 if y_rel > 0 else -1.0
        candidates = []
        for candidate_pid, candidate in mirror.items():
          if candidate_pid == pid:
            continue
          if (abs(candidate[0] - d_rel) <= BOSCH_MIRROR_BIRTH_DD_MAX_M and
              abs(candidate[2] - obj[2]) <= BOSCH_MIRROR_BIRTH_DV_MAX_MPS and
              side * candidate[1] < side * y_rel -
              (BOSCH_MIRROR_BIRTH_BEYOND_MIN_M + BOSCH_MIRROR_BIRTH_INSIDE_MIN_M)):
            candidates.append((candidate_pid, candidate))
        if not candidates:
          reason = 'NO_PARENT'
        else:
          passing = []
          last_fail = 'PARENT_NOT_DIRECT'
          for candidate_pid, candidate in candidates:
            if (candidate[3] < BOSCH_MIRROR_BIRTH_PARENT_AGE_SCANS or not fresh(candidate) or
                world(candidate) < BOSCH_MIRROR_BIRTH_MOVE_MIN_MPS):
              continue
            excluded = set(obj[4]) | set(candidate[4])
            y_lo = side * candidate[1] + BOSCH_MIRROR_BIRTH_INSIDE_MIN_M
            y_hi = side * y_rel - BOSCH_MIRROR_BIRTH_BEYOND_MIN_M
            found_wall, wall_reason, support_count, cover_fraction = self._wall(
              side, y_lo, y_hi,
              lambda candidate_wall_y: d_rel * candidate_wall_y / y_rel if y_rel else math.nan,
              excluded)
            if found_wall is None:
              last_fail = wall_reason
              support, cover = max(support, support_count), max(cover, cover_fraction)
              continue
            found_residual = y_rel - (2 * found_wall - candidate[1])
            if abs(found_residual) > BOSCH_MIRROR_BIRTH_RESID_MAX_M:
              last_fail = 'MIRROR_RESIDUAL'
              wall_y, residual, parent = found_wall, found_residual, candidate_pid
              continue
            passing.append((abs(found_residual), candidate_pid, found_wall,
                            found_residual, support_count, cover_fraction))
          if not passing:
            reason = last_fail
          else:
            passing.sort()
            _abs_resid, parent, wall_y, residual, support, cover = passing[0]
      if reason is None:
        decisions.append(BoschMirrorBirthDecision(pid, 'QUALIFY', 'OK', parent,
                                                    wall_y, residual, 1))
      elif reason != 'NOT_APPLICABLE':
        decisions.append(BoschMirrorBirthDecision(pid, 'NO_DECISION', reason,
                                                    parent, wall_y, residual, 0))
    return mirror, {decision.physical_track_id: decision for decision in decisions}

  def update(self, objects, raw_tracks, timestamp_ns, v_ego, yaw_rate_left,
             *, camera_associations=(), word0_pids=(), live_pids=None):
    self._pending_events = []
    self.last_decisions = ()
    self.would_suppress = frozenset()
    if self.mode == BOSCH_MIRROR_BIRTH_OFF:
      self.holds.clear()
      self.seen_births.clear()
      self.hist.clear()
      self.last_ns = timestamp_ns
      self.last_events = ()
      self.last_mirror_objects = {}
      self.last_decisions_by_pid = {}
      del self._pending_events
      return self.would_suppress

    if self.last_ns is not None:
      delta_ns = timestamp_ns - self.last_ns
      if delta_ns <= 0 or delta_ns >= BOSCH_MIRROR_BIRTH_GAP_NS:
        for state in tuple(self.holds.values()):
          self._record_event(state, 'RELEASE', 'GAP_RESET', timestamp_ns)
          self.release_count += 1
        self.holds.clear()
        self.seen_births.clear()
        self._reset_shadow()
      else:
        dt = delta_ns * 1e-9
        theta = (yaw_rate_left or 0.0) * dt
        cos_theta, sin_theta = math.cos(theta), math.sin(theta)
        neg_sin, travel = -sin_theta, v_ego * dt
        # Move only the scans that stay in history; the per-point arithmetic is unchanged.
        kept = (self.hist[-(BOSCH_MIRROR_BIRTH_HIST_SCANS - 1):]
                if BOSCH_MIRROR_BIRTH_HIST_SCANS > 1 else [])
        self.hist = [(scan_ns, [(cos_theta * (x1 := x - travel) + sin_theta * y,
                                 neg_sin * x1 + cos_theta * y, raw_id)
                                for x, y, raw_id in points])
                     for scan_ns, points in kept]
    self.last_ns = timestamp_ns

    # Physical IDs are never reused. Keep birth guards for coasting live IDs,
    # but retired IDs cannot return and need no lifetime-long tombstone.
    if live_pids is not None:
      self.seen_births.intersection_update(live_pids)

    mirror, decisions = self._mirror_decisions(
      timestamp_ns, v_ego, yaw_rate_left, raw_tracks, objects,
      camera_associations, word0_pids)
    # M3 reads these exact per-scan v1 objects and decisions. It never asks M1
    # to recompute and does not mutate either mapping.
    self.last_mirror_objects = mirror
    self.last_decisions_by_pid = decisions

    # Existing holds are advanced before this scan's newborns, matching mbh_sim.py.
    for pid in list(self.holds):
      state = self.holds[pid]
      obj = mirror.get(pid)
      if obj is None:
        self._record_event(state, 'RELEASE', 'PID_DEAD', timestamp_ns)
        self.release_count += 1
        del self.holds[pid]
        continue
      state.held_scans += 1
      parent_obj = mirror.get(state.parent_pid)
      side = 1 if state.wall_y_m > 0 else -1
      release_reason = None
      if (obj[7] or obj[6] or pid in set(word0_pids) or
          (camera_associations or {}).get(pid, (0, -1))[0] == BOSCH_CAMERA_ASSOC_ASSIGNED and
          (camera_associations or {}).get(pid, (0, -1))[1] >= 0):
        release_reason = 'INDEPENDENT_IDENTITY'
      elif (side * obj[1] < side * state.wall_y_m + BOSCH_MIRROR_BIRTH_INSIDE_MARGIN_M or
            abs(obj[1]) < BOSCH_MIRROR_BIRTH_Y_MIN_M):
        release_reason = 'MOVED_INSIDE'
      else:
        state.parent_miss_scans = state.parent_miss_scans + 1 if parent_obj is None else 0
        if state.parent_miss_scans >= BOSCH_MIRROR_BIRTH_PARENT_MISS_SCANS:
          release_reason = 'PARENT_LOST'
        elif parent_obj is not None:
          diverged = (abs(obj[2] - parent_obj[2]) > BOSCH_MIRROR_BIRTH_DIVERGE_DV_MPS or
                      abs(obj[0] - parent_obj[0]) > BOSCH_MIRROR_BIRTH_DIVERGE_DD_M)
          state.diverge_scans = state.diverge_scans + 1 if diverged else 0
          if state.diverge_scans >= BOSCH_MIRROR_BIRTH_DIVERGE_SCANS:
            release_reason = 'DIVERGED'
      if (release_reason is None and
          state.held_scans > BOSCH_MIRROR_BIRTH_HOLD_SCANS):
        release_reason = 'EXPIRED'
      if release_reason is not None:
        self._record_event(state, 'RELEASE', release_reason, timestamp_ns)
        self.release_count += 1
        del self.holds[pid]

    seen = self.seen_births
    for pid, obj in mirror.items():
      if obj[3] != 1 or pid in seen:
        continue
      seen.add(pid)
      decision = decisions.get(pid)
      if (decision is None or decision.action != 'QUALIFY' or
          abs(obj[1]) < BOSCH_MIRROR_BIRTH_Y_MIN_M):
        continue
      state = _BoschMirrorBirthHoldState(
        pid, decision.parent_pid, decision.wall_y_m, decision.residual_m,
        obj[0], obj[1], obj[2], timestamp_ns)
      self.holds[pid] = state
      self._record_event(state, 'HOLD', 'MIRROR_BIRTH_QUALIFIED', timestamp_ns)
      self.hold_count += 1

    self.would_suppress = frozenset(self.holds)
    self.last_decisions = tuple(decisions.values())
    self.last_events = tuple(self._pending_events)
    self.peak_hold_count = max(self.peak_hold_count, len(self.holds))
    del self._pending_events
    return self.would_suppress

  def publication_view(self, objects):
    if (self.mode != BOSCH_MIRROR_BIRTH_ACTIVE or not objects or
        not self.would_suppress):
      return objects
    suppressed = {obj.physical_track_id for obj in objects
                  if obj.physical_track_id in self.would_suppress}
    if not suppressed:
      return objects
    self.publication_suppressed += len(suppressed)
    return tuple(obj for obj in objects if obj.physical_track_id not in suppressed)


@dataclass
class _BoschMirrorM3HoldState:
  physical_track_id: int
  path: str
  parent_pid: int
  wall_y_m: float
  residual_m: float
  d_rel_m: float
  y_rel_m: float
  v_rel_mps: float
  v_ego_mps: float
  birth_ns: int
  held_scans: int = 1
  parent_miss_scans: int = 0
  diverge_scans: int = 0
  extension_scans: int = 0


class BoschRoadEdgePublicationFilter:
  """Bosch-only current-edge exclusion; never changes tracking or measured data.

  Model publication time is the freshness basis, as in M3. No prior-edge
  fallback, motion correction, pair inference, or suppression hold is used.
  """

  FRESH_NS = 200_000_000
  RADAR_TO_CAMERA_M = 1.52

  def __init__(self, enabled=True):
    self.enabled = enabled
    self.model_edges = []
    self.publication_model_edges = []
    self.last_ns = None
    self.source_model_ns = None
    self.would_suppress = frozenset()
    self.publication_suppressed = 0
    self._validated_lines = None

  def invalidate_model(self):
    self.model_edges.clear()
    self.publication_model_edges.clear()
    self._validated_lines = None
    self._last_ingest = None
    self.source_model_ns = None
    self.would_suppress = frozenset()

  def reset(self):
    self.invalidate_model()
    self.last_ns = None

  def ingest_model(self, model, model_ns):
    # The context arrives at 100 Hz with the same 20 Hz message. A snapshot is
    # never modified after construction, so repeating the last ingest of the
    # same one rebuilds identical lists; only its clearing of would_suppress
    # for an unusable sample has an effect. Other inputs are compared by content.
    last = getattr(self, '_last_ingest', None)
    if (last is not None and last[0] is model and last[1] == model_ns and
        type(model) is BoschModelSnapshot):
      if last[2]:
        self.would_suppress = frozenset()
      return
    # Keep invalid samples too: an uncertain current edge must not silently
    # fall back to a previously confident one.
    sample = publication_sample = None
    edges = getattr(model, 'roadEdges', ())
    stds = tuple(getattr(model, 'roadEdgeStds', ()))
    if len(edges) == 2 and len(stds) == 2 and all(math.isfinite(s) and s >= 0 for s in stds):
      lines = tuple((tuple(e.x), tuple(e.y)) for e in edges)
      # Compare contents, not publication time or mutable message identity.
      # Binary doubles preserve signed zero; retain only one validated shape.
      key = tuple((struct.pack(f'<{len(xs)}d', *xs), struct.pack(f'<{len(ys)}d', *ys)) for xs, ys in lines)
      if self._validated_lines is not None and key == self._validated_lines[0]:
        publication_sample = (self._validated_lines[1], stds)
      elif all(len(xs) >= 2 and len(xs) == len(ys) and
             all(math.isfinite(v) for v in xs + ys) and
             all(a < b for a, b in zip(xs, xs[1:])) for xs, ys in lines):
        # np.interp otherwise converts both tuples for every edge/object pair.
        # Own the float arrays once per sample and reuse them without changing
        # validation, interpolation arithmetic, or model freshness decisions.
        publication_sample = (tuple((np.asarray(xs, dtype=float), np.asarray(ys, dtype=float)) for xs, ys in lines), stds)
        self._validated_lines = (key, publication_sample[0])
    # The existing scan gate still requires two confident edges. Retain the
    # same validated arrays for the final distant-corner check, which may use
    # a confident exterior edge independently of an uncertain opposite edge.
    if publication_sample is not None and all(s <= 1 for s in stds):
      sample = publication_sample
    self.publication_model_edges = [(ns, value) for ns, value in self.publication_model_edges if ns != model_ns]
    self.publication_model_edges.append((int(model_ns), publication_sample))
    self.publication_model_edges.sort(key=lambda item: item[0])
    self.publication_model_edges = self.publication_model_edges[-4:]
    self.model_edges = [(ns, value) for ns, value in self.model_edges if ns != model_ns]
    self.model_edges.append((int(model_ns), sample))
    self.model_edges.sort(key=lambda item: item[0])
    self.model_edges = self.model_edges[-4:]
    if sample is None:
      self.would_suppress = frozenset()
    self._last_ingest = (model, model_ns, sample is None)

  def update(self, objects, timestamp_ns, *, valid=True, camera_associations=None, word0_pids=()):
    self.would_suppress = frozenset()
    self.source_model_ns = None
    if self.last_ns is not None and timestamp_ns < self.last_ns:
      self.reset()
    self.last_ns = timestamp_ns
    if not self.enabled or not valid:
      return
    current = next(((ns, sample) for ns, sample in reversed(self.model_edges) if ns <= timestamp_ns), None)
    if current is None:
      return
    model_ns, sample = current
    if sample is None or not 0 <= timestamp_ns - model_ns <= self.FRESH_NS:
      return
    self.source_model_ns = model_ns
    lines, stds = sample
    protected = set(word0_pids)
    camera_associations = camera_associations or {}
    hidden = set()
    for obj in objects:
      association = camera_associations.get(obj.physical_track_id)
      if (obj.timestamp_ns != timestamp_ns or obj.oem_selected or obj.vision_supported or
          obj.physical_track_id in protected or
          (association is not None and association[0] == BOSCH_CAMERA_ASSOC_ASSIGNED and association[1] >= 0)):
        continue
      # roadEdges use the model/camera longitudinal origin, as in M3.
      d, right = obj.d_rel + self.RADAR_TO_CAMERA_M, -obj.y_rel
      if not math.isfinite(d) or not math.isfinite(right) or not all(xs[0] <= d <= xs[-1] for xs, _ in lines):
        continue
      left_edge, right_edge = (float(np.interp(d, xs, ys)) for xs, ys in lines)
      if left_edge >= right_edge:
        continue
      if right < left_edge - 1.0 - 2.0 * stds[0] or right > right_edge + 1.0 + 2.0 * stds[1]:
        hidden.add(obj.physical_track_id)
    self.would_suppress = frozenset(hidden)

  def publication_view(self, objects, timestamp_ns):
    if (not self.enabled or not self.would_suppress or timestamp_ns is None or
        self.last_ns is None or self.source_model_ns is None or
        not 0 <= timestamp_ns - self.last_ns <= self.FRESH_NS or
        not 0 <= timestamp_ns - self.source_model_ns <= self.FRESH_NS):
      return objects
    result = tuple(obj for obj in objects if obj.timestamp_ns != self.last_ns or
                   obj.physical_track_id not in self.would_suppress)
    self.publication_suppressed += len(objects) - len(result)
    return result

  def publication_surface_view(self, provider, objects, timestamp_ns):
    """Recheck current geometry at the coordinates the native points will use.

    Fresh unsupported moving singletons outside the path corridor qualify.
    A distant turn may use its exterior edge independently when the current
    path bends by more than the whole road width. Competing raw edges alone
    do not veto this conjunction, but a genuinely tied assignment does.
    Current support or invalid geometry releases immediately; no holds.
    Only points published at or beyond BOSCH_SURFACE_CORNER_MIN_RANGE_M qualify.
    """
    if (not self.enabled or not objects or timestamp_ns is None or
        provider.can_error or provider.wrong_config):
      return objects
    current = next(((ns, sample) for ns, sample in reversed(self.publication_model_edges) if ns <= timestamp_ns), None)
    if current is None or current[1] is None or not 0 <= timestamp_ns - current[0] <= self.FRESH_NS:
      return objects
    lines, stds = current[1]
    if not any(s <= 1 for s in stds):
      return objects
    shared, kept = {}, []
    for obj in objects:
      # The earlier scan gate and the final native projection have different
      # coordinates and model availability. Never reuse a scan verdict here.
      if len(obj.members) != 1 or obj.members[0].timestamp_ns != provider.last_scan_timestamp_ns:
        kept.append(obj)
        continue
      wire_d, wire_y, wire_v = bosch_native_surface_coordinates(obj, timestamp_ns)
      d, right = wire_d + self.RADAR_TO_CAMERA_M, -wire_y
      if (not all(math.isfinite(v) for v in (d, right, wire_v)) or
          wire_d < BOSCH_SURFACE_CORNER_MIN_RANGE_M or
          not all(xs[0] <= d <= xs[-1] for xs, _ in lines)):
        kept.append(obj)
        continue
      left_edge, right_edge = (float(np.interp(d, xs, ys)) for xs, ys in lines)
      outside = (stds[0] <= 1 and left_edge - right - 1.0 - 2.0 * stds[0] > 0,
                 stds[1] <= 1 and right - right_edge - 1.0 - 2.0 * stds[1] > 0)
      if left_edge >= right_edge or not any(outside):
        kept.append(obj)
        continue
      decision = provider.unsupported_birth.verdict(provider, obj, timestamp_ns,
                                                     _shared=shared, _first_only=False)
      if decision['exclude'] and all(s <= 1 for s in stds):
        continue
      if not decision['exclude'] and decision['reason'] == 'AMBIGUOUS_RAW':
        decision = provider.unsupported_birth.verdict(provider, obj, timestamp_ns,
            _shared=shared, _first_only=False, _require_single_candidate=False)
      if not decision['exclude']:
        kept.append(obj)
        continue
      _, _, yaw, path, _ = shared['context'][1]
      # Both motion and the current path must agree about the turn. Removing
      # initial heading isolates curvature; a nearby gentle bend is retained.
      if 'curve_path' not in shared:
        xs, ys = zip(*path)
        shared['curve_path'] = (xs, ys) if all(a < b for a, b in zip(xs, xs[1:])) else None
      curve_path = shared['curve_path']
      if curve_path is None:
        kept.append(obj)
        continue
      xs, ys = curve_path
      heading = (ys[1] - ys[0]) / (xs[1] - xs[0])
      bend = float(np.interp(obj.d_rel, xs, ys)) - ys[0] - heading * (obj.d_rel - xs[0])
      exterior = 0 if yaw < 0 and bend > 0 else 1 if yaw > 0 and bend < 0 else None
      if exterior is None or abs(bend) <= right_edge - left_edge or not outside[exterior]:
        kept.append(obj)
    removed = len(objects) - len(kept)
    self.publication_suppressed += removed
    return tuple(kept) if removed else objects


class BoschMirrorM3Shadow:
  """M3 edge-birth/coupled-extension state with an optional publication-only ACTIVE view."""

  def __init__(self, mode=BOSCH_MIRROR_M3_MODE):
    if mode not in (BOSCH_MIRROR_M3_OFF, BOSCH_MIRROR_M3_SHADOW, BOSCH_MIRROR_M3_ACTIVE):
      raise ValueError('invalid Bosch mirror M3 mode')
    self.mode = mode
    self.model_edges: list[tuple[int, tuple, tuple, tuple]] = []
    self.holds: dict[int, _BoschMirrorM3HoldState] = {}
    self.seen_births: set[int] = set()
    self.last_ns: int | None = None
    self.last_edge_sample: tuple[int, tuple, tuple, tuple] | None = None
    self.last_events: tuple[dict, ...] = ()
    self.publication_suppressed = 0
    self.last_publication_suppressed: frozenset[int] = frozenset()
    self.reset_count = 0

  def ingest_model(self, model, model_ns):
    """Keep the newest four road-edge samples from fresh, valid modelV2 input."""
    if self.mode not in (BOSCH_MIRROR_M3_SHADOW, BOSCH_MIRROR_M3_ACTIVE) or model is None:
      return
    edges = getattr(model, 'roadEdges', ())
    if len(edges) < 2:
      return
    model_ns = int(model_ns)
    if any(sample[0] == model_ns for sample in self.model_edges):
      return
    xs = tuple(float(x) for x in edges[0].x)
    y_left = tuple(-float(y) for y in edges[0].y)
    y_right = tuple(-float(y) for y in edges[1].y)
    sample = (model_ns, xs, y_left, y_right)
    self.model_edges.append(sample)
    self.model_edges.sort(key=lambda item: item[0])
    self.model_edges = self.model_edges[-4:]

  def _edge_at(self, scan_ns):
    for sample in reversed(self.model_edges):
      if sample[0] <= scan_ns:
        if scan_ns - sample[0] <= BOSCH_MIRROR_M3_EDGE_FRESH_NS:
          return sample
        return None
    return None

  def _record_event(self, state, action, reason, timestamp_ns):
    fields = dict(
      action=action, ns=int(timestamp_ns), pid=state.physical_track_id,
      birth_ns=state.birth_ns,
      path=state.path, parent=state.parent_pid, wall_y=state.wall_y_m,
      resid=state.residual_m, d=state.d_rel_m, y=state.y_rel_m,
      v=state.v_rel_mps, v_ego=state.v_ego_mps)
    if action == 'RELEASE':
      fields.update(reason=reason, held_scans=state.held_scans,
                    ext_scans=state.extension_scans)
    self._pending_events.append(fields)

  def reset(self, timestamp_ns=None, reason='STATE_RESET'):
    self._pending_events = []
    release_ns = self.last_ns if timestamp_ns is None else int(timestamp_ns)
    for state in tuple(self.holds.values()):
      if release_ns is not None:
        self._record_event(state, 'RELEASE', reason, release_ns)
    self.holds.clear()
    self.seen_births.clear()
    self.last_ns = None
    self.last_edge_sample = None
    self.reset_count += 1
    self.last_events = tuple(self._pending_events)
    del self._pending_events

  @staticmethod
  def _independent(pid, obj, camera_associations, word0_pids):
    association = (camera_associations or {}).get(pid, (0, -1))
    return bool(obj[7] or obj[6] or pid in set(word0_pids or ()) or
                (association[0] == BOSCH_CAMERA_ASSOC_ASSIGNED and association[1] >= 0))

  def _edge_birth(self, pid, obj, mirror, edge, v_ego):
    if (edge is None or not math.isfinite(v_ego) or v_ego < BOSCH_MIRROR_M3_VEGO_MIN_MPS or
        obj[0] > BOSCH_MIRROR_M3_D_MAX_M):
      return None
    side = 1 if obj[1] > 0 else -1
    if BOSCH_MIRROR_M3_EDGE_LEFT_ONLY and side < 0:
      return None
    _edge_ns, xs, y_left, y_right = edge
    ys = y_left if side > 0 else y_right
    if not xs or len(xs) != len(ys):
      return None
    wall_y = float(np.interp(obj[0] + BOSCH_MIRROR_M3_RADAR_TO_CAMERA_M, xs, ys))
    if not math.isfinite(wall_y):
      return None
    if (side * wall_y <= BOSCH_MIRROR_M3_EDGE_SIDE_MIN_M or
        side * obj[1] < side * wall_y + BOSCH_MIRROR_M3_BEYOND_MIN_M):
      return None

    best = None
    for parent_pid, parent in mirror.items():
      if parent_pid == pid or parent[3] < BOSCH_MIRROR_M3_PARENT_AGE_SCANS:
        continue
      if abs(parent[2] + v_ego) < BOSCH_MIRROR_M3_PARENT_VABS_MIN_MPS:
        continue
      if side * parent[1] > side * wall_y - BOSCH_MIRROR_M3_PARENT_INSIDE_MIN_M:
        continue
      if (abs(obj[0] - parent[0]) > BOSCH_MIRROR_M3_PARENT_DD_MAX_M or
          abs(obj[2] - parent[2]) > BOSCH_MIRROR_M3_PARENT_DV_MAX_MPS):
        continue
      residual = abs(obj[1] - (2 * wall_y - parent[1]))
      if residual <= BOSCH_MIRROR_M3_RESID_MAX_M and (best is None or residual < best[2]):
        best = (parent_pid, wall_y, residual)
    return best

  def update(self, timestamp_ns, v_ego, mirror_objects, mirror_decisions, *,
             camera_associations=None, word0_pids=(), mirror_enabled=True, live_pids=None):
    """Read M1's current scan result and make an independent SHADOW decision."""
    timestamp_ns = int(timestamp_ns)
    self._pending_events = []
    self.last_edge_sample = None
    if self.mode not in (BOSCH_MIRROR_M3_SHADOW, BOSCH_MIRROR_M3_ACTIVE) or not mirror_enabled:
      if self.holds or self.seen_births:
        self.reset(timestamp_ns, 'M3_OFF' if self.mode == BOSCH_MIRROR_M3_OFF else 'M1_OFF')
      else:
        self.last_events = ()
      return frozenset()

    mirror = mirror_objects or {}
    decisions = mirror_decisions or {}
    word0 = set(word0_pids or ())
    edge = self._edge_at(timestamp_ns)
    self.last_edge_sample = edge
    if self.last_ns is not None:
      delta_ns = timestamp_ns - self.last_ns
      if delta_ns <= 0 or delta_ns >= BOSCH_MIRROR_M3_GAP_NS:
        for state in tuple(self.holds.values()):
          self._record_event(state, 'RELEASE', 'GAP_RESET', timestamp_ns)
        self.holds.clear()
        self.seen_births.clear()
        self.reset_count += 1
    self.last_ns = timestamp_ns

    if live_pids is not None:
      self.seen_births.intersection_update(live_pids)

    for pid in list(self.holds):
      state = self.holds[pid]
      obj = mirror.get(pid)
      if obj is None:
        self._record_event(state, 'RELEASE', 'PID_DEAD', timestamp_ns)
        del self.holds[pid]
        continue
      state.held_scans += 1
      parent = mirror.get(state.parent_pid)
      side = 1 if state.wall_y_m > 0 else -1
      reason = None
      if self._independent(pid, obj, camera_associations, word0):
        reason = 'INDEPENDENT_IDENTITY'
      elif (side * obj[1] < side * state.wall_y_m + BOSCH_MIRROR_M3_INSIDE_MARGIN_M or
            abs(obj[1]) < BOSCH_MIRROR_M3_Y_MIN_M):
        reason = 'MOVED_INSIDE'
      else:
        state.parent_miss_scans = state.parent_miss_scans + 1 if parent is None else 0
        if state.parent_miss_scans >= BOSCH_MIRROR_M3_PARENT_MISS_SCANS:
          reason = 'PARENT_LOST'
        elif parent is not None:
          diverged = (abs(obj[2] - parent[2]) > BOSCH_MIRROR_M3_DIVERGE_DV_MPS or
                      abs(obj[0] - parent[0]) > BOSCH_MIRROR_M3_DIVERGE_DD_M)
          state.diverge_scans = state.diverge_scans + 1 if diverged else 0
          if state.diverge_scans >= BOSCH_MIRROR_M3_DIVERGE_SCANS:
            reason = 'DIVERGED'
      if reason is None and state.held_scans > BOSCH_MIRROR_M3_HOLD_SCANS:
        coupled = (parent is not None and
                   abs(obj[0] - parent[0]) <= BOSCH_MIRROR_M3_EXT_DD_M and
                   abs(obj[2] - parent[2]) <= BOSCH_MIRROR_M3_EXT_DV_MPS and
                   abs(obj[1] - (2 * state.wall_y_m - parent[1])) <= BOSCH_MIRROR_M3_EXT_RESID_M)
        if not coupled:
          reason = 'EXPIRED'
        elif state.held_scans > BOSCH_MIRROR_M3_EXT_MAX_SCANS:
          reason = 'EXT_CAP'
        else:
          state.extension_scans += 1
      if reason is not None:
        self._record_event(state, 'RELEASE', reason, timestamp_ns)
        del self.holds[pid]

    for pid, obj in mirror.items():
      if obj[3] != 1 or pid in self.seen_births:
        continue
      self.seen_births.add(pid)
      if abs(obj[1]) < BOSCH_MIRROR_M3_Y_MIN_M:
        continue
      decision = decisions.get(pid)
      path = None
      parent_pid = wall_y = residual = None
      if decision is not None and decision.action == 'QUALIFY':
        path = 'V1'
        parent_pid, wall_y, residual = decision.parent_pid, decision.wall_y_m, decision.residual_m
      elif (BOSCH_MIRROR_M3_EDGE and
            not self._independent(pid, obj, camera_associations, word0)):
        edge_result = self._edge_birth(pid, obj, mirror, edge, v_ego)
        if edge_result is not None:
          path = 'EDGE'
          parent_pid, wall_y, residual = edge_result
      if path is None:
        continue
      state = _BoschMirrorM3HoldState(
        pid, path, parent_pid, float(round(wall_y, 3)), float(round(residual, 3)), float(obj[0]),
        float(obj[1]), float(obj[2]), float(v_ego), timestamp_ns)
      self.holds[pid] = state
      self._record_event(state, 'HOLD', 'MIRROR_BIRTH_QUALIFIED', timestamp_ns)

    self.last_events = tuple(self._pending_events)
    del self._pending_events
    return frozenset(self.holds)

  def publication_view(self, objects):
    """In ACTIVE, remove M3-held physical PIDs after the independent M1 view."""
    self.last_publication_suppressed = frozenset()
    if self.mode != BOSCH_MIRROR_M3_ACTIVE or not objects or not self.holds:
      return objects
    suppressed = frozenset(obj.physical_track_id for obj in objects
                            if obj.physical_track_id in self.holds)
    if not suppressed:
      return objects
    self.publication_suppressed += len(suppressed)
    self.last_publication_suppressed = suppressed
    return tuple(obj for obj in objects if obj.physical_track_id not in suppressed)


# Provisional acquisition policy; a normal first observation can also be deferred.
@lru_cache(maxsize=8)
def _unsupported_birth_segments(path):
  points = tuple((float(x), -float(y)) for x, y in path)
  segments = []
  for (x0, y0), (x1, y1) in zip(points, points[1:]):
    dx, dy = x1 - x0, y1 - y0
    length = math.hypot(dx, dy)
    if length >= 1e-6:
      segments.append((x0, y0, dx / length, dy / length, length))
  return points, tuple(segments)


def birth_d_path(path, x, y):
  """Nearest clamped polyline normal, with model right-positive y inverted.

  The caller forbids range extrapolation (x <= final path x), so the terminal
  extrapolation branch of the control projector cannot apply here.
  """
  points, segments = _unsupported_birth_segments(path)
  if not segments:
    return y - points[0][1]
  best_distance = math.inf
  best = None
  for x0, y0, tx, ty, length in segments:
    dx, dy = tx * length, ty * length
    ratio = min(1.0, max(0.0, ((x - x0) * dx + (y - y0) * dy) / (length * length)))
    ox, oy = x - (x0 + ratio * dx), y - (y0 + ratio * dy)
    distance = ox * ox + oy * oy
    if distance < best_distance:
      best_distance, best = distance, -ty * ox + tx * oy
  return best


class BoschUnsupportedBirthFilter:
  def __init__(self, enabled=True):
    self.enabled = bool(enabled)
    self.last_decisions = {}

  @staticmethod
  def _context(provider, publication_ns):
    context = provider._unsupported_birth_context
    if context is None or publication_ns is None:
      return 'MISSING_RECEIVE_CONTEXT', None, None
    received_ns, ego, yaw, path, path_ns = context
    if not 0 <= publication_ns - received_ns <= 200_000_000:
      return 'STALE_EGO_CONTEXT', None, None
    if not math.isfinite(ego) or ego < 3.0 or yaw is None or not math.isfinite(yaw) or abs(yaw / ego) > .02:
      return 'MISSING_OR_INVALID_MOTION', None, None
    if (path_ns is None or not 0 <= publication_ns - path_ns <= 200_000_000 or len(path) < 2
        or not all(math.isfinite(float(v)) for point in path for v in point)):
      return 'MISSING_OR_STALE_PATH', None, None
    scan_ns = provider.last_scan_timestamp_ns
    if scan_ns is None or not scan_ns <= received_ns <= publication_ns or publication_ns - scan_ns > 150_000_000:
      return 'STALE_SCAN', None, None
    return None, context, scan_ns

  def verdict(self, provider, obj, publication_ns, *, _shared=None, _first_only=True, _require_single_candidate=True):
    def keep(reason):
      return {'exclude': False, 'reason': reason}
    if not self.enabled:
      return keep('DISABLED')
    if provider.can_error or provider.wrong_config:
      return keep('INVALID_PROVIDER')
    if len(obj.members) != 1 or (_first_only and obj.age_scans != 1):
      return keep('NOT_FIRST_SINGLETON_OBSERVATION')
    if _shared is None:
      reason, context, scan_ns = self._context(provider, publication_ns)
    else:
      if 'context' not in _shared:
        _shared['context'] = self._context(provider, publication_ns)
      reason, context, scan_ns = _shared['context']
    if reason is not None:
      return keep(reason)
    received_ns, ego, yaw, path, path_ns = context
    if obj.timestamp_ns != scan_ns:
      return keep('NOT_FIRST_SINGLETON_OBSERVATION')
    member = obj.members[0]
    identity = provider.tracker.raw_manager.last_identity_evidence.get(member.raw_track_id)
    if identity is None or identity[0] != member.timestamp_ns or member.timestamp_ns > scan_ns:
      return keep('MISSING_RAW_LINK')
    if (_require_single_candidate and identity[1] > 1) or identity[2]:
      return keep('AMBIGUOUS_RAW')
    if not all(math.isfinite(v) for v in (obj.d_rel, obj.y_rel, obj.v_rel)):
      return keep('INVALID_MEASUREMENT')
    if abs(ego + obj.v_rel) <= 1.0:
      return keep('STATIONARY_LIKE')
    if obj.d_rel < path[0][0] or obj.d_rel > path[-1][0]:
      return keep('OUTSIDE_PATH_EXTENT')
    d_path = birth_d_path(path, obj.d_rel, obj.y_rel)
    if d_path is None or not math.isfinite(d_path) or abs(d_path) <= 1.8:
      return keep('CURRENT_PATH_CORRIDOR')
    pid = obj.physical_track_id
    cache = provider.camera_extended
    prior = cache.last_associations.get(pid)
    if (obj.vision_supported or obj.oem_selected or pid in provider._debug_processed_pids
        or (cache.last_ns == scan_ns and prior is not None and prior[0] in (1, 2))):
      return keep('CURRENT_NATIVE_SUPPORT')
    if _shared is None:
      snapshot = cache.camera.snapshot(publication_ns) if cache.camera is not None else None
    else:
      if 'snapshot' not in _shared:
        _shared['snapshot'] = cache.camera.snapshot(publication_ns) if cache.camera is not None else None
      snapshot = _shared['snapshot']
    if snapshot is None or not 0 <= publication_ns - snapshot[3] <= 200_000_000:
      return keep('MISSING_OR_STALE_CAMERA')
    if _shared is None:
      candidates = tuple(snapshot[0][:snapshot[1]])
    else:
      if 'candidates' not in _shared:
        _shared['candidates'] = tuple(snapshot[0][:snapshot[1]])
      candidates = _shared['candidates']
    association = cache._associate(obj, candidates, snapshot[1])
    if association[0] in (1, 2):
      return keep('CURRENT_A0_SUPPORTED_OR_AMBIGUOUS')
    return {'exclude': True, 'reason': 'MOVING_FIRST_SINGLETON_OFFPATH_CURRENTLY_UNSUPPORTED',
            'dPath': d_path, 'world_longitudinal_speed_approx': ego + obj.v_rel,
            'current_a0': association, 'source_ns': member.timestamp_ns,
            'snapshot_complete_ns': snapshot[3], 'publication_ns': publication_ns}

  def publication_view(self, provider, objects, publication_ns):
    self.last_decisions = {}
    if not self.enabled or not objects or provider.can_error or provider.wrong_config:
      return objects
    # Reuse only within this synchronous publication. A second publication,
    # even at the same timestamp, must read changed path/camera support again.
    shared = {}
    kept = []
    for obj in objects:
      if obj.age_scans != 1 or len(obj.members) != 1:
        kept.append(obj)
        continue
      decision = self.verdict(provider, obj, publication_ns, _shared=shared)
      if decision['exclude']:
        self.last_decisions[obj.physical_track_id] = decision
      else:
        kept.append(obj)
    return tuple(kept) if self.last_decisions else objects


class BoschRadarProvider:
  def __init__(self, bus: int, *, qualification=True, camera_bus=1,
               camera_extended_mode=BOSCH_CAMERA_EXTENDED_MODE, p91_mode=BOSCH_P91_MODE,
               oem_gate_mode=BOSCH_OEM_GATE_MODE, scc_bus=BOSCH_SCC_BUS,
               curve_reacquire_mode=BOSCH_CAMERA_CURVE_REACQUIRE_MODE,
               provisional_bundle=True, family_companion_mode=BOSCH_FAMILY_COMPANION_MODE,
               burst_multireturn_mode=BOSCH_BURST_MULTIRETURN_MODE,
               b5_mode=BOSCH_B5_MODE, sidepass_lateral_mode=BOSCH_SIDEPASS_LATERAL_MODE,
               mirror_birth_mode=BOSCH_MIRROR_BIRTH_MODE, unsupported_birth=True,
               mature_group_retention=True):
    self.bus = bus
    self.camera_bus = camera_bus
    self.scc_bus = scc_bus
    self.tracker = BoschPhysicalTracker(group_config=BoschGroupingConfig(
      provisional_enabled=provisional_bundle, mature_retention_enabled=mature_group_retention))
    self.camera_extended = BoschCameraExtendedGrouping(camera_extended_mode, curve_reacquire_mode)
    self.publication_aliases = BoschPublicationAliasAllocator()
    self.qualifier = _BoschPublicationPassThrough() if qualification else None
    self.family_companion = _BoschFamilyCompanionFilter(family_companion_mode)
    self.burst_multireturn = _BoschBurstMultiReturnDefer(burst_multireturn_mode)
    self.b5_birth_defer = BoschBirthB5Defer(b5_mode)
    self.mirror_birth_hold = BoschMirrorBirthHold(mirror_birth_mode)
    self.mirror_m3_shadow = BoschMirrorM3Shadow()
    self.road_edge_filter = BoschRoadEdgePublicationFilter()
    self.unsupported_birth = BoschUnsupportedBirthFilter(unsupported_birth)
    self._unsupported_birth_context = None
    self.sidepass_lateral = _BoschSidePassLateralEstimator(sidepass_lateral_mode)
    self.p91 = _BoschPersistentSpatialCloneFilter(p91_mode)
    self.oem_gate = _BoschOemValidationGate(oem_gate_mode)
    self.scc_obj_valid = None
    self.scc_obj_valid_ns = 0
    self.can_error = False
    self.wrong_config = False
    self.last_scan_timestamp_ns = None
    self._debug_objects = ()
    self._debug_phase_ns = None
    self._debug_closed_ns = None
    self._debug_tick = None
    self._debug_complete = False
    self._debug_oem_word = None
    self._debug_oem_slot = None
    self._debug_oem_matches = 0
    self._debug_processed_word = None
    self._debug_processed_pids = frozenset()
    self._debug_word0_active = False
    self._debug_word1_active = False
    self._debug_oem_state = BOSCH_OEM_STATE_NONE
    self._debug_gate_suppress = frozenset()
    self._debug_gate_reasons = {}
    self._debug_oem_intent = None
    self._debug_timeout = False
    self._frames = []
    self._anchors = []
    self._order = 0
    self._start_ns = None
    self._last_now_ns = None
    self._last_closed_anchor_ns = None
    self._last_output_ns = None
    self._pending_error = False
    self.test_last_suppressed = ()
    self.test_last_active_groups = 0
    self.last_oem_state = BOSCH_OEM_STATE_NONE
    self.companion_pairs = {}
    self.companion_deferred_points = 0
    self.companion_defer_scans = 0
    self.last_companion_deferred = ()
    self._companion_ns = None
    self._companion_prev_ns = None
    self._companion_hidden = {}
    self._companion_shared_start = {}
    self.last_oem_slot = None
    self.oem_nearer_corrections = 0
    self.oem_nearer_scans = 0
    self.last_oem_nearer = ()

  def _companion_pairs_update(self, objects):
    """같은 scan에 대해 한 번만 돌며 (먼 PID -> 가까운 PID) 지연 대상 쌍을 만든다."""
    ext = self.camera_extended
    timestamp_ns = ext.last_ns
    if (self._companion_prev_ns is not None and
        timestamp_ns - self._companion_prev_ns > BOSCH_CAMERA_OBSERVATION_GAP_NS):
      self.companion_pairs = {}
      self._companion_shared_start = {}
    self._companion_prev_ns = timestamp_ns
    if (ext.last_camera_ns is None or
        not 0 <= timestamp_ns - ext.last_camera_ns <= BOSCH_CAMERA_OBSERVATION_GAP_NS):
      # camera가 낡으면 identity 근거가 없다. fail-open으로 상태를 버린다.
      self.companion_pairs = {}
      self._companion_shared_start = {}
      return {}
    by_pid = {obj.physical_track_id: obj for obj in objects if obj.timestamp_ns == timestamp_ns}
    episodes = {}
    for pid, verdict in ext.last_associations.items():
      if verdict[0] == BOSCH_CAMERA_ASSOC_ASSIGNED and verdict[1] >= 0 and pid in by_pid:
        episodes.setdefault(verdict[1], []).append(pid)
    validated = self.last_oem_state == BOSCH_OEM_STATE_VALIDATED
    v_ego = ext.last_v_ego
    yaw = ext.last_yaw_rate if ext.last_yaw_rate is not None and math.isfinite(ext.last_yaw_rate) else 0.0
    next_pairs = {}
    next_shared_start = {}
    hidden = {}
    for episode, pids in sorted(episodes.items()):
      if len(pids) < 2:
        continue
      camera = ext.last_camera_by_episode.get(episode)
      if camera is None:
        continue
      limit = BOSCH_COMPANION_DEFER_DD_MAX_M
      for width, bound in BOSCH_COMPANION_DEFER_WIDTH_DD_M:
        if camera.width_m < width:
          limit = min(limit, bound)
          break
      members = sorted(pids, key=lambda pid: (by_pid[pid].d_rel, pid))
      for index in range(len(members) - 1):
        near, far = by_pid[members[index]], by_pid[members[index + 1]]
        key = (episode, members[index], members[index + 1])
        confirmations, last_ns = self.companion_pairs.get(key, (0, 0))
        shared_start_ns = self._companion_shared_start.get(key, timestamp_ns)
        conflict = False
        if math.isfinite(v_ego):
          world_near = abs(near.v_rel + v_ego - yaw * near.y_rel)
          world_far = abs(far.v_rel + v_ego - yaw * far.y_rel)
          conflict = min(world_near, world_far) <= 0.6 and max(world_near, world_far) >= 1.4
        baseline_ok = (validated and near.oem_selected and not far.oem_selected and not conflict and
                       BOSCH_COMPANION_DEFER_DD_MIN_M < far.d_rel - near.d_rel <= limit and
                       abs(far.y_rel - near.y_rel) <= BOSCH_COMPANION_DEFER_DY_MAX_M and
                       abs(far.v_rel - near.v_rel) <= BOSCH_COMPANION_DEFER_DV_MAX_MPS and
                       abs(near.y_rel) <= BOSCH_COMPANION_DEFER_Y_MAX_M and
                       abs(far.y_rel) <= BOSCH_COMPANION_DEFER_Y_MAX_M)
        split, handoff = self.tracker.group_manager.common_ancestry_evidence(
          far, near, shared_start_ns, timestamp_ns)
        common_ancestry = split and handoff
        if baseline_ok and common_ancestry:
          confirmations, last_ns = confirmations + 1, timestamp_ns
        elif (not common_ancestry or
              not (confirmations >= BOSCH_COMPANION_DEFER_CONFIRMATIONS and
                   0 <= timestamp_ns - last_ns <= BOSCH_COMPANION_DEFER_HOLD_NS)):
          # 근거가 끊기면 즉시 발행을 되돌린다. hold는 확인된 쌍에만 준다.
          confirmations = 0
        if len(next_pairs) < BOSCH_COMPANION_DEFER_STATE_MAX:
          next_pairs[key] = (confirmations, last_ns)
          next_shared_start[key] = shared_start_ns
        if confirmations >= BOSCH_COMPANION_DEFER_CONFIRMATIONS:
          hidden[members[index + 1]] = members[index]
    self.companion_pairs = next_pairs
    self._companion_shared_start = next_shared_start
    return hidden

  def _oem_nearer_view(self, objects):
    """같은 scan의 OEM word1 member가 대표보다 가까우면 발행 좌표를 그 member로 바꾼다.

    `publication_view`의 마지막 단계에서만 동작하고 반환 tuple은 곧바로 RadarData가
    되므로 provider 내부 상태에 되먹임이 없다. 거리는 가까워지는 방향으로만 바뀐다.
    """
    if (BOSCH_OEM_NEARER_PUBLICATION_MODE != BOSCH_OEM_NEARER_PUBLICATION_ACTIVE or
        not objects or self.last_oem_slot is None or self.last_scan_timestamp_ns is None):
      self.last_oem_nearer = ()
      return objects
    slot = self.last_oem_slot
    scan_ns = self.last_scan_timestamp_ns
    corrected = None
    moved = []
    for index, obj in enumerate(objects):
      if len(obj.members) < 2 or obj.timestamp_ns != scan_ns:
        continue
      owned = next((m for m in obj.members if m.slot == slot), None)
      if owned is None or owned.d_rel >= obj.d_rel:
        continue
      if corrected is None:
        corrected = list(objects)
      # Move the published surface only. The continuity anchor -- the object's
      # own d/y/v and representative_raw_track_id -- is carried over untouched,
      # so the object handed to RadarData still reports the state the tracker
      # will project forward next scan.
      corrected[index] = BoschPhysicalObject(
        obj.physical_track_id, obj.timestamp_ns, obj.members,
        obj.representative_raw_track_id, obj.d_rel, obj.y_rel, obj.v_rel,
        obj.oem_selected, obj.vision_supported, obj.age_scans, obj.grouping_evidence,
        BoschPublishedSurface(owned.raw_track_id, owned.d_rel, owned.y_rel, owned.v_rel))
      moved.append(obj.physical_track_id)
    self.last_oem_nearer = tuple(moved)
    if corrected is None:
      return objects
    self.oem_nearer_corrections += len(moved)
    self.oem_nearer_scans += 1
    return tuple(corrected)

  def _final_view(self, objects):
    return self._oem_nearer_view(self._companion_view(objects))

  def _companion_view(self, objects):
    """Prior common raw/group ancestry가 확인된 pair의 먼 표면만 발행에서 미룬다."""
    ext = self.camera_extended
    if (BOSCH_COMPANION_DEFER_MODE != BOSCH_COMPANION_DEFER_ACTIVE or
        ext.mode == BOSCH_CAMERA_EXTENDED_OFF or ext.last_ns is None or not objects):
      self.last_companion_deferred = ()
      return objects
    if ext.last_ns != self._companion_ns:
      self._companion_ns = ext.last_ns
      self._companion_hidden = self._companion_pairs_update(objects)
    hidden = self._companion_hidden
    if not hidden:
      self.last_companion_deferred = ()
      return objects
    present = {obj.physical_track_id for obj in objects}
    # 가까운 쪽이 실제로 발행될 때만 먼 쪽을 숨긴다. 마지막 contact는 지우지 않는다.
    deferred = frozenset(far for far, near in hidden.items() if far in present and near in present)
    self.last_companion_deferred = tuple(sorted(deferred))
    if not deferred:
      return objects
    self.companion_deferred_points += len(deferred)
    self.companion_defer_scans += 1
    return tuple(obj for obj in objects if obj.physical_track_id not in deferred)

  def _provisional_birth_view(self, objects):
    manager = self.tracker.group_manager
    if not objects or not manager.provisional_hidden:
      return objects
    scan_ns = objects[0].timestamp_ns
    ext = self.camera_extended
    strict = {}
    if ext.last_ns == scan_ns:
      strict = {pid: verdict[1] for pid, verdict in ext.last_associations.items()
                if verdict[0] == BOSCH_CAMERA_ASSOC_ASSIGNED and verdict[1] >= 0}
    oem_pids = set(self._debug_processed_pids)
    oem_pids.update(obj.physical_track_id for obj in objects if obj.oem_selected)
    return manager.provisional_publication_view(
      objects, strict_associations=strict, oem_pids=oem_pids)

  def _burst_view(self, objects):
    """Defer this scan's proven burst multi-return surfaces, publication only."""
    burst = self.burst_multireturn
    if not objects or not all(obj.timestamp_ns == burst.last_ns for obj in objects):
      return objects
    return burst.publication_view(objects)

  def publication_view(self, objects, timestamp_ns=None):
    result = self._publication_view_without_road_edge(objects, timestamp_ns)
    result = self.road_edge_filter.publication_view(result, timestamp_ns)
    if BOSCH_UNSUPPORTED_BIRTH_PUBLICATION:
      result = self.unsupported_birth.publication_view(self, result, timestamp_ns)
    if BOSCH_SURFACE_CORNER_GATE:
      result = self.road_edge_filter.publication_surface_view(self, result, timestamp_ns)
    return result

  def _publication_view_without_road_edge(self, objects, timestamp_ns=None):
    """Apply Bosch-only final-publication filters after alias allocation.

    Camera-extended PUBLICATION keeps the existing representative-only output
    when its maturity and current-member conditions are satisfied.
    """
    objects = self._provisional_birth_view(objects)
    p91_suppressed = (self.p91.would_suppress if self.p91.mode == BOSCH_P91_ACTIVE and
                      objects and all(obj.timestamp_ns == self.p91.last_ns for obj in objects) else frozenset())
    if p91_suppressed:
      self.p91.publication_suppressed += len(p91_suppressed)
      objects = tuple(obj for obj in objects if obj.physical_track_id not in p91_suppressed)
    gate = self.oem_gate
    withheld = (gate.would_withhold if gate.mode == BOSCH_OEM_GATE_ACTIVE and
                objects and all(obj.timestamp_ns == gate.last_ns for obj in objects) else frozenset())
    if withheld:
      gate.publication_withheld += len(withheld)
      objects = tuple(obj for obj in objects if obj.physical_track_id not in withheld)
    ext = self.camera_extended
    if ext.mode != BOSCH_CAMERA_EXTENDED_PUBLICATION:
      return self.sidepass_lateral.publication_view(
        self.mirror_m3_shadow.publication_view(
          self.mirror_birth_hold.publication_view(
            self.b5_birth_defer.publication_view(
              self._burst_view(self.family_companion.publication_view(self._final_view(objects)))))))
    if not ext.mature_groups or not objects:
      self.test_last_suppressed = ()
      self.test_last_active_groups = 0
      return self.sidepass_lateral.publication_view(
        self.mirror_m3_shadow.publication_view(
          self.mirror_birth_hold.publication_view(
            self.b5_birth_defer.publication_view(
              self._burst_view(self.family_companion.publication_view(self._final_view(objects)))))))
    # 다른 scan의 tuple 또는 qualification에서 대표가 빠진 그룹은 baseline으로 연다.
    by_pid = {obj.physical_track_id: obj for obj in objects}
    suppressed = set()
    active_groups = 0
    for rep in ext.representatives:
      members = tuple(sorted(rep.members))
      if members not in ext.mature_groups or not all(
          p in by_pid and by_pid[p].timestamp_ns == ext.last_ns for p in members):
        continue
      suppressed.update(p for p in members if p != rep.representative_pid)
      active_groups += 1
    self.test_last_suppressed = tuple(sorted(suppressed))
    self.test_last_active_groups = active_groups
    return self.sidepass_lateral.publication_view(
      self.mirror_m3_shadow.publication_view(
        self.mirror_birth_hold.publication_view(
          self.b5_birth_defer.publication_view(
            self._burst_view(self.family_companion.publication_view(self._final_view(
              tuple(obj for obj in objects if obj.physical_track_id not in suppressed) if suppressed else objects)))))))

  @property
  def slot_to_ids(self):
    return {member.slot: (member.raw_track_id, obj.physical_track_id)
            for obj in self._debug_objects for member in obj.members}

  @property
  def debug_snapshot(self):
    # Replay can request the full mapping explicitly.
    if self._debug_timeout:
      return {'bus': self.bus, 'timeout': True, 'last_scan_timestamp_ns': self.last_scan_timestamp_ns,
              'objects': [], 'slot_to_ids': {}}
    if self._debug_phase_ns is None:
      return {}
    return {
      'bus': self.bus, 'phase_ns': self._debug_phase_ns, 'scan_timestamp_ns': self.last_scan_timestamp_ns,
      'closed_ns': self._debug_closed_ns, 'tick': self._debug_tick, 'complete': self._debug_complete,
      'can_error': self.can_error, 'wrong_config': self.wrong_config,
      'oem_word': self._debug_oem_word, 'oem_selected_slot': self._debug_oem_slot,
      'oem_match_count': self._debug_oem_matches, 'slot_to_ids': self.slot_to_ids,
      'processed_target_word': self._debug_processed_word,
      'processed_target_pids': sorted(self._debug_processed_pids),
      'p91_mode': self.p91.mode, 'p91_would_suppress': sorted(self.p91.would_suppress),
      'p91_decisions': self.p91.decisions,
      'oem_state': self._debug_oem_state, 'oem_word0_active': self._debug_word0_active,
      'oem_word1_active': self._debug_word1_active, 'scc_obj_valid': self._scc_validity(self.last_scan_timestamp_ns or 0),
      'oem_intent_mps2': self._debug_oem_intent, 'oem_gate_mode': self.oem_gate.mode,
      'oem_gate_withhold': sorted(self._debug_gate_suppress), 'oem_gate_reasons': self._debug_gate_reasons,
      'curve_reacquire_mode': self.camera_extended.curve_reacquire_mode,
      'curve_reacquire': self.camera_extended.last_curve_reacquire,
      'curve_reacquire_would': self.camera_extended.last_curve_reacquire_would,
      'curve_reacquire_state_count': len(self.camera_extended.curve_reacquire_histories),
      'provisional_hidden': dict(self.tracker.group_manager.provisional_hidden),
      'provisional_decisions': [vars(decision) for decision in self.tracker.group_manager.last_provisional_decisions],
      'family_companion_suppressed': sorted(self.family_companion.would_suppress),
      'family_companion_decisions': [vars(decision) for decision in self.family_companion.last_decisions],
      'burst_multireturn_mode': self.burst_multireturn.mode,
      'burst_multireturn_suppressed': sorted(self.burst_multireturn.would_suppress),
      'burst_multireturn_decisions': [vars(decision) for decision in self.burst_multireturn.last_decisions],
      'b5_mode': self.b5_birth_defer.mode,
      'b5_suppressed': sorted(self.b5_birth_defer.would_suppress),
      'b5_decisions': [{name: getattr(decision, name) for name in decision.__dataclass_fields__}
                       for decision in self.b5_birth_defer.last_decisions],
      'sidepass_lateral_mode': self.sidepass_lateral.mode,
      'sidepass_lateral_published_y': dict(sorted(self.sidepass_lateral.published_y.items())),
      'sidepass_lateral_decisions': [vars(decision) for decision in self.sidepass_lateral.last_decisions],
      'objects': [{'physicalTrackId': obj.physical_track_id,
                   'rawTrackIds': [member.raw_track_id for member in obj.members],
                   'slots': list(obj.member_slots),
                   'measurement_ns': [member.timestamp_ns for member in obj.members],
                   'representative_rawTrackId': obj.representative_raw_track_id,
                   'representative_slot': next(member.slot for member in obj.members
                                               if member.raw_track_id == obj.representative_raw_track_id),
                   'oem_selected': obj.oem_selected} for obj in self._debug_objects],
    }

  def _ingest_scc(self, timestamp_ns, address, payload):
    """Decode the two stock-SCC observations this provider consumes.

    Both are read as evidence about the OEM's own target decision. A short or
    malformed frame is dropped rather than reported: a missing observation must
    fail open, never fault the radar.
    """
    if len(payload) < 8:
      return
    value = int.from_bytes(bytes(payload), 'little')
    if address == BOSCH_SCC11_ADDR:
      self.scc_obj_valid = bool((value >> BOSCH_SCC11_OBJ_VALID_BIT) & 1)
      self.scc_obj_valid_ns = timestamp_ns
    else:
      self.oem_gate.intent.ingest(timestamp_ns, ((value >> BOSCH_SCC12_AREQ_RAW_BIT) & 0x7FF) * .01 - 10.23)

  def _scc_validity(self, timestamp_ns):
    """SCC11.ObjValid at this scan, or None when the stream is absent/stale."""
    if self.scc_obj_valid is None or timestamp_ns - self.scc_obj_valid_ns > BOSCH_SCC_STALE_NS:
      return None
    return self.scc_obj_valid

  def update(self, can_packets, now_ns: int, v_ego: float, yaw_rate_left=None, vision=(), *, path=(), path_ns=None, path_source_ns=None):
    """Consume (receive_ns, [(address, payload, src), ...]) CAN packets.

    Return None when no window has closed, otherwise only freshly observed
    physical objects. Empty/error updates at 10 Hz after 0.3 s without a scan
    clear downstream points even when CAN reception has stopped completely.
    All closed scans in a batch update association; only the latest is emitted.
    """
    if self._last_now_ns is not None and now_ns < self._last_now_ns:
      raise ValueError('provider receive clock must not regress')
    self._unsupported_birth_context = (now_ns, v_ego, yaw_rate_left, tuple(path), path_ns)
    self._last_now_ns = now_ns
    if self._start_ns is None:
      self._start_ns = now_ns
    # Roughly six thousand frames a second reach this loop and about two hundred
    # are ours, so the reject path stays two indexed comparisons: no unpacking,
    # no attribute lookups and no payload copy until a frame is actually kept.
    bus = self.bus
    camera = self.camera_extended.camera
    camera_bus = self.camera_bus
    scc_bus = self.scc_bus
    frames = self._frames
    anchors = self._anchors
    order = self._order
    for timestamp_ns, messages in can_packets:
      future = timestamp_ns > now_ns
      for message in messages:
        address = message[0]
        if (camera is not None and message[2] == camera_bus and
            BOSCH_CAMERA_HEADER <= address <= BOSCH_CAMERA_LAST_FAMILY):
          if future:
            camera.fault_ns = timestamp_ns
          else:
            camera.ingest(timestamp_ns, address, bytes(message[1]))
          continue
        if message[2] != bus:
          # Stock SCC observation only. It never becomes a detection, an object
          # or an actuator command; it can only withhold an unvalidated one.
          if message[2] == scc_bus and address in BOSCH_SCC_ADDRESSES and not future:
            self._ingest_scc(timestamp_ns, address, message[1])
          continue
        if address < 0x601 or address > 0x612:
          continue
        if future:
          self._pending_error = True
          continue
        payload = bytes(message[1])
        if address == 0x612:
          if len(payload) != 8:
            self._pending_error = True
            continue
          tick = int.from_bytes(payload[1:4], 'little')
          if tick % 10 == 0:
            if self._last_closed_anchor_ns is not None and timestamp_ns <= self._last_closed_anchor_ns:
              continue
            if not any(ns == timestamp_ns for ns, _ in anchors):
              anchors.append((timestamp_ns, tick))
        else:
          frames.append(_BoschCanFrame(timestamp_ns, address, payload, order))
          order += 1
    self._order = order
    if len(self._anchors) > 1:
      self._anchors.sort()
    output = None
    while self._anchors and self._anchors[0][0] + BOSCH_WINDOW_NS <= now_ns:
      phase_ns, tick = self._anchors[0]
      assigned, retained = [], []
      for frame in self._frames:
        closest = (phase_ns if len(self._anchors) == 1 else
                   min(self._anchors, key=lambda anchor: abs(anchor[0] - frame.timestamp_ns))[0])
        if closest == phase_ns and abs(phase_ns - frame.timestamp_ns) <= BOSCH_WINDOW_NS:
          assigned.append(frame)
        else:
          retained.append(frame)
      self._frames = retained
      self._anchors.pop(0)
      self._last_closed_anchor_ns = phase_ns
      # Context must be fresh at receipt and close to this particular scan, also
      # when a batch closes several old windows. No path extrapolation in time.
      # modelV2.logMonoTime is a publication time: the scene the path describes
      # was captured a model-pipeline latency earlier, 43.6 ms at the median on
      # route259. Judge freshness on that capture time when the caller supplies
      # it, so a path is used exactly when its scene precedes this scan and it
      # had already been received. Without the source time the previous
      # publication-time gate stands unchanged.
      if path_ns is None or not (0 <= now_ns - path_ns <= 200_000_000):
        scan_path = ()
      elif path_source_ns:
        scan_path = path if phase_ns - 200_000_000 <= path_source_ns <= phase_ns else ()
      else:
        scan_path = path if phase_ns - 200_000_000 <= path_ns <= phase_ns + BOSCH_WINDOW_NS else ()
      output = self._finish_scan(phase_ns, tick, assigned, v_ego, yaw_rate_left, vision, scan_path)
      self._last_output_ns = now_ns

    # Preserve the pre-anchor half-window. Pending future anchors may already
    # own older frames when update() receives a large batch.
    oldest = (self._anchors[0][0] if self._anchors else now_ns) - BOSCH_WINDOW_NS
    if self._frames:
      self._frames = [frame for frame in self._frames if frame.timestamp_ns >= oldest]
    since = self.last_scan_timestamp_ns if self.last_scan_timestamp_ns is not None else self._start_ns
    if now_ns - since >= BOSCH_STALE_NS and (output is not None or self._last_output_ns is None or now_ns - self._last_output_ns >= BOSCH_OUTPUT_INTERVAL_NS):
      self.can_error = True
      self._debug_objects = ()
      self._debug_timeout = True
      self.tracker.group_manager.reset_provisional(now_ns, 'provider_timeout')
      self.tracker.group_manager.reset_common_ancestry()
      self.companion_pairs = {}
      self._companion_shared_start = {}
      self._companion_hidden = {}
      self._companion_ns = None
      self._companion_prev_ns = None
      self.last_companion_deferred = ()
      self.family_companion.reset(now_ns, 'STATE_RESET')
      self.burst_multireturn.reset(now_ns, 'STATE_RESET')
      self.b5_birth_defer.reset('PROVIDER_TIMEOUT')
      self.mirror_birth_hold.reset(now_ns, 'PROVIDER_TIMEOUT')
      self.mirror_m3_shadow.reset(now_ns, 'PROVIDER_TIMEOUT')
      self.road_edge_filter.reset()
      self.sidepass_lateral.reset(now_ns, 'PROVIDER_TIMEOUT')
      self.p91.update((), now_ns, v_ego, yaw_rate=yaw_rate_left)
      self.oem_gate.update((), now_ns, v_ego, state=BOSCH_OEM_STATE_NONE)
      self._debug_gate_suppress = frozenset()
      self._debug_gate_reasons = {}
      self._last_output_ns = now_ns
      return ()
    return output

  def _finish_scan(self, phase_ns, tick, frames, v_ego, yaw_rate_left, vision, path=()):
    bucket = {}
    malformed = self._pending_error or any(len(frame.payload) != 8 for frame in frames)
    self._pending_error = False
    for frame in sorted(frames, key=lambda frame: (frame.timestamp_ns, frame.order)):
      if len(frame.payload) != 8:
        continue
      previous = bucket.get(frame.address)
      if previous is None or abs(frame.timestamp_ns - phase_ns) < abs(previous.timestamp_ns - phase_ns):
        bucket[frame.address] = frame
    complete = BOSCH_TRACK_ADDRESSES.issubset(bucket)
    availability_ns = max([phase_ns] + [frame.timestamp_ns for address, frame in bucket.items() if address in BOSCH_TRACK_ADDRESSES])
    if not complete:
      availability_ns = max(availability_ns, phase_ns + BOSCH_WINDOW_NS)
    detections = []
    unsupported = False
    for frame in sorted(bucket.values(), key=lambda frame: (frame.timestamp_ns, frame.order)):
      if frame.address not in BOSCH_TRACK_ADDRESSES:
        continue
      decoded, unknown = bosch_decode_frame(frame.timestamp_ns, frame.address, frame.payload)
      detections.extend(decoded)
      unsupported |= unknown
    # A delayed or overlapping scan must not turn an old sample into a fresh
    # measurement. The normal 100 ms phase cadence has disjoint 40 ms windows.
    previous_ns = self.tracker.raw_manager.last_timestamp_ns
    if previous_ns is not None:
      if availability_ns <= previous_ns:
        self.can_error = True
        self._debug_objects = ()
        return ()
      fresh = [detection for detection in detections if detection.timestamp_ns > previous_ns]
      malformed |= len(fresh) != len(detections)
      detections = fresh
    oem = bucket.get(0x601)
    oem_word = int.from_bytes(oem.payload[4:8], 'little') if oem else None
    processed_word = int.from_bytes(oem.payload[0:4], 'little') if oem else None
    matches = [detection.slot for detection in detections if oem_word == detection.raw_word]
    oem_slot = matches[0] if matches else None
    word1_active = oem_word is not None and oem_word != BOSCH_INACTIVE_WORD and not oem_word & (1 << 31)
    word0_active = (processed_word is not None and processed_word != BOSCH_INACTIVE_WORD and
                    not processed_word & (1 << 31))
    oem_state = _BoschOemValidationGate.classify(word0_active, word1_active)
    retention = self.tracker.group_manager.mature_retention
    if retention is not None:
      retention.ingest_path(path, availability_ns)
    objects = self.tracker.update(availability_ns, detections, yaw_rate=yaw_rate_left,
                                  v_ego=v_ego, oem_slot=oem_slot, vision=vision)
    self.last_scan_timestamp_ns = availability_ns
    self.can_error = not complete or malformed or unsupported
    self.wrong_config = unsupported
    self._debug_objects = objects
    self._debug_phase_ns = phase_ns
    self._debug_closed_ns = self._last_now_ns
    self._debug_tick = tick
    self._debug_complete = complete
    self._debug_oem_word = oem_word
    self._debug_oem_slot = oem_slot
    self._debug_oem_matches = len(matches)
    self._debug_processed_word = processed_word
    self._debug_timeout = False
    self.camera_extended.update(availability_ns, objects, v_ego, yaw_rate_left)
    # 모든 모드에서 qualifier 이력과 alias 할당에는 동일한 physical 집합을 전달한다.
    # P91 ACTIVE veto는 allocator 이후 최종 publication에만 적용한다.
    qualified = (self.qualifier.update(objects, availability_ns, v_ego, path, yaw_rate=yaw_rate_left)
                 if self.qualifier is not None else objects)
    processed_pids = _bosch_match_processed_target_word(qualified, processed_word)
    self._debug_processed_pids = processed_pids
    scc_valid = self._scc_validity(availability_ns)
    self._debug_word0_active = word0_active
    self._debug_word1_active = word1_active
    self._debug_oem_state = oem_state
    self.last_oem_state = oem_state
    self.last_oem_slot = oem_slot
    strict = {pid: verdict[1] for pid, verdict in self.camera_extended.last_associations.items()
              if verdict[0] == BOSCH_CAMERA_ASSOC_ASSIGNED and verdict[1] >= 0}
    family_oem_pids = set(processed_pids)
    family_oem_pids.update(obj.physical_track_id for obj in qualified if obj.oem_selected)
    s32_pids = {pid for pair in self.tracker.group_manager.provisional_pid_pairs.values() for pid in pair}
    self.family_companion.update(
      qualified, availability_ns, v_ego, yaw_rate=yaw_rate_left,
      strict_associations=strict, oem_pids=family_oem_pids, excluded_pids=s32_pids, path=path)
    # Burst multi-return defer reads the same scan the B1 family filter just read.
    # Only a word0 record the OEM also validated counts as identity evidence here,
    # for the reason P91 uses below: word0 alone has been observed on a clone.
    self.burst_multireturn.update(
      qualified, availability_ns, v_ego, yaw_rate=yaw_rate_left,
      word0_validated_pids=processed_pids if oem_state == BOSCH_OEM_STATE_VALIDATED
      else frozenset())
    self.oem_gate.update(qualified, availability_ns, v_ego, state=oem_state,
                         word0_pids=processed_pids, oem_valid=scc_valid)
    self._debug_gate_suppress = self.oem_gate.would_withhold
    self._debug_gate_reasons = self.oem_gate.reasons
    self._debug_oem_intent = self.oem_gate.intent.median(availability_ns)
    # word0 alone no longer counts as P91 support. Only a word0 record that the
    # OEM also validated in the same scan can clear a clone suspicion.
    p91_word0 = processed_pids if oem_state == BOSCH_OEM_STATE_VALIDATED and scc_valid is not False else frozenset()
    self.p91.update(qualified, availability_ns, v_ego, word0_pids=p91_word0, yaw_rate=yaw_rate_left)
    # B5 reads the complete post-grouping tuple and the already-computed
    # camera/OEM evidence, but no existing filter reads B5 state. Its action is
    # applied only after alias allocation in publication_view().
    self.b5_birth_defer.update(
      qualified, self.tracker.last_raw_tracks, availability_ns, self._last_now_ns,
      v_ego, yaw_rate_left,
      camera_associations=self.camera_extended.last_associations,
      oem_state=oem_state, word0_pids=processed_pids,
      live_pids=self.tracker.group_manager.states,
      live_raw_ids=self.tracker.raw_manager._states)
    # M1 reads the exact qualified tuple and scan evidence already seen by B5.
    # Its publication hold is independent and cannot feed back into B5 or tracking.
    self.mirror_birth_hold.update(
      qualified, self.tracker.last_raw_tracks, availability_ns, v_ego, yaw_rate_left,
      camera_associations=self.camera_extended.last_associations,
      word0_pids=processed_pids, live_pids=self.tracker.group_manager.states)
    self.mirror_m3_shadow.update(
      availability_ns, v_ego, self.mirror_birth_hold.last_mirror_objects,
      self.mirror_birth_hold.last_decisions_by_pid,
      camera_associations=self.camera_extended.last_associations,
      word0_pids=processed_pids,
      mirror_enabled=self.mirror_birth_hold.mode != BOSCH_MIRROR_BIRTH_OFF,
      live_pids=self.tracker.group_manager.states)
    self.road_edge_filter.update(
      qualified, availability_ns, valid=not self.can_error and not self.wrong_config,
      camera_associations=self.camera_extended.last_associations, word0_pids=processed_pids)
    # Side-pass lateral estimate: same scan, same OEM evidence the other
    # publication stages read; applied only by publication_view().
    self.sidepass_lateral.update(
      qualified, availability_ns, v_ego, path=path,
      oem_pids=processed_pids if oem_state == BOSCH_OEM_STATE_VALIDATED else frozenset())
    return qualified

# End Bosch MRRevo14F passive radar
