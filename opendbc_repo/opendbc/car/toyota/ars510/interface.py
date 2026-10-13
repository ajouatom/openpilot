"""openpilot-shaped radar interface on the native 0x80 decode.

Feed it raw CAN frames `(time_s, bus, address, data)`. It reassembles 0x80 records on the radar bus, checks
the record CRC, decodes every occupied slot, and returns RadarData-shaped dicts:

    {"time_s": t, "radarData": {"points": [...], "errors": {...}}}

with points carrying dRel (m, forward), yRel (m, LEFT positive), vRel (m/s, over-ground velocity minus ego
speed) and trackId (from the radar's own slot / age lifecycle).

Ego speed comes from Toyota SPEED (0xB4) on the car bus, or from `set_ego_speed`. With a fresh ego speed vRel is
over-ground speed minus ego speed; otherwise it is NaN, and BASE_CONFIG withholds such points (a NaN would stay in
radard's per-track Kalman for good).

Configs (docs/12 explains the filter, docs/11 compares the profiles with vision only). The default install profile
`acc` (the radar's ACC target alone) lives in upstream/ars510_acc_target.py; the configs below are the all-tracks
decode and its filtered variants:
  FUSED_CONFIG       the `fused` install profile (openpilot/install.py --profile fused): one Kalman speed filter per
                     track fusing the object list and the radar's ACC target, each weighted by its own uncertainty,
                     plus a yaw-rate path gate
  COLORED_CONFIG     experimental: fused with a colored-noise (bias) state for the object list
  BASE_CONFIG        the unfiltered radar decode with only what radard needs (the 'raw' install profile)
  ALL_TRACKS_CONFIG  every valid track from age 1 with the radar's own IDs: the decode-level view for analysis
"""
from __future__ import annotations

from dataclasses import dataclass, replace
from math import exp, isfinite, nan, pi
from collections.abc import Iterable

from .constants import (ACC_TARGET_POS_ADDR, ACC_TARGET_VREL_ADDR, CAR_BUS, ID80_ADDR, RADAR_BUS, SUMMARY_ADDRS,
                        TOYOTA_KINEMATICS_ADDR, TOYOTA_SPEED_ADDR)
from .objects import NativeObject, decode_native_slot
from .record import id80_crc_ok, occupied_slots
from .support import parse_0x192, parse_acc_target_position, parse_acc_target_vrel
from .tracks import NativeTrackIdAssigner
from .transport import Id80RecordAssembler

EGO_ACCEL_TAU_S = 0.3  # smoothing of the ego-speed derivative used for aRel
ACCEL_SCALE = 0.04  # m/s^2 per code of the filtered over-ground acceleration 84|10 (docs/03)
GUARD_ID_STRIDE = 10_000_000  # added to a trackId after a guard episode, so radard restarts that track's filter
RELINK_MIN_LOST_AGE = 30
RELINK_MIN_PUBLISH_AGE = 6


@dataclass(frozen=True)
class NativeInterfaceConfig:
    radar_bus: int = RADAR_BUS
    ego_speed_bus: int = CAR_BUS
    max_ego_speed_age_s: float = 0.5
    include_metadata: bool = True
    # Hold back tracks younger than this many cycles. Young tracks carry unconverged range and velocity
    # (stopped cars read as moving; one lead read about 39 m while ~100 m away and walked out over about 5 s). 60 cycles ~ 3.6 s.
    min_publish_age: int = 1
    # When a new track is first published, give it the trackId of a settled track (age >= 30) lost within
    # this many seconds if it starts where that track predicts and the match is one-to-one. 0 disables.
    # Requires min_publish_age >= RELINK_MIN_PUBLISH_AGE so the lost ID has ended (IDs stay unique).
    relink_max_gap_s: float = 0.0
    # Multiplies the decoded over-ground velocity. Against Toyota 0xB4, steady following reads 0.149 m/s/code
    # instead of nominal 0.150 in the original fit. This empirical 0.667% profile setting is
    # separate from physical calibration and the ~1-1.5% discrepancy between ego references (docs/06).
    # Use 1.0 with a carState.vEgo or GPS ego reference.
    vground_scale: float = 1.0
    # Withhold points whose vRel rests on a stale ego speed.
    drop_unresolved_vrel: bool = False
    # Velocity-aided range: predict dRel with vRel, correct toward measured dRel with this gain (0 disables).
    range_fusion_gain: float = 0.0
    # Saturation guard: withhold a mature track's point while its velocity code is the invalid 1023 (or 0), and after
    # that until the velocity is back within sat_recover_mps of the last good value (the sentinel decays over ~6
    # records) or guard_hold_s has passed. The track then continues under a new trackId (docs/12). The speed filter's
    # robust update absorbs the sentinel on its own, so FUSED_CONFIG leaves this off.
    drop_saturated_codes: bool = False
    sat_recover_mps: float = 5.0
    guard_hold_s: float = 1.0
    guard_min_age: int = 60  # younger tracks converge on their own and stay unguarded
    # The radar's own trackers, used as extra speed readings by the speed filter:
    # ACC target (0x235 / 0x237): matched to an object-list track by position, cost = |dRel - x| / max(acc_match_range_m,
    # acc_match_range_frac * x) + |yRel - y| / 0.5 < 1 with margin > 1 over the next track, for tracks of age >=
    # acc_match_min_age (x is the ACC target's 0.025 m distance, y its 0.01 m lateral (the object list's lateral code is 0.015 m); the range scale grows with range
    # because the object list reads far cars short of the ACC distance, by 15-20 m for some newly detected cars, docs/06,
    # docs/12). The association is kept while the track
    # and a continuous ACC target persist, even when the object-list position slides away with an excursion; it re-matches
    # when the ACC target jumps (range > acc_sticky_jump_m or lateral > 1 m between updates), the track disappears, or its
    # cost exceeds acc_sticky_max_cost. The associated track takes the ACC distance as the measurement of the range
    # fusion (acc_range): the radar's own smooth range, which the vision lead agrees with.
    acc_target_max_age_s: float = 0.1
    acc_match_range_m: float = 12.0
    acc_match_range_frac: float = 0.4
    acc_range: bool = True
    acc_match_min_age: int = 20
    acc_sticky_jump_m: float = 8.0
    acc_sticky_max_cost: float = 4.0
    # Path gate (radard weights lateral position only through the camera's uncertainty and checks only distance and
    # speed, so far out it can pair the vision lead with a car a lane over): beyond
    # path_gate_min_range_m, a track other than the ACC target's that is more than path_gate_m from the ego path predicted
    # from yaw rate and speed (constant curvature: y - yaw / v * d^2 / 2) is withheld: a car a lane over, also on a
    # curve. Closer in, cars moving into the lane must stay visible.
    path_gate_m: float = 0.0  # 0 = off (fused: 2.5)
    path_gate_min_range_m: float = 15.0
    # Far cars come from the camera (docs/12 "Far cars"): beyond far_acc_only_m only the ACC target's track is
    # published, as Toyota's own ACC acts on its ACC target alone. Far non-ACC tracks carry the object list's false
    # closings. 0 = off (fused: 60)
    far_acc_only_m: float = 0.0
    # Stationary tracks (|over-ground speed| < path_gate_stationary_mps) within path_gate_min_range_m are path-gated to
    # path_gate_stationary_m of the path curved with max(v_ego, 2 m/s): parked cars and roadside objects close in stay out
    # of the lane, where radard's low-speed rule would otherwise stop for them as the path curves away. 0 = off (fused: 0.5 m/s, 1.5 m)
    path_gate_stationary_mps: float = 0.0
    path_gate_stationary_m: float = 1.5
    # Summaries (0x192 / 0x194): the positions of the radar's selected targets. Their range slope over summary_window_s
    # gives a relative speed; a summary attaches to the track at its position (range within max(5 m, 15 %), lateral within
    # summary_match_lat_m, unambiguous) and stays attached while the track stays within 25 % in range and twice that
    # lateral limit. Used up to summary_max_range_m (the optical check favours the summary speed out to ~80 m, docs/05);
    # the ACC-associated track is left to the ACC target. A fork option: off in FUSED_CONFIG, on in COLORED_CONFIG.
    summary_max_range_m: float = 80.0  # summary_sigma_mps <= 0 below turns the summaries off
    summary_window_s: float = 1.0
    summary_match_lat_m: float = 1.0
    summary_scales: tuple[tuple[float, float], ...] = ((0.0625, -10.0), (0.0625, -10.0))  # m per code, offset m (0x192, 0x194)
    # Fused speed filter (docs/12 "The model"): one per-track Kalman filter on the over-ground speed. Each
    # reading is weighted by
    # its own standard deviation: the object-list speed by speed_sigma_per_code * 240|7 (the radar's velocity-error
    # scale, calibrated against its ACC target), times young_sigma_scale for tracks younger than young_age (measured:
    # young tracks err more than 240|7 says); the radar's ACC target speed for the associated track by acc_sigma_mps;
    # the target-range summary speed (option, up to summary_max_range_m) by summary_sigma_mps. The lead's speed may change
    # with lead_accel_std_mps2 (process noise); a reading beyond innovation_gate_sigma standard deviations is clamped
    # to that bound (robust update). A track is first published once its speed std is below publish_speed_std_mps.
    # Range stays outside the filter (the object list's range rate and speed disagree by 10-20 %, docs/12);
    # range_fusion_gain smooths the published range separately.
    fused_speed_filter: bool = False
    speed_sigma_per_code: float = 0.045
    young_sigma_scale: float = 1.8
    young_age: int = 100
    acc_sigma_mps: float = 0.5
    summary_sigma_mps: float = 0.5
    lead_accel_std_mps2: float = 1.5
    innovation_gate_sigma: float = 3.0
    publish_speed_std_mps: float = 0.75
    # Colored measurement noise (state augmentation): the object-list speed error persists for ~1.2 s (lag-1
    # autocorrelation 0.95 per 60 ms record), so the filter also estimates it as a slowly varying bias b
    # (first-order Gauss-Markov, time constant speed_bias_tau_s, stationary sd speed_bias_sigma_per_code x scale);
    # the object list measures v + b, the ACC target and summaries measure v. 0 = off (white noise only).
    # The noise scale of a reading is 240|7 x (1 + speed_sigma_ego_gain x v_ego / 30 m/s)
    #   x (1 + speed_sigma_accel_gain x |84|10| / 100): errors grow with ego speed and with the radar's acceleration.
    speed_bias_tau_s: float = 0.0
    speed_bias_sigma_per_code: float = 0.0
    speed_sigma_ego_gain: float = 0.0
    speed_sigma_accel_gain: float = 0.0
    # Colored filter only: a new track's first reading gets this extra sd per 240|7 count (first readings are often
    # far off, 240|7 near 160), and a tracker reading (ACC target, summary) beyond the innovation gate means the
    # speed state has diverged: its variance is widened to the innovation before the update (covariance inflation).
    speed_init_sigma_per_code: float = 0.0
    speed_divergence_inflation: bool = False


# Every valid track, radar's own IDs: the decode-level view.
ALL_TRACKS_CONFIG = NativeInterfaceConfig(min_publish_age=1, relink_max_gap_s=0.0)
# The unfiltered radar decode with only the validity rules radard needs: the "raw" install profile (docs/08).
BASE_CONFIG = NativeInterfaceConfig(
    min_publish_age=60, relink_max_gap_s=3.5, vground_scale=0.149 / 0.15, drop_unresolved_vrel=True,
    drop_saturated_codes=True,
)
# The `fused` install profile (docs/12, docs/11; the default is `acc`): the base decode plus range fusion and one uncertainty-weighted speed
# filter per track that fuses the object list and the radar's ACC target (matched by position, 0.4 x range scale; its
# track takes the ACC distance as range; the ACC speed is weighted at 0.13 m/s, acc_sigma_mps). The track-ID relink and the saturation guard of the base decode are off:
# with the filter on, the driving is the same with them on or off (docs/12). Beyond 15 m, tracks other than the ACC target's that sit
# more than 2.5 m from the yaw-predicted path are withheld (docs/12 "Path gate"). The summaries are off. The openpilot
# version (upstream/ars510_radar.py) is this profile in one file.
FUSED_CONFIG = replace(BASE_CONFIG, range_fusion_gain=0.1, relink_max_gap_s=0.0, drop_saturated_codes=False,
                       fused_speed_filter=True, path_gate_m=2.5, summary_sigma_mps=0.0, acc_sigma_mps=0.13, far_acc_only_m=60.0,
                       path_gate_stationary_mps=0.5)

# Experimental (fork only, docs/12 "Kalman variants tested"): fused with the object-list error as a 1.2 s Gauss-Markov
# bias state, noise scaled by ego speed and the radar's acceleration reading (fitted on the hidden-ACC teacher). Offline
# bench at the 0.125 m/s ACC unit: 6.8 % / 7.8 % false-closing cycles against 7.6 % / 8.0 % for fused on held-out / fresh
# drives (kalman_variants.json); it holds a far excursion that recovers longer than fused does.
COLORED_CONFIG = replace(FUSED_CONFIG, speed_sigma_per_code=0.001, speed_bias_tau_s=1.19, speed_bias_sigma_per_code=0.0096,
                         speed_sigma_ego_gain=0.48, speed_sigma_accel_gain=0.51, young_sigma_scale=1.0,
                         lead_accel_std_mps2=1.0, speed_init_sigma_per_code=0.045, speed_divergence_inflation=True,
                         summary_sigma_mps=0.5, acc_sigma_mps=0.5)

NATIVE_VREL_STATUS = "native_over_ground_minus_ego"
UNRESOLVED_NAN = "unresolved_nan"


def parse_toyota_yaw_rate(data: bytes) -> float | None:
    """Toyota KINEMATICS (0x24) YAW_RATE, big-endian 1|10 at 0.244 deg/s, offset -125: rad/s, left positive."""
    if len(data) < 2:
        return None
    raw = ((data[0] & 0x03) << 8) | data[1]
    return (raw * 0.244 - 125.0) * pi / 180.0


def parse_toyota_speed_mps(data: bytes) -> float | None:
    """Toyota SPEED (0xB4): bytes 5-6 big-endian, 0.01 km/h."""
    if len(data) < 7:
        return None
    return int.from_bytes(data[5:7], "big") * 0.01 / 3.6


class Ars510NativeRadarInterface:
    def __init__(self, config: NativeInterfaceConfig | None = None) -> None:
        self.config = config or NativeInterfaceConfig()
        self._assembler = Id80RecordAssembler()
        self._tracks = NativeTrackIdAssigner()
        self._v_ego: float | None = None
        self._v_ego_time_s: float | None = None
        self._a_ego: float | None = None  # smoothed derivative of ego speed
        self._yaw: tuple[float, float] | None = None  # (time, yaw rate rad/s, left positive)
        self.crc_failures = 0
        self.records = 0
        self.settling_suppressed = 0
        self.relinks = 0
        self.unresolved_suppressed = 0
        self._first: dict[int, tuple[float, float, float]] = {}  # native id -> (t, x, y) at first age>=2 sample
        self._last: dict[int, tuple[float, float, float, float, int]] = {}  # -> (t, x, y, vrel, age) latest
        self._out_id: dict[int, int] = {}
        self._claimed: set[int] = set()
        self._range_est: dict[int, tuple[float, float, float]] = {}
        self._fused: dict[int, tuple[float, float, float]] = {}  # tid -> (t, over-ground speed, variance)
        self._acc_vrel: tuple[float, float] | None = None  # (time, closing speed)
        self._acc_pos: tuple[float, float, float] | None = None  # (time, x, y)
        self._acc_assoc: tuple[int, float, float] | None = None  # (tid, last x, last y)
        self.path_gated = 0
        self._summary_hist: dict[int, list[tuple[float, float]]] = {0x192: [], 0x194: []}  # (time, range m)
        self._summary_assoc: dict[int, int | None] = {0x192: None, 0x194: None}
        self._summary_y: dict[int, float] = {0x192: 0.0, 0x194: 0.0}  # latest summary lateral, m
        self._guard_state: dict[int, list] = {}  # tid -> [t_last, ref_v, episode_start | None, saturated_in_episode]
        self._guard_gen: dict[int, int] = {}
        self.guard_rejected = 0

    # ---- inputs -----------------------------------------------------------------------------------
    def set_ego_speed(self, v_ego_mps: float, time_s: float) -> None:
        if isfinite(v_ego_mps) and isfinite(time_s):
            if self._v_ego is not None and self._v_ego_time_s is not None and 0.0 < time_s - self._v_ego_time_s < 0.5:
                dt = time_s - self._v_ego_time_s
                a = (v_ego_mps - self._v_ego) / dt
                k = 1.0 - exp(-dt / EGO_ACCEL_TAU_S)
                self._a_ego = a if self._a_ego is None else self._a_ego + k * (a - self._a_ego)
            else:
                self._a_ego = None
            self._v_ego, self._v_ego_time_s = float(v_ego_mps), float(time_s)

    def _fresh_ego_speed(self, time_s: float) -> float | None:
        if self._v_ego is None or self._v_ego_time_s is None:
            return None
        if abs(time_s - self._v_ego_time_s) > self.config.max_ego_speed_age_s:
            return None
        return self._v_ego

    def _fresh_yaw_rate(self, time_s: float) -> float | None:
        if self._yaw is None or abs(time_s - self._yaw[0]) > self.config.max_ego_speed_age_s:
            return None
        return self._yaw[1]

    def update_frame(self, time_s: float, bus: int, addr: int, data: bytes) -> dict | None:
        if bus == self.config.ego_speed_bus and addr == TOYOTA_KINEMATICS_ADDR:
            w = parse_toyota_yaw_rate(bytes(data))
            if w is not None:
                self._yaw = (float(time_s), w)
            return None
        if bus == self.config.ego_speed_bus and addr == TOYOTA_SPEED_ADDR:
            v = parse_toyota_speed_mps(bytes(data))
            if v is not None:
                self.set_ego_speed(v, time_s)
            return None
        if bus == self.config.radar_bus and addr in SUMMARY_ADDRS:
            if self.config.fused_speed_filter and self.config.summary_sigma_mps > 0:
                self._summary_update(addr, float(time_s), bytes(data))
            return None
        if bus == self.config.radar_bus and addr in (ACC_TARGET_VREL_ADDR, ACC_TARGET_POS_ADDR):
            data = bytes(data)
            available = len(data) == 8 and (
                bool(data[1] & 4) if addr == ACC_TARGET_VREL_ADDR
                else data[2:] != bytes.fromhex("003E80000000")
            )
            if not available:
                # Idle payloads decode numerically; target loss clears both cached halves.
                self._acc_vrel = self._acc_pos = None
                return None
            if addr == ACC_TARGET_VREL_ADDR:
                v = parse_acc_target_vrel(bytes(data))
                self._acc_vrel = (time_s, v) if v is not None else self._acc_vrel
            else:
                p = parse_acc_target_position(bytes(data))
                self._acc_pos = (time_s, p[0], p[1]) if p is not None else self._acc_pos
            return None
        if bus != self.config.radar_bus or addr != ID80_ADDR:
            return None
        complete = self._assembler.push(time_s, bytes(data))
        if complete is None:
            return None
        if not id80_crc_ok(complete.payload):
            self.crc_failures += 1
            return None
        self.records += 1
        return self._payload(complete.time_s, complete.payload)

    def update_many(self, frames: Iterable[tuple[float, int, int, bytes]]) -> list[dict]:
        out = []
        for time_s, bus, addr, data in frames:
            p = self.update_frame(time_s, bus, addr, data)
            if p is not None:
                out.append(p)
        return out

    # ---- track re-link ----------------------------------------------------------------------------
    def _relink(self, nid: int, time_s: float, seen: set[int]) -> int:
        cfg = self.config
        if cfg.relink_max_gap_s <= 0 or cfg.min_publish_age < RELINK_MIN_PUBLISH_AGE or nid not in self._first:
            return nid

        def matches(first: tuple[float, float, float], lid: int) -> bool:
            t0, x0, y0 = first
            tl, xl, yl, vl, al = self._last[lid]
            gap = t0 - tl
            if al < RELINK_MIN_LOST_AGE or not (0.0 < gap <= cfg.relink_max_gap_s):
                return False
            x_pred = xl + (vl if isfinite(vl) else 0.0) * gap
            return abs(x0 - x_pred) <= max(3.0, 0.10 * xl) + 1.0 * gap and abs(y0 - yl) <= 1.2 + 0.5 * gap

        lost = [lid for lid in self._last if lid not in seen and lid != nid and lid not in self._claimed
                and time_s - self._last[lid][0] > self._tracks.max_gap_s]
        cands = [lid for lid in lost if matches(self._first[nid], lid)]
        if len(cands) != 1:
            return nid
        lid = cands[0]
        # ambiguity: another unpublished new track also starts where the lost one predicts
        if any(o != nid and o not in self._out_id and o in self._first and matches(self._first[o], lid)
               for o in self._last if o in seen):
            self._claimed.add(lid)  # nobody inherits an ambiguous ID
            return nid
        self._claimed.add(lid)
        self.relinks += 1
        return self._out_id.get(lid, lid)

    # ---- the radar's own trackers -------------------------------------------------------------------
    def _acc_target_match(self, time_s: float, decoded: list) -> tuple[int | None, float]:
        """The track the radar's ACC target describes (see the config) and the target's relative speed."""
        cfg = self.config
        if not cfg.fused_speed_filter or self._acc_vrel is None or self._acc_pos is None:
            self._acc_assoc = None
            return None, nan
        if abs(time_s - self._acc_vrel[0]) > cfg.acc_target_max_age_s or abs(time_s - self._acc_pos[0]) > cfg.acc_target_max_age_s:
            self._acc_assoc = None
            return None, nan
        _, ax, ay = self._acc_pos
        rs = max(cfg.acc_match_range_m, cfg.acc_match_range_frac * ax)
        costs = sorted((abs(obj.d_rel - ax) / rs + abs(obj.y_rel - ay) / 0.5, tid, obj.age)
                       for _, obj, tid in decoded if obj.geometry_valid and obj.lateral_valid)
        if self._acc_assoc is not None:
            tid0, ax0, ay0 = self._acc_assoc
            own = next((c for c, tid, _ in costs if tid == tid0), None)
            if own is not None and own < cfg.acc_sticky_max_cost and abs(ax - ax0) <= cfg.acc_sticky_jump_m \
                    and abs(ay - ay0) <= 1.0:
                self._acc_assoc = (tid0, ax, ay)
                return tid0, self._acc_vrel[1]
            self._acc_assoc = None
        if not costs or costs[0][0] >= 1.0 or costs[0][2] < cfg.acc_match_min_age:
            return None, nan
        if len(costs) > 1 and costs[1][0] - costs[0][0] <= 1.0:
            return None, nan
        self._acc_assoc = (costs[0][1], ax, ay)
        return costs[0][1], self._acc_vrel[1]

    def _summary_update(self, addr: int, time_s: float, data: bytes) -> None:
        hist = self._summary_hist[addr]
        word = parse_0x192(data)
        if word is None:
            hist.clear(); self._summary_assoc[addr] = None
            return
        scale, offset = self.config.summary_scales[0 if addr == 0x192 else 1]
        x = word.range_code13 * scale + offset
        if hist and (time_s - hist[-1][0] > 0.3 or abs(x - hist[-1][1]) > 5.0):  # gap or a new target
            hist.clear(); self._summary_assoc[addr] = None
        hist.append((time_s, x))
        self._summary_y[addr] = word.y_rel
        while hist and hist[0][0] < time_s - self.config.summary_window_s:
            hist.pop(0)

    def _summary_speed(self, addr: int, time_s: float) -> tuple[float, float] | None:
        """(range m, relative speed m/s) of a summary from a least-squares slope, or None while too short/stale."""
        hist = self._summary_hist[addr]
        w = self.config.summary_window_s
        if len(hist) < 8 or hist[-1][0] - hist[0][0] < 0.7 * w or time_s - hist[-1][0] > 0.3:
            return None
        n = len(hist); mt = sum(t for t, _ in hist) / n; mx = sum(x for _, x in hist) / n
        den = sum((t - mt) ** 2 for t, _ in hist)
        return (hist[-1][1], sum((t - mt) * (x - mx) for t, x in hist) / den) if den > 0 else None

    def _summary_match(self, time_s: float, decoded: list, v_ego: float | None, acc_tid: int | None) -> dict[int, float]:
        """Tracks attached to a summary -> summary relative speed (see the config)."""
        cfg = self.config
        out: dict[int, float] = {}
        if v_ego is None:
            return out
        tracks = {tid: obj for _, obj, tid in decoded if obj.geometry_valid and obj.lateral_valid and obj.age >= 20}
        for addr in SUMMARY_ADDRS:
            sv = self._summary_speed(addr, time_s)
            if sv is None:
                self._summary_assoc[addr] = None
                continue
            x, v = sv
            y = self._summary_y[addr]
            tid = self._summary_assoc[addr]
            if tid is not None and (tid not in tracks or abs(tracks[tid].d_rel - x) > max(8.0, 0.25 * x)
                                    or abs(tracks[tid].y_rel - y) > 2.0 * cfg.summary_match_lat_m):
                tid = None
            if tid is None:
                cands = sorted((abs(o.d_rel - x), t) for t, o in tracks.items()
                               if abs(o.y_rel - y) < cfg.summary_match_lat_m and abs(o.d_rel - x) < max(5.0, 0.15 * x))
                if len(cands) == 1 or (len(cands) > 1 and cands[1][0] - cands[0][0] > 3.0):
                    tid = cands[0][1]
            self._summary_assoc[addr] = tid
            if tid is not None and tid != acc_tid and tid not in out:
                out[tid] = v
        return out

    def _fused_speed(self, tid: int, time_s: float, obj: NativeObject,
                     extra: list[tuple[float, float]], v_ego: float = 0.0) -> tuple[float, float]:
        """Fused over-ground speed and its standard deviation (see fused_speed_filter in the config)."""
        cfg = self.config
        young = 1.0
        if obj.age < cfg.young_age:
            w = min(max((cfg.young_age - obj.age) / max(cfg.young_age - 60, 1), 0.0), 1.0)
            young = 1.0 + (cfg.young_sigma_scale - 1.0) * w
        if cfg.speed_bias_tau_s > 0:
            scale = max(obj.vel_unc_code, 1) * young * (1.0 + cfg.speed_sigma_ego_gain * v_ego / 30.0) \
                * (1.0 + cfg.speed_sigma_accel_gain * abs(obj.accel_like_code) / 100.0)
            return self._fused_speed_colored(tid, time_s, obj, extra, scale)
        sig = cfg.speed_sigma_per_code * max(obj.vel_unc_code, 1)
        if obj.age < cfg.young_age:
            sig *= young
        vg = obj.v_long_ground * cfg.vground_scale
        st = self._fused.get(tid)
        if st is None or len(st) != 3 or not 0.0 < time_s - st[0] <= 0.5:
            self._fused[tid] = (time_s, vg, sig * sig)
            return vg, sig
        t0, v, p = st
        p += (cfg.lead_accel_std_mps2 * (time_s - t0)) ** 2
        for z, r in [(vg, sig * sig)] + [(z, s * s) for z, s in extra]:
            s = p + r
            innov = z - v
            lim = cfg.innovation_gate_sigma * s ** 0.5
            if cfg.innovation_gate_sigma > 0 and abs(innov) > lim:
                innov = lim if innov > 0 else -lim
            k = p / s
            v += k * innov
            p *= 1.0 - k
        self._fused[tid] = (time_s, v, p)
        return v, p ** 0.5

    def _fused_speed_colored(self, tid: int, time_s: float, obj: NativeObject,
                             extra: list[tuple[float, float]], scale: float) -> tuple[float, float]:
        """Speed filter with the object-list error as a first-order Gauss-Markov bias state (state [v, b])."""
        cfg = self.config
        sw = cfg.speed_sigma_per_code * scale
        sb = cfg.speed_bias_sigma_per_code * scale
        vg = obj.v_long_ground * cfg.vground_scale
        st = self._fused.get(tid)
        if st is None or len(st) != 6 or not 0.0 < time_s - st[0] <= 0.5:
            p00 = sw * sw + sb * sb + (cfg.speed_init_sigma_per_code * scale) ** 2
            self._fused[tid] = (time_s, vg, 0.0, p00, 0.0, sb * sb)
            return vg, p00 ** 0.5
        t0, v, b, p00, p01, p11 = st
        dt = time_s - t0
        phi = exp(-dt / cfg.speed_bias_tau_s)
        b *= phi
        p00 += (cfg.lead_accel_std_mps2 * dt) ** 2
        p01 *= phi
        p11 = phi * phi * p11 + sb * sb * (1.0 - phi * phi)
        for z, r, h1 in [(vg, sw * sw, 1.0)] + [(z, s * s, 0.0) for z, s in extra]:
            ph0, ph1 = p00 + h1 * p01, p01 + h1 * p11  # P H^T, H = [1, h1]
            s = ph0 + h1 * ph1 + r
            innov = z - (v + h1 * b)
            lim = cfg.innovation_gate_sigma * s ** 0.5
            if cfg.speed_divergence_inflation and h1 == 0.0 and lim > 0 and abs(innov) > lim:
                p00 += (abs(innov) / cfg.innovation_gate_sigma) ** 2 - s + r
                ph0, s = p00, p00 + r
                lim = cfg.innovation_gate_sigma * s ** 0.5
            if cfg.innovation_gate_sigma > 0 and abs(innov) > lim:
                innov = lim if innov > 0 else -lim
            k0, k1 = ph0 / s, ph1 / s
            v += k0 * innov
            b += k1 * innov
            p00, p01, p11 = p00 - k0 * ph0, p01 - k0 * ph1, p11 - k1 * ph1
        self._fused[tid] = (time_s, v, b, p00, p01, p11)
        return v, p00 ** 0.5

    def _fused_range(self, tid: int, time_s: float, d_meas: float, vrel: float) -> float:
        prev = self._range_est.get(tid)
        if prev is None or not isfinite(vrel) or not isfinite(prev[2]) or not 0.0 < time_s - prev[0] <= 0.5:
            d = d_meas
        else:
            d = prev[1] + 0.5 * (prev[2] + vrel) * (time_s - prev[0])
            d += self.config.range_fusion_gain * (d_meas - d)
        self._range_est[tid] = (time_s, d, vrel)
        return d

    # ---- output -----------------------------------------------------------------------------------
    def _guard(self, tid: int, time_s: float, v: float, code: int, age: int) -> bool:
        """True when this record of the track is rejected by the saturation guard."""
        cfg = self.config
        sat = cfg.drop_saturated_codes and (code >= 1023 or code == 0)
        st = self._guard_state.get(tid)
        if st is not None and time_s - st[0] > 0.5:
            st = None  # the track was silent: restart the reference
        if st is None:
            if age < cfg.guard_min_age:
                self._guard_state.pop(tid, None)
                return False
            self._guard_state[tid] = [time_s, nan, time_s, True] if sat else [time_s, v, None, False]
            self.guard_rejected += sat
            return sat
        st[0] = time_s
        ref = st[1]
        bad = sat
        if not sat and isfinite(ref):
            dv = abs(v - ref)
            bad = st[3] and dv > cfg.sat_recover_mps
        if not bad:
            if st[2] is not None:  # episode over: continue under a new trackId
                self._guard_gen[tid] = self._guard_gen.get(tid, 0) + 1
                st[2], st[3] = None, False
            st[1] = v
            return False
        if st[2] is None:
            st[2] = time_s
        if sat:
            st[3] = True
        elif time_s - st[2] > cfg.guard_hold_s:  # a persistent new level is a real step
            self._guard_gen[tid] = self._guard_gen.get(tid, 0) + 1
            st[1], st[2], st[3] = v, None, False
            return False
        self.guard_rejected += 1
        return True

    def _yv_rel(self, obj: NativeObject, time_s: float) -> float:
        """Lateral velocity in the ego frame: the radar's over-ground vy minus the rotation term yaw rate x range."""
        w = self._fresh_yaw_rate(time_s)
        if w is None or obj.v_lat_code in (0, 1023):
            return nan
        return float(obj.v_lat_ground - w * obj.d_rel)

    def _a_rel(self, obj: NativeObject, v_ego: float | None) -> float:
        """The radar's filtered over-ground acceleration minus ego acceleration (lags vRel by about 0.5 s)."""
        if v_ego is None or self._a_ego is None or obj.accel_like_code in (-511, 512):
            return nan
        return float(obj.accel_like_code * ACCEL_SCALE - self._a_ego)

    def _payload(self, time_s: float, record: bytes) -> dict:
        cfg = self.config
        v_ego = self._fresh_ego_speed(time_s)
        decoded = []
        for slot, seg in occupied_slots(record):
            obj = decode_native_slot(slot, seg)
            # every occupied slot updates the lifecycle, including age 0, so every restart is seen
            decoded.append((slot, obj, self._tracks.update(time_s, slot, obj.age)))
        seen = {tid for _, _, tid in decoded}
        acc_tid, acc_vrel = self._acc_target_match(time_s, decoded)
        use_summaries = cfg.fused_speed_filter and cfg.summary_sigma_mps > 0
        sum_speed = self._summary_match(time_s, decoded, v_ego, acc_tid) if use_summaries else {}
        points = []
        for slot, obj, tid in decoded:
            if not obj.geometry_valid or not obj.lateral_valid:
                continue
            if cfg.drop_saturated_codes and self._guard(tid, time_s, obj.v_long_ground, obj.vel_code, obj.age):
                continue
            speed_std = 0.0
            if cfg.fused_speed_filter and v_ego is not None:
                extra = []
                if tid == acc_tid and acc_vrel is not None:
                    extra.append((acc_vrel + v_ego, cfg.acc_sigma_mps))
                if tid in sum_speed and (cfg.summary_max_range_m <= 0 or obj.d_rel <= cfg.summary_max_range_m):
                    extra.append((sum_speed[tid] + v_ego, cfg.summary_sigma_mps))
                v_ground, speed_std = self._fused_speed(tid, time_s, obj, extra, v_ego)
                vrel = float(v_ground - v_ego)
            else:
                v_ground = obj.v_long_ground * cfg.vground_scale
                vrel = float(v_ground - v_ego) if v_ego is not None else nan
            d_meas = self._acc_pos[1] if cfg.acc_range and acc_tid is not None and tid == acc_tid and self._acc_pos is not None else obj.d_rel
            d_rel = self._fused_range(tid, time_s, d_meas, vrel) if cfg.range_fusion_gain > 0 else d_meas
            if obj.age >= 2:
                self._first.setdefault(tid, (time_s, obj.d_rel, obj.y_rel))
                self._last[tid] = (time_s, obj.d_rel, obj.y_rel, vrel, obj.age)
            if obj.age < cfg.min_publish_age:
                self.settling_suppressed += 1
                continue
            if cfg.far_acc_only_m > 0 and tid != acc_tid and d_rel > cfg.far_acc_only_m:
                self.path_gated += 1
                continue
            still = cfg.path_gate_stationary_mps > 0 and abs(v_ground) < cfg.path_gate_stationary_mps and d_rel <= cfg.path_gate_min_range_m
            if cfg.path_gate_m > 0 and tid != acc_tid and (d_rel > cfg.path_gate_min_range_m or still) and v_ego is not None:
                yaw = self._fresh_yaw_rate(time_s)
                width, v_curve = (cfg.path_gate_stationary_m, max(v_ego, 2.0)) if still else (cfg.path_gate_m, max(v_ego, 5.0))
                if yaw is not None and abs(obj.y_rel - yaw / v_curve * d_rel * d_rel / 2) > width:
                    self.path_gated += 1
                    continue
            if tid not in self._out_id and speed_std > cfg.publish_speed_std_mps and cfg.fused_speed_filter:
                self.settling_suppressed += 1
                continue
            if cfg.drop_unresolved_vrel and not isfinite(vrel):
                self.unresolved_suppressed += 1
                continue
            if tid not in self._out_id:
                self._out_id[tid] = self._relink(tid, time_s, seen)
            point = {
                "trackId": int(self._out_id[tid]) + GUARD_ID_STRIDE * self._guard_gen.get(tid, 0),
                "dRel": float(d_rel),
                "yRel": float(obj.y_rel),
                "vRel": vrel,
                "aRel": self._a_rel(obj, v_ego),
                "yvRel": self._yv_rel(obj, time_s),
                "measured": not obj.predicted,  # 107|1, like Tesla's Meas; older radard versions store it with the track
            }
            if cfg.include_metadata:
                point.update(slot=slot, age=obj.age, v_long_ground=float(v_ground), native_id=tid, existence_pct=obj.existence_pct,
                             move_state=obj.move_state, oncoming_flag=obj.oncoming_flag,
                             vrel_status=NATIVE_VREL_STATUS if v_ego is not None else UNRESOLVED_NAN)
            points.append(point)
        self._prune(time_s)
        errors = {"canError": False, "radarFault": False, "wrongConfig": False, "radarUnavailableTemporary": False}
        return {"time_s": float(time_s), "radarData": {"points": points, "errors": errors}}

    def _prune(self, time_s: float) -> None:
        if len(self._last) > 400:  # forget tracks lost long ago
            horizon = time_s - max(self.config.relink_max_gap_s, 1.0) - 60.0
            for k in [k for k, v in self._last.items() if v[0] < horizon]:
                self._last.pop(k, None)
                self._first.pop(k, None)
        for store in (self._range_est, self._fused, self._guard_state):
            if len(store) > 400:
                for k in [k for k, v in store.items() if v[0] < time_s - 5.0]:
                    store.pop(k, None)
        # Published IDs and guard generations may outlive position updates during a guard.
        # Keep every native slot lifecycle ID, as well as every retained relink candidate.
        if max(len(self._out_id), len(self._claimed), len(self._guard_gen)) > 400:
            retained = set(self._last) | {state[0] for state in self._tracks._state.values()}
            for store in (self._out_id, self._guard_gen):
                for k in store.keys() - retained:
                    store.pop(k)
            self._claimed.intersection_update(retained)
