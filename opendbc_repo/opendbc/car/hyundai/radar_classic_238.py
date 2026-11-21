# classic Hyundai 0x238 객체 목록의 검증된 운동학 필드를 해석하는 디코더
from dataclasses import dataclass


CLASSIC_238_START_ADDR = 0x238
CLASSIC_238_SLOT_COUNT = 10
CLASSIC_238_FRAMES_PER_SLOT = 3
CLASSIC_238_END_ADDR = CLASSIC_238_START_ADDR + CLASSIC_238_SLOT_COUNT * CLASSIC_238_FRAMES_PER_SLOT - 1
CLASSIC_238_MAX_TRIPLET_AGE_NS = 150_000_000
# One triplet arrives within 13.2 ms in K7 captures. The 2-bit counter repeats
# every 4 scans (~120 ms), so frames from different scans must not be combined.
CLASSIC_238_MAX_TRIPLET_SPREAD_NS = 30_000_000
CLASSIC_238_MAX_D_REL_DELTA_M = 3.0
CLASSIC_238_MAX_Y_REL_DELTA_M = 1.5
CLASSIC_238_MAX_V_LEAD_DELTA_MS = 5.0
# The radar's 6-bit object ID is reused often (all 64 values within ~45 s at an
# intersection), so it only biases association; the physical gates still decide.
CLASSIC_238_SAME_ID_BONUS = 1.0
CLASSIC_238_OTHER_ID_PENALTY = 1.0
# A dropped track may be resumed only by the same ID within this gap and the same
# physical gates; longer gaps are treated as ID reuse by another object.
CLASSIC_238_ID_REACQUIRE_NS = 300_000_000


@dataclass(frozen=True)
class Classic238Object:
  status: int
  d_rel: float
  y_rel: float
  v_lead: float
  raw: int

  @classmethod
  def from_first_frame(cls, payload: bytes) -> "Classic238Object":
    if len(payload) != 8:
      raise ValueError(f"classic 0x238 object frame must be 8 bytes, got {len(payload)}")

    raw = int.from_bytes(payload, "big")
    status = (raw >> 48) & 0x7
    d_rel = ((raw >> 51) & 0x1FFF) * 0.05
    y_sensor = ((raw >> 36) & 0xFFF) * 0.05 - 102.4
    v_lead = ((raw >> 20) & 0xFFF) * 0.05 - 100.0
    return cls(status=status, d_rel=d_rel, y_rel=-y_sensor, v_lead=v_lead, raw=raw)


@dataclass(frozen=True)
class Classic238Triplet:
  obj: Classic238Object
  rolling_counters: tuple[int, int, int]
  object_sequence: int
  second_frame: bytes
  third_frame: bytes

  @property
  def counter_consistent(self) -> bool:
    return len(set(self.rolling_counters)) == 1

  @property
  def object_id(self) -> int | None:
    # 0x238 bits 1-6 are mirrored in 0x239 bits 0-5; use the ID only when both agree.
    object_id = (self.obj.raw >> 1) & 0x3F
    return object_id if (self.second_frame[7] & 0x3F) == object_id else None

  @classmethod
  def from_frames(cls, first: bytes, second: bytes, third: bytes) -> "Classic238Triplet":
    if len(second) != 8 or len(third) != 8:
      raise ValueError("classic 0x238 follow-up frames must be 8 bytes")

    obj = Classic238Object.from_first_frame(first)
    # The same two-bit rolling counter is present in all three frames. It was
    # identical for every complete triplet in the K7, K5 and Sonata captures.
    first_counter = (obj.raw >> 18) & 0x3
    second_counter = (second[0] >> 6) & 0x3
    third_counter = (third[0] >> 6) & 0x3
    # This byte advances while a slot is active and normally freezes while it
    # is empty. It does not reset to a fixed value when a new object appears,
    # so expose it as a sequence value rather than claiming it is an object ID.
    object_sequence = third[3]
    return cls(obj=obj, rolling_counters=(first_counter, second_counter, third_counter),
               object_sequence=object_sequence, second_frame=second, third_frame=third)


@dataclass(frozen=True)
class Classic238RadarKinematics:
  d_rel: float
  y_rel: float
  v_rel: float
  v_lead: float
  a_rel: float
  yv_rel: float

  @classmethod
  def from_triplet(cls, triplet: Classic238Triplet, v_ego: float) -> "Classic238RadarKinematics":
    return cls(d_rel=triplet.obj.d_rel, y_rel=triplet.obj.y_rel,
               v_rel=triplet.obj.v_lead - v_ego, v_lead=triplet.obj.v_lead,
               a_rel=float("nan"), yv_rel=0.0)


@dataclass(frozen=True)
class Classic238DisplayTrack:
  track_id: int
  slot: int
  status: int
  object_sequence: int
  kinematics: Classic238RadarKinematics
  last_seen_ns: int
  object_id: int | None = None


@dataclass(frozen=True)
class Classic238TrackObservation:
  slot: int
  status: int
  object_sequence: int
  kinematics: Classic238RadarKinematics
  object_id: int | None = None


class Classic238AssociationTracker:
  def __init__(self, max_age_ns: int = CLASSIC_238_MAX_TRIPLET_AGE_NS):
    self.max_age_ns = max_age_ns
    self.next_track_id = 0
    self.tracks: dict[int, Classic238DisplayTrack] = {}
    self.lost: dict[int, Classic238DisplayTrack] = {}

  @staticmethod
  def _association_cost(track: Classic238DisplayTrack, observation: Classic238TrackObservation,
                        mono_time_ns: int) -> float | None:
    age_s = max(0.0, (mono_time_ns - track.last_seen_ns) * 1e-9)
    predicted_d_rel = track.kinematics.d_rel + track.kinematics.v_rel * age_s
    d_delta = abs(observation.kinematics.d_rel - predicted_d_rel)
    y_delta = abs(observation.kinematics.y_rel - track.kinematics.y_rel)
    v_delta = abs(observation.kinematics.v_lead - track.kinematics.v_lead)
    if (d_delta > CLASSIC_238_MAX_D_REL_DELTA_M
        or y_delta > CLASSIC_238_MAX_Y_REL_DELTA_M
        or v_delta > CLASSIC_238_MAX_V_LEAD_DELTA_MS):
      return None
    slot_penalty = 0.0 if observation.slot == track.slot else 0.2
    id_term = 0.0
    if track.object_id is not None and observation.object_id is not None:
      id_term = -CLASSIC_238_SAME_ID_BONUS if track.object_id == observation.object_id else CLASSIC_238_OTHER_ID_PENALTY
    return (d_delta / CLASSIC_238_MAX_D_REL_DELTA_M
            + y_delta / CLASSIC_238_MAX_Y_REL_DELTA_M
            + v_delta / CLASSIC_238_MAX_V_LEAD_DELTA_MS
            + slot_penalty + id_term)

  def update_scan(self, mono_time_ns: int,
                  observations: list[Classic238TrackObservation]) -> list[Classic238DisplayTrack]:
    self.current(mono_time_ns)
    pairs = []
    for track_id, track in self.tracks.items():
      for index, observation in enumerate(observations):
        cost = self._association_cost(track, observation, mono_time_ns)
        if cost is not None:
          pairs.append((cost, track_id, index))

    assigned_tracks = set()
    assigned_observations = set()
    for _, track_id, index in sorted(pairs):
      if track_id in assigned_tracks or index in assigned_observations:
        continue
      observation = observations[index]
      self.tracks[track_id] = Classic238DisplayTrack(
        track_id=track_id, slot=observation.slot, status=observation.status,
        object_sequence=observation.object_sequence, kinematics=observation.kinematics,
        last_seen_ns=mono_time_ns, object_id=observation.object_id,
      )
      assigned_tracks.add(track_id)
      assigned_observations.add(index)

    for index, observation in enumerate(observations):
      if index in assigned_observations:
        continue
      track_id = self._reacquire(observation, mono_time_ns)
      if track_id is None:
        track_id = self.next_track_id
        self.next_track_id += 1
      self.tracks[track_id] = Classic238DisplayTrack(
        track_id=track_id, slot=observation.slot, status=observation.status,
        object_sequence=observation.object_sequence, kinematics=observation.kinematics,
        last_seen_ns=mono_time_ns, object_id=observation.object_id,
      )
    return self.current(mono_time_ns)

  def _reacquire(self, observation: Classic238TrackObservation, mono_time_ns: int) -> int | None:
    if observation.object_id is None:
      return None
    candidates = [(cost, track_id) for track_id, track in self.lost.items()
                  if track.object_id == observation.object_id
                  and (cost := self._association_cost(track, observation, mono_time_ns)) is not None]
    if not candidates:
      return None
    track_id = min(candidates)[1]
    del self.lost[track_id]
    return track_id

  def current(self, mono_time_ns: int) -> list[Classic238DisplayTrack]:
    stale_ids = [track_id for track_id, track in self.tracks.items()
                 if mono_time_ns - track.last_seen_ns > self.max_age_ns]
    for track_id in stale_ids:
      track = self.tracks.pop(track_id)
      if track.object_id is not None:
        self.lost[track_id] = track
    self.lost = {track_id: track for track_id, track in self.lost.items()
                 if mono_time_ns - track.last_seen_ns <= CLASSIC_238_ID_REACQUIRE_NS}
    return [self.tracks[track_id] for track_id in sorted(self.tracks)]


class Classic238DisplayTracker:
  def __init__(self, max_age_ns: int = CLASSIC_238_MAX_TRIPLET_AGE_NS):
    self.max_age_ns = max_age_ns
    self.assembler = Classic238Assembler(max_age_ns=max_age_ns)
    self.association = Classic238AssociationTracker(max_age_ns=max_age_ns)
    self.pending: dict[int, Classic238TrackObservation] = {}
    self.pending_counter: int | None = None

  def _flush(self, mono_time_ns: int) -> None:
    if self.pending:
      self.association.update_scan(mono_time_ns, list(self.pending.values()))
      self.pending.clear()

  def finish_scan(self, mono_time_ns: int) -> None:
    self._flush(mono_time_ns)

  def update(self, mono_time_ns: int, address: int, payload: bytes, v_ego: float) -> None:
    assembled = self.assembler.update(mono_time_ns, address, payload)
    if assembled is None:
      return

    slot, triplet = assembled
    counter = triplet.rolling_counters[0]
    if self.pending_counter is not None and counter != self.pending_counter:
      self._flush(mono_time_ns)
    self.pending_counter = counter
    if triplet.obj.status == 0:
      self.pending.pop(slot, None)
    else:
      self.pending[slot] = Classic238TrackObservation(
        slot=slot, status=triplet.obj.status, object_sequence=triplet.object_sequence,
        kinematics=Classic238RadarKinematics.from_triplet(triplet, v_ego),
        object_id=triplet.object_id,
      )
    if slot == CLASSIC_238_SLOT_COUNT - 1:
      self._flush(mono_time_ns)

  def current(self, mono_time_ns: int) -> list[Classic238DisplayTrack]:
    return self.association.current(mono_time_ns)


# Display-only classicRadarTracks IDs: the radar's own 6-bit object ID (0-63)
# when the two frames agree, otherwise this offset plus the local track ID.
CLASSIC_238_DISPLAY_FALLBACK_ID_OFFSET = 1000


def classic_238_display_ids(tracks) -> list[int]:
  """Radar object IDs for display, unique within one message.

  The freshest track keeps a duplicated object ID (the radar may reuse an ID
  while an older track is still held); the others use the fallback range.
  """
  owners: dict[int, Classic238DisplayTrack] = {}
  for track in tracks:
    if track.object_id is None:
      continue
    owner = owners.get(track.object_id)
    if owner is None or track.last_seen_ns > owner.last_seen_ns:
      owners[track.object_id] = track
  return [track.object_id if track.object_id is not None and owners[track.object_id] is track
          else CLASSIC_238_DISPLAY_FALLBACK_ID_OFFSET + track.track_id
          for track in tracks]


class Classic238Assembler:
  def __init__(self, max_age_ns: int = CLASSIC_238_MAX_TRIPLET_AGE_NS,
               max_spread_ns: int = CLASSIC_238_MAX_TRIPLET_SPREAD_NS):
    self.max_age_ns = max_age_ns
    self.max_spread_ns = min(max_spread_ns, max_age_ns)
    self.frames: list[list[tuple[int, bytes] | None]] = [
      [None] * CLASSIC_238_FRAMES_PER_SLOT for _ in range(CLASSIC_238_SLOT_COUNT)
    ]
    self.last_emitted: list[tuple[int, int, int] | None] = [None] * CLASSIC_238_SLOT_COUNT

  def update(self, mono_time_ns: int, address: int, payload: bytes) -> tuple[int, Classic238Triplet] | None:
    slot, role = classic_238_address_role(address)
    if len(payload) != 8:
      raise ValueError(f"classic 0x238 frame must be 8 bytes, got {len(payload)}")
    self.frames[slot][role] = (mono_time_ns, payload)
    complete = self.frames[slot]
    if any(frame is None for frame in complete):
      return None
    typed = [frame for frame in complete if frame is not None]
    times = tuple(frame[0] for frame in typed)
    if max(times) - min(times) > self.max_spread_ns or mono_time_ns - max(times) > self.max_age_ns:
      return None
    triplet = Classic238Triplet.from_frames(*(frame[1] for frame in typed))
    if not triplet.counter_consistent or self.last_emitted[slot] == times:
      return None
    self.last_emitted[slot] = times
    return slot, triplet


def classic_238_address_role(address: int) -> tuple[int, int]:
  if not CLASSIC_238_START_ADDR <= address <= CLASSIC_238_END_ADDR:
    raise ValueError(f"address 0x{address:x} is outside classic 0x238 object range")

  offset = address - CLASSIC_238_START_ADDR
  return offset // CLASSIC_238_FRAMES_PER_SLOT, offset % CLASSIC_238_FRAMES_PER_SLOT


# EnableRadarTracks value that selects this stream on a KIA_K7_PE that sends it. It performs no
# ECU write and does not need 0x500 tracks. Modes 2/3 use SCC and stay unchanged.
CLASSIC_238_TRACK_MODE = 1
# Status 2 is a confirmed moving object. Status 6 is the same object ID and
# position continuing as the near in-lane lead in stop-and-go (2026-09-29: 65%
# in lane, median 4.5 m, range within ~0.4 m of status 2 against vision); without
# it 99% of those samples fell back to vision while stopped. Status 1/3/4/5 are
# adjacent-lane, short-lived or roadside stationary objects and stay excluded.
# trackState keeps the raw value; downstream checks only require >= 2.
CLASSIC_238_LONGITUDINAL_STATUSES = frozenset({2, 6})
# Status 1 is mostly adjacent-lane traffic, but inside the ego lane it is the
# lead vision sees (2026-09-29/30, 39 bundles: 95% vision agreement, vision
# carried the lead 83% of those samples) - e.g. a truck ahead at 8.5 m. Within
# 20 m its range matches status 2 (median radar-vision -0.11/-0.05 m); farther
# it reads 1-3 m long, and at |y| 1.2-2 m 66% had no vision lead. It keeps raw
# trackState 1, which downstream treats as tentative: vision support required.
CLASSIC_238_TENTATIVE_STATUS = 1
CLASSIC_238_TENTATIVE_MAX_D_REL_M = 20.0
CLASSIC_238_TENTATIVE_MAX_ABS_Y_REL_M = 1.2
# A 10 m minimum was tried after one low-speed test read near stopped leads ~2 m
# farther than vision. A 70-minute engaged drive with near radar leads agreed with
# vision within 0.5 m below 10 m (stopped and moving) and held steadier stop gaps.
CLASSIC_238_MIN_D_REL_M = 3.0
CLASSIC_238_MAX_D_REL_M = 200.0
# Confirmed objects in the adjacent lanes are passed on as well so radard can follow a
# merging car's lateral motion and raise its cut-in risk before it reaches the lane.
# A +/-2.0 m gate hid a 70 km/h merge at 13 m until it was 1.9 m from the centre
# (2026-10-01); radard's own path/vision matching still decides leadOne.
CLASSIC_238_MAX_ABS_Y_REL_M = 4.5
# Oncoming and turning traffic at intersections reports negative ground speed.
CLASSIC_238_MIN_V_LEAD_MS = -1.0


def classic_238_radar_available(fingerprint_bus1) -> bool:
  """All 30 triplet addresses are present as 8-byte frames and no legacy 0x500 radar exists."""
  return (all(fingerprint_bus1.get(addr) == 8 for addr in range(CLASSIC_238_START_ADDR, CLASSIC_238_END_ADDR + 1))
          and 0x500 not in fingerprint_bus1)


def classic_238_longitudinal_tracks(tracks, now_ns: int) -> list[Classic238DisplayTrack]:
  # Only fresh status-2/6 objects near the ego lane (and near in-lane status 1)
  # that are not approaching over the ground are candidates.
  return [
    track for track in tracks
    if ((track.status in CLASSIC_238_LONGITUDINAL_STATUSES
         or (track.status == CLASSIC_238_TENTATIVE_STATUS
             and track.kinematics.d_rel <= CLASSIC_238_TENTATIVE_MAX_D_REL_M
             and abs(track.kinematics.y_rel) <= CLASSIC_238_TENTATIVE_MAX_ABS_Y_REL_M))
        and now_ns - track.last_seen_ns <= CLASSIC_238_MAX_TRIPLET_AGE_NS
        and CLASSIC_238_MIN_D_REL_M <= track.kinematics.d_rel <= CLASSIC_238_MAX_D_REL_M
        and abs(track.kinematics.y_rel) <= CLASSIC_238_MAX_ABS_Y_REL_M
        and track.kinematics.v_lead >= CLASSIC_238_MIN_V_LEAD_MS)
  ]
