"""Recover Group 3 identity from raw CAN while retaining logged ego-speed alignment."""
from collections import deque
from types import SimpleNamespace

from opendbc.car.hyundai.radar_group3 import GROUP3_START, GROUP3_COUNT, GROUP3_TRACK_ID_START, Group3TrackIds, decode_group3
from opendbc.car.radar_tracks import MyTrack


class Group3Replay:
  def __init__(self, bus):
    self.bus = bus
    self.pending = {}
    self.identities = Group3TrackIds()
    self.cycles = deque(maxlen=4)
    self.tracks = {}
    self.last_cycle = None
    self.last_points = ()

  def consume(self, t, address, raw, bus):
    if bus != self.bus or not GROUP3_START <= address < GROUP3_START + GROUP3_COUNT or len(raw) != 24:
      return
    self.pending[address] = decode_group3(raw)
    if address == GROUP3_START + GROUP3_COUNT - 1:
      assignments = self.identities.update(self.pending)
      self.cycles.append((t, {slot: (track_id, self.pending[slot]) for slot, track_id in assignments.items()}))
      self.pending = {}

  @staticmethod
  def is_front(point):
    return str(point.radarSource) == "frontRadar" and (
      32 <= point.trackId < 62 or point.trackId >= GROUP3_TRACK_ID_START
    )

  def correct(self, t, recorded):
    front = [point for point in recorded if self.is_front(point)]
    others = [point for point in recorded if not self.is_front(point)]
    # Match full geometry to avoid using the next CAN cycle when logging is
    # scheduled ahead of liveTracks. Old logs identify slots; new logs use IDs.
    best = None
    for cycle_t, objects in self.cycles:
      if not 0 <= t - cycle_t <= 0.15:
        continue
      matches = {}
      for point in front:
        for slot, (track_id, obj) in objects.items():
          if track_id in matches or (point.trackId < GROUP3_TRACK_ID_START and slot != GROUP3_START + point.trackId - 32):
            continue
          if abs(point.dRel - obj.d_rel) < 0.02 and abs(point.yRel - obj.y) < 0.02 and abs(point.vRel - obj.v) < 0.02:
            matches[track_id] = point
            break
      if best is None or len(matches) >= len(best[1]):
        best = (cycle_t, matches)
    if best is None:
      self.tracks = {}
      self.last_cycle = None
      return tuple(others)
    cycle_t, matches = best
    if cycle_t == self.last_cycle:
      return tuple(others) + self.last_points
    output, tracks = [], {}
    for track_id, recorded_point in matches.items():
      point = SimpleNamespace(trackId=track_id, radarSource="frontRadar", measured=True,
                              dRel=recorded_point.dRel, yRel=recorded_point.yRel, vRel=recorded_point.vRel,
                              vLead=recorded_point.vLead, aRel=float("nan"), yvRel=0.0, trackState=0,
                              aLead=0.0, jLead=0.0)
      track = self.tracks.get(track_id)
      if track is None or self.last_cycle is None or cycle_t - self.last_cycle > 0.075:
        track = MyTrack(track_id, point, 0.05)
      track.update(point, 0.0)
      track.write_acceleration(point)
      tracks[track_id] = track
      output.append(point)
    self.tracks = tracks
    self.last_cycle, self.last_points = cycle_t, tuple(output)
    return tuple(others) + self.last_points
