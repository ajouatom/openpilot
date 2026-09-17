"""Group 3 objects move between CAN addresses; addresses are transport slots."""
from collections import Counter
from dataclasses import dataclass
import math

GROUP3_START = 0x400
GROUP3_COUNT = 30
GROUP3_TRACK_ID_START = 1_000_000


@dataclass(frozen=True)
class Group3Object:
  object_id: int
  x: float
  y: float
  v: float
  length: float

  @property
  def d_rel(self):
    return max(0.0, self.x - self.length * 0.5 - 0.1)

  @classmethod
  def from_signals(cls, msg):
    return cls(int(msg["OBJECT_ID"]), msg["LONG_DIST"], msg["LAT_DIST"], msg["REL_SPEED"], msg["OBJECT_LENGTH"])


def decode_group3(raw: bytes) -> Group3Object:
  if len(raw) != 24:
    raise ValueError("Group 3 objects require 24 bytes")
  word = int.from_bytes(raw, "little")

  def signed(start, size):
    value = (word >> start) & ((1 << size) - 1)
    return value - (1 << size) if value & (1 << (size - 1)) else value
  return Group3Object((word >> 24) & 0x7f, ((word >> 64) & 0x7ff) * 0.1,
                      signed(75, 12) * 0.05, signed(87, 11) * 0.1, ((word >> 41) & 0x7f) * 0.1)


class Group3TrackIds:
  def __init__(self):
    self.next_id = GROUP3_TRACK_ID_START
    self.previous = {}

  def update(self, objects: dict[int, Group3Object]):
    """Consume only fresh slots from one 20 Hz cycle. Return slot -> track ID."""
    counts = Counter(obj.object_id for obj in objects.values())
    assignments, current = {}, {}
    for slot, obj in sorted(objects.items()):
      # Zero is empty. An ambiguous duplicate must never share filter history.
      if not (0 < obj.object_id < 128 and counts[obj.object_id] == 1 and 0 <= obj.x < 204.7):
        continue
      if not all(math.isfinite(value) for value in (obj.x, obj.y, obj.v, obj.length)):
        continue
      previous = self.previous.get(obj.object_id)
      continuous = previous is not None and (
        abs(obj.x - (previous[1].x + previous[1].v * 0.05)) <= 8.0
        and abs(obj.y - previous[1].y) <= 3.0
        and abs(obj.v - previous[1].v) <= 4.0
      )
      # Native IDs can be recycled; also break history at the undecoded range wrap.
      if continuous:
        track_id = previous[0]
      else:
        track_id = self.next_id
        self.next_id += 1
      assignments[slot] = track_id
      current[obj.object_id] = (track_id, obj)
    self.previous = current
    return assignments
