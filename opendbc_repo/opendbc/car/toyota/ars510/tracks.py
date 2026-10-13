"""Track IDs from the radar's own slot / age lifecycle.

An object keeps its slot while the radar tracks it and its age counts up by one per cycle (saturating at 126).
A new ID starts whenever a slot's age stays level or drops (restart, age 0) or the slot goes quiet for longer than
`max_gap_s`. Every ID lives in one slot only; within 60 m the track keeps one camera identity for at least 95% of its
life on 77 of 87 camera-checked tracks (docs/06_accuracy.md).
"""
from __future__ import annotations

from math import isfinite

from .objects import AGE_SATURATION


class NativeTrackIdAssigner:
    def __init__(self, max_gap_s: float = 0.3) -> None:
        self.max_gap_s = max_gap_s
        self._state: dict[int, tuple[int, int, float]] = {}  # slot -> (track id, age, time)
        self._next_id = 1
        self._last_time_s: float | None = None

    def update(self, time_s: float, slot: int, age: int) -> int:
        """Updates must be chronological; an exact repeat of the same (slot, time) is idempotent."""
        if not isfinite(time_s):
            raise ValueError("time_s must be finite")
        if self._last_time_s is not None and time_s < self._last_time_s:
            raise ValueError("track updates must be chronological; reset per route or segment")
        prev = self._state.get(slot)
        if prev is not None and time_s == prev[2]:
            if age != prev[1]:
                raise ValueError("conflicting ages for the same slot and timestamp")
            return prev[0]
        continues = (
            prev is not None
            and (time_s - prev[2]) <= self.max_gap_s
            and (age > prev[1] or age == prev[1] == AGE_SATURATION)
        )
        track_id = prev[0] if continues else self._new_id()
        self._state[slot] = (track_id, age, time_s)
        self._last_time_s = time_s
        return track_id

    def _new_id(self) -> int:
        tid = self._next_id
        self._next_id += 1
        return tid
