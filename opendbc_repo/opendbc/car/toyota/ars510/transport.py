"""Reassembly of the segmented 0x80 / 0x85 records.

Each record starts with a first frame whose bytes 0-1 are a fixed marker (0x12 0xE4 for 0x80, 0x10 0x90 for
0x85). Consecutive frames follow; bytes 1..7 of every frame, first frame included, are appended. The
consecutive-frame sequence nibble wraps every 16 frames, so each record is reassembled in code, which tracks which part
of the record a frame carries.
"""
from __future__ import annotations

from dataclasses import dataclass
from math import isfinite

from .constants import (
    ID80_FIRST_FRAME,
    ID80_RECORD_FRAMES,
    ID80_RECORD_LEN,
    ID85_FIRST_FRAME,
    ID85_RECORD_FRAMES,
    ID85_RECORD_LEN,
)


@dataclass(frozen=True)
class CompletedRecord:
    time_s: float  # timestamp of the first frame
    payload: bytes
    frame_count: int


class SegmentedRecordAssembler:
    def __init__(self, first_frame: tuple[int, int], frames_per_record: int, record_len: int) -> None:
        self.first_frame = first_frame
        self.frames_per_record = frames_per_record
        self.record_len = record_len
        self.reset()

    def reset(self) -> None:
        self._chunks: list[bytes] = []
        self._start_time_s: float | None = None
        self._last_time_s: float | None = None

    def _is_start(self, data: bytes) -> bool:
        return data[0] == self.first_frame[0] and data[1] == self.first_frame[1]

    def push(self, time_s: float, data: bytes) -> CompletedRecord | None:
        """Feed one 8-byte frame; returns a record when its last frame arrives."""
        if len(data) != 8 or not isfinite(time_s):
            self.reset()
            return None
        if self._chunks and self._last_time_s is not None and time_s < self._last_time_s:
            self.reset()
        if self._is_start(data):
            self._chunks = [bytes(data[1:8])]
            self._start_time_s = time_s
        elif self._chunks:
            self._chunks.append(bytes(data[1:8]))
        else:
            return None
        self._last_time_s = time_s
        if len(self._chunks) != self.frames_per_record:
            return None
        record = b"".join(self._chunks)
        start = self._start_time_s if self._start_time_s is not None else time_s
        n = len(self._chunks)
        self.reset()
        if len(record) != self.record_len:
            return None
        return CompletedRecord(float(start), record, n)


class Id80RecordAssembler(SegmentedRecordAssembler):
    def __init__(self) -> None:
        super().__init__(ID80_FIRST_FRAME, ID80_RECORD_FRAMES, ID80_RECORD_LEN)


class Id85RecordAssembler(SegmentedRecordAssembler):
    def __init__(self) -> None:
        super().__init__(ID85_FIRST_FRAME, ID85_RECORD_FRAMES, ID85_RECORD_LEN)
