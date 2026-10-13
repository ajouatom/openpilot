"""0x80 object record: CRC check and slot extraction."""
from __future__ import annotations

import zlib
from collections.abc import Iterator

from .constants import (
    ID80_CRC_END,
    ID80_CRC_FIELD,
    ID80_CRC_START,
    ID80_IDLE_SLOT,
    ID80_OBJECT_COUNT,
    ID80_OBJECT_LEN,
    ID80_OBJECT_START,
    ID80_RECORD_LEN,
)


def id80_crc_ok(record: bytes) -> bool:
    """zlib CRC32 over record[1:737], stored little-endian at record[737:741]."""
    if len(record) != ID80_RECORD_LEN:
        return False
    observed = int.from_bytes(record[ID80_CRC_FIELD[0]:ID80_CRC_FIELD[1]], "little")
    return observed == (zlib.crc32(record[ID80_CRC_START:ID80_CRC_END]) & 0xFFFFFFFF)


def id80_crc(record_body: bytes) -> int:
    """CRC value for record[1:737] (useful for building synthetic records)."""
    return zlib.crc32(record_body) & 0xFFFFFFFF


def slot_bytes(record: bytes, slot: int) -> bytes:
    start = ID80_OBJECT_START + slot * ID80_OBJECT_LEN
    return record[start:start + ID80_OBJECT_LEN]


def occupied_slots(record: bytes) -> Iterator[tuple[int, bytes]]:
    """(slot index, 36 slot bytes) for every slot that differs from the idle template."""
    for slot in range(ID80_OBJECT_COUNT):
        b = slot_bytes(record, slot)
        if b != ID80_IDLE_SLOT:
            yield slot, b


def live_object_count(record: bytes) -> int:
    """Header field (record bits 115..119, little-endian): number of allocated slots: slots whose index field equals
    their position, which are the slots with age >= 1.

    Exact on every record of the bundled samples (header_allocation_count.json); an integrity cross-check (docs/02).
    """
    return (int.from_bytes(record[14:16], "little") >> 3) & 0x1F
