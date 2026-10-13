"""CRC-checked ID85 record: prefix, ten 12-byte cells, CRC.

The checksum boundary is verified. A populated cell is one lane / road-boundary curve (offset, heading, curvature; docs/04).
The CRC and trailer follow the ten cells.
"""
from __future__ import annotations

from dataclasses import dataclass
import zlib

from .constants import ID85_RECORD_LEN

HEADER_LEN = 21
CELL_COUNT = 10
CELL_LEN = 12
CRC_START, CRC_END = 141, 145

# Lane / road-curve polynomial carried by a populated cell: y(x) = c0 + c1 x + c2 x^2 / 2 + c3 x^3 / 6, left positive.
# Nominal conversions; heading 1.6-2.2e-5 rad and curvature 2.0-2.7e-6 1/m per code bound the units (docs/04).
CURVE_OFFSET_ZERO, CURVE_OFFSET_M = 2000, 0.01
CURVE_HEADING_ZERO, CURVE_HEADING_RAD = 31200, -1.8e-5      # code up = lane pointing right
CURVE_CURVATURE_ZERO, CURVE_CURVATURE_PER_M = 16020, 2.5e-6  # code up = lane curving left
CURVE_RATE_ZERO, CURVE_RATE_PER_M2 = 500, 4e-6               # c3, d(curvature)/ds; nominal unit (docs/04)

# Bits of prefix bytes 17-19 that copy a cell's "empty" flag (1 when bit 30 of that cell is 0). Cells 0-5 carry the copies.
PREFIX_UNPOPULATED_COPIES = {
    0: ((19, 4), (19, 5), (19, 6), (19, 7)), 1: ((19, 1), (19, 2), (19, 3)), 2: ((18, 4), (18, 6), (18, 7)),
    3: ((17, 2), (17, 3)), 4: ((17, 4), (17, 5), (17, 7)), 5: ((18, 1), (18, 3)),
}


@dataclass(frozen=True)
class ShellCell:
    index: int
    payload: bytes

    @property
    def parameters_present(self) -> bool:
        """Cell bit 30: 1 when the cell holds curve parameters instead of the default block 84 03 F4 01 (docs/04)."""
        if len(self.payload) != CELL_LEN:
            raise ValueError(f"0x85 cell must be {CELL_LEN} bytes")
        return bool(self.payload[3] & 0x40)

    def _field(self, start: int, length: int) -> int:
        if len(self.payload) != CELL_LEN:
            raise ValueError(f"0x85 cell must be {CELL_LEN} bytes")
        return (int.from_bytes(self.payload, "little") >> start) & ((1 << length) - 1)

    @property
    def offset_code(self) -> int:
        """c0: raw `32|12` lateral offset of the curve at the radar, (code - 2000) * 0.01 m, left positive."""
        return self._field(32, 12)

    @property
    def heading_code(self) -> int:
        """c1: raw `48|16` curve heading (unpopulated cells hold the default 900)."""
        return self._field(48, 16)

    @property
    def curvature_code(self) -> int:
        """c2: raw `64|15` curve curvature (unpopulated cells hold the default 500)."""
        return self._field(64, 15)

    @property
    def curve_flag(self) -> int:
        """Raw `79|1` flag next to the curvature: the same in cells 2 and 3 on 99 % of rows; set on 79-80 % of cells 2/3 (docs/04)."""
        return self._field(79, 1)

    @property
    def rate_code(self) -> int:
        """c3: raw `10|10` curvature rate (offset binary, zero about 500; the unpopulated default is 1023)."""
        return self._field(10, 10)

    @property
    def lane_offset_m(self) -> float | None:
        """Nominal c0 in metres (left positive), None for an empty cell."""
        return (self.offset_code - CURVE_OFFSET_ZERO) * CURVE_OFFSET_M if self.parameters_present else None

    @property
    def heading_rad(self) -> float | None:
        """Nominal c1: left-positive tangent dy/dx at the radar. Unit bounded to about 1.6-2.2e-5 rad/code; None when unpopulated."""
        return (self.heading_code - CURVE_HEADING_ZERO) * CURVE_HEADING_RAD if self.parameters_present else None

    @property
    def curvature_per_m(self) -> float | None:
        """Nominal c2: left-positive d2y/dx2. Unit bounded to about 2.0-2.7e-6 1/m per code; None when unpopulated."""
        return (self.curvature_code - CURVE_CURVATURE_ZERO) * CURVE_CURVATURE_PER_M if self.parameters_present else None

    @property
    def curvature_rate_per_m2(self) -> float | None:
        """Nominal c3 in 1/m^2 (about 4e-6 per code, sign and zero replicated); None when unpopulated."""
        return (self.rate_code - CURVE_RATE_ZERO) * CURVE_RATE_PER_M2 if self.parameters_present else None


def prefix_flags_consistent(prefix_or_record: bytes, cells: list["ShellCell"]) -> bool:
    """True when prefix bytes 17-19 hold the inverted parameters-present flag of cells 0-5 (about 99.96 % of records)."""
    return all(((prefix_or_record[by] >> bi) & 1) == (0 if cells[k].parameters_present else 1)
               for k, bits in PREFIX_UNPOPULATED_COPIES.items() for by, bi in bits)


def id85_crc_ok(record: bytes) -> bool:
    if len(record) != ID85_RECORD_LEN:
        return False
    return int.from_bytes(record[CRC_START:CRC_END], "little") == zlib.crc32(record[1:CRC_START])


def parse_shell(record: bytes) -> tuple[bytes, list[ShellCell]]:
    """Return the proposed 21-byte prefix and ten raw cells; reject bad CRC."""
    if len(record) != ID85_RECORD_LEN:
        raise ValueError(f"0x85 record must be {ID85_RECORD_LEN} bytes, got {len(record)}")
    if not id85_crc_ok(record):
        raise ValueError("0x85 CRC mismatch")
    cells = []
    for k in range(CELL_COUNT):
        o = HEADER_LEN + CELL_LEN * k
        cells.append(ShellCell(k, record[o:o + CELL_LEN]))
    return record[:HEADER_LEN], cells
