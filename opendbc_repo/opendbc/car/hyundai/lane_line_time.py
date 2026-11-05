"""Timestamp of the exact cached OEM lane description; no control gate changes."""


def lane_line_time(parsers, selected):
  if selected is None:
    return 0
  for parser in parsers:
    if parser is None:
      continue
    for name in ('CAM_0x2a4', 'CAM_0x362'):
      if parser.vl.get(name) is selected:
        stamps = parser.ts_nanos.get(name, {})
        return min((stamps.get(key, 0) for key in
                    ('LEFT_LANE_TYPE', 'RIGHT_LANE_TYPE', 'LEFT_LANE_COLOR', 'RIGHT_LANE_COLOR')), default=0)
  return 0
