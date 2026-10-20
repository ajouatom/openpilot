import itertools
from pathlib import Path
import random
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'cluster'))
from cluster_h264_pipeline import _h264_start_codes, _h264_nals


def reference_starts(data):
  positions = []
  index = 0
  while index + 3 <= len(data):
    size = 4 if data[index:index + 4] == b'\0\0\0\1' else 3 if data[index:index + 3] == b'\0\0\1' else 0
    if size:
      positions.append((index, size))
    index += size or 1
  return positions


def test_annex_b_prefix_search_matches_reference_at_every_short_boundary():
  # Includes adjacent/truncated prefixes and arbitrarily placed leading zeros.
  for length in range(10):
    for values in itertools.product((0, 1, 2), repeat=length):
      data = bytes(values)
      assert _h264_start_codes(data) == reference_starts(data)


def test_large_access_unit_preserves_prefixes_payload_and_trailing_zero_rules():
  rng = random.Random(8122)
  units = [b'\0\0\0\1\x67abc', b'\0\0\1\x68def']
  units += [b'\0\0\0\1\x65' + rng.randbytes(4091) for _ in range(203)]
  data = b'\0' + b''.join(units) + b'\0\0'
  assert len(data) > 830000
  assert _h264_start_codes(data) == reference_starts(data)
  for start, payload, end in _h264_nals(data):
    assert data[start:payload] in (b'\0\0\1', b'\0\0\0\1')
    assert payload < end and data[end - 1] != 0

def test_wired_encoder_does_not_require_wifi_qp_api():
  from types import SimpleNamespace
  from cluster_h264_pipeline import H264UsbPipeline
  obj = SimpleNamespace(usb_display=SimpleNamespace())
  H264UsbPipeline._set_native_qp_range(obj, SimpleNamespace(), 123)
