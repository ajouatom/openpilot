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


def test_s3_rejects_old_native_library_instead_of_ignoring_qp_limits():
  from types import SimpleNamespace
  import pytest
  from cluster_h264_pipeline import H264UsbPipeline
  from cluster_wifi_display import S3WifiDisplay
  obj = SimpleNamespace(usb_display=S3WifiDisplay({}))
  with pytest.raises(RuntimeError, match='rebuilding'):
    H264UsbPipeline._set_native_qp_range(obj, SimpleNamespace(), 123)


def test_s3_passes_full_qp_range_and_propagates_driver_failure():
  from types import SimpleNamespace
  import pytest
  from cluster_h264_pipeline import H264UsbPipeline
  from cluster_wifi_display import S3WifiDisplay
  calls = []
  result = [0]

  def setter(*args):
    calls.append(args)
    return result[0]

  obj = SimpleNamespace(usb_display=S3WifiDisplay({}), _native_error_text=lambda message: message)
  lib = SimpleNamespace(cluster_h264_encoder_bridge_set_qp_range=setter)
  H264UsbPipeline._set_native_qp_range(obj, lib, 123)
  assert calls == [(123, 28, 51)]
  result[0] = -1
  with pytest.raises(RuntimeError, match='QP range setup failed'):
    H264UsbPipeline._set_native_qp_range(obj, lib, 123)


def test_wifi_h264_defaults_preserve_explicit_rate_and_bitrate():
  import ast
  from types import SimpleNamespace
  path = Path(__file__).resolve().parents[1] / 'cluster/main.py'
  tree = ast.parse(path.read_text())
  main = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == 'main')
  wifi = next(n for n in main.body if isinstance(n, ast.If) and ast.unparse(n.test) == 'wifi_config is not None')
  h264 = next(n for n in wifi.body if isinstance(n, ast.If) and ast.unparse(n.test) == "args.usb_codec == 'h264'")
  code = compile(ast.Module(body=h264.body, type_ignores=[]), str(path), 'exec')
  for explicit, bitrate, want_bitrate, want_rc in [
      (False, 'auto', '1M', 'cbr-cfr'), (True, '750k', '750k', 'vbr-cfr')]:
    args = SimpleNamespace(usb_h264_bitrate=bitrate, usb_h264_rate_control='vbr-cfr',
                           usb_h264_rate_control_from_cli=explicit, usb_h264_diagnose_interval=0)
    exec(code, {'args': args})
    assert args.usb_h264_bitrate == want_bitrate
    assert args.usb_h264_rate_control == want_rc
    assert args.usb_h264_gop == 10 and not args.usb_h264_wait_ack
