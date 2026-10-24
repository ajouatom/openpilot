"""The CPU budget must not change the frame format or apply to hardware encoders."""
from pathlib import Path
import shutil
import subprocess
import sys
import inspect

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'cluster'))
from cluster_h264_pipeline import H264UsbPipeline


def pipeline(threads=0, width=64, height=96):
  return H264UsbPipeline(
    usb_display=None, width=width, height=height, encoder_align=1, fps=10,
    bitrate='2333k', gop=1, backend='ffmpeg', library_path='',
    ffmpeg_path='ffmpeg', ffmpeg_encoder='libx264', device_path='',
    input_format='nv12', slice_max_bytes=0, rate_control='vbr-cfr',
    realtime_priority=False, requested_chunk_size=0, wait_for_ack=False,
    soft_ack=False, dump_path='', debug=False, ffmpeg_threads=threads,
  )


@pytest.mark.parametrize('encoder', ['libx264', 'h264_v4l2m2m', 'h264_omx'])
def test_cpu_budget_is_scoped_to_software_encoder(encoder):
  original = pipeline()._ffmpeg_command('ffmpeg', encoder, 'h264')
  limited = pipeline(1)._ffmpeg_command('ffmpeg', encoder, 'h264')
  if encoder != 'libx264':
    assert limited == original
    return
  assert '-threads:v' not in original and '-filter_threads' not in original
  assert limited.index('-filter_threads') < limited.index('-i')
  assert limited.index('-threads:v') > limited.index('pipe:0')
  for flag in ('-filter_threads', '-threads:v'):
    index = limited.index(flag)
    assert limited[index + 1] == '1'
    del limited[index:index + 2]
  assert limited == original  # resolution, rate, bitrate, GOP, pixel format, muxer


@pytest.mark.parametrize('threads', [-1, 17])
def test_reject_invalid_cpu_budget(threads):
  with pytest.raises(ValueError):
    pipeline(threads)


@pytest.mark.parametrize('values,expected', [([], 0), (['1'], 1), (['1', '0'], 0)])
def test_cli_passes_budget_and_allows_explicit_automatic_override(monkeypatch, values, expected):
  import main as cluster
  signature = inspect.signature(cluster.run_demo)
  calls = []
  monkeypatch.setattr(cluster, 'TICI', False)
  monkeypatch.setattr(cluster, 'apply_cluster_encoder_param', lambda args: 'test')
  monkeypatch.setattr(cluster, 'run_demo', lambda *args, **kwargs: calls.append(signature.bind(*args, **kwargs)))
  argv = ['cluster', '--input', 'random', '--output', 'window', '--fps', '10', '--usb-brightness', '50']
  for value in values:
    argv.extend(['--usb-h264-ffmpeg-threads', value])
  monkeypatch.setattr(sys, 'argv', argv)
  cluster.main(exit_on_error=False)
  assert calls[0].arguments['usb_h264_ffmpeg_threads'] == expected


@pytest.mark.parametrize('threads', [0, 1])
@pytest.mark.parametrize('width,height', [(64, 96), (462, 1920)])
def test_real_encoder_keeps_all_frames_and_orientation(threads, width, height):
  ffmpeg = shutil.which('ffmpeg')
  if not ffmpeg:
    pytest.skip('native FFmpeg unavailable')
  frames = []
  for frame in range(8):
    # Asymmetric quadrants expose stride, channel and orientation regressions.
    upper = bytes((220, 220, 30 + frame * 20, 255)) * (width // 2) + bytes((20, 220, 30 + frame * 20, 255)) * (width // 2)
    lower = bytes((220, 20, 30 + frame * 20, 255)) * (width // 2) + bytes((20, 20, 30 + frame * 20, 255)) * (width // 2)
    frames.append(upper * (height // 2) + lower * (height // 2))
  # Existing -fflags nobuffer consumes the initial raw-video probe frame.
  # Prime it separately, then require every measured frame in both modes.
  encoded = subprocess.run(pipeline(threads, width, height)._ffmpeg_command(ffmpeg, 'libx264', 'h264'),
                           input=frames[0] + b''.join(frames), capture_output=True, check=True, timeout=30)
  decoded = subprocess.run([ffmpeg, '-v', 'error', '-f', 'h264', '-i', 'pipe:0',
                            '-threads:v', '1', '-filter_threads', '1', '-pix_fmt', 'rgb24',
                            '-f', 'rawvideo', 'pipe:1'], input=encoded.stdout,
                           capture_output=True, check=True, timeout=30).stdout
  assert len(decoded) == 8 * width * height * 3
  for frame in range(8):
    for x, y in ((width // 4, height // 4), (3 * width // 4, height // 4),
                 (width // 4, 3 * height // 4), (3 * width // 4, 3 * height // 4)):
      pos = ((frame * height + y) * width + x) * 3
      expected = (220 if x < width // 2 else 20, 220 if y < height // 2 else 20, 30 + frame * 20)
      assert max(abs(a - b) for a, b in zip(decoded[pos:pos + 3], expected)) < 12
