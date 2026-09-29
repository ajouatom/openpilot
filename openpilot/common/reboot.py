"""Device reboot with a bounded, best-effort chime; parent imports only stdlib."""
import argparse
from pathlib import Path
import subprocess
import sys
import time
import wave


SOUND_TIMEOUT = 4.0
REBOOT_TIMEOUT = 15.0
SOUND_PATH = Path(__file__).resolve().parents[1] / 'selfdrive/assets/sounds_eng/prompt.wav'
PARAMS_PATH = Path('/data/params/d')


def _device_only():
  if not (Path('/AGNOS').exists() or Path('/TICI').exists()):
    raise RuntimeError('Device reboot is disabled on this computer.')


def _sound_volume() -> float:
  # Keep startup recovery independent of the native Params extension.
  try:
    adjust = int((PARAMS_PATH / 'SoundVolumeAdjust').read_text()) / 100.0
  except (OSError, ValueError):
    adjust = 1.0
  return max(0.0, min(1.0, 0.5 * adjust))


def _play_sound():
  # Isolated child: missing dependencies, a busy device or hung audio must never
  # prevent reboot. This also works offroad, when soundd is stopped.
  import numpy as np
  import sounddevice as sd

  volume = _sound_volume()
  if volume == 0:
    return
  with wave.open(str(SOUND_PATH), 'rb') as wav:
    if wav.getsampwidth() != 2:
      raise ValueError('Expected 16-bit reboot sound')
    samples = np.frombuffer(wav.readframes(wav.getnframes()), dtype='<i2')
    samples = samples.reshape(-1, wav.getnchannels()).astype(np.float32) / 32768.0
    # A short silent lead-in lets the output device wake before the chime.
    lead_in = np.zeros((int(wav.getframerate() * 0.15), wav.getnchannels()), dtype=np.float32)
    sd.play(np.concatenate((lead_in, samples * volume)), wav.getframerate(), blocking=True)


def play_reboot_sound():
  try:
    subprocess.run([sys.executable, str(Path(__file__).resolve()), '--sound-only'],
                   check=True, timeout=SOUND_TIMEOUT, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
  except (OSError, subprocess.SubprocessError) as exc:
    print(f'[reboot] chime unavailable: {type(exc).__name__}', flush=True)


def reboot_device():
  _device_only()
  play_reboot_sound()
  subprocess.run(['sudo', '-n', 'reboot'], check=True, capture_output=True, timeout=REBOOT_TIMEOUT)


def spawn_reboot(delay: float = 0.0):
  """Keep web handlers responsive while the child finishes audio before reboot."""
  _device_only()
  return subprocess.Popen([sys.executable, str(Path(__file__).resolve()), '--delay', str(delay)], start_new_session=True)


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('--sound-only', action='store_true')
  parser.add_argument('--delay', type=float, default=0.0)
  args = parser.parse_args()
  if args.sound_only:
    _play_sound()
  else:
    _device_only()
    time.sleep(max(0.0, min(5.0, args.delay)))
    reboot_device()


if __name__ == '__main__':
  main()
