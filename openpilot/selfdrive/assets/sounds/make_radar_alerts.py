#!/usr/bin/env python3
"""Generate the radar cut-in tone."""

from array import array
import math
from pathlib import Path
import wave


SAMPLE_RATE = 48_000
AMPLITUDE = 0.62 * (2**15 - 1)
FADE_SECONDS = 0.006


def tone(frequency: float, duration: float) -> array:
  sample_count = round(SAMPLE_RATE * duration)
  fade_count = round(SAMPLE_RATE * FADE_SECONDS)
  samples = array("h")
  for index in range(sample_count):
    envelope = min(1.0, index / fade_count, (sample_count - 1 - index) / fade_count)
    sample = AMPLITUDE * envelope * math.sin(2.0 * math.pi * frequency * index / SAMPLE_RATE)
    samples.append(round(sample))
  return samples


def write_alert(path: Path, tones: tuple[tuple[float, float], ...]) -> None:
  samples = array("h")
  for frequency, duration in tones:
    samples.extend(tone(frequency, duration))
  with wave.open(str(path), "wb") as output:
    output.setnchannels(1)
    output.setsampwidth(2)
    output.setframerate(SAMPLE_RATE)
    output.writeframes(samples.tobytes())


def main() -> None:
  output_dir = Path(__file__).resolve().parent.parent / "sounds_eng"
  write_alert(output_dir / "radar_cutin.wav", ((1400.0, 0.140), (1900.0, 0.180)))


if __name__ == "__main__":
  main()
