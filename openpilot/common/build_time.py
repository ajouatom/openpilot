"""Offline clock floor for device startup, before any native build imports."""
from datetime import UTC, datetime
from pathlib import Path
import subprocess
import time


def _utc(epoch: float) -> str:
  return datetime.fromtimestamp(epoch, UTC).isoformat()


def ensure_build_time(repo: Path) -> None:
  # Use the checked-out commit, not an upstream ref or author date. No network.
  result = subprocess.run(['git', '-C', str(repo), 'show', '-s', '--format=%ct', 'HEAD'],
                          check=True, capture_output=True, text=True, timeout=10)
  commit_time = int(result.stdout.strip())
  if commit_time <= 0:
    raise ValueError('Invalid HEAD commit timestamp')
  commit_utc = _utc(commit_time)
  now = time.time()  # noqa: TID251 -- compare the system wall clock with Git's UTC epoch
  print(f'[build-time] device={_utc(now)} HEAD={commit_utc}', flush=True)
  if now >= commit_time:
    print('[build-time] Clock already meets commit floor; unchanged.', flush=True)
    return

  # Recheck immediately before setting: NTP may have caught up during Git I/O
  # or logging. Do not deliberately move an already-current clock backwards.
  if time.time() >= commit_time:  # noqa: TID251
    print('[build-time] Clock caught up; unchanged.', flush=True)
    return
  target = commit_time + 1
  print(f'[build-time] Advancing clock to {_utc(target)} (offline commit floor).', flush=True)
  subprocess.run(['sudo', '-n', 'date', '-u', '-s', f'@{target}'],
                 check=True, capture_output=True, text=True, timeout=10)
  after = time.time()  # noqa: TID251
  print(f'[build-time] After correction: {_utc(after)}', flush=True)
  if after < commit_time:
    raise RuntimeError('System clock is still older than HEAD after correction')


def main() -> int:
  try:
    ensure_build_time(Path(__file__).resolve().parents[2])
  except (OSError, ValueError, OverflowError, RuntimeError, subprocess.SubprocessError) as exc:
    detail = getattr(exc, 'stderr', None) or str(exc)
    print(f'[build-time] Failed: {detail.strip()}', flush=True)
    return 1
  return 0


if __name__ == '__main__':
  raise SystemExit(main())
