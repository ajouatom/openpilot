"""Detect a changed checkout without replacing the manager's startup identity."""
import json
from pathlib import Path
import re
import subprocess


UPDATE_CHECK_INTERVAL = 5.0


def read_checkout_commit(repo: Path) -> str | None:
  try:
    if (repo / '.git').exists():
      result = subprocess.run(['git', '--no-optional-locks', 'rev-parse', '--verify', 'HEAD^{commit}'],
                              cwd=repo, capture_output=True, text=True, timeout=1)
      commit = result.stdout.strip() if result.returncode == 0 else None
    else:
      # Packaged releases may have build metadata without a Git directory.
      commit = json.loads((repo / 'build.json').read_text(encoding='utf-8'))['openpilot']['git_commit']
  except (OSError, subprocess.SubprocessError, ValueError, KeyError, TypeError):
    return None
  return commit.lower() if isinstance(commit, str) and re.fullmatch(r'[0-9a-fA-F]{40}|[0-9a-fA-F]{64}', commit) else None


class UpdateStatus:
  def __init__(self, repo: str):
    self.repo = Path(repo)
    # Captured under the boot checkout lock, before managed processes start.
    # Never recapture this on ignition changes or after a failed read.
    self.running_commit = read_checkout_commit(self.repo)
    self.reboot_required = False
    self._candidate_commit: str | None = None
    self._next_check = 0.0

  def update(self, now: float) -> bool:
    if now < self._next_check:
      return self.reboot_required
    self._next_check = now + UPDATE_CHECK_INTERVAL
    installed_commit = read_checkout_commit(self.repo)
    changed = bool(self.running_commit and installed_commit and installed_commit != self.running_commit)
    # Require two successful, matching reads to avoid a transient checkout change.
    self.reboot_required = changed and installed_commit == self._candidate_commit
    self._candidate_commit = installed_commit if changed else None
    return self.reboot_required
