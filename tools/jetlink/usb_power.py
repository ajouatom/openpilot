#!/usr/bin/env python3
"""Root-only USB input-current guard; never changes USB roles or restores draw.

Adapted from zoompilot/jetlink 491cf1ef8d6786ed51cf995e18fc8721389066c6
(Copyright (c) 2026 Zeph Leggett, MIT). Keep this helper stdlib-only.
"""
import os
from pathlib import Path
import sys

POWER_ROLE = Path('/sys/class/usbpd/usbpd0/current_pr')
VOTER = Path('/sys/kernel/debug/pmic-votable/USB_ICL')


def cut_input(role=POWER_ROLE, voter=VOTER):
  """Return applied/deferred; raise on missing nodes, conflict or failed writes.

  The caller debounces an empty port. Check again here, immediately before
  changing the charger, because sudo/process startup can overlap attachment.
  Even explicit iOS configuration is not proof of an independently powered
  comma, so Carrot deliberately omits upstream's attached-iPhone exception.
  """
  if role.read_text().strip() != 'none':
    return 'deferred'
  active, value = voter / 'force_active', voter / 'force_val'
  forced = active.read_text().strip()
  if forced == '1':
    if value.read_text().strip() == '0':
      return 'applied'  # A previous daemon already installed this boot's guard.
    raise RuntimeError('USB_ICL has an existing nonzero forced vote')
  if forced != '0':
    raise RuntimeError('unknown USB_ICL force_active value')
  # Missing nodes must not be created, even in a malformed/test filesystem.
  if not value.is_file():
    raise RuntimeError('USB_ICL force_val is missing')
  if role.read_text().strip() != 'none':
    return 'deferred'
  value.write_text('0\n')
  if role.read_text().strip() != 'none':
    return 'deferred'  # force_val alone is inactive; do not enable it now.
  active.write_text('1\n')
  if active.read_text().strip() != '1' or value.read_text().strip() != '0':
    raise RuntimeError('USB_ICL zero-current vote did not read back')
  return 'applied'


def main():
  if len(sys.argv) != 1 or os.geteuid() != 0:
    print('jetlink USB power helper requires root and takes no arguments', file=sys.stderr)
    return 1
  try:
    result = cut_input()
  except (OSError, RuntimeError) as exc:
    print(f'jetlink USB input-current protection unavailable: {exc}', file=sys.stderr)
    return 1
  print(result)
  return 0 if result == 'applied' else 3


if __name__ == '__main__':
  sys.exit(main())
