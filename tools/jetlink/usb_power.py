#!/usr/bin/env python3
"""Root-only auxiliary USB input-current cut for independently powered installs.

This branch requires power through the vehicle/OBD-C connector. Apply even
with a host already attached; never change USB roles or restore USB input.

Adapted from zoompilot/jetlink 491cf1ef8d6786ed51cf995e18fc8721389066c6
(Copyright (c) 2026 Zeph Leggett, MIT). Keep this helper stdlib-only.
"""
import json
import os
from pathlib import Path
import sys

VOTER = Path('/sys/kernel/debug/pmic-votable/USB_ICL')


def input_status(voter=VOTER):
  active = (voter / 'force_active').read_text().strip()
  value = (voter / 'force_val').read_text().strip()
  return {'force_active': active, 'force_val': value,
          'input_blocked': active == '1' and value == '0'}


def cut_input(voter=VOTER):
  """Force zero input for every attachment state; require readback, never undo.

  No power-role, CAN, voltage or parking gate: the owner explicitly requires
  independent main power, including at cold boot with the host connected.
  An existing nonzero forced input limit is replaced by zero without ever
  disabling the force. Only the two USB_ICL nodes may be written.
  """
  active, value = voter / 'force_active', voter / 'force_val'
  current = input_status(voter)
  if current['force_active'] not in ('0', '1'):
    raise RuntimeError('unknown USB_ICL force_active value')
  # Missing nodes must not be created, even in a malformed/test filesystem.
  if not all(path.is_file() for path in (active, value)):
    raise RuntimeError('USB_ICL voter node is missing')
  value.write_text('0\n')
  # force_val only updates the voter variable. Writing force_active invokes
  # the PMIC callback, even when already active. Reassert it on every apply:
  # a previous failed callback can leave the variables at 1/0 without a cut.
  active.write_text('1\n')
  if not input_status(voter)['input_blocked']:
    raise RuntimeError('USB_ICL zero-current vote did not read back')
  return 'applied'


def main():
  if sys.argv[1:] not in ([], ['--status']) or os.geteuid() != 0:
    print('jetlink USB power helper requires root; only --status is optional', file=sys.stderr)
    return 1
  try:
    if sys.argv[1:] == ['--status']:
      print(json.dumps(input_status(), sort_keys=True))
      return 0
    result = cut_input()
  except (OSError, RuntimeError) as exc:
    print(f'jetlink USB input-current protection unavailable: {exc}', file=sys.stderr)
    return 1
  print(result)
  return 0


if __name__ == '__main__':
  sys.exit(main())
