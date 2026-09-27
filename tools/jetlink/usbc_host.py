"""Select the Orin Nano carrier's Type-C host policy through FUSB301 CC negotiation.

NVIDIA jetson_36.4.7 drivers/usb/typec/fusb301.c: fsw_trysnk=0 prevents
detach from restoring the sink preference. fmode=1 selects SRC and performs
error recovery/detach; the driver sets USB_ROLE_HOST only after detecting Rd.
Never force usb_role directly or write I2C registers behind the driver.
"""
import argparse
from pathlib import Path
import re
import time

BASE = Path('/sys/bus/i2c/devices/1-0025/fusb301')


def supported(root=Path('/')):
  try:
    compatible = (root/'proc/device-tree/compatible').read_bytes().split(b'\0')
    release = (root/'etc/nv_tegra_release').read_text()
    return (any(x.startswith(b'nvidia,p3768-') for x in compatible)
            and re.search(r'# R36 \(release\), REVISION: 4\.7,', release) is not None)
  except OSError:
    return False


def select_host(base=BASE):
  old_mode_text = (base/'fmode').read_text().strip()
  match = re.fullmatch(r'(?:SRC|SRC\+ACC|SNK|SNK\+ACC|DRP|DRP\+ACC)\((\d+)\)', old_mode_text)
  old_try = (base/'fsw_trysnk').read_text().strip()
  if not match or int(match[1]) not in (1, 2, 4, 8, 16, 32) or old_try not in ('0', '1'):
    raise ValueError('Unrecognized FUSB301 driver interface')
  old_mode = match[1]
  if old_mode == '1' and old_try == '0':
    return False  # Installing on an already-working link must not detach it.
  try:
    (base/'fsw_trysnk').write_text('0\n')
    (base/'fmode').write_text('1\n')
  except OSError:
    # Restore the previous policy when the kernel rejects a write.
    try:
      (base/'fmode').write_text(old_mode + '\n')
    finally:
      (base/'fsw_trysnk').write_text(old_try + '\n')
    raise
  return True


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('--restore-dual-role', action='store_true')
  args = parser.parse_args()
  if not supported():
    raise RuntimeError('USB-C policy supports P3768 / L4T 36.4.7 only')
  deadline = time.monotonic() + 30
  while not (BASE/'fmode').exists():
    if time.monotonic() >= deadline:
      raise RuntimeError('FUSB301 driver did not appear')
    time.sleep(.25)
  if args.restore_dual_role:
    (BASE/'fsw_trysnk').write_text('0\n')
    (BASE/'fmode').write_text('32\n')
    (BASE/'fsw_trysnk').write_text('1\n')
    print('USB-C default dual-role / Try.SNK restored')
  else:
    changed = select_host()
    print('USB-C host policy ' + ('selected' if changed else 'already selected'))


if __name__ == '__main__':
  main()
