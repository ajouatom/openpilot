"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of jetlink and is licensed under the MIT License.
See the LICENSE file in the root directory for more details.

Suspend the Jetson when nobody is talking to it.

On an always-on supply the box idles at ~7 W, a flat battery over a long park.
Deep suspend keeps the engine resident: ~6 s back, against ~65 s for a cold boot
and a plan reload.

USB is the wake source and both edges wake it, so this is a loop rather than a
command from the comma: ignition-off pulls the gadget and wakes the box, and no
gadget for SLEEP_AFTER means sleep again. Writing /sys/power/state blocks until
resume, so the poll loop continues where it stopped and the engine host, the
CUDA context and libusb all survive.

Not every attempt sleeps: the freezer can return EBUSY, and a wake edge can land
between the check and the write, neither of them visible in the write's return
value. suspend_stats is the proof, and a failure backs off rather than freezing
the box again 20 s later.
"""
from __future__ import annotations

import errno
import logging
import time
from pathlib import Path

log = logging.getLogger('jetlink.sleep')

# Longer than the handover's re-enumeration, observed at 45 to 70 s: sleeping
# inside that gap costs the next connect a resume.
SLEEP_AFTER = 120.0

RETRY_MIN = 10.0
RETRY_MAX = 300.0

# The kernel's own freezer timeout; an attempt cannot take longer than this
# and still have failed to freeze.
FREEZER_TIMEOUT = 20.0

# The USB wake is not guaranteed: a sleeping Jetson answered a bind with a bus
# reset and no enumeration through four connect cycles and needed its button,
# which in the car is a whole drive on the small model. An RTC alarm does not
# depend on the path that just failed, and costs under a tenth of a watt.
WAKE_BACKSTOP = 1800.0
RTC = '/sys/class/rtc/rtc0'

# Where the hubs live, and the class code that says a USB device is one.
USB_DEVICES = '/sys/bus/usb/devices'
HUB_CLASS = '09'


def _boottime() -> float:
  # counts through a suspend, unlike CLOCK_MONOTONIC, so it measures the sleep
  clock = getattr(time, 'CLOCK_BOOTTIME', None)
  return time.clock_gettime(clock) if clock is not None else time.monotonic()


class Sleeper:
  """Decides when the server has been orphaned long enough to suspend.

  `touch()` whenever a gadget is seen; `idle()` from the poll loop whenever
  it is not. `idle()` returns without doing anything until the orphan timeout
  has run out, then suspends and returns once the box is back.
  """

  def __init__(self, after: float = SLEEP_AFTER, power: str | Path = '/sys/power',
               rtc: str | Path = RTC, backstop: float = WAKE_BACKSTOP,
               usb: str | Path = USB_DEVICES):
    self.after = float(after)
    self.power = Path(power)
    self.rtc = Path(rtc)
    self.backstop = float(backstop)
    self.usb = Path(usb)
    self.enabled = True
    self._last_seen = time.monotonic()
    self._retry_at = 0.0
    self._backoff = RETRY_MIN
    self.slept = 0
    self.failed = 0

  def touch(self) -> None:
    self._last_seen = time.monotonic()
    self._backoff = RETRY_MIN

  def orphaned(self) -> bool:
    now = time.monotonic()
    return (self.enabled and now - self._last_seen >= self.after
            and now >= self._retry_at)

  def idle(self) -> bool:
    """Called with no gadget present. Returns True if we slept."""
    if not self.orphaned():
      return False
    ok = self.suspend()
    now = time.monotonic()
    if ok:
      # Woken by an edge; give whatever caused it the full timeout to show up.
      self._last_seen = now
      self._backoff = RETRY_MIN
    else:
      self._retry_at = now + self._backoff
      self._backoff = min(self._backoff * 2, RETRY_MAX)
    return ok

  # -- sysfs -----------------------------------------------------------------

  def _read(self, name: str) -> str:
    try:
      return (self.power / name).read_text().strip()
    except OSError:
      return ''

  def _read_int(self, path: str) -> int:
    try:
      return int((self.power / path).read_text())
    except (OSError, ValueError):
      return -1

  def _select_deep(self) -> bool:
    """s2idle keeps the CPUs in idle states and saves nothing worth having."""
    modes = self._read('mem_sleep')
    if not modes:
      # No mem_sleep at all: "mem" means whatever the platform does.
      return True
    if '[deep]' in modes:
      return True
    if 'deep' not in modes.split():
      log.error("deep suspend is not available (mem_sleep: %s), not sleeping", modes)
      return False
    try:
      (self.power / 'mem_sleep').write_text('deep')
    except OSError as e:
      log.error("could not select deep suspend: %s", e)
      return False
    return True

  def suspend(self) -> bool:
    if not self._select_deep():
      self.enabled = False
      return False
    before = self._read_int('suspend_stats/success')
    t0 = _boottime()
    log.info("no gadget for %.0f s, suspending", self.after)
    self._check_usb_wakeup()
    armed = self._arm_backstop()
    try:
      self._enter()
    except OSError as e:
      self._disarm_backstop(armed)
      if e.errno in (errno.EACCES, errno.EPERM, errno.EROFS, errno.ENOENT):
        # Configuration, not weather: run.sh and the unit mount /sys/power
        # read-write; see docs/transport.md.
        log.error("cannot write %s (%s); sleep disabled", self.power / 'state', e)
        self.enabled = False
        self.failed += 1
        return False
      # EBUSY is the freezer giving up, EINVAL a mode the platform refused.
      # Both are worth another try later.
      log.warning("suspend failed: %s (%s)", e, self._failure())
      self.failed += 1
      return False
    self._disarm_backstop(armed)
    asleep = _boottime() - t0
    after = self._read_int('suspend_stats/success')
    if before >= 0 and after <= before:
      # A clean return with the counter unmoved: a wake edge landed during the
      # freeze and the box never left.
      log.warning("suspend returned after %.1f s without sleeping (%s)",
                  asleep, self._failure())
      self.failed += 1
      return False
    self.slept += 1
    log.info("resumed after %.0f s asleep", asleep)
    return True

  def _check_usb_wakeup(self) -> list[str]:
    """Arm remote wakeup on every hub, and say so loudly when it cannot.

    The comma hangs off the onboard Realtek hub, which has to signal a connect
    up before the root hub or tegra-xusb hear about it, and which ships disarmed
    while those do not. Disarmed, a sleeping Jetson answered a bind with a bus
    reset and needed its button; armed, it resumed 4 s after the bind. Which hub
    carries the gadget depends on the negotiated speed, and the USB 2 one ships
    armed, so check them all.

    Arming belongs to the host (99-jetlink-usb-wakeup.rules, and the unit's
    ExecStartPre) because /sys is read-only here; try anyway, for a deployment
    that mounts it read-write.
    """
    disarmed = []
    try:
      devices = sorted(self.usb.iterdir())
    except OSError:
      return disarmed          # no USB tree visible; nothing to say about it
    for dev in devices:
      try:
        if (dev / 'bDeviceClass').read_text().strip() != HUB_CLASS:
          continue
        wakeup = dev / 'power' / 'wakeup'
        if wakeup.read_text().strip() != 'disabled':
          continue
      except OSError:
        # Not a hub: this directory also holds interfaces ("2-1:1.0"), which
        # have no bDeviceClass, so the read raises. Counting those as failures
        # named six interfaces in an error about a rule that was working.
        continue
      try:
        wakeup.write_text('enabled\n')
      except OSError:
        disarmed.append(dev.name)
    if disarmed:
      log.error("hub(s) %s are not armed for remote wakeup and could not be armed from "
                "in here (/sys is read-only): the comma presenting its gadget may not "
                "wake this box. Install 99-jetlink-usb-wakeup.rules on the host.",
                ', '.join(disarmed))
    return disarmed

  def _arm_backstop(self) -> bool:
    """Set an RTC alarm, so a wake the USB edge misses still happens.

    Against the RTC's own count, not the wall clock: this box boots unset and
    never sees NTP in the car, so since_epoch may be years out and only both
    sides coming from the same counter matters.
    """
    if self.backstop <= 0:
      return False
    try:
      alarm = self.rtc / 'wakealarm'
      now = int((self.rtc / 'since_epoch').read_text().strip())
      alarm.write_text('0\n')            # a stale alarm blocks setting a new one
      alarm.write_text(f"{now + int(self.backstop)}\n")
      return True
    except (OSError, ValueError) as e:
      # No RTC, no alarm support, or not writable from in here. The USB edge
      # is still the wake source; this was only the backstop.
      log.warning("could not arm the %.0f s wake backstop: %s", self.backstop, e)
      return False

  def _disarm_backstop(self, armed: bool) -> None:
    if not armed:
      return
    try:
      (self.rtc / 'wakealarm').write_text('0\n')
    except OSError:
      pass   # it has either fired or it fires once and is spent

  def _enter(self) -> None:
    # Blocks until resume. Split out so a test can stand in for the kernel.
    (self.power / 'state').write_text('mem')

  def _failure(self) -> str:
    step = self._read('suspend_stats/last_failed_step') or '?'
    dev = self._read('suspend_stats/last_failed_dev')
    return f"last failed step {step}" + (f" in {dev}" if dev else '')
