"""Display placement and bounded RT slices for the onroad UI."""
import os
import subprocess
import sys
import threading
import time
from pathlib import Path

LITTLE_CORES = {0, 1, 2, 3}
DISPLAY_NICE = 19

# --- Bounded RT slice for the render thread (onroad only) --------------------
# nice 19 keeps the display out of the way, but then any background burst
# preempts the render thread: a C3 in engaged driving measured ~0.6 s/s of
# runqueue wait against ~0.3 s/s of frame CPU, i.e. 13-14 fps instead of 20.
# Raising nice ("core tune") restored the frames but let the display take CPU
# without a limit, which delayed other work.  Keeping the render thread at a
# low-priority SCHED_FIFO inside a cgroup with a cpu.rt_runtime_us budget
# bounds that: the thread preempts CFS work just enough to hit its frame
# deadline, is throttled past the budget, and every driving RT thread
# (priorities 51..55) still preempts it trivially.  The root group's RT budget
# is left untouched, so driving RT keeps its full allocation.  Setup needs
# root once per boot; AGNOS grants passwordless sudo.
DISPLAY_RT_PRIO = 8  # below every driving RT priority (51..55)
DISPLAY_RT_PERIOD_US = 1_000_000
DISPLAY_RT_BUDGET_US = 400_000  # 40% of one CPU per second
DISPLAY_RT_GROUP = 'openpilot_ui'
DISPLAY_RT_CGROUP = '/sys/fs/cgroup/cpu'
DISPLAY_RT_BUDGET_PARAM = '/data/params/d/CarrotUiRtBudgetUs'
DISPLAY_RT_SETUP_TIMEOUT_S = 10

# Optional schedtune (WALT) boost.  Android raises a task's performance point
# this way without touching nice; on this C3 a boost of 15/30 + prefer_idle
# changed nothing measurable (13.7 fps with and without), so it stays opt-in
# through CARROT_UI_SCHEDTUNE=1.
SCHEDTUNE_CGROUP = '/sys/fs/cgroup/schedtune'
SCHEDTUNE_GROUP = 'openpilot_ui'
SCHEDTUNE_BOOST = 15


def _current_user() -> str:
  try:
    import pwd
    return pwd.getpwuid(os.getuid()).pw_name
  except (ImportError, KeyError):
    return os.environ.get('USER', 'comma')


def _warn(message: str) -> None:
  try:
    from openpilot.common.swaglog import cloudlog
    cloudlog.warning(message)
  except Exception:
    pass


def _run_root(script: str) -> bool:
  try:
    done = subprocess.run(['sudo', '-n', 'sh', '-c', script], capture_output=True, text=True,
                          timeout=DISPLAY_RT_SETUP_TIMEOUT_S, check=False)
    return done.returncode == 0
  except (OSError, subprocess.SubprocessError):
    return False


def _rt_setup_script(user: str, budget_us: int) -> str:
  group = f'{DISPLAY_RT_CGROUP}/{DISPLAY_RT_GROUP}'
  return '; '.join((
    'set -e',
    f'mkdir -p {DISPLAY_RT_CGROUP}',
    f'mountpoint -q {DISPLAY_RT_CGROUP} || mount -t cgroup -o cpu none {DISPLAY_RT_CGROUP}',
    f'mkdir -p {group}',
    f'echo {DISPLAY_RT_PERIOD_US} > {group}/cpu.rt_period_us',
    f'echo {budget_us} > {group}/cpu.rt_runtime_us',
    f'chown -R {user}:{user} {group}',
  ))


def _schedtune_setup_script(user: str) -> str:
  group = f'{SCHEDTUNE_CGROUP}/{SCHEDTUNE_GROUP}'
  return '; '.join((
    'set -e',
    f'mkdir -p {SCHEDTUNE_CGROUP}',
    f'mountpoint -q {SCHEDTUNE_CGROUP} || mount -t cgroup -o schedtune none {SCHEDTUNE_CGROUP}',
    f'mkdir -p {group}',
    f'chown -R {user}:{user} {group}',
  ))


def core_online(core: int) -> bool:
  try:
    return Path(f'/sys/devices/system/cpu/cpu{core}/online').read_text().strip() == '1'
  except OSError:
    return False


def thread_ids(pid: int | str = 'self') -> list[int]:
  try:
    return [int(p.name) for p in Path(f'/proc/{pid}/task').iterdir()]
  except FileNotFoundError:
    return []  # Encoder exited before its worker sweep.


class DisplayScheduler:
  def __init__(self, onroad_core: int, *, enabled: bool, include_little: bool = False,
               rt_budget: bool = False, schedtune_boost: bool = False):
    self.core = onroad_core
    self.include_little = include_little
    self.enabled = enabled and sys.platform == 'linux'
    self.onroad = None
    self.next_check = 0.0
    # bounded onroad RT slice for the calling (render) thread
    self.rt_enabled = rt_budget and sys.platform == 'linux' and os.getenv('CARROT_UI_RT', '1') != '0'
    self.rt_onroad = None
    self.rt_tid: int | None = None
    self.rt_setup_ok: bool | None = None
    # optional schedtune (WALT) performance-point boost
    self.schedtune_enabled = schedtune_boost and sys.platform == 'linux'
    self.schedtune_onroad = None
    self.schedtune_setup_ok: bool | None = None

  def update(self, onroad: bool, *, force: bool = False, child_pid: int | None = None) -> None:
    if not self.enabled:
      return
    now = time.monotonic()
    if not force and onroad == self.onroad and now < self.next_check:
      return
    self.onroad = onroad
    self.next_check = now + 0.5
    self._update_rt(onroad)
    self._update_schedtune(onroad)
    use_big = onroad and core_online(self.core)
    cores = {self.core} if use_big else LITTLE_CORES
    if onroad and self.include_little:
      cores = cores | LITTLE_CORES
    low_priority = use_big or (onroad and self.include_little)
    nice = DISPLAY_NICE if low_priority else 0
    workers = thread_ids()
    if child_pid is not None:
      workers.extend(thread_ids(child_pid))
    for tid in workers:
      try:
        # While the render thread carries the bounded RT slice its policy is
        # owned there; the sweep must not demote it back to SCHED_OTHER.
        if tid != self.rt_tid:
          # Lower display priority before moving onto camera/model CPUs. New
          # workers inherit this policy and are checked on the next sweep.
          if os.sched_getscheduler(tid) != os.SCHED_OTHER:
            os.sched_setscheduler(tid, os.SCHED_OTHER, os.sched_param(0))
          if low_priority and os.getpriority(os.PRIO_PROCESS, tid) != nice:
            os.setpriority(os.PRIO_PROCESS, tid, nice)
        if os.sched_getaffinity(tid) != cores:
          try:
            os.sched_setaffinity(tid, cores)
          except OSError:
            # A power-save transition can offline the target after the check.
            os.sched_setaffinity(tid, LITTLE_CORES)
        if tid != self.rt_tid and not low_priority and os.getpriority(os.PRIO_PROCESS, tid) != nice:
          try:
            os.setpriority(os.PRIO_PROCESS, tid, nice)
          except PermissionError:
            # Older service limits may prohibit raising nice again. Retaining
            # low priority is safe; never leave the worker pinned to an offline CPU.
            pass
      except ProcessLookupError:
        pass  # Worker exited during enumeration.

  # --- bounded RT slice -------------------------------------------------------

  def _update_rt(self, onroad: bool) -> None:
    if not self.rt_enabled or onroad == self.rt_onroad:
      return
    self.rt_onroad = onroad
    if not onroad:
      self._drop_rt()
      return
    if not self._ensure_rt_setup():
      return
    tid = threading.get_native_id()
    try:
      (Path(DISPLAY_RT_CGROUP) / DISPLAY_RT_GROUP / 'tasks').write_text(f'{tid}\n')
      os.sched_setscheduler(tid, os.SCHED_FIFO, os.sched_param(DISPLAY_RT_PRIO))
      self.rt_tid = tid
    except OSError:
      self._drop_rt(tid)
      _warn('display_scheduling: RT display slice unavailable; keeping nice 19')

  def _drop_rt(self, tid: int | None = None) -> None:
    target = self.rt_tid if tid is None else tid
    self.rt_tid = None
    if target is None:
      return
    try:
      os.sched_setscheduler(target, os.SCHED_OTHER, os.sched_param(0))
    except OSError:
      pass

  def _rt_budget_us(self) -> int:
    try:
      budget = int(Path(DISPLAY_RT_BUDGET_PARAM).read_text().strip())
    except (OSError, ValueError):
      return DISPLAY_RT_BUDGET_US
    return min(max(budget, 50_000), DISPLAY_RT_PERIOD_US)

  def _ensure_rt_setup(self) -> bool:
    if self.rt_setup_ok is not None:
      return self.rt_setup_ok
    try:
      self.rt_setup_ok = (Path(DISPLAY_RT_CGROUP) / DISPLAY_RT_GROUP / 'cpu.rt_runtime_us').exists()
    except OSError:
      self.rt_setup_ok = False
    if not self.rt_setup_ok:
      self.rt_setup_ok = _run_root(_rt_setup_script(_current_user(), self._rt_budget_us()))
    if not self.rt_setup_ok:
      _warn('display_scheduling: could not create the RT display cgroup; staying on nice 19')
    return self.rt_setup_ok

  # --- optional schedtune boost -----------------------------------------------

  def _update_schedtune(self, onroad: bool) -> None:
    if not self.schedtune_enabled or onroad == self.schedtune_onroad:
      return
    self.schedtune_onroad = onroad
    group = Path(SCHEDTUNE_CGROUP) / SCHEDTUNE_GROUP
    if not onroad:
      try:
        (group / 'schedtune.boost').write_text('0\n')
      except OSError:
        pass
      return
    if self.schedtune_setup_ok is None:
      try:
        self.schedtune_setup_ok = (group / 'schedtune.boost').exists()
      except OSError:
        self.schedtune_setup_ok = False
      if not self.schedtune_setup_ok:
        self.schedtune_setup_ok = _run_root(_schedtune_setup_script(_current_user()))
    if not self.schedtune_setup_ok:
      _warn('display_scheduling: could not create the schedtune group')
      return
    try:
      (group / 'cgroup.procs').write_text(f'{os.getpid()}\n')
      (group / 'schedtune.boost').write_text(f'{SCHEDTUNE_BOOST}\n')
    except OSError:
      _warn('display_scheduling: schedtune boost unavailable')
