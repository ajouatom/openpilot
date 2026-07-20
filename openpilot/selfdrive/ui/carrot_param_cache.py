from collections.abc import Callable
from dataclasses import dataclass
from typing import Generic, TypeVar


CARROT_PARAM_REFRESH_INTERVAL = 0.5
PARAM_READ_RETRY_INTERVAL = 0.05
PENDING_WRITE_TIMEOUT = 1.0

T = TypeVar("T")
_NO_PENDING_WRITE = object()


class TimedSnapshotCache(Generic[T]):
  """Keep the last complete snapshot and retry failed reads with bounded backoff."""

  def __init__(self, initial: T, loader: Callable[[], T], interval: float = CARROT_PARAM_REFRESH_INTERVAL):
    self._value = initial
    self._loader = loader
    self._interval = interval
    self._next_refresh_time = 0.0
    self._read_failures = 0
    self._pending_write: object = _NO_PENDING_WRITE
    self._pending_write_deadline = 0.0

  @property
  def value(self) -> T:
    return self._value

  @property
  def next_refresh_time(self) -> float:
    return self._next_refresh_time

  def refresh(self, now: float) -> T:
    if now < self._next_refresh_time:
      return self._value

    try:
      value = self._loader()
    except Exception:
      # Retry once on the next 20 Hz UI frame, then back off to the normal TTL
      # so a persistent Params failure cannot restore per-frame I/O indefinitely.
      retry_delay = min(self._interval, PARAM_READ_RETRY_INTERVAL * (2 ** min(self._read_failures, 4)))
      self._read_failures += 1
      self._next_refresh_time = now + retry_delay
      return self._value

    self._read_failures = 0
    if self._pending_write is not _NO_PENDING_WRITE:
      if value != self._pending_write and now < self._pending_write_deadline:
        # A nonblocking write has not reached the Params file yet. Keep the
        # local source of truth and check for acknowledgement on the next TTL.
        self._next_refresh_time = now + self._interval
        return self._value
      self._pending_write = _NO_PENDING_WRITE

    self._value = value
    self._next_refresh_time = now + self._interval
    return value

  def store_pending(self, value: T, now: float) -> None:
    """Keep a nonblocking write authoritative until observed or its timeout."""
    self._pending_write = value
    self._pending_write_deadline = now + PENDING_WRITE_TIMEOUT
    self._read_failures = 0
    self._value = value
    self._next_refresh_time = now + self._interval


@dataclass(frozen=True)
class BorderParamSnapshot:
  top_left: str = ""
  top_left_op_long: str = ""
  custom_sr: float = 0.0
  bottom_left: str = ""
  bottom_right: str = ""


@dataclass(frozen=True)
class RealtimeUiParamSnapshot:
  record_audio: bool = False
  is_metric: bool = False
  always_on_dm: bool = False


def read_border_params(params, params_memory) -> BorderParamSnapshot:
  car_name = params.get("CarName") or ""
  hyundai_camera_scc = params.get_int("HyundaiCameraSCC") > 0
  nnff_model_name = params.get("NNFFModelName") or ""
  custom_sr = params.get_float("CustomSR") / 10.0
  git_branch = params.get("GitBranch") or ""
  network_address = params_memory.get("NetworkAddress") or ""

  top_left = car_name + ("(CAMERA SCC)" if hyundai_camera_scc else "")
  top_left_op_long = top_left if hyundai_camera_scc else car_name + " - OP Long"
  if nnff_model_name:
    top_left += ",NNFF"
    top_left_op_long += ",NNFF"

  return BorderParamSnapshot(
    top_left=top_left,
    top_left_op_long=top_left_op_long,
    custom_sr=custom_sr,
    bottom_left=git_branch,
    bottom_right=network_address,
  )


def read_realtime_ui_params(params) -> RealtimeUiParamSnapshot:
  return RealtimeUiParamSnapshot(
    record_audio=params.get_bool("RecordAudio"),
    is_metric=params.get_bool("IsMetric"),
    always_on_dm=params.get_bool("AlwaysOnDM"),
  )


def read_screen_record(params) -> bool:
  return params.get_bool("ScreenRecord")
