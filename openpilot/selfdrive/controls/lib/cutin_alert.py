from dataclasses import dataclass


CUTIN_ALERT_REID_MAX_DREL_M = 1.5
CUTIN_ALERT_REID_MAX_YREL_M = 0.75
CUTIN_ALERT_REID_MAX_VREL_MPS = 2.5
CUTIN_ALERT_SAME_ID_MAX_DREL_M = 3.0
CUTIN_ALERT_SAME_ID_MAX_YREL_M = 1.0
CUTIN_ALERT_SAME_ID_MAX_VREL_MPS = 5.0


@dataclass(frozen=True)
class CutinAlertCandidate:
  track_id: int
  d_rel: float
  y_rel: float
  v_rel: float


class CutinAlertTracker:
  """Detect newly appearing physical cut-in candidates without repeating on re-ID."""

  def __init__(self) -> None:
    self.previous: tuple[CutinAlertCandidate, ...] = ()

  @staticmethod
  def _same_object(current: CutinAlertCandidate, previous: CutinAlertCandidate) -> bool:
    same_track = current.track_id >= 0 and current.track_id == previous.track_id
    d_limit = CUTIN_ALERT_SAME_ID_MAX_DREL_M if same_track else CUTIN_ALERT_REID_MAX_DREL_M
    y_limit = CUTIN_ALERT_SAME_ID_MAX_YREL_M if same_track else CUTIN_ALERT_REID_MAX_YREL_M
    v_limit = CUTIN_ALERT_SAME_ID_MAX_VREL_MPS if same_track else CUTIN_ALERT_REID_MAX_VREL_MPS
    return (
      abs(current.d_rel - previous.d_rel) <= d_limit and
      abs(current.y_rel - previous.y_rel) <= y_limit and
      abs(current.v_rel - previous.v_rel) <= v_limit
    )

  def update(
    self,
    candidates: tuple[CutinAlertCandidate, ...],
    enabled: bool = True,
    alert_candidates: tuple[CutinAlertCandidate, ...] | None = None,
  ) -> bool:
    current = candidates if enabled else ()
    audible = (
      current
      if alert_candidates is None
      else alert_candidates if enabled else ()
    )
    retained = tuple(
      candidate for candidate in current
      if any(self._same_object(candidate, previous) for previous in self.previous)
    )
    newly_audible = tuple(
      candidate for candidate in audible
      if not any(self._same_object(candidate, previous) for previous in self.previous)
    )
    alert = bool(newly_audible)
    self.previous = retained + tuple(
      candidate for candidate in newly_audible
      if not any(self._same_object(candidate, value) for value in retained)
    )
    return alert

  def reset(self) -> None:
    self.previous = ()
