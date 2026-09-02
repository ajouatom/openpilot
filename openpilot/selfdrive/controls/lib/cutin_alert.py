from dataclasses import dataclass


CUTIN_ALERT_REID_MAX_DREL_M = 1.5
CUTIN_ALERT_REID_MAX_YREL_M = 0.75
CUTIN_ALERT_REID_MAX_VREL_MPS = 2.5
CUTIN_ALERT_SAME_ID_MAX_DREL_M = 3.0
CUTIN_ALERT_SAME_ID_MAX_YREL_M = 1.0
CUTIN_ALERT_SAME_ID_MAX_VREL_MPS = 5.0
CUTIN_ALERT_PROMOTION_MAX_DREL_M = 0.1
CUTIN_ALERT_PROMOTION_MAX_YREL_M = 0.1
CUTIN_ALERT_PROMOTION_MAX_VREL_MPS = 0.1


@dataclass(frozen=True)
class CutinAlertCandidate:
  track_id: int
  d_rel: float
  y_rel: float
  v_rel: float


def promoted_cutin_candidates(
  candidates: tuple[CutinAlertCandidate, ...],
  lead_two: CutinAlertCandidate | None,
) -> tuple[CutinAlertCandidate, ...]:
  """Return only the detected CUT-IN that was actually selected as leadTwo."""
  if lead_two is None or lead_two.track_id < 0:
    return ()
  return tuple(
    candidate for candidate in candidates
    if (
      candidate.track_id == lead_two.track_id
      and abs(candidate.d_rel - lead_two.d_rel) <= CUTIN_ALERT_PROMOTION_MAX_DREL_M
      and abs(candidate.y_rel - lead_two.y_rel) <= CUTIN_ALERT_PROMOTION_MAX_YREL_M
      and abs(candidate.v_rel - lead_two.v_rel) <= CUTIN_ALERT_PROMOTION_MAX_VREL_MPS
    )
  )


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

  def update(self, candidates: tuple[CutinAlertCandidate, ...], enabled: bool = True) -> bool:
    current = candidates if enabled else ()
    alert = bool(current) and any(
      not any(self._same_object(candidate, previous) for previous in self.previous)
      for candidate in current
    )
    self.previous = current
    return alert

  def reset(self) -> None:
    self.previous = ()
