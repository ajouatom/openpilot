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
CUTIN_ALERT_MEMORY_S = 3.0
CUTIN_ALERT_MAX_ABS_VLAT_MPS = 4.0
CUTIN_ALERT_MIN_V_EGO_MPS = 15.0 / 3.6


@dataclass(frozen=True)
class CutinAlertCandidate:
  track_id: int
  d_rel: float
  y_rel: float
  v_rel: float
  v_lat: float = 0.0


def promoted_cutin_candidates(
  candidates: tuple[CutinAlertCandidate, ...],
  lead_two: CutinAlertCandidate | None,
) -> tuple[CutinAlertCandidate, ...]:
  """Return only the detected CUT-IN that was actually selected as leadTwo."""
  if lead_two is None or lead_two.track_id <= 0:  # 0 = forced-SCC fallback object
    return ()
  return tuple(
    candidate for candidate in candidates
    if (
      candidate.track_id == lead_two.track_id
      and abs(candidate.d_rel - lead_two.d_rel) <= CUTIN_ALERT_PROMOTION_MAX_DREL_M
      and abs(candidate.y_rel - lead_two.y_rel) <= CUTIN_ALERT_PROMOTION_MAX_YREL_M
      and abs(candidate.v_rel - lead_two.v_rel) <= CUTIN_ALERT_PROMOTION_MAX_VREL_MPS
      and abs(candidate.v_lat) <= CUTIN_ALERT_MAX_ABS_VLAT_MPS  # crossing traffic, not a cut-in
    )
  )


class CutinAlertTracker:
  """Detect newly appearing physical cut-in candidates without repeating on re-ID."""

  def __init__(self) -> None:
    self.previous: tuple[CutinAlertCandidate, ...] = ()
    self.recent: dict[int, tuple[CutinAlertCandidate, float]] = {}

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

  @staticmethod
  def _same_track_recently(current: CutinAlertCandidate, recent: CutinAlertCandidate) -> bool:
    # y is not limited: a cutting-in car moves laterally while bouncing between leadOne and leadTwo
    return (
      abs(current.d_rel - recent.d_rel) <= CUTIN_ALERT_SAME_ID_MAX_DREL_M and
      abs(current.v_rel - recent.v_rel) <= CUTIN_ALERT_SAME_ID_MAX_VREL_MPS
    )

  def _seen(self, candidate: CutinAlertCandidate) -> bool:
    if any(self._same_object(candidate, previous) for previous in self.previous):
      return True
    recent = self.recent.get(candidate.track_id) if candidate.track_id > 0 else None
    return recent is not None and self._same_track_recently(candidate, recent[0])

  def update(self, candidates: tuple[CutinAlertCandidate, ...], enabled: bool = True,
             now_s: float | None = None) -> bool:
    current = candidates if enabled else ()
    if now_s is not None:
      self.recent = {tid: (c, t) for tid, (c, t) in self.recent.items() if now_s - t <= CUTIN_ALERT_MEMORY_S}
    alert = bool(current) and any(not self._seen(candidate) for candidate in current)
    self.previous = current
    if now_s is not None:
      self.recent.update({c.track_id: (c, now_s) for c in current if c.track_id > 0})
    return alert

  def reset(self) -> None:
    self.previous = ()
    self.recent = {}
