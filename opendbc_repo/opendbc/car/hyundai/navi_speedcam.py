"""Stock-navigation speed-camera kind policy for Hyundai CAN-FD 0x4BE/0x4A3.

0x4BE PROLONG_VALUE (profile type 16) carries a camera kind in bits 0..3, a
speed code in bits 4..8 and extra flags above bit 9. Kinds were matched against
the stock navigation screen and labelled drives (2026-09-27):

  kind 0  fixed speed camera (also section start/end cameras)
  kind 1  signal-and-speed camera
  kind 2  mobile-camera box (spot camera; labelled 2026-09-27, e.g. 0x152 = box @100 km/h)
  kind 3  mobile enforcement zone (blue circle; enforcement anywhere in the zone)
  value >> 9 == 0x321: rear-facing flag, independent of the kind
    kind 0 + 0x321  rear speed camera (TMAP 75)
    kind 1 + 0x321  rear signal-and-speed camera (TMAP 76)

0x4A3 only says "a camera warning is active" without a kind, so this policy
attaches the kind of the nearby same-speed preview to the current warning.
"""

KIND_FIXED = 0
KIND_SIGNAL = 1
KIND_BOX = 2
KIND_MOBILE_ZONE = 3
CAMERA_KINDS = (KIND_FIXED, KIND_SIGNAL, KIND_BOX, KIND_MOBILE_ZONE)

CLASS_MOBILE_ZONE = 1
CLASS_BOX = 2
CLASS_HARD = 3  # fixed, signal, rear and unknown cameras always decelerate
# [experimental] an unknown warning (no camera preview matched) may be skipped with its own
# toggle, but never at 30 km/h or below: school and senior zones warn there, often without a preview.
PROTECTED_ZONE_KPH = 30

PREVIEW_SATURATED_OFFSET = 1985  # offsets near the ~2 km horizon only bound the position
# A stock warning starts about 6.1 m per km/h of the enforced speed before its camera
# (labelled drives: 30 -> 195 m, 50 -> 296-318 m, 60 -> 361-366 m, 100 -> ~615 m).
WARNING_LEAD_M_PER_KPH = 6.1
WARNING_LEAD_TOLERANCE = 0.2
WARNING_LEAD_MIN_TOLERANCE_M = 60.0
MATCH_BEHIND_M = 50.0
# A mobile zone suppresses deceleration, so it must be the zone that started this warning:
# its target sits one lead ahead of the warning start (44/44 zone warnings in 37 drives
# within -12..+17 m, 2026-09-30). A kind 3 preview elsewhere never turns a warning into a zone.
ZONE_LEAD_TOLERANCE_M = 60.0
# [experimental] Some cars/roads start the warning earlier: 2026-10-05 field logs (Gangwon, 60 km/h)
# had a box, a fixed camera and a mobile zone all at 7.6-7.7 m/km/h. With early_lead the far end of
# every lead window grows to EARLY_LEAD_FACTOR x lead; the near end is unchanged.
EARLY_LEAD_FACTOR = 1.35
RECENT_PREVIEW_M = 2600.0
PREVIEW_MERGE_M = 20.0  # the stock navigation repeats each preview (~3x); same as carstate event merging
PREVIEW_KEEP_M = 3000.0


def decode_camera_profile(value, offset):
  """Return (kind, speed_kph, flagged) for a type-16 camera preview, else None."""
  if value in (0, 0xFFFFFFFF) or not 0 < offset < 8191:
    return None
  low = value & 0x1FF
  kind = low & 0xF
  speed_code = low >> 4
  if kind not in CAMERA_KINDS or not 1 < speed_code <= 31:
    return None
  return kind, (speed_code - 1) * 5, (value >> 9) != 0


class SpeedcamPolicy:
  def __init__(self):
    self.previews = []
    self.early_lead = False
    self.reset_warning()

  def reset_warning(self):
    self.warning_active = False
    self.warning_speed = 0
    self.warning_start_distance = None
    self.warning_class = None
    self.skipped = False

  def clear(self):
    self.previews = []
    self.reset_warning()

  def add_preview(self, value, offset, total_distance):
    decoded = decode_camera_profile(value, offset)
    if decoded is None:
      return
    kind, speed, flagged = decoded
    target = total_distance + offset
    saturated = offset >= PREVIEW_SATURATED_OFFSET
    self.previews = [p for p in self.previews if total_distance - p["received"] <= PREVIEW_KEEP_M]
    for p in self.previews:
      if (p["kind"], p["speed"], p["flagged"]) == (kind, speed, flagged) and abs(p["target"] - target) < PREVIEW_MERGE_M:
        # A repeat refreshes the entry; a precise position wins over a horizon-bound one.
        p["received"] = total_distance
        if not saturated or p["saturated"]:
          p["target"] = target
        p["saturated"] = p["saturated"] and saturated
        return
    self.previews.append({
      "kind": kind, "speed": speed, "flagged": flagged, "received": total_distance,
      "target": target, "saturated": saturated,
    })

  def classify(self, speed, total_distance, starting=True, start_distance=None):
    """Classify the warning at total_distance from same-speed previews (None when none match).

    When a warning starts its camera sits one warning lead ahead. A saturated preview
    (entered at the ~2 km horizon) matches only when its target is at that lead;
    a precise preview may also be nearer, e.g. a warning that began before logging.
    Upgrades during a warning use precise previews only. A mobile zone (unflagged kind 3)
    counts only when its target is one lead ahead of the warning start, whenever it arrived.
    """
    lead = speed * WARNING_LEAD_M_PER_KPH
    tolerance = max(WARNING_LEAD_MIN_TOLERANCE_M, lead * WARNING_LEAD_TOLERANCE)
    if start_distance is None:
      start_distance = total_distance
    far = lead + tolerance
    zone_far = lead + ZONE_LEAD_TOLERANCE_M
    if self.early_lead:
      far = max(far, lead * EARLY_LEAD_FACTOR)
      zone_far = max(zone_far, lead * EARLY_LEAD_FACTOR)
    candidates = []
    for p in self.previews:
      if p["speed"] != speed or total_distance - p["received"] > RECENT_PREVIEW_M:
        continue
      if (p["kind"] == KIND_MOBILE_ZONE and not p["flagged"] and
          not lead - ZONE_LEAD_TOLERANCE_M <= p["target"] - start_distance <= zone_far):
        continue
      ahead = p["target"] - total_distance
      if p["saturated"]:
        matched = starting and lead - tolerance <= ahead <= far
      else:
        matched = -MATCH_BEHIND_M <= ahead <= far
      if matched:
        candidates.append(p)
    if not candidates:
      return None
    if any(p["flagged"] or p["kind"] in (KIND_FIXED, KIND_SIGNAL) for p in candidates):
      return CLASS_HARD
    if any(p["kind"] == KIND_BOX for p in candidates):
      return CLASS_BOX
    return CLASS_MOBILE_ZONE

  def update(self, warning_active, warning_speed, total_distance, accel_rising,
             mobile_zone_decel, skip_box, skip_mobile_zone, skip_unknown=False):
    """Return (suppress_warning, consume_accel_button)."""
    if not warning_active or warning_speed <= 0:
      self.reset_warning()
      return False, False

    starting = not self.warning_active or warning_speed != self.warning_speed
    if starting:
      self.reset_warning()
      self.warning_active = True
      self.warning_speed = warning_speed
      self.warning_start_distance = total_distance

    current = self.classify(warning_speed, total_distance, starting=starting, start_distance=self.warning_start_distance)
    # A warning never downgrades: a fixed camera inside a mobile zone keeps it hard.
    if current is not None and (self.warning_class is None or current > self.warning_class):
      self.warning_class = current
    warning_class = CLASS_HARD if self.warning_class is None else self.warning_class

    if warning_class == CLASS_MOBILE_ZONE and not mobile_zone_decel:
      return True, False

    skippable = ((warning_class == CLASS_BOX and skip_box) or
                 (warning_class == CLASS_MOBILE_ZONE and skip_mobile_zone) or
                 (self.warning_class is None and skip_unknown and warning_speed > PROTECTED_ZONE_KPH))
    consume = False
    if skippable and accel_rising and not self.skipped:
      self.skipped = True
      consume = True
    return self.skipped and skippable, consume
