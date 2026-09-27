"""Stock-navigation speed-camera kind policy for Hyundai CAN-FD 0x4BE/0x4A3.

0x4BE PROLONG_VALUE (profile type 16) carries a camera kind in bits 0..3, a
speed code in bits 4..8 and extra flags above bit 9. Kinds were matched against
the stock navigation screen and labelled drives (2026-09-27):

  kind 0  fixed speed camera (also section start/end cameras)
  kind 1  signal-and-speed camera
  kind 2  unconfirmed spot camera (likely a mobile-camera box)
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

PREVIEW_SATURATED_OFFSET = 1985  # offsets near the ~2 km horizon only bound the position
# A stock warning starts about 6.1 m per km/h of the enforced speed before its camera
# (labelled drives: 30 -> 195 m, 50 -> 296-318 m, 60 -> 361-366 m, 100 -> ~615 m).
WARNING_LEAD_M_PER_KPH = 6.1
WARNING_LEAD_TOLERANCE = 0.2
WARNING_LEAD_MIN_TOLERANCE_M = 60.0
MATCH_BEHIND_M = 50.0
RECENT_PREVIEW_M = 2600.0
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
    self.reset_warning()

  def reset_warning(self):
    self.warning_active = False
    self.warning_speed = 0
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
    self.previews.append({
      "kind": kind, "speed": speed, "flagged": flagged, "received": total_distance,
      "target": total_distance + offset, "saturated": offset >= PREVIEW_SATURATED_OFFSET,
    })
    self.previews = [p for p in self.previews if total_distance - p["received"] <= PREVIEW_KEEP_M]

  def classify(self, speed, total_distance, starting=True):
    """Classify the warning at total_distance from same-speed previews (None when none match).

    When a warning starts its camera sits one warning lead ahead. A saturated preview
    (entered at the ~2 km horizon) matches only when its target is at that lead;
    a precise preview may also be nearer, e.g. a warning that began before logging.
    Upgrades during a warning use precise previews only.
    """
    lead = speed * WARNING_LEAD_M_PER_KPH
    tolerance = max(WARNING_LEAD_MIN_TOLERANCE_M, lead * WARNING_LEAD_TOLERANCE)
    candidates = []
    for p in self.previews:
      if p["speed"] != speed or total_distance - p["received"] > RECENT_PREVIEW_M:
        continue
      ahead = p["target"] - total_distance
      if p["saturated"]:
        matched = starting and abs(ahead - lead) <= tolerance
      else:
        matched = -MATCH_BEHIND_M <= ahead <= lead + tolerance
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
             mobile_zone_decel, skip_box, skip_mobile_zone):
    """Return (suppress_warning, consume_accel_button)."""
    if not warning_active or warning_speed <= 0:
      self.reset_warning()
      return False, False

    starting = not self.warning_active or warning_speed != self.warning_speed
    if starting:
      self.reset_warning()
      self.warning_active = True
      self.warning_speed = warning_speed

    current = self.classify(warning_speed, total_distance, starting=starting)
    # A warning never downgrades: a fixed camera inside a mobile zone keeps it hard.
    if current is not None and (self.warning_class is None or current > self.warning_class):
      self.warning_class = current
    warning_class = CLASS_HARD if self.warning_class is None else self.warning_class

    if warning_class == CLASS_MOBILE_ZONE and not mobile_zone_decel:
      return True, False

    skippable = ((warning_class == CLASS_BOX and skip_box) or
                 (warning_class == CLASS_MOBILE_ZONE and skip_mobile_zone))
    consume = False
    if skippable and accel_rising and not self.skipped:
      self.skipped = True
      consume = True
    return self.skipped and skippable, consume
