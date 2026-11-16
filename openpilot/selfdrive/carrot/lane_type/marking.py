"""Light helpers shared by the lane marking process and card (no OpenCV/numpy import).

CcncLaneTypeDetection turns the display-only recogniser on: 0 off (existing CCNC lane display unchanged), 1 StripNet
(frozen strip model). The value 2 (StripNet in the former 3-way selector, whose 1 was the Scanner) is still read as
StripNet, so a stored 1 or 2 both run StripNet: the profile Scanner is no longer run on the car (2026-10-21); MODES keeps its
name for offline replays (tools/lane_profile/replay_marking.py builds marking_daemon.Recogniser(1) itself).
The StripNet model is assets/lane_strip_v4.* (strip_model.MODEL_NAME); the model choice setting was removed (DaBin 10-26).
The recogniser publishes the display-filtered (two identical results within 0.4 s, 0.5 s expiry) left/right type and
colour on customReservedRawData2; card keeps only the latest packet and the CCNC sender applies it without waiting.
"""
import json

PARAM = 'CcncLaneTypeDetection'
MODES = {1: 'scanner', 2: 'stripnet'}  # Recogniser engines; the daemon only runs STRIPNET
STRIPNET = 2
PACKET_TYPE, PACKET_VERSION = 'ccncLaneMarking', 1
MAX_AGE_NS = 500_000_000  # the display policy's expiry (result.MAX_AGE_NS)
TYPES, COLORS = ('dashed', 'solid'), ('white', 'yellow')


def mode_of(params):
  """STRIPNET when CcncLaneTypeDetection is on (1, or the former selector's 2), else 0 (off)."""
  try:
    mode = params.get_int(PARAM)
  except (TypeError, ValueError):
    return 0
  return STRIPNET if mode in (1, 2) else 0


def parse(payload, received_ns):
  """Packet -> {'mode', 'frameId', 'left': {'type': (value, expires_ns)|None, 'color': ...}, 'right': ...}.

  Expiry is anchored on card's receive time minus the head's age at publish, so it never depends on the
  recogniser's clock or on how late card reads the packet."""
  data = json.loads(payload)
  if not isinstance(data, dict) or data.get('type') != PACKET_TYPE or data.get('version') != PACKET_VERSION:
    raise ValueError('expected a ccncLaneMarking v1 packet')
  out = {'mode': data.get('mode'), 'frameId': data.get('frameId')}
  for side in ('left', 'right'):
    entry = data.get(side) or {}
    heads = {}
    for head, classes in (('type', TYPES), ('color', COLORS)):
      value, age = entry.get(head), entry.get(head + 'AgeNs')
      if value in classes and isinstance(age, int) and not isinstance(age, bool) and 0 <= age <= MAX_AGE_NS:
        heads[head] = (value, received_ns - age + MAX_AGE_NS)
      else:
        heads[head] = None
    reset_ns = entry.get('displayResetNs')
    camera_ns = data.get('cameraMonoTimeNanos')
    if (isinstance(reset_ns, int) and not isinstance(reset_ns, bool) and isinstance(camera_ns, int)
        and not isinstance(camera_ns, bool) and 0 < reset_ns <= camera_ns):
      heads['displayResetNs'] = reset_ns
    out[side] = heads
  return out


def current(marking, side, now_ns):
  """(type, colour) still within the display expiry, each None when unknown or expired."""
  if not marking:
    return None, None
  heads = marking.get(side) or {}
  return tuple(h[0] if h is not None and now_ns <= h[1] else None for h in (heads.get('type'), heads.get('color')))
