"""Stateful semantic display preview. No packer, CAN publisher or vehicle caller.

1/2/4 have DBC labels, not a Sonata hardware validation certificate. Code 8
is a dashed-display hypothesis; 10 and 14 stay unused/unverified. The meanings
of recognised yellow paint and an existing orange warning remain distinct.
"""
import math

from openpilot.selfdrive.carrot.lane_type.result import MAX_AGE_NS, MIN_CONFIDENCE

HIDDEN = 1
WHITE_SOLID = 2
ORANGE_SOLID = 4
WHITE_DASHED = 8
REPEAT_WINDOW_NS = 400_000_000


def fresh(stamp, now_ns):
  return isinstance(stamp, int) and not isinstance(stamp, bool) and 0 <= now_ns-stamp <= MAX_AGE_NS


class DisplayPolicy:
  """Two fresh confirmations per head; independent absolute 0.5s expiry.

  A default white solid style requires confirmed white and solid heads. Neither
  head can manufacture or extend the other. A known dashed yellow boundary has
  an orange-solid rendering fallback, explicitly logged as such.
  """
  def __init__(self):
    self.sides = {}

  def reset(self):
    self.sides.clear()

  def _confirm_side(self, side, result, now_ns):
    """The display filter: two fresh identical results within REPEAT_WINDOW_NS confirm a head, a
    confirmed head is refreshed by the same value and expires MAX_AGE_NS after its last evidence."""
    if not isinstance(result, dict):
      result = {}
    if result.get('trackingReset') or result.get('reason') in (
        'lane_change', 'tracking_lost', 'tracking_lost_or_stale', 'invalid_lane_geometry', 'not_detected'):
      self.sides.pop(side, None)
    state = self.sides.setdefault(side, {'heads': {}, 'pending': {}, 'last_now': now_ns})
    if now_ns < state['last_now']:
      state['heads'].clear()
      state['pending'].clear()
    state['last_now'] = now_ns
    for head, classes in [('type', ('dashed', 'solid')), ('color', ('white', 'yellow'))]:
      old = state['heads'].get(head)
      if old and not fresh(old['stamp'], now_ns):
        state['heads'].pop(head, None)
        old = None
      value = result.get(head)
      confidence = result.get(head+'Confidence', 0)
      stamp = result.get(head+'MonoTimeNanos')
      eligible = (value in classes and isinstance(confidence, (int, float)) and not isinstance(confidence, bool) and
                  math.isfinite(confidence) and MIN_CONFIDENCE <= confidence <= 1 and fresh(stamp, now_ns))
      if head == 'type' and value == 'dashed':
        eligible &= (result.get('dashedPatternObserved') is True and
                     result.get('dashedPatternMonoTimeNanos') == stamp)
      if eligible and (old is None or stamp > old['stamp']):
        if old and old['value'] == value:
          state['heads'][head] = {'value': value, 'stamp': stamp, 'confidence': confidence}
          state['pending'].pop(head, None)
        else:
          pending = state['pending'].get(head)
          if pending and pending['value'] == value and pending['stamp'] < stamp <= pending['stamp']+REPEAT_WINDOW_NS:
            state['heads'][head] = {'value': value, 'stamp': stamp, 'confidence': confidence}
            state['pending'].pop(head, None)
          elif pending is None or stamp != pending['stamp']:
            state['pending'][head] = {'value': value, 'stamp': stamp}
      pending = state['pending'].get(head)
      if pending and now_ns-pending['stamp'] > REPEAT_WINDOW_NS:
        state['pending'].pop(head, None)
    return state

  def confirm(self, packet, now_ns):
    """Run only the display filter for a recogniser packet; returns the confirmed heads per side as
    {'left': {'type': {'value', 'stamp', 'confidence'}, 'color': {...}}, 'right': {...}}."""
    valid_packet = isinstance(packet, dict) and fresh(packet.get('cameraMonoTimeNanos'), now_ns)
    out = {}
    for side in ('left', 'right'):
      state = self._confirm_side(side, packet.get(side, {}) if valid_packet else {}, now_ns)
      out[side] = {k: dict(v) for k, v in state['heads'].items()}
    return out

  def update(self, baseline, packet, *, enabled, model_lanes, now_ns, warnings=None):
    values = dict(baseline)
    output = {'values': values, 'sides': {}, 'canTransmitEnabled': False, 'hardwareValidated': False, 'classificationValidated': False,
              'baselineAvailable': bool(baseline), 'policy': 'lane-marking-preview-v2'}
    if not enabled or model_lanes not in (1, 2):
      self.reset()
      return dict(output, reason='disabled_or_model_lanes_off')
    valid_packet = (isinstance(packet, dict) and packet.get('type') == 'ccncLaneType' and packet.get('version') == 1 and
                    fresh(packet.get('cameraMonoTimeNanos'), now_ns))
    for side in ('left', 'right'):
      field = 'LANELINE_'+side.upper()
      result = packet.get(side, {}) if valid_packet else {}
      state = self._confirm_side(side, result, now_ns)
      kind = state['heads'].get('type', {}).get('value', 'unknown')
      color = state['heads'].get('color', {}).get('value', 'unknown')
      semantic, code, reason = 'hidden', HIDDEN, 'unconfirmed_expired_or_insufficient_pattern'
      if kind in ('solid', 'dashed') and color in ('white', 'yellow'):
        if color == 'yellow':
          semantic, code, reason = 'orange_solid', ORANGE_SOLID, 'recognised_yellow_boundary'
        elif kind == 'dashed':
          semantic, code, reason = 'white_dashed', WHITE_DASHED, 'repeating_pattern_candidate_unvalidated'
        else:
          semantic, code, reason = 'white_solid', WHITE_SOLID, 'confirmed_solid_default_white'
      warning = (warnings or {}).get(side, {})
      active_warning = any(warning.get(k) for k in ('laneDeparture', 'blindSpot', 'laneLineWarning'))
      preserve = active_warning or warning.get('unknown') or baseline.get(field) in (1, 4, 14)
      if preserve:
        # Preserve the actual existing warning value and blink phase, including
        # hidden. Without a baseline, log the warning with no invented code.
        code, semantic, reason = baseline.get(field), 'existing_warning', 'existing_warning_priority'
      elif code is not None:
        values[field] = code
      output['sides'][side] = {'semantic': semantic, 'candidateCode': code, 'reason': reason,
        'classifiedType': kind, 'classifiedColor': color, 'defaultWhiteStyle': semantic == 'white_solid',
        'renderingTypeFallback': kind == 'dashed' and color == 'yellow',
        'warning': dict(warning), 'heads': {k: dict(v) for k, v in state['heads'].items()},
        'codeVerifiedOnSonata': False}
    return output
