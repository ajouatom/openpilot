"""Strict result decoding and a CAN-free CCNC verification preview."""
TYPE_CLASSES = ('dashed', 'solid')
COLOR_CLASSES = ('white', 'yellow')
MAX_AGE_NS = 500_000_000
MIN_CONFIDENCE = 0.85


def unknown(reason='unconfirmed'):
  state = 'not_detected' if reason == 'not_detected' else 'unknown'
  return {'type': state, 'color': state, 'typeConfidence': 0.0, 'colorConfidence': 0.0, 'reason': reason}


def classify(type_scores, color_scores, threshold=MIN_CONFIDENCE):
  """The model exports probabilities; type and color can abstain independently."""
  import numpy as np
  result = {}
  for name, values, classes in [('type', type_scores, TYPE_CLASSES), ('color', color_scores, COLOR_CLASSES)]:
    p = np.asarray(values)
    if p.shape != (len(classes),) or not np.isfinite(p).all() or (p < 0).any() or (p > 1).any() or not np.isclose(p.sum(), 1, atol=1e-4):
      raise ValueError('invalid lane probabilities')
    index = int(p.argmax())
    confidence = float(p[index])
    result[name+'Candidate'] = classes[index]
    result[name] = classes[index] if confidence >= threshold else 'unknown'
    result[name+'Confidence'] = confidence
  result['reason'] = 'classified' if 'unknown' not in (result['type'], result['color']) else 'low_confidence'
  return result


