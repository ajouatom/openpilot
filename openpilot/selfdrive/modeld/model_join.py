"""Fresh vehicle-state gate shared by external inference backends."""


def may_join(now, messages, valid, received):
  required = ('carState', 'selfdriveState', 'carControl')
  if not all(valid.get(k, False) and 0 <= now - received.get(k, -1e6) < .25 for k in required):
    return False
  cs = messages['carState']
  return (cs.standstill and abs(cs.vEgo) < .01) or cs.steeringPressed
