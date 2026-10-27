"""Negotiated Carrot v2 extension for the exact, graph-stateful N16 export.

This changes only the response prefix. The engine must still validate its full
output and commit all recurrent feedback before projecting a successful reply.
"""
import hashlib
import json

CAPABILITY = 'carrot_n16_driving_output_v1'
IPC_CAPABILITY = 'carrot_n16_driving_output_ipc_v1'
FLAG = 1 << 5
IPC_FLAG = 1 << 1  # bit zero of the local request remains RESET_QUEUES
COUNT = 2066
FULL_COUNT = 18580
OMITTED = frozenset(('hidden_state', 'map_features', 'navigation_features', 'pad'))
MODEL_SHA256 = '01effc679eb46d6036ce0302673a87fe2dcfcc95a679daeeb929679e53c3647a'
SPEC_SHA256 = '63b2097c114bade03c5ec7ded978d73032a93bb525c03f9b78eddf49809f8b71'


def canonical(value):
  return json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False)


def descriptor():
  return {'schema': 1, 'model_sha256': MODEL_SHA256, 'spec_sha256': SPEC_SHA256,
          'dtype': 'float32-le', 'start': 0, 'count': COUNT, 'full_count': FULL_COUNT}


def matches(value):
  # JSON equality alone accepts True == 1 and 2066.0 == 2066.
  try:
    return type(value) is dict and canonical(value) == canonical(descriptor())
  except (TypeError, ValueError):
    return False


def eligible(spec):
  return (spec is not None and spec.stateful and spec.sha256 == MODEL_SHA256 and
          hashlib.sha256(canonical(spec.to_dict()).encode()).hexdigest() == SPEC_SHA256)
