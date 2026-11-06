import json
from pathlib import Path

import numpy as np
import pytest

from openpilot.cereal import log
from openpilot.selfdrive.modeld.constants import Plan
from openpilot.selfdrive.modeld.fill_model_msg import PublishState, fill_model_msg
from openpilot.selfdrive.modeld.parse_model_outputs import Parser


@pytest.mark.parametrize('name', ['n16', 'md-v3', 'light-speed'])
def test_publish_real_schema_orientation_uncertainty_without_changing_predictions(name):
  spec = json.loads((Path(__file__).parents[1] / 'model_catalog' / f'{name}.json').read_text())
  raw = {key: np.zeros((1, stop-start), np.float32) for key, (start, stop) in spec['output_slices'].items()}
  outputs = Parser().parse_outputs(raw)
  outputs['plan'][0, :, Plan.ORIENTATION_RATE] = np.arange(99, dtype=np.float32).reshape(33, 3) / 100
  original = outputs['plan'].copy()
  action = log.ModelDataV2.Action.new_message(desiredAcceleration=0.3, desiredCurvature=0.002)
  messages = []
  for scale in (0., 1.):
    expected = scale * np.arange(1, 100, dtype=np.float32).reshape(33, 3) / 1000
    outputs['plan_stds'][0, :, Plan.ORIENTATION_RATE] = expected
    message = log.Event.new_message()
    message.init('modelV2')
    fill_model_msg(message, outputs, action, PublishState(), 10, 10, 10, 0., 123, 0.04, True)
    with log.Event.from_bytes(message.to_bytes()) as decoded:
      rate = decoded.modelV2.orientationRate
      np.testing.assert_allclose([rate.xStd, rate.yStd, rate.zStd], expected.T)
      np.testing.assert_allclose([rate.x, rate.y, rate.z], original[0, :, Plan.ORIENTATION_RATE].T)
      data = decoded.to_dict()
    for key in ('xStd', 'yStd', 'zStd'):
      data['modelV2']['orientationRate'].pop(key)
    messages.append(data)
  assert messages[0] == messages[1]
  np.testing.assert_array_equal(outputs['plan'], original)
