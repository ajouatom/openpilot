from __future__ import annotations

import numpy as np

from openpilot.selfdrive.carrot.radar_lead_multitask_train import TrainingData, distill_nonmanual_head


def test_distill_nonmanual_head_preserves_manual_corrections() -> None:
  data = TrainingData(
    features=np.zeros((3, 1), dtype=np.float32),
    labels=np.asarray(((0.0, 1.0), (1.0, 0.0), (0.0, 1.0)), dtype=np.float32),
    weights=np.asarray(((1.0, 0.08), (1.0, 10.0), (1.0, 0.0)), dtype=np.float32),
    groups=np.arange(3, dtype=np.int32),
    manual=np.asarray(((False, False), (False, True), (False, False)), dtype=np.bool_),
  )

  distilled = distill_nonmanual_head(data, 1, np.asarray((0.25, 0.5, 0.75), dtype=np.float32), 0.3)

  np.testing.assert_allclose(distilled.labels[:, 1], (0.25, 0.0, 1.0))
  np.testing.assert_allclose(distilled.weights[:, 1], (0.3, 10.0, 0.0))
  np.testing.assert_array_equal(data.labels[:, 1], (1.0, 0.0, 1.0))
