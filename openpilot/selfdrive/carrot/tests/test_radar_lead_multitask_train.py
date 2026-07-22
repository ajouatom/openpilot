from __future__ import annotations

import numpy as np

from openpilot.selfdrive.carrot.radar.tools.radar_lead_multitask_train import (
  TrainingData,
  distill_nonmanual_head,
  group_metrics,
  manual_context_indices,
)


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


def test_manual_context_indices_keeps_manual_groups_and_is_deterministic() -> None:
  data = TrainingData(
    features=np.zeros((12, 1), dtype=np.float32),
    labels=np.zeros((12, 3), dtype=np.float32),
    weights=np.ones((12, 3), dtype=np.float32),
    groups=np.repeat(np.arange(6, dtype=np.int32), 2),
    manual=np.zeros((12, 3), dtype=np.bool_),
  )
  data.manual[2, 1] = True
  data.manual[8, 1] = True
  indices = np.arange(12, dtype=np.int64)

  first = manual_context_indices(data, indices, 1, 1.0, 7)
  second = manual_context_indices(data, indices, 1, 1.0, 7)

  assert np.array_equal(first, second)
  selected_groups = set(data.groups[first])
  assert {1, 4}.issubset(selected_groups)
  assert len(selected_groups) == 4
  assert all(np.count_nonzero(data.groups[first] == group) == 2 for group in selected_groups)


def test_group_metrics_ranks_candidates_within_each_group() -> None:
  metrics = group_metrics(
    probabilities=np.asarray((0.9, 0.8, 0.7, 0.6, 0.95, 0.1), dtype=np.float32),
    labels=np.asarray((1, 0, 1, 0, 0, 0), dtype=np.float32),
    weights=np.asarray((1, 1, 1, 1, 1, 0), dtype=np.float32),
    groups=np.asarray((7, 7, 9, 9, 11, 11), dtype=np.int32),
    threshold=0.75,
    max_outputs=1,
  )

  assert metrics["tp"] == 1
  assert metrics["fp"] == 1
  assert metrics["fn"] == 1
  assert metrics["target_groups"] == 2
  assert metrics["none_groups"] == 1
  assert metrics["none_accuracy"] == 0.0
