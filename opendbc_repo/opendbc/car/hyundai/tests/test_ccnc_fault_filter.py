import pytest

from opendbc.car.hyundai.hyundaicanfd import _hide_lca_service_warning


@pytest.mark.parametrize(
  ("fault_lca", "fault_das", "expected_lca", "expected_das"),
  [
    (1, 1, 0, 0),
    (1, 0, 0, 0),
    (1, 2, 0, 2),
    (0, 1, 0, 1),
    (2, 1, 2, 1),
    (3, 1, 3, 1),
  ],
)
def test_hide_lca_service_warning(fault_lca, fault_das, expected_lca, expected_das):
  values = {"FAULT_LCA": fault_lca, "FAULT_DAS": fault_das}

  _hide_lca_service_warning(values)

  assert values["FAULT_LCA"] == expected_lca
  assert values["FAULT_DAS"] == expected_das
