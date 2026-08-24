import pytest

from opendbc.car.hyundai.hyundaicanfd import _hide_replaced_adas_service_warning


@pytest.mark.parametrize(
  ("fault_lca", "fault_hda", "fault_das", "expected_lca", "expected_hda", "expected_das"),
  [
    (1, 0, 1, 0, 0, 0),
    (0, 1, 1, 0, 0, 0),
    (1, 1, 1, 0, 0, 0),
    (1, 0, 0, 0, 0, 0),
    (0, 1, 2, 0, 0, 2),
    (0, 0, 1, 0, 0, 1),
    (2, 0, 1, 2, 0, 1),
    (0, 2, 1, 0, 2, 1),
    (3, 3, 1, 3, 3, 1),
  ],
)
def test_hide_replaced_adas_service_warning(fault_lca, fault_hda, fault_das,
                                            expected_lca, expected_hda, expected_das):
  values = {"FAULT_LCA": fault_lca, "FAULT_HDA": fault_hda, "FAULT_DAS": fault_das}

  _hide_replaced_adas_service_warning(values)

  assert values["FAULT_LCA"] == expected_lca
  assert values["FAULT_HDA"] == expected_hda
  assert values["FAULT_DAS"] == expected_das
