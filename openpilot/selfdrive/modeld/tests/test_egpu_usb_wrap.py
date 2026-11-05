from openpilot.selfdrive.modeld.egpu_usb_wrap import configure_usb_wrap
from openpilot.selfdrive.modeld.egpu_usb_wait import RUNTIME_SHA, MD_V3_RUNTIME_SHA


def test_other_runtime_pairs_are_not_patched():
  for manifest in ({}, {'runtime': None}, {'runtime': []}, {'runtime': {}},
                   {'runtime': {'sha256': RUNTIME_SHA}}, {'runtime': {'sha256': MD_V3_RUNTIME_SHA}},
                   {'runtime': {'sha256': 'unknown'}}):
    assert configure_usb_wrap(manifest) is False
