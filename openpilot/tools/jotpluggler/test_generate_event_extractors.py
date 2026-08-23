from pathlib import Path

import capnp

from openpilot.tools.jotpluggler.generate_event_extractors import Generator, MAX_EXPANDED_SCALAR_LIST_SIZE


REPO_ROOT = Path(__file__).parents[3]


def test_model_trajectory_lists_are_expanded():
  capnp.remove_import_hook()
  log = capnp.load(str(REPO_ROOT / "openpilot/cereal/log.capnp"),
                   imports=[str(REPO_ROOT / "opendbc_repo/opendbc/car")])
  generated = Generator(log.Event.schema).generate()

  position_x = generated.index('= "/modelV2/position/x";')
  position_x_extractor = generated[position_x:position_x + 200]

  assert MAX_EXPANDED_SCALAR_LIST_SIZE == 33
  assert f".size() <= {MAX_EXPANDED_SCALAR_LIST_SIZE}" in position_x_extractor
