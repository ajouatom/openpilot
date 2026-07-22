from pathlib import Path

from opendbc import DBC_PATH
from opendbc.dbc.generator import generator
from openpilot.tools.jotpluggler.dbc_generation import get_jotpluggler_generated_dbcs


def test_generated_dbcs_are_available():
  generated_dbcs = get_jotpluggler_generated_dbcs()

  assert generated_dbcs
  assert all(name.endswith("_generated") for name in generated_dbcs)
  assert all(content.strip() for content in generated_dbcs.values())
  assert "hyundai_canfd_generated" in generated_dbcs

  if not hasattr(generator, "generate_all"):
    on_disk_dbcs = {path.stem for path in Path(DBC_PATH).glob("*_generated.dbc")}
    assert generated_dbcs.keys() == on_disk_dbcs
