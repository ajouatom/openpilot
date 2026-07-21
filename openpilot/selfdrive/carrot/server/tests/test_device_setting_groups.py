from openpilot.selfdrive.carrot.server.services import device_info


def test_defaults_are_derived_from_the_groups_not_restated():
  from_groups = {name: default for _group, entries in device_info.DEVICE_SETTING_GROUPS for name, default in entries}
  assert device_info.DEVICE_SETTING_DEFAULTS == from_groups


def test_no_parameter_appears_in_two_groups():
  names = [name for _group, entries in device_info.DEVICE_SETTING_GROUPS for name, _default in entries]
  assert len(names) == len(set(names))


def test_every_default_belongs_to_exactly_one_group():
  grouped = device_info.get_device_setting_group_names()
  flattened = [name for names in grouped.values() for name in names]
  assert sorted(flattened) == sorted(device_info.DEVICE_SETTING_DEFAULTS)


def test_the_group_mapping_keeps_declaration_order():
  grouped = device_info.get_device_setting_group_names()
  assert list(grouped) == [group for group, _entries in device_info.DEVICE_SETTING_GROUPS]
  assert grouped["Device"][0] == "DeviceType"
  assert grouped["Toggles"][0] == "OpenpilotEnabledToggle"


def test_the_groups_the_web_renders_are_all_present():
  grouped = device_info.get_device_setting_group_names()
  # Network is read-only and has no parameters, so it is intentionally absent.
  assert set(grouped) == {"Device", "Software", "Toggles", "Developer"}


def test_parameters_the_ui_depends_on_are_not_dropped():
  grouped = device_info.get_device_setting_group_names()
  # Each of these drives a specific control; losing one silently blanks it.
  assert "LongitudinalPersonality" in grouped["Toggles"]
  assert "ExperimentalModeConfirmed" in grouped["Toggles"]
  assert "SoftwareMenu" in grouped["Device"]
  assert {"GithubUsername", "GithubSshKeys"} <= set(grouped["Developer"])


def test_the_mapping_is_a_copy_callers_cannot_corrupt():
  first = device_info.get_device_setting_group_names()
  first["Device"].append("Injected")
  assert "Injected" not in device_info.get_device_setting_group_names()["Device"]
