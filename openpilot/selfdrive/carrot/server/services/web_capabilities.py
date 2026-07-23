from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, List, Mapping


@dataclass(frozen=True)
class WebCapability:
  id: str
  setting_key: str
  label_key: str
  locked_label_key: str


# Capabilities are deliberately separate from individual features. A web
# setting can opt into one by declaring `requires_capability`, while non-setting
# surfaces can query the same capability id directly.
WEB_CAPABILITIES = (
  WebCapability(
    id="web_lab",
    setting_key="web_lab_enabled",
    label_key="web_lab",
    locked_label_key="web_lab_locked",
  ),
)

_CAPABILITY_BY_ID = {capability.id: capability for capability in WEB_CAPABILITIES}


def is_known_web_capability(capability_id: str) -> bool:
  return str(capability_id or "") in _CAPABILITY_BY_ID


def web_capability_client_spec() -> List[Dict[str, str]]:
  return [
    {
      "id": capability.id,
      "settingKey": capability.setting_key,
      "labelKey": capability.label_key,
      "lockedLabelKey": capability.locked_label_key,
    }
    for capability in WEB_CAPABILITIES
  ]


def resolve_web_capabilities(settings: Mapping[str, Any]) -> Dict[str, bool]:
  return {
    capability.id: bool(settings.get(capability.setting_key, False))
    for capability in WEB_CAPABILITIES
  }


def set_web_capability_enabled(capability_id: str, enabled: bool) -> Dict[str, Any]:
  capability = _CAPABILITY_BY_ID.get(str(capability_id or ""))
  if capability is None:
    raise ValueError(f"unknown web capability: {capability_id}")

  # Local import avoids coupling the declarative capability catalog to the
  # persistence implementation during module initialization.
  from . import web_settings

  current = web_settings.read_web_settings()
  was_enabled = bool(current.get(capability.setting_key, False))
  updates: Dict[str, Any] = {capability.setting_key: bool(enabled)}

  # Enabling is an unlock, never an implicit feature activation. Reset gated
  # values on the first transition and whenever the capability is disabled.
  if not enabled or not was_enabled:
    updates.update(web_settings.web_setting_defaults_for_capability(capability.id))

  settings = web_settings.update_web_settings(updates)
  return {
    "id": capability.id,
    "enabled": bool(settings.get(capability.setting_key, False)),
    "settings": settings,
  }
