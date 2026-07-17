from __future__ import annotations

import functools
import json
from pathlib import Path
from typing import Any


_RESOURCE_DIR = Path(__file__).resolve().parent


@functools.lru_cache(maxsize=1)
def field_resource() -> dict[str, Any]:
  with (_RESOURCE_DIR / "field_units.v1.json").open(encoding="utf-8") as file:
    payload = json.load(file)
  return payload if isinstance(payload, dict) else {}


def field_overrides() -> dict[str, dict[str, Any]]:
  fields = field_resource().get("fields")
  return fields if isinstance(fields, dict) else {}


@functools.lru_cache(maxsize=1)
def service_resource() -> dict[str, Any]:
  with (_RESOURCE_DIR / "services.v1.json").open(encoding="utf-8") as file:
    payload = json.load(file)
  return payload if isinstance(payload, dict) else {}


def service_overrides() -> dict[str, dict[str, Any]]:
  services = service_resource().get("services")
  return services if isinstance(services, dict) else {}
