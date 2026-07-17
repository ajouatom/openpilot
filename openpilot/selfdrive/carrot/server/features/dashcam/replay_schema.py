from __future__ import annotations

import functools
import os
from typing import Any

from openpilot.cereal import log
from openpilot.cereal.services import SERVICE_LIST

from .replay_semantics import (
  field_semantics,
  group_definitions,
  registry_provenance,
  registry_versions,
  service_semantics,
  statistics_policy_definitions,
)


def _type_name(field: Any) -> tuple[str, str]:
  proto = field.proto
  if proto.which() == "group":
    return "group", ""
  type_proto = proto.slot.type
  kind = type_proto.which()
  if kind != "list":
    return kind, ""
  element_kind = type_proto.list.elementType.which()
  return "list", element_kind


def _enum_values(field: Any) -> list[dict[str, Any]]:
  try:
    return [
      {"name": str(name), "value": int(value)}
      for name, value in field.schema.enumerants.items()
    ]
  except Exception:
    return []


def _field_row(service: str, path: str, capnp_type: str, enum_values: list[dict[str, Any]] | None = None) -> dict[str, Any]:
  return {
    "path": path,
    "type": capnp_type,
    "enumValues": enum_values or [],
    **field_semantics(service, path, capnp_type).as_dict(),
  }


def _flatten_struct(service: str, schema: Any, prefix: str, active_schemas: set[int]) -> list[dict[str, Any]]:
  try:
    schema_id = int(schema.node.id)
  except Exception:
    schema_id = id(schema)
  if schema_id in active_schemas:
    return [_field_row(service, prefix, "recursive-struct")]
  active_schemas.add(schema_id)
  rows: list[dict[str, Any]] = []
  try:
    fields = sorted(schema.fields.items(), key=lambda item: int(item[1].proto.codeOrder))
    if not fields:
      return [_field_row(service, prefix, "marker")]
    for name, field in fields:
      path = f"{prefix}.{name}" if prefix else str(name)
      kind, element_kind = _type_name(field)
      if kind in ("struct", "group"):
        rows.extend(_flatten_struct(service, field.schema, path, active_schemas))
      elif kind == "list" and element_kind == "struct":
        rows.extend(_flatten_struct(service, field.schema.elementType, f"{path}[]", active_schemas))
      elif kind == "list":
        rows.append(_field_row(service, f"{path}[]", f"list<{element_kind}>"))
      elif kind == "enum":
        rows.append(_field_row(service, path, "enum", _enum_values(field)))
      else:
        rows.append(_field_row(service, path, kind))
    return rows
  finally:
    active_schemas.discard(schema_id)


@functools.lru_cache(maxsize=1)
def current_schema_catalog() -> tuple[dict[str, Any], ...]:
  event_schema = log.Event.schema
  union_names = set(event_schema.union_fields)
  entries: list[dict[str, Any]] = []
  for service, field in sorted(event_schema.fields.items(), key=lambda item: int(item[1].proto.codeOrder)):
    if service not in union_names:
      continue
    kind, element_kind = _type_name(field)
    if kind == "struct":
      fields = _flatten_struct(service, field.schema, service, set())
    elif kind == "list" and element_kind == "struct":
      fields = _flatten_struct(service, field.schema.elementType, f"{service}[]", set())
    elif kind == "list":
      fields = [_field_row(service, f"{service}[]", f"list<{element_kind}>")]
    elif kind == "enum":
      fields = [_field_row(service, service, "enum", _enum_values(field))]
    else:
      fields = [_field_row(service, service, kind)]
    config = SERVICE_LIST.get(service)
    semantics = service_semantics(service)
    entries.append({
      "service": service,
      **semantics.as_dict(),
      "registered": config is not None,
      "shouldLog": bool(config.should_log) if config is not None else None,
      "nominalHz": float(config.frequency) if config is not None else 0.0,
      "qlogDecimation": int(config.decimation) if config is not None and config.decimation is not None else None,
      "schemaMissing": False,
      "fields": fields,
    })
  existing = {entry["service"] for entry in entries}
  for service, config in SERVICE_LIST.items():
    if service in existing:
      continue
    semantics = service_semantics(service)
    entries.append({
      "service": service,
      **semantics.as_dict(),
      "registered": True,
      "shouldLog": bool(config.should_log),
      "nominalHz": float(config.frequency),
      "qlogDecimation": int(config.decimation) if config.decimation is not None else None,
      "schemaMissing": True,
      "fields": [],
    })
  return tuple(entries)


def schema_response(
  recorded_schema: dict[str, Any],
  observed_services: set[str],
  service_filter: str = "",
  include_fields: bool = False,
) -> dict[str, Any]:
  recorded_commit = str(recorded_schema.get("gitCommit") or "")
  current_commit = str(os.environ.get("GIT_COMMIT") or "")
  if recorded_commit and current_commit and recorded_commit == current_commit:
    schema_status = "exact-commit"
  elif recorded_commit:
    schema_status = "current-reader-compatible-unverified"
  else:
    schema_status = "recorded-commit-unknown"
  services = []
  for entry in current_schema_catalog():
    if service_filter and entry["service"] != service_filter:
      continue
    fields = entry["fields"] if include_fields or service_filter else []
    services.append({
      **{key: value for key, value in entry.items() if key != "fields"},
      "fieldCount": len(entry["fields"]),
      "fields": fields,
      "observed": entry["service"] in observed_services,
      "schemaStatus": schema_status,
    })
  return {
    **registry_versions(),
    "registry": registry_provenance(),
    "groups": group_definitions(),
    "statisticsPolicies": statistics_policy_definitions(),
    "recordedSchema": dict(recorded_schema),
    "currentSchemaCommit": current_commit,
    "serviceCount": len(services),
    "fieldCount": sum(int(entry["fieldCount"]) for entry in services),
    "services": services,
  }


def field_definition(service: str, path: str) -> dict[str, Any] | None:
  for entry in current_schema_catalog():
    if entry["service"] != service:
      continue
    for field in entry["fields"]:
      if field["path"] == path:
        return dict(field)
    return None
  return None


def service_fields(service: str) -> list[dict[str, Any]]:
  for entry in current_schema_catalog():
    if entry["service"] == service:
      return [dict(field) for field in entry["fields"]]
  return []


def service_definition(service: str) -> dict[str, Any] | None:
  for entry in current_schema_catalog():
    if entry["service"] == service:
      return {
        **{key: value for key, value in entry.items() if key != "fields"},
        "fieldCount": len(entry["fields"]),
        "fields": [dict(field) for field in entry["fields"]],
      }
  return None
