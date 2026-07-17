from __future__ import annotations

import functools
import re

from .models import FieldSemantics, ServiceSemantics, StatisticsPolicy
from .overrides import CURATED_FIELD_UNITS
from .policies import (
  GROUPS,
  GROUP_LABEL_KEYS,
  STATISTICS_POLICIES,
  display_precision,
  group_for_domain,
  policy_key,
  recommended_view,
  sequence_like,
)
from .resources import field_overrides, field_resource, service_overrides, service_resource


CATALOG_VERSION = 2
SEMANTIC_VERSION = 1
STATISTICS_VERSION = 1


def registry_versions() -> dict[str, int]:
  return {
    "catalogVersion": CATALOG_VERSION,
    "semanticVersion": SEMANTIC_VERSION,
    "statisticsVersion": STATISTICS_VERSION,
  }


def registry_provenance() -> dict[str, object]:
  fields = field_resource()
  services = service_resource()
  return {
    "fieldResourceVersion": int(fields.get("version") or 0),
    "serviceResourceVersion": int(services.get("version") or 0),
    "schemaCommit": str(fields.get("schemaCommit") or services.get("schemaCommit") or ""),
    "schemaBranch": str(fields.get("schemaBranch") or services.get("schemaBranch") or ""),
    "fieldOverrideCount": len(field_overrides()),
    "serviceOverrideCount": len(service_overrides()),
  }


def group_definitions() -> list[dict[str, str]]:
  return [{"key": key, "labelKey": label_key} for key, label_key in GROUPS]


def statistics_policy_definitions() -> list[dict[str, object]]:
  return [policy.as_dict() for policy in STATISTICS_POLICIES.values()]


@functools.cache
def service_semantics(service: str) -> ServiceSemantics:
  override = service_overrides().get(service, {})
  domain = str(override.get("domain") or "")
  group = group_for_domain(domain, service)
  return ServiceSemantics(
    service=service,
    group=group,
    group_label_key=GROUP_LABEL_KEYS[group],
    domain=domain or "unclassified",
    purpose=str(override.get("purpose") or ""),
    recommended_view=str(override.get("recommendedView") or "raw/advanced"),
    privacy=str(override.get("privacy") or "unknown"),
    source="curated-service-audit" if override else "inferred-service-name",
  )


def _inferred_unit(path: str, capnp_type: str) -> tuple[str, str, str]:
  leaf = re.sub(r"\[\]", "", path.rsplit(".", 1)[-1]).lower()
  suffixes = (
    ("monotimenanos", "ns"), ("timestampnanos", "ns"), ("timenanos", "ns"),
    ("timestampmillis", "ms"), ("timemillis", "ms"), ("distancem", "m"),
    ("headingdeg", "deg"), ("bearingdeg", "deg"), ("speedkph", "km/h"),
    ("limitkph", "km/h"), ("timesec", "s"), ("durationsec", "s"),
    ("latencyms", "ms"), ("lagms", "ms"),
  )
  for suffix, unit in suffixes:
    if leaf.endswith(suffix):
      return unit, "inferred-name", f"field suffix {suffix}"
  normalized = capnp_type.lower()
  if normalized == "bool":
    return "boolean", "not-applicable", "Cap'n Proto bool"
  if normalized == "enum":
    return "categorical", "not-applicable", "Cap'n Proto enum"
  if normalized in ("text", "data", "void", "marker") or normalized.startswith("list<"):
    return "", "not-applicable", f"Cap'n Proto {capnp_type}"
  if re.search(r"(id|index|idx|sequence|count|counter|version|status|flags?|type|code|mode|source)$", leaf):
    return "count/code", "not-applicable", "identifier/count/code field name"
  if re.search(r"(prob|probability|confidence|ratio|factor|progress|percent|pct)$", leaf):
    return "ratio", "inferred-dimensionless", "dimensionless field name"
  if normalized.startswith(("int", "uint", "float")):
    return "", "unresolved-numeric", "unit not verified"
  return "", "not-applicable", "non-numeric field"


def _privacy(service: str, path: str) -> str:
  value = f"{service}.{path}".lower()
  if "driver" in value or "face" in value:
    return "biometric-camera"
  if any(token in value for token in ("latitude", "longitude", "navroute", "polyline", "roadname", "navipaths")):
    return "route-location"
  if service in ("can", "sendcan", "pandaStates"):
    return "vehicle-bus"
  if service in ("rawAudioData", "audioFeedback", "soundPressure"):
    return "audio"
  if service in ("logMessage", "errorLogMessage", "androidLog", "procLog"):
    return "potential-sensitive-text"
  leaf = path.rsplit(".", 1)[-1].lower()
  if leaf in ("dat", "data", "payload", "raw") or leaf.endswith("data"):
    return "potential-sensitive-binary"
  service_meta = service_semantics(service)
  service_privacy = service_meta.privacy.lower().replace("_", "-")
  if service_privacy in ("location", "route-location"):
    return "route-location"
  if service_privacy in ("camera-driver", "biometric", "biometric-camera"):
    return "biometric-camera"
  if service_privacy.startswith("audio"):
    return "audio"
  if service_privacy == "vehicle-identifier":
    return "vehicle-identifier"
  if service_privacy == "potential-sensitive-text":
    return "potential-sensitive-text"
  if service_privacy in ("vehicle-bus", "camera", "device-process", "device-network", "user-input", "driving", "low"):
    return service_privacy
  if service_meta.group == "location":
    return "route-location"
  return "driving"


@functools.cache
def field_semantics(service: str, path: str, capnp_type: str) -> FieldSemantics:
  override = field_overrides().get(path, {})
  normalized_type = capnp_type.lower()
  numeric_type = normalized_type.startswith(("int", "uint", "float"))
  if path in CURATED_FIELD_UNITS and numeric_type:
    unit = CURATED_FIELD_UNITS[path]
    unit_status = "verified-source"
    unit_basis = "code-reviewed schema comment or openpilot convention"
    source = "curated-code-review"
  elif override and numeric_type:
    unit = str(override.get("unit") or "")
    unit_status = str(override.get("unitStatus") or "unresolved-numeric")
    unit_basis = str(override.get("unitBasis") or "curated semantic audit")
    source = "curated-field-audit"
  else:
    unit, unit_status, unit_basis = _inferred_unit(path, capnp_type)
    source = "runtime-schema-inference"
  policy = policy_key(capnp_type, path, unit_status)
  service_meta = service_semantics(service)
  return FieldSemantics(
    service=service,
    path=path,
    capnp_type=capnp_type,
    group=service_meta.group,
    unit=unit,
    unit_status=unit_status,
    unit_basis=unit_basis,
    privacy=_privacy(service, path),
    recommended_view=recommended_view(policy, unit_status),
    statistics_policy=policy,
    sequence_like=sequence_like(path),
    display_precision=display_precision(unit, policy),
    label_key="",
    source=source,
    semantic_version=SEMANTIC_VERSION,
    statistics_version=STATISTICS_VERSION,
  )


def statistics_policy(key: str) -> StatisticsPolicy:
  return STATISTICS_POLICIES.get(key, STATISTICS_POLICIES["raw-metadata"])


def validate_registry() -> list[str]:
  errors = []
  known_groups = set(GROUP_LABEL_KEYS)
  for service in service_overrides():
    if service_semantics(service).group not in known_groups:
      errors.append(f"unknown group for {service}")
  valid_statuses = {
    "verified-source", "inferred-name", "inferred-dimensionless", "not-applicable", "unresolved-numeric",
  }
  for path, override in field_overrides().items():
    if not path or not isinstance(override, dict):
      errors.append(f"bad field override {path!r}")
      continue
    if str(override.get("unitStatus") or "") not in valid_statuses:
      errors.append(f"bad unit status for {path}")
  return errors
