from __future__ import annotations

import argparse
import csv
import json
from collections import OrderedDict
from pathlib import Path
from typing import Any


UNIT_STATUS_MAP = {
  "verified_explicit": "verified-source",
  "inferred_name_strong": "inferred-name",
  "inferred_dimensionless": "inferred-dimensionless",
  "not_applicable_code_or_count": "not-applicable",
}


def _rows(path: Path) -> list[dict[str, str]]:
  with path.open(encoding="utf-8-sig", newline="") as file:
    return list(csv.DictReader(file))


def _write(path: Path, payload: dict[str, Any]) -> None:
  path.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def generate(field_audit: Path, service_audit: Path, output_dir: Path) -> None:
  field_rows = _rows(field_audit)
  service_rows = _rows(service_audit)
  fields: OrderedDict[str, dict[str, str]] = OrderedDict()
  for row in sorted(field_rows, key=lambda item: item.get("field_path", "")):
    status = UNIT_STATUS_MAP.get(row.get("unit_resolution_status", ""))
    unit = row.get("resolved_unit_or_semantic", "")
    path = row.get("field_path", "")
    if not status or not unit or not path:
      continue
    fields[path] = {
      "unit": unit,
      "unitStatus": status,
      "unitBasis": row.get("unit_resolution_basis", ""),
    }

  services: OrderedDict[str, dict[str, str]] = OrderedDict()
  for row in sorted(service_rows, key=lambda item: item.get("service", "")):
    service = row.get("service", "")
    if not service:
      continue
    services[service] = {
      "domain": row.get("category", ""),
      "purpose": row.get("purpose_ko", ""),
      "recommendedView": row.get("recommended_view", ""),
      "privacy": row.get("privacy_risk", ""),
    }

  output_dir.mkdir(parents=True, exist_ok=True)
  field_head = field_rows[0] if field_rows else {}
  service_head = service_rows[0] if service_rows else {}
  _write(output_dir / "field_units.v1.json", {
    "version": 1,
    "schemaCommit": field_head.get("schema_commit", ""),
    "schemaBranch": field_head.get("schema_branch", ""),
    "fields": fields,
  })
  _write(output_dir / "services.v1.json", {
    "version": 1,
    "schemaCommit": service_head.get("schema_commit", ""),
    "schemaBranch": service_head.get("schema_branch", ""),
    "services": services,
  })


def main() -> None:
  parser = argparse.ArgumentParser(description="Generate the replay semantic registry resources from audited CSV files.")
  parser.add_argument("field_audit", type=Path)
  parser.add_argument("service_audit", type=Path)
  parser.add_argument("--output-dir", type=Path, default=Path(__file__).resolve().parent)
  args = parser.parse_args()
  generate(args.field_audit, args.service_audit, args.output_dir)


if __name__ == "__main__":
  main()
