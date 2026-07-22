#!/usr/bin/env python3
from __future__ import annotations

import argparse
import fnmatch
import json
import os
from pathlib import Path
import re
import subprocess
import sys
from typing import Any


REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_MAP = REPO_ROOT / "docs" / "user" / "docs_map.json"
EMPTY_BASES = {"", "0" * 40, "0" * 64}
INVALID_REASONS = {"n/a", "na", "none", "없음", "해당 없음", "-", "<reason>", "<사유>"}


def load_mapping(path: Path) -> dict[str, Any]:
  with path.open(encoding="utf-8") as f:
    mapping = json.load(f)

  if mapping.get("version") != 1 or not isinstance(mapping.get("rules"), list):
    raise ValueError(f"unsupported documentation map: {path}")

  for rule in mapping["rules"]:
    for key in ("id", "description", "code_paths", "doc_sets"):
      if key not in rule:
        raise ValueError(f"rule is missing {key!r}: {rule}")
    if not rule["code_paths"] or not rule["doc_sets"] or any(not doc_set for doc_set in rule["doc_sets"]):
      raise ValueError(f"rule must contain code_paths and non-empty doc_sets: {rule['id']}")
  return mapping


def normalize_path(path: str) -> str:
  return path.strip().replace("\\", "/").removeprefix("./")


def matches_any(path: str, patterns: list[str]) -> bool:
  normalized = normalize_path(path)
  return any(fnmatch.fnmatchcase(normalized, normalize_path(pattern)) for pattern in patterns)


def find_missing_docs(changed_files: list[str], mapping: dict[str, Any]) -> list[dict[str, Any]]:
  changed = [normalize_path(path) for path in changed_files if normalize_path(path)]
  missing: list[dict[str, Any]] = []
  for rule in mapping["rules"]:
    matched_code = sorted(path for path in changed if matches_any(path, rule["code_paths"]))
    if not matched_code:
      continue
    complete_doc_set = any(
      all(any(matches_any(path, [doc_path]) for path in changed) for doc_path in doc_set)
      for doc_set in rule["doc_sets"]
    )
    if not complete_doc_set:
      missing.append({**rule, "matched_code": matched_code})
  return missing


def extract_override_reason(pr_body: str) -> str:
  match = re.search(r"(?im)^\s*Docs-Not-Needed\s*:\s*(.+?)\s*$", pr_body or "")
  if match is None:
    return ""
  reason = match.group(1).strip()
  if reason.casefold() in INVALID_REASONS or len(reason) < 8:
    return ""
  return reason


def git_output(*args: str) -> str:
  result = subprocess.run(
    ["git", *args], cwd=REPO_ROOT, check=True, text=True, encoding="utf-8", capture_output=True,
  )
  return result.stdout.strip()


def resolve_base(base: str, head: str) -> str:
  candidate = (base or "").strip()
  if candidate not in EMPTY_BASES:
    try:
      git_output("rev-parse", "--verify", f"{candidate}^{{commit}}")
      return candidate
    except subprocess.CalledProcessError:
      pass
  return git_output("rev-parse", f"{head}^")


def changed_files_from_git(base: str, head: str) -> list[str]:
  resolved_base = resolve_base(base, head)
  output = git_output("diff", "--name-only", "--diff-filter=ACMRD", f"{resolved_base}...{head}")
  return [normalize_path(line) for line in output.splitlines() if line.strip()]


def build_parser() -> argparse.ArgumentParser:
  parser = argparse.ArgumentParser(description="Check that mapped user docs change with user-visible code.")
  parser.add_argument("--base", default="", help="base Git ref or SHA; defaults to HEAD^")
  parser.add_argument("--head", default="HEAD", help="head Git ref or SHA")
  parser.add_argument("--map", type=Path, default=DEFAULT_MAP, help="path to docs_map.json")
  parser.add_argument(
    "--changed-file", action="append", default=[], help="explicit changed path; repeat to bypass git diff",
  )
  return parser


def main(argv: list[str] | None = None) -> int:
  args = build_parser().parse_args(argv)
  mapping = load_mapping(args.map)
  changed_files = args.changed_file or changed_files_from_git(args.base, args.head)
  missing = find_missing_docs(changed_files, mapping)

  if not missing:
    print(f"User docs check passed ({len(changed_files)} changed files).")
    return 0

  override_reason = extract_override_reason(os.environ.get("PR_BODY", ""))
  if override_reason:
    print(f"User docs update skipped: {override_reason}")
    for rule in missing:
      print(f"  - {rule['id']}: {', '.join(rule['matched_code'])}")
    return 0

  print("User-visible code changed without a mapped user-documentation update:", file=sys.stderr)
  for rule in missing:
    print(f"\n[{rule['id']}] {rule['description']}", file=sys.stderr)
    print(f"  code: {', '.join(rule['matched_code'])}", file=sys.stderr)
    formatted_sets = [" + ".join(doc_set) for doc_set in rule["doc_sets"]]
    print(f"  update one complete set: {'; or '.join(formatted_sets)}", file=sys.stderr)
  print(
    "\nUpdate the relevant document, or add `Docs-Not-Needed: <concrete reason>` to the PR body.",
    file=sys.stderr,
  )
  return 1


if __name__ == "__main__":
  raise SystemExit(main())
