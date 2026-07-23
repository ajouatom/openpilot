#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
from typing import Any

import generate
import validate


SPEC_ROOT = Path(__file__).resolve().parent
DEFAULT_CATALOG = generate.DEFAULT_CATALOG


def _write_text(path: Path, text: str) -> None:
  path.parent.mkdir(parents=True, exist_ok=True)
  path.write_bytes(text.encode("utf-8"))


def _issue_payload(page: str, issue: validate.ValidationIssue) -> dict[str, Any]:
  return {
    "page": page,
    "line": issue.line,
    "location": issue.location,
    "message": issue.message,
  }


def _list_section(title: str, values: list[str], *, limit: int = 40) -> list[str]:
  lines = [f"### {title} · {len(values)}", ""]
  if not values:
    return [*lines, "- 없음", ""]
  lines.extend(f"- `{value}`" for value in values[:limit])
  if len(values) > limit:
    lines.append(f"- 나머지 {len(values) - limit}건은 `summary.json`에서 확인")
  lines.append("")
  return lines


def build_summary(report: dict[str, Any]) -> str:
  result = report["result"]
  page_changes = result["pageChanges"]
  setting_changes = result["settingChanges"]
  review = result["review"]
  issues = report["validationIssues"]
  warnings = result["warnings"]
  status = "PASS" if report["ok"] else "FAIL"
  lines = [
    "# Carrot Wiki 설정 검사",
    "",
    f"**{status}** · 설정 {result['settings']}개 · 생성 파일 {result['pages']}개",
    "",
    "| 항목 | 추가 | 변경 | 이동 | 삭제 |",
    "|---|---:|---:|---:|---:|",
    (
      f"| 설정 | {len(setting_changes['added'])} | {len(setting_changes['changed'])} | "
      f"{len(setting_changes['moved'])} | {len(setting_changes['removed'])} |"
    ),
    (
      f"| 페이지 | {len(page_changes['added'])} | {len(page_changes['changed'])} | "
      f"- | {len(page_changes['removed'])} |"
    ),
    "",
    f"- 검토 완료: {review['current']}개",
    f"- 검토 필요: {review['needs_review']}개",
    f"- 카탈로그 경고: {len(warnings)}개",
    f"- 구조·Markdown 오류: {len(issues)}개",
    "",
  ]
  lines.extend(_list_section("추가 설정", setting_changes["added"]))
  lines.extend(_list_section("변경 설정", setting_changes["changed"]))
  lines.extend(_list_section("이동 설정", setting_changes["moved"]))
  lines.extend(_list_section("삭제 설정", setting_changes["removed"]))
  if warnings:
    lines.extend(("### 경고", ""))
    lines.extend(f"- {warning}" for warning in warnings[:80])
    if len(warnings) > 80:
      lines.append(f"- 나머지 {len(warnings) - 80}건은 `summary.json`에서 확인")
    lines.append("")
  if issues:
    lines.extend(("### 검증 오류", ""))
    for issue in issues[:80]:
      line = f":{issue['line']}" if issue["line"] is not None else ""
      lines.append(f"- `{issue['page']}{line}` - {issue['message']}")
    if len(issues) > 80:
      lines.append(f"- 나머지 {len(issues) - 80}건은 `summary.json`에서 확인")
    lines.append("")
  lines.extend((
    "### Artifact",
    "",
    "- `summary.json`: 전체 설정·페이지 Diff, 검토 상태, 경고와 검증 오류",
    "- `summary.md`: 사람이 읽는 요약",
    "- `pages.diff`: 현재 Wiki 대비 생성 페이지의 unified diff",
    "",
    "이 검사는 Wiki와 저장소에 쓰지 않습니다.",
    "",
  ))
  return "\n".join(lines)


def run_check(
  *,
  catalog: Path,
  wiki_dir: Path | None,
  output_dir: Path,
  locales: tuple[str, ...],
  catalog_commit: str | None,
  wiki_commit: str | None,
) -> tuple[int, dict[str, Any]]:
  output_dir = output_dir.resolve()
  if wiki_dir is not None and wiki_dir.resolve() == output_dir:
    raise generate.GenerationError("report output directory must differ from the read-only Wiki directory")
  result = generate.generate(
    catalog,
    wiki_dir=wiki_dir,
    locales=locales,
    catalog_commit=catalog_commit,
    wiki_commit=wiki_commit,
  )
  validation_issues: list[dict[str, Any]] = []
  for page, text in sorted(result.pages.items()):
    if not page.endswith(".md"):
      continue
    validation_issues.extend(
      _issue_payload(page, issue)
      for issue in validate.validate_wiki_markdown(text)
    )

  expected_index_hash = generate._canonical_hash({
    key: value for key, value in result.index.items() if key != "contentHash"
  })
  if result.index.get("contentHash") != expected_index_hash:
    validation_issues.append({
      "page": generate.INDEX_NAME,
      "line": None,
      "location": "$.contentHash",
      "message": "index contentHash does not match canonical content",
    })

  report = {
    "ok": not validation_issues,
    "result": result.report(),
    "validationIssues": validation_issues,
  }
  report["result"]["mode"] = "read-only-check"
  _write_text(output_dir / "summary.json", json.dumps(report, ensure_ascii=False, sort_keys=True, indent=2) + "\n")
  _write_text(output_dir / "summary.md", build_summary(report))
  _write_text(output_dir / "pages.diff", result.unified_diff())
  return (0 if report["ok"] else 1), report


def build_parser() -> argparse.ArgumentParser:
  parser = argparse.ArgumentParser(description="Run the read-only Carrot Wiki settings PR check.")
  parser.add_argument("--catalog", type=Path, default=DEFAULT_CATALOG)
  parser.add_argument("--wiki-dir", type=Path, help="read-only Wiki checkout used as the comparison source")
  parser.add_argument("--output-dir", type=Path, required=True, help="local report Artifact directory")
  parser.add_argument("--locales", nargs="+", default=list(generate.DEFAULT_LOCALES))
  parser.add_argument("--catalog-commit")
  parser.add_argument("--wiki-commit")
  return parser


def main(argv: list[str] | None = None) -> int:
  args = build_parser().parse_args(argv)
  try:
    status, report = run_check(
      catalog=args.catalog,
      wiki_dir=args.wiki_dir,
      output_dir=args.output_dir,
      locales=tuple(args.locales),
      catalog_commit=args.catalog_commit,
      wiki_commit=args.wiki_commit,
    )
  except (generate.GenerationError, OSError) as error:
    failure = {
      "ok": False,
      "error": str(error),
    }
    try:
      _write_text(
        args.output_dir.resolve() / "summary.json",
        json.dumps(failure, ensure_ascii=False, sort_keys=True, indent=2) + "\n",
      )
      _write_text(
        args.output_dir.resolve() / "summary.md",
        f"# Carrot Wiki 설정 검사\n\n**FAIL** · {error}\n\n이 검사는 Wiki와 저장소에 쓰지 않습니다.\n",
      )
    except OSError:
      pass
    print(f"Wiki settings PR check failed: {error}", file=sys.stderr)
    return 1

  print(build_summary(report), end="")
  return status


if __name__ == "__main__":
  raise SystemExit(main())
