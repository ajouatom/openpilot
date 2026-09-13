import importlib.util
import json
from pathlib import Path
import subprocess

import pytest

REPO_ROOT = Path(__file__).resolve().parents[3]
MAP_PATH = REPO_ROOT / "docs" / "user" / "docs_map.json"
CHECKER_PATH = REPO_ROOT / "tools" / "docs" / "check_user_docs.py"
SPEC = importlib.util.spec_from_file_location("check_user_docs", CHECKER_PATH)
assert SPEC is not None and SPEC.loader is not None
CHECKER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(CHECKER)

extract_override_reason = CHECKER.extract_override_reason
find_missing_docs = CHECKER.find_missing_docs
load_mapping = CHECKER.load_mapping


def test_mapping_is_valid():
  mapping = load_mapping(MAP_PATH)
  assert mapping["version"] == 1
  assert {rule["id"] for rule in mapping["rules"]} >= {
    "settings-catalog-and-web", "cruise-buttons-and-shared-control", "radar", "tesla-vehicle-integration",
  }


def test_mapped_code_requires_documentation():
  mapping = load_mapping(MAP_PATH)
  missing = find_missing_docs(["openpilot/selfdrive/controls/radard.py"], mapping)
  assert [rule["id"] for rule in missing] == ["radar"]


def test_related_document_satisfies_rule():
  mapping = load_mapping(MAP_PATH)
  changed = [
    "openpilot/selfdrive/controls/radard.py",
    "docs/user/ko/radar.md",
    "docs/user/en/radar.md",
  ]
  assert find_missing_docs(changed, mapping) == []


def test_one_language_does_not_satisfy_rule():
  mapping = load_mapping(MAP_PATH)
  changed = ["openpilot/selfdrive/controls/radard.py", "docs/user/ko/radar.md"]
  assert [rule["id"] for rule in find_missing_docs(changed, mapping)] == ["radar"]


def test_any_complete_shared_cruise_document_pair_satisfies_rule():
  mapping = load_mapping(MAP_PATH)
  changed = [
    "openpilot/selfdrive/car/cruise.py",
    "docs/user/ko/buttons-presets.md",
    "docs/user/en/buttons-presets.md",
  ]
  assert find_missing_docs(changed, mapping) == []


def test_tesla_vehicle_change_requires_bilingual_docs():
  mapping = load_mapping(MAP_PATH)
  code_path = "opendbc_repo/opendbc/car/tesla/carstate.py"

  missing = find_missing_docs([code_path, "docs/user/ko/tesla.md"], mapping)
  assert [rule["id"] for rule in missing] == ["tesla-vehicle-integration"]

  changed = [code_path, "docs/user/ko/tesla.md", "docs/user/en/tesla.md"]
  assert find_missing_docs(changed, mapping) == []


def test_independent_rules_each_require_a_document():
  mapping = load_mapping(MAP_PATH)
  changed = [
    "openpilot/selfdrive/controls/radard.py",
    "openpilot/selfdrive/carrot_settings.json",
    "docs/user/ko/radar.md",
    "docs/user/en/radar.md",
  ]
  missing = find_missing_docs(changed, mapping)
  assert [rule["id"] for rule in missing] == ["settings-catalog-and-web"]


def test_override_requires_a_concrete_reason():
  assert extract_override_reason("Docs-Not-Needed: 내부 함수만 분리했고 사용자 동작은 동일함")
  assert extract_override_reason("Docs-Not-Needed: N/A") == ""
  assert extract_override_reason("Docs-Not-Needed:") == ""
  assert extract_override_reason("Docs-Not-Needed:\nUnrelated template instructions") == ""
  assert extract_override_reason("Docs-Not-Needed:\r\nUnrelated template instructions") == ""
  assert extract_override_reason("Docs-Not-Needed: Internal formatting change only.\r\n") == "Internal formatting change only."


@pytest.fixture(autouse=True)
def clear_pr_body(monkeypatch):
  monkeypatch.delenv("PR_BODY", raising=False)


def test_internal_radar_and_web_changes_are_advisory(capsys):
  assert CHECKER.main([
    "--changed-file", "openpilot/selfdrive/carrot/radar_motion/primary.py",
    "--changed-file", "openpilot/selfdrive/carrot/web/src/features/tools/egpu_model.js",
  ]) == 0
  output = capsys.readouterr().out
  assert "Docs review suggested [radar]" in output
  assert "Docs review suggested [carrot-web-pages]" in output


def test_settings_still_require_both_languages():
  changed = ["--changed-file", "openpilot/selfdrive/carrot_settings.json"]
  assert CHECKER.main(changed) == 1
  changed += ["--changed-file", "docs/user/ko/settings.md"]
  assert CHECKER.main(changed) == 1
  changed += ["--changed-file", "docs/user/en/settings.md"]
  assert CHECKER.main(changed) == 0


def test_advisory_change_does_not_hide_missing_settings_docs():
  assert CHECKER.main([
    "--changed-file", "openpilot/selfdrive/carrot/radar_motion/primary.py",
    "--changed-file", "openpilot/selfdrive/carrot_settings.json",
  ]) == 1


def test_settings_internal_change_accepts_pr_reason(monkeypatch):
  monkeypatch.setenv("PR_BODY", "Docs-Not-Needed: Internal refactor with unchanged settings behavior.")
  assert CHECKER.main(["--changed-file", "openpilot/selfdrive/carrot_settings.json"]) == 0


def test_invalid_enforcement_is_rejected(tmp_path):
  mapping = load_mapping(MAP_PATH)
  mapping["rules"][0]["enforcement"] = "disabled"
  path = tmp_path / "map.json"
  path.write_text(json.dumps(mapping), encoding="utf-8")
  with pytest.raises(ValueError, match="invalid enforcement"):
    load_mapping(path)


@pytest.fixture
def git_repo(tmp_path, monkeypatch):
  def git(*args):
    return subprocess.check_output(["git", "-C", str(tmp_path), *args], text=True, encoding="utf-8").strip()

  git("init", "-b", "main")
  git("config", "user.name", "Docs checker test")
  git("config", "user.email", "docs-test@example.com")
  git("config", "commit.gpgsign", "false")

  def commit(path, content, message):
    target = tmp_path / path
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(content, encoding="utf-8")
    git("add", path)
    git("commit", "-m", message)
    return git("rev-parse", "HEAD")

  base = commit("README", "base", "Base commit")
  monkeypatch.setattr(CHECKER, "REPO_ROOT", tmp_path)
  return git, commit, base


def test_push_accepts_reason_only_when_explicitly_enabled(git_repo):
  _, commit, base = git_repo
  commit("openpilot/selfdrive/carrot_settings.json", "{}", "Refactor\n\nDocs-Not-Needed: Settings behavior is unchanged.")
  assert CHECKER.main(["--base", base]) == 1
  assert CHECKER.main(["--base", base, "--commit-overrides"]) == 0


@pytest.mark.parametrize("exempt_first", [True, False])
def test_one_commit_reason_cannot_hide_another_settings_change(git_repo, exempt_first):
  _, commit, base = git_repo
  reasons = ["Internal\n\nDocs-Not-Needed: Settings behavior is unchanged.", "Change settings default"]
  if not exempt_first:
    reasons.reverse()
  for index, reason in enumerate(reasons):
    commit("openpilot/selfdrive/carrot_settings.json", str(index), reason)
  assert CHECKER.main(["--base", base, "--commit-overrides"]) == 1


def test_push_can_supply_bilingual_docs_in_a_later_commit(git_repo):
  _, commit, base = git_repo
  commit("openpilot/selfdrive/carrot_settings.json", "{}", "Change default")
  commit("docs/user/ko/settings.md", "Korean guide", "Document Korean settings")
  commit("docs/user/en/settings.md", "English guide", "Document English settings")
  assert CHECKER.main(["--base", base, "--commit-overrides"]) == 0


def test_merge_changes_require_docs_or_a_reason_on_the_merge(git_repo):
  git, commit, base = git_repo
  git("checkout", "-b", "feature")
  commit("openpilot/selfdrive/carrot_settings.json", "{}", "Change default")
  git("checkout", "main")
  commit("README", "main changed", "Main branch work")
  git("merge", "--no-ff", "feature", "-m", "Merge feature")
  assert CHECKER.main(["--base", base, "--commit-overrides"]) == 1


def test_commit_override_cannot_be_combined_with_explicit_paths():
  with pytest.raises(ValueError, match="Git change range"):
    CHECKER.main(["--commit-overrides", "--changed-file", "openpilot/selfdrive/carrot_settings.json"])
