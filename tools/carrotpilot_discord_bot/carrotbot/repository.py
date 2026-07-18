from __future__ import annotations

import json
from pathlib import Path
import subprocess
from typing import Any

from carrotbot.device_logs import DeviceLogError, DeviceLogs


TEXT_EXTENSIONS = {
  ".c",
  ".cc",
  ".cpp",
  ".h",
  ".hpp",
  ".py",
  ".json",
  ".md",
  ".txt",
  ".toml",
  ".yaml",
  ".yml",
  ".sh",
  ".capnp",
  ".dbc",
  ".ini",
  ".cfg",
}
DENIED_PARTS = {".git", ".ssh", "secrets", "credentials", "__pycache__"}
DENIED_NAMES = {".env", "bot.env", "id_rsa", "id_ed25519"}


class RepositoryError(RuntimeError):
  pass


class Repository:
  def __init__(self, root: Path, repo_url: str, branch: str, device_logs_root: Path | None = None):
    self.root = root
    self.repo_url = repo_url
    self.branch = branch
    self.device_logs = DeviceLogs(device_logs_root)

  def _run(self, args: list[str], *, cwd: Path | None = None, timeout: int = 120) -> str:
    result = subprocess.run(
      args,
      cwd=cwd,
      capture_output=True,
      text=True,
      encoding="utf-8",
      errors="replace",
      timeout=timeout,
      check=False,
    )
    if result.returncode != 0:
      detail = (result.stderr or result.stdout).strip()[-1200:]
      raise RepositoryError(f"명령 실패({result.returncode}): {' '.join(args[:3])}: {detail}")
    return result.stdout.strip()

  def ensure_available(self) -> None:
    if (self.root / ".git").is_dir():
      self.update()
      return
    if self.root.exists() and any(self.root.iterdir()):
      raise RepositoryError(f"비어 있지 않은 경로에 Git 저장소가 없습니다: {self.root}")
    self.root.parent.mkdir(parents=True, exist_ok=True)
    self._run(
      [
        "git",
        "clone",
        "--filter=blob:none",
        "--single-branch",
        "--branch",
        self.branch,
        self.repo_url,
        str(self.root),
      ],
      timeout=900,
    )

  def update(self) -> None:
    self._require_repo()
    self._run(["git", "pull", "--ff-only", "origin", self.branch], cwd=self.root, timeout=300)

  def _require_repo(self) -> None:
    if not (self.root / ".git").is_dir():
      raise RepositoryError("carrotpilot 저장소가 아직 준비되지 않았습니다.")

  def repo_info(self) -> dict[str, str]:
    self._require_repo()
    return {
      "branch": self._run(["git", "branch", "--show-current"], cwd=self.root),
      "commit": self._run(["git", "rev-parse", "--short=12", "HEAD"], cwd=self.root),
      "subject": self._run(["git", "log", "-1", "--pretty=%s"], cwd=self.root),
    }

  def _safe_scope(self, relative: str | None) -> Path:
    self._require_repo()
    candidate = self.root if not relative else self.root / relative
    resolved_root = self.root.resolve()
    resolved = candidate.resolve()
    try:
      rel = resolved.relative_to(resolved_root)
    except ValueError as exc:
      raise RepositoryError("저장소 밖의 경로는 접근할 수 없습니다.") from exc
    if any(part.lower() in DENIED_PARTS for part in rel.parts):
      raise RepositoryError("보호된 경로는 접근할 수 없습니다.")
    if not resolved.exists():
      raise RepositoryError(f"경로를 찾을 수 없습니다: {relative}")
    return resolved

  def _safe_file(self, relative: str) -> Path:
    path = self._safe_scope(relative)
    if not path.is_file():
      raise RepositoryError(f"파일이 아닙니다: {relative}")
    if path.name.lower() in DENIED_NAMES or path.suffix.lower() not in TEXT_EXTENSIONS:
      raise RepositoryError("허용되지 않은 파일 형식 또는 보호된 파일입니다.")
    return path

  def search_code(self, query: str, path_prefix: str | None = None) -> str:
    query = query.strip()
    if not 2 <= len(query) <= 120 or "\n" in query or "\r" in query:
      raise RepositoryError("검색어는 줄바꿈 없이 2~120자여야 합니다.")
    scope = self._safe_scope(path_prefix)
    args = [
      "rg",
      "-n",
      "-i",
      "-F",
      "--max-count",
      "5",
      "--glob",
      "!.git/**",
      "--glob",
      "!third_party/**",
      "--glob",
      "!**/__pycache__/**",
      "--glob",
      "!**/*.bin",
      "--glob",
      "!**/*.onnx",
      "--glob",
      "!**/*.jpg",
      "--glob",
      "!**/*.png",
      "--",
      query,
      str(scope),
    ]
    result = subprocess.run(
      args,
      cwd=self.root,
      capture_output=True,
      text=True,
      encoding="utf-8",
      errors="replace",
      timeout=30,
      check=False,
    )
    if result.returncode not in {0, 1}:
      raise RepositoryError((result.stderr or "코드 검색 실패").strip()[-1000:])
    if result.returncode == 1:
      return "검색 결과 없음"
    lines = result.stdout.splitlines()[:80]
    normalized = [line.replace(str(self.root) + "/", "").replace(str(self.root) + "\\", "") for line in lines]
    return "\n".join(normalized)[:18000]

  def find_files(self, name: str) -> str:
    name = name.strip().lower()
    if not 2 <= len(name) <= 80 or any(ch in name for ch in "\r\n\0"):
      raise RepositoryError("파일 검색어는 2~80자여야 합니다.")
    output = self._run(["rg", "--files", "-g", "!.git/**", "-g", "!third_party/**"], cwd=self.root, timeout=30)
    matches = []
    for item in output.splitlines():
      path = Path(item)
      if name in item.lower() and path.suffix.lower() in TEXT_EXTENSIONS:
        matches.append(item)
      if len(matches) >= 60:
        break
    return "\n".join(matches) if matches else "검색 결과 없음"

  def read_file(self, path: str, start_line: int, end_line: int) -> str:
    if start_line < 1 or end_line < start_line:
      raise RepositoryError("줄 범위가 올바르지 않습니다.")
    end_line = min(end_line, start_line + 249)
    target = self._safe_file(path)
    lines = target.read_text(encoding="utf-8", errors="replace").splitlines()
    selected = lines[start_line - 1 : end_line]
    if not selected:
      return "요청한 줄 범위에 내용이 없습니다."
    return "\n".join(f"{index}:{line}" for index, line in enumerate(selected, start=start_line))[:20000]

  def call_tool(self, name: str, arguments: dict[str, Any]) -> str:
    try:
      if name == "repo_info":
        result: Any = self.repo_info()
      elif name == "search_code":
        result = self.search_code(arguments["query"], arguments.get("path_prefix"))
      elif name == "find_files":
        result = self.find_files(arguments["name"])
      elif name == "read_file":
        result = self.read_file(arguments["path"], arguments["start_line"], arguments["end_line"])
      elif name == "inspect_device_logs":
        result = self.device_logs.inspect(
          arguments["dongle_id"],
          arguments["question"],
          arguments["requested_date"],
          arguments["session_offset"],
        )
      else:
        raise RepositoryError(f"알 수 없는 도구입니다: {name}")
      return result if isinstance(result, str) else json.dumps(result, ensure_ascii=False)
    except (KeyError, TypeError, ValueError, OSError, RepositoryError, DeviceLogError) as exc:
      return json.dumps({"error": str(exc)}, ensure_ascii=False)
