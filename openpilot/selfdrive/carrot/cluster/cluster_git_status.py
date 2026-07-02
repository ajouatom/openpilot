from __future__ import annotations

import os
import subprocess
import threading
import time
from pathlib import Path

from cluster_models import GitBranchStatus


GIT_STATUS_REFRESH_SECONDS = 60.0
GIT_COMMAND_TIMEOUT_SECONDS = 4.0


def find_git_root(start: Path) -> Path | None:
    path = start.resolve()
    candidates = (path, *path.parents)
    for candidate in candidates:
        if (candidate / ".git").exists():
            return candidate
    return None


def resolve_git_dir(repo_path: Path) -> Path | None:
    git_path = repo_path / ".git"
    if git_path.is_dir():
        return git_path
    if not git_path.is_file():
        return None
    try:
        text = git_path.read_text(encoding="utf-8", errors="replace").strip()
    except OSError:
        return None
    prefix = "gitdir:"
    if not text.lower().startswith(prefix):
        return None
    target = Path(text[len(prefix) :].strip())
    if not target.is_absolute():
        target = repo_path / target
    return target


def read_head_branch(repo_path: Path) -> str | None:
    git_dir = resolve_git_dir(repo_path)
    if git_dir is None:
        return None
    try:
        head = (git_dir / "HEAD").read_text(encoding="utf-8", errors="replace").strip()
    except OSError:
        return None
    ref_prefix = "ref: refs/heads/"
    if head.startswith(ref_prefix):
        return head[len(ref_prefix) :]
    return head[:12] if head else None


class GitBranchStatusProvider:
    def __init__(
        self,
        start_path: Path,
        refresh_interval_s: float = GIT_STATUS_REFRESH_SECONDS,
        command_timeout_s: float = GIT_COMMAND_TIMEOUT_SECONDS,
    ) -> None:
        self.repo_path = find_git_root(start_path)
        self.refresh_interval_s = max(5.0, float(refresh_interval_s))
        self.command_timeout_s = max(0.5, float(command_timeout_s))
        initial_branch = read_head_branch(self.repo_path) if self.repo_path is not None else None
        initial_detail = "확인 중" if self.repo_path is not None else "저장소 없음"
        self._status = GitBranchStatus(initial_branch or "git", "unknown", initial_detail)
        self._next_refresh = 0.0
        self._lock = threading.Lock()
        self._worker: threading.Thread | None = None

    def status(self) -> GitBranchStatus:
        now = time.monotonic()
        with self._lock:
            worker_alive = self._worker is not None and self._worker.is_alive()
            if self.repo_path is not None and now >= self._next_refresh and not worker_alive:
                self._next_refresh = now + self.refresh_interval_s
                self._worker = threading.Thread(target=self._refresh, name="cluster-git-status", daemon=True)
                self._worker.start()
            return self._status

    def _refresh(self) -> None:
        status = self._read_status()
        with self._lock:
            self._status = status

    def _read_status(self) -> GitBranchStatus:
        if self.repo_path is None:
            return GitBranchStatus("git", "unknown", "저장소 없음")

        branch = self._current_branch()
        if branch is None:
            return GitBranchStatus(read_head_branch(self.repo_path) or "HEAD", "unknown", "브렌치 아님")

        remote_name, remote_branch = self._tracking_branch(branch)
        if remote_name is None or remote_branch is None:
            return GitBranchStatus(branch, "missing", "원격 설정 없음")

        remote_exists = self._remote_branch_exists(remote_name, remote_branch)
        if remote_exists is False:
            return GitBranchStatus(branch, "missing", "원격에서 삭제됨")
        if remote_exists is None:
            return GitBranchStatus(branch, "unknown", "원격 확인 실패")

        behind_count = self._behind_count(remote_name, remote_branch)
        if behind_count is None:
            return GitBranchStatus(branch, "unknown", "pull 확인 실패")
        if behind_count > 0:
            return GitBranchStatus(branch, "pull", f"git pull 가능 +{behind_count}")
        return GitBranchStatus(branch, "ok")

    def _current_branch(self) -> str | None:
        result = self._git("branch", "--show-current", timeout_s=1.0)
        if result.returncode == 0:
            branch = result.stdout.strip()
            if branch:
                return branch
        return None

    def _tracking_branch(self, branch: str) -> tuple[str | None, str | None]:
        remotes = self._remote_names()
        if not remotes:
            return None, None

        upstream_result = self._git("for-each-ref", "--format=%(upstream:short)", f"refs/heads/{branch}", timeout_s=1.0)
        upstream = upstream_result.stdout.strip() if upstream_result.returncode == 0 else ""
        if upstream:
            for remote_name in sorted(remotes, key=len, reverse=True):
                prefix = f"{remote_name}/"
                if upstream.startswith(prefix):
                    remote_branch = upstream[len(prefix) :]
                    if remote_branch:
                        return remote_name, remote_branch

        remote_name = "origin" if "origin" in remotes else remotes[0]
        return remote_name, branch

    def _remote_names(self) -> list[str]:
        remotes_result = self._git("remote", timeout_s=1.0)
        if remotes_result.returncode != 0:
            return []
        return [line.strip() for line in remotes_result.stdout.splitlines() if line.strip()]

    def _remote_branch_exists(self, remote_name: str, remote_branch: str) -> bool | None:
        result = self._git(
            "ls-remote",
            "--exit-code",
            "--heads",
            remote_name,
            remote_branch,
            timeout_s=self.command_timeout_s,
        )
        if result.returncode == 0:
            return True
        if result.returncode == 2:
            return False
        return None

    def _behind_count(self, remote_name: str, remote_branch: str) -> int | None:
        tracking_ref = f"refs/remotes/{remote_name}/{remote_branch}"
        fetch_result = self._git(
            "fetch",
            "--quiet",
            "--prune",
            remote_name,
            f"+refs/heads/{remote_branch}:{tracking_ref}",
            timeout_s=self.command_timeout_s,
        )
        if fetch_result.returncode != 0:
            return None

        result = self._git("rev-list", "--count", f"HEAD..{tracking_ref}", timeout_s=1.0)
        if result.returncode != 0:
            return None
        try:
            return max(0, int(result.stdout.strip()))
        except ValueError:
            return None

    def _git(self, *args: str, timeout_s: float) -> subprocess.CompletedProcess[str]:
        env = os.environ.copy()
        env["GIT_TERMINAL_PROMPT"] = "0"
        try:
            return subprocess.run(
                ("git", "-C", str(self.repo_path), *args),
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                encoding="utf-8",
                errors="replace",
                env=env,
                timeout=timeout_s,
                check=False,
            )
        except (OSError, subprocess.TimeoutExpired) as exc:
            return subprocess.CompletedProcess(args, returncode=124, stdout="", stderr=str(exc))
