"""Repair migrated checkout configuration without changing the selected model."""
from __future__ import annotations

import subprocess

from openpilot.common.repo_update import child_lock_kwargs


class GitConfigError(Exception):
  pass


def repair_git_config(repo_dir: str, *, remote: str | None = None, repair_upstream: bool = True) -> tuple[int, str]:
  """Verify remote heads, remove obsolete fetch refs, and reconnect missing upstreams.

  A valid differently named upstream is intentional and is preserved. A missing
  upstream may only fall back to the current branch's exact name on the selected
  remote. Never select a different model branch or replace a remote URL here.
  """
  messages: list[str] = []

  def git(*args: str, allowed: tuple[int, ...] = (0,), timeout: float = 15.0) -> str:
    proc = subprocess.run(
      ["git", *args], cwd=repo_dir, capture_output=True, text=True, encoding="utf-8", errors="replace", timeout=timeout, **child_lock_kwargs(),
    )
    output = "\n".join(part.strip() for part in (proc.stdout, proc.stderr) if part.strip())
    if proc.returncode not in allowed:
      raise GitConfigError(output or f"git {args[0]} failed ({proc.returncode})")
    return output

  try:
    branch = git("symbolic-ref", "--quiet", "--short", "HEAD", allowed=(0, 1))
    configured_remote = git("config", "--get", f"branch.{branch}.remote", allowed=(0, 1)) if branch else ""
    remotes = git("remote").splitlines()
    selected_remote = remote or configured_remote or ("origin" if "origin" in remotes else "")
    if selected_remote == ".":
      return 0, "Git configuration: local upstream preserved."
    if not selected_remote and not remotes:
      return 0, "Git configuration: no remote configured."
    if selected_remote not in remotes:
      raise GitConfigError("Current Git remote is missing; select the intended repository before retrying.")

    messages.append(f"Checking Git configuration for {selected_remote}.")
    # Inspect the server, not potentially stale refs/remotes left in the checkout.
    advertised = git("ls-remote", "--heads", selected_remote, timeout=30.0)
    heads = {}
    for line in advertised.splitlines():
      parts = line.split()
      if len(parts) == 2 and parts[1].startswith("refs/heads/"):
        heads[parts[1][len("refs/heads/"):]] = parts[0]

    target = ""
    merge_refs = git("config", "--get-all", f"branch.{branch}.merge", allowed=(0, 1)).splitlines() if branch else []
    if repair_upstream and branch:
      if len(merge_refs) > 1:
        # A vehicle selects one branch. Recover duplicate/stale merge entries
        # using that exact branch, never by choosing an arbitrary model.
        if len(set(merge_refs)) > 1 and branch not in heads:
          raise GitConfigError("Multiple upstream branches configured and no matching remote branch exists.")
        local = git("config", "--local", "--get-all", f"branch.{branch}.merge", allowed=(0, 1)).splitlines()
        if merge_refs != local:
          raise GitConfigError("Multiple upstream branches are inherited from outside this checkout; correct their source config.")
        if len(set(merge_refs)) > 1:
          target = branch
      merge_ref = merge_refs[0] if merge_refs else ""
      tracked_branch = merge_ref[len("refs/heads/"):] if merge_ref.startswith("refs/heads/") else ""
      if not target and configured_remote == selected_remote and tracked_branch in heads:
        target = tracked_branch
      elif not target and branch in heads:
        target = branch
      elif not target:
        raise GitConfigError(f"No valid upstream or matching remote branch for {branch}; select the intended branch before retrying.")

    key = f"remote.{selected_remote}.fetch"
    specs = git("config", "--get-all", key, allowed=(0, 1)).splitlines()
    local_specs = git("config", "--local", "--get-all", key, allowed=(0, 1)).splitlines()
    obsolete = []
    for spec in specs:
      source = spec.removeprefix("+").split(":", 1)[0]
      if source.startswith("refs/heads/") and "*" not in source and source[len("refs/heads/"):] not in heads:
        if spec not in local_specs:
          raise GitConfigError(f"Obsolete fetch ref {source} is inherited from outside this checkout; its source config must be corrected.")
        obsolete.append(spec)

    for spec in dict.fromkeys(obsolete):
      git("config", "--local", "--fixed-value", "--unset-all", key, spec)
      messages.append(f"Removed obsolete fetch ref: {spec}")
    wildcard = f"+refs/heads/*:refs/remotes/{selected_remote}/*"
    if wildcard not in specs:
      # Keep tag mappings, negative refspecs, and other deliberate mappings.
      git("config", "--local", "--add", key, wildcard)
      messages.append(f"Enabled remote branch tracking for {selected_remote}.")

    fetch_refs = [f"+refs/heads/{target}:refs/remotes/{selected_remote}/{target}"] if target else []
    output = git("fetch", "--prune", "--no-recurse-submodules", selected_remote, *fetch_refs, timeout=180.0)
    if output:
      messages.append(output)
    if target:
      upstream = f"{selected_remote}/{target}"
      fetched_head = git("rev-parse", "--verify", f"refs/remotes/{upstream}")
      if fetched_head != heads[target]:
        raise GitConfigError(f"Remote branch {upstream} changed or was excluded from fetch; retry after checking its fetch configuration.")
      actual_upstream = git("rev-parse", "--abbrev-ref", "--symbolic-full-name", "@{upstream}", allowed=(0, 128))
      if configured_remote != selected_remote or merge_refs != [f"refs/heads/{target}"] or actual_upstream != upstream:
        if len(merge_refs) > 1:
          git("config", "--local", "--replace-all", f"branch.{branch}.merge", f"refs/heads/{target}")
        git("branch", f"--set-upstream-to={upstream}", branch)
        messages.append(f"Upstream repaired: {branch} -> {upstream}")
      else:
        messages.append(f"Upstream verified: {branch} -> {upstream}")
    messages.append("Git configuration verified.")
    return 0, "\n".join(messages)
  except (GitConfigError, OSError, subprocess.TimeoutExpired) as exc:
    messages.append(f"Git configuration repair failed: {exc}")
    return 1, "\n".join(messages)


def prepare_git_pull(repo_dir: str) -> tuple[int, str, str]:
  """Under repo_lock, fetch the selected branch and pin its single commit.

  Callers merge this object ID, never FETCH_HEAD: unrelated fetches can rewrite
  FETCH_HEAD between Git commands, even when the selected branch is correct.
  """
  rc, output = repair_git_config(repo_dir)
  if rc:
    return rc, output, ""
  try:
    proc = subprocess.run(
      ["git", "rev-parse", "--verify", "@{upstream}^{commit}"], cwd=repo_dir,
      capture_output=True, text=True, encoding="utf-8", errors="replace", timeout=15, **child_lock_kwargs(),
    )
    if proc.returncode:
      return proc.returncode, output + "\n" + proc.stderr.strip(), ""
    return 0, output, proc.stdout.strip()
  except (OSError, subprocess.TimeoutExpired) as exc:
    return 1, output + "\n" + str(exc), ""


def main() -> int:
  """Recovery terminal entry point; caller holds the repository lock."""
  import os

  rc, output, target = prepare_git_pull(os.getcwd())
  print(output, flush=True)
  if rc:
    return rc
  for args in (["reset", "--hard"], ["merge", "--ff-only", target]):
    rc = subprocess.call(["git", *args], **child_lock_kwargs())
    if rc:
      return rc
  return 0


if __name__ == "__main__":
  raise SystemExit(main())
