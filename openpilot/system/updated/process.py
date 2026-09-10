"""Updater commands must not leave Git workers running after onroad shutdown."""
import os
import signal
import subprocess


def command_env() -> dict[str, str]:
  env = os.environ.copy()
  # Command-scope config is inherited by Git children, including submodules.
  # Preserve any existing entries while overriding maintenance for this update.
  count = int(env.get("GIT_CONFIG_COUNT", "0"))
  for key, value in (("gc.auto", "0"), ("gc.autoDetach", "false"), ("maintenance.auto", "false")):
    env[f"GIT_CONFIG_KEY_{count}"] = key
    env[f"GIT_CONFIG_VALUE_{count}"] = value
    count += 1
  env["GIT_CONFIG_COUNT"] = str(count)
  return env


def run(cmd: list[str], cwd: str | None = None) -> str:
  # check_output kills only its direct child on KeyboardInterrupt. A gc child
  # can already have spawned repack/pack-objects, which would survive as orphans.
  proc = subprocess.Popen(cmd, cwd=cwd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                          encoding="utf8", env=command_env(), start_new_session=(os.name != "nt"))
  try:
    output, _ = proc.communicate()
    if proc.returncode:
      raise subprocess.CalledProcessError(proc.returncode, cmd, output=output)
    return output
  finally:
    # Kill the group even if the direct child has already exited. Git workers
    # may ignore SIGINT/SIGTERM; finish before manager's five-second kill limit.
    try:
      if os.name == "nt":
        if proc.poll() is None:
          proc.kill()
      else:
        os.killpg(proc.pid, signal.SIGKILL)
    except ProcessLookupError:
      pass
    proc.wait()
    if proc.stdout is not None:
      proc.stdout.close()
