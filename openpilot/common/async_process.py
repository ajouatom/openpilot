"""Async subprocesses whose process group cannot outlive timeout/cancellation."""
import asyncio
import os
import signal

from openpilot.common.repo_update import child_lock_kwargs, recover_stale_index_lock


async def run_locked_thread(function, *args, **kwargs):
  # to_thread continues running after cancellation. Keep the caller's lock held
  # until the inspection/unlink has actually finished.
  task = asyncio.create_task(asyncio.to_thread(function, *args, **kwargs))
  try:
    return await asyncio.shield(task)
  except asyncio.CancelledError:
    await asyncio.shield(task)
    raise


async def prepare_repo(repo_dir: str) -> bool:
  return await run_locked_thread(recover_stale_index_lock, repo_dir)


def process_group_kwargs() -> dict:
  return {"start_new_session": True, **child_lock_kwargs()} if os.name != "nt" else {}


async def stop_process(proc) -> None:
  def send(sig):
    try:
      if os.name == "nt":
        proc.terminate() if sig == signal.SIGTERM else proc.kill()
      else:
        os.killpg(proc.pid, sig)
    except ProcessLookupError:
      pass

  send(signal.SIGTERM)
  try:
    await asyncio.wait_for(proc.wait(), 1.0)
  except TimeoutError:
    pass
  # The parent may exit before its children. Kill the remaining group as well.
  send(signal.SIGKILL if os.name != "nt" else signal.SIGTERM)
  await proc.wait()


async def run_process(args, *, cwd=None, timeout=None) -> tuple[int, str]:  # noqa: ASYNC109
  proc = await asyncio.create_subprocess_exec(
    *args, cwd=cwd, stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.STDOUT,
    **process_group_kwargs(),
  )
  output = asyncio.create_task(proc.communicate())
  try:
    stdout, _ = await asyncio.wait_for(asyncio.shield(output), timeout)
    return int(proc.returncode or 0), (stdout or b"").decode("utf-8", "replace").strip()
  except (TimeoutError, asyncio.CancelledError):
    await asyncio.shield(stop_process(proc))
    await asyncio.shield(output)
    raise
