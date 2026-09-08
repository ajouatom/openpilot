"""NAS Python 3.8+ updater. Run from a root DSM scheduled task every five minutes."""
# ruff: noqa: UP017 -- DSM Python 3.8 does not provide datetime.UTC.
import argparse
from datetime import datetime, timezone
import json
import logging
from logging.handlers import RotatingFileHandler
from pathlib import Path
import re
import subprocess
import time
import urllib.request


def atomic_write(path, text):
  temporary = path.with_name(path.name + ".tmp")
  temporary.write_text(text, encoding="utf-8")
  temporary.replace(path)


def pinned_compose(text, image):
  if not re.fullmatch(r"sha256:[0-9a-f]{64}", image):
    raise ValueError("Deployment requires an immutable local image ID")
  result, count = re.subn(r"(?m)^(    image:)\s*[^\r\n]+", r'\1 "' + image + '"', text)
  if count != 1 or not re.search(r"(?m)^  carrot-upload:\s*$", text):
    raise ValueError("Expected the existing single-service carrot-upload compose file")
  return result


class Updater:
  def __init__(self, root, config):
    self.root = root
    self.config = config
    self.compose_path = root / "compose.yaml"
    self.state_path = root / "auto-update-state.json"
    self.journal_path = root / "auto-update-transaction.json"
    self.expected_path = root / "auto-update-expected.json"
    self.state = json.loads(self.state_path.read_text()) if self.state_path.exists() else {}
    self.compose = ["docker", "compose"]
    try:
      self.run(*self.compose, "version")
    except subprocess.CalledProcessError:
      self.compose = ["docker-compose"]

  def run(self, *args, timeout=600, stdin_data=None):
    try:
      result = subprocess.run(args, cwd=str(self.root), check=True, text=True,
                              capture_output=True, timeout=timeout, input=stdin_data)
    except subprocess.CalledProcessError as error:
      logging.exception("%s failed: %s", args[0], (error.stderr or "")[-4000:])
      raise
    return result.stdout.strip()

  def compose_run(self, *args):
    return self.run(*self.compose, "-p", "carrot-route-vault", "-f", "compose.yaml", *args)

  def save(self, **values):
    self.state.update(values, checkedAt=int(datetime.now(timezone.utc).timestamp()))
    atomic_write(self.state_path, json.dumps(self.state, indent=2) + "\n")

  def current_image(self):
    container = self.compose_run("ps", "-q", "carrot-upload")
    if not container or "\n" in container:
      raise RuntimeError("Expected one running production container")
    return self.run("docker", "inspect", "--format", "{{.Image}}", container)

  def probe(self, image, online=False):
    args = ["docker", "run", "--rm", "-i", "--name", "carrot-route-vault-update-probe", "--network", "host",
            "--read-only", "--cap-drop", "ALL", "--security-opt", "no-new-privileges:true",
            "--memory", "2g", "--user", self.config.get("user", "1026:100"), "--group-add", "101",
            "--tmpfs", "/tmp:size=64m", "-v", self.config["data_root"] + ":/data/openpilot:ro"]
    args += [image, "python", "/app/deploy_probe.py"]
    request = {"config": self.config}
    if online:
      args += ["--online"]
      request["expected"] = json.loads(self.expected_path.read_text())["replay"]
    # A previous host crash may have left only this specifically named probe behind.
    try:
      self.run("docker", "rm", "-f", "carrot-route-vault-update-probe", timeout=30)
    except subprocess.CalledProcessError:
      pass
    try:
      return json.loads(self.run(*args, timeout=480, stdin_data=json.dumps(request)))
    finally:
      try:
        self.run("docker", "rm", "-f", "carrot-route-vault-update-probe", timeout=30)
      except subprocess.CalledProcessError:
        pass

  def restore(self, transaction):
    atomic_write(self.compose_path, transaction["previousCompose"])
    self.compose_run("up", "-d", "--no-build", "carrot-upload")
    if self.current_image() != transaction["previousImage"]:
      raise RuntimeError("Rollback did not restore the previous image")
    self.wait_healthy()
    self.save(status="rolled-back", failedImage=transaction["candidate"], image=transaction["previousImage"])
    self.journal_path.unlink()

  def wait_healthy(self):
    deadline = time.monotonic() + 90
    while True:
      try:
        with urllib.request.urlopen(self.config["base_url"].rstrip("/") + "/api/v1/health", timeout=10) as response:
          if response.status == 200 and json.load(response).get("ok"):
            return
      except (OSError, ValueError):
        pass
      if time.monotonic() >= deadline:
        raise RuntimeError("Restored image failed its health check")
      time.sleep(2)

  def update(self):
    if self.journal_path.exists():
      logging.warning("Recovering interrupted deployment")
      self.restore(json.loads(self.journal_path.read_text()))
    logging.info("Checking registry image %s", self.config["image"])
    self.run("docker", "pull", self.config["image"], timeout=1200)
    candidate = self.run("docker", "image", "inspect", "--format", "{{.Id}}", self.config["image"])
    previous = self.current_image()
    if candidate == previous:
      self.save(status="current", image=previous)
      return
    if candidate == self.state.get("failedImage"):
      self.save(status="quarantined")
      return
    # Preserve an immutable rollback reference even during the first migration from a mutable tag.
    original = self.compose_path.read_text()
    rollback = pinned_compose(original, previous)
    replacement = pinned_compose(original, candidate)
    transaction = {"candidate": candidate, "previousImage": previous, "previousCompose": rollback}
    switched = False
    try:
      self.save(status="validating", candidate=candidate)
      logging.info("Replaying configured regression log with candidate %s", candidate)
      expected = self.probe(candidate)
      if not re.fullmatch(r"[0-9a-f]{40}", expected["sourceCommit"]):
        raise ValueError("Candidate has no valid committed source identity")
      atomic_write(self.expected_path, json.dumps(expected))
      atomic_write(self.journal_path, json.dumps(transaction))
      switched = True
      atomic_write(self.compose_path, replacement)
      logging.info("Recreating production container and checking served replay")
      self.compose_run("up", "-d", "--no-build", "carrot-upload")
      if self.current_image() != candidate:
        raise RuntimeError("Production container did not use the pinned candidate")
      actual = self.probe(candidate, online=True)
      if actual != expected:
        raise ValueError("Online and offline validation differ")
      self.save(status="updated", image=candidate, previousImage=previous,
                sourceCommit=actual["sourceCommit"], replay=actual["replay"],
                verifiedAt=int(datetime.now(timezone.utc).timestamp()), failedImage=None, error=None, candidate=None)
      self.journal_path.unlink()
      logging.info("Updated and verified source %s", actual["sourceCommit"])
    except Exception as error:
      self.save(status="failed", failedImage=candidate, error=str(error))
      if switched:
        self.restore(transaction)
      raise


def main():
  import fcntl  # DSM/Linux only; pure transaction tests also run on Windows.
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument("directory", type=Path)
  args = parser.parse_args()
  root = args.directory.resolve()
  handler = RotatingFileHandler(root / "auto-update.log", maxBytes=1024 * 1024, backupCount=2)
  logging.basicConfig(level=logging.INFO, handlers=[handler], format="%(asctime)s %(levelname)s %(message)s")
  with (root / "auto-update.lock").open("a") as lock:
    try:
      fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
      return
    try:
      Updater(root, json.loads((root / "auto-update.json").read_text())).update()
    except Exception:
      logging.exception("Automatic update failed; inspect status/transaction before retrying")
      raise


if __name__ == "__main__":
  main()
