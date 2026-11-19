#!/usr/bin/env python3
"""Delete what a device build did not use from cargo's target directory.

`device_build.sh` keeps cargo's JSON messages of the build that just
succeeded. Cargo reports every unit of that build, fresh or rebuilt, so the
16-hex unit hashes in those messages are exactly what the next build can
reuse; outputs of older versions, toolchains or features carry other hashes
and are removed. A binary is reported only by its uplifted path, which is a
hard link of `deps/<name>-<hash>`; the inode gives its hash.

Usage: device_sweep_target.py TARGET_DIR MESSAGES_JSONL
"""
import json
import os
import re
import shutil
import sys

HASHED = re.compile(r'^(.+)-([0-9a-f]{16})(\..+)?$')


def unit_hash(name):
  match = HASHED.match(name)
  return match.group(2) if match else None


def profile_of(path):
  """The profile directory (e.g. target/<triple>/release) of an output path."""
  parts = path.split(os.sep)
  for marker in ('deps', 'build', '.fingerprint'):
    if marker in parts:
      return os.sep.join(parts[:parts.index(marker)])
  return os.path.dirname(path)


def size_of(path):
  if os.path.islink(path) or not os.path.isdir(path):
    try:
      return os.lstat(path).st_size
    except OSError:
      return 0
  total = 0
  for directory, _, files in os.walk(path):
    for name in files:
      try:
        total += os.lstat(os.path.join(directory, name)).st_size
      except OSError:
        pass
  return total


class Sweep:
  def __init__(self):
    self.freed = 0
    self.removed = 0

  def remove(self, path):
    self.freed += size_of(path)
    self.removed += 1
    if os.path.isdir(path) and not os.path.islink(path):
      shutil.rmtree(path, ignore_errors=True)
    else:
      try:
        os.unlink(path)
      except OSError:
        pass


def read_messages(path):
  keep, uplifted, profiles, units = set(), set(), set(), 0
  inodes = {}
  with open(path, errors='replace') as messages:
    for line in messages:
      line = line.strip()
      if not line.startswith('{'):
        continue
      try:
        message = json.loads(line)
      except ValueError:
        continue
      reason = message.get('reason')
      if reason == 'build-script-executed':
        out_dir = message.get('out_dir') or ''
        found = unit_hash(os.path.basename(os.path.dirname(out_dir)))
        if found:
          keep.add(found)
          profiles.add(profile_of(out_dir))
        continue
      if reason != 'compiler-artifact':
        continue
      units += 1
      paths = list(message.get('filenames') or [])
      if message.get('executable'):
        paths.append(message['executable'])
      for path in paths:
        profiles.add(profile_of(path))
        hashes = [found for found in map(unit_hash, path.split(os.sep)) if found]
        if hashes:
          keep.update(hashes)
          continue
        # An uplifted output: keep it and find the unit it links to.
        uplifted.add(path)
        deps = os.path.join(os.path.dirname(path), 'deps')
        if deps not in inodes:
          table = {}
          try:
            for entry in os.scandir(deps):
              if entry.is_file(follow_symlinks=False):
                table.setdefault(entry.inode(), []).append(entry.name)
          except OSError:
            pass
          inodes[deps] = table
        try:
          inode = os.stat(path).st_ino
        except OSError:
          continue
        for name in inodes[deps].get(inode, []):
          found = unit_hash(name)
          if found:
            keep.add(found)
  return keep, uplifted, profiles, units


def sweep_profile(profile, keep, uplifted, sweep):
  for sub in ('deps', '.fingerprint', 'build'):
    directory = os.path.join(profile, sub)
    try:
      entries = list(os.scandir(directory))
    except OSError:
      continue
    for entry in entries:
      found = unit_hash(entry.name)
      if found and found not in keep:
        sweep.remove(entry.path)
  for sub in ('incremental', 'examples'):
    path = os.path.join(profile, sub)
    if os.path.isdir(path):
      sweep.remove(path)
  wanted = set(uplifted) | {path + '.d' for path in uplifted}
  for entry in os.scandir(profile):
    if entry.is_file(follow_symlinks=False) and not entry.name.startswith('.') \
       and entry.path not in wanted:
      sweep.remove(entry.path)


def sweep_unused_dirs(target, profiles, sweep):
  """Directories of other profiles or targets (e.g. an old debug build)."""
  needed = set()
  for profile in profiles:
    path = profile
    while len(path) > len(target):
      needed.add(path)
      path = os.path.dirname(path)
  for directory in sorted(needed | {target}):
    for entry in os.scandir(directory):
      if entry.is_dir(follow_symlinks=False) and entry.path not in needed \
         and directory not in profiles:
        sweep.remove(entry.path)


def main(argv):
  if len(argv) != 3:
    print('Usage: device_sweep_target.py TARGET_DIR MESSAGES_JSONL')
    return 2
  target = os.path.realpath(argv[1])
  keep, uplifted, profiles, units = read_messages(argv[2])
  profiles = {os.path.realpath(p) for p in profiles if p}
  inside = all(p.startswith(target + os.sep) for p in profiles)
  if units == 0 or not keep or not profiles or not inside:
    print(f'target sweep skipped: {units} units, {len(keep)} hashes, profiles {sorted(profiles)}')
    return 0
  uplifted = {os.path.realpath(p) for p in uplifted}
  sweep = Sweep()
  for profile in profiles:
    sweep_profile(profile, keep, uplifted, sweep)
  sweep_unused_dirs(target, profiles, sweep)
  print(f'target sweep: kept {len(keep)} units of {units} outputs, '
        f'removed {sweep.removed} entries, {sweep.freed / 2**20:.0f} MB')
  return 0


if __name__ == '__main__':
  sys.exit(main(sys.argv))
