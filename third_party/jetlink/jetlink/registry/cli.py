"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of jetlink and is licensed under the MIT License.
See the LICENSE file in the root directory for more details.

`jetlink-models`: the registry from a terminal, on every platform.

    jetlink-models list                     what large models exist
    jetlink-models fetch <ref>              download one ahead of a drive
    jetlink-models prepare <ref>            download it and build an engine
    jetlink-models inventory                what this cache holds
    jetlink-models rm <sha256> --model      get the disk back

Progress goes to stderr, one line per whole percent, so stdout stays a value a
script can read. Exit codes: 0 ok, 1 usage or not found, 2 network, 3 the bytes
did not verify.
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

from jetlink.registry import NetworkError, Registry, RegistryError, VerifyError, is_ref, is_sha256
from jetlink.server import platform

MB = 1 << 20


def main(argv=None) -> int:
  # --cache is on every subcommand rather than the top level: argparse copies a
  # subparser's defaults over the outer namespace, so one flag in both places
  # would silently lose the outer value.
  common = argparse.ArgumentParser(add_help=False)
  common.add_argument('--cache', default=str(platform.default_cache_dir()), help='cache root (default: %(default)s)')

  parser = argparse.ArgumentParser(prog='jetlink-models', description="Large driving models: what exists, what is here, and how to get it")
  sub = parser.add_subparsers(dest='command', required=True)

  p = sub.add_parser('list', parents=[common], help='the big-model catalog')
  p.add_argument('--refresh', action='store_true', help='refetch the catalog and resolve missing pointers')
  p.add_argument('--json', action='store_true')
  p.set_defaults(run=_cmd_list)

  p = sub.add_parser('resolve', parents=[common], help='the sha256 and size behind a catalog ref')
  p.add_argument('ref')
  p.add_argument('--json', action='store_true')
  p.set_defaults(run=_cmd_resolve)

  p = sub.add_parser('fetch', parents=[common], help='download a model into the cache')
  p.add_argument('ref_or_sha256', metavar='REF_OR_SHA256')
  p.set_defaults(run=_cmd_fetch)

  p = sub.add_parser('import', parents=[common], help='take a model from disk into the cache')
  p.add_argument('path')
  p.add_argument('--name')
  p.set_defaults(run=_cmd_import)

  p = sub.add_parser('inventory', parents=[common], help='what this cache holds')
  p.add_argument('--json', action='store_true')
  p.set_defaults(run=_cmd_inventory)

  p = sub.add_parser('rm', parents=[common], help='delete a model, its engines, or both')
  p.add_argument('sha256')
  p.add_argument('--artifacts', action='store_true', help='delete every engine built from it')
  p.add_argument('--model', action='store_true', help='delete the downloaded onnx')
  p.set_defaults(run=_cmd_rm)

  p = sub.add_parser('prepare', parents=[common], help='fetch if needed, then build an engine')
  p.add_argument('ref_or_sha256', metavar='REF_OR_SHA256')
  p.add_argument('--backend', default='auto')
  p.add_argument('--device', default='auto')
  p.set_defaults(run=_cmd_prepare)

  args = parser.parse_args(argv)
  root = Path(args.cache)
  try:
    return args.run(args, Registry(root), root)
  except VerifyError as e:
    print(f"jetlink-models: {e}", file=sys.stderr)
    return 3
  except NetworkError as e:
    print(f"jetlink-models: {e}", file=sys.stderr)
    return 2
  except (RegistryError, OSError, ValueError) as e:
    print(f"jetlink-models: {e}", file=sys.stderr)
    return 1


# --- commands ----------------------------------------------------------------

def _cmd_list(args, registry: Registry, root: Path) -> int:
  payload = registry.catalog(refresh=args.refresh)
  if args.refresh:
    missing = [m['ref'] for m in payload['models'] if not m['sha256']]
    if missing:
      for ref, result in registry.resolve_missing(missing).items():
        if isinstance(result, Exception):
          print(f"could not resolve {ref[:10]}: {result}", file=sys.stderr)
    payload = registry.catalog()   # the cache is fresh now, so this touches no network
  if args.json:
    print(json.dumps(payload))
    return 0

  if payload['error']:
    print(f"catalog refresh failed: {payload['error']}", file=sys.stderr)
  state = _states(registry)
  print(f"{'#':>3}  {'name':<44} {'ref':<10} {'built':<10} {'size':>7}  state")
  for model in payload['models']:
    size = f"{model['bytes'] // MB} MB" if model['bytes'] else '?'
    print(f"{model['index']:>3}  {model['name'][:44]:<44} {model['ref'][:10]:<10} {model['build_time'][:10]:<10} {size:>7}  "
          f"{state.get(model['sha256'] or '', '-')}")
  return 0


def _cmd_resolve(args, registry: Registry, root: Path) -> int:
  if not is_ref(args.ref):
    print(f"jetlink-models: {args.ref!r} is not a 40 character commit", file=sys.stderr)
    return 1
  pointer = registry.resolve(args.ref)
  if args.json:
    print(json.dumps({'ref': args.ref, 'sha256': pointer.oid, 'bytes': pointer.size}))
  else:
    print(f"{pointer.oid} {pointer.size}")
  return 0


def _cmd_fetch(args, registry: Registry, root: Path) -> int:
  total = _size_hint(registry, args.ref_or_sha256)
  path = registry.fetch(args.ref_or_sha256, progress=_stderr_progress(total))
  print('', file=sys.stderr)
  print(path)
  return 0


def _cmd_import(args, registry: Registry, root: Path) -> int:
  path = Path(args.path)
  total = path.stat().st_size if path.is_file() else 0
  local = registry.import_model(path, name=args.name, progress=_stderr_progress(total))
  print('', file=sys.stderr)
  print(f"{local.sha256} {registry.model_path(local.sha256)}")
  return 0


def _cmd_inventory(args, registry: Registry, root: Path) -> int:
  payload = registry.inventory(_cache_for(root))
  if args.json:
    print(json.dumps(payload))
    return 0
  print('models')
  for model in payload['models']:
    print(f"  {model['sha256'][:16]}  {model['bytes'] // MB:>6} MB  {model['name'] or '(unknown)'}")
  print('engines')
  for artifact in payload['artifacts']:
    current = ' *' if artifact['current'] else '  '
    print(f"  {artifact['sha256'][:16]}{current} {artifact['bytes'] // MB:>6} MB  {artifact['backend']:<8} {artifact['device']}")
  disk = payload['disk']
  print(f"disk: models {disk['models_bytes'] // MB} MB, engines {disk['engines_bytes'] // MB} MB, free {disk['free_bytes'] // MB} MB")
  if payload['last_loaded']:
    print(f"last loaded: {payload['last_loaded'][:16]}")
  return 0


def _cmd_rm(args, registry: Registry, root: Path) -> int:
  if not (args.artifacts or args.model):
    print('jetlink-models: say what to remove, --artifacts or --model or both', file=sys.stderr)
    return 1
  if not is_sha256(args.sha256):
    print(f"jetlink-models: {args.sha256!r} is not a 64 character sha256", file=sys.stderr)
    return 1
  registry.remove(args.sha256, artifacts=args.artifacts, model=args.model)
  removed = ' and '.join(part for part, wanted in (('engines', args.artifacts), ('the model', args.model)) if wanted)
  print(f"removed {removed} for {args.sha256[:16]}")
  return 0


def _cmd_prepare(args, registry: Registry, root: Path) -> int:
  print('building outside the server; stop any running jetlink-server that uses this cache first', file=sys.stderr)
  total = _size_hint(registry, args.ref_or_sha256)
  path = registry.fetch(args.ref_or_sha256, progress=_stderr_progress(total))
  print('', file=sys.stderr)
  from jetlink.server.main import main as server_main
  return server_main(['--backend', args.backend, '--device', args.device, '--cache', str(root), '--build', str(path)])


# --- helpers -----------------------------------------------------------------

def _cache_for(root: Path):
  """An EngineCache for `current`, without choosing a backend until it is asked."""
  from jetlink.server.cache import EngineCache
  return EngineCache(root)


def _states(registry: Registry) -> dict[str, str]:
  """sha256 to `downloaded` / `prepared`, for the list table."""
  payload = registry.inventory(_cache_for(registry.root))
  out = {model['sha256']: 'downloaded' for model in payload['models']}
  for artifact in payload['artifacts']:
    out[artifact['sha256']] = 'prepared'
  return out


def _size_hint(registry: Registry, ref_or_sha256: str) -> int:
  """The download's size, for the progress line, without failing if it is unknown."""
  try:
    if is_ref(ref_or_sha256):
      return registry.resolve(ref_or_sha256).size
    return registry._pointer_for(ref_or_sha256).size
  except RegistryError:
    return 0


def _stderr_progress(total: int):
  def report(frac: float) -> None:
    done = int(frac * total)
    print(f"\r{int(frac * 100):>3}% {done // MB}/{total // MB} MB", end='', file=sys.stderr, flush=True)
  return report
