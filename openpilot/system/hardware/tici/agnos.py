#!/usr/bin/env python3
import hashlib
import json
import lzma
import os
from pathlib import Path
import struct
import subprocess
import time
from collections.abc import Generator
from urllib.parse import urlsplit

import requests

SPARSE_CHUNK_FMT = struct.Struct('H2xI4x')
CAIBX_URL = "https://commadist.azureedge.net/agnosupdate/"

AGNOS_MANIFEST_FILE = "openpilot/system/hardware/tici/agnos.json"
DOWNLOAD_CACHE_DIR = Path(os.getenv("AGNOS_DOWNLOAD_CACHE_DIR", "/data/agnos-update-cache"))
UPDATE_CONFIRMATION_FILE = Path(os.getenv("AGNOS_UPDATE_CONFIRMATION_FILE", "/data/agnos-update-confirmed"))
UPDATE_LOCK_FILE = Path(os.getenv("AGNOS_UPDATE_LOCK_FILE", "/tmp/agnos-update.lock"))
SWAP_MAX_ATTEMPTS = 5
SWAP_RETRY_DELAY = 1.0
VERIFY_FLASH_MAX_ATTEMPTS = 3
DOWNLOAD_RETRY_ATTEMPTS = 5
DOWNLOAD_RETRY_DELAY = 5.0
DOWNLOAD_REQUEST_TIMEOUT = (10, 60)


def manifest_digest(manifest_path: str | Path) -> str:
  return hashlib.sha256(Path(manifest_path).read_bytes()).hexdigest()


def manifest_download_urls(manifest_path: str | Path) -> tuple[str, ...]:
  update = json.loads(Path(manifest_path).read_text(encoding="utf-8"))
  urls: list[str] = []
  origins: set[tuple[str, str]] = set()
  for partition in update:
    url = partition.get("url")
    if not isinstance(url, str):
      continue
    parsed = urlsplit(url)
    origin = (parsed.scheme, parsed.netloc)
    if origin not in origins:
      origins.add(origin)
      urls.append(url)
  return tuple(urls)


def mark_update_confirmed(manifest_path: str | Path) -> None:
  UPDATE_CONFIRMATION_FILE.parent.mkdir(parents=True, exist_ok=True)
  temporary_path = UPDATE_CONFIRMATION_FILE.with_suffix(UPDATE_CONFIRMATION_FILE.suffix + ".tmp")
  with temporary_path.open("w", encoding="utf-8") as output:
    output.write(manifest_digest(manifest_path) + "\n")
    output.flush()
    os.fsync(output.fileno())
  os.replace(temporary_path, UPDATE_CONFIRMATION_FILE)


def update_confirmed(manifest_path: str | Path) -> bool:
  try:
    return UPDATE_CONFIRMATION_FILE.read_text(encoding="utf-8").strip() == manifest_digest(manifest_path)
  except OSError:
    return False


def clear_update_confirmation() -> None:
  UPDATE_CONFIRMATION_FILE.unlink(missing_ok=True)


def report_progress(stage: str, progress: int) -> None:
  print(f"{stage}: {max(0, min(progress, 100))}", flush=True)


def acquire_update_lock():
  try:
    import fcntl
  except ImportError:
    return None

  UPDATE_LOCK_FILE.parent.mkdir(parents=True, exist_ok=True)
  lock_file = UPDATE_LOCK_FILE.open("w", encoding="utf-8")
  try:
    fcntl.flock(lock_file, fcntl.LOCK_EX | fcntl.LOCK_NB)
  except BlockingIOError:
    lock_file.close()
    raise RuntimeError("Another AGNOS updater is already running") from None
  lock_file.write(f"{os.getpid()}\n")
  lock_file.flush()
  return lock_file


class StreamingDecompressor:
  def __init__(self, url: str, cache_path: Path | None = None) -> None:
    self.buf = b""

    self.source = cache_path.open("rb") if cache_path is not None else None
    self.req = None if self.source is not None else requests.get(
      url, stream=True, headers={'Accept-Encoding': None}, timeout=DOWNLOAD_REQUEST_TIMEOUT,
    )
    self.it = (iter(lambda: self.source.read(1024 * 1024), b"") if self.source is not None
               else self.req.iter_content(chunk_size=1024 * 1024))
    self.decompressor = lzma.LZMADecompressor(format=lzma.FORMAT_AUTO)
    self.eof = False
    self.sha256 = hashlib.sha256()

  def read(self, length: int) -> bytes:
    while len(self.buf) < length and not self.eof:
      if self.decompressor.needs_input:
        if self.req is not None:
          self.req.raise_for_status()

        try:
          compressed = next(self.it)
        except StopIteration:
          self.eof = True
          break
      else:
        compressed = b''

      self.buf += self.decompressor.decompress(compressed, max_length=length)

      if self.decompressor.eof:
        self.eof = True
        break

    result = self.buf[:length]
    self.buf = self.buf[length:]

    self.sha256.update(result)
    return result

  def close(self) -> None:
    if self.source is not None:
      self.source.close()
    if self.req is not None:
      self.req.close()


def file_checksum(path: Path) -> str:
  digest = hashlib.sha256()
  with path.open("rb") as source:
    for chunk in iter(lambda: source.read(1024 * 1024), b""):
      digest.update(chunk)
  return digest.hexdigest()


def download_to_cache(partition: dict, cloudlog) -> Path | None:
  compressed_hash = partition.get("compressed_hash")
  if not isinstance(compressed_hash, str):
    return None

  DOWNLOAD_CACHE_DIR.mkdir(parents=True, exist_ok=True)
  final_path = DOWNLOAD_CACHE_DIR / f"{partition['name']}-{compressed_hash}.img.xz"
  partial_path = final_path.with_suffix(final_path.suffix + ".part")
  if final_path.is_file():
    if file_checksum(final_path).lower() == compressed_hash.lower():
      return final_path
    cloudlog.warning(f"Discarding invalid cached {partition['name']} image")
    final_path.unlink()

  offset = partial_path.stat().st_size if partial_path.is_file() else 0
  expected_size = partition.get("compressed_size")
  if not isinstance(expected_size, int):
    expected_size = None
  if expected_size is not None and offset >= expected_size:
    if offset == expected_size and file_checksum(partial_path).lower() == compressed_hash.lower():
      os.replace(partial_path, final_path)
      return final_path
    cloudlog.warning(f"Discarding invalid partial {partition['name']} image")
    partial_path.unlink()
    offset = 0

  headers: dict[str, str | None] = {'Accept-Encoding': None}
  if offset:
    headers['Range'] = f"bytes={offset}-"

  cloudlog.info(f"Downloading {partition['name']} cache from byte {offset}")
  report_progress(f"Connecting for {partition['name']}", 0)
  response = requests.get(partition['url'], stream=True, headers=headers, timeout=DOWNLOAD_REQUEST_TIMEOUT)
  response.raise_for_status()

  if offset and response.status_code != 206:
    cloudlog.warning(f"Server ignored resume for {partition['name']}; restarting the cached download")
    offset = 0

  if expected_size is None:
    content_length = response.headers.get("Content-Length")
    expected_size = offset + int(content_length) if content_length is not None else None

  mode = "ab" if offset else "wb"
  last_p = -1
  try:
    with partial_path.open(mode) as output:
      for chunk in response.iter_content(chunk_size=1024 * 1024):
        if not chunk:
          continue
        output.write(chunk)
        if expected_size:
          p = int(output.tell() / expected_size * 100)
          if p != last_p:
            last_p = p
            report_progress(f"Downloading {partition['name']}", p)
      output.flush()
      os.fsync(output.fileno())
  finally:
    response.close()

  downloaded_size = partial_path.stat().st_size
  if expected_size is not None and downloaded_size != expected_size:
    raise requests.ConnectionError(
      f"Incomplete {partition['name']} download: {downloaded_size} of {expected_size} bytes"
    )
  actual_hash = file_checksum(partial_path)
  if actual_hash.lower() != compressed_hash.lower():
    partial_path.unlink(missing_ok=True)
    raise requests.ConnectionError(f"Compressed {partition['name']} cache hash mismatch: {actual_hash}")

  os.replace(partial_path, final_path)
  return final_path


def unsparsify(f: StreamingDecompressor) -> Generator[bytes, None, None]:
  # https://source.android.com/devices/bootloader/images#sparse-format
  magic = struct.unpack("I", f.read(4))[0]
  assert(magic == 0xed26ff3a)

  # Version
  major = struct.unpack("H", f.read(2))[0]
  minor = struct.unpack("H", f.read(2))[0]
  assert(major == 1 and minor == 0)

  f.read(2)  # file header size
  f.read(2)  # chunk header size

  block_sz = struct.unpack("I", f.read(4))[0]
  f.read(4)  # total blocks
  num_chunks = struct.unpack("I", f.read(4))[0]
  f.read(4)  # crc checksum

  for _ in range(num_chunks):
    chunk_type, out_blocks = SPARSE_CHUNK_FMT.unpack(f.read(12))

    if chunk_type == 0xcac1:  # Raw
      # TODO: yield in smaller chunks. Yielding only block_sz is too slow. Largest observed data chunk is 252 MB.
      yield f.read(out_blocks * block_sz)
    elif chunk_type == 0xcac2:  # Fill
      filler = f.read(4) * (block_sz // 4)
      for _ in range(out_blocks):
        yield filler
    elif chunk_type == 0xcac3:  # Don't care
      yield b""
    else:
      raise Exception("Unhandled sparse chunk type")


# noop wrapper with same API as unsparsify() for non sparse images
def noop(f: StreamingDecompressor) -> Generator[bytes, None, None]:
  while len(chunk := f.read(1024 * 1024)) > 0:
    yield chunk


def get_target_slot_number() -> int:
  current_slot = subprocess.check_output(["abctl", "--boot_slot"], encoding='utf-8').strip()
  return 1 if current_slot == "_a" else 0


def slot_number_to_suffix(slot_number: int) -> str:
  assert slot_number in (0, 1)
  return '_a' if slot_number == 0 else '_b'


def get_partition_path(target_slot_number: int, partition: dict) -> str:
  path = f"/dev/disk/by-partlabel/{partition['name']}"

  if partition.get('has_ab', True):
    path += slot_number_to_suffix(target_slot_number)

  return path


def get_raw_hash(path: str, partition_size: int) -> str:
  raw_hash = hashlib.sha256()
  pos, chunk_size = 0, 1024 * 1024

  with open(path, 'rb+') as out:
    while pos < partition_size:
      n = min(chunk_size, partition_size - pos)
      raw_hash.update(out.read(n))
      pos += n

  return raw_hash.hexdigest().lower()


def verify_partition(target_slot_number: int, partition: dict[str, str | int], force_full_check: bool = False) -> bool:
  full_check = partition['full_check'] or force_full_check
  path = get_partition_path(target_slot_number, partition)

  if not isinstance(partition['size'], int):
    return False

  partition_size: int = partition['size']

  if not isinstance(partition['hash_raw'], str):
    return False

  partition_hash: str = partition['hash_raw']

  if full_check:
    return get_raw_hash(path, partition_size) == partition_hash.lower()
  else:
    with open(path, 'rb+') as out:
      out.seek(partition_size)
      return out.read(64) == partition_hash.lower().encode()


def clear_partition_hash(target_slot_number: int, partition: dict) -> None:
  path = get_partition_path(target_slot_number, partition)
  with open(path, 'wb+') as out:
    partition_size = partition['size']

    out.seek(partition_size)
    out.write(b"\x00" * 64)
    os.sync()


def extract_compressed_image(target_slot_number: int, partition: dict, cloudlog):
  path = get_partition_path(target_slot_number, partition)
  cache_path = download_to_cache(partition, cloudlog)
  downloader = StreamingDecompressor(partition['url'], cache_path)

  try:
    with open(path, 'wb+') as out:
      # Flash partition
      last_p = 0
      raw_hash = hashlib.sha256()
      f = unsparsify if partition['sparse'] else noop
      for chunk in f(downloader):
        raw_hash.update(chunk)
        out.write(chunk)
        p = int(out.tell() / partition['size'] * 100)
        if p != last_p:
          last_p = p
          report_progress(f"Installing {partition['name']}", p)

      if raw_hash.hexdigest().lower() != partition['hash_raw'].lower():
        raise Exception(f"Raw hash mismatch '{raw_hash.hexdigest().lower()}'")

      if downloader.sha256.hexdigest().lower() != partition['hash'].lower():
        raise Exception("Uncompressed hash mismatch")

      if out.tell() != partition['size']:
        raise Exception("Uncompressed size mismatch")

      os.sync()
  finally:
    downloader.close()

  if cache_path is not None:
    cache_path.unlink(missing_ok=True)


def extract_casync_image(target_slot_number: int, partition: dict, cloudlog):
  # The standalone updater does not use casync. Import it only for this path
  # so recovery/standalone flashing does not require pycryptodome.
  import openpilot.system.updated.casync.casync as casync

  path = get_partition_path(target_slot_number, partition)
  seed_path = path[:-1] + ('b' if path[-1] == 'a' else 'a')

  target = casync.parse_caibx(partition['casync_caibx'])

  sources: list[tuple[str, casync.ChunkReader, casync.ChunkDict]] = []

  # First source is the current partition.
  try:
    raw_hash = get_raw_hash(seed_path, partition['size'])
    caibx_url = f"{CAIBX_URL}{partition['name']}-{raw_hash}.caibx"

    try:
      cloudlog.info(f"casync fetching {caibx_url}")
      sources += [('seed', casync.FileChunkReader(seed_path), casync.build_chunk_dict(casync.parse_caibx(caibx_url)))]
    except requests.RequestException:
      cloudlog.error(f"casync failed to load {caibx_url}")
  except Exception:
    cloudlog.exception("casync failed to hash seed partition")

  # Second source is the target partition, this allows for resuming
  sources += [('target', casync.FileChunkReader(path), casync.build_chunk_dict(target))]

  # Finally we add the remote source to download any missing chunks
  sources += [('remote', casync.RemoteChunkReader(partition['casync_store']), casync.build_chunk_dict(target))]

  last_p = 0

  def progress(cur):
    nonlocal last_p
    p = int(cur / partition['size'] * 100)
    if p != last_p:
      last_p = p
      report_progress(f"Installing {partition['name']}", p)

  stats = casync.extract(target, sources, path, progress)
  cloudlog.error(f'casync done {json.dumps(stats)}')

  os.sync()
  if not verify_partition(target_slot_number, partition, force_full_check=True):
    raise Exception(f"Raw hash mismatch '{partition['hash_raw'].lower()}'")


def flash_partition(target_slot_number: int, partition: dict, cloudlog, standalone=False):
  cloudlog.info(f"Downloading and writing {partition['name']}")
  report_progress(f"Checking {partition['name']}", 0)

  if verify_partition(target_slot_number, partition):
    cloudlog.info(f"Already flashed {partition['name']}")
    return

  # Clear hash before flashing in case we get interrupted
  full_check = partition['full_check']
  if not full_check:
    clear_partition_hash(target_slot_number, partition)

  path = get_partition_path(target_slot_number, partition)

  if ('casync_caibx' in partition) and not standalone:
    extract_casync_image(target_slot_number, partition, cloudlog)
  else:
    extract_compressed_image(target_slot_number, partition, cloudlog)

  # Write hash after successful flash
  if not full_check:
    with open(path, 'wb+') as out:
      out.seek(partition['size'])
      out.write(partition['hash_raw'].lower().encode())


def swap(manifest_path: str, target_slot_number: int, cloudlog) -> None:
  update = json.load(open(manifest_path))
  for partition in update:
    if not partition.get('full_check', False):
      clear_partition_hash(target_slot_number, partition)

  last_output = ""
  for attempt in range(1, SWAP_MAX_ATTEMPTS + 1):
    report_progress(f"Switching boot slot {attempt}/{SWAP_MAX_ATTEMPTS}", 100)
    try:
      last_output = subprocess.check_output(
        ["abctl", "--set_active", str(target_slot_number)],
        stderr=subprocess.STDOUT,
        encoding='utf8',
      )
    except subprocess.CalledProcessError as e:
      last_output = e.output or str(e)

    if ("No such file or directory" not in last_output) and ("lun as boot lun" in last_output):
      cloudlog.info(f"Swap successful {last_output}")
      return

    cloudlog.error(f"Swap failed ({attempt}/{SWAP_MAX_ATTEMPTS}): {last_output}")
    if attempt < SWAP_MAX_ATTEMPTS:
      time.sleep(SWAP_RETRY_DELAY)

  raise RuntimeError(f"Failed to switch boot slot after {SWAP_MAX_ATTEMPTS} attempts: {last_output.strip()}")


def flash_agnos_update(manifest_path: str, target_slot_number: int, cloudlog, standalone=False) -> None:
  update = json.load(open(manifest_path))

  cloudlog.info(f"Target slot {target_slot_number}")

  # set target slot as unbootable
  os.system(f"abctl --set_unbootable {target_slot_number}")

  for partition in update:
    success = False

    for retries in range(DOWNLOAD_RETRY_ATTEMPTS):
      try:
        flash_partition(target_slot_number, partition, cloudlog, standalone)
        success = True
        break

      except requests.exceptions.RequestException as e:
        cloudlog.exception("Failed")
        retry_number = retries + 1
        cloudlog.info(f"Failed to download {partition['name']}, retrying ({retry_number}/{DOWNLOAD_RETRY_ATTEMPTS})")
        report_progress(f"Network retry {retry_number}/{DOWNLOAD_RETRY_ATTEMPTS}: {type(e).__name__}", 0)
        if retry_number < DOWNLOAD_RETRY_ATTEMPTS:
          time.sleep(DOWNLOAD_RETRY_DELAY)

    if not success:
      cloudlog.info(f"Failed to flash {partition['name']}, aborting")
      raise Exception("Maximum retries exceeded")

  cloudlog.info(f"AGNOS ready on slot {target_slot_number}")


def verify_agnos_update(manifest_path: str, target_slot_number: int) -> bool:
  update = json.load(open(manifest_path))
  return all(verify_partition(target_slot_number, partition) for partition in update)


if __name__ == "__main__":
  import argparse
  import logging

  parser = argparse.ArgumentParser(description="Flash and verify AGNOS update",
                                   formatter_class=argparse.ArgumentDefaultsHelpFormatter)

  parser.add_argument("--verify", action="store_true", help="Verify and perform swap if update ready")
  parser.add_argument("--swap", action="store_true", help="Verify and perform swap, downloads if necessary")
  parser.add_argument("manifest", help="Manifest json")
  args = parser.parse_args()

  logging.basicConfig(level=logging.INFO)

  update_lock = acquire_update_lock()
  target_slot_number = get_target_slot_number()
  if args.verify:
    report_progress("Verifying update", 0)
    if verify_agnos_update(args.manifest, target_slot_number):
      swap(args.manifest, target_slot_number, logging)
      exit(0)
    exit(1)
  elif args.swap:
    for attempt in range(VERIFY_FLASH_MAX_ATTEMPTS + 1):
      report_progress(f"Verifying update {attempt + 1}/{VERIFY_FLASH_MAX_ATTEMPTS + 1}", 0)
      if verify_agnos_update(args.manifest, target_slot_number):
        break
      if attempt >= VERIFY_FLASH_MAX_ATTEMPTS:
        raise RuntimeError(f"AGNOS verification failed after {VERIFY_FLASH_MAX_ATTEMPTS} flash attempts")
      logging.error(f"Verification failed. Flashing AGNOS ({attempt + 1}/{VERIFY_FLASH_MAX_ATTEMPTS})")
      flash_agnos_update(args.manifest, target_slot_number, logging, standalone=True)

    logging.warning(f"Verification succeeded. Swapping to slot {target_slot_number}")
    swap(args.manifest, target_slot_number, logging)
    report_progress("Update complete; rebooting", 100)
  else:
    flash_agnos_update(args.manifest, target_slot_number, logging, standalone=True)
