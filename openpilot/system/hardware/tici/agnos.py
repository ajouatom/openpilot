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

import requests

SPARSE_CHUNK_FMT = struct.Struct('H2xI4x')
CAIBX_URL = "https://commadist.azureedge.net/agnosupdate/"

AGNOS_MANIFEST_FILE = "openpilot/system/hardware/tici/agnos.json"
DOWNLOAD_CACHE_DIR = Path(os.getenv("AGNOS_DOWNLOAD_CACHE_DIR", "/data/agnos-update-cache"))


class StreamingDecompressor:
  def __init__(self, url: str, cache_path: Path | None = None) -> None:
    self.buf = b""

    self.source = cache_path.open("rb") if cache_path is not None else None
    self.req = None if self.source is not None else requests.get(
      url, stream=True, headers={'Accept-Encoding': None}, timeout=60,
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
  response = requests.get(partition['url'], stream=True, headers=headers, timeout=60)
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
            print(f"Downloading {partition['name']}: {p}", flush=True)
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
          print(f"Installing {partition['name']}: {p}", flush=True)

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
      print(f"Installing {partition['name']}: {p}", flush=True)

  stats = casync.extract(target, sources, path, progress)
  cloudlog.error(f'casync done {json.dumps(stats)}')

  os.sync()
  if not verify_partition(target_slot_number, partition, force_full_check=True):
    raise Exception(f"Raw hash mismatch '{partition['hash_raw'].lower()}'")


def flash_partition(target_slot_number: int, partition: dict, cloudlog, standalone=False):
  cloudlog.info(f"Downloading and writing {partition['name']}")

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

  while True:
    out = subprocess.check_output(f"abctl --set_active {target_slot_number}", shell=True, stderr=subprocess.STDOUT, encoding='utf8')
    if ("No such file or directory" not in out) and ("lun as boot lun" in out):
      cloudlog.info(f"Swap successful {out}")
      break
    else:
      cloudlog.error(f"Swap failed {out}")


def flash_agnos_update(manifest_path: str, target_slot_number: int, cloudlog, standalone=False) -> None:
  update = json.load(open(manifest_path))

  cloudlog.info(f"Target slot {target_slot_number}")

  # set target slot as unbootable
  os.system(f"abctl --set_unbootable {target_slot_number}")

  for partition in update:
    success = False

    for retries in range(10):
      try:
        flash_partition(target_slot_number, partition, cloudlog, standalone)
        success = True
        break

      except requests.exceptions.RequestException:
        cloudlog.exception("Failed")
        cloudlog.info(f"Failed to download {partition['name']}, retrying ({retries})")
        time.sleep(10)

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

  target_slot_number = get_target_slot_number()
  if args.verify:
    if verify_agnos_update(args.manifest, target_slot_number):
      swap(args.manifest, target_slot_number, logging)
      exit(0)
    exit(1)
  elif args.swap:
    while not verify_agnos_update(args.manifest, target_slot_number):
      logging.error("Verification failed. Flashing AGNOS")
      flash_agnos_update(args.manifest, target_slot_number, logging, standalone=True)

    logging.warning(f"Verification succeeded. Swapping to slot {target_slot_number}")
    swap(args.manifest, target_slot_number, logging)
  else:
    flash_agnos_update(args.manifest, target_slot_number, logging, standalone=True)
