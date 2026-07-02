#!/usr/bin/env python3
import os, time, argparse
import av
from PIL import Image

import openpilot.cereal.messaging as messaging

SOCK_NAME = "roadEncodeData"

def atomic_write_jpeg(path: str, img: Image.Image, quality: int = 80):
  tmp = path + ".tmp"
  img.save(tmp, format="JPEG", quality=quality, optimize=False)
  os.replace(tmp, path)

def main(addr: str, conflate: bool, dump_path: str, debug: bool, every_n: int):
  os.environ["ZMQ"] = "1"
  messaging.reset_context()
  sock = messaging.sub_sock(SOCK_NAME, None, addr=addr, conflate=conflate)

  codec = av.CodecContext.create("hevc", "r")

  W = H = None
  seen_header = False
  decoded = dropped = 0
  last_stat = time.time()

  while True:
    msgs = messaging.drain_sock(sock, wait_for_one=True)
    if not msgs:
      continue

    for evt in msgs:
      evta = getattr(evt, evt.which())

      if W is None:
        W, H = int(evta.width), int(evta.height)
        if debug:
          print(f"[road] stream size {W}x{H}", flush=True)

      if (not seen_header) and evta.header:
        try:
          codec.decode(av.packet.Packet(evta.header))
          seen_header = True
          if debug:
            print("[road] header decoded", flush=True)
        except Exception as e:
          if debug:
            print("[road] header decode error:", e, flush=True)

      try:
        frames = codec.decode(av.packet.Packet(evta.data))
      except Exception:
        dropped += 1
        continue

      if not frames:
        dropped += 1
        continue

      decoded += 1
      if dump_path and (decoded % every_n == 0):
        f = frames[0]
        # PyAV -> RGB numpy -> PIL
        rgb = f.to_ndarray(format="rgb24")
        img = Image.fromarray(rgb)
        atomic_write_jpeg(dump_path, img, quality=80)

    now = time.time()
    if debug and now - last_stat > 1.0:
      print(f"[road] decoded={decoded} dropped={dropped} batch={len(msgs)}", flush=True)
      last_stat = now

if __name__ == "__main__":
  ap = argparse.ArgumentParser()
  ap.add_argument("addr")
  ap.add_argument("--conflate", action="store_true")
  ap.add_argument("--dump", default="/tmp/road.jpg")
  ap.add_argument("--debug", action="store_true")
  ap.add_argument("--every", type=int, default=3, help="write every N decoded frames")
  args = ap.parse_args()

  main(args.addr, args.conflate, args.dump, args.debug, args.every)
