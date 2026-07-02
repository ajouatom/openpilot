#!/usr/bin/env python3
import os
import time
import argparse
import numpy as np
import av

import openpilot.cereal.messaging as messaging

# openpilot encoderd EncodeData uses this flag for keyframe
V4L2_BUF_FLAG_KEYFRAME = 8

def ensure_dir(p):
  os.makedirs(p, exist_ok=True)

def main():
  parser = argparse.ArgumentParser()
  parser.add_argument("addr", help="C3X IP (where bridge.cc is publishing roadEncodeData)")
  parser.add_argument("--model", default="yolo11n.pt", help="yolo11n.pt or yolo11s.pt etc")
  parser.add_argument("--conf", type=float, default=0.25)
  parser.add_argument("--imgsz", type=int, default=640)
  parser.add_argument("--device", default="0", help="0 for cuda:0 (if torch sees it), or 'cpu'")
  parser.add_argument("--skip", type=int, default=2, help="run YOLO every N frames (reduce load)")
  parser.add_argument("--conflate", action="store_true", help="ZMQ conflate (latest-only) - good for realtime")
  parser.add_argument("--print_every", type=int, default=1, help="print every N yolo runs")

  # NEW: dump frames for verification
  parser.add_argument("--dump_dir", default="/tmp/orin_dump", help="where to save jpg frames")
  parser.add_argument("--dump_every", type=int, default=30, help="save a raw decoded frame every N decoded frames (0 disables)")
  parser.add_argument("--dump_max", type=int, default=50, help="max number of saved raw frames")
  parser.add_argument("--dump_yolo", action="store_true", help="also save YOLO overlay frames when YOLO runs")
  parser.add_argument("--dump_first", action="store_true", help="save the first decoded frame immediately (recommended)")

  args = parser.parse_args()

  # OpenCV only for saving jpg + overlay text/boxes
  import cv2

  # Ultralytics YOLO
  from ultralytics import YOLO
  model = YOLO(args.model)

  # Subscribe to roadEncodeData from C3X via ZMQ
  os.environ["ZMQ"] = "1"
  messaging.reset_context()
  sock = messaging.sub_sock("roadEncodeData", None, addr=args.addr, conflate=args.conflate)

  # HEVC decoder (PyAV/FFmpeg)
  codec = av.CodecContext.create("hevc", "r")

  seen_iframe = False
  last_encode_id = -1
  frame_cnt = 0
  yolo_run_cnt = 0

  ensure_dir(args.dump_dir)
  raw_dump_cnt = 0
  yolo_dump_cnt = 0
  first_dump_done = False

  print("[orin] subscribing roadEncodeData from", args.addr)
  print("[orin] model:", args.model, "conf:", args.conf, "imgsz:", args.imgsz, "device:", args.device)
  print("[orin] dump_dir:", args.dump_dir, "dump_every:", args.dump_every, "dump_max:", args.dump_max,
        "dump_yolo:", args.dump_yolo, "dump_first:", args.dump_first)

  def save_jpg(path, img_bgr, quality=90):
    ok = cv2.imwrite(path, img_bgr, [int(cv2.IMWRITE_JPEG_QUALITY), int(quality)])
    if not ok:
      print("[orin] WARN: failed to write", path)
    return ok

  def draw_boxes(img_bgr, r):
    # r is results[0]
    boxes = r.boxes
    if boxes is None or len(boxes) == 0:
      return 0
    names = r.names
    xyxy = boxes.xyxy.cpu().numpy()
    confs = boxes.conf.cpu().numpy()
    clss = boxes.cls.cpu().numpy().astype(int)

    n = 0
    for (x1, y1, x2, y2), cf, ci in zip(xyxy, confs, clss):
      x1i, y1i, x2i, y2i = int(x1), int(y1), int(x2), int(y2)
      cls_name = names.get(ci, str(ci)) if isinstance(names, dict) else str(ci)
      label = f"{cls_name} {cf:.2f}"

      cv2.rectangle(img_bgr, (x1i, y1i), (x2i, y2i), (0, 255, 0), 2)
      cv2.putText(img_bgr, label, (x1i, max(0, y1i - 5)),
                  cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2, cv2.LINE_AA)
      n += 1
    return n

  while True:
    msgs = messaging.drain_sock(sock, wait_for_one=True)
    for evt in msgs:
      evta = getattr(evt, evt.which())  # EncodeData

      encode_id = int(evta.idx.encodeId)
      if last_encode_id != -1 and encode_id != last_encode_id + 1:
        print(f"[orin] DROP? encodeId {last_encode_id} -> {encode_id}")
      last_encode_id = encode_id

      # need first keyframe to init decoder correctly
      if not seen_iframe:
        if not (evta.idx.flags & V4L2_BUF_FLAG_KEYFRAME):
          continue
        try:
          codec.decode(av.packet.Packet(evta.header))
        except Exception as e:
          print("[orin] header decode error:", e)
          continue
        seen_iframe = True
        print("[orin] got first iframe/header, start decoding...")

      # decode this frame
      try:
        frames = codec.decode(av.packet.Packet(evta.data))
      except Exception as e:
        print("[orin] decode error:", e)
        continue
      if len(frames) < 1:
        continue

      frame = frames[0]
      img_bgr = frame.to_ndarray(format="bgr24")
      frame_cnt += 1

      # dump first frame immediately (for sanity)
      if args.dump_first and (not first_dump_done):
        ts = int(time.time() * 1000)
        path = os.path.join(args.dump_dir, f"raw_first_eid{encode_id}_t{ts}.jpg")
        save_jpg(path, img_bgr)
        first_dump_done = True
        print("[orin] saved first frame:", path, "shape=", img_bgr.shape)

      # periodic raw dump (decoded frame, no yolo)
      if args.dump_every > 0 and raw_dump_cnt < args.dump_max and (frame_cnt % args.dump_every) == 0:
        ts = int(time.time() * 1000)
        path = os.path.join(args.dump_dir, f"raw_f{frame_cnt:07d}_eid{encode_id}_t{ts}.jpg")
        if save_jpg(path, img_bgr):
          raw_dump_cnt += 1
          print("[orin] saved raw frame:", path, "shape=", img_bgr.shape)

      # YOLO skip
      if args.skip > 1 and (frame_cnt % args.skip) != 0:
        continue

      # YOLO inference
      results = model.predict(
        source=img_bgr,
        imgsz=args.imgsz,
        conf=args.conf,
        device=args.device,
        verbose=False
      )

      yolo_run_cnt += 1
      if args.print_every > 1 and (yolo_run_cnt % args.print_every) != 0:
        continue

      r = results[0]
      boxes = r.boxes
      det_n = 0 if (boxes is None) else len(boxes)

      print(f"[orin] encodeId={encode_id} det={det_n}")

      # print detections
      if boxes is not None and det_n > 0:
        names = r.names
        xyxy = boxes.xyxy.cpu().numpy()
        confs = boxes.conf.cpu().numpy()
        clss = boxes.cls.cpu().numpy().astype(int)
        for (x1, y1, x2, y2), cf, ci in zip(xyxy, confs, clss):
          cls_name = names.get(ci, str(ci)) if isinstance(names, dict) else str(ci)
          print(f"  - {cls_name:>12s} conf={cf:.2f} box=({x1:.1f},{y1:.1f})-({x2:.1f},{y2:.1f})")

      # optional: dump yolo overlay frame
      if args.dump_yolo:
        img_vis = img_bgr.copy()#!/usr/bin/env python3
import os
import time
import argparse
import numpy as np
import av
import cv2

import openpilot.cereal.messaging as messaging

# openpilot encoderd EncodeData uses this flag for keyframe
V4L2_BUF_FLAG_KEYFRAME = 8


def main():
  parser = argparse.ArgumentParser()
  parser.add_argument("addr", help="C3X IP (where bridge.cc is publishing roadEncodeData)")
  parser.add_argument("--model", default="yolo11n.pt", help="yolo11n.pt or yolo11s.pt etc")
  parser.add_argument("--conf", type=float, default=0.25)
  parser.add_argument("--imgsz", type=int, default=640)
  parser.add_argument("--device", default="0", help="0 for cuda:0 (if torch sees it), or 'cpu'")
  parser.add_argument("--skip", type=int, default=2, help="run YOLO every N frames (reduce load)")
  parser.add_argument("--conflate", action="store_true", help="ZMQ conflate (latest-only) - good for realtime")
  parser.add_argument("--print_every", type=int, default=1, help="print every N yolo runs")

  # Save annotated images (for verification)
  parser.add_argument("--save_dir", default="/tmp/orin_yolo", help="save annotated jpg here")
  parser.add_argument("--save_every", type=int, default=1, help="save every N yolo runs")
  parser.add_argument("--save_max", type=int, default=200, help="max saved images")
  parser.add_argument("--save_raw_first", action="store_true", help="save first decoded raw frame too")
  parser.add_argument("--jpg_quality", type=int, default=90, help="jpeg quality 0~100")

  args = parser.parse_args()

  os.makedirs(args.save_dir, exist_ok=True)
  save_cnt = 0
  raw_first_saved = False

  # Ultralytics YOLO
  from ultralytics import YOLO
  model = YOLO(args.model)

  # Subscribe to roadEncodeData from C3X via ZMQ
  os.environ["ZMQ"] = "1"
  messaging.reset_context()
  sock = messaging.sub_sock("roadEncodeData", None, addr=args.addr, conflate=args.conflate)

  # HEVC decoder (PyAV/FFmpeg)
  codec = av.CodecContext.create("hevc", "r")

  seen_iframe = False
  last_encode_id = -1
  frame_cnt = 0
  yolo_run_cnt = 0

  print("[orin] subscribing roadEncodeData from", args.addr)
  print("[orin] model:", args.model, "conf:", args.conf, "imgsz:", args.imgsz, "device:", args.device)
  print("[orin] save_dir:", args.save_dir, "save_every:", args.save_every, "save_max:", args.save_max)

  def save_jpg(path, img_bgr):
    ok = cv2.imwrite(path, img_bgr, [int(cv2.IMWRITE_JPEG_QUALITY), int(args.jpg_quality)])
    if not ok:
      print("[orin] WARN: failed to write", path)
    return ok

  def draw_and_save(img_bgr, r, encode_id, frame_cnt, infer_ms):
    nonlocal save_cnt
    if save_cnt >= args.save_max:
      return

    img = img_bgr.copy()

    det_n = 0 if (r.boxes is None) else len(r.boxes)
    info = f"eid={encode_id} frame={frame_cnt} det={det_n} infer={infer_ms:.1f}ms imgsz={args.imgsz}"
    cv2.putText(img, info, (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2, cv2.LINE_AA)

    # Draw boxes
    if r.boxes is not None and det_n > 0:
      names = r.names
      xyxy = r.boxes.xyxy.cpu().numpy()
      confs = r.boxes.conf.cpu().numpy()
      clss = r.boxes.cls.cpu().numpy().astype(int)

      for (x1, y1, x2, y2), cf, ci in zip(xyxy, confs, clss):
        x1i, y1i, x2i, y2i = int(x1), int(y1), int(x2), int(y2)
        cls_name = names.get(ci, str(ci)) if isinstance(names, dict) else str(ci)
        label = f"{cls_name} {cf:.2f}"

        cv2.rectangle(img, (x1i, y1i), (x2i, y2i), (0, 255, 0), 2)
        cv2.putText(img, label, (x1i, max(0, y1i - 6)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2, cv2.LINE_AA)

    path = os.path.join(args.save_dir, f"yolo_eid{encode_id}_f{frame_cnt:07d}_d{det_n}.jpg")
    if save_jpg(path, img):
      save_cnt += 1
      print("[orin] saved:", path)

  while True:
    # wait_for_one=True blocks until at least one message arrives
    msgs = messaging.drain_sock(sock, wait_for_one=True)
    for evt in msgs:
      evta = getattr(evt, evt.which())  # EncodeData

      encode_id = int(evta.idx.encodeId)
      if last_encode_id != -1 and encode_id != last_encode_id + 1:
        print(f"[orin] DROP? encodeId {last_encode_id} -> {encode_id}")
      last_encode_id = encode_id

      # need first keyframe to init decoder correctly
      if not seen_iframe:
        if not (evta.idx.flags & V4L2_BUF_FLAG_KEYFRAME):
          continue
        # feed header once
        try:
          codec.decode(av.packet.Packet(evta.header))
        except Exception as e:
          print("[orin] header decode error:", e)
          continue
        seen_iframe = True
        print("[orin] got first iframe/header, start decoding...")

      # decode this frame
      try:
        frames = codec.decode(av.packet.Packet(evta.data))
      except Exception as e:
        print("[orin] decode error:", e)
        continue

      if len(frames) < 1:
        continue

      frame = frames[0]
      img_bgr = frame.to_ndarray(format="bgr24")
      frame_cnt += 1

      # Optional: save first raw decoded frame
      if args.save_raw_first and not raw_first_saved:
        raw_path = os.path.join(args.save_dir, f"raw_first_eid{encode_id}_f{frame_cnt:07d}.jpg")
        if save_jpg(raw_path, img_bgr):
          raw_first_saved = True
          print("[orin] saved raw first:", raw_path, "shape=", img_bgr.shape)

      # Skip inference frames
      if args.skip > 1 and (frame_cnt % args.skip) != 0:
        continue

      # YOLO inference timing
      t0 = time.perf_counter()
      results = model.predict(
        source=img_bgr,
        imgsz=args.imgsz,
        conf=args.conf,
        device=args.device,
        verbose=False
      )
      infer_ms = (time.perf_counter() - t0) * 1000.0

      yolo_run_cnt += 1

      # Print every N yolo runs
      if args.print_every <= 1 or (yolo_run_cnt % args.print_every) == 0:
        r = results[0]
        boxes = r.boxes
        det_n = 0 if (boxes is None) else len(boxes)
        print(f"[orin] encodeId={encode_id} det={det_n} infer={infer_ms:.1f}ms")

        if boxes is not None and det_n > 0:
          names = r.names
          xyxy = boxes.xyxy.cpu().numpy()
          confs = boxes.conf.cpu().numpy()
          clss = boxes.cls.cpu().numpy().astype(int)
          for (x1, y1, x2, y2), cf, ci in zip(xyxy, confs, clss):
            cls_name = names.get(ci, str(ci)) if isinstance(names, dict) else str(ci)
            print(f"  - {cls_name:>12s} conf={cf:.2f} box=({x1:.1f},{y1:.1f})-({x2:.1f},{y2:.1f})")

      # Save annotated image every N yolo runs
      if args.save_every > 0 and (yolo_run_cnt % args.save_every) == 0:
        r = results[0]
        draw_and_save(img_bgr, r, encode_id, frame_cnt, infer_ms)

      # Stop saving if reached max (still keep running inference/prints)
      if save_cnt >= args.save_max:
        # You can break if you want to stop entirely:
        # return
        pass


if __name__ == "__main__":
  try:
    main()
  except KeyboardInterrupt:
    pass

