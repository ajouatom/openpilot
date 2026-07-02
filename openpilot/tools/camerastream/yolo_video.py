#!/usr/bin/env python3
import argparse
import time
import cv2
from ultralytics import YOLO


def is_traffic_light(det_name: str) -> bool:
  # 데이터셋에 따라 라벨명이 다를 수 있어서 넉넉하게 잡습니다.
  # 예: "traffic light", "trafficlight", "tl", "signal" 등
  n = det_name.lower().replace("_", " ").strip()
  return ("traffic" in n and "light" in n) or ("trafficlight" in n) or (n in ["tl", "signal", "signal light"])


def main():
  parser = argparse.ArgumentParser()
  parser.add_argument("--model", default="best.pt", help="YOLO .pt path")
  parser.add_argument("--source", default=0,
                      help="video path OR camera index (0,1,...) OR rtsp url")
  parser.add_argument("--imgsz", type=int, default=640)
  parser.add_argument("--conf", type=float, default=0.25)
  parser.add_argument("--iou", type=float, default=0.45)
  parser.add_argument("--device", default="0", help="0,1,... or 'cpu'")
  parser.add_argument("--show", action="store_true", help="show window")
  parser.add_argument("--save", default="", help="output mp4 path (optional)")
  parser.add_argument("--only_tl", action="store_true", help="filter traffic light only")
  parser.add_argument("--max_fps", type=float, default=0.0, help="limit fps (0=unlimited)")
  args = parser.parse_args()

  # source 파싱: 숫자면 카메라 인덱스로
  src = args.source
  if isinstance(src, str) and src.isdigit():
    src = int(src)

  cap = cv2.VideoCapture(src)
  if not cap.isOpened():
    raise RuntimeError(f"Failed to open source: {args.source}")

  model = YOLO(args.model)

  w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH) or 0)
  h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT) or 0)
  in_fps = float(cap.get(cv2.CAP_PROP_FPS) or 30.0)

  writer = None
  if args.save:
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    writer = cv2.VideoWriter(args.save, fourcc, in_fps, (w, h))

  prev_t = time.time()
  frame_idx = 0

  while True:
    ok, frame = cap.read()
    if not ok:
      break

    # FPS 제한(옵션)
    if args.max_fps > 0:
      now = time.time()
      dt = now - prev_t
      min_dt = 1.0 / args.max_fps
      if dt < min_dt:
        time.sleep(min_dt - dt)
      prev_t = time.time()

    # YOLO 추론
    results = model.predict(
      source=frame,
      imgsz=args.imgsz,
      conf=args.conf,
      iou=args.iou,
      device=args.device,
      verbose=False,
    )[0]

    names = results.names  # class id -> name
    annotated = frame

    # 박스 수동 그리기(“신호등만” 필터 가능)
    if results.boxes is not None and len(results.boxes) > 0:
      print (results)
      for b in results.boxes:
        cls_id = int(b.cls.item())
        name = names.get(cls_id, str(cls_id))
        if args.only_tl and not is_traffic_light(name):
          continue

        conf = float(b.conf.item())
        x1, y1, x2, y2 = map(int, b.xyxy[0].tolist())

        cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(annotated, f"{name} {conf:.2f}", (x1, max(0, y1 - 7)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    if writer is not None:
      writer.write(annotated)

    if args.show:
      cv2.imshow("YOLO Traffic Light", annotated)
      key = cv2.waitKey(1) & 0xFF
      if key == 27 or key == ord('q'):
        break

    frame_idx += 1

  cap.release()
  if writer is not None:
    writer.release()
  cv2.destroyAllWindows()


if __name__ == "__main__":
  main()
