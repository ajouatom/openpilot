#!/usr/bin/env python3
import os
import gc
import math
import time
import ctypes
import numpy as np
from pathlib import Path
from typing import Tuple, Dict

from cereal import messaging
from cereal.messaging import PubMaster, SubMaster
from cereal.visionipc import VisionIpcClient, VisionStreamType, VisionBuf
from openpilot.common.params import Params
from openpilot.common.realtime import set_realtime_priority, Ratekeeper
from openpilot.selfdrive.modeld.carrot.yolov8 import YOLOv8

from system.camerad.snapshot.snapshot import extract_image

#MODEL_WIDTH = 1440
#MODEL_HEIGHT = 960
#MODEL_PATH = Path(__file__).parent / 'models/carrot-512.onnx'

def main():
  gc.disable()
  set_realtime_priority(0)

  vipc_client = VisionIpcClient("camerad", VisionStreamType.VISION_STREAM_ROAD, True)
  while not vipc_client.connect(False):
    time.sleep(0.1)
  assert vipc_client.is_connected()
  #cloudlog.warning(f"connected with buffer size: {vipc_client.buffer_len}")

  #sm = SubMaster(["liveCalibration"])
  pm = PubMaster(["carrotModel"])
  #carrot = YOLOv8(Path(__file__).parent / 'carrot-512.onnx', conf_thres=0.1, iou_thres=0.1)
  carrot = YOLOv8(Path(__file__).parent / 'yolov8n.onnx', conf_thres=0.3, iou_thres=0.5)
  last = 0
  rk = Ratekeeper(5, print_delay_threshold=None)

  time.sleep(5)

  while True:
    buf = vipc_client.recv()
    if buf is None:
      continue

    #sm.update(0)

    if Params().get_int("ShowDebugUI") > 1:
      t1 = time.perf_counter()
      #print("########### CARROT.start ##########")
      boxes, scores, class_ids = carrot(buf)
      #print("########### CARROT.end ##########")
      #if len(class_ids) > 0:
      #    print("boxes={}, scores={}, class_ids={}".format(boxes, scores, class_ids))

      t2 = time.perf_counter()

      #pm.send("driverStateV2", get_driverstate_packet(model_output, vipc_client.frame_id, vipc_client.timestamp_sof, t2 - t1, dsp_execution_time))
      msg = messaging.new_message('carrotModel', valid=True)
      detections = msg.carrotModel.init('detections', len(boxes))
      for i in range(len(boxes)):
        detection = detections[i]
        box = detection.init('box')
        box.xMin = float(boxes[i][0])
        box.yMin = float(boxes[i][1])
        box.xMax = float(boxes[i][2])
        box.yMax = float(boxes[i][3])
        detection.score = float(scores[i])
        detection.classId = int(class_ids[i])

    
      pm.send("carrotModel", msg)
      print("carrotmodeld: %.2fs, from last %.2fs\n" % (t2 - t1, t1 - last))
      last = t1
    rk.keep_time()


if __name__ == "__main__":
  main()
