"""Bounded central lead brake-lamp candidate experiment. No actuation/ONNX.

There is deliberately no OFF classifier: absence of a candidate is unknown.
Rear-body/whole-lamp learning and encounter-separated validation remain needed.
"""
import math
import cv2
import numpy as np

from openpilot.common.transformations.camera import DEVICE_CAMERAS, view_frame_from_device_frame
from openpilot.common.transformations.orientation import rot_from_euler


class LeadBrakeDetector:
  def __init__(self):
    self.reset()

  def reset(self):
    self.ident = None
    self.box = None
    self.since = self.last = None
    self.count = 0

  def unknown(self, reason):
    self.reset()
    return dict(state='unknown', reason=reason, box=None)

  def process(self, rgb, stamp, lead, vision, car, calibration):
    try:
      distance, lateral = float(lead['dRel']), -float(lead['yRel'])
      yaw, steer, speed = float(car['yawRate']), float(car['steeringAngleDeg']), float(car['vEgo'])
      ident = int(lead['radarTrackId'])
      if not all(math.isfinite(v) for v in (stamp, distance, lateral, yaw, steer, speed)):
        return self.unknown('invalid_context')
      if (not lead['status'] or not lead['radar'] or ident < 0 or not 3 <= distance <= 35
          or float(vision.get('prob', 0)) < .8):
        return self.unknown('no_matched_lead')
      if abs(yaw) > .06 or (abs(steer) > 15 and abs(speed) > 2):
        return self.unknown('turning')
      depth = distance + 1.52
      vx, vy = float(vision['x'][0]), float(vision['y'][0])
      if not math.isfinite(vx+vy) or abs(vx-depth) > max(2., .15*distance) or abs(vy-lateral) > .5:
        return self.unknown('vision_radar_mismatch')
      if calibration['calStatus'] != 'calibrated' or rgb.shape != (760, 1344, 3):
        return self.unknown('uncalibrated')
      height = float(calibration['height'][0])
      rpy = np.asarray(calibration['rpyCalib'], dtype=float)
      if rpy.shape != (3,) or not np.isfinite(rpy).all() or not .7 <= height <= 2.2:
        return self.unknown('uncalibrated')
      # Restricted to the C4 road camera geometry used by the supplying worker.
      p = DEVICE_CAMERAS[('mici', 'os04c10')].fcam.intrinsics @ view_frame_from_device_frame @ rot_from_euler(rpy[None, :])[0]
      points = np.array([p @ [depth, lateral+y, height-z] for y in (-.27, .27) for z in (1.25, 1.9)])
      if (points[:, 2] <= 0).any():
        return self.unknown('projection')
      xy = points[:, :2]/points[:, 2, None]
      x1, y1 = np.floor(xy.min(axis=0)).astype(int)
      x2, y2 = np.ceil(xy.max(axis=0)).astype(int)
      if not (0 <= x1 < x2 <= 1344 and 0 <= y1 < y2 <= 760 and (x2-x1)*(y2-y1) <= 24000):
        return self.unknown('crop_budget')
      patch = rgb[y1:y2, x1:x2].astype(np.int16)
      r, g, b = cv2.split(patch)
      mask = ((r >= 160) & (r-g >= 45) & (r-b >= 45)).astype(np.uint8)*255
      mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((1, 3), np.uint8))
      candidates = []
      for c in cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]:
        x, y, w, h = cv2.boundingRect(c)
        if (cv2.contourArea(c) >= 4 and 2.2 <= w/max(h, 1) <= 18 and .07 <= w*depth/1141.5 <= .7
            and x > 0 and y > 0 and x+w < x2-x1 and y+h < y2-y1):
          candidates.append([int(x+x1), int(y+y1), int(x+x1+w), int(y+y1+h)])
      if len(candidates) != 1:
        return self.unknown('absent_or_ambiguous')
      box = candidates[0]
      continuous = (ident == self.ident and self.last is not None and 0 < stamp-self.last <= .25
                    and abs(box[0]-self.box[0]) <= max(8, box[2]-box[0])
                    and abs(box[1]-self.box[1]) <= max(6, box[3]-box[1]))
      if not continuous:
        self.since, self.count = stamp, 0
      self.ident, self.box, self.last = ident, box, stamp
      self.count += 1
      confirmed = self.count >= 3 and stamp-self.since >= .15
      return dict(state='candidate' if confirmed else 'unknown', reason='central_lamp' if confirmed else 'confirming',
                  box=box, lead_id=ident, distance=distance)
    except (KeyError, IndexError, ValueError, TypeError, OverflowError):
      return self.unknown('invalid_context')
