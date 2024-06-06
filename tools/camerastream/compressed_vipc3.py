#!/usr/bin/env python3
import av
import os
import sys
import numpy as np
import multiprocessing
import time
import cv2  # Import OpenCV for image display

import cereal.messaging as messaging
from cereal.visionipc import VisionIpcServer, VisionStreamType

V4L2_BUF_FLAG_KEYFRAME = 8

# start encoderd
# also start cereal messaging bridge
# then run this "./compressed_vipc.py <ip>"

ENCODE_SOCKETS = {
  VisionStreamType.VISION_STREAM_ROAD: "roadEncodeData",
}

def decoder(addr, vipc_server, vst, W, H, debug=False):
  sock_name = ENCODE_SOCKETS[vst]
  if debug:
    print(f"start decoder for {sock_name}, {W}x{H}")

  codec = av.CodecContext.create("hevc", "r")

  os.environ["ZMQ"] = "1"
  messaging.context = messaging.Context()
  sock = messaging.sub_sock(sock_name, None, addr=addr, conflate=False)
  cnt = 0
  last_idx = -1
  seen_iframe = False

  time_q = []
  last_capture_time = time.time()
  while 1:
    msgs = messaging.drain_sock(sock, wait_for_one=True)
    for evt in msgs:
      evta = getattr(evt, evt.which())
      if debug and evta.idx.encodeId != 0 and evta.idx.encodeId != (last_idx+1):
        print("DROP PACKET!")
      last_idx = evta.idx.encodeId
      if not seen_iframe and not (evta.idx.flags & V4L2_BUF_FLAG_KEYFRAME):
        if debug:
          print("waiting for iframe")
        continue
      time_q.append(time.monotonic())
      network_latency = (int(time.time()*1e9) - evta.unixTimestampNanos)/1e6
      frame_latency = ((evta.idx.timestampEof/1e9) - (evta.idx.timestampSof/1e9))*1000
      process_latency = ((evt.logMonoTime/1e9) - (evta.idx.timestampEof/1e9))*1000

      # put in header (first)
      if not seen_iframe:
        codec.decode(av.packet.Packet(evta.header))
        seen_iframe = True

      frames = codec.decode(av.packet.Packet(evta.data))
      if len(frames) == 0:
        if debug:
          print("DROP SURFACE")
        continue
      assert len(frames) == 1
      img_yuv = frames[0].to_ndarray(format=av.video.format.VideoFormat('yuv420p'))

      # Capture and display the frame every second
      current_time = time.time()
      if current_time - last_capture_time >= 1:
        img_rgb = cv2.cvtColor(img_yuv, cv2.COLOR_YUV2BGR_I420)
        cv2.imwrite(f"frame_{cnt}.jpg", img_rgb)  # Save the frame for debugging
        #cv2.imshow("Captured Frame", img_rgb)
        #cv2.waitKey(1)  # Display the frame for 1 ms
        last_capture_time = current_time

      img_yuv = img_yuv.flatten()
      uv_offset = H*W
      y = img_yuv[:uv_offset]
      uv = img_yuv[uv_offset:].reshape(2, -1).ravel('F')
      img_yuv = np.hstack((y, uv))

      vipc_server.send(vst, img_yuv.data, cnt, int(time_q[0]*1e9), int(time.monotonic()*1e9))
      cnt += 1

      pc_latency = (time.monotonic()-time_q[0])*1000
      time_q = time_q[1:]
      if debug:
        print("%2d %4d %.3f %.3f roll %6.2f ms latency %6.2f ms + %6.2f ms + %6.2f ms = %6.2f ms"
              % (len(msgs), evta.idx.encodeId, evt.logMonoTime/1e9, evta.idx.timestampEof/1e6, frame_latency,
                 process_latency, network_latency, pc_latency, process_latency+network_latency+pc_latency ), len(evta.data), sock_name)

class CompressedVipc:
  def __init__(self, addr, vision_streams, debug=False):
    print("getting frame sizes")
    os.environ["ZMQ"] = "1"
    messaging.context = messaging.Context()
    sm = messaging.SubMaster([ENCODE_SOCKETS[s] for s in vision_streams], addr=addr)
    while min(sm.recv_frame.values()) == 0:
      sm.update(100)
    os.environ.pop("ZMQ")
    messaging.context = messaging.Context()

    self.vipc_server = VisionIpcServer("camerad")
    for vst in vision_streams:
      ed = sm[ENCODE_SOCKETS[vst]]
      self.vipc_server.create_buffers(vst, 4, False, ed.width, ed.height)
    self.vipc_server.start_listener()

    self.procs = []
    for vst in vision_streams:
      ed = sm[ENCODE_SOCKETS[vst]]
      p = multiprocessing.Process(target=decoder, args=(addr, self.vipc_server, vst, ed.width, ed.height, debug))
      p.start()
      self.procs.append(p)

  def join(self):
    for p in self.procs:
      p.join()

  def kill(self):
    for p in self.procs:
      p.terminate()
    self.join()

if __name__ == "__main__":
  addr = "192.168.0.28"
  debug = True

  vision_streams = [
    VisionStreamType.VISION_STREAM_ROAD,
  ]

  cvipc = CompressedVipc(addr, vision_streams, debug=debug)
  cvipc.join()
