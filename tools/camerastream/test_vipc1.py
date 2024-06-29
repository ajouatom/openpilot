import os
import time
import cv2
import threading
import numpy as np
from queue import Queue
import cereal.messaging as messaging
from cereal.visionipc import VisionIpcServer, VisionStreamType
from ultralytics import YOLO  # Import YOLO from ultralytics

V4L2_BUF_FLAG_KEYFRAME = 8

# Define encode sockets
ENCODE_SOCKETS = {
    VisionStreamType.VISION_STREAM_ROAD: "roadEncodeData",
    VisionStreamType.VISION_STREAM_WIDE_ROAD: "wideRoadEncodeData",
    VisionStreamType.VISION_STREAM_DRIVER: "driverEncodeData",
}

def load_yolov8_model():
    return YOLO("yolov8n.pt")  # Load the YOLOv8 model

def run_yolov8_on_frame(model, frame):
    results = model(frame)
    return results

def resize_image(image, scale_percent):
    width = int(image.shape[1] * scale_percent / 100)
    height = int(image.shape[0] * scale_percent / 100)
    dim = (width, height)
    return cv2.resize(image, dim, interpolation=cv2.INTER_AREA)

def frame_processor(frame_queue, yolov8_model, debug=False):
    # Ensure OpenCV is properly initialized
    blank_image = np.zeros((100, 100, 3), np.uint8)
    cv2.imshow("Initialization", blank_image)
    cv2.waitKey(1)
    cv2.destroyAllWindows()
    
    while True:
        frame = frame_queue.get()
        if frame is None:
            break
        img_rgb, cnt = frame

        # Resize image for faster processing
        img_rgb_resized = resize_image(img_rgb, 50)

        if debug:
            results = run_yolov8_on_frame(yolov8_model, img_rgb_resized)
            for result in results:
                if result.boxes:
                    for box in result.boxes:
                        xyxy = box.xyxy[0].cpu().numpy()
                        conf = box.conf[0].cpu().numpy()
                        cls = box.cls[0].cpu().numpy()
                        cls_name = yolov8_model.names[int(cls)]
                        cv2.rectangle(img_rgb, (int(xyxy[0] * 2), int(xyxy[1] * 2)), (int(xyxy[2] * 2), int(xyxy[3] * 2)), (0, 255, 0), 2)
                        cv2.putText(img_rgb, f"{cls_name}: {conf:.2f}", (int(xyxy[0] * 2), int(xyxy[1] * 2) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)

        cv2.imshow("Captured Frame with YOLOv8", img_rgb)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    cv2.destroyAllWindows()

def decoder(addr, vipc_server, vst, W, H, frame_queue, debug=False):
    import av  # Import av after OpenCV initialization

    sock_name = ENCODE_SOCKETS[vst]
    if debug:
        print(f"Start decoder for {sock_name}, {W}x{H}")

    codec = av.CodecContext.create("hevc", "r")

    os.environ["ZMQ"] = "1"
    messaging.context = messaging.Context()
    sock = messaging.sub_sock(sock_name, None, addr=addr, conflate=False)
    cnt = 0
    last_idx = -1
    seen_iframe = False

    while True:
        msgs = messaging.drain_sock(sock, wait_for_one=True)
        for evt in msgs:
            evta = getattr(evt, evt.which())
            if debug and evta.idx.encodeId != 0 and evta.idx.encodeId != (last_idx + 1):
                print("DROP PACKET!")
            last_idx = evta.idx.encodeId
            if not seen_iframe and not (evta.idx.flags & V4L2_BUF_FLAG_KEYFRAME):
                if debug:
                    print("Waiting for iframe")
                continue

            if not seen_iframe:
                codec.decode(av.packet.Packet(evta.header))
                seen_iframe = True

            frames = codec.decode(av.packet.Packet(evta.data))
            if len(frames) == 0:
                continue

            if frame_queue.qsize() < 10:  # Increase queue size to buffer more frames
                img_yuv = frames[0].to_ndarray(format=av.video.format.VideoFormat('yuv420p'))
                img_rgb = cv2.cvtColor(img_yuv, cv2.COLOR_YUV2BGR_I420)
                frame_queue.put((img_rgb, cnt))

            cnt += 1
            print("%2d" % (len(msgs)))

class CompressedVipc:
    def __init__(self, addr, vision_streams, debug=False):
        print("Getting frame sizes")
        os.environ["ZMQ"] = "1"
        messaging.context = messaging.Context()
        sm = messaging.SubMaster([ENCODE_SOCKETS[s] for s in vision_streams], addr=addr)
        while min(sm.recv_frame.values()) == 0:
            sm.update(100)
        os.environ.pop("ZMQ")
        messaging.context = messaging.Context()

        self.frame_queue = Queue(maxsize=10)  # Set maximum queue size to buffer more frames
        self.procs = []
        yolov8_model = load_yolov8_model()
        self.display_thread = threading.Thread(target=frame_processor, args=(self.frame_queue, yolov8_model, debug))
        self.display_thread.start()
        time.sleep(1.0)

        self.vipc_server = VisionIpcServer("camerad")
        for vst in vision_streams:
            ed = sm[ENCODE_SOCKETS[vst]]
            self.vipc_server.create_buffers(vst, 4, False, ed.width, ed.height)
        self.vipc_server.start_listener()

        for vst in vision_streams:
            ed = sm[ENCODE_SOCKETS[vst]]
            p = threading.Thread(target=decoder, args=(addr, self.vipc_server, vst, ed.width, ed.height, self.frame_queue, debug))
            p.start()
            self.procs.append(p)

    def join(self):
        for p in self.procs:
            p.join()
        self.frame_queue.put(None)
        self.display_thread.join()

    def kill(self):
        for p in self.procs:
            p.terminate()
        self.join()

def main():
    addr = "192.168.0.28"
    debug = True

    vision_streams = [
        VisionStreamType.VISION_STREAM_ROAD,
    ]

    cvipc = CompressedVipc(addr, vision_streams, debug=debug)
    cvipc.join()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
