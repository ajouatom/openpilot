import time
import cv2
import numpy as np
import onnxruntime as ort

from openpilot.selfdrive.modeld.carrot.yolov8.utils import xywh2xyxy, draw_detections, multiclass_nms



def yuv_to_rgb(y, u, v):
  ul = np.repeat(np.repeat(u, 2).reshape(u.shape[0], y.shape[1]), 2, axis=0).reshape(y.shape)
  vl = np.repeat(np.repeat(v, 2).reshape(v.shape[0], y.shape[1]), 2, axis=0).reshape(y.shape)

  yuv = np.dstack((y, ul, vl)).astype(np.int16)
  yuv[:, :, 1:] -= 128

  m = np.array([
    [1.00000,  1.00000, 1.00000],
    [0.00000, -0.39465, 2.03211],
    [1.13983, -0.58060, 0.00000],
  ])
  rgb = np.dot(yuv, m).clip(0, 255)
  return rgb.astype(np.uint8)

def yuv_to_rgb_opencv(y, u, v):
    # U, V 채널 업샘플링
    u_upsampled = cv2.resize(u, (y.shape[1], y.shape[0]), interpolation=cv2.INTER_LINEAR)
    v_upsampled = cv2.resize(v, (y.shape[1], y.shape[0]), interpolation=cv2.INTER_LINEAR)

    # YUV를 하나의 이미지로 결합
    yuv = cv2.merge([y, u_upsampled, v_upsampled])

    # YUV에서 RGB로 변환
    rgb = cv2.cvtColor(yuv, cv2.COLOR_YUV2RGB)

    return rgb

def extract_image(buf):
  now2 = time.monotonic()
  y = np.array(buf.data[:buf.uv_offset], dtype=np.uint8).reshape((-1, buf.stride))[:buf.height, :buf.width]
  u = np.array(buf.data[buf.uv_offset::2], dtype=np.uint8).reshape((-1, buf.stride//2))[:buf.height//2, :buf.width//2]
  v = np.array(buf.data[buf.uv_offset+1::2], dtype=np.uint8).reshape((-1, buf.stride//2))[:buf.height//2, :buf.width//2]
          

  return yuv_to_rgb_opencv(y, u, v)

def extract_image2(buf):
    # Y 채널
    y = np.array(buf.data[:buf.uv_offset], dtype=np.uint8).reshape((-1, buf.stride))[:buf.height, :buf.width]

    # U 채널 (Y 채널 해상도의 절반)
    u_offset = buf.uv_offset
    u = np.array(buf.data[u_offset:u_offset + (buf.stride // 2) * (buf.height // 2)], dtype=np.uint8).reshape((-1, buf.stride // 2))

    # V 채널 (Y 채널 해상도의 절반)
    v_offset = u_offset + (buf.stride // 2) * (buf.height // 2)
    v = np.array(buf.data[v_offset:v_offset + (buf.stride // 2) * (buf.height // 2)], dtype=np.uint8).reshape((-1, buf.stride // 2))

    # YUV 이미지를 결합
    yuv = np.concatenate([y, u, v], axis=0)

    # OpenCV를 사용하여 I420에서 RGB로 변환
    rgb = cv2.cvtColor(yuv, cv2.COLOR_YUV2RGB_I420)

    return rgb
class YOLOv8:

    def __init__(self, path, conf_thres=0.5, iou_thres=0.5):
        self.conf_threshold = conf_thres
        self.iou_threshold = iou_thres

        # Initialize model
        self.initialize_model(path)

    def __call__(self, image):
        return self.detect_objects(image)

    def initialize_model(self, path):
        #self.session = onnxruntime.InferenceSession(path,
        #                                            providers=onnxruntime.get_available_providers())
        providers = ort.get_available_providers()
        print(providers)
        if 'CUDAExecutionProvider' in providers:
            self.session = ort.InferenceSession(path, providers=['CUDAExecutionProvider'])
        elif 'TensorrtExecutionProvider' in providers:
            self.session = ort.InferenceSession(path, providers=['TensorrtExecutionProvider'])
        else:
            print("No GPU providers available. Using CPU.")
            self.session = ort.InferenceSession(path, providers=['CPUExecutionProvider'])

        # Get model info
        self.get_input_details()
        self.get_output_details()


    def detect_objects(self, image):
        now1 = time.monotonic()
        input_tensor = self.prepare_input(image)
        now2 = time.monotonic()

        #return [],[],[]
        # Perform inference on the image
        #print("########### YOLO... inference ##########")

        outputs = self.inference(input_tensor)
        #print("########### YOLO... inference ok ##########")
        now3 = time.monotonic()
        self.boxes, self.scores, self.class_ids = self.process_output(outputs)
        #print("execTime_____ = {:.6f}, {:.6f}, {:.6f}".format(now2-now1, now3-now2, time.monotonic() - now3))

        return self.boxes, self.scores, self.class_ids


    def prepare_input(self, image):
        input_img = extract_image(image)

        self.img_height, self.img_width = 1208, 1928 #image.height, image.width #image.shape[:2]

        #input_img = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        #image_path = "/home/yun/media_log/fcamera.hevc_20231212_145229.704.jpg"
        #image = cv2.imread(image_path)
        #if image is None:
        #  #print(f"Error: Could not load image at {image_path}")
        #  pass
        #else:
        #   cv2.imshow("hello", input_img)
        #   cv2.waitKey(1)

        #input_img = image

        # Resize input image
        input_img = cv2.resize(input_img, (self.input_width, self.input_height))

        # Scale input pixel values to 0 to 1
        input_img = input_img / 255.0
        input_img = input_img.transpose(2, 0, 1)
        input_tensor = input_img[np.newaxis, :, :, :].astype(np.float32)

        return input_tensor


    def inference(self, input_tensor):
        start = time.perf_counter()
        outputs = self.session.run(self.output_names, {self.input_names[0]: input_tensor})

        #print(f"Inference time: {(time.perf_counter() - start)*1000:.2f} ms")
        return outputs

    def process_output(self, output):
        predictions = np.squeeze(output[0]).T

        # Filter out object confidence scores below threshold
        scores = np.max(predictions[:, 4:], axis=1)
        predictions = predictions[scores > self.conf_threshold, :]
        scores = scores[scores > self.conf_threshold]

        if len(scores) == 0:
            return [], [], []

        # Get the class with the highest confidence
        class_ids = np.argmax(predictions[:, 4:], axis=1)

        # Get bounding boxes for each object
        boxes = self.extract_boxes(predictions)

        # Apply non-maxima suppression to suppress weak, overlapping bounding boxes
        # indices = nms(boxes, scores, self.iou_threshold)
        indices = multiclass_nms(boxes, scores, class_ids, self.iou_threshold)

        return boxes[indices], scores[indices], class_ids[indices]

    def extract_boxes(self, predictions):
        # Extract boxes from predictions
        boxes = predictions[:, :4]

        # Scale boxes to original image dimensions
        boxes = self.rescale_boxes(boxes)

        # Convert boxes to xyxy format
        boxes = xywh2xyxy(boxes)

        return boxes

    def rescale_boxes(self, boxes):

        # Rescale boxes to original image dimensions
        input_shape = np.array([self.input_width, self.input_height, self.input_width, self.input_height])
        boxes = np.divide(boxes, input_shape, dtype=np.float32)
        boxes *= np.array([self.img_width, self.img_height, self.img_width, self.img_height])
        return boxes

    def draw_detections(self, image, draw_scores=True, mask_alpha=0.4):

        return draw_detections(image, self.boxes, self.scores,
                               self.class_ids, mask_alpha)

    def get_input_details(self):
        model_inputs = self.session.get_inputs()
        self.input_names = [model_inputs[i].name for i in range(len(model_inputs))]

        self.input_shape = model_inputs[0].shape
        self.input_height = self.input_shape[2]
        self.input_width = self.input_shape[3]
        print("CARROT model = {},{}", self.input_width, self.input_height)

    def get_output_details(self):
        model_outputs = self.session.get_outputs()
        self.output_names = [model_outputs[i].name for i in range(len(model_outputs))]


if __name__ == '__main__':
    from imread_from_url import imread_from_url

    model_path = "../models/yolov8m.onnx"

    # Initialize YOLOv8 object detector
    yolov8_detector = YOLOv8(model_path, conf_thres=0.3, iou_thres=0.5)

    img_url = "https://live.staticflickr.com/13/19041780_d6fd803de0_3k.jpg"
    img = imread_from_url(img_url)

    # Detect Objects
    yolov8_detector(img)

    # Draw detections
    combined_img = yolov8_detector.draw_detections(img)
    cv2.namedWindow("Output", cv2.WINDOW_NORMAL)
    cv2.imshow("Output", combined_img)
    cv2.waitKey(0)


