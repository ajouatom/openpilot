import torch
import torchvision.transforms as T
from PIL import Image
import numpy as np

# Load YOLO model
yolo_model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)
yolo_model = yolo_model.autoshape()  # for PIL/cv2/np inputs and NMS

transform = T.Compose([T.ToTensor()])

# Test YOLO with a sample image
def run_yolo(frame):
    img = Image.fromarray(frame)
    img = transform(img).unsqueeze(0)
    results = yolo_model(img)
    return results

# Load a sample image and run YOLO
sample_image = np.random.randint(0, 255, (640, 480, 3), dtype=np.uint8)
results = run_yolo(sample_image)
print(results.pandas().xyxy[0])
