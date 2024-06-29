import cv2  # Import OpenCV for image display
#import os
#import sys
import numpy as np
import threading
import time
from queue import Queue

import cereal.messaging as messaging
from cereal.visionipc import VisionIpcServer, VisionStreamType


def main():
  # 이미지의 크기 설정 (예: 512x512)
  height = 512
  width = 512

  # 랜덤 이미지 생성 (0에서 255 사이의 값으로 채워진 height x width x 3 크기의 배열)
  random_image = np.random.randint(0, 256, (height, width, 3), dtype=np.uint8)

  # 이미지 표시
  print("show image")
  cv2.imshow('Random Image', random_image)
  print("show end")
  import av

  # 아무 키나 누를 때까지 대기
  cv2.waitKey(0)

  # 창 닫기
  cv2.destroyAllWindows()

if __name__ == "__main__":
  main()
