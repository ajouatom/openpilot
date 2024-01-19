import os
import re
from openpilot.common.params import Params

def get_car_names(dir_path):
  names = []
  for folder in os.listdir(dir_path):
    file_path = os.path.join(dir_path, folder, 'values.py')
    if os.path.exists(file_path):
      with open(file_path) as f:
        content = f.read()
        car_class_match = re.search(r'class CAR\(StrEnum\):([\s\S]*?)(?=^\w)', content, re.MULTILINE)
        if car_class_match:
          names += re.findall(r'=\s*"([^"]+)"', car_class_match.group(1))
  return sorted(names)

def main():
  Params().put("CarModels", ','.join(get_car_names('/data/openpilot/selfdrive/car')))

if __name__ == "__main__":
  main()
