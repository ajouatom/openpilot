import json
import os
import sys

if len(sys.argv) < 2:
    print("사용법: python script.py <json_file_path>")
    sys.exit(1)

json_file_path = sys.argv[1]

# JSON 파일 경로
#json_file_path = 'data.json'

# 저장할 디렉토리 경로
directory = '/data/params/d/'

# JSON 파일 읽기
with open(json_file_path, 'r') as file:
    data = json.load(file)

# 디렉토리 생성 (존재하지 않는 경우)
#os.makedirs(directory, exist_ok=True)

for item in data:
    filename = item['filename']
    content = item['content']
    file_path = os.path.join(directory, filename)

    with open(file_path, 'w') as file:
        file.write(content)

#print("파일이 성공적으로 저장되었습니다.")
