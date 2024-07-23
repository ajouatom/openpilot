#!/usr/bin/env python3
import os
import json

folder_path = "/data/params/d" 
output_file = "/data/backup_params.json" 

result = [] 

for filename in os.listdir(folder_path):
    file_path = os.path.join(folder_path, filename)
    if os.path.isfile(file_path) and os.path.getsize(file_path) < 16:
        with open(file_path, "rb") as f:
            content = f.read().decode("utf-8", "ignore")
            result.append({
                "filename": filename,
                "content": content
            })
sorted_data = sorted(result, key=lambda x: x['filename'])
#print(json.dumps(result, indent=4))
with open(output_file, "w") as f:
    json.dump(sorted_data, f, indent=2, ensure_ascii=False)
