"""Run the offline signal observer on a video; write JSONL and annotated MP4.

python tools/signal_analysis/replay_signal.py VIDEO OUTPUT_DIR --start-frame 0 --end-frame 199
Dependencies: numpy, opencv-python-headless, imageio-ffmpeg. No car connection.
"""
import argparse
import json
from pathlib import Path
import cv2
import imageio_ffmpeg
import numpy as np
from signal_tracker import SignalTracker


def main():
  parser=argparse.ArgumentParser(description=__doc__)
  parser.add_argument('video',type=Path)
  parser.add_argument('output_dir',type=Path)
  parser.add_argument('--start-frame',type=int,default=0)
  parser.add_argument('--end-frame',type=int,required=True)
  parser.add_argument('--fps',type=float,default=20.)
  parser.add_argument('--timestamps',type=Path,help='Optional JSON {frame_index: timestamp_eof_ns}; otherwise frame/fps')
  args=parser.parse_args()
  if args.start_frame<0 or args.end_frame<args.start_frame or args.fps<=0:
    parser.error('Invalid frame range or fps')
  args.output_dir.mkdir(parents=True,exist_ok=True)
  for name in ['observer.mp4','observer.jsonl','summary.json']:
    if (args.output_dir/name).exists():parser.error(f'Output exists: {name}; choose another directory')
  timestamps=json.loads(args.timestamps.read_text()) if args.timestamps else None
  cv2.setNumThreads(2)
  frames=imageio_ffmpeg.read_frames(str(args.video),pix_fmt='rgb24',input_params=['-threads','2'])
  meta=next(frames);width,height=meta['size']
  writer=imageio_ffmpeg.write_frames(str(args.output_dir/'observer.mp4'),(width,height),fps=args.fps,codec='libx264',pix_fmt_in='rgb24',pix_fmt_out='yuv420p',macro_block_size=2,output_params=['-preset','veryfast','-crf','22','-movflags','+faststart'])
  writer.send(None)
  observer=SignalTracker();counts=dict(red=0,green=0,unknown=0)
  try:
    with (args.output_dir/'observer.jsonl').open('w') as output:
      for index,raw in enumerate(frames):
        if index<args.start_frame:continue
        if index>args.end_frame:break
        rgb=np.frombuffer(raw,np.uint8).reshape(height,width,3).copy()
        timestamp=timestamps[str(index)]/1e9 if timestamps else index/args.fps
        result=observer.process(rgb,timestamp);counts[result['state']]+=1
        output.write(json.dumps(dict(frame=index,timestamp=timestamp,**result))+'\n')
        for track in result['tracks']:
          x1,y1,x2,y2=map(int,track['box']);color=(0,255,100) if track['state']=='green' else (255,70,70) if track['state']=='red' else (255,210,0)
          cv2.rectangle(rgb,(x1,y1),(x2,y2),color,2)
          cv2.putText(rgb,f"{track['id']}:{track['state']}",(x1,max(20,y1-4)),cv2.FONT_HERSHEY_SIMPLEX,.45,color,1,cv2.LINE_AA)
        cv2.rectangle(rgb,(0,0),(width,66),(15,20,30),-1)
        cv2.putText(rgb,f"OBSERVED: {result['state'].upper()} | frame {index}",(15,28),cv2.FONT_HERSHEY_SIMPLEX,.7,(255,255,255),2,cv2.LINE_AA)
        cv2.putText(rgb,'Offline visual observation - no lane assignment or vehicle control',(15,53),cv2.FONT_HERSHEY_SIMPLEX,.5,(255,255,255),1,cv2.LINE_AA)
        writer.send(rgb)
  finally:
    frames.close();writer.close()
  expected=args.end_frame-args.start_frame+1
  summary=dict(video=str(args.video),requested_frames=expected,processed_frames=sum(counts.values()),counts=counts,complete=sum(counts.values())==expected,timing='camera EOF' if timestamps else 'nominal frame/fps',scope='Offline observer only; no ground truth supplied, no lane identity or movement permission')
  (args.output_dir/'summary.json').write_text(json.dumps(summary,indent=2))
  print(json.dumps(summary))
  if not summary['complete']:raise SystemExit('Input ended before requested final frame')


if __name__=='__main__':main()
