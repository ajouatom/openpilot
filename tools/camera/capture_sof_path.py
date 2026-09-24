import collections, json, os, signal, sys, time
from pathlib import Path
from openpilot.cereal import messaging

os.sched_setaffinity(0,{0,1,2,3}); os.nice(19)
mode=sys.argv[1] if len(sys.argv)>1 else 'baseline'
duration=float(sys.argv[2]) if len(sys.argv)>2 else 60
label=sys.argv[3] if len(sys.argv)>3 else str(int(time.time()))
outdir=Path('/data/camera-bt2-investigation')/label
outdir.mkdir(parents=True,exist_ok=False)
guard=messaging.SubMaster(['carState','selfdriveState','deviceState'])
for _ in range(50):
 guard.update(100)
 if guard.all_alive(): break
assert guard.all_alive()
assert str(guard['carState'].gearShifter)=='park' and guard['carState'].vEgo<.01 and not guard['selfdriveState'].enabled
assert guard['deviceState'].started
instances=[]
def wr(root, name, val): (root/name).write_text(str(val))
def instance(name,mask,size):
 p=Path('/sys/kernel/debug/tracing/instances')/name
 p.mkdir();instances.append(p)
 wr(p,'tracing_on',0);wr(p,'buffer_size_kb',size);wr(p,'trace_clock','boot');wr(p,'tracing_cpumask',mask)
 return p
def ev(p,name,filt=None):
 q=p/'events'/name
 if filt is not None:wr(q,'filter',filt)
 wr(q,'enable',1)
def finish_signal(signum,frame):raise InterruptedError(f'signal {signum}')
signal.signal(signal.SIGTERM,finish_signal);signal.signal(signal.SIGINT,finish_signal)
rows=collections.deque(maxlen=9000);counts=collections.Counter();maxima={};last={};trigger=None
meta={'mode':mode,'duration_requested':duration,'label':label,'guard':'P, stopped, disabled, started','build':Path('/BUILD').read_text(),'start_ns':time.clock_gettime_ns(time.CLOCK_BOOTTIME),'mem_before':Path('/proc/meminfo').read_text(),'irq_before':Path('/proc/interrupts').read_text()}
meta['pid']=os.getpid()
meta['params']={k:Path('/data/params/d',k).read_text() for k in ['DisableDM','UsbGpuActive','IsOnroad'] if Path('/data/params/d',k).exists()}
(outdir/'started.json').write_text(json.dumps(meta))
try:
 if mode=='trace':
  cam=instance('carrot_bt2_camera','ff',256)
  for e in ['cam_isp_activated_irq','cam_req_mgr_apply_request','cam_submit_to_hw','cam_buf_done','cam_irq_handled','cam_irq_activated']:
   ev(cam,'camera/'+e)
  for e in ['cam_csid_sof_history','cam_ife_irq_payload']:
   if (cam/'events/camera'/e).exists():ev(cam,'camera/'+e)
  ev(cam,'irq/irq_handler_entry','irq == 8')
  ev(cam,'irq/irq_handler_exit','irq == 8')
  cpu=instance('carrot_bt2_cpu6','40',512)
  for e in ['irq/irq_handler_entry','irq/irq_handler_exit','irq/softirq_entry','irq/softirq_exit','sched/sched_switch','sched/sched_wakeup','sched/sched_waking','workqueue/workqueue_execute_start','workqueue/workqueue_execute_end']:
   ev(cpu,e)
  for p in instances:wr(p,'tracing_on',1)
 poller=messaging.Poller()
 names=['roadCameraState','wideRoadCameraState','driverCameraState','cameraOdometry','livePose','carState','selfdriveState','deviceState','logMessage']
 socks={messaging.sub_sock(n,poller=poller,conflate=(n in ['carState','selfdriveState','deviceState'])):n for n in names}
 start=time.monotonic();deadline=start+duration;next_status=start+30;last_guard=start
 print(json.dumps({'started':label,'mode':mode,'seconds':duration}),flush=True)
 while time.monotonic()<deadline:
  for sock in poller.poll(100):
   for msg in messaging.drain_sock(sock):
    n=msg.which()
    v=getattr(msg,n);now=time.monotonic();counts[n]+=1
    row={'s':n,'t':msg.logMonoTime}
    if n.endswith('CameraState'):
     row.update(f=v.frameId,r=v.requestId,sof=v.timestampSof,eof=v.timestampEof)
     prev=last.get(n)
     if prev:
      row['dt_ms']=(v.timestampSof-prev['sof'])/1e6;row['df']=v.frameId-prev['f']
      maxima[n]=max(maxima.get(n,0),row['dt_ms'])
      if row['dt_ms']>75 and now-start>3 and trigger is None:
       trigger=dict(row);deadline=min(deadline,now+.2)
       print(json.dumps({'trigger':trigger}),flush=True)
     last[n]=dict(row)
    elif n=='carState':
     last_guard=now
     assert str(v.gearShifter)=='park' and v.vEgo<.1, 'Vehicle no longer parked'
     continue
    elif n=='selfdriveState':
     assert not v.enabled,'Controls enabled'
     continue
    elif n=='deviceState':
     assert v.started,'Vehicle offroad'
     row.update(cpu=list(v.cpuUsagePercent),temp=list(v.cpuTempC),mem=v.memoryUsagePercent)
    elif n=='livePose':row.update(inputsOK=v.inputsOK,sensorsOK=v.sensorsOK,posenetOK=v.posenetOK,valid=msg.valid)
    elif n=='cameraOdometry':row.update(frameId=v.frameId,valid=msg.valid)
    else:
     text=str(v)
     if not any(x in text.lower() for x in ['camera','cam_','ife','csid','overflow','bubble','sof','proclog','sched']):continue
     row['text']=text[:3000]
    rows.append(row)
  assert time.monotonic()-last_guard<3,'carState guard stale'
  if time.monotonic()>next_status:
   print(json.dumps({'elapsed':round(time.monotonic()-start,1),'max_sof_ms':maxima,'counts':dict(counts)}),flush=True);next_status+=30
 meta['elapsed']=time.monotonic()-start
except BaseException as e:
 meta['error']=repr(e)
 raise
finally:
 for p in instances:wr(p,'tracing_on',0)
 meta.update(trigger=trigger,max_sof_ms=maxima,counts=dict(counts),end_ns=time.clock_gettime_ns(time.CLOCK_BOOTTIME),mem_after=Path('/proc/meminfo').read_text(),irq_after=Path('/proc/interrupts').read_text())
 for p in instances:
  meta[p.name+'_stats']={q.parent.name:q.read_text() for q in (p/'per_cpu').glob('cpu*/stats')}
  with (p/'trace').open() as src,(outdir/(p.name+'.txt')).open('w') as dst:
   while chunk:=src.read(1024*1024):dst.write(chunk)
  wr(p,'events/enable',0);p.rmdir()
 (outdir/'metadata.json').write_text(json.dumps(meta))
 with (outdir/'messages.jsonl').open('w') as f:
  for row in rows:f.write(json.dumps(row)+'\n')
 print(json.dumps({'finished':str(outdir),'trigger':trigger,'max_sof_ms':maxima,'error':meta.get('error')}),flush=True)
