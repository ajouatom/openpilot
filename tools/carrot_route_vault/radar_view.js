// Browser presentation of the production desktop replay; no decision logic runs here.
const style = document.createElement('style');
style.textContent = `
.radar-layout{display:grid;grid-template-columns:minmax(0,1.2fr) minmax(320px,1fr);gap:16px;align-items:stretch}
.radar-layout>section:first-child{display:flex;align-items:center;min-width:0;background:#090e14}
.radar-layout .video-stage{width:100%}
.radar-review{min-width:0;margin-bottom:24px}
.radar-transport{display:flex;align-items:center;gap:14px;margin-top:16px;padding:12px 16px}
.radar-transport [data-time]{font-size:12px;white-space:nowrap;font-variant-numeric:tabular-nums}
.radar-plots{margin-top:16px;padding:16px;min-width:0}
.radar-plot-legend{display:flex;gap:8px 18px;flex-wrap:wrap;font-size:12px;margin-bottom:8px}
.radar-panel{min-width:0}.radar-panel h2{margin:0}.radar-toolbar{display:flex;gap:10px;align-items:center;flex-wrap:wrap;margin:12px 0}
.radar-toolbar label{font-size:12px;color:var(--muted)}.radar-toolbar select{width:auto;padding:6px;display:block}
.radar-map{display:block;width:100%;height:clamp(280px,30vw,420px);background:#090e14;border-radius:12px;touch-action:manipulation}
.radar-layout,.radar-plots{cursor:pointer}
.radar-graph{display:block;width:100%;height:350px;background:#090e14;border-radius:8px}
.radar-status{min-height:2.8em;color:var(--muted);font-size:13px;margin:10px 0;overflow-wrap:anywhere}
.radar-legend{font-size:12px;line-height:1.8;color:#c1cbd6}.radar-readout{font-size:13px;font-variant-numeric:tabular-nums;white-space:pre-wrap;overflow-wrap:anywhere}
.radar-detail{font-size:12px;white-space:pre-wrap;max-height:190px;overflow:auto;color:#b9c7d6}
.radar-scrub{flex:1;min-width:0;width:100%;height:24px;padding:0;border:0;box-shadow:none;accent-color:var(--orange)}.radar-error{color:#ff9696}
@media(max-width:850px){.radar-layout{grid-template-columns:1fr}.radar-map{height:350px}.radar-transport{gap:8px;padding:10px}.radar-transport [data-time]{font-size:10px}.radar-plots{padding:10px}}
`;
document.head.append(style);

export function attachRadarReview(video) {
  const panel = document.createElement('section');
  panel.className = 'card card-pad radar-panel';
  panel.innerHTML = `<div class="eyebrow">Radar validation</div><h2>레이더 검증</h2>
    <div class="radar-toolbar"><label>레이더 소스<select data-sensor><option value="auto">자동</option><option value="front">전방</option><option value="corner">코너 포함</option></select></label>
    <label>표시 거리<select data-range><option>60</option><option selected>130</option><option>200</option></select></label>
    <button type="button" data-retry>다시 불러오기</button></div>
    <p class="radar-status" role="status" aria-live="polite"></p>
    <canvas class="radar-map" aria-label="레이더와 차선, 선행차를 위에서 본 화면"></canvas>
    <div class="radar-legend">● <span style="color:#56baff">전방</span> · <span style="color:#cd91ff">코너</span> · <span style="color:#ffb653">비전</span> / <span style="color:#67edc0">실선: 재계산 Lead</span> · <span style="color:#ffda70">점선: 기록된 Lead</span><br>레이더 점을 누르면 해당 트랙의 판정 근거를 볼 수 있습니다.</div>
    <pre class="radar-detail"></pre>`;
  const videoSection = video.closest('section');
  const review = document.createElement('section');
  review.className = 'radar-review';
  videoSection.before(review);
  const layout = document.createElement('div');
  layout.className = 'radar-layout';
  layout.append(videoSection, panel);
  review.append(layout);
  const transport = document.createElement('div');
  transport.className = 'card radar-transport';
  transport.innerHTML = `<button type="button" data-toggle-play disabled>재생</button>
    <input class="radar-scrub" type="range" min="0" max="0" value="0" step="0.01" aria-label="영상과 레이더 재생 위치" disabled>
    <span data-time>0.00 / 0.00 s</span>`;
  const plots = document.createElement('section');
  plots.className = 'card radar-plots';
  plots.innerHTML = `<div class="radar-plot-legend"><span style="color:#f68e37">L1 거리 (레이더)</span><span style="color:#f5d348">L2 거리</span><span style="color:#4891ff">비전 거리 / 비전 L1</span><span style="color:#f75ea0">SCC 거리·가속도</span><span style="color:#3ecd82">L1 속도·Carrot 목표 가속도</span></div>
    <canvas class="radar-graph" aria-label="선행차 거리와 속도, SCC와 Carrot 가속도 그래프. 누르면 재생 또는 일시정지"></canvas>
    <div class="radar-readout"></div>
    <p class="muted tiny">기록된 Lead는 업로드 당시 차량의 판정입니다. 재계산은 서버의 검증 코드로 실행하며 민감도는 3으로 고정됩니다. 레이더 소스 선택은 이 화면의 분석에만 적용됩니다.</p>`;
  review.append(transport, plots);
  video.controls = false;
  const find = s => review.querySelector(s);
  const map = find('.radar-map'), graph = find('.radar-graph'), scrub = find('.radar-scrub');
  const status = find('.radar-status'), play = find('[data-toggle-play]');
  let frames = [], times = [], payload = null, index = 0, segment = null;
  let generation = 0, controller = null, running = false, lastTick = 0, current = 0, selectedTrack = null;
  let hitPoints = [], frameHandle = 0;
  let graphTimes = new Map(), graphCacheKey = '';
  const graphBackground = document.createElement('canvas');
  const num = (v, digits=1) => Number.isFinite(v) ? v.toFixed(digits) : '—';
  const valid = v => typeof v === 'number' && Number.isFinite(v);
  const videoAvailable = () => Boolean(video.getAttribute('src')) && Number.isFinite(video.duration) && video.duration > 0;
  const usesVideo = () => videoAvailable() && (!payload || payload.videoAligned);
  const duration = () => frames.length ? times.at(-1) : (videoAvailable() ? video.duration : 0);
  function setStatus(message, error=false) { status.textContent = message; status.classList.toggle('radar-error', error); }
  function nearest(t) {
    let lo=0, hi=times.length;
    while(lo<hi){const mid=(lo+hi)>>1;if(times[mid]<=t)lo=mid+1;else hi=mid;}
    return Math.max(0,lo-1);
  }
  function canvasContext(canvas) {
    const w=canvas.clientWidth, h=canvas.clientHeight, ratio=Math.min(devicePixelRatio||1,2);
    if(canvas.width!==Math.round(w*ratio)||canvas.height!==Math.round(h*ratio)){canvas.width=Math.round(w*ratio);canvas.height=Math.round(h*ratio);}
    const ctx=canvas.getContext('2d');ctx.setTransform(ratio,0,0,ratio,0,0);ctx.clearRect(0,0,w,h);
    ctx.font='11px system-ui';return {ctx,w,h};
  }
  function drawMap(frame) {
    const {ctx,w,h}=canvasContext(map), range=Number(find('[data-range]').value);
    const xy=(d,y)=>[w/2-y*(w-50)/30,25+(range-d)*(h-50)/(range+30)];
    function line(points,color,width=1,dashed=false){ctx.beginPath();ctx.strokeStyle=color;ctx.lineWidth=width;ctx.setLineDash(dashed?[6,5]:[]);let started=false;for(const [d,y] of points||[]){if(!valid(d)||!valid(y))continue;const p=xy(d,y);if(!started){ctx.moveTo(...p);started=true;}else ctx.lineTo(...p);}ctx.stroke();ctx.setLineDash([]);}
    for(let d=-20;d<=range;d+=20){line([[d,-15],[d,15]],'#24313f');ctx.fillStyle='#8592a3';ctx.fillText(`${d} m`,8,xy(d,0)[1]-4);}
    for(const y of [-10,-5,0,5,10])line([[-30,y],[range,y]],'#182430');
    hitPoints=[];
    if(!frame)return;
    (frame.lane_lines||[]).forEach((points,i)=>line(points.map(([x,y])=>[x,-y]),`rgba(207,218,228,${Math.max(.15,frame.lane_probs[i]||0)})`,1,true));
    line(frame.path.map(([x,y])=>[x,-y]),'#3b9687',3);
    const [cx,cy]=xy(0,0);ctx.fillStyle='#dae3eb';ctx.fillRect(cx-7,cy-3,14,22);
    function ring(d,y,color,label,dashed=false){if(!valid(d)||!valid(y))return;const [x,z]=xy(d,y);ctx.strokeStyle=color;ctx.lineWidth=2;ctx.setLineDash(dashed?[4,3]:[]);ctx.strokeRect(x-10,z-10,20,20);ctx.setLineDash([]);ctx.fillStyle=color;ctx.fillText(label,x+13,z-6);}
    for(const p of frame.points||[]){if(p.d_rel < -30||p.d_rel>range||Math.abs(p.y_rel)>15)continue;const [x,y]=xy(p.d_rel,p.y_rel);const chosen=p.track_id===selectedTrack;ctx.beginPath();ctx.arc(x,y,chosen?6:3.5,0,Math.PI*2);ctx.fillStyle=p.source.startsWith('corner')?'#cd91ff':'#56baff';ctx.fill();ctx.fillText(String(p.track_id),x+6,y+12);hitPoints.push({x,y,id:p.track_id});}
    for(const lead of frame.model_leads||[]){if(lead.probability>=.1)ring(lead.x-payload.radarToCamera,-lead.y,'#ffb653',`V ${num(lead.probability,2)}`);}
    for(const [key,label] of [['recorded_one','R1'],['recorded_two','R2']]){const p=frame[key];if(p?.status)ring(p.d_rel,p.y_rel,'#ffda70',label,true);}
    for(const [key,label] of [['lead_one','L1'],['lead_two','L2']]){const p=frame.selection?.[key];if(p)ring(p.d_rel,p.y_rel,'#67edc0',label);}
  }
  function drawGraph() {
    const {ctx,w,h}=canvasContext(graph), end=duration()||1, range=Number(find('[data-range]').value);
    const left=44, right=w-44, top=28, bottom=h*.55, accelTop=bottom+44, accelBottom=h-28;
    const x=t=>left+Math.max(0,Math.min(1,t/end))*(right-left);
    const distanceY=d=>bottom-Math.max(0,Math.min(1,d/range))*(bottom-top);
    const speedY=v=>bottom-Math.max(0,Math.min(1,v/140))*(bottom-top);
    const accelY=a=>accelBottom-(Math.max(-4,Math.min(2,a))+4)/6*(accelBottom-accelTop);
    const key=[graph.width,graph.height,range,generation,frames.length,end].join(':');
    if(key!==graphCacheKey){
      graphBackground.width=graph.width;graphBackground.height=graph.height;
      const bg=graphBackground.getContext('2d');bg.setTransform(graph.width/w,0,0,graph.height/h,0,0);bg.font='11px system-ui';
      function grid(y,label,color='#8592a3',rightLabel=false){bg.strokeStyle='#283440';bg.beginPath();bg.moveTo(left,y);bg.lineTo(right,y);bg.stroke();bg.fillStyle=color;bg.textAlign=rightLabel?'left':'right';bg.fillText(label,rightLabel?right+6:left-6,y+4);bg.textAlign='left';}
      bg.fillStyle='#b9c7d6';bg.fillText('거리 (m)',left,16);bg.fillStyle='#3ecd82';bg.textAlign='right';bg.fillText('L1 속도 (km/h)',right,16);bg.textAlign='left';
      for(const d of [0,range/2,range])grid(distanceY(d),String(Math.round(d)));
      for(const v of [0,70,140]){bg.fillStyle='#3ecd82';bg.fillText(String(v),right+6,speedY(v)+4);}
      bg.fillStyle='#b9c7d6';bg.fillText('SCC 가속도 / Carrot 목표 가속도 (m/s²)',left,accelTop-16);
      for(const a of [-4,-2,0,2])grid(accelY(a),String(a));
      for(let i=0;i<=4;i++){const t=end*i/4;bg.fillStyle='#8592a3';bg.textAlign=i===0?'left':i===4?'right':'center';bg.fillText(`${num(t,1)} s`,x(t),h-8);}
      bg.textAlign='left';
      function series(name,y,width=1.7){
        for(const run of payload?.graphs?.[name]||[]){
          bg.strokeStyle=run.color;bg.fillStyle=run.color;bg.lineWidth=width;bg.beginPath();let started=false;
          for(const [t,value] of run.samples){const time=graphTimes.get(t);if(!valid(time)||!valid(value))continue;const px=x(time),py=y(value);if(started)bg.lineTo(px,py);else{bg.moveTo(px,py);started=true;}}
          bg.stroke();
          if(run.samples.length===1){const [t,value]=run.samples[0],time=graphTimes.get(t);if(valid(time)&&valid(value)){bg.beginPath();bg.arc(x(time),y(value),2,0,Math.PI*2);bg.fill();}}
        }
      }
      series('sccDistance',distanceY);series('leadOne',distanceY);series('leadTwo',distanceY);series('vision',distanceY);series('leadSpeed',speedY,1.4);
      series('sccAccel',accelY);series('carrotAccel',accelY,2.2);graphCacheKey=key;
    }
    ctx.drawImage(graphBackground,0,0,w,h);
    ctx.strokeStyle='#e1e7ed';ctx.beginPath();ctx.moveTo(x(current),top);ctx.lineTo(x(current),accelBottom);ctx.stroke();
  }
  function draw() {
    const frame=frames[index];drawMap(frame);drawGraph();
    const end=duration();scrub.max=String(end);scrub.disabled=!end;play.disabled=!end;
    scrub.value=String(current);find('[data-time]').textContent=`${num(current,2)} / ${num(end,2)} s`;
    play.textContent=(running||(usesVideo()&&!video.paused))?'일시정지':'재생';
    if(!frame){find('.radar-readout').textContent='';find('.radar-detail').textContent='';return;}
    const lead=p=>p?`#${p.track_id} · ${num(p.d_rel)} m · ${num(p.v_lead*3.6)} km/h`:'없음';
    find('.radar-readout').textContent=`차속 ${num(frame.v_ego*3.6)} km/h · 조향 ${num(frame.steering_angle_deg)}°\n재계산 L1 ${lead(frame.selection?.lead_one)}\n재계산 L2 ${lead(frame.selection?.lead_two)}\n기록된 L1 ${lead(frame.recorded_one?.status?frame.recorded_one:null)}\nSCC ${num(frame.scc_distance_m)} m · SCC 가속 ${num(frame.scc_a_req_raw,2)} · Carrot 목표 ${num(frame.carrot_a_target,2)} m/s²`;
    const candidates=frame.selection?.cutin_diagnostics||[], chosen=candidates.find(p=>p.track_id===selectedTrack);
    find('.radar-detail').textContent=chosen?`#${chosen.track_id} ${chosen.stage} · ${chosen.reason}\n${chosen.detail}`:candidates.filter(p=>['CUT-IN','RAW-CUTIN','PREDECEL'].includes(p.stage)).map(p=>`#${p.track_id} ${p.stage}: ${p.reason}`).join('\n');
  }
  function seek(t){if(!duration())return;current=Math.max(0,Math.min(t,duration()));index=nearest(current);if(usesVideo())video.currentTime=Math.min(current,video.duration);draw();}
  function tick(now){
    frameHandle=0;
    if(usesVideo()&&!video.paused){current=Math.min(Math.max(0,video.currentTime),duration());index=nearest(current);draw();}
    else if(running){current=Math.min(times.at(-1),current+(now-lastTick)/1000);index=nearest(current);if(current>=times.at(-1))running=false;draw();}
    lastTick=now;if(running||(usesVideo()&&!video.paused))frameHandle=requestAnimationFrame(tick);
  }
  function animate(){lastTick=performance.now();if(!frameHandle)frameHandle=requestAnimationFrame(tick);}
  function pause(){running=false;video.pause();draw();}
  function resume(){if(!duration())return;if(current>=duration())seek(0);if(usesVideo()){video.play().catch(()=>setStatus('영상을 재생하지 못했습니다. 다시 재생을 눌러 주세요.'));}else{running=true;animate();draw();}}
  function togglePlayback(){if(running||(usesVideo()&&!video.paused))pause();else resume();}
  play.onclick=togglePlayback;
  review.addEventListener('click',e=>{if(!e.target.closest('button,input,select,label,a'))togglePlayback();});
  video.addEventListener('play',animate);
  for(const event of ['loadedmetadata','durationchange','timeupdate','seeked','pause','ended','emptied'])video.addEventListener(event,()=>{if(usesVideo()){current=Math.max(0,Math.min(video.currentTime,duration()));index=nearest(current);}draw();});
  scrub.oninput=()=>seek(Number(scrub.value));
  map.onclick=e=>{const rect=map.getBoundingClientRect();let distance=18;selectedTrack=null;for(const p of hitPoints){const d=Math.hypot(e.clientX-rect.left-p.x,e.clientY-rect.top-p.y);if(d<distance){distance=d;selectedTrack=p.id;}}draw();};
  new ResizeObserver(draw).observe(review);
  find('[data-range]').onchange=draw;
  async function load(selected) {
    segment=selected;generation++;const token=generation;controller?.abort();controller=new AbortController();
    const signal=controller.signal;running=false;frames=[];times=[];payload=null;index=0;current=0;selectedTrack=null;scrub.disabled=true;draw();
    setStatus('레이더 검증 데이터를 준비하고 있습니다…');
    const query=new URLSearchParams({sensor:find('[data-sensor]').value});
    const url=`${location.pathname.replace(/\/$/,'')}/radar/${encodeURIComponent(selected.index)}?${query}`;
    try {
      const deadline=Date.now()+360000;
      while(token===generation){
        const response=await fetch(url,{signal,cache:'no-store'});
        if(response.status===202||response.status===503){if(Date.now()>deadline)throw new Error('분석 준비가 지연되고 있습니다. 잠시 후 다시 불러와 주세요.');await new Promise((resolve,reject)=>{const abort=()=>{clearTimeout(timer);reject(new DOMException('Aborted','AbortError'));};const timer=setTimeout(()=>{signal.removeEventListener('abort',abort);resolve();},2000);signal.addEventListener('abort',abort,{once:true});});continue;}
        if(!response.ok)throw new Error(response.status===404?'이 세그먼트에는 분석할 로그가 없습니다.':response.status===422?'레이더 분석에 실패했습니다. 로그 또는 서버 분석 환경을 확인해 주세요.':`레이더 데이터를 불러오지 못했습니다 (${response.status}).`);
        const data=await response.json();if(token!==generation)return;
        if(data.schemaVersion!==1||!Array.isArray(data.frames)||!data.frames.length)throw new Error('레이더 검증 데이터가 비어 있습니다.');
        data.videoAligned=data.videoAligned&&Boolean(selected.videoUrl);
        payload=data;frames=data.frames;times=frames.map(f=>data.videoAligned?f.video_time_s:f.time_s);
        graphTimes=new Map(frames.map((f,i)=>[f.time_s,times[i]]));graphCacheKey='';
        if(times.some((t,i)=>!valid(t)||(i&&t<times[i-1])))throw new Error('레이더 시간 정보가 올바르지 않습니다.');
        if(!data.videoAligned)video.pause();
        setStatus(`${selected.name} · ${data.sensor} · 민감도 ${data.sensitivity} · 코드 ${data.sourceVersion} · ${frames.length} 프레임${data.videoAligned?' · 영상 동기화':' · 영상 시간 정보 없음: 레이더 별도 재생'}${data.sourceLog?.startsWith('qlog')?' · qlog 사용 (희소 기록)':''}`);
        current=data.videoAligned?Math.max(0,Math.min(video.currentTime,times.at(-1))):0;index=nearest(current);draw();animate();return;
      }
    } catch(error){if(error.name!=='AbortError'&&token===generation){frames=[];times=[];payload=null;draw();setStatus(error.message,true);}}
  }
  find('[data-sensor]').onchange=()=>{if(segment)load(segment);};
  find('[data-retry]').onclick=()=>{if(segment)load(segment);};
  window.addEventListener('pagehide',()=>{controller?.abort();cancelAnimationFrame(frameHandle);running=false;});
  draw();return {load};
}
