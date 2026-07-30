"use strict";

import { KAIGEN_FONT_URI } from "./assets.js";

/* 오버레이 자체 CSS. 모듈이 <style>로 1회 주입한다(css 빌드 배선 불필요, self-contained).
 * 컨테이너 쿼리(cqw)로 스테이지 크기에 스케일 → 세로에서 자동 축소.
 * 폰트는 토큰(--chud-font/-weight)으로 교체 가능. */
export const STYLE_ID = "carrot-hud-overlay-style";

export const CSS = `
@font-face{
  font-family:"CarrotClusterHud";
  src:url("${KAIGEN_FONT_URI}") format("truetype");
  font-display:swap;
  font-style:normal;
  font-weight:900;
}
/* 구 HUD 완전 배제 — 신규 오버레이가 대체. 기존 DOM은 호환 수명주기 루트로만 남기고
 * 실데이터 렌더는 신규 오버레이 하나가 담당한다. !important로 표시를 차단. */
#driveHudCard { display:none !important; }
/* 미니 HUD는 구 HUD가 아니라 별개 기능(웹설정 > HUD > 미니 HUD)이다. 평소에는 이 오버레이가
 * 화면을 차지하므로 함께 숨기되, 미니 HUD 모드가 실제로 켜졌을 때는 그 자체가 유일한 표시
 * surface이므로 차단하면 안 된다(차단하면 다른 UI는 visibility:hidden 이라 화면이 통째로 빈다). */
html:not([data-carrot-mini-hud="1"]) #carrotMiniHud { display:none !important; }

.chud{
  position:absolute; pointer-events:none; z-index:7; isolation:isolate;
  /* 영상 콘텐츠 사각형에 앵커(정사이즈/크롭/축소 버튼과 정합). 없으면 스테이지 전체. */
  left:var(--carrot-viewport-left, 0px);
  top:var(--carrot-viewport-top, 0px);
  width:var(--carrot-viewport-width, 100%);
  height:var(--carrot-viewport-height, 100%);
  container-type:size; container-name:chud;
  --chud-font:"CarrotClusterHud", "Segoe UI", system-ui, sans-serif;
  --chud-weight:900;
  --chud-white:#fff; --chud-carrot:#14bc68; --chud-limit:#de4840; --chud-amber:#f4ac36; --chud-stroke:#05090c;
  --chud-muted:#96a0ac;
  /* 코너 온도바: 높이를 토큰으로 정의해 좌하단 존이 정확히 그만큼 올라간다.
   * (line-height:1 이라 높이 = 글자 크기 + 상하 패딩) */
  --chud-devtemp-font:clamp(12px,2cqw,20px);
  --chud-devtemp-pad-block:clamp(2px,0.5cqw,5px);
  --chud-devtemp-pad-inline:clamp(5px,1.2cqw,11px);
  --chud-devtemp-height:calc(var(--chud-devtemp-font) + var(--chud-devtemp-pad-block) * 2);
}
.chud::before,.chud::after{
  content:"";position:absolute;left:0;right:0;z-index:0;pointer-events:none;
}
.chud::before{
  top:0;height:clamp(72px,16%,148px);
  background:linear-gradient(to bottom,rgba(0,0,0,.64) 0%,rgba(0,0,0,.30) 42%,rgba(0,0,0,0) 100%);
}
.chud::after{
  bottom:0;height:clamp(104px,24%,212px);
  background:linear-gradient(to bottom,rgba(0,0,0,0) 0%,rgba(0,0,0,.28) 42%,rgba(0,0,0,.74) 100%);
}
.chud[hidden],.chud [hidden]{display:none!important}
.chud-zone{position:absolute;z-index:1;display:flex}
.chud-zone--tl{left:0;top:0;padding:clamp(10px,3cqw,28px);
  flex-direction:column;align-items:flex-start;gap:clamp(10px,2.4cqw,22px)}
.chud-zone--tr{right:0;top:0;padding:clamp(10px,3cqw,28px);align-items:flex-start;
  gap:clamp(6px,1.4cqw,14px)}
/* 방향지시등/비상등은 최상단이 아니라 좌상단 제한속도 밴드와 상하 높이를 맞춘다.
 * (제한속도 중심 ≈ tl패딩+상단아이콘행+gap+표지반높이 → 아래 clamp가 그 밴드 중심을 추종) */
.chud-zone--tc{left:50%;top:0;transform:translateX(-50%);padding:clamp(8px,2cqw,20px);
  padding-top:clamp(54px,11cqw,108px);gap:clamp(10px,3cqw,26px)}
/* 온도바 높이만큼 들어올린다(패딩 대신 bottom 앵커 — degradation의 padding 단축
 * 지정이 예약분을 덮어쓰지 못한다). */
.chud-zone--bl{left:0;bottom:var(--chud-devtemp-height);padding:clamp(10px,3cqw,30px);
  flex-direction:column;align-items:flex-start;gap:clamp(6px,1.2cqw,12px)}
.chud-zone--br{right:clamp(98px,11cqw,122px);bottom:0;padding:clamp(10px,3cqw,30px)}
.chud-row{display:flex;align-items:center;gap:clamp(8px,1.6cqw,18px)}
/* 코너 스트립 — 화면 모서리에 완전 밀착(인셋 0). 존과 같은 z-index지만 DOM에서
 * 먼저 오므로 겹칠 때 항상 다른 HUD 아래에 깔린다. 배경 그라디언트보다는 위. */
.chud-corner{position:absolute;z-index:1;display:flex;pointer-events:none}
.chud-corner--bl{left:0;bottom:0}

/* 기기 CPU 온도 — 완전 검은 바 + 흰 글자. 좁아져도 숨기지 않는다. */
.chud-devtemp{
  font-family:var(--chud-font);font-weight:var(--chud-weight);
  font-size:var(--chud-devtemp-font);line-height:1;
  padding:var(--chud-devtemp-pad-block) var(--chud-devtemp-pad-inline);
  color:var(--chud-white);background:#000;
  font-variant-numeric:tabular-nums;letter-spacing:.01em;white-space:nowrap}

/* accel/steer 게이지 */
.chud-gauge-column{display:grid;grid-template-columns:minmax(0,1fr);gap:0}
.chud-gauge{width:clamp(40px,6cqw,74px);height:auto;display:block;
  filter:drop-shadow(0 2px 6px rgba(0,0,0,.5))}
.chud-gauge-frame{fill:rgba(0,0,0,0);stroke:rgba(240,244,248,.745);stroke-width:2}
.chud-gauge-mid{stroke:rgb(98,112,128);stroke-width:3}
.chud-gauge-value,.chud-gauge-label{font-family:var(--chud-font);font-weight:900;
  paint-order:stroke;stroke:var(--chud-stroke);stroke-width:2.4}
.chud-level{width:clamp(40px,6cqw,74px);height:auto;display:block;
  filter:drop-shadow(0 2px 6px rgba(0,0,0,.5))}
.chud-level-frame{fill:transparent;stroke:rgba(240,244,248,.745);stroke-width:2}
.chud-level-value,.chud-level-label{font-family:var(--chud-font);font-weight:900;
  paint-order:stroke;stroke:var(--chud-stroke);stroke-width:2.4}

/* TPMS */
.chud-tpms{width:clamp(84px,12cqw,120px);height:auto;display:block;
  filter:drop-shadow(0 2px 6px rgba(0,0,0,.55))}
.chud-tpms-car rect:first-child{fill:rgba(8,15,22,.88);stroke:rgba(235,242,248,.92);stroke-width:1.2}
.chud-tpms-car rect:nth-child(2){fill:rgba(105,214,242,.82)}
.chud-tpms-car rect:nth-child(n+3){fill:rgba(5,9,12,.96)}
.chud-tpms-value{font-family:var(--chud-font);font-weight:800;paint-order:stroke;
  stroke:var(--chud-stroke);stroke-width:2}

/* 방향지시등 — 점멸 */
.chud-turn{width:clamp(120px,28cqw,280px);height:auto;display:block;
  filter:drop-shadow(0 2px 6px rgba(0,0,0,.55))}
.chud-blinker{fill:#14bc68;stroke:#087641;stroke-width:3;stroke-linejoin:round;
  animation:chud-blink 0.9s steps(1,end) infinite}
@keyframes chud-blink{0%,50%{opacity:1}50.01%,100%{opacity:.12}}
@media (prefers-reduced-motion:reduce){.chud-blinker{animation:none;opacity:1}}

/* 신호등 */
.chud-tlight{width:clamp(26px,4.4cqw,40px);height:auto;display:block;margin-bottom:2px;
  filter:drop-shadow(0 2px 6px rgba(0,0,0,.55))}
.chud-tlight-halo{opacity:.28}
.chud-tlight-dot{stroke:rgba(255,255,255,.85);stroke-width:3}

/* LFA 아이콘의 flex 점유 폭은 모드와 무관하게 휠 크기로 고정한다.
 * 2x 레인 날개는 absolute overflow로만 그려 옆 Wi-Fi/시계가 움직이지 않는다. */
.chud-lfa{position:relative;display:flex;align-items:center;justify-content:center;
  --chud-lfa-size:clamp(28px,5.6cqw,56px);--chud-lfa-lane-width:clamp(56px,11.2cqw,112px);
  --chud-lfa-armed-opacity:.48;--chud-lfa-active-opacity:.78;
  width:var(--chud-lfa-size);height:var(--chud-lfa-size);flex:0 0 auto}
.chud-lfa-img{position:relative;z-index:1;height:var(--chud-lfa-size);width:var(--chud-lfa-size);display:block;object-fit:contain;
  transform-origin:50% 50%;transition:transform .12s linear,filter .2s,opacity .2s;
  filter:drop-shadow(0 2px 5px rgba(0,0,0,.55))}
.chud-lfa:not(.is-active) .chud-lfa-img{opacity:.5;filter:grayscale(.7) drop-shadow(0 2px 5px rgba(0,0,0,.55))}
/* 레인 초록 날개(carrot_wheel_lane 실루엣). 폭 2x·높이=휠(cluster LFA_LANE_ICON_WIDTH_SCALE=2),
 * 휠 뒤에 깔리고 좌우로 삐져나온다. 색=배경(활성 초록/비활성 muted), 모양=mask(JS가 URL 주입). */
.chud-lfa-lane{position:absolute;left:50%;top:50%;
  transform:translate(-50%,-56%);pointer-events:none;z-index:0;
  width:var(--chud-lfa-lane-width);height:var(--chud-lfa-size);
  background-color:var(--chud-muted);opacity:0;visibility:hidden;
  -webkit-mask-repeat:no-repeat;mask-repeat:no-repeat;
  -webkit-mask-position:center;mask-position:center;
  -webkit-mask-size:contain;mask-size:contain;
  filter:drop-shadow(0 2px 5px rgba(0,0,0,.5));
  transition:opacity .18s ease-out,background-color .18s ease-out,visibility 0s linear .18s}
.chud-lfa.has-lane .chud-lfa-lane{opacity:var(--chud-lfa-armed-opacity);visibility:visible;transition-delay:0s}
.chud-lfa.is-lane-active .chud-lfa-lane{opacity:var(--chud-lfa-active-opacity)}
.chud-lfa.is-lane-active.is-active .chud-lfa-lane{background-color:var(--chud-carrot);opacity:1}
@media (prefers-reduced-motion:reduce){.chud-lfa-lane{transition:none}}

/* WiFi */
.chud-wifi{height:clamp(24px,4.8cqw,48px);width:clamp(24px,4.8cqw,48px);display:block;color:#fff;
  filter:drop-shadow(0 2px 5px rgba(0,0,0,.6))}

/* 시계 */
.chud-clock{font-family:var(--chud-font);font-weight:var(--chud-weight);
  font-size:clamp(24px,4.8cqw,48px);line-height:1;color:#fff;letter-spacing:.01em;
  -webkit-text-stroke:2px var(--chud-stroke);paint-order:stroke fill;
  font-variant-numeric:tabular-nums;display:flex;align-items:center;height:clamp(24px,4.8cqw,48px)}

/* 제한속도 */
.chud-limit{width:clamp(46px,8cqw,84px);height:auto;display:block;
  margin-left:clamp(2px,1.2cqw,10px);filter:drop-shadow(0 3px 7px rgba(0,0,0,.5))}
.chud-limit-num{font-family:var(--chud-font);font-weight:900;fill:#0d1116}

/* 속도판 */
.chud-speed{display:block;width:clamp(180px,30cqw,384px);height:auto}
.chud-speed text{font-family:var(--chud-font);paint-order:stroke;
  stroke:var(--chud-stroke);stroke-linejoin:round}
.chud-t-speed{font-weight:var(--chud-weight);fill:#fff;stroke-width:3}
.chud-t-set{font-weight:var(--chud-weight);fill:var(--chud-carrot);stroke-width:2}
.chud-t-gap{font-weight:800;fill:#fff;stroke-width:2}
.chud-t-gear{font-weight:var(--chud-weight);fill:#fff;stroke-width:2}
.chud-t-set.is-muted,.chud-t-gap.is-muted,.chud-t-gear.is-muted{fill:var(--chud-muted)}
/* EV 텔테일(항상 초록) / 크루즈 오버라이드(색은 위젯이 mode별 인라인 지정) */
.chud-t-ev{font-weight:var(--chud-weight);fill:var(--chud-carrot);stroke-width:3}
.chud-t-override{font-weight:var(--chud-weight);stroke-width:2}
.chud-t-override-label{font-weight:800;stroke-width:2}
/* 주행모드 배지(연비/안전/일반/고속) — 색은 위젯이 mode별 인라인(fill), 배경 α는 여기서(200/255) */
.chud-drive-mode-box{fill-opacity:.784;stroke:#fff;stroke-width:2;stroke-linejoin:round}
/* 흰 글자 + 두꺼운 어두운 외곽선으로 어느 배경(흰 '일반' 포함)에서도 또렷·강조. */
.chud-drive-mode-label{font-family:var(--chud-font);font-weight:var(--chud-weight);fill:#fff;
  paint-order:stroke;stroke:var(--chud-stroke);stroke-width:3;stroke-linejoin:round;
  letter-spacing:.02em}
.chud-gear-box{fill:rgba(5,9,12,.82);stroke:#fff;stroke-width:3}
.chud-gear-box.is-muted{stroke:var(--chud-muted)}
.chud-speed-traffic{overflow:visible}

/* degradation: 좁으면 우선순위 낮은 것부터 숨김(P0 속도판은 항상 유지) */
.chud[data-shed~="p5"] .chud-zone--br{display:none}
.chud[data-shed~="p4"] .chud-zone--tr{display:none}
.chud[data-shed~="p3"] .chud-lfa,
.chud[data-shed~="p3"] .chud-wifi,
.chud[data-shed~="p2"] .chud-clock{display:none}
.chud[data-shed~="p1"] .chud-limit{display:none}

/* reposition: 축소 단계마다 남은 위젯의 여백/배율을 조여 겹침 방지(요건 a).
 * 숨김과 별개로, 좁아질수록 상단 아이콘/여백을 단계적으로 축소. P0 속도판은 유지. */
.chud[data-shed~="p3"] .chud-zone--tl{padding:clamp(8px,2.4cqw,20px);gap:clamp(7px,1.8cqw,16px)}
.chud[data-shed~="p2"] .chud-zone--bl{padding:clamp(8px,2.4cqw,18px)}
.chud[data-shed~="p1"] .chud-zone{padding:clamp(6px,2cqw,12px)}

/* 작은 영상 영역에서는 축소 한계만 소폭 키워 가독성을 확보한다.
 * 커진 결과가 다른 존을 침범하면 layout.js가 낮은 우선순위부터 제거한다. */
@container chud (max-width:520px){
  .chud-gauge,.chud-level{width:clamp(44px,16cqw,64px)}
  .chud-lfa{--chud-lfa-size:clamp(32px,12cqw,46px);--chud-lfa-lane-width:clamp(64px,24cqw,92px)}
  .chud-wifi{width:clamp(28px,10cqw,40px);height:clamp(28px,10cqw,40px)}
  .chud-clock{font-size:clamp(28px,10cqw,40px);height:clamp(28px,10cqw,40px)}
  .chud-limit{width:clamp(52px,16cqw,68px)}
  /* 이 구간은 제한속도 표지가 커져 밴드 중심이 내려가므로 topCenter 하강값을 보정. */
  .chud-zone--tc{padding-top:clamp(60px,15cqw,86px)}
  .chud-turn{width:clamp(132px,42cqw,200px)}
  .chud-speed{width:clamp(188px,68cqw,208px)}
}
`;

export function injectStyle(doc) {
  if (!doc || doc.getElementById(STYLE_ID)) return;
  const style = doc.createElement("style");
  style.id = STYLE_ID;
  style.textContent = CSS;
  (doc.head || doc.documentElement).appendChild(style);
}
