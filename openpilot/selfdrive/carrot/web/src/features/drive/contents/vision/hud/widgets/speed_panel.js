"use strict";

import { SPEED_BG_URI, TRAFFIC_GREEN_URI, TRAFFIC_RED_URI } from "../assets.js";
import { COLORS } from "../tokens.js";
import { svg, setText } from "../dom.js";

// 숫자 정중앙 보정: baseline = 설계 중심 + cap-height 절반(0.35em). 폰트별 미세조정.
const CAP = 0.35;

// Exact cluster design coordinates. The browser only scales this viewBox; it
// never recomputes or approximates the widget's internal geometry.
const VIEWBOX = "0 270 384 140";
const SPEED_X = 85.2;
const SPEED_Y = 351;
const SET_SPEED_X = 249.2;
const SET_SPEED_Y = 339;
const GAP_X = 297.68;
const GAP_Y = 389.856;
const GEAR_X = 356;
const GEAR_Y = 379.6;
// Cluster parity (cluster_renderer.py, SPEED_HUD_SCALE=0.8 already folded in like
// the other panel coords). EV telltale sits between the vehicle and set speeds;
// the cruise override rides the panel's top-right above the set speed.
const EV_X = 181;      // SPEED_EV_CENTER_X
const EV_Y = 351;      // SPEED_EV_CENTER_Y = SPEED_VALUE_CENTER_Y
const EV_FS = 28;      // SPEED_EV_FONT_SIZE
const OVERRIDE_X = 331.6;       // SPEED_PANEL_X + 392*0.8
const OVERRIDE_SPEED_Y = 319;   // SPEED_PANEL_Y + 30*0.8
const OVERRIDE_LABEL_Y = 291.8; // SPEED_PANEL_Y - 4*0.8
const OVERRIDE_SPEED_FS = 41.6; // 52*0.8
const OVERRIDE_LABEL_FS = 24;   // 가독성 위해 클러스터(25*0.8=20)보다 소폭 확대

function trafficIcon(doc, { uri, crop, centerX }) {
  const size = 34;
  const root = svg(doc, "svg", {
    class: "chud-speed-traffic",
    x: centerX - size * 0.5,
    y: 286 - size * 0.5,
    width: size,
    height: size,
    viewBox: crop,
    preserveAspectRatio: "xMidYMid meet",
  });
  root.append(svg(doc, "image", {
    href: uri,
    x: 0,
    y: 0,
    width: 256,
    height: 256,
  }));
  return root;
}

export function createSpeedPanel(doc) {
  const root = svg(doc, "svg", {
    class: "chud-speed",
    viewBox: VIEWBOX,
    role: "img",
    "aria-label": "속도",
  });
  const background = svg(doc, "image", {
    class: "chud-speed-bg-image",
    href: SPEED_BG_URI,
    x: 18,
    y: 295,
    width: 304,
    height: 113.6,
    opacity: 235 / 255,
    preserveAspectRatio: "none",
  });
  // 신호등은 한 번에 하나만 켜지므로 빨강/초록을 같은 자리에 겹쳐 둔다(토글 시 위치 안 바뀜).
  const TRAFFIC_CENTER_X = SPEED_X - 38;
  const redLight = trafficIcon(doc, {
    uri: TRAFFIC_RED_URI,
    crop: "72 7 52 52",
    centerX: TRAFFIC_CENTER_X,
  });
  const greenLight = trafficIcon(doc, {
    uri: TRAFFIC_GREEN_URI,
    crop: "132 7 52 52",
    centerX: TRAFFIC_CENTER_X,
  });
  const speed = svg(doc, "text", {
    class: "chud-t-speed",
    x: SPEED_X,
    y: SPEED_Y,
    "font-size": 89.6,
    "text-anchor": "middle",  });
  const setSpeed = svg(doc, "text", {
    class: "chud-t-set",
    x: SET_SPEED_X,
    y: SET_SPEED_Y,
    "font-size": 46.4,
    "text-anchor": "middle",  });
  const gap = svg(doc, "text", {
    class: "chud-t-gap",
    x: GAP_X,
    y: GAP_Y,
    "font-size": 28,
    "text-anchor": "middle",  });
  const gearBox = svg(doc, "rect", {
    class: "chud-gear-box",
    x: GEAR_X - 22.5,
    y: GEAR_Y - 29,
    width: 45,
    height: 58,
    rx: 8,
  });
  const gear = svg(doc, "text", {
    class: "chud-t-gear",
    x: GEAR_X,
    y: GEAR_Y,
    "font-size": 48,
    "text-anchor": "middle",  });
  const ev = svg(doc, "text", {
    class: "chud-t-ev",
    x: EV_X,
    y: EV_Y,
    "font-size": EV_FS,
    "text-anchor": "middle",  });
  const overrideLabel = svg(doc, "text", {
    class: "chud-t-override-label",
    x: OVERRIDE_X,
    y: OVERRIDE_LABEL_Y,
    "font-size": OVERRIDE_LABEL_FS,
    "text-anchor": "middle",  });
  const overrideSpeed = svg(doc, "text", {
    class: "chud-t-override",
    x: OVERRIDE_X,
    y: OVERRIDE_SPEED_Y,
    "font-size": OVERRIDE_SPEED_FS,
    "text-anchor": "middle",  });
  root.append(background, redLight, greenLight, speed, setSpeed, gap, gearBox, gear, ev, overrideLabel, overrideSpeed);

  // 설계 중심 cy + cap 보정으로 baseline(y) 설정. x/text-anchor는 그대로(가로 중앙).
  function place(el, cy, fs) {
    el.setAttribute("font-size", String(fs));
    el.setAttribute("y", String(cy + CAP * fs));
  }

  // 오버라이드 마지막 값(비활성 시 회색으로 유지하기 위함).
  let lastOverride = null;

  function update(data = {}) {
    const speedValue = Number(data.speed);
    const speedText = Number.isFinite(speedValue) ? String(Math.max(0, Math.round(speedValue))) : "0";
    place(speed, SPEED_Y, speedText.length <= 2 ? 89.6 : 89.6 * 0.86);
    setText(speed, speedText);

    const hasSetSpeed = data.setSpeed != null && data.setSpeed !== "";
    place(setSpeed, SET_SPEED_Y, 46.4);
    setText(setSpeed, hasSetSpeed ? data.setSpeed : "--");
    setSpeed.classList.toggle("is-muted", !hasSetSpeed);

    const hasGap = data.gap != null && data.gap !== "";
    place(gap, GAP_Y, 28);
    setText(gap, hasGap ? data.gap : "--");
    gap.classList.toggle("is-muted", !hasGap);

    const hasGear = data.gear != null && data.gear !== "";
    place(gear, GEAR_Y, 48);
    setText(gear, hasGear ? data.gear : "--");
    gear.classList.toggle("is-muted", !hasGear);
    gearBox.classList.toggle("is-muted", !hasGear);

    const trafficState = Number(data.trafficState);
    redLight.style.display = trafficState === 1 ? "" : "none";
    greenLight.style.display = trafficState === 2 ? "" : "none";

    const evActive = data.evActive === true;
    ev.style.display = evActive ? "" : "none";
    if (evActive) {
      place(ev, EV_Y, EV_FS);
      setText(ev, "EV");
    }

    // 오버라이드: 비활성/사라지는 조건에서도 숨기지 않고 "마지막 값을 회색"으로 유지(요청).
    const override = data.cruiseOverride;
    const overrideKph = override != null ? Number(override.kph) : null;
    const active = Number.isFinite(overrideKph) && overrideKph > 0;
    if (active) {
      lastOverride = { kph: Math.round(overrideKph), label: override.label || "", mode: override.mode };
    }
    const show = lastOverride != null;
    overrideSpeed.style.display = show ? "" : "none";
    overrideLabel.style.display = show ? "" : "none";
    if (show) {
      // 활성: mode 1 eco(초록) / 2 감속(주황) / 그 외 흰색. 비활성: 회색(muted).
      const color = !active
        ? COLORS.muted
        : lastOverride.mode === 1 ? COLORS.carrot : lastOverride.mode === 2 ? COLORS.override : COLORS.white;
      place(overrideSpeed, OVERRIDE_SPEED_Y, OVERRIDE_SPEED_FS);
      setText(overrideSpeed, String(lastOverride.kph));
      overrideSpeed.style.fill = color;
      place(overrideLabel, OVERRIDE_LABEL_Y, OVERRIDE_LABEL_FS);
      setText(overrideLabel, lastOverride.label);
      overrideLabel.style.fill = color;
    }
  }

  return { el: root, update };
}
