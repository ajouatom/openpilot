"use strict";

import { SPEED_BG_URI, TRAFFIC_GREEN_URI, TRAFFIC_RED_URI } from "../assets.js";
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
  const redLight = trafficIcon(doc, {
    uri: TRAFFIC_RED_URI,
    crop: "72 7 52 52",
    centerX: SPEED_X,
  });
  const greenLight = trafficIcon(doc, {
    uri: TRAFFIC_GREEN_URI,
    crop: "132 7 52 52",
    centerX: SPEED_X - 38,
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
  root.append(background, redLight, greenLight, speed, setSpeed, gap, gearBox, gear);

  // 설계 중심 cy + cap 보정으로 baseline(y) 설정. x/text-anchor는 그대로(가로 중앙).
  function place(el, cy, fs) {
    el.setAttribute("font-size", String(fs));
    el.setAttribute("y", String(cy + CAP * fs));
  }

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
  }

  return { el: root, update };
}
