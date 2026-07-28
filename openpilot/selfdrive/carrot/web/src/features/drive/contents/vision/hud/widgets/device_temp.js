"use strict";

import { el, setText } from "../dom.js";

/* 기기 CPU 온도 — 좌하단 코너 밀착 다크바.
 * 값은 가장 뜨거운 코어(max), 소수점 1자리. CPU 온도는 클러스터/네이티브와 같이
 * 항상 섭씨다(속도류처럼 isMetric으로 단위가 바뀌지 않는다).
 * 데이터가 없어도 숨기지 않고 자리표시자를 유지한다(미니 HUD와 동일한 규칙). */
const PLACEHOLDER = "--°C";

export function formatDeviceTemp(value) {
  // Number(null) is 0, so an absent reading has to be rejected before coercion.
  if (value === null || value === undefined || value === "") return PLACEHOLDER;
  const temp = Number(value);
  return Number.isFinite(temp) ? `${temp.toFixed(1)}°C` : PLACEHOLDER;
}

export function createDeviceTemp(doc) {
  const root = el(doc, "div", {
    class: "chud-devtemp",
    text: PLACEHOLDER,
    attrs: { role: "status", "aria-label": "CPU 온도" },
  });

  function update(data = {}) {
    setText(root, formatDeviceTemp(data.cpuTemp));
  }

  return { el: root, update };
}
