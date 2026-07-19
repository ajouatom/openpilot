"use strict";

import { el, setText } from "../dom.js";

/* 시계: HH:MM:SS. 외곽선(-webkit-text-stroke)로 대비 확보.
 * data.clock 있으면 사용, 없으면 로컬 시간 틱. */
export function createClock(doc) {
  const root = el(doc, "div", { class: "chud-clock", attrs: { role: "timer", "aria-label": "시계" } });
  let timer = 0;

  function localTime() {
    const d = new Date();
    const p = (n) => String(n).padStart(2, "0");
    return `${p(d.getHours())}:${p(d.getMinutes())}:${p(d.getSeconds())}`;
  }

  function update(data = {}) {
    if (data.clock) setText(root, data.clock);
    else setText(root, localTime());
  }

  function start() {
    if (timer) return;
    update();
    timer = setInterval(() => { if (!root.textContent || root.dataset.local !== "0") update(); }, 1000);
    root.dataset.local = "1";
  }

  function stop() { if (timer) { clearInterval(timer); timer = 0; } }

  return { el: root, update, start, stop };
}
