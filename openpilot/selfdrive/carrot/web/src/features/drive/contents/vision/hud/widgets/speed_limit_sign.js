"use strict";

import { svg, setText, setHidden } from "../dom.js";

// 숫자 정중앙 보정: baseline을 cap-height 절반만큼 내려 시각 중심을 cy에 맞춤.
// cap-height ≈ 0.7em → 보정 0.35em. 폰트에 따라 미세조정.
const CAP = 0.35;

export function createSpeedLimitSign(doc) {
  const root = svg(doc, "svg", {
    class: "chud-limit", viewBox: "0 0 84 84", role: "img", "aria-label": "제한속도",
  });
  const num = svg(doc, "text", {
    class: "chud-limit-num", x: 42, y: 42, "font-size": 42,
    "text-anchor": "middle",
  });
  root.append(
    svg(doc, "circle", { cx: 42, cy: 42, r: 42, fill: "var(--chud-limit)" }),
    svg(doc, "circle", { cx: 42, cy: 42, r: 36, fill: "#fff" }),
    num,
  );

  function update(data = {}) {
    const value = Number(data.speedLimit);
    // 주행 컨텍스트에서만 표시(idle/주차 시 지도 도로제한 노출 방지).
    const driving = Number(data.speed) > 0 || data.lfaActive === true;
    const has = Number.isFinite(value) && value > 0 && driving;
    setHidden(root, !has);
    if (!has) return;
    const text = String(Math.round(value));
    const fs = text.length <= 2 ? 42 : 36;
    num.setAttribute("font-size", String(fs));
    num.setAttribute("y", String(42 + CAP * fs)); // 42 = 원 중심 cy
    setText(num, text);
  }

  return { el: root, update };
}
