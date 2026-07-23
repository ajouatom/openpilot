"use strict";

import { svg, setHidden, setText } from "../dom.js";
import { COLORS } from "../tokens.js";

const TOP = 248;
const BOTTOM = 346;

export function createLevelGauge(doc, {
  valueKey,
  label,
  normalColor = COLORS.carrot,
  lowColor = COLORS.limit,
  lowThreshold = 0.15,
  ariaLabel = label,
  hideWhenUnavailable = false,
}) {
  // viewBox 하단을 라벨(y=361) 아래로 넉넉히 두어 굵고 큰 라벨이 잘리지 않게 한다.
  const root = svg(doc, "svg", {
    class: "chud-level", viewBox: "0 216 62 156", role: "img", "aria-label": ariaLabel,
  });
  const frame = svg(doc, "rect", {
    class: "chud-level-frame", x: 1, y: TOP, width: 60, height: BOTTOM - TOP, rx: 18,
  });
  const fill = svg(doc, "rect", {
    class: "chud-level-fill", x: 9, y: BOTTOM - 8, width: 44, height: 0, rx: 13,
  });
  const valueText = svg(doc, "text", {
    class: "chud-level-value", x: 31, y: TOP - 16, "font-size": 20,
    "text-anchor": "middle", "dominant-baseline": "central",
  });
  const labelText = svg(doc, "text", {
    class: "chud-level-label", x: 31, y: BOTTOM + 15, "font-size": 20,
    "text-anchor": "middle", "dominant-baseline": "central",
  });
  setText(labelText, label);
  root.append(frame, fill, valueText, labelText);

  function update(data = {}) {
    const rawValue = data[valueKey];
    const value = Number(rawValue);
    const inRange = rawValue !== null && rawValue !== undefined && rawValue !== ""
      && Number.isFinite(value) && value >= 0 && value <= 1;
    const available = inRange && (!hideWhenUnavailable || value > 0);
    if (hideWhenUnavailable) {
      setHidden(root, !available);
      if (!available) return;
    }
    const ratio = available ? Math.max(0, Math.min(1, value)) : 0;
    const height = ratio * (BOTTOM - TOP - 16);
    const color = !available ? COLORS.muted : ratio <= lowThreshold ? lowColor : normalColor;
    fill.setAttribute("y", BOTTOM - 8 - height);
    fill.setAttribute("height", height);
    fill.setAttribute("fill", color);
    valueText.setAttribute("fill", color);
    setText(valueText, available ? `${Math.round(ratio * 100)}%` : "--%");
  }

  return { el: root, update };
}
