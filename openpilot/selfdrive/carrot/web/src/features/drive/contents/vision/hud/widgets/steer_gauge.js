"use strict";

import { createBipolarGauge } from "./bipolar_gauge.js";
import { COLORS } from "../tokens.js";

/* steer 게이지: 조향 출력(정규화 -1..1). 우(+)=BLUE, 좌(-)=AMBER. 클러스터 동작. */
export function createSteerGauge(doc) {
  const gauge = createBipolarGauge(doc, { label: "steer", ariaLabel: "조향출력" });

  function update(data = {}) {
    const out = data.steerOutput;
    if (out == null || !Number.isFinite(Number(out))) {
      gauge.update({ ratio: 0, color: COLORS.muted, text: "--" });
      return;
    }
    const ratio = Math.max(-1, Math.min(1, Number(out)));
    const color = ratio > 0.01 ? COLORS.blue : ratio < -0.01 ? COLORS.amber : COLORS.muted;
    gauge.update({ ratio, color, text: `${ratio >= 0 ? "+" : ""}${Math.round(ratio * 100)}%` });
  }

  return { el: gauge.el, update };
}
