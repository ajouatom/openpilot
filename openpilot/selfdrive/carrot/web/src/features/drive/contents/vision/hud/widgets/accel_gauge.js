"use strict";

import { createBipolarGauge } from "./bipolar_gauge.js";
import { COLORS } from "../tokens.js";

/* accel 게이지: 종가속(m/s²). 클러스터 MAX_ACCEL_MPS2와 동일. */
const MAX_ACCEL = 5;

export function createAccelGauge(doc) {
  const gauge = createBipolarGauge(doc, { label: "accel", ariaLabel: "가속도" });

  function update(data = {}) {
    const accel = data.accel;
    if (accel == null || !Number.isFinite(Number(accel))) {
      gauge.update({ ratio: 0, color: COLORS.muted, text: "--" });
      return;
    }
    const value = Number(accel);
    const ratio = value / MAX_ACCEL;
    const color = value > 0.02 ? COLORS.carrot : value < -0.02 ? COLORS.limit : COLORS.muted;
    gauge.update({ ratio, color, text: `${value >= 0 ? "+" : ""}${value.toFixed(2)}` });
  }

  return { el: gauge.el, update };
}
