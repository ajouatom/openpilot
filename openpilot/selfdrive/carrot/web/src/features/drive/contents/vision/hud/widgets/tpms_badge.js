"use strict";

import { svg, setText } from "../dom.js";
import { COLORS } from "../tokens.js";

const LOW_PRESSURE_PSI = 31;

export function createTpmsBadge(doc) {
  const root = svg(doc, "svg", {
    class: "chud-tpms", viewBox: "0 0 80 68", role: "img", "aria-label": "Tire pressure",
  });
  const carX = 32.5;
  const carY = 17;
  const car = svg(doc, "g", { class: "chud-tpms-car" }, [
    svg(doc, "rect", { x: carX, y: carY, width: 15, height: 34, rx: 5 }),
    svg(doc, "rect", { x: carX + 3, y: carY + 5, width: 9, height: 10, rx: 3 }),
    svg(doc, "rect", { x: 30, y: 22, width: 3, height: 8, rx: 1.2 }),
    svg(doc, "rect", { x: 47, y: 22, width: 3, height: 8, rx: 1.2 }),
    svg(doc, "rect", { x: 30, y: 38, width: 3, height: 8, rx: 1.2 }),
    svg(doc, "rect", { x: 47, y: 38, width: 3, height: 8, rx: 1.2 }),
  ]);
  const positions = {
    fl: [10, 20],
    fr: [70, 20],
    rl: [10, 48],
    rr: [70, 48],
  };
  const values = Object.fromEntries(Object.entries(positions).map(([key, [x, y]]) => [
    key,
    svg(doc, "text", {
      class: "chud-tpms-value", x, y, "font-size": 16,
      "text-anchor": "middle", "dominant-baseline": "central",
    }),
  ]));
  root.append(car, values.fl, values.fr, values.rl, values.rr);

  function update(data = {}) {
    const tpms = data.tpms && typeof data.tpms === "object" ? data.tpms : {};
    for (const [key, text] of Object.entries(values)) {
      const pressure = Number(tpms[key]);
      const valid = Number.isFinite(pressure) && pressure >= 5 && pressure <= 100;
      text.setAttribute("fill", !valid ? COLORS.muted : pressure < LOW_PRESSURE_PSI ? COLORS.limit : COLORS.white);
      setText(text, valid ? Math.round(pressure) : "--");
    }
  }

  return { el: root, update };
}
