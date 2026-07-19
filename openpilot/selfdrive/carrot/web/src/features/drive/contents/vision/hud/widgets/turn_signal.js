"use strict";

import { svg } from "../dom.js";

function arrow(doc, side) {
  const left = side === "left";
  const cx = left ? 60 : 220;
  const point = (x, y) => `${cx + (left ? -x : x)} ${40 + y}`;
  const d = [
    `M ${point(-36, -16)}`,
    `L ${point(12, -16)}`,
    `L ${point(12, -38)}`,
    `L ${point(60, 0)}`,
    `L ${point(12, 38)}`,
    `L ${point(12, 16)}`,
    `L ${point(-36, 16)} Z`,
  ].join(" ");
  return svg(doc, "path", { class: `chud-blinker chud-blinker--${side}`, d });
}

export function createTurnSignal(doc) {
  const root = svg(doc, "svg", {
    class: "chud-turn", viewBox: "0 0 280 80", role: "img", "aria-label": "방향지시등",
  });
  const left = arrow(doc, "left");
  const right = arrow(doc, "right");
  root.append(left, right);

  function update(data = {}) {
    const l = data.leftBlinker === true;
    const r = data.rightBlinker === true;
    root.style.display = l || r ? "" : "none";
    left.style.display = l ? "" : "none";
    right.style.display = r ? "" : "none";
  }

  return { el: root, update };
}
