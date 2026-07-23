"use strict";

import { svg, setText } from "../dom.js";

const W = 62;
const TOP = 88;
const BOTTOM = 186;
const MID = (TOP + BOTTOM) / 2;
const INNER_X = 3;
const INNER_TOP = 90;
const INNER_BOTTOM = 184;
const INNER_WIDTH = 56;
const INNER_RADIUS = 16;
const VALUE_Y = 64;
const LABEL_Y = 201;
let gaugeClipSequence = 0;

export function createBipolarGauge(doc, { label, ariaLabel }) {
  const root = svg(doc, "svg", {
    class: "chud-gauge",
    viewBox: "0 47 62 169",
    role: "img",
    "aria-label": ariaLabel || label,
  });
  const clipId = `chud-gauge-clip-${++gaugeClipSequence}`;
  const defs = svg(doc, "defs", {}, [
    svg(doc, "clipPath", { id: clipId }, [
      svg(doc, "rect", {
        x: INNER_X,
        y: INNER_TOP,
        width: INNER_WIDTH,
        height: INNER_BOTTOM - INNER_TOP,
        rx: INNER_RADIUS,
      }),
    ]),
  ]);
  const frame = svg(doc, "rect", {
    class: "chud-gauge-frame", x: 1, y: TOP, width: 60, height: BOTTOM - TOP, rx: 18,
  });
  const midline = svg(doc, "line", {
    class: "chud-gauge-mid", x1: 1, y1: MID, x2: 61, y2: MID,
  });
  const fill = svg(doc, "rect", {
    class: "chud-gauge-fill",
    x: INNER_X,
    width: INNER_WIDTH,
    "clip-path": `url(#${clipId})`,
  });
  const valueText = svg(doc, "text", {
    class: "chud-gauge-value", x: 31, y: VALUE_Y, "font-size": 20,
    "text-anchor": "middle", "dominant-baseline": "central",
  });
  const labelText = svg(doc, "text", {
    class: "chud-gauge-label", x: 31, y: LABEL_Y, "font-size": 20,
    "text-anchor": "middle", "dominant-baseline": "central", fill: "#fff",
  });
  setText(labelText, label);
  root.append(defs, frame, fill, midline, valueText, labelText);

  function update(data = {}) {
    const has = data.ratio != null && Number.isFinite(Number(data.ratio));
    const ratio = has ? Math.max(-1, Math.min(1, Number(data.ratio))) : 0;
    const color = data.color || "#96a0ac";
    const maxHeight = (INNER_BOTTOM - INNER_TOP) / 2;
    const height = Math.abs(ratio) * maxHeight;
    fill.setAttribute("y", ratio >= 0 ? MID - height : MID);
    fill.setAttribute("height", height);
    fill.setAttribute("fill", height < 0.5 ? "transparent" : color);
    valueText.setAttribute("fill", color);
    setText(valueText, data.text == null ? "--" : data.text);
  }

  return { el: root, update };
}
