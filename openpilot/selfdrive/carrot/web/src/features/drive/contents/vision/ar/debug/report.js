"use strict";

/* 진단 이력을 "집계"로 요약한다.
 *
 * 프레임마다 한 줄씩 쌓으면 수백 줄이 되어 붙여넣기가 잘린다. 그런데 그 줄들은
 * 대부분 같은 내용의 반복이다. 그래서 상태·사유·스킵을 종류별로 세고, 수치는
 * 범위(min~max)로만 남긴다. 길이는 수십 분의 1이 되고 정보량은 오히려 늘어난다.
 */

import { timestamp } from "./format.js";

function bump(map, key) {
  if (!key) return;
  map.set(key, (map.get(key) || 0) + 1);
}

function span(range, value) {
  const n = Number(value);
  if (!Number.isFinite(n)) return;
  range.min = range.min === null ? n : Math.min(range.min, n);
  range.max = range.max === null ? n : Math.max(range.max, n);
}

function rangeText(range, unit = "") {
  if (range.min === null) return "—";
  return range.min === range.max
    ? `${range.min}${unit}`
    : `${range.min}${unit} ~ ${range.max}${unit}`;
}

/** 많이 나온 순으로 "내용 xN". */
function topLines(map, limit = 6) {
  return [...map.entries()]
    .sort((a, b) => b[1] - a[1])
    .slice(0, limit)
    .map(([key, count]) => `  ${key}  x${count}`);
}

export function createArDebugAggregate(options = {}) {
  const now = typeof options.now === "function" ? options.now : () => Date.now();
  const startedAt = now();

  const states = new Map();
  const reasons = new Map();
  const skips = new Map();
  const gates = new Map();
  const holds = new Map();
  const tracks = new Map();
  const anchorX = { min: null, max: null };
  const anchorY = { min: null, max: null };
  const counts = { sign: 0, anchored: 0, drawn: 0, frames: 0, withAnchor: 0, withDraw: 0 };
  let fixed = null;
  let last = null;

  function observe(view) {
    counts.frames += 1;
    bump(states, view.badge);
    bump(reasons, view.reason);
    bump(gates, view.gate);
    bump(holds, view.hold);
    bump(tracks, view.track);
    for (const skip of view.skips || []) bump(skips, skip);
    if (view.anchor) { span(anchorX, view.anchor.x); span(anchorY, view.anchor.y); }
    if (Number(view.anchored) > 0) counts.withAnchor += 1;
    if (Number(view.drawn) > 0) counts.withDraw += 1;
    counts.sign = Math.max(counts.sign, Number(view.sign) || 0);
    counts.anchored = Math.max(counts.anchored, Number(view.anchored) || 0);
    counts.drawn = Math.max(counts.drawn, Number(view.drawn) || 0);
    // 변하지 않는 환경 정보는 마지막 값 하나만 남긴다.
    fixed = view.fixed || fixed;
    last = view;
  }

  function report() {
    const elapsed = now() - startedAt;
    const out = [
      `AR 진단 요약  ${timestamp(elapsed)} 동안 ${counts.frames}프레임`,
      `현재  ${last ? `${last.badge} ${last.sign}·${last.anchored}·${last.drawn} ${last.track} ${last.hold}` : "—"}`,
      `도달  앵커생성 ${counts.withAnchor}프레임 / 렌더 ${counts.withDraw}프레임 / 최대 ${counts.sign}·${counts.anchored}·${counts.drawn}`,
      `앵커  x ${rangeText(anchorX, "m")} · y ${rangeText(anchorY, "m")}`,
    ];
    if (fixed) out.push(`환경  ${fixed}`);
    const blocks = [
      ["상태", states], ["track", tracks], ["hold", holds], ["gate", gates],
      ["렌더 스킵", skips], ["차단 사유", reasons],
    ];
    for (const [label, map] of blocks) {
      if (!map.size) continue;
      out.push(`─ ${label} (${map.size}종)`);
      out.push(...topLines(map));
    }
    return out.join("\n");
  }

  return Object.freeze({
    observe,
    report,
    get frames() { return counts.frames; },
    reset() {
      states.clear(); reasons.clear(); skips.clear();
      gates.clear(); holds.clear(); tracks.clear();
      anchorX.min = anchorX.max = anchorY.min = anchorY.max = null;
      counts.frames = counts.withAnchor = counts.withDraw = 0;
      counts.sign = counts.anchored = counts.drawn = 0;
      last = null;
    },
  });
}
