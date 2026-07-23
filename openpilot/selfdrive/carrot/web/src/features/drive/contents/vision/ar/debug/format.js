"use strict";

/* AR 진단 값 포맷터 — 순수 함수만. DOM도 상태도 모른다.
 *
 * fields.js의 선언이 여기 함수를 이름으로 참조한다. 포맷 규칙을 한곳에 모아
 * 패널·로그·JSON이 같은 문자열을 쓰게 한다.
 */

export const TONE = Object.freeze({
  OK: "ok", WARN: "warn", IDLE: "idle", OFF: "off",
});

export function num(value, fallback = null) {
  const n = Number(value);
  return Number.isFinite(n) ? n : fallback;
}

export function drawnCount(status) {
  const worker = num(status?.worker?.renderer?.drawn);
  return worker === null ? (num(status?.renderer?.drawn, 0) ?? 0) : worker;
}

/** 켜짐/꺼짐 한 글자 묶음. flags({G:true,R:false}) → "G-" */
export function flags(map) {
  return Object.entries(map).map(([k, on]) => (on ? k : "-")).join("");
}

export function metres(value, digits = 0) {
  const n = num(value);
  return n === null ? "—" : `${n.toFixed(digits)}m`;
}

export function onOff(value) {
  return value === true ? "ON" : "off";
}

/** 파이프라인 어디까지 왔는지 한 줄 판정. 패널 헤더와 로그 서명이 공유한다. */
export function summarize(status) {
  if (!status || status.active !== true) {
    return { tone: TONE.OFF, label: "AR 꺼짐", hint: "주행화면 비활성" };
  }
  const sign = num(status.composition?.signCount, 0) ?? 0;
  const anchored = num(status.composition?.anchoredCount, 0) ?? 0;
  const drawn = drawnCount(status);
  if (drawn > 0) return { tone: TONE.OK, label: "표시중", hint: `그림 ${drawn}` };
  if (anchored > 0) return { tone: TONE.WARN, label: "앵커O·렌더X", hint: "렌더러가 못 그림" };
  if (sign > 0) return { tone: TONE.WARN, label: "표지O·앵커X", hint: "앵커 생성 실패" };
  return { tone: TONE.IDLE, label: "표지 없음", hint: "안내/후보 없음" };
}

/** compose가 워커 안에서 본 입력. 앵커 실패 원인이 여기서 드러난다. */
export function composeGate(diag) {
  if (!diag) return "—";
  const parts = [
    `gate:${flags({ G: diag.geo, R: diag.route, P: diag.precise })}`,
    `pts:${diag.routePts}`,
    `veh:${diag.veh}${diag.head}`,
    `match:${diag.matchIdx ?? "-"}`,
    `path:${diag.pathM === null || diag.pathM === undefined ? "-" : `${diag.pathM}m`}`,
  ];
  if (Array.isArray(diag.fails) && diag.fails.length) {
    parts.push(`fail:${diag.fails.map((f) => `${f.src}@${f.dist}m`).join(",")}`);
  }
  return parts.join(" ");
}

/** 첫 앵커의 자차 기준 좌표. 화면 밖으로 나갔는지 가릴 때 본다. */
export function anchorSample(sample) {
  if (!sample) return "—";
  return `x${sample.x} y${sample.y} z${sample.z}`;
}

export function rendererLine(status) {
  const r = status?.worker?.renderer || status?.renderer || null;
  if (!r) return "—";
  const bits = [`draw ${num(r.drawn, 0)}`, `skip ${num(r.skipped, 0)}`];
  if (r.lastReason) bits.push(String(r.lastReason));
  // 표지가 왜 안 그려졌는지는 skips에만 있다("투영 실패", "표시 억제" 등).
  if (Array.isArray(r.skips) && r.skips.length) bits.push(`[${r.skips.join(" | ")}]`);
  return bits.join(" · ");
}

export function canvasLine(status) {
  const w = status?.worker;
  if (!w) return "—";
  const v = w.viewport || {};
  return `${w.canvas || "—"} @ ${num(v.left, 0)},${num(v.top, 0)}`;
}

export function calibrationLine(status, diagnose) {
  const cal = diagnose?.calStatusRaw;
  const plausible = status?.sync?.calibrationPlausible;
  const calibrated = status?.sync?.calibrated;
  return `cal:${cal ?? "—"} ${flags({ C: calibrated === true, P: plausible === true })}`;
}

/** 가장 설명력 있는 사유 하나. tracking > problems > sync 순. */
export function primaryReason(status, diagnose) {
  const drawn = drawnCount(status);
  return status?.tracking?.reasons?.[0]
    || (Array.isArray(diagnose?.problems) && diagnose.problems.length ? diagnose.problems[0] : null)
    || status?.sync?.reasons?.[0]
    || (drawn > 0 ? "" : summarize(status).hint);
}

/* ---------- 소스/센서 ---------- */

export function sourceFrames(status, diagnose) {
  const cam = diagnose?.camFrameId ?? "—";
  const model = diagnose?.modelFrameId ?? "—";
  return `cam ${cam} · model ${model}`;
}

export function sourceAges(status) {
  const s = status?.sync;
  if (!s) return "—";
  return ["model", "odometry", "pose", "navi"]
    .map((k) => `${k[0]}${num(s[`${k}AgeMs`], "—")}`)
    .join(" ");
}

export function naviState(diagnose) {
  const n = diagnose?.navi;
  if (!n) return "—";
  return `${flags({ P: n.present, C: n.connected, G: n.guidanceActive, R: n.routePresent })}`
    + `${n.offRoute ? " offRoute" : ""} age${num(n.ageMs, "—")}`;
}

/* ---------- 위치 품질 ---------- */

export function positionQualityLine(status) {
  const q = status?.positionQuality;
  if (!q) return "—";
  const bits = [
    `geo:${q.canUseGeo ? "ON" : "off"}`,
    `route:${q.canUseRoute ? "ON" : "off"}`,
  ];
  if (q.horizontalAccuracyM !== null && q.horizontalAccuracyM !== undefined) {
    bits.push(`acc${Number(q.horizontalAccuracyM).toFixed(1)}m`);
  }
  if (q.separationM !== null && q.separationM !== undefined) {
    bits.push(`sep${Number(q.separationM).toFixed(1)}m`);
  }
  if (q.positionSigmaM !== null && q.positionSigmaM !== undefined) {
    bits.push(`σ${Number(q.positionSigmaM).toFixed(1)}m`);
  }
  return bits.join(" ");
}

export function positionQualityReason(status) {
  const q = status?.positionQuality;
  const geo = q?.reasons?.[0];
  const route = q?.routeReasons?.[0];
  return [route && `route:${route}`, geo && `geo:${geo}`].filter(Boolean).join(" | ") || "—";
}

/* ---------- 시계/타임라인 ---------- */

export function timelineLine(status) {
  const t = status?.timeline;
  if (!t) return "—";
  return `${t.domain || "—"} ${t.discontinuity ? `끊김(${t.discontinuityReason || "?"})` : "연속"}`;
}

/* target이 최신 관측보다 maxExtrapolationMs 이상 앞서면 sampleAt이 null을 주고
 * odometry 전파가 끊긴다. 그 간격을 직접 보여 준다. */
export function odometryTimelineLine(status) {
  const o = status?.odometryTimeline;
  if (!o) return "—";
  const target = num(status?.presentedClock?.targetTimestampNs);
  const newest = num(o.lastObservationTimestampNs);
  const lagMs = target === null || newest === null ? null : Math.round((target - newest) / 1e6);
  const limit = num(o.maxExtrapolationMs, 150);
  return `samples ${num(o.samples, 0)} · lag ${lagMs === null ? "—" : `${lagMs}ms`}/${limit}ms`;
}

export function worldPoseLine(status) {
  const w = status?.worldPose;
  if (!w) return "—";
  return w.initialized ? `init · ${num(w.integrations, 0)}회` : "미초기화";
}

/* ---------- 리플레이/성능 ---------- */

export function replayLine(diagnose) {
  const r = diagnose?.replay;
  if (!r) return "라이브";
  const time = r.currentTime === undefined ? "" : ` ${Number(r.currentTime).toFixed(1)}s`;
  return `${r.active ? "재생" : "정지"}${r.loading ? "·로딩" : ""}${time} ${r.segment || ""}`.trim();
}

export function frameSignalLine(diagnose) {
  const f = diagnose?.frameSignal;
  if (!f) return "—";
  return `${f.mode || "—"} rx${num(f.received, 0)} ${f.lastSource || ""}`.trim();
}

export function performanceLine(status) {
  const bits = [status?.mode || "—", `fps ${num(status?.currentFps, "—")}`, `frames ${num(status?.frames, 0)}`];
  const w = status?.worker;
  if (w) bits.push(`worker ${w.ready ? "ok" : "대기"}${w.broken ? `·${w.broken}` : ""}`);
  return bits.join(" · ");
}

export function timestamp(elapsedMs) {
  const total = Math.max(0, Math.round(elapsedMs / 100) / 10);
  return `${String(Math.floor(total / 60)).padStart(2, "0")}:${(total % 60).toFixed(1).padStart(4, "0")}`;
}
