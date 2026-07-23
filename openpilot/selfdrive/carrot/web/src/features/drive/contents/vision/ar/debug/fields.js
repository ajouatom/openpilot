"use strict";

/* 패널에 무엇을 어떤 순서로 보여줄지에 대한 유일한 선언.
 *
 * 패널(panel.js)은 이 배열만 보고 DOM을 만든다. 항목을 추가·삭제·재배치할 때
 * 여기만 고치면 되고, 화면·로그·JSON이 자동으로 같은 내용을 쓴다.
 *
 * value(status, diagnose) → 문자열, tone(status, diagnose) → TONE(선택).
 */

import {
  TONE, anchorSample, calibrationLine, canvasLine, composeGate,
  drawnCount, frameSignalLine, naviState, num, odometryTimelineLine, onOff,
  performanceLine, positionQualityLine, positionQualityReason, rendererLine,
  replayLine, sourceAges, sourceFrames, timelineLine, worldPoseLine,
} from "./format.js";
import { formatCanvasProbe, probeArCanvas } from "./dom_probe.js";

/** 파이프라인 단계 요약. 어디서 끊겼는지 위에서 아래로 읽힌다. */
export const AR_DEBUG_SECTIONS = Object.freeze([
  Object.freeze({
    id: "pipeline",
    label: "파이프라인",
    fields: Object.freeze([
      Object.freeze({
        id: "counts",
        label: "표지·앵커·그림",
        value: (s) => [
          num(s?.composition?.signCount, 0) ?? 0,
          num(s?.composition?.anchoredCount, 0) ?? 0,
          drawnCount(s),
        ].join(" · "),
        tone: (s) => (drawnCount(s) > 0 ? TONE.OK : TONE.WARN),
      }),
      Object.freeze({
        id: "tracking",
        label: "track·hold",
        value: (s) => {
          const t = s?.tracking;
          const gate = `${t?.canCreateAnchor ? "C" : "-"}${t?.canPropagateAnchor ? "P" : "-"}`;
          return `${t?.state || "—"}[${gate}] · ${s?.hold?.state || "—"}(${num(s?.hold?.anchorCount, 0) ?? 0})`;
        },
        tone: (s) => (s?.tracking?.canCreateAnchor ? TONE.OK : TONE.WARN),
      }),
      Object.freeze({
        id: "sources",
        label: "navi·route",
        value: (s) => `${onOff(s?.sync?.naviUsable)} · ${onOff(s?.positionQuality?.canUseRoute)}`,
        tone: (s) => (s?.sync?.naviUsable && s?.positionQuality?.canUseRoute ? TONE.OK : TONE.WARN),
      }),
    ]),
  }),
  Object.freeze({
    id: "compose",
    label: "앵커 입력",
    fields: Object.freeze([
      Object.freeze({
        id: "gate",
        label: "gate",
        value: (s) => composeGate(s?.composition?.diag),
        wide: true,
      }),
      Object.freeze({
        id: "anchor",
        label: "첫 앵커",
        value: (s) => anchorSample(s?.hold?.sample),
      }),
    ]),
  }),
  Object.freeze({
    id: "render",
    label: "렌더 경로",
    fields: Object.freeze([
      Object.freeze({ id: "renderer", label: "renderer", value: (s) => rendererLine(s), wide: true }),
      Object.freeze({ id: "canvas", label: "canvas", value: (s) => canvasLine(s) }),
      Object.freeze({
        // 투명도 0이면 "그렸다"고 보고돼도 화면에는 아무것도 없다.
        id: "alpha",
        label: "alpha",
        value: (s) => {
          const r = s?.worker?.renderer || s?.renderer || null;
          const track = num(s?.tracking?.alpha, null);
          const minVis = num(r?.minimumVisibilityAlpha, null);
          return `track ${track ?? "—"} · vis ${minVis ?? "—"}`;
        },
        tone: (s) => {
          const a = num(s?.tracking?.alpha, 1);
          return a !== null && a > 0.05 ? TONE.OK : TONE.WARN;
        },
      }),
      Object.freeze({
        // 워커가 "그렸다"고 해도 캔버스가 0 크기거나 덮여 있으면 화면엔 없다.
        id: "surface",
        label: "표면",
        value: () => formatCanvasProbe(probeArCanvas()),
        tone: () => {
          const p = probeArCanvas();
          if (!p?.rect) return TONE.WARN;
          const hidden = p.style.display === "none" || p.style.visibility === "hidden"
            || Number(p.style.opacity) === 0;
          const covered = p.coveredBy && p.coveredBy !== "없음";
          return (!hidden && p.rect.w > 0 && p.rect.h > 0 && !covered) ? TONE.OK : TONE.WARN;
        },
        wide: true,
      }),
      Object.freeze({
        id: "calibration",
        label: "보정",
        value: (s, d) => calibrationLine(s, d),
        tone: (s) => (s?.sync?.calibrationPlausible === true ? TONE.OK : TONE.WARN),
      }),
    ]),
  }),
  Object.freeze({
    id: "clock",
    label: "시계",
    fields: Object.freeze([
      Object.freeze({
        id: "presented",
        label: "presented·gap",
        value: (s) => `${s?.sync?.presentedClockConfidence || "—"} · ${s?.sync?.frameIdGap ?? "—"}`,
        tone: (s) => (s?.sync?.presentedClockConfidence === "unmapped" ? TONE.WARN : TONE.OK),
      }),
      Object.freeze({ id: "timeline", label: "timeline", value: (s) => timelineLine(s) }),
      Object.freeze({ id: "world", label: "worldPose", value: (s) => worldPoseLine(s) }),
      Object.freeze({ id: "odoTimeline", label: "odometry", value: (s) => odometryTimelineLine(s) }),
      Object.freeze({ id: "frameSignal", label: "frame신호", value: (s, d) => frameSignalLine(d), wide: true }),
    ]),
  }),
  Object.freeze({
    id: "sources",
    label: "소스",
    fields: Object.freeze([
      Object.freeze({ id: "frames", label: "frameId", value: (s, d) => sourceFrames(s, d) }),
      Object.freeze({ id: "ages", label: "age(ms)", value: (s) => sourceAges(s) }),
      Object.freeze({ id: "navi", label: "navi", value: (s, d) => naviState(d), wide: true }),
      Object.freeze({
        id: "syncState",
        label: "sync",
        value: (s) => `${s?.sync?.state || "—"} ${s?.sync?.canDrawPrecise ? "precise" : "—"}/${s?.sync?.canHoldAnchor ? "hold" : "—"}`,
        tone: (s) => (s?.sync?.canDrawPrecise ? TONE.OK : TONE.WARN),
      }),
    ]),
  }),
  Object.freeze({
    id: "quality",
    label: "위치 품질",
    fields: Object.freeze([
      Object.freeze({ id: "pq", label: "판정", value: (s) => positionQualityLine(s), wide: true }),
      Object.freeze({ id: "pqReason", label: "사유", value: (s) => positionQualityReason(s), wide: true }),
    ]),
  }),
  Object.freeze({
    id: "runtime",
    label: "런타임",
    fields: Object.freeze([
      Object.freeze({ id: "perf", label: "성능", value: (s) => performanceLine(s), wide: true }),
      Object.freeze({ id: "replay", label: "리플레이", value: (s, d) => replayLine(d), wide: true }),
    ]),
  }),
]);

/**
 * 로그 한 줄. 붙여넣기만으로 원인 추적이 되도록 파이프라인 전 구간을 담는다.
 * 이 문자열이 바뀔 때만 이력에 쌓이므로, 값이 그대로면 줄이 늘지 않는다.
 */
export function logLine(summary, status, reason, diagnose) {
  return [
    summary.label,
    [
      num(status?.composition?.signCount, 0) ?? 0,
      num(status?.composition?.anchoredCount, 0) ?? 0,
      drawnCount(status),
    ].join("·"),
    `trk:${status?.tracking?.state || "—"}[${status?.tracking?.canCreateAnchor ? "C" : "-"}${status?.tracking?.canPropagateAnchor ? "P" : "-"}]`,
    `hold:${status?.hold?.state || "—"}`,
    composeGate(status?.composition?.diag),
    `anchor:${anchorSample(status?.hold?.sample)}`,
    `draw:${rendererLine(status)}`,
    `surface:${formatCanvasProbe(probeArCanvas())}`,
    `canvas:${canvasLine(status)}`,
    `sync:${status?.sync?.state || "—"}/${status?.sync?.presentedClockConfidence || "—"}`,
    `ages:${sourceAges(status)}`,
    `frames:${sourceFrames(status, diagnose)}`,
    `navi:${naviState(diagnose)}`,
    `pq:${positionQualityLine(status)}`,
    `world:${worldPoseLine(status)}`,
    `tl:${timelineLine(status)}`,
    `cal:${calibrationLine(status, diagnose)}`,
    `perf:${performanceLine(status)}`,
    reason ? `· ${reason}` : "",
  ].filter(Boolean).join(" ");
}
