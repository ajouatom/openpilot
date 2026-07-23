"use strict";

/* AR 진단 패널의 DOM 컴포넌트.
 *
 * 표시할 내용은 fields.js의 선언에서만 온다. 이 파일은 그 선언을 DOM으로 만들고
 * 갱신하는 일만 한다(항목이 늘어도 여기는 그대로다). 스타일은 style.css의
 * 클래스와 앱 디자인 토큰을 쓴다 — 인라인 스타일 없음.
 *
 * 복사는 프레임 로그가 아니라 report.js의 집계 요약을 낸다. 같은 줄이 수백 번
 * 반복되는 원본은 붙여넣을 때 잘리기 때문이다.
 */

import { AR_DEBUG_SECTIONS } from "./fields.js";
import { copyToClipboard } from "./log.js";
import { createArDebugAggregate } from "./report.js";
import { createArReplayDiagnosticCapture } from "./capture.js";
import { formatCanvasProbe, probeArCanvas } from "./dom_probe.js";
import {
  calibrationLine, canvasLine, composeGate, drawnCount, num, primaryReason, summarize,
} from "./format.js";

const ACTIONS = Object.freeze([
  Object.freeze({ id: "copy", label: "복사", payload: (ctx) => ctx.report() }),
  Object.freeze({ id: "json", label: "JSON", payload: (ctx) => ctx.json() }),
]);

function el(documentRoot, tag, className, text) {
  const node = documentRoot.createElement(tag);
  if (className) node.className = className;
  if (text !== undefined) node.textContent = text;
  return node;
}

/** status/diagnose → 집계기가 세는 한 프레임의 관측값. */
function observationOf(status, diagnose, summary, reason) {
  const renderer = status?.worker?.renderer || status?.renderer || null;
  const probe = probeArCanvas();
  return {
    badge: summary.label,
    reason,
    gate: composeGate(status?.composition?.diag),
    hold: `hold:${status?.hold?.state || "—"}`,
    track: `trk:${status?.tracking?.state || "—"}[${status?.tracking?.canCreateAnchor ? "C" : "-"}${status?.tracking?.canPropagateAnchor ? "P" : "-"}]`,
    skips: Array.isArray(renderer?.skips) ? renderer.skips : [],
    anchor: status?.hold?.sample || null,
    sign: num(status?.composition?.signCount, 0),
    anchored: num(status?.composition?.anchoredCount, 0),
    drawn: drawnCount(status),
    fixed: `surface ${formatCanvasProbe(probe)} · canvas ${canvasLine(status)} · ${
      calibrationLine(status, diagnose)} · worker ${status?.worker?.ready ? "ok" : "대기"}`,
  };
}

export function createArDebugPanel(options = {}) {
  const target = options.target || globalThis;
  const documentRoot = options.document || target?.document;
  const mountRoot = options.mountRoot;
  if (!documentRoot?.createElement || !mountRoot?.appendChild) return null;

  const sections = options.sections || AR_DEBUG_SECTIONS;
  const aggregate = options.aggregate || createArDebugAggregate({
    now: () => target.performance?.now?.() ?? Date.now(),
  });
  const capture = options.capture || createArReplayDiagnosticCapture({ target });

  const host = el(documentRoot, "div", "ar-debug");
  host.id = "carrotArDebugPanel";

  const head = el(documentRoot, "div", "ar-debug__head");
  const badge = el(documentRoot, "span", "ar-debug__badge");
  const badgeDot = el(documentRoot, "span", "ar-debug__badgeDot");
  const badgeText = el(documentRoot, "span", null, "AR");
  badge.append(badgeDot, badgeText);
  head.append(badge);

  const captureButton = el(documentRoot, "button", "ar-debug__action ar-debug__action--capture", "60초 진단");
  captureButton.type = "button";
  captureButton.disabled = true;
  captureButton.title = "리플레이 0~60초 AR 상태를 자동 수집해 JSON 파일로 저장";
  captureButton.addEventListener("click", () => {
    const state = capture.snapshot();
    const request = state.running ? capture.stop() : capture.start();
    Promise.resolve(request).catch((error) => target.console?.error?.("[carrot AR debug] capture failed", error));
  });
  head.append(captureButton);

  const captureSummaryButton = el(documentRoot, "button", "ar-debug__action", "진단요약");
  captureSummaryButton.type = "button";
  captureSummaryButton.disabled = true;
  captureSummaryButton.title = "마지막 60초 진단의 핵심 원인과 대표 시각만 복사";
  captureSummaryButton.addEventListener("click", () => {
    const payload = capture.snapshot().lastResult?.report || "";
    if (!payload) return;
    copyToClipboard(target, payload).then((ok) => {
      if (!ok) target.console?.log?.(payload);
      captureSummaryButton.textContent = ok ? "복사됨" : "콘솔출력";
      target.setTimeout?.(() => { captureSummaryButton.textContent = "진단요약"; }, 1200);
    });
  });
  head.append(captureSummaryButton);

  const context = { report: () => aggregate.report(), json: () => json() };
  for (const action of ACTIONS) {
    const button = el(documentRoot, "button", "ar-debug__action", action.label);
    button.type = "button";
    button.addEventListener("click", () => {
      const payload = action.payload(context);
      copyToClipboard(target, payload).then((ok) => {
        if (!ok) target.console?.log?.(payload);
        button.textContent = ok ? "복사됨" : "콘솔출력";
        target.setTimeout?.(() => { button.textContent = action.label; }, 1200);
      });
    });
    head.append(button);
  }

  // 선언 → DOM. 각 field의 value 노드만 기억해 두고 갱신 때 텍스트만 바꾼다.
  const valueNodes = new Map();
  const body = el(documentRoot, "div", "ar-debug__body");
  for (const section of sections) {
    body.append(el(documentRoot, "div", "ar-debug__sectionLabel", section.label));
    for (const field of section.fields) {
      const row = el(documentRoot, "div", `ar-debug__row${field.wide ? " ar-debug__row--wide" : ""}`);
      row.append(el(documentRoot, "span", "ar-debug__key", field.label));
      const value = el(documentRoot, "span", "ar-debug__value");
      row.append(value);
      body.append(row);
      valueNodes.set(field.id, { node: value, field });
    }
  }

  const reason = el(documentRoot, "div", "ar-debug__reason");
  const foot = el(documentRoot, "div", "ar-debug__foot");
  foot.append(el(documentRoot, "span", null, "수집중"));
  const footCount = el(documentRoot, "span", "ar-debug__footCount", "0");
  foot.append(footCount);

  host.append(head, body, reason, foot);
  mountRoot.appendChild(host);

  let lastStatus = null;
  let lastDiagnose = null;
  let captureAvailable = false;
  const unsubscribeCapture = capture.subscribe((state) => {
    const elapsed = Math.round((state.progress || 0) * (state.durationSeconds || 60));
    captureButton.dataset.captureState = state.phase;
    captureButton.disabled = !state.running && !captureAvailable;
    captureSummaryButton.disabled = !state.lastResult?.report;
    if (state.running) captureButton.textContent = `중지 ${elapsed}/${Math.round(state.durationSeconds)}초`;
    else if (state.phase === "complete") captureButton.textContent = "다시 진단";
    else if (state.phase === "warning") captureButton.textContent = "불완전·재시도";
    else if (state.phase === "error") captureButton.textContent = "진단 재시도";
    else captureButton.textContent = "60초 진단";
    if (state.error) captureButton.title = state.error;
  });

  function json() {
    return JSON.stringify({
      report: aggregate.report(), status: lastStatus, diagnose: lastDiagnose,
    }, null, 2);
  }

  function update(status, diagnose) {
    lastStatus = status || null;
    lastDiagnose = diagnose || null;
    captureAvailable = diagnose?.replay?.active === true && diagnose?.replay?.ready === true;
    captureButton.disabled = !capture.snapshot().running && !captureAvailable;
    if (host.hidden) return;

    const summary = summarize(status);
    badge.className = `ar-debug__badge ar-debug__value--${summary.tone}`;
    badgeText.textContent = summary.label;

    for (const { node, field } of valueNodes.values()) {
      const text = field.value(status, diagnose);
      node.textContent = text;
      node.title = text;          // 잘린 값은 툴팁으로 전문을 본다
      const tone = field.tone?.(status, diagnose);
      node.className = `ar-debug__value${tone ? ` ar-debug__value--${tone}` : ""}`;
    }

    const why = primaryReason(status, diagnose);
    reason.textContent = why || "";
    reason.hidden = !why;

    aggregate.observe(observationOf(status, diagnose, summary, why));
    footCount.textContent = `${aggregate.frames}프레임`;
  }

  return Object.freeze({
    element: host,
    update,
    setVisible(next) { host.hidden = next === false; return !host.hidden; },
    report: () => aggregate.report(),
    logText: () => aggregate.report(),
    json,
    snapshot: () => ({ status: lastStatus, diagnose: lastDiagnose }),
    captureStart: (settings) => capture.start(settings),
    captureStop: () => capture.stop(),
    captureStatus: () => capture.snapshot(),
    destroy() {
      unsubscribeCapture?.();
      capture.destroy().catch?.(() => {});
      host.remove?.();
    },
  });
}
