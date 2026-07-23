"use strict";

/* AR 진단 오버레이의 수명주기.
 *
 * 웹 설정(vision_ar_debug)이 켜져 있고 주행/리플레이 화면이 활성일 때만 패널을
 * 만들고 폴링한다. 설정을 끄면 패널을 즉시 없앤다 — 꺼져 있으면 DOM도 타이머도
 * 남기지 않는다.
 */

import { readWebSettingUnlocked } from "../../../../../../shared/web/capabilities.js";
import { createArDebugPanel } from "./panel.js";

export { createArDebugPanel } from "./panel.js";
export { createArDebugLog, copyToClipboard, AR_DEBUG_LOG_LIMIT } from "./log.js";
export { AR_DEBUG_SECTIONS, logLine } from "./fields.js";
export { createArDebugAggregate } from "./report.js";
export {
  AR_REPLAY_CAPTURE_SCHEMA,
  AR_REPLAY_CAPTURE_SECONDS,
  AR_REPLAY_CAPTURE_MAX_FRAMES,
  compactArDiagnosticFrame,
  createArReplayDiagnosticCapture,
} from "./capture.js";
export { probeArCanvas, formatCanvasProbe, AR_CANVAS_SELECTOR } from "./dom_probe.js";
export * from "./format.js";

export const VISION_AR_DEBUG_SETTING_KEY = "vision_ar_debug";
export const AR_DEBUG_POLL_MS = 250;

/** 설정 저장소 어디에 있든 같은 방식으로 읽는다(활성화 gate와 동일한 순서). */
export function readVisionArDebugEnabled(target = globalThis) {
  const unlocked = typeof target?.isWebSettingUnlocked === "function"
    ? target.isWebSettingUnlocked(VISION_AR_DEBUG_SETTING_KEY)
    : readWebSettingUnlocked(VISION_AR_DEBUG_SETTING_KEY, target);
  if (!unlocked) return false;
  if (typeof target?.getWebSettingByKey === "function") {
    return Boolean(target.getWebSettingByKey(VISION_AR_DEBUG_SETTING_KEY, false));
  }
  const state = target?.CarrotWebSettingsState || {};
  if (Object.prototype.hasOwnProperty.call(state, VISION_AR_DEBUG_SETTING_KEY)) {
    return Boolean(state[VISION_AR_DEBUG_SETTING_KEY]);
  }
  return Boolean(target?.__CARROT_BOOTSTRAP__?.webSettings?.[VISION_AR_DEBUG_SETTING_KEY]);
}

export function createArDebugOverlay(options = {}) {
  const target = options.target || globalThis;
  const documentRoot = options.document || target?.document || null;
  const mountRoot = options.mountRoot || null;
  const pollMs = Number(options.pollMs) > 0 ? Number(options.pollMs) : AR_DEBUG_POLL_MS;
  const isEnabled = typeof options.isEnabled === "function"
    ? options.isEnabled
    : () => readVisionArDebugEnabled(target);
  const readRuntime = typeof options.readRuntime === "function"
    ? options.readRuntime
    : () => target.CarrotVisionAr;

  let panel = null;
  let timer = null;
  let active = false;

  function pump() {
    if (!panel) return;
    const runtime = readRuntime();
    let status = null;
    let diagnose = null;
    try { status = runtime?.status?.() || null; } catch {}
    try { diagnose = runtime?.diagnose?.() || null; } catch {}
    panel.update(status, diagnose);
  }

  function teardown() {
    if (timer !== null) { target.clearInterval?.(timer); timer = null; }
    panel?.destroy();
    panel = null;
  }

  /** 설정과 활성 상태를 함께 반영한다. 매 폴링마다 불려도 싸다. */
  function sync() {
    const shouldShow = active && isEnabled();
    if (!shouldShow) {
      if (panel) teardown();
      return false;
    }
    if (!panel) {
      panel = createArDebugPanel({ target, document: documentRoot, mountRoot });
      if (!panel) return false;
      pump();
    }
    if (timer === null) timer = target.setInterval?.(() => { sync(); pump(); }, pollMs) ?? null;
    return true;
  }

  return Object.freeze({
    activate() { active = true; return sync(); },
    deactivate() { active = false; teardown(); return false; },
    refresh: sync,
    isVisible: () => Boolean(panel),
    logText: () => panel?.logText() || "",
    json: () => panel?.json() || "",
    snapshot: () => panel?.snapshot() || null,
    captureStart: (settings) => panel?.captureStart(settings) || Promise.resolve(false),
    captureStop: () => panel?.captureStop() || Promise.resolve(false),
    captureStatus: () => panel?.captureStatus() || null,
    destroy() { active = false; teardown(); },
  });
}
