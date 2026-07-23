/* 당근비전 AR opt-in gate.
 *
 * 설정이 꺼져 있으면 runtime 자체를 만들지 않는다. 이미 켜진 상태에서 설정을
 * 끄면 destroy()까지 호출해 data lease, Worker, canvas를 한 번에 해제한다.
 * 서버 저장 실패로 web setting이 rollback되는 경우도 같은 change event를 통해
 * 원래 상태로 되돌아간다.
 */

import { readWebSettingUnlocked } from "../../../../../shared/web/capabilities.js";

export const VISION_AR_SETTING_KEY = "vision_ar_enabled";
export const VISION_AR_AVAILABLE = true;

export function readVisionArEnabled(target = globalThis) {
  if (!VISION_AR_AVAILABLE) return false;
  const unlocked = typeof target?.isWebSettingUnlocked === "function"
    ? target.isWebSettingUnlocked(VISION_AR_SETTING_KEY)
    : readWebSettingUnlocked(VISION_AR_SETTING_KEY, target);
  if (!unlocked) return false;
  if (typeof target?.getWebSettingByKey === "function") {
    return Boolean(target.getWebSettingByKey(VISION_AR_SETTING_KEY, false));
  }
  if (Object.prototype.hasOwnProperty.call(target?.CarrotWebSettingsState || {}, VISION_AR_SETTING_KEY)) {
    return Boolean(target.CarrotWebSettingsState[VISION_AR_SETTING_KEY]);
  }
  return Boolean(target?.__CARROT_BOOTSTRAP__?.webSettings?.[VISION_AR_SETTING_KEY]);
}

export function createVisionArActivationGate(options = {}) {
  const target = options.target || globalThis;
  const documentRoot = options.documentRoot || target?.document || null;
  const createRuntime = options.createRuntime;
  if (typeof createRuntime !== "function") {
    throw new TypeError("Vision AR activation gate requires createRuntime");
  }

  let visionActive = false;
  let destroyed = false;
  let runtime = null;
  let lastError = "";

  function enabled() {
    return readVisionArEnabled(target);
  }

  function disposeRuntime() {
    if (!runtime) return false;
    const previous = runtime;
    runtime = null;
    previous.destroy?.();
    return true;
  }

  function sync() {
    if (destroyed) return false;
    if (!enabled() || documentRoot?.hidden === true) {
      lastError = "";
      disposeRuntime();
      return false;
    }
    if (!visionActive) {
      disposeRuntime();
      return false;
    }
    try {
      runtime ||= createRuntime();
      if (!runtime) throw new Error("AR runtime을 만들 수 없습니다");
      lastError = "";
      runtime.activate?.();
      return true;
    } catch (error) {
      lastError = String(error?.message || error || "AR activation failed");
      target?.console?.warn?.("[vision ar] activate failed", error);
      disposeRuntime();
      return false;
    }
  }

  function onWebSettingsChange(event) {
    const keys = event?.detail?.keys || [event?.detail?.key];
    if (keys.includes(VISION_AR_SETTING_KEY)) sync();
  }

  function onVisibilityChange() {
    sync();
  }

  target?.addEventListener?.("carrot:websettingschange", onWebSettingsChange);
  target?.addEventListener?.("carrot:webcapabilitieschange", sync);
  documentRoot?.addEventListener?.("visibilitychange", onVisibilityChange);

  function activateVision() {
    if (destroyed) return false;
    visionActive = true;
    return sync();
  }

  function deactivateVision() {
    if (destroyed) return false;
    visionActive = false;
    return disposeRuntime();
  }

  function resize() {
    return visionActive && enabled() ? (runtime?.resize?.() ?? false) : false;
  }

  function status() {
    const runtimeStatus = runtime?.status?.() || {};
    return Object.freeze({
      ...runtimeStatus,
      enabled: enabled(),
      visionActive,
      documentHidden: documentRoot?.hidden === true,
      runtimeCreated: Boolean(runtime),
      lastError,
    });
  }

  function destroy() {
    if (destroyed) return false;
    destroyed = true;
    visionActive = false;
    target?.removeEventListener?.("carrot:websettingschange", onWebSettingsChange);
    target?.removeEventListener?.("carrot:webcapabilitieschange", sync);
    documentRoot?.removeEventListener?.("visibilitychange", onVisibilityChange);
    disposeRuntime();
    return true;
  }

  return Object.freeze({
    activateVision,
    deactivateVision,
    resize,
    sync,
    status,
    destroy,
  });
}
