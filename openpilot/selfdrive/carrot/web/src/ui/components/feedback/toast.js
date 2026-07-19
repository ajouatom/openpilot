export const TOAST_TONE_GLYPH = Object.freeze({
  default: "i",
  info: "i",
  success: "✓",
  error: "✕",
  warn: "!",
  offline: "!",
  hint: "i",
});

export const TOAST_TONE_CLASS = Object.freeze({
  default: "",
  info: "is-info",
  success: "is-success",
  error: "is-error",
  warn: "is-warn",
  offline: "is-warn",
  hint: "is-hint",
});

export const DEFAULT_TOAST_DURATION = 2600;
export const DEFAULT_TOAST_REMOVE_DELAY = 220;

function normalizedTone(tone) {
  const value = typeof tone === "string" ? tone : "default";
  return Object.hasOwn(TOAST_TONE_GLYPH, value) ? value : "default";
}

function normalizedDelay(value, fallback) {
  if (value == null) return fallback;
  const number = Number(value);
  return Number.isFinite(number) && number >= 0 ? number : fallback;
}

function mountedInHost(element, host) {
  if (!element || !host) return false;
  if (typeof host.contains === "function") return host.contains(element);
  return element.parentNode === host;
}

function removeElement(element) {
  if (!element) return;
  if (typeof element.remove === "function") element.remove();
  else element.parentNode?.removeChild?.(element);
}

/** Create a single-surface toast controller with injectable scheduling. */
export function createToastController(options = {}, injectedEnvironment = {}) {
  const environment = options.environment || injectedEnvironment;
  const host = options.host || null;
  const documentRoot = options.documentRoot || environment.document || host?.ownerDocument || globalThis.document;
  const setTimer = environment.setTimeout || options.setTimeout || globalThis.setTimeout?.bind(globalThis);
  const clearTimer = environment.clearTimeout || options.clearTimeout || globalThis.clearTimeout?.bind(globalThis);
  const requestFrame = environment.requestAnimationFrame
    || options.requestAnimationFrame
    || ((callback) => setTimer(callback, 0));
  const cancelFrame = environment.cancelAnimationFrame || options.cancelAnimationFrame || clearTimer;
  const defaultDuration = normalizedDelay(options.defaultDuration, DEFAULT_TOAST_DURATION);
  const removeDelay = normalizedDelay(options.removeDelay, DEFAULT_TOAST_REMOVE_DELAY);

  let serial = 0;
  let activeToast = null;
  let hideTimer = null;
  let removeTimer = null;
  let frameHandle = null;
  let current = null;
  let destroyed = false;

  function clearSchedules() {
    if (hideTimer != null) clearTimer?.(hideTimer);
    if (removeTimer != null) clearTimer?.(removeTimer);
    if (frameHandle != null) cancelFrame?.(frameHandle);
    hideTimer = null;
    removeTimer = null;
    frameHandle = null;
  }

  function removeActive(token, toast) {
    if (destroyed || token !== serial || toast !== activeToast) return;
    removeElement(toast);
    activeToast = null;
    removeTimer = null;
    current = null;
  }

  function scheduleRemoval(token, toast) {
    if (removeDelay === 0) {
      removeActive(token, toast);
      return;
    }
    removeTimer = setTimer?.(() => removeActive(token, toast), removeDelay) ?? null;
  }

  function scheduleDismiss(token, toast) {
    if (destroyed || token !== serial || toast !== activeToast) return;
    toast.classList?.remove?.("is-visible");
    hideTimer = null;
    scheduleRemoval(token, toast);
  }

  function snapshot() {
    const active = Boolean(activeToast && mountedInHost(activeToast, host));
    return Object.freeze({
      active,
      visible: active && Boolean(activeToast.classList?.contains?.("is-visible")),
      message: active ? current?.message ?? "" : "",
      tone: active ? current?.tone ?? "default" : null,
      duration: active ? current?.duration ?? null : null,
      glyph: active ? current?.glyph ?? "" : "",
      className: active ? activeToast.className || "" : "",
      serial,
      destroyed,
    });
  }

  function ensureToast() {
    if (activeToast && mountedInHost(activeToast, host)) return activeToast;
    if (!host || typeof documentRoot?.createElement !== "function") return null;
    const toast = documentRoot.createElement("div");
    if (typeof host.replaceChildren === "function") host.replaceChildren(toast);
    else {
      while (host.firstChild) host.removeChild(host.firstChild);
      host.appendChild(toast);
    }
    activeToast = toast;
    return toast;
  }

  function show(message, showOptions = {}) {
    if (destroyed || message == null || String(message).length === 0) return snapshot();
    const toast = ensureToast();
    if (!toast) return snapshot();

    clearSchedules();
    const token = ++serial;
    const tone = normalizedTone(showOptions.tone);
    const duration = normalizedDelay(showOptions.duration, defaultDuration);
    const toneClass = TOAST_TONE_CLASS[tone];
    const glyph = TOAST_TONE_GLYPH[tone];

    toast.className = toneClass ? `app-toast ${toneClass}` : "app-toast";

    const icon = documentRoot.createElement("span");
    icon.className = "app-toast__icon";
    icon.setAttribute?.("aria-hidden", "true");
    icon.textContent = glyph;

    const text = documentRoot.createElement("div");
    text.className = "app-toast__msg";
    text.textContent = String(message);
    toast.replaceChildren(icon, text);

    current = { message: String(message), tone, duration, glyph };
    frameHandle = requestFrame?.(() => {
      if (destroyed || token !== serial || toast !== activeToast) return;
      frameHandle = null;
      toast.classList?.add?.("is-visible");
    }) ?? null;
    hideTimer = setTimer?.(() => scheduleDismiss(token, toast), duration) ?? null;
    return snapshot();
  }

  function dismiss(dismissOptions = {}) {
    if (destroyed || !activeToast) return snapshot();
    const toast = activeToast;
    clearSchedules();
    const token = ++serial;
    toast.classList?.remove?.("is-visible");
    if (dismissOptions.immediate === true) removeActive(token, toast);
    else scheduleRemoval(token, toast);
    return snapshot();
  }

  function destroy() {
    if (destroyed) return;
    clearSchedules();
    serial += 1;
    removeElement(activeToast);
    activeToast = null;
    current = null;
    destroyed = true;
  }

  return Object.freeze({ show, dismiss, destroy, snapshot });
}
