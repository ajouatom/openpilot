export const FOCUSABLE_SELECTOR = [
  "a[href]",
  "area[href]",
  "button:not([disabled])",
  'input:not([disabled]):not([type="hidden"])',
  "select:not([disabled])",
  "textarea:not([disabled])",
  '[tabindex]:not([tabindex="-1"])',
  '[contenteditable]:not([contenteditable="false"])',
  "audio[controls]",
  "video[controls]",
  "iframe",
  "object",
  "embed",
  "summary",
].join(",");

function focusTrapEnvironment(container, environment) {
  const documentRoot = environment.document ?? container?.ownerDocument ?? globalThis.document;
  const view = environment.window ?? documentRoot?.defaultView ?? globalThis.window ?? globalThis;
  const requestFrame = environment.requestAnimationFrame
    ?? view?.requestAnimationFrame?.bind(view)
    ?? ((callback) => globalThis.setTimeout(callback, 0));
  const cancelFrame = environment.cancelAnimationFrame
    ?? view?.cancelAnimationFrame?.bind(view)
    ?? ((handle) => globalThis.clearTimeout(handle));
  const getComputedStyle = environment.getComputedStyle
    ?? view?.getComputedStyle?.bind(view)
    ?? (() => ({}));
  return { documentRoot, requestFrame, cancelFrame, getComputedStyle };
}

export function getFocusableElements(root, environment = {}) {
  if (!root?.querySelectorAll) return [];
  const { getComputedStyle } = focusTrapEnvironment(root, environment);
  return Array.from(root.querySelectorAll(FOCUSABLE_SELECTOR)).filter((element) => {
    if (element.disabled || element.hidden || element.hasAttribute?.("disabled")) return false;
    const tabIndex = element.getAttribute?.("tabindex");
    if (tabIndex != null && Number(tabIndex) < 0) return false;
    let current = element;
    while (current && current !== root) {
      if (current.hasAttribute?.("inert") || current.getAttribute?.("aria-hidden") === "true") return false;
      current = current.parentElement ?? current.parentNode;
    }

    const rect = element.getBoundingClientRect?.();
    if (rect && rect.width === 0 && rect.height === 0) return false;
    const style = getComputedStyle(element) || {};
    return style.visibility !== "hidden" && style.display !== "none";
  });
}

function resolveInitialFocus(container, value) {
  const resolved = typeof value === "function" ? value() : value;
  if (!resolved) return null;
  if (typeof resolved === "string") return container.querySelector?.(resolved) ?? null;
  return typeof resolved.focus === "function" ? resolved : null;
}

export function createFocusTrap(container, options = {}, environment = {}) {
  if (!container) {
    return Object.freeze({
      activate() {},
      deactivate() {},
      isActive: () => false,
    });
  }

  const { documentRoot, requestFrame, cancelFrame } = focusTrapEnvironment(container, environment);
  let active = false;
  let activationSerial = 0;
  let restoreTo = null;
  let frameHandle = null;

  function onKeydown(event) {
    if (!active) return;
    if (event.key === "Escape" && typeof options.escape === "function") {
      options.escape(event);
      return;
    }
    if (event.key !== "Tab") return;

    const focusable = getFocusableElements(container, environment);
    if (!focusable.length) {
      event.preventDefault();
      container.focus?.();
      return;
    }

    const first = focusable[0];
    const last = focusable[focusable.length - 1];
    const focused = documentRoot?.activeElement;
    if (event.shiftKey) {
      if (focused === first || !container.contains?.(focused)) {
        event.preventDefault();
        last.focus();
      }
    } else if (focused === last || !container.contains?.(focused)) {
      event.preventDefault();
      first.focus();
    }
  }

  function cancelPendingFrame() {
    if (frameHandle == null) return;
    cancelFrame(frameHandle);
    frameHandle = null;
  }

  return Object.freeze({
    activate() {
      if (active) return;
      active = true;
      const currentActivation = ++activationSerial;
      restoreTo = options.returnFocus || documentRoot?.activeElement || null;
      if (!container.hasAttribute?.("tabindex")) container.setAttribute?.("tabindex", "-1");
      documentRoot?.addEventListener?.("keydown", onKeydown, true);

      if (options.focusInitial === false) return;
      const requestedInitial = resolveInitialFocus(container, options.initialFocus);
      const initialIsContained = !requestedInitial
        || requestedInitial === container
        || typeof container.contains !== "function"
        || container.contains(requestedInitial);
      const initial = (initialIsContained ? requestedInitial : null)
        || getFocusableElements(container, environment)[0]
        || container;
      frameHandle = requestFrame(() => {
        frameHandle = null;
        if (!active || currentActivation !== activationSerial) return;
        initial?.focus?.({ preventScroll: options.initialFocusPreventScroll !== false });
      });
    },

    deactivate(deactivateOptions = {}) {
      if (!active) {
        activationSerial += 1;
        cancelPendingFrame();
        return;
      }
      active = false;
      activationSerial += 1;
      cancelPendingFrame();
      documentRoot?.removeEventListener?.("keydown", onKeydown, true);

      const target = restoreTo;
      restoreTo = null;
      if (deactivateOptions.restoreFocus === false || typeof target?.focus !== "function") return;
      const isConnected = typeof documentRoot?.contains === "function"
        ? documentRoot.contains(target)
        : target.isConnected !== false;
      if (isConnected) target.focus({ preventScroll: true });
    },

    isActive() {
      return active;
    },
  });
}
