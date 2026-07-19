const mountedPopovers = new WeakMap();

function requireEventTarget(value, name) {
  if (!value || typeof value.addEventListener !== "function" || typeof value.removeEventListener !== "function") {
    throw new TypeError(`${name} must be an event target`);
  }
  return value;
}

export function createPopoverController(options = {}, injectedEnvironment = {}) {
  const { root, trigger, panel, beforeOpen, onOpenChange } = options;
  const environment = options.environment || injectedEnvironment;
  const documentRoot = options.documentRoot || environment.document || root?.ownerDocument || globalThis.document;

  requireEventTarget(root, "root");
  requireEventTarget(trigger, "trigger");
  requireEventTarget(panel, "panel");
  requireEventTarget(documentRoot, "documentRoot");
  if (typeof root.contains !== "function" || typeof panel.contains !== "function") {
    throw new TypeError("root and panel must support contains()");
  }

  const previous = mountedPopovers.get(root);
  if (previous) previous.destroy();

  let openState = trigger.getAttribute?.("aria-expanded") === "true" && panel.hidden !== true;
  let destroyed = false;
  let controller;

  function sync() {
    if (destroyed) return false;
    root.classList?.toggle?.("is-open", openState);
    panel.hidden = !openState;
    trigger.setAttribute?.("aria-expanded", openState ? "true" : "false");
    panel.setAttribute?.("aria-hidden", openState ? "false" : "true");
    if (typeof onOpenChange === "function") onOpenChange(openState, controller);
    return openState;
  }

  function open() {
    if (destroyed) return false;
    if (!openState && typeof beforeOpen === "function" && beforeOpen(controller) === false) return false;
    openState = true;
    return sync();
  }

  function close(options = {}) {
    if (destroyed) return false;
    const wasOpen = openState;
    openState = false;
    sync();
    if (options.restoreFocus === true) trigger.focus?.();
    return wasOpen;
  }

  function toggle() {
    return openState ? (close(), false) : open();
  }

  function onTriggerClick() {
    toggle();
  }

  function onTriggerKeydown(event) {
    if (event.key !== "Escape" || !openState) return;
    event.preventDefault?.();
    close({ restoreFocus: true });
  }

  function onDocumentPointerdown(event) {
    if (!openState || root.contains(event.target) || panel.contains(event.target)) return;
    close();
  }

  function onDocumentKeydown(event) {
    if (!openState || event.key !== "Escape") return;
    event.preventDefault?.();
    close({ restoreFocus: true });
  }

  function destroy() {
    if (destroyed) return;
    if (openState) {
      openState = false;
      sync();
    }
    trigger.removeEventListener("click", onTriggerClick);
    trigger.removeEventListener("keydown", onTriggerKeydown);
    documentRoot.removeEventListener("pointerdown", onDocumentPointerdown);
    documentRoot.removeEventListener("keydown", onDocumentKeydown);
    destroyed = true;
    if (mountedPopovers.get(root) === controller) mountedPopovers.delete(root);
  }

  controller = Object.freeze({
    open,
    close,
    toggle,
    sync,
    destroy,
    get isOpen() {
      return !destroyed && openState;
    },
  });

  if (panel.id) {
    trigger.setAttribute?.("aria-controls", panel.id);
    trigger.setAttribute?.("aria-describedby", panel.id);
  }
  sync();
  trigger.addEventListener("click", onTriggerClick);
  trigger.addEventListener("keydown", onTriggerKeydown);
  documentRoot.addEventListener("pointerdown", onDocumentPointerdown);
  documentRoot.addEventListener("keydown", onDocumentKeydown);
  mountedPopovers.set(root, controller);
  return controller;
}
