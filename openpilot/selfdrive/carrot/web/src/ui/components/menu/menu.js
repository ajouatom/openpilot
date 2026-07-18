const DEFAULT_ITEM_SELECTOR = '[role^="menuitem"]';
const mountedMenus = new WeakMap();

function requireEventTarget(value, name) {
  if (!value || typeof value.addEventListener !== "function" || typeof value.removeEventListener !== "function") {
    throw new TypeError(`${name} must be an event target`);
  }
  return value;
}

function itemIsDisabled(item) {
  return Boolean(
    item?.disabled
    || item?.hidden
    || item?.getAttribute?.("disabled") != null
    || item?.getAttribute?.("aria-disabled") === "true"
    || item?.getAttribute?.("aria-hidden") === "true"
    || item?.classList?.contains?.("is-disabled"),
  );
}

function closestItem(target, selector, panel) {
  let item = typeof target?.closest === "function" ? target.closest(selector) : null;
  if (!item) {
    for (let current = target; current && current !== panel; current = current.parentElement || current.parentNode) {
      if (typeof current.matches === "function" && current.matches(selector)) {
        item = current;
        break;
      }
    }
  }
  return item && panel.contains(item) ? item : null;
}

function focusElement(element) {
  if (typeof element?.focus === "function") element.focus();
}

/**
 * Mount an accessible menu-button controller.
 *
 * The optional environment keeps browser globals out of the behavior module and
 * makes the controller usable in focused DOM tests.
 */
export function createMenuController(options = {}, injectedEnvironment = {}) {
  const {
    root,
    trigger,
    panel,
    itemSelector = DEFAULT_ITEM_SELECTOR,
    beforeOpen,
    onSelect,
  } = options;
  const environment = options.environment || injectedEnvironment;
  const documentRoot = options.documentRoot || environment.document || root?.ownerDocument || globalThis.document;

  requireEventTarget(root, "root");
  requireEventTarget(trigger, "trigger");
  requireEventTarget(panel, "panel");
  requireEventTarget(documentRoot, "documentRoot");
  if (typeof root.contains !== "function" || typeof panel.contains !== "function") {
    throw new TypeError("root and panel must support contains()");
  }
  if (typeof panel.querySelectorAll !== "function") {
    throw new TypeError("panel must support querySelectorAll()");
  }
  if (typeof itemSelector !== "string" || !itemSelector.trim()) {
    throw new TypeError("itemSelector must be a non-empty string");
  }

  const previous = mountedMenus.get(root);
  if (previous) previous.destroy();

  let openState = trigger.getAttribute?.("aria-expanded") === "true"
    || (!panel.hidden && root.classList?.contains?.("is-open"));
  let destroyed = false;
  let controller;

  function enabledItems() {
    return Array.from(panel.querySelectorAll(itemSelector)).filter((item) => !itemIsDisabled(item));
  }

  function sync() {
    if (destroyed) return false;
    root.classList?.toggle?.("is-open", openState);
    panel.hidden = !openState;
    trigger.setAttribute?.("aria-expanded", openState ? "true" : "false");
    panel.setAttribute?.("aria-hidden", openState ? "false" : "true");
    return openState;
  }

  function focusBoundary(which) {
    const items = enabledItems();
    if (!items.length) return null;
    const item = which === "last" ? items[items.length - 1] : items[0];
    focusElement(item);
    return item;
  }

  function open(openOptions = {}) {
    if (destroyed) return false;
    if (!openState) {
      if (typeof beforeOpen === "function" && beforeOpen(controller) === false) return false;
      openState = true;
      sync();
    } else {
      sync();
    }

    const focus = openOptions === true || openOptions.focusFirst === true
      ? "first"
      : openOptions.focusLast === true
        ? "last"
        : openOptions.focus;
    if (focus === "first" || focus === "last") focusBoundary(focus);
    return true;
  }

  function close(closeOptions = {}) {
    if (destroyed) return false;
    const wasOpen = openState;
    openState = false;
    sync();
    if (closeOptions.restoreFocus === true) focusElement(trigger);
    return wasOpen;
  }

  function toggle(toggleOptions = {}) {
    if (destroyed) return false;
    if (openState) {
      close({ restoreFocus: toggleOptions.restoreFocus === true });
      return false;
    }
    open(toggleOptions);
    return openState;
  }

  function moveFocus(event) {
    const items = enabledItems();
    if (!items.length) return false;
    const activeElement = environment.getActiveElement?.() || documentRoot.activeElement;
    const currentIndex = items.indexOf(activeElement);
    let nextIndex;
    if (event.key === "ArrowDown") nextIndex = currentIndex < 0 ? 0 : (currentIndex + 1) % items.length;
    else if (event.key === "ArrowUp") nextIndex = currentIndex < 0 ? items.length - 1 : (currentIndex - 1 + items.length) % items.length;
    else if (event.key === "Home") nextIndex = 0;
    else if (event.key === "End") nextIndex = items.length - 1;
    else return false;
    event.preventDefault?.();
    focusElement(items[nextIndex]);
    return true;
  }

  function onTriggerClick() {
    toggle();
  }

  function onTriggerKeydown(event) {
    if (event.key !== "ArrowDown" && event.key !== "ArrowUp") return;
    event.preventDefault?.();
    open({ focus: event.key === "ArrowUp" ? "last" : "first" });
  }

  function onPanelClick(event) {
    if (!openState) return;
    const item = closestItem(event.target, itemSelector, panel);
    if (!item || itemIsDisabled(item)) return;
    close({ restoreFocus: true });
    if (typeof onSelect === "function") onSelect(item, event, controller);
  }

  function onPanelKeydown(event) {
    if (!openState) return;
    if (event.key === "Tab") {
      close();
      return;
    }
    if (event.key === "Escape") {
      event.preventDefault?.();
      event.stopPropagation?.();
      close({ restoreFocus: true });
      return;
    }
    moveFocus(event);
  }

  function onDocumentPointerdown(event) {
    if (!openState) return;
    const target = event.target;
    if (root.contains(target) || trigger.contains?.(target) || panel.contains(target)) return;
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
    panel.removeEventListener("click", onPanelClick);
    panel.removeEventListener("keydown", onPanelKeydown);
    documentRoot.removeEventListener("pointerdown", onDocumentPointerdown);
    documentRoot.removeEventListener("keydown", onDocumentKeydown);
    destroyed = true;
    if (mountedMenus.get(root) === controller) mountedMenus.delete(root);
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
    get destroyed() {
      return destroyed;
    },
  });

  trigger.setAttribute?.("aria-haspopup", "menu");
  if (panel.id) trigger.setAttribute?.("aria-controls", panel.id);
  if (!panel.getAttribute?.("role")) panel.setAttribute?.("role", "menu");
  sync();

  trigger.addEventListener("click", onTriggerClick);
  trigger.addEventListener("keydown", onTriggerKeydown);
  panel.addEventListener("click", onPanelClick);
  panel.addEventListener("keydown", onPanelKeydown);
  documentRoot.addEventListener("pointerdown", onDocumentPointerdown);
  documentRoot.addEventListener("keydown", onDocumentKeydown);
  mountedMenus.set(root, controller);

  return controller;
}
