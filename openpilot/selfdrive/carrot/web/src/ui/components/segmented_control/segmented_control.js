const mountedControls = new WeakMap();

function readAttribute(element, name) {
  return typeof element?.getAttribute === "function" ? element.getAttribute(name) : null;
}

function isDisabled(item) {
  if (item?.disabled === true) return true;
  if (typeof item?.hasAttribute === "function" && item.hasAttribute("disabled")) return true;
  return String(readAttribute(item, "aria-disabled") || "").toLowerCase() === "true";
}

function isSelected(item, selectedAttribute) {
  const value = readAttribute(item, selectedAttribute);
  return value !== null && String(value).toLowerCase() !== "false";
}

function itemValue(item) {
  if (item?.dataset && Object.prototype.hasOwnProperty.call(item.dataset, "value")) {
    return String(item.dataset.value);
  }
  const dataValue = readAttribute(item, "data-value");
  if (dataValue !== null) return dataValue;
  const value = readAttribute(item, "value");
  if (value !== null) return value;
  if (item?.value !== undefined && item.value !== null) return String(item.value);
  return null;
}

function containsTarget(item, target) {
  if (item === target) return true;
  return typeof item?.contains === "function" && item.contains(target);
}

function normalizeOptions(options) {
  const itemSelector = String(options?.itemSelector || "").trim();
  if (!itemSelector) throw new TypeError("A segmented control itemSelector is required");

  return Object.freeze({
    itemSelector,
    selectedAttribute: String(options?.selectedAttribute || "aria-selected"),
    orientation: options?.orientation === "vertical" ? "vertical" : "horizontal",
    activation: options?.activation === "manual" ? "manual" : "automatic",
    onActivate: typeof options?.onActivate === "function" ? options.onActivate : null,
  });
}

function requireRoot(root) {
  if (!root
      || typeof root !== "object"
      || typeof root.querySelectorAll !== "function"
      || typeof root.addEventListener !== "function"
      || typeof root.removeEventListener !== "function") {
    throw new TypeError("A segmented control root element is required");
  }
}

export function createSegmentedControl(root, options = {}) {
  requireRoot(root);
  const mounted = mountedControls.get(root);
  if (mounted) {
    mounted.sync();
    return mounted;
  }

  const config = normalizeOptions(options);
  const originalTabindex = new Map();
  let items = [];
  let rovingItem = null;
  let destroyed = false;
  let controller = null;

  function queryItems() {
    return Array.from(root.querySelectorAll(config.itemSelector));
  }

  function rememberTabindex(item) {
    if (!originalTabindex.has(item)) originalTabindex.set(item, readAttribute(item, "tabindex"));
  }

  function writeTabindex(item, value) {
    rememberTabindex(item);
    item.setAttribute("tabindex", String(value));
  }

  function selectedItem(candidateItems = items) {
    return candidateItems.find((item) => isSelected(item, config.selectedAttribute)) || null;
  }

  function refresh({ preferSelected = true } = {}) {
    items = queryItems();
    const enabledItems = items.filter((item) => !isDisabled(item));
    const selected = selectedItem(items);
    const selectedEnabled = selected && enabledItems.includes(selected) ? selected : null;

    if (preferSelected && selectedEnabled) rovingItem = selectedEnabled;
    else if (!enabledItems.includes(rovingItem)) rovingItem = selectedEnabled || enabledItems[0] || null;

    for (const item of items) writeTabindex(item, item === rovingItem ? 0 : -1);
    return items;
  }

  function currentSnapshot() {
    const currentItems = queryItems();
    const selected = selectedItem(currentItems);
    const selectedIndex = selected ? currentItems.indexOf(selected) : -1;
    return Object.freeze({
      itemCount: currentItems.length,
      selectedIndex,
      selectedValue: selected ? itemValue(selected) : null,
      destroyed,
    });
  }

  function sync() {
    if (!destroyed) refresh({ preferSelected: true });
    return currentSnapshot();
  }

  function eventItem(event) {
    return items.find((item) => containsTarget(item, event?.target)) || null;
  }

  function updateRovingItem(item) {
    rovingItem = item;
    for (const candidate of items) writeTabindex(candidate, candidate === item ? 0 : -1);
  }

  function activate(item, event, source) {
    if (!config.onActivate) return;
    const index = items.indexOf(item);
    try {
      config.onActivate(item, index, Object.freeze({ source, event, controller }));
    } finally {
      // Activation may synchronously replace items or update ARIA selection.
      // Keep keyboard focus as the roving target while reconciling that DOM.
      if (!destroyed) refresh({ preferSelected: false });
    }
  }

  function focusItem(item, event) {
    if (!item) return;
    event?.preventDefault?.();
    updateRovingItem(item);
    if (typeof item.focus === "function") item.focus();
    if (config.activation === "automatic") activate(item, event, "keyboard");
  }

  function moveTarget(event, direction) {
    refresh({ preferSelected: false });
    const enabledItems = items.filter((item) => !isDisabled(item));
    if (!enabledItems.length) return null;

    const fromEvent = eventItem(event);
    const current = enabledItems.includes(fromEvent)
      ? fromEvent
      : (enabledItems.includes(rovingItem) ? rovingItem : enabledItems[0]);
    const currentIndex = enabledItems.indexOf(current);

    if (direction === "first") return enabledItems[0];
    if (direction === "last") return enabledItems[enabledItems.length - 1];
    const offset = direction === "previous" ? -1 : 1;
    return enabledItems[(currentIndex + offset + enabledItems.length) % enabledItems.length];
  }

  function onKeydown(event) {
    if (destroyed || event?.isComposing || event?.altKey || event?.ctrlKey || event?.metaKey) return;

    let direction = null;
    if (event.key === "Home") direction = "first";
    else if (event.key === "End") direction = "last";
    else if (config.orientation === "horizontal" && event.key === "ArrowLeft") direction = "previous";
    else if (config.orientation === "horizontal" && event.key === "ArrowRight") direction = "next";
    else if (config.orientation === "vertical" && event.key === "ArrowUp") direction = "previous";
    else if (config.orientation === "vertical" && event.key === "ArrowDown") direction = "next";

    if (direction) {
      focusItem(moveTarget(event, direction), event);
      return;
    }

    if (config.activation !== "manual"
        || (event.key !== "Enter" && event.key !== " " && event.key !== "Spacebar")) return;

    refresh({ preferSelected: false });
    const item = eventItem(event);
    if (!item || isDisabled(item)) return;
    event.preventDefault?.();
    updateRovingItem(item);
    activate(item, event, "keyboard");
  }

  function onFocusin(event) {
    if (destroyed) return;
    refresh({ preferSelected: false });
    const item = eventItem(event);
    if (item && !isDisabled(item)) updateRovingItem(item);
  }

  function destroy() {
    if (destroyed) return false;
    destroyed = true;
    root.removeEventListener("keydown", onKeydown);
    root.removeEventListener("focusin", onFocusin);
    for (const [item, tabindex] of originalTabindex) {
      if (tabindex === null) item.removeAttribute("tabindex");
      else item.setAttribute("tabindex", tabindex);
    }
    originalTabindex.clear();
    items = [];
    rovingItem = null;
    if (mountedControls.get(root) === controller) mountedControls.delete(root);
    return true;
  }

  controller = Object.freeze({ sync, destroy, snapshot: currentSnapshot });
  mountedControls.set(root, controller);
  root.addEventListener("keydown", onKeydown);
  root.addEventListener("focusin", onFocusin);
  sync();
  return controller;
}
