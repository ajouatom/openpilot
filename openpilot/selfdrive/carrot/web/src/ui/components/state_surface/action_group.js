const MAX_ACTIONS = 2;
const ACTION_TONES = new Set(["primary", "secondary", "danger"]);

function text(value) {
  return String(value ?? "").trim();
}

function resolvedText(value) {
  if (typeof value !== "function") return text(value);
  try {
    return text(value());
  } catch (_) {
    return "";
  }
}

function resolveDocument(options, environment) {
  const documentRoot = environment?.document
    ?? options.environment?.document
    ?? options.document
    ?? globalThis.document;
  if (!documentRoot || typeof documentRoot.createElement !== "function") {
    throw new TypeError("createActionGroup requires a document environment");
  }
  return documentRoot;
}

function normalizedTone(value) {
  const tone = text(value);
  return ACTION_TONES.has(tone) ? tone : "secondary";
}

export function normalizeActionDescriptors(values) {
  if (!Array.isArray(values)) return Object.freeze([]);
  if (values.length > MAX_ACTIONS) {
    throw new RangeError(`State surface action groups support at most ${MAX_ACTIONS} actions`);
  }

  const ids = new Set();
  const actions = [];
  for (const value of values) {
    const source = value && typeof value === "object" ? value : {};
    const id = text(source.id);
    const label = resolvedText(source.label);
    const onActivate = source.onActivate ?? source.invoke;
    if (!id || ids.has(id) || !label || typeof onActivate !== "function") continue;
    ids.add(id);
    actions.push(Object.freeze({
      id,
      label,
      tone: normalizedTone(source.tone ?? source.kind),
      disabled: Boolean(source.disabled),
      onActivate,
    }));
  }
  return Object.freeze(actions);
}

function joinedClasses(values) {
  return Array.from(new Set(values.flatMap((value) => text(value).split(/\s+/)).filter(Boolean))).join(" ");
}

export function createActionGroup(options = {}, environment = {}) {
  const host = options.host;
  if (!host || typeof host.append !== "function") return null;

  const documentRoot = resolveDocument(options, environment);
  const root = documentRoot.createElement("div");
  root.setAttribute("role", "group");
  root.setAttribute("aria-busy", "false");
  root.dataset.layout = "single";
  root.dataset.sizing = "full";
  root.dataset.pending = "false";
  root.hidden = true;
  host.append(root);

  const records = new Map();
  let actions = Object.freeze([]);
  let actionById = new Map();
  let pending = null;
  let activationGeneration = 0;
  let destroyed = false;
  let ariaLabel = text(options.ariaLabel) || "Status actions";

  const groupClassName = text(options.className);
  const actionClassName = text(options.actionClassName);
  const actionToneClassPrefix = text(options.actionToneClassPrefix);
  const onPendingChange = typeof options.onPendingChange === "function"
    ? options.onPendingChange
    : null;

  function pendingActionId() {
    return pending?.id || "";
  }

  function actionClasses(action) {
    return joinedClasses([
      "btn",
      action.tone === "primary" ? "btn--filled" : "",
      action.tone === "danger" ? "btn--danger" : "",
      "c-action-group__action",
      `c-action-group__action--${action.tone}`,
      actionClassName,
      actionToneClassPrefix ? `${actionToneClassPrefix}${action.tone}` : "",
    ]);
  }

  function groupClasses() {
    const single = actions.length <= 1;
    return joinedClasses([
      "c-action-group",
      "c-action-group--nowrap",
      single ? "c-action-group--single" : "c-action-group--split",
      single ? "c-action-group--full" : "c-action-group--equal",
      groupClassName,
    ]);
  }

  function sync() {
    if (destroyed) return;
    const isPending = Boolean(pending);
    const single = actions.length <= 1;
    root.className = groupClasses();
    root.hidden = actions.length === 0;
    root.dataset.layout = single ? "single" : "split";
    root.dataset.sizing = single ? "full" : "equal";
    root.dataset.pending = String(isPending);
    root.setAttribute("aria-label", ariaLabel);
    root.setAttribute("aria-busy", String(isPending));

    for (const action of actions) {
      const button = records.get(action.id).button;
      const disabled = isPending || action.disabled;
      button.className = actionClasses(action);
      button.dataset.actionId = action.id;
      button.dataset.tone = action.tone;
      button.textContent = action.label;
      button.disabled = disabled;
      button.tabIndex = disabled ? -1 : 0;
      button.setAttribute("aria-disabled", String(disabled));
      button.setAttribute("tabindex", String(button.tabIndex));
      if (disabled) button.setAttribute("disabled", "");
      else button.removeAttribute("disabled");
    }

    onPendingChange?.(pendingActionId());
  }

  async function activate(id) {
    const action = actionById.get(text(id));
    if (destroyed || !action || action.disabled || pending) return false;

    const activation = Object.freeze({ id: action.id, generation: ++activationGeneration });
    pending = activation;
    sync();
    try {
      await action.onActivate();
    } catch (_) {
      // Presentation actions are best effort. Feature runtimes own error policy.
    } finally {
      if (destroyed || pending !== activation) return true;
      pending = null;
      sync();
    }
    return true;
  }

  function removeRecord(id) {
    const record = records.get(id);
    if (!record) return;
    record.button.removeEventListener("click", record.listener);
    record.button.remove();
    records.delete(id);
  }

  function setActions(values, update = {}) {
    if (destroyed) return false;
    const nextActions = normalizeActionDescriptors(values);
    const retainedIds = new Set(nextActions.map((action) => action.id));

    if (pending && !retainedIds.has(pending.id)) {
      pending = null;
      activationGeneration += 1;
    }
    for (const id of records.keys()) {
      if (!retainedIds.has(id)) removeRecord(id);
    }

    for (const action of nextActions) {
      if (records.has(action.id)) continue;
      const button = documentRoot.createElement("button");
      button.type = "button";
      const listener = () => void activate(action.id);
      button.addEventListener("click", listener);
      records.set(action.id, { button, listener });
    }

    actions = nextActions;
    actionById = new Map(actions.map((action) => [action.id, action]));
    if (update.ariaLabel != null) ariaLabel = text(update.ariaLabel) || "Status actions";

    const currentOrder = Array.from(root.children).map((button) => button.dataset.actionId);
    const nextOrder = actions.map((action) => action.id);
    if (currentOrder.join("\u0000") !== nextOrder.join("\u0000")) {
      for (const action of actions) root.append(records.get(action.id).button);
    }
    sync();
    return true;
  }

  function setAriaLabel(value) {
    if (destroyed) return false;
    ariaLabel = text(value) || "Status actions";
    sync();
    return true;
  }

  function snapshot() {
    const snapshotActions = actions.map((action) => Object.freeze({
      id: action.id,
      label: action.label,
      tone: action.tone,
      disabled: action.disabled,
    }));
    return Object.freeze({
      actions: Object.freeze(snapshotActions),
      pendingActionId: pendingActionId(),
      layout: actions.length <= 1 ? "single" : "split",
      sizing: actions.length <= 1 ? "full" : "equal",
    });
  }

  function destroy() {
    if (destroyed) return false;
    destroyed = true;
    pending = null;
    activationGeneration += 1;
    for (const id of Array.from(records.keys())) removeRecord(id);
    actions = Object.freeze([]);
    actionById = new Map();
    root.remove();
    return true;
  }

  return Object.freeze({
    element: root,
    setActions,
    setAriaLabel,
    activate,
    snapshot,
    destroy,
  });
}

export const ACTION_GROUP_MAX_ACTIONS = MAX_ACTIONS;
