"use strict";

// Shared presentation for an exclusive stream that is currently owned by a
// different browser. Stream transports keep ownership policy; this component
// owns only feature identity, status copy, actions and accessible DOM.
globalThis.StreamOwnershipNotice = (() => {
  const ACTION_KIND = Object.freeze({
    PRIMARY: "primary",
    SECONDARY: "secondary",
  });
  const actionKinds = new Set(Object.values(ACTION_KIND));
  const instances = new WeakMap();
  const apis = new WeakSet();
  let nextInstanceId = 0;

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

  function normalizeFeature(value) {
    if (typeof value === "string" || typeof value === "function") {
      return Object.freeze({ id: "", label: value });
    }
    const feature = value && typeof value === "object" ? value : {};
    return Object.freeze({
      id: feature.id,
      label: feature.label,
    });
  }

  function normalizeActions(values) {
    if (!Array.isArray(values)) return Object.freeze([]);
    const ids = new Set();
    const actions = [];
    for (const value of values) {
      const action = value && typeof value === "object" ? value : {};
      const id = text(action.id);
      const label = resolvedText(action.label);
      if (!id || ids.has(id) || !label || typeof action.invoke !== "function") continue;
      const kind = actionKinds.has(action.kind) ? action.kind : ACTION_KIND.SECONDARY;
      ids.add(id);
      actions.push(Object.freeze({
        id,
        label,
        kind,
        invoke: action.invoke,
        disabled: Boolean(action.disabled),
      }));
    }
    return Object.freeze(actions);
  }

  function create(options = {}) {
    const host = options.host;
    if (!host || typeof host.append !== "function") return null;
    const existing = instances.get(host);
    if (existing) {
      if (options.feature != null) existing.setFeature(options.feature);
      return existing;
    }

    let featureDefinition = normalizeFeature(options.feature);
    const instanceId = `stream-ownership-notice-${++nextInstanceId}`;
    const root = document.createElement("section");
    root.className = ["stream-ownership-notice", text(options.className)].filter(Boolean).join(" ");
    root.hidden = true;
    root.setAttribute("aria-hidden", "true");
    root.setAttribute("aria-busy", "false");
    root.dataset.state = "blocked";
    root.dataset.pending = "false";

    const body = document.createElement("div");
    body.className = "stream-ownership-notice__body";

    const copy = document.createElement("div");
    copy.className = "stream-ownership-notice__copy";
    copy.setAttribute("role", "status");
    copy.setAttribute("aria-live", "polite");
    copy.setAttribute("aria-atomic", "true");

    const featureElement = document.createElement("h2");
    featureElement.id = `${instanceId}-feature`;
    featureElement.className = "stream-ownership-notice__feature";
    const titleElement = document.createElement("p");
    titleElement.id = `${instanceId}-title`;
    titleElement.className = "stream-ownership-notice__title";
    const descriptionElement = document.createElement("p");
    descriptionElement.id = `${instanceId}-description`;
    descriptionElement.className = "stream-ownership-notice__description";
    copy.append(featureElement, titleElement, descriptionElement);

    const actionsElement = document.createElement("div");
    actionsElement.className = "stream-ownership-notice__actions";
    actionsElement.setAttribute("role", "group");
    body.append(copy, actionsElement);
    root.append(body);
    host.append(root);

    const actionElements = new Map();
    let currentView = null;
    let presentation = null;
    let currentActions = Object.freeze([]);
    let pendingActionId = "";

    function actionById(id) {
      return currentActions.find((action) => action.id === id) || null;
    }

    function syncActions() {
      const retainedIds = new Set(currentActions.map((action) => action.id));
      for (const [id, record] of actionElements) {
        if (retainedIds.has(id)) continue;
        record.button.removeEventListener("click", record.listener);
        record.button.remove();
        actionElements.delete(id);
      }

      for (const action of currentActions) {
        let record = actionElements.get(action.id);
        if (!record) {
          const button = document.createElement("button");
          const listener = () => void invokeAction(action.id);
          button.type = "button";
          button.dataset.actionId = action.id;
          button.addEventListener("click", listener);
          record = { button, listener };
          actionElements.set(action.id, record);
        }
        record.button.className = [
          "btn",
          action.kind === ACTION_KIND.PRIMARY ? "btn--filled" : "",
          "stream-ownership-notice__action",
          `stream-ownership-notice__action--${action.kind}`,
        ].filter(Boolean).join(" ");
        record.button.dataset.kind = action.kind;
        record.button.textContent = action.label;
        record.button.disabled = Boolean(pendingActionId) || action.disabled;
      }

      const currentOrder = Array.from(actionsElement.children).map((button) => button.dataset.actionId);
      const nextOrder = currentActions.map((action) => action.id);
      if (currentOrder.join("\u0000") !== nextOrder.join("\u0000")) {
        for (const action of currentActions) actionsElement.append(actionElements.get(action.id).button);
      }

      actionsElement.hidden = currentActions.length === 0;
      actionsElement.dataset.layout = currentActions.length > 1 ? "split" : "single";
      root.dataset.pending = String(Boolean(pendingActionId));
      root.setAttribute("aria-busy", String(Boolean(pendingActionId)));
    }

    async function invokeAction(id) {
      const action = actionById(id);
      if (!action || action.disabled || pendingActionId) return;
      pendingActionId = action.id;
      syncActions();
      try {
        await action.invoke();
      } catch (_) {
      } finally {
        if (pendingActionId !== action.id) return;
        pendingActionId = "";
        syncActions();
      }
    }

    function resolvePresentation(view = {}) {
      const feature = normalizeFeature(featureDefinition);
      return Object.freeze({
        featureId: resolvedText(feature.id),
        featureLabel: resolvedText(feature.label),
        title: resolvedText(view.title),
        description: resolvedText(view.description),
        actions: normalizeActions(view.actions),
      });
    }

    function show(view = {}) {
      currentView = view && typeof view === "object" ? { ...view } : {};
      presentation = resolvePresentation(currentView);
      currentActions = presentation.actions;

      if (presentation.featureId) root.dataset.featureId = presentation.featureId;
      else delete root.dataset.featureId;
      featureElement.textContent = presentation.featureLabel;
      featureElement.hidden = !presentation.featureLabel;
      titleElement.textContent = presentation.title;
      titleElement.hidden = !presentation.title;
      descriptionElement.textContent = presentation.description;
      descriptionElement.hidden = !presentation.description;
      root.setAttribute(
        "aria-labelledby",
        presentation.title ? titleElement.id : featureElement.id,
      );
      if (presentation.description) root.setAttribute("aria-describedby", descriptionElement.id);
      else root.removeAttribute("aria-describedby");
      actionsElement.setAttribute(
        "aria-label",
        presentation.featureLabel || presentation.title || "Stream ownership actions",
      );
      syncActions();
      root.hidden = false;
      root.setAttribute("aria-hidden", "false");
      return true;
    }

    function setFeature(feature) {
      featureDefinition = normalizeFeature(feature);
      if (root.hidden || !currentView) return true;
      show(currentView);
      return true;
    }

    function hide() {
      if (root.hidden) return false;
      root.hidden = true;
      root.setAttribute("aria-hidden", "true");
      pendingActionId = "";
      currentActions = Object.freeze([]);
      currentView = null;
      presentation = null;
      syncActions();
      return true;
    }

    function snapshot() {
      const actions = currentActions.map((action) => Object.freeze({
        id: action.id,
        label: action.label,
        kind: action.kind,
        disabled: action.disabled,
      }));
      return Object.freeze({
        visible: !root.hidden,
        featureId: presentation?.featureId || "",
        featureLabel: presentation?.featureLabel || "",
        title: presentation?.title || "",
        description: presentation?.description || "",
        actions: Object.freeze(actions),
        pendingActionId,
      });
    }

    function destroy() {
      pendingActionId = "";
      currentActions = Object.freeze([]);
      currentView = null;
      presentation = null;
      for (const record of actionElements.values()) {
        record.button.removeEventListener("click", record.listener);
      }
      actionElements.clear();
      instances.delete(host);
      apis.delete(api);
      root.remove();
    }

    const api = Object.freeze({ show, hide, setFeature, snapshot, destroy });
    instances.set(host, api);
    apis.add(api);
    return api;
  }

  function isNotice(value) {
    return Boolean(value && typeof value === "object" && apis.has(value));
  }

  return Object.freeze({ ACTION_KIND, create, isNotice });
})();
