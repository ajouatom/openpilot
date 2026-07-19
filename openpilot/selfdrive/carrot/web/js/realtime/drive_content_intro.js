"use strict";

// Shared status surface for Drive Workspace content. Feature runtimes provide
// identity and state; this module owns presentation, accessibility and an
// optional source adapter for future content modules.
globalThis.DriveContentIntro = (() => {
  const STATUS = Object.freeze({
    PREPARING: "preparing",
    WAITING: "waiting",
    EMPTY: "empty",
    UNAVAILABLE: "unavailable",
    UNSUPPORTED: "unsupported",
    PAUSED: "paused",
    RECOVERING: "recovering",
  });
  const BUSY_STATUS = Object.freeze([
    STATUS.PREPARING,
    STATUS.WAITING,
    STATUS.RECOVERING,
  ]);
  const EVENT = Object.freeze({
    CHANGE: "drive:contentintrochange",
  });
  const instances = new WeakMap();
  const apis = new WeakSet();

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
      label: feature.label ?? feature.name,
    });
  }

  function normalizeAction(value) {
    const action = value && typeof value === "object" ? value : {};
    const invoke = action.invoke ?? action.onInvoke ?? action.run;
    const label = resolvedText(action.label ?? action.text);
    if (!label || typeof invoke !== "function") return null;
    return Object.freeze({
      label,
      invoke,
      disabled: Boolean(action.disabled),
    });
  }

  function normalizeStatus(view = {}) {
    const source = view.status && typeof view.status === "object" ? view.status : {};
    const kind = text(
      source.kind
      ?? source.id
      ?? (typeof view.status === "string" ? view.status : view.state),
    ) || STATUS.WAITING;
    const explicitBusy = source.busy ?? view.busy;
    return Object.freeze({
      kind,
      message: resolvedText(source.message ?? source.label ?? view.message ?? view.title),
      description: resolvedText(source.description ?? view.description ?? view.detail),
      busy: typeof explicitBusy === "boolean" ? explicitBusy : BUSY_STATUS.includes(kind),
      action: normalizeAction(source.action ?? view.action),
    });
  }

  function create(options = {}) {
    const host = options.host;
    if (!host || typeof host.append !== "function") return null;
    const existing = instances.get(host);
    if (existing) {
      if (options.feature != null) existing.setFeature(options.feature);
      return existing;
    }

    let featureDefinition = normalizeFeature(options.feature ?? {
      id: options.featureId,
      label: options.featureLabel,
    });

    const root = document.createElement("section");
    root.className = ["drive-content-intro", text(options.className)].filter(Boolean).join(" ");
    root.hidden = true;
    root.setAttribute("aria-hidden", "true");
    root.setAttribute("aria-busy", "false");

    const body = document.createElement("div");
    body.className = "drive-content-intro__body";
    const copy = document.createElement("div");
    copy.className = "drive-content-intro__copy";
    copy.setAttribute("role", "status");
    copy.setAttribute("aria-live", "polite");
    copy.setAttribute("aria-atomic", "true");
    const featureElement = document.createElement("h2");
    featureElement.className = "drive-content-intro__feature";
    const statusElement = document.createElement("p");
    statusElement.className = "drive-content-intro__status";
    const descriptionElement = document.createElement("span");
    descriptionElement.className = "drive-content-intro__description visually-hidden";
    const actionElement = document.createElement("button");
    actionElement.className = "btn btn--filled drive-content-intro__action";
    actionElement.type = "button";
    actionElement.hidden = true;
    copy.append(featureElement, statusElement, descriptionElement);
    body.append(copy, actionElement);
    root.append(body);
    host.append(root);

    let state = "hidden";
    let signature = "";
    let presentation = null;
    let currentView = null;
    let enterFrame = 0;
    let hideTimer = 0;
    let sourceDisconnect = null;
    let currentAction = null;
    let actionPending = false;

    function syncAction() {
      const visible = Boolean(currentAction);
      actionElement.hidden = !visible;
      actionElement.textContent = currentAction?.label || "";
      actionElement.disabled = !visible || actionPending || Boolean(currentAction?.disabled);
    }

    function invokeAction() {
      if (!currentAction || currentAction.disabled || actionPending) return;
      const action = currentAction;
      actionPending = true;
      syncAction();
      Promise.resolve()
        .then(() => action.invoke())
        .catch(() => {})
        .finally(() => {
          actionPending = false;
          syncAction();
        });
    }

    actionElement.addEventListener("click", invokeAction);

    function clearMotionJobs() {
      if (enterFrame) cancelAnimationFrame(enterFrame);
      if (hideTimer) clearTimeout(hideTimer);
      enterFrame = 0;
      hideTimer = 0;
    }

    function timeMs(value) {
      const item = text(value);
      const number = Number.parseFloat(item);
      if (!Number.isFinite(number)) return 0;
      return item.endsWith("ms") ? number : number * 1000;
    }

    function transitionMs(element) {
      const style = getComputedStyle(element);
      const durations = style.transitionDuration.split(",").map(timeMs);
      const delays = style.transitionDelay.split(",").map(timeMs);
      return durations.reduce((longest, duration, index) => (
        Math.max(longest, duration + (delays[index % delays.length] || 0))
      ), 0);
    }

    function resolvedPresentation(view = {}) {
      const feature = normalizeFeature(view.feature ?? (view.label != null
        ? { id: featureDefinition.id, label: view.label }
        : featureDefinition));
      const status = normalizeStatus(view);
      return Object.freeze({
        featureId: resolvedText(feature.id),
        featureLabel: resolvedText(feature.label),
        status: status.kind,
        message: status.message,
        description: status.description,
        busy: status.busy,
        action: status.action,
      });
    }

    function snapshot() {
      return Object.freeze({
        state,
        visible: state !== "hidden",
        featureId: presentation?.featureId || "",
        featureLabel: presentation?.featureLabel || "",
        status: presentation?.status || "",
        message: presentation?.message || "",
        actionLabel: presentation?.action?.label || "",
        busy: state !== "hidden" && Boolean(presentation?.busy),
        connected: sourceDisconnect !== null,
      });
    }

    function publish(reason) {
      if (typeof globalThis.dispatchEvent !== "function" || typeof globalThis.CustomEvent !== "function") return;
      globalThis.dispatchEvent(new CustomEvent(EVENT.CHANGE, {
        detail: { ...snapshot(), reason: text(reason) },
      }));
    }

    function finishHide() {
      if (state !== "hidden") return;
      clearMotionJobs();
      root.hidden = true;
      delete root.dataset.motion;
      delete root.dataset.state;
      delete root.dataset.status;
      delete root.dataset.busy;
    }

    function finishHideOnTransition(event) {
      if (event.target === root && event.propertyName === "opacity") finishHide();
    }

    root.addEventListener("transitionend", finishHideOnTransition);

    function present(view = {}) {
      const next = resolvedPresentation(view);
      const nextSignature = [
        next.featureId,
        next.featureLabel,
        next.status,
        next.message,
        next.description,
        next.busy,
        next.action?.label || "",
        Boolean(next.action?.disabled),
      ].join("\u0000");
      currentAction = next.action;
      if (state !== "hidden" && signature === nextSignature) {
        syncAction();
        return false;
      }

      const shouldEnter = root.hidden || state === "hidden";
      clearMotionJobs();
      currentView = view && typeof view === "object" ? { ...view } : {};
      presentation = next;
      state = next.status;
      signature = nextSignature;
      root.dataset.state = next.status;
      root.dataset.status = next.status;
      root.dataset.busy = String(next.busy);
      if (next.featureId) root.dataset.featureId = next.featureId;
      else delete root.dataset.featureId;
      root.hidden = false;
      root.setAttribute("aria-hidden", "false");
      root.setAttribute("aria-busy", String(next.busy));
      featureElement.textContent = next.featureLabel;
      featureElement.hidden = !next.featureLabel;
      statusElement.textContent = next.message;
      statusElement.hidden = !next.message;
      descriptionElement.textContent = next.description;
      descriptionElement.hidden = !next.description;
      syncAction();

      if (shouldEnter) {
        root.dataset.motion = "entering";
        void root.offsetWidth;
        enterFrame = requestAnimationFrame(() => {
          enterFrame = 0;
          if (state !== "hidden") root.dataset.motion = "visible";
        });
      } else {
        root.dataset.motion = "visible";
      }
      publish("present");
      return true;
    }

    function setStatus(status, options = {}) {
      const statusView = status && typeof status === "object" ? status : { kind: status };
      return present({ ...options, status: statusView });
    }

    function setFeature(feature) {
      featureDefinition = normalizeFeature(feature);
      if (state === "hidden" || !currentView) return true;
      const nextView = { ...currentView };
      delete nextView.feature;
      return present(nextView);
    }

    function hide() {
      if (state === "hidden") return false;
      clearMotionJobs();
      currentAction = null;
      actionPending = false;
      syncAction();
      state = "hidden";
      root.setAttribute("aria-hidden", "true");
      root.setAttribute("aria-busy", "false");
      root.dataset.busy = "false";
      root.dataset.motion = "leaving";
      publish("hide");
      const duration = Math.max(transitionMs(root), transitionMs(body));
      if (duration <= 0) finishHide();
      else hideTimer = window.setTimeout(finishHide, duration + 50);
      return true;
    }

    function disconnect() {
      if (!sourceDisconnect) return false;
      const dispose = sourceDisconnect;
      sourceDisconnect = null;
      dispose();
      publish("disconnect");
      return true;
    }

    // A source implements subscribe(listener) and may expose snapshot(). This
    // keeps future content runtimes independent from this component's DOM.
    function connect(source, mapView = (value) => value) {
      if (!source || typeof source.subscribe !== "function") {
        throw new TypeError("DriveContentIntro.connect requires a subscribable source");
      }
      if (typeof mapView !== "function") {
        throw new TypeError("DriveContentIntro.connect mapView must be a function");
      }
      disconnect();
      const apply = (value) => {
        const view = mapView(value);
        if (view == null || view.hidden === true) hide();
        else present(view);
      };
      const subscription = source.subscribe(apply);
      sourceDisconnect = typeof subscription === "function"
        ? subscription
        : () => subscription?.unsubscribe?.();
      if (typeof source.snapshot === "function") apply(source.snapshot());
      publish("connect");
      return disconnect;
    }

    function destroy() {
      disconnect();
      clearMotionJobs();
      actionElement.removeEventListener("click", invokeAction);
      root.removeEventListener("transitionend", finishHideOnTransition);
      instances.delete(host);
      apis.delete(api);
      root.remove();
    }

    const api = Object.freeze({
      present,
      show: present,
      setStatus,
      setFeature,
      hide,
      connect,
      disconnect,
      snapshot,
      destroy,
    });
    instances.set(host, api);
    apis.add(api);
    return api;
  }

  function isIntro(value) {
    return Boolean(value && typeof value === "object" && apis.has(value));
  }

  return Object.freeze({ STATUS, BUSY_STATUS, EVENT, create, isIntro });
})();
