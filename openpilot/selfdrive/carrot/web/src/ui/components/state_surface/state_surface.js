import { createActionGroup, normalizeActionDescriptors } from "./action_group.js";

export const STATE = Object.freeze({
  EMPTY: "empty",
  LOADING: "loading",
  DEGRADED: "degraded",
  ERROR: "error",
  OWNERSHIP: "ownership",
});

export const TONE = Object.freeze({
  NEUTRAL: "neutral",
  INFO: "info",
  WARNING: "warning",
  ERROR: "error",
});

export const ACTION_KIND = Object.freeze({
  PRIMARY: "primary",
  SECONDARY: "secondary",
  DANGER: "danger",
});

export const VARIANT = Object.freeze({
  COVER: "cover",
  COMPACT: "compact",
});

const STATES = new Set(Object.values(STATE));
const TONES = new Set(Object.values(TONE));
const VARIANTS = new Set(Object.values(VARIANT));
const DEFAULT_TONE = Object.freeze({
  [STATE.EMPTY]: TONE.NEUTRAL,
  [STATE.LOADING]: TONE.INFO,
  [STATE.DEGRADED]: TONE.WARNING,
  [STATE.ERROR]: TONE.ERROR,
  [STATE.OWNERSHIP]: TONE.WARNING,
});
const instances = new WeakMap();
const surfaces = new WeakSet();
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

function joinedClasses(values) {
  return Array.from(new Set(values.flatMap((value) => text(value).split(/\s+/)).filter(Boolean))).join(" ");
}

function normalizedState(value) {
  const state = text(value);
  return STATES.has(state) ? state : STATE.EMPTY;
}

function normalizedTone(value, state) {
  const tone = text(value);
  return TONES.has(tone) ? tone : DEFAULT_TONE[state];
}

function normalizedVariant(value) {
  const variant = text(value);
  return VARIANTS.has(variant) ? variant : VARIANT.COVER;
}

function normalizeFeatureLabel(value) {
  if (value && typeof value === "object") {
    return value.featureLabel ?? value.label ?? value.name;
  }
  return value;
}

function resolveDocument(options, environment) {
  const documentRoot = environment?.document
    ?? options.environment?.document
    ?? options.document
    ?? globalThis.document;
  if (!documentRoot || typeof documentRoot.createElement !== "function") {
    throw new TypeError("createStateSurface requires a document environment");
  }
  return documentRoot;
}

function showArguments(stateOrDescriptor, maybeDescriptor) {
  if (typeof stateOrDescriptor === "string") {
    return {
      state: normalizedState(stateOrDescriptor),
      descriptor: maybeDescriptor && typeof maybeDescriptor === "object" ? maybeDescriptor : {},
    };
  }
  const descriptor = stateOrDescriptor && typeof stateOrDescriptor === "object"
    ? stateOrDescriptor
    : {};
  return {
    state: normalizedState(descriptor.state),
    descriptor,
  };
}

export function createStateSurface(options = {}, environment = {}) {
  const host = options.host;
  if (!host || typeof host.append !== "function") return null;
  const existing = instances.get(host);
  if (existing) {
    const nextFeature = options.featureLabel ?? options.feature;
    if (nextFeature != null) existing.setFeature(nextFeature);
    return existing;
  }

  const documentRoot = resolveDocument(options, environment);
  const instanceId = `c-state-surface-${++nextInstanceId}`;
  const customClassName = text(options.className);
  const defaultVariant = normalizedVariant(options.variant);
  let featureDefinition = normalizeFeatureLabel(options.featureLabel ?? options.feature);
  let currentView = null;
  let presentation = null;
  let currentState = STATE.EMPTY;
  let currentTone = TONE.NEUTRAL;
  let currentVariant = defaultVariant;
  let pendingActionId = "";
  let destroyed = false;

  const root = documentRoot.createElement("section");
  root.className = joinedClasses([
    "c-state-surface",
    `c-state-surface--${currentTone}`,
    `c-state-surface--state-${currentState}`,
    `c-state-surface--${currentVariant}`,
    customClassName,
  ]);
  root.hidden = true;
  root.setAttribute("aria-hidden", "true");
  root.setAttribute("aria-busy", "false");
  root.dataset.state = currentState;
  root.dataset.tone = currentTone;
  root.dataset.variant = currentVariant;
  root.dataset.pending = "false";

  const body = documentRoot.createElement("div");
  body.className = "c-state-surface__body";

  const copy = documentRoot.createElement("div");
  copy.className = "c-state-surface__copy";
  copy.setAttribute("role", "status");
  copy.setAttribute("aria-live", "polite");
  copy.setAttribute("aria-atomic", "true");

  const featureElement = documentRoot.createElement("h2");
  featureElement.id = `${instanceId}-feature`;
  featureElement.className = "c-state-surface__feature";
  const titleElement = documentRoot.createElement("p");
  titleElement.id = `${instanceId}-title`;
  titleElement.className = "c-state-surface__title";
  const descriptionElement = documentRoot.createElement("p");
  descriptionElement.id = `${instanceId}-description`;
  descriptionElement.className = "c-state-surface__description";
  copy.append(featureElement, titleElement, descriptionElement);
  body.append(copy);
  root.append(body);
  host.append(root);

  function syncBusy() {
    const busy = currentState === STATE.LOADING || Boolean(pendingActionId);
    root.dataset.pending = String(Boolean(pendingActionId));
    root.setAttribute("aria-busy", String(busy));
  }

  const actionGroup = createActionGroup({
    host: body,
    className: "c-state-surface__actions",
    actionClassName: "c-state-surface__action",
    actionToneClassPrefix: "c-state-surface__action--",
    onPendingChange(nextPendingActionId) {
      pendingActionId = nextPendingActionId;
      syncBusy();
    },
  }, { document: documentRoot });

  function resolvedPresentation(state, descriptor) {
    const featureLabel = descriptor.featureLabel != null
      ? descriptor.featureLabel
      : featureDefinition;
    return Object.freeze({
      state,
      featureLabel: resolvedText(featureLabel),
      title: resolvedText(descriptor.title),
      description: resolvedText(descriptor.description),
      tone: normalizedTone(descriptor.tone, state),
      variant: normalizedVariant(descriptor.variant ?? defaultVariant),
      actions: normalizeActionDescriptors(descriptor.actions),
    });
  }

  function syncCopy(next) {
    currentState = next.state;
    currentTone = next.tone;
    currentVariant = next.variant;
    root.className = joinedClasses([
      "c-state-surface",
      `c-state-surface--${currentTone}`,
      `c-state-surface--state-${currentState}`,
      `c-state-surface--${currentVariant}`,
      customClassName,
    ]);
    root.dataset.state = currentState;
    root.dataset.tone = currentTone;
    root.dataset.variant = currentVariant;

    featureElement.textContent = next.featureLabel;
    featureElement.hidden = !next.featureLabel;
    titleElement.textContent = next.title;
    titleElement.hidden = !next.title;
    descriptionElement.textContent = next.description;
    descriptionElement.hidden = !next.description;

    if (next.title || next.featureLabel) {
      root.setAttribute("aria-labelledby", next.title ? titleElement.id : featureElement.id);
      root.removeAttribute("aria-label");
    } else {
      root.removeAttribute("aria-labelledby");
      root.setAttribute("aria-label", "Status");
    }
    if (next.description) root.setAttribute("aria-describedby", descriptionElement.id);
    else root.removeAttribute("aria-describedby");
  }

  function show(stateOrDescriptor = {}, maybeDescriptor) {
    if (destroyed) return false;
    const { state, descriptor } = showArguments(stateOrDescriptor, maybeDescriptor);
    const next = resolvedPresentation(state, descriptor);

    currentView = { ...descriptor };
    presentation = next;
    syncCopy(next);
    actionGroup.setActions(next.actions, {
      ariaLabel: `${next.featureLabel || next.title || "Status"} actions`,
    });
    root.hidden = false;
    root.setAttribute("aria-hidden", "false");
    syncBusy();
    return true;
  }

  function hide() {
    if (destroyed || root.hidden) return false;
    currentView = null;
    presentation = null;
    currentState = STATE.EMPTY;
    currentTone = TONE.NEUTRAL;
    currentVariant = defaultVariant;
    actionGroup.setActions([]);
    pendingActionId = "";
    root.hidden = true;
    root.setAttribute("aria-hidden", "true");
    root.dataset.state = currentState;
    root.dataset.tone = currentTone;
    root.dataset.variant = currentVariant;
    root.className = joinedClasses([
      "c-state-surface",
      `c-state-surface--${currentTone}`,
      `c-state-surface--state-${currentState}`,
      `c-state-surface--${currentVariant}`,
      customClassName,
    ]);
    syncBusy();
    return true;
  }

  function setFeature(feature) {
    if (destroyed) return false;
    featureDefinition = normalizeFeatureLabel(feature);
    if (root.hidden || !currentView) return true;
    return show(currentState, currentView);
  }

  function snapshot() {
    const actionSnapshot = actionGroup.snapshot();
    return Object.freeze({
      visible: !destroyed && !root.hidden,
      state: presentation?.state || STATE.EMPTY,
      featureLabel: presentation?.featureLabel || "",
      title: presentation?.title || "",
      description: presentation?.description || "",
      tone: presentation?.tone || TONE.NEUTRAL,
      variant: presentation?.variant || defaultVariant,
      actions: actionSnapshot.actions,
      pendingActionId: actionSnapshot.pendingActionId,
    });
  }

  function destroy() {
    if (destroyed) return false;
    destroyed = true;
    currentView = null;
    presentation = null;
    pendingActionId = "";
    actionGroup.destroy();
    instances.delete(host);
    surfaces.delete(api);
    root.remove();
    return true;
  }

  const api = Object.freeze({ show, hide, setFeature, snapshot, destroy });
  instances.set(host, api);
  surfaces.add(api);
  return api;
}

export function isStateSurface(value) {
  return Boolean(value && typeof value === "object" && surfaces.has(value));
}
