import {
  ACTION_KIND,
  createStateSurface,
  isStateSurface,
  STATE,
  TONE,
  VARIANT,
} from "./state_surface.js";

const installedTargets = new WeakMap();
const ACTION_KINDS = new Set(Object.values(ACTION_KIND));

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

function normalizeLegacyFeature(value) {
  if (typeof value === "string" || typeof value === "function") {
    return Object.freeze({ id: "", label: value });
  }
  const feature = value && typeof value === "object" ? value : {};
  return Object.freeze({
    id: feature.id,
    label: feature.label ?? feature.name,
  });
}

function normalizedActionKind(value) {
  const kind = text(value);
  return ACTION_KINDS.has(kind) ? kind : ACTION_KIND.SECONDARY;
}

function translateLegacyActions(values) {
  if (!Array.isArray(values)) return [];
  return values.map((value) => {
    const action = value && typeof value === "object" ? value : {};
    return {
      id: action.id,
      label: action.label,
      tone: normalizedActionKind(action.kind ?? action.tone),
      disabled: Boolean(action.disabled),
      onActivate: action.invoke ?? action.onActivate,
    };
  });
}

function createLegacyFacade(stateSurfaceFacade) {
  const instances = new WeakMap();
  const notices = new WeakSet();

  function create(options = {}) {
    const host = options.host;
    if (!host || typeof host.append !== "function") return null;
    const existing = instances.get(host);
    if (existing) {
      if (options.feature != null) existing.setFeature(options.feature);
      return existing;
    }

    let featureDefinition = normalizeLegacyFeature(options.feature);
    const surface = stateSurfaceFacade.create({
      host,
      className: options.className,
      featureLabel: featureDefinition.label,
    });
    if (!surface) return null;
    let destroyed = false;

    function show(view = {}) {
      if (destroyed) return false;
      const descriptor = view && typeof view === "object" ? view : {};
      return surface.show(STATE.OWNERSHIP, {
        title: descriptor.title,
        description: descriptor.description,
        tone: TONE.NEUTRAL,
        actions: translateLegacyActions(descriptor.actions),
      });
    }

    function hide() {
      return destroyed ? false : surface.hide();
    }

    function setFeature(feature) {
      if (destroyed) return false;
      featureDefinition = normalizeLegacyFeature(feature);
      return surface.setFeature(featureDefinition.label);
    }

    function snapshot() {
      const current = surface.snapshot();
      const actions = current.actions.map((action) => Object.freeze({
        id: action.id,
        label: action.label,
        kind: action.tone,
        disabled: action.disabled,
      }));
      return Object.freeze({
        visible: current.visible,
        featureId: current.visible ? resolvedText(featureDefinition.id) : "",
        featureLabel: current.featureLabel,
        title: current.title,
        description: current.description,
        actions: Object.freeze(actions),
        pendingActionId: current.pendingActionId,
      });
    }

    function destroy() {
      if (destroyed) return false;
      destroyed = true;
      instances.delete(host);
      notices.delete(api);
      surface.destroy();
      return true;
    }

    const api = Object.freeze({ show, hide, setFeature, snapshot, destroy });
    instances.set(host, api);
    notices.add(api);
    return api;
  }

  function isNotice(value) {
    return Boolean(value && typeof value === "object" && notices.has(value));
  }

  return Object.freeze({ ACTION_KIND, create, isNotice });
}

export function createStateSurfaceFacade(environment = {}) {
  return Object.freeze({
    STATE,
    TONE,
    VARIANT,
    ACTION_KIND,
    create(options = {}) {
      return createStateSurface(options, environment);
    },
    isSurface: isStateSurface,
  });
}

function installCarrotUINamespace(target, stateSurfaceFacade) {
  const current = target.CarrotUI;
  if (current && typeof current === "object" && Object.isExtensible(current)) {
    current.stateSurface = stateSurfaceFacade;
    return current;
  }
  const namespace = current && typeof current === "object" ? { ...current } : {};
  namespace.stateSurface = stateSurfaceFacade;
  target.CarrotUI = namespace;
  return namespace;
}

export function installStateSurfaceFacade(target = globalThis, environment = {}) {
  if (!target || (typeof target !== "object" && typeof target !== "function")) {
    throw new TypeError("installStateSurfaceFacade requires an object target");
  }

  let installation = installedTargets.get(target);
  if (!installation) {
    const resolvedEnvironment = {
      ...environment,
      document: environment.document ?? target.document ?? globalThis.document,
    };
    const stateSurface = createStateSurfaceFacade(resolvedEnvironment);
    installation = Object.freeze({
      stateSurface,
      streamOwnershipNotice: createLegacyFacade(stateSurface),
    });
    installedTargets.set(target, installation);
  }

  installCarrotUINamespace(target, installation.stateSurface);
  target.StreamOwnershipNotice = installation.streamOwnershipNotice;
  return installation.stateSurface;
}
