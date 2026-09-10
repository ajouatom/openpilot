import { createFeaturePanelLifecycle } from "../../../shared/feature_panels/lifecycle.js";

function normalizeContext(context = {}) {
  return Object.freeze({
    group: String(context.group || ""),
    detailMode: context.detailMode === true,
    detailName: String(context.detailName || ""),
    root: context.root || null,
  });
}

function normalizeMounted(value, lifecycle) {
  if (!value || typeof value !== "object") {
    lifecycle.destroy();
    return null;
  }
  return Object.freeze({
    root: value.root || null,
    sync: typeof value.sync === "function" ? value.sync : null,
    destroy: typeof value.destroy === "function" ? value.destroy : null,
    lifecycle,
  });
}

/**
 * Mounts feature-specific panels only after the Settings renderer has created
 * their real Param section. It replaces per-feature MutationObservers and
 * keeps network work opt-in at the adapter level.
 */
export function createSettingsExtensionRegistry() {
  const extensions = new Map();
  const mounted = new Map();

  function destroyMounted(id) {
    const current = mounted.get(id);
    if (!current) return false;
    mounted.delete(id);
    try {
      current.destroy?.();
    } finally {
      current.lifecycle.destroy();
    }
    return true;
  }

  function register(extension) {
    const id = String(extension?.id || "").trim();
    if (!id || typeof extension?.matches !== "function" || typeof extension?.mount !== "function") {
      throw new TypeError("A settings extension needs id, matches, and mount");
    }
    if (extensions.has(id)) throw new Error(`Settings extension already registered: ${id}`);
    extensions.set(id, Object.freeze({ id, matches: extension.matches, mount: extension.mount }));
    return () => {
      destroyMounted(id);
      return extensions.delete(id);
    };
  }

  function sync(rawContext = {}) {
    const context = normalizeContext(rawContext);
    for (const extension of extensions.values()) {
      const current = mounted.get(extension.id);
      const detached = current?.root?.isConnected === false;
      if (detached) destroyMounted(extension.id);

      const shouldMount = Boolean(context.root) && extension.matches(context);
      const active = mounted.get(extension.id);
      if (!shouldMount) {
        destroyMounted(extension.id);
        continue;
      }
      if (active) {
        active.sync?.(context);
        continue;
      }

      const lifecycle = createFeaturePanelLifecycle();
      const next = normalizeMounted(extension.mount(Object.freeze({ ...context, lifecycle })), lifecycle);
      if (next) mounted.set(extension.id, next);
    }
  }

  function destroy() {
    for (const id of [...mounted.keys()]) destroyMounted(id);
  }

  return Object.freeze({ register, sync, destroy });
}

const installedTargets = new WeakMap();

export function installSettingsExtensionRegistry(target = globalThis) {
  if (!target || (typeof target !== "object" && typeof target !== "function")) {
    throw new TypeError("Settings extension target must be an object");
  }
  const existing = installedTargets.get(target);
  if (existing) return existing;

  const registry = createSettingsExtensionRegistry();
  target.CarrotSettingsExtensions = registry;
  installedTargets.set(target, registry);
  return registry;
}
