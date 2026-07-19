export const DRIVE_CONTENT_STATE = Object.freeze({
  UNMOUNTED: "unmounted",
  INACTIVE: "inactive",
  ACTIVE: "active",
  DESTROYED: "destroyed",
});

export const DRIVE_CONTENT_HOOK_NAMES = Object.freeze([
  "mount",
  "activate",
  "deactivate",
  "resize",
  "status",
  "destroy",
]);

const instances = new WeakSet();

function runtimeTarget(environment) {
  return environment?.target || globalThis;
}

export function createDriveContent(name, hooks = {}, environment = {}) {
  const contentName = String(name || "").trim();
  if (!contentName) throw new TypeError("DriveContent requires a name");
  for (const hookName of DRIVE_CONTENT_HOOK_NAMES) {
    if (hooks[hookName] != null && typeof hooks[hookName] !== "function") {
      throw new TypeError(`DriveContent ${contentName}.${hookName} must be a function`);
    }
  }

  const target = runtimeTarget(environment);
  let root = null;
  let state = DRIVE_CONTENT_STATE.UNMOUNTED;
  let keepWarm = false;
  let generation = 0;

  function assertNotDestroyed(operation) {
    if (state === DRIVE_CONTENT_STATE.DESTROYED) {
      throw new Error(`DriveContent ${contentName} cannot ${operation} after destroy`);
    }
  }

  function coreStatus() {
    return {
      name: contentName,
      state,
      mounted: root !== null,
      active: state === DRIVE_CONTENT_STATE.ACTIVE,
      keepWarm,
      rootConnected: Boolean(root && (root.isConnected ?? true)),
      generation,
    };
  }

  function publish(reason) {
    if (typeof target.dispatchEvent !== "function" || typeof target.CustomEvent !== "function") return;
    target.dispatchEvent(new target.CustomEvent("drive:contentstatechange", {
      detail: { ...coreStatus(), reason: String(reason || "") },
    }));
  }

  function mount(nextRoot) {
    assertNotDestroyed("mount");
    if (!nextRoot || typeof nextRoot !== "object") {
      throw new TypeError(`DriveContent ${contentName}.mount requires a root element`);
    }
    if (root === nextRoot) return false;
    if (state === DRIVE_CONTENT_STATE.ACTIVE) {
      throw new Error(`DriveContent ${contentName} must deactivate before moving to another root`);
    }

    const previousRoot = root;
    hooks.mount?.(nextRoot, Object.freeze({
      previousRoot,
      remount: previousRoot !== null,
    }));
    root = nextRoot;
    state = DRIVE_CONTENT_STATE.INACTIVE;
    keepWarm = false;
    generation += 1;
    publish(previousRoot === null ? "mount" : "remount");
    return true;
  }

  function activate(context = {}) {
    assertNotDestroyed("activate");
    if (root === null) throw new Error(`DriveContent ${contentName} must mount before activate`);
    const result = hooks.activate?.(context);
    const changed = state !== DRIVE_CONTENT_STATE.ACTIVE || keepWarm;
    state = DRIVE_CONTENT_STATE.ACTIVE;
    keepWarm = false;
    if (changed) {
      generation += 1;
      publish("activate");
    }
    return result ?? changed;
  }

  function deactivate(options = {}) {
    assertNotDestroyed("deactivate");
    if (root === null || state !== DRIVE_CONTENT_STATE.ACTIVE) return false;
    const nextKeepWarm = Boolean(options.keepWarm);
    const result = hooks.deactivate?.({ ...options, keepWarm: nextKeepWarm });
    state = DRIVE_CONTENT_STATE.INACTIVE;
    keepWarm = nextKeepWarm;
    generation += 1;
    publish("deactivate");
    return result ?? true;
  }

  function resize(rect) {
    assertNotDestroyed("resize");
    if (root === null) return false;
    return hooks.resize?.(rect, { active: state === DRIVE_CONTENT_STATE.ACTIVE }) ?? false;
  }

  function status() {
    let detail = {};
    try {
      const value = hooks.status?.();
      if (value && typeof value === "object") detail = value;
    } catch (error) {
      detail = { error: String(error?.message || error || "status failed") };
    }
    return { ...detail, ...coreStatus() };
  }

  function destroy() {
    if (state === DRIVE_CONTENT_STATE.DESTROYED) return false;
    let failure = null;
    try {
      if (state === DRIVE_CONTENT_STATE.ACTIVE) {
        hooks.deactivate?.({ keepWarm: false, reason: "destroy" });
      }
    } catch (error) {
      failure = error;
    }
    try {
      hooks.destroy?.(root);
    } catch (error) {
      failure ??= error;
    } finally {
      root = null;
      state = DRIVE_CONTENT_STATE.DESTROYED;
      keepWarm = false;
      generation += 1;
      publish("destroy");
    }
    if (failure) throw failure;
    return true;
  }

  const api = Object.freeze({
    name: contentName,
    mount,
    activate,
    deactivate,
    resize,
    status,
    destroy,
  });
  instances.add(api);
  return api;
}

export function isDriveContent(value) {
  return Boolean(value && typeof value === "object" && instances.has(value));
}

export const DriveContent = Object.freeze({
  STATE: DRIVE_CONTENT_STATE,
  HOOK_NAMES: DRIVE_CONTENT_HOOK_NAMES,
  create: createDriveContent,
  isContent: isDriveContent,
});

export function installDriveContentFacade(target = globalThis) {
  target.DriveContent = DriveContent;
  return DriveContent;
}
