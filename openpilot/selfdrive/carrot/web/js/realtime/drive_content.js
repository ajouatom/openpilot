"use strict";

globalThis.DriveContent = (() => {
  const STATE = Object.freeze({
    UNMOUNTED: "unmounted",
    INACTIVE: "inactive",
    ACTIVE: "active",
    DESTROYED: "destroyed",
  });
  const HOOK_NAMES = Object.freeze([
    "mount",
    "activate",
    "deactivate",
    "resize",
    "status",
    "destroy",
  ]);
  const instances = new WeakSet();

  function create(name, hooks = {}) {
    const contentName = String(name || "").trim();
    if (!contentName) throw new TypeError("DriveContent requires a name");
    for (const hookName of HOOK_NAMES) {
      if (hooks[hookName] != null && typeof hooks[hookName] !== "function") {
        throw new TypeError(`DriveContent ${contentName}.${hookName} must be a function`);
      }
    }

    let root = null;
    let state = STATE.UNMOUNTED;
    let keepWarm = false;
    let generation = 0;

    function assertNotDestroyed(operation) {
      if (state === STATE.DESTROYED) {
        throw new Error(`DriveContent ${contentName} cannot ${operation} after destroy`);
      }
    }

    function coreStatus() {
      return {
        name: contentName,
        state,
        mounted: root !== null,
        active: state === STATE.ACTIVE,
        keepWarm,
        rootConnected: Boolean(root && (root.isConnected ?? true)),
        generation,
      };
    }

    function publish(reason) {
      if (typeof globalThis.dispatchEvent !== "function" || typeof globalThis.CustomEvent !== "function") return;
      globalThis.dispatchEvent(new CustomEvent("drive:contentstatechange", {
        detail: { ...coreStatus(), reason: String(reason || "") },
      }));
    }

    function mount(nextRoot) {
      assertNotDestroyed("mount");
      if (!nextRoot || typeof nextRoot !== "object") {
        throw new TypeError(`DriveContent ${contentName}.mount requires a root element`);
      }
      if (root !== null) {
        if (root !== nextRoot) throw new Error(`DriveContent ${contentName} root is fixed after mount`);
        return false;
      }
      hooks.mount?.(nextRoot);
      root = nextRoot;
      state = STATE.INACTIVE;
      generation += 1;
      publish("mount");
      return true;
    }

    function activate(context = {}) {
      assertNotDestroyed("activate");
      if (root === null) throw new Error(`DriveContent ${contentName} must mount before activate`);
      const result = hooks.activate?.(context);
      const changed = state !== STATE.ACTIVE || keepWarm;
      state = STATE.ACTIVE;
      keepWarm = false;
      if (changed) {
        generation += 1;
        publish("activate");
      }
      return result ?? changed;
    }

    function deactivate(options = {}) {
      assertNotDestroyed("deactivate");
      if (root === null || state !== STATE.ACTIVE) return false;
      const nextKeepWarm = Boolean(options.keepWarm);
      const result = hooks.deactivate?.({ ...options, keepWarm: nextKeepWarm });
      state = STATE.INACTIVE;
      keepWarm = nextKeepWarm;
      generation += 1;
      publish("deactivate");
      return result ?? true;
    }

    function resize(rect) {
      assertNotDestroyed("resize");
      if (root === null) return false;
      return hooks.resize?.(rect, { active: state === STATE.ACTIVE }) ?? false;
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
      if (state === STATE.DESTROYED) return false;
      let failure = null;
      try {
        if (state === STATE.ACTIVE) hooks.deactivate?.({ keepWarm: false, reason: "destroy" });
      } catch (error) {
        failure = error;
      }
      try {
        hooks.destroy?.(root);
      } catch (error) {
        failure ??= error;
      } finally {
        root = null;
        state = STATE.DESTROYED;
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

  function isContent(value) {
    return Boolean(value && typeof value === "object" && instances.has(value));
  }

  return Object.freeze({ STATE, HOOK_NAMES, create, isContent });
})();
