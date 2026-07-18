import { getDriveVisionHudRenderer } from "./hud_renderer.js";

const contentInstances = new WeakMap();

// Vision-owned lifecycle for the green driving HUD. Other modules feed state
// through this API instead of controlling #driveHudCard directly.
export function createDriveVisionHudContent(options = {}) {
  const target = options.target || globalThis;
  const documentRoot = options.document || target.document;
  const root = options.root || documentRoot?.getElementById?.("driveHudCard");
  const workspace = options.workspace || documentRoot?.getElementById?.("carrotDriveWorkspace");
  const stage = options.stage || documentRoot?.getElementById?.("carrotStage");
  const renderer = options.renderer || getDriveVisionHudRenderer(target);
  if (!root || !workspace || !renderer?.init || !renderer?.update) return null;

  const suppressions = new Set();
  let mounted = false;
  let active = false;
  let initialized = false;
  let destroyed = false;
  let lastPayload = null;

  function viewportOrientation() {
    const viewport = target.visualViewport;
    const width = Number(viewport?.width || documentRoot.documentElement.clientWidth || target.innerWidth || 0);
    const height = Number(viewport?.height || documentRoot.documentElement.clientHeight || target.innerHeight || 0);
    return height >= width ? "vertical" : "horizontal";
  }

  function ensureInitialized() {
    if (initialized || destroyed) return false;
    renderer.init();
    initialized = true;
    return true;
  }

  function syncVisibility() {
    const orientation = viewportOrientation();
    const compact = stage?.classList.contains("is-drive-workspace-compact-primary") === true;
    const suppressed = destroyed || !active || orientation === "vertical" || compact || suppressions.size > 0;
    root.dataset.visionHudOrientation = orientation;
    root.dataset.visionHudCompact = String(compact);
    root.classList.toggle("is-vision-hud-suppressed", suppressed);
    root.hidden = suppressed;
    root.inert = suppressed;
    if (suppressed) root.setAttribute("aria-hidden", "true");
    else root.removeAttribute("aria-hidden");
    return !suppressed;
  }

  function mount(nextRoot = root) {
    if (destroyed) throw new Error("Vision HUD cannot mount after destroy");
    if (nextRoot !== root) throw new Error("Vision HUD must use #driveHudCard");
    if (mounted) return false;
    mounted = true;
    syncVisibility();
    return true;
  }

  function activate() {
    if (destroyed) throw new Error("Vision HUD cannot activate after destroy");
    const changed = !active;
    mounted = true;
    active = true;
    ensureInitialized();
    if (lastPayload) renderer.update(lastPayload);
    syncVisibility();
    renderer.relayout?.();
    return changed;
  }

  function deactivate() {
    const changed = active;
    active = false;
    syncVisibility();
    return changed;
  }

  function resize() {
    if (destroyed || !active) return false;
    syncVisibility();
    renderer.relayout?.();
    return true;
  }

  function update(payload) {
    if (!payload || destroyed) return false;
    lastPayload = payload;
    ensureInitialized();
    renderer.update(payload);
    syncVisibility();
    return true;
  }

  function setLoading(value) {
    if (destroyed) return false;
    root.classList.toggle("driveHudCard--loading", Boolean(value));
    return true;
  }

  function setSuppressed(reason, value) {
    if (destroyed) return false;
    const key = String(reason || "external");
    if (value) suppressions.add(key);
    else suppressions.delete(key);
    if (key === "replay-insights") root.classList.toggle("is-replay-suppressed", Boolean(value));
    return syncVisibility();
  }

  function renderText() {
    if (destroyed) return false;
    ensureInitialized();
    renderer.renderText?.();
    return true;
  }

  function status() {
    return Object.freeze({
      mounted,
      active,
      initialized,
      destroyed,
      orientation: viewportOrientation(),
      visible: !root.hidden && !root.classList.contains("is-vision-hud-suppressed"),
      suppressions: Object.freeze(Array.from(suppressions)),
    });
  }

  function destroy() {
    if (destroyed) return false;
    active = false;
    mounted = false;
    destroyed = true;
    lastPayload = null;
    suppressions.clear();
    root.classList.remove("is-replay-suppressed", "driveHudCard--loading");
    syncVisibility();
    return true;
  }

  function handleLayoutChange() {
    if (!destroyed) resize();
  }

  for (const eventName of [
    "resize",
    "orientationchange",
    "carrot:viewportlayout",
    "carrot:pagechange",
    "drive:workspacelayoutchange",
  ]) target.addEventListener(eventName, handleLayoutChange, { passive: true });

  if (options.autoMount !== false) mount();
  return Object.freeze({
    root,
    mount,
    activate,
    deactivate,
    resize,
    update,
    setLoading,
    setSuppressed,
    renderText,
    status,
    destroy,
  });
}

export function getDriveVisionHudContent(target = globalThis, options = {}) {
  const existing = contentInstances.get(target);
  if (existing) return existing;
  const content = createDriveVisionHudContent({ ...options, target });
  if (content) contentInstances.set(target, content);
  return content;
}

export function installDriveVisionHudContentFacade(target = globalThis, options = {}) {
  const content = getDriveVisionHudContent(target, options);
  target.DriveVisionHudContent = content;
  return content;
}
