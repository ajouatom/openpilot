const contentInstances = new WeakMap();

// Vision-owned lifecycle for the driving HUD. Other modules feed state through
// this API instead of controlling the overlay directly. Rendering belongs to the
// Carrot HUD overlay (hud/); this module only owns activation, suppression, and
// payload replay.
export function createDriveVisionHudContent(options = {}) {
  const target = options.target || globalThis;
  const documentRoot = options.document || target.document;
  const root = options.root || documentRoot?.getElementById?.("driveHudCard");
  const workspace = options.workspace || documentRoot?.getElementById?.("carrotDriveWorkspace");
  const stage = options.stage || documentRoot?.getElementById?.("carrotStage");
  if (!root || !workspace) return null;

  const suppressions = new Set();
  let mounted = false;
  let active = false;
  let destroyed = false;
  let lastPayload = null;

  function presentationOverlay() {
    return target.CarrotHudOverlay || null;
  }

  function viewportSize() {
    const viewport = target.visualViewport;
    return {
      width: Number(viewport?.width || documentRoot.documentElement.clientWidth || target.innerWidth || 0),
      height: Number(viewport?.height || documentRoot.documentElement.clientHeight || target.innerHeight || 0),
    };
  }

  function viewportOrientation() {
    const { width, height } = viewportSize();
    return height >= width ? "vertical" : "horizontal";
  }

  function syncPresentation() {
    const overlay = presentationOverlay();
    if (!overlay) return false;
    for (const reason of suppressions) overlay.setSuppressed?.(reason, true);
    if (active && !destroyed) overlay.activate?.();
    else overlay.deactivate?.();
    if (lastPayload) overlay.update?.(lastPayload);
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
    const overlay = presentationOverlay();
    syncPresentation();
    syncVisibility();
    overlay?.relayout?.(viewportSize());
    return changed;
  }

  function deactivate() {
    const changed = active;
    active = false;
    presentationOverlay()?.deactivate?.();
    syncVisibility();
    return changed;
  }

  function resize() {
    if (destroyed || !active) return false;
    syncVisibility();
    presentationOverlay()?.relayout?.(viewportSize());
    return true;
  }

  function update(payload) {
    if (!payload || destroyed) return false;
    // Held even while the overlay is missing: syncPresentation replays the last
    // payload as soon as the overlay installs.
    lastPayload = payload;
    presentationOverlay()?.update?.(payload);
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
    presentationOverlay()?.setSuppressed?.(key, Boolean(value));
    if (key === "replay-insights") root.classList.toggle("is-replay-suppressed", Boolean(value));
    return syncVisibility();
  }

  function renderText() {
    if (destroyed) return false;
    if (lastPayload) presentationOverlay()?.update?.(lastPayload);
    return true;
  }

  function status() {
    const presentation = presentationOverlay()?.status?.() || null;
    return Object.freeze({
      mounted,
      active,
      destroyed,
      orientation: viewportOrientation(),
      visible: !root.hidden && !root.classList.contains("is-vision-hud-suppressed"),
      suppressions: Object.freeze(Array.from(suppressions)),
      presentation,
    });
  }

  function destroy() {
    if (destroyed) return false;
    active = false;
    mounted = false;
    destroyed = true;
    lastPayload = null;
    suppressions.clear();
    presentationOverlay()?.destroy?.();
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
    syncPresentation,
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
