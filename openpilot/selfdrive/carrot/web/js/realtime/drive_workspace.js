"use strict";

globalThis.DriveWorkspace = (() => {
  const SLOT = Object.freeze({
    PRIMARY: "primary",
    SECONDARY: "secondary",
  });
  const ORIENTATION = Object.freeze({
    HORIZONTAL: "horizontal",
    VERTICAL: "vertical",
  });
  const LAYOUT_MODE = Object.freeze({
    SPLIT: "split",
    AREA_1: "area_1",
    AREA_2: "area_2",
  });
  const DEFAULT_GEOMETRY = Object.freeze({
    dividerWidth: 8,
    dividerHeight: 8,
    settleMs: 240,
    minRatio: 0.3,
    maxRatio: 0.7,
    compactPrimaryRatio: 0.5,
    compactPrimaryMinWidth: 0,
    compactPrimaryMinHeight: 0,
    ratioStep: 0.05,
    initialRatio: 0.7,
    verticalRatio: null,
  });
  const CORE_CLASS_NAMES = Object.freeze({
    active: "is-drive-workspace-active",
    resizing: "is-drive-workspace-resizing",
    settling: "is-drive-workspace-settling",
    primarySplit: "is-drive-workspace-split",
    primaryCompact: "is-drive-workspace-compact-primary",
    documentCompact: "is-drive-workspace-compact-primary",
    horizontal: "is-drive-workspace-horizontal",
    vertical: "is-drive-workspace-vertical",
    swapped: "is-drive-workspace-swapped",
    area1Only: "is-drive-workspace-area-1-only",
    area2Only: "is-drive-workspace-area-2-only",
  });
  const workspaces = new WeakMap();

  function create(options = {}) {
    const root = options.root;
    const primary = options.primary;
    const secondary = options.secondary;
    const divider = options.divider;
    if (!root || !primary || !secondary || !divider) return null;
    const existing = workspaces.get(root);
    if (existing) return existing;

    root.classList.add("drive-workspace");
    primary.dataset.driveWorkspaceSlot = SLOT.PRIMARY;
    secondary.dataset.driveWorkspaceSlot = SLOT.SECONDARY;
    divider.dataset.driveWorkspaceDivider = "";

    const geometry = { ...DEFAULT_GEOMETRY, ...(options.geometry || {}) };
    const classNames = { ...CORE_CLASS_NAMES, ...(options.classNames || {}) };
    const styleProperties = {
      dividerWidth: "--drive-workspace-divider-width",
      secondaryWidth: "--drive-workspace-secondary-width",
      dividerHeight: "--drive-workspace-divider-height",
      secondaryHeight: "--drive-workspace-secondary-height",
      ...(options.styleProperties || {}),
    };
    const events = {
      layout: "drive:workspacelayoutchange",
      resizeStart: "drive:workspaceresizestart",
      resizeEnd: "drive:workspaceresizeend",
      ...(options.events || {}),
    };

    let active = false;
    let orientation = normalizeOrientation(options.orientation);
    let layoutMode = normalizeLayoutMode(options.layout?.mode);
    let area1Slot = String(options.layout?.area1Slot || SLOT.PRIMARY);
    let ratios = null;
    let ratioFrameRequest = 0;
    let pendingRatio = null;
    let pointerId = null;
    let dragRect = null;
    let pointerOffset = 0;
    let settleTimer = 0;
    let appliedGeometrySignature = "";
    let contentResizeFrameRequest = 0;
    let forceContentResize = false;
    const contents = new Map([
      [SLOT.PRIMARY, null],
      [SLOT.SECONDARY, null],
    ]);
    const contentRectSignatures = new Map();

    function stateClassNames(key) {
      return [...new Set([CORE_CLASS_NAMES[key], classNames[key]].filter(Boolean))];
    }

    function toggleStateClass(element, key, enabled) {
      if (!element) return;
      for (const name of stateClassNames(key)) element.classList.toggle(name, enabled);
    }

    function hasStateClass(element, key) {
      return Boolean(element && stateClassNames(key).some((name) => element.classList.contains(name)));
    }

    function removeStateClasses(element, ...keys) {
      if (!element) return;
      const names = keys.flatMap(stateClassNames);
      if (names.length) element.classList.remove(...names);
    }

    function normalizeOrientation(value) {
      return String(value || "").toLowerCase() === ORIENTATION.VERTICAL
        ? ORIENTATION.VERTICAL
        : ORIENTATION.HORIZONTAL;
    }

    function normalizeLayoutMode(value) {
      const normalized = String(value || "").trim().toLowerCase();
      return Object.values(LAYOUT_MODE).includes(normalized) ? normalized : LAYOUT_MODE.SPLIT;
    }

    function effectiveRatio() {
      return ratios?.[orientation] ?? snap(geometry.initialRatio);
    }

    function syncDividerState() {
      const interactive = active && layoutMode === LAYOUT_MODE.SPLIT;
      divider.setAttribute("aria-disabled", String(!interactive));
      divider.setAttribute("aria-hidden", String(!interactive));
      divider.tabIndex = interactive ? 0 : -1;
    }

    function syncOrientationState() {
      const vertical = orientation === ORIENTATION.VERTICAL;
      toggleStateClass(root, "horizontal", !vertical);
      toggleStateClass(root, "vertical", vertical);
      root.dataset.driveWorkspaceOrientation = orientation;
      divider.setAttribute("aria-orientation", vertical ? "horizontal" : "vertical");
      syncDividerState();
    }

    function syncLayoutState() {
      const swapped = area1Slot === SLOT.SECONDARY;
      toggleStateClass(root, "swapped", swapped);
      toggleStateClass(root, "area1Only", layoutMode === LAYOUT_MODE.AREA_1);
      toggleStateClass(root, "area2Only", layoutMode === LAYOUT_MODE.AREA_2);
      root.dataset.driveWorkspaceMode = layoutMode;
      primary.dataset.driveWorkspaceArea = swapped ? LAYOUT_MODE.AREA_2 : LAYOUT_MODE.AREA_1;
      secondary.dataset.driveWorkspaceArea = swapped ? LAYOUT_MODE.AREA_1 : LAYOUT_MODE.AREA_2;
      syncDividerState();
    }

    function dispatch(name, detail) {
      if (name) globalThis.dispatchEvent(new CustomEvent(name, { detail }));
    }

    function normalizeSlot(slot) {
      const normalized = String(slot || "").trim().toLowerCase();
      if (!contents.has(normalized)) throw new RangeError(`Unknown DriveWorkspace slot: ${slot}`);
      return normalized;
    }

    function slotElement(slot) {
      return normalizeSlot(slot) === SLOT.PRIMARY ? primary : secondary;
    }

    function slotVisible(slot) {
      const normalized = normalizeSlot(slot);
      if (!active) return normalized === SLOT.PRIMARY;
      if (layoutMode === LAYOUT_MODE.SPLIT) return true;
      const visibleSlot = layoutMode === LAYOUT_MODE.AREA_1
        ? area1Slot
        : (area1Slot === SLOT.PRIMARY ? SLOT.SECONDARY : SLOT.PRIMARY);
      return normalized === visibleSlot;
    }

    function syncSlotPresentation() {
      for (const slot of Object.values(SLOT)) {
        const element = slotElement(slot);
        const visible = slotVisible(slot);
        element.hidden = !visible;
        element.inert = !visible;
        element.dataset.driveWorkspaceVisible = String(visible);
        if (visible) element.removeAttribute("aria-hidden");
        else element.setAttribute("aria-hidden", "true");
      }
      divider.hidden = !active || layoutMode !== LAYOUT_MODE.SPLIT;
      syncDividerState();
    }

    function contentChangeDetail(slot, action, content) {
      return {
        workspaceId: String(root.id || ""),
        slot,
        action,
        contentName: String(content?.name || ""),
      };
    }

    function measureSlot(slot) {
      const rect = slotElement(slot).getBoundingClientRect();
      return Object.freeze({
        x: Number(rect.x ?? rect.left ?? 0),
        y: Number(rect.y ?? rect.top ?? 0),
        top: Number(rect.top || 0),
        right: Number(rect.right || 0),
        bottom: Number(rect.bottom || 0),
        left: Number(rect.left || 0),
        width: Math.max(0, Number(rect.width || 0)),
        height: Math.max(0, Number(rect.height || 0)),
        devicePixelRatio: Math.max(1, Number(globalThis.devicePixelRatio || 1)),
      });
    }

    function rectSignature(rect) {
      return [
        rect.left,
        rect.top,
        rect.width,
        rect.height,
        rect.devicePixelRatio,
      ].map((value) => Number(value).toFixed(2)).join("|");
    }

    function resizeContent(slot, force = false) {
      const normalized = normalizeSlot(slot);
      const content = contents.get(normalized);
      if (!content || !slotVisible(normalized)) return false;
      const rect = measureSlot(normalized);
      const signature = rectSignature(rect);
      if (!force && contentRectSignatures.get(normalized) === signature) return false;
      content.resize(rect);
      contentRectSignatures.set(normalized, signature);
      return true;
    }

    function resizeContents(options = {}) {
      if (pointerId !== null || hasStateClass(root, "resizing")) return false;
      const force = Boolean(options.force);
      let changed = false;
      for (const slot of Object.values(SLOT)) changed = resizeContent(slot, force) || changed;
      return changed;
    }

    function scheduleContentResize(force = false) {
      forceContentResize = forceContentResize || Boolean(force);
      if (contentResizeFrameRequest) return;
      contentResizeFrameRequest = globalThis.requestAnimationFrame(() => {
        contentResizeFrameRequest = 0;
        const shouldForce = forceContentResize;
        forceContentResize = false;
        resizeContents({ force: shouldForce });
      });
    }

    function registerContent(slot, content) {
      const normalized = normalizeSlot(slot);
      if (!globalThis.DriveContent?.isContent?.(content)) {
        throw new TypeError(`DriveWorkspace ${normalized} requires a DriveContent instance`);
      }
      const current = contents.get(normalized);
      if (current === content) return false;
      if (current) throw new Error(`DriveWorkspace ${normalized} already contains ${current.name}`);
      contents.set(normalized, content);
      slotElement(normalized).dataset.driveContent = content.name;
      dispatch("drive:workspacecontentchange", contentChangeDetail(normalized, "register", content));
      syncSlotPresentation();
      syncContentActivity("workspace content registered");
      scheduleContentResize(true);
      return true;
    }

    function unregisterContent(slot, expectedContent = null) {
      const normalized = normalizeSlot(slot);
      const current = contents.get(normalized);
      if (!current || (expectedContent && current !== expectedContent)) return false;
      contents.set(normalized, null);
      contentRectSignatures.delete(normalized);
      delete slotElement(normalized).dataset.driveContent;
      dispatch("drive:workspacecontentchange", contentChangeDetail(normalized, "unregister", current));
      return true;
    }

    function getContent(slot) {
      return contents.get(normalizeSlot(slot));
    }

    function syncContentActivity(reason = "workspace layout") {
      let changed = false;
      for (const slot of Object.values(SLOT)) {
        const content = contents.get(slot);
        if (!content) continue;
        const status = content.status?.() || {};
        const shouldBeActive = slotVisible(slot);
        if (shouldBeActive && !status.active) {
          content.activate({ reason });
          changed = true;
        } else if (!shouldBeActive && status.active) {
          content.deactivate({ keepWarm: false, reason });
          changed = true;
        }
      }
      return changed;
    }

    function contentSnapshot() {
      const status = {};
      for (const slot of Object.values(SLOT)) {
        const content = contents.get(slot);
        status[slot] = content?.status?.() || null;
      }
      return status;
    }

    function clamp(value) {
      const numeric = Number(value);
      const configuredFallback = Number(geometry.initialRatio);
      const fallback = Number.isFinite(configuredFallback) ? configuredFallback : DEFAULT_GEOMETRY.initialRatio;
      return Math.max(geometry.minRatio, Math.min(
        geometry.maxRatio,
        Number.isFinite(numeric) ? numeric : fallback,
      ));
    }

    function snap(value) {
      const clamped = clamp(value);
      const stepped = Math.round(clamped / geometry.ratioStep) * geometry.ratioStep;
      const normalized = Math.max(geometry.minRatio, Math.min(geometry.maxRatio, stepped));
      return Number(normalized.toFixed(2));
    }

    ratios = {
      [ORIENTATION.HORIZONTAL]: snap(geometry.initialRatio),
      [ORIENTATION.VERTICAL]: snap(
        Number.isFinite(Number(geometry.verticalRatio)) ? geometry.verticalRatio : geometry.initialRatio,
      ),
    };
    area1Slot = normalizeSlot(area1Slot);
    root.dataset.driveWorkspaceActive = String(active);
    syncOrientationState();
    syncLayoutState();
    syncSlotPresentation();

    function syncCompactPrimary(measuredExtent = 0) {
      const area1Ratio = effectiveRatio();
      const primaryRatio = area1Slot === SLOT.PRIMARY ? area1Ratio : 1 - area1Ratio;
      const vertical = orientation === ORIENTATION.VERTICAL;
      const availableExtent = Math.max(0, measuredExtent || (vertical ? root.clientHeight : root.clientWidth));
      const dividerExtent = active && layoutMode === LAYOUT_MODE.SPLIT
        ? (vertical ? geometry.dividerHeight : geometry.dividerWidth)
        : 0;
      const primaryExtent = Math.max(0, availableExtent - dividerExtent) * primaryRatio;
      const crossExtent = Math.max(0, vertical ? root.clientWidth : root.clientHeight);
      const primaryWidth = vertical ? crossExtent : primaryExtent;
      const primaryHeight = vertical ? primaryExtent : crossExtent;
      const belowReadableSize = primaryWidth < geometry.compactPrimaryMinWidth
        || primaryHeight < geometry.compactPrimaryMinHeight;
      const compactPrimary = active
        && layoutMode === LAYOUT_MODE.SPLIT
        && (primaryRatio <= geometry.compactPrimaryRatio || belowReadableSize);
      toggleStateClass(primary, "primaryCompact", compactPrimary);
      toggleStateClass(document.documentElement, "documentCompact", compactPrimary);
      return compactPrimary;
    }

    function layoutDetail() {
      return {
        workspaceId: String(root.id || ""),
        active,
        ratio: effectiveRatio(),
        ratios: { ...ratios },
        orientation,
        mode: layoutMode,
        area1Slot,
        visibleSlots: Object.values(SLOT).filter(slotVisible),
        compactPrimary: syncCompactPrimary(),
      };
    }

    function dispatchLayout() {
      const detail = layoutDetail();
      dispatch(events.layout, detail);
      options.onLayout?.(detail);
    }

    function persist(targetOrientation = orientation) {
      const normalized = normalizeOrientation(targetOrientation);
      options.persist?.(ratios[normalized], normalized);
    }

    function applyGeometry(renderContent = true, measuredExtent = 0) {
      const vertical = orientation === ORIENTATION.VERTICAL;
      const currentRatio = effectiveRatio();
      const splitActive = active && layoutMode === LAYOUT_MODE.SPLIT;
      const availableExtent = Math.max(0, measuredExtent || (vertical ? root.clientHeight : root.clientWidth));
      const compactPrimary = syncCompactPrimary(availableExtent);
      const configuredDividerExtent = vertical ? geometry.dividerHeight : geometry.dividerWidth;
      const dividerExtent = splitActive && availableExtent > configuredDividerExtent ? configuredDividerExtent : 0;
      const secondaryExtent = dividerExtent
        ? Math.max(0, (availableExtent - configuredDividerExtent) * (1 - currentRatio))
        : 0;
      const dividerWidth = vertical ? 0 : dividerExtent;
      const secondaryWidth = vertical ? 0 : secondaryExtent;
      const dividerHeight = vertical ? dividerExtent : 0;
      const secondaryHeight = vertical ? secondaryExtent : 0;
      const ratioPercent = Math.round(currentRatio * 100);
      const ratioLabel = `${ratioPercent}%`;
      if (divider.dataset.ratio !== ratioLabel) divider.dataset.ratio = ratioLabel;
      const secondaryRatioLabel = `${100 - ratioPercent}%`;
      const primaryRatioLabel = area1Slot === SLOT.PRIMARY ? ratioLabel : secondaryRatioLabel;
      const secondarySlotRatioLabel = area1Slot === SLOT.SECONDARY ? ratioLabel : secondaryRatioLabel;
      if (primary.dataset.splitRatio !== primaryRatioLabel) primary.dataset.splitRatio = primaryRatioLabel;
      if (secondary.dataset.splitRatio !== secondarySlotRatioLabel) secondary.dataset.splitRatio = secondarySlotRatioLabel;
      if (divider.getAttribute("aria-valuenow") !== String(ratioPercent)) {
        divider.setAttribute("aria-valuenow", String(ratioPercent));
      }
      const geometrySignature = [
        orientation,
        layoutMode,
        area1Slot,
        dividerWidth,
        secondaryWidth.toFixed(2),
        dividerHeight,
        secondaryHeight.toFixed(2),
        compactPrimary ? 1 : 0,
      ].join("|");
      const geometryChanged = geometrySignature !== appliedGeometrySignature;
      if (geometryChanged) {
        appliedGeometrySignature = geometrySignature;
        root.style.setProperty(styleProperties.dividerWidth, `${dividerWidth}px`);
        root.style.setProperty(styleProperties.secondaryWidth, `${secondaryWidth}px`);
        root.style.setProperty(styleProperties.dividerHeight, `${dividerHeight}px`);
        root.style.setProperty(styleProperties.secondaryHeight, `${secondaryHeight}px`);
        options.onGeometry?.({ active, ratio: currentRatio, orientation, mode: layoutMode, area1Slot, compactPrimary, renderContent });
      }
      if (renderContent) scheduleContentResize();
      return geometryChanged;
    }

    function cancelDrag() {
      const previousPointerId = pointerId;
      const wasDragging = previousPointerId !== null || hasStateClass(root, "resizing");
      pointerId = null;
      dragRect = null;
      pointerOffset = 0;
      if (previousPointerId !== null && divider.hasPointerCapture?.(previousPointerId)) {
        divider.releasePointerCapture?.(previousPointerId);
      }
      if (ratioFrameRequest) cancelAnimationFrame(ratioFrameRequest);
      ratioFrameRequest = 0;
      globalThis.clearTimeout(settleTimer);
      settleTimer = 0;
      if (pendingRatio !== null) ratios[orientation] = pendingRatio;
      pendingRatio = null;
      ratios[orientation] = snap(ratios[orientation]);
      syncCompactPrimary();
      removeStateClasses(root, "resizing", "settling");
      options.onResizeStateChange?.(false);
      if (wasDragging) {
        const detail = layoutDetail();
        dispatch(events.resizeEnd, detail);
        options.onResizeEnd?.(detail);
      }
    }

    function setActive(value) {
      const next = Boolean(value);
      if (next === active) return false;
      active = next;
      root.dataset.driveWorkspaceActive = String(active);
      if (active) {
        syncLayoutState();
        toggleStateClass(root, "active", true);
        toggleStateClass(primary, "primarySplit", layoutMode === LAYOUT_MODE.SPLIT);
        syncSlotPresentation();
        applyGeometry(false);
        syncContentActivity("workspace activated");
        requestAnimationFrame(() => {
          if (!active) return;
          scheduleContentResize(true);
        });
      } else {
        cancelDrag();
        toggleStateClass(root, "active", false);
        toggleStateClass(primary, "primarySplit", false);
        syncSlotPresentation();
        syncContentActivity("workspace deactivated");
        applyGeometry();
      }
      dispatchLayout();
      return true;
    }

    function flushPendingRatio() {
      ratioFrameRequest = 0;
      if (pendingRatio === null) return;
      ratios[orientation] = clamp(pendingRatio);
      pendingRatio = null;
      const extent = orientation === ORIENTATION.VERTICAL ? dragRect?.height : dragRect?.width;
      applyGeometry(false, extent || 0);
    }

    function queueRatio(value) {
      pendingRatio = snap(value);
      if (!ratioFrameRequest) ratioFrameRequest = requestAnimationFrame(flushPendingRatio);
    }

    function updateFromPointer(event) {
      if (!dragRect || layoutMode !== LAYOUT_MODE.SPLIT) return;
      const coalesced = event.getCoalescedEvents?.();
      const point = coalesced?.length ? coalesced[coalesced.length - 1] : event;
      const vertical = orientation === ORIENTATION.VERTICAL;
      const dividerExtent = vertical ? geometry.dividerHeight : geometry.dividerWidth;
      const dragExtent = vertical ? dragRect.height : dragRect.width;
      if (dragExtent <= dividerExtent) return;
      const pointPosition = vertical ? point.clientY : point.clientX;
      const dragStart = vertical ? dragRect.top : dragRect.left;
      const dividerCenter = pointPosition - pointerOffset;
      queueRatio(
        (dividerCenter - dragStart - (dividerExtent / 2))
        / (dragExtent - dividerExtent),
      );
    }

    function finishDrag(event) {
      if (pointerId === null || (event && event.pointerId !== pointerId)) return;
      const previousPointerId = pointerId;
      pointerId = null;
      dragRect = null;
      pointerOffset = 0;
      if (divider.hasPointerCapture?.(previousPointerId)) divider.releasePointerCapture?.(previousPointerId);
      if (ratioFrameRequest) cancelAnimationFrame(ratioFrameRequest);
      ratioFrameRequest = 0;
      if (pendingRatio !== null) ratios[orientation] = clamp(pendingRatio);
      pendingRatio = null;
      toggleStateClass(root, "resizing", false);
      toggleStateClass(root, "settling", true);
      ratios[orientation] = snap(ratios[orientation]);
      options.onResizeStateChange?.(false);
      applyGeometry();
      globalThis.clearTimeout(settleTimer);
      settleTimer = globalThis.setTimeout(() => {
        settleTimer = 0;
        toggleStateClass(root, "settling", false);
      }, geometry.settleMs);
      persist();
      dispatchLayout();
      const detail = layoutDetail();
      dispatch(events.resizeEnd, detail);
      options.onResizeEnd?.(detail);
    }

    divider.addEventListener("pointerdown", (event) => {
      if (!active || layoutMode !== LAYOUT_MODE.SPLIT) return;
      event.preventDefault();
      pointerId = event.pointerId;
      dragRect = root.getBoundingClientRect();
      const dividerRect = divider.getBoundingClientRect();
      pointerOffset = orientation === ORIENTATION.VERTICAL
        ? event.clientY - (dividerRect.top + (dividerRect.height / 2))
        : event.clientX - (dividerRect.left + (dividerRect.width / 2));
      globalThis.clearTimeout(settleTimer);
      settleTimer = 0;
      toggleStateClass(root, "settling", false);
      toggleStateClass(root, "resizing", true);
      options.onResizeStateChange?.(true);
      const detail = layoutDetail();
      dispatch(events.resizeStart, detail);
      options.onResizeStart?.(detail);
      divider.setPointerCapture?.(event.pointerId);
      updateFromPointer(event);
    });
    divider.addEventListener("pointermove", (event) => {
      if (event.pointerId !== pointerId || !divider.hasPointerCapture?.(event.pointerId)) return;
      updateFromPointer(event);
    });
    for (const eventName of ["pointerup", "pointercancel", "lostpointercapture"]) {
      divider.addEventListener(eventName, finishDrag);
    }
    // Pointer capture can be interrupted by viewport rotation, embedded browser
    // chrome, or a pointer leaving the divider. Window-level fallbacks keep the
    // drag lifecycle and persisted ratio consistent across those environments.
    globalThis.addEventListener("pointermove", (event) => {
      if (event.pointerId !== pointerId) return;
      updateFromPointer(event);
    }, { passive: true });
    globalThis.addEventListener("pointerup", finishDrag, { capture: true, passive: true });
    globalThis.addEventListener("pointercancel", finishDrag, { capture: true, passive: true });
    globalThis.addEventListener("blur", () => finishDrag(), { passive: true });
    divider.addEventListener("keydown", (event) => {
      if (!active || layoutMode !== LAYOUT_MODE.SPLIT) return;
      const decreaseKey = orientation === ORIENTATION.VERTICAL ? "ArrowUp" : "ArrowLeft";
      const increaseKey = orientation === ORIENTATION.VERTICAL ? "ArrowDown" : "ArrowRight";
      if (![decreaseKey, increaseKey, "Home", "End"].includes(event.key)) return;
      event.preventDefault();
      if (event.key === "Home") ratios[orientation] = geometry.minRatio;
      else if (event.key === "End") ratios[orientation] = geometry.maxRatio;
      else ratios[orientation] = snap(
        ratios[orientation] + (event.key === decreaseKey ? -geometry.ratioStep : geometry.ratioStep),
      );
      applyGeometry();
      persist();
      dispatchLayout();
    });

    globalThis.addEventListener("resize", () => scheduleContentResize(true), { passive: true });
    globalThis.addEventListener("orientationchange", () => scheduleContentResize(true), { passive: true });
    if (globalThis.visualViewport) {
      globalThis.visualViewport.addEventListener("resize", () => scheduleContentResize(true), { passive: true });
    }
    if (typeof globalThis.ResizeObserver === "function") {
      const workspaceResizeObserver = new globalThis.ResizeObserver(() => {
        if (active) applyGeometry();
        else scheduleContentResize(true);
      });
      workspaceResizeObserver.observe(root);
      const contentResizeObserver = new globalThis.ResizeObserver(() => scheduleContentResize());
      contentResizeObserver.observe(primary);
      contentResizeObserver.observe(secondary);
    }

    function setOrientation(value) {
      const nextOrientation = normalizeOrientation(value);
      if (nextOrientation === orientation) return false;
      cancelDrag();
      orientation = nextOrientation;
      appliedGeometrySignature = "";
      syncOrientationState();
      syncLayoutState();
      applyGeometry();
      dispatchLayout();
      return true;
    }

    function setRatio(value, setOptions = {}) {
      const config = typeof setOptions === "boolean" ? { persist: setOptions } : setOptions;
      const targetOrientation = normalizeOrientation(config.orientation || orientation);
      // Resize/viewport/settings observers may re-apply the last persisted
      // value while the user is still dragging. The active pointer owns the
      // current orientation until finishDrag() commits it.
      if (targetOrientation === orientation && pointerId !== null) return false;
      const nextRatio = snap(value);
      if (ratios[targetOrientation] === nextRatio) return false;
      ratios[targetOrientation] = nextRatio;
      if (targetOrientation === orientation) applyGeometry();
      if (config.persist) persist(targetOrientation);
      if (targetOrientation === orientation) dispatchLayout();
      return true;
    }

    function setLayout(value = {}) {
      const nextMode = normalizeLayoutMode(value.mode);
      const nextArea1Slot = normalizeSlot(value.area1Slot || SLOT.PRIMARY);
      if (nextMode === layoutMode && nextArea1Slot === area1Slot) return false;
      cancelDrag();
      layoutMode = nextMode;
      area1Slot = nextArea1Slot;
      appliedGeometrySignature = "";
      syncLayoutState();
      syncSlotPresentation();
      toggleStateClass(primary, "primarySplit", active && layoutMode === LAYOUT_MODE.SPLIT);
      syncContentActivity("workspace layout changed");
      applyGeometry();
      dispatchLayout();
      return true;
    }

    function snapshot() {
      return layoutDetail();
    }

    const api = Object.freeze({
      setActive,
      setOrientation,
      setRatio,
      setLayout,
      applyGeometry,
      cancelDrag,
      snapshot,
      registerContent,
      unregisterContent,
      getContent,
      contentSnapshot,
      syncContentActivity,
      resizeContents,
    });
    workspaces.set(root, api);
    return api;
  }

  function get(root) {
    return root && typeof root === "object" ? (workspaces.get(root) || null) : null;
  }

  return Object.freeze({ SLOT, ORIENTATION, LAYOUT_MODE, create, get });
})();
