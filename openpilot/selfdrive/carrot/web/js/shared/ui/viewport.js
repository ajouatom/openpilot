"use strict";

/* ── Shared layout-mode resolver ──────────────────────────────────────────
   Single source of truth for the coarse app layout. Wide is not just
   "landscape": unfolded foldables and tablet-like panels can report a portrait
   viewport while still having enough room for the rail + split layout.

     wide = aspect >= 13/10 OR fold/segment API says segmented OR large panel
     tall/square = compact chrome, split layout disabled

   Keep the CSS media query copies in responsive.css, layout_tokens.css, and
   src/features/settings/styles/device.css in sync with LAYOUT_WIDE_QUERY. */
const LAYOUT_WIDE_QUERY = "(min-aspect-ratio: 13/10), (horizontal-viewport-segments: 2), (vertical-viewport-segments: 2), (min-width: 640px) and (min-height: 650px)";
const LAYOUT_WIDE_MIN_WIDTH = 640;
const LAYOUT_WIDE_MIN_HEIGHT = 650;
const LAYOUT_SQUARE_RATIO = 0.92;

function layoutViewportSize() {
  const root = document.documentElement;
  return {
    width: Math.max(0, Math.round(root?.clientWidth || window.innerWidth || 0)),
    height: Math.max(0, Math.round(root?.clientHeight || window.innerHeight || 0)),
  };
}

function hasViewportSegments() {
  const segments = window.viewport?.segments;
  return Array.isArray(segments) && segments.length > 1;
}

function getDevicePostureType() {
  const directType = navigator.devicePosture?.type;
  if (typeof directType === "string" && directType) return directType;
  if (typeof window.matchMedia !== "function") return "";
  try {
    if (window.matchMedia("(device-posture: folded)").matches) return "folded";
    if (window.matchMedia("(device-posture: continuous)").matches) return "continuous";
  } catch (e) {}
  return "";
}

function isWideLayout() {
  if (hasViewportSegments()) return true;
  if (typeof window.matchMedia === "function") {
    const mql = window.matchMedia(LAYOUT_WIDE_QUERY);
    if (mql && typeof mql.matches === "boolean") return mql.matches;
  }
  const { width, height } = layoutViewportSize();
  return width / Math.max(height, 1) >= 1.3 || (width >= LAYOUT_WIDE_MIN_WIDTH && height >= LAYOUT_WIDE_MIN_HEIGHT);
}

function getLayoutOrientation() {
  // Keep physical/window orientation separate from the coarse `wide` mode:
  // large portrait tablets may be wide-capable while portrait-only surfaces
  // still need to mount into the lower HUD dock.
  const { width, height } = layoutViewportSize();
  return width > height ? "landscape" : "portrait";
}

function computeLayoutMode() {
  if (isWideLayout()) return "wide";
  const { width: w, height: h } = layoutViewportSize();
  // Near-square (foldable unfolded, some multi-window splits) vs true portrait.
  // Both use the narrow chrome; the distinction is exposed for components that
  // want to use the extra width without re-enabling the wide chrome.
  return w / h >= LAYOUT_SQUARE_RATIO ? "square" : "tall";
}

window.CarrotLayout = {
  WIDE_QUERY: LAYOUT_WIDE_QUERY,
  isWide: isWideLayout,
  mode: computeLayoutMode,
  orientation: getLayoutOrientation,
  hasViewportSegments,
  devicePosture: getDevicePostureType,
};

let appViewportMetricsBound = false;
let appUnobscuredViewportHeight = 0;
let appKeyboardInset = 0;
let driveHudLayoutObserversBound = false;
let driveHudLayoutRaf = 0;


function updateAppViewportMetrics() {
  const vv = window.visualViewport;
  const rawHeight = Math.max(320, Math.round(vv?.height || window.innerHeight || 0));
  const rawTop = Math.max(0, Math.round(vv?.offsetTop || 0));
  const rawWidth = Math.max(320, Math.round(vv?.width || window.innerWidth || 0));

  // Some Samsung Internet builds expose VirtualKeyboard while also shrinking
  // the visual viewport. Treat that as a viewport-resize keyboard, not as an
  // overlay keyboard, otherwise consumers subtract the keyboard twice.
  const layoutHeight = Math.max(
    rawHeight,
    Math.round(document.documentElement?.clientHeight || 0),
    Math.round(window.innerHeight || 0),
  );
  const visualViewportKeyboardHeight = Math.max(0, Math.round(layoutHeight - rawHeight - rawTop));
  const virtualKeyboardHeight = Math.max(
    0,
    Math.round((appVirtualKeyboard?.boundingRect && appVirtualKeyboard.boundingRect.height) || 0),
  );

  // Keyboard-open state, so keyboard-aware surfaces can hug the keyboard when it
  // is up and center when it is not (desktop, keyboard-less, or before focus).
  // VirtualKeyboard bounding rect on Chromium (visualViewport doesn't shrink
  // there); the visualViewport delta elsewhere.
  const keyboardHeight = Math.max(virtualKeyboardHeight, visualViewportKeyboardHeight);
  const keyboardOpen = keyboardHeight > 120;
  if (!keyboardOpen) appUnobscuredViewportHeight = rawHeight;
  const viewportShrankForKeyboard = keyboardOpen && (
    visualViewportKeyboardHeight > 120
    || (appUnobscuredViewportHeight > 0 && rawHeight < appUnobscuredViewportHeight - 120)
  );
  // Only an actual overlay needs an extra bottom inset. A resized viewport has
  // already reserved the same space in --app-vv-height.
  appKeyboardInset = appVirtualKeyboard && !viewportShrankForKeyboard ? virtualKeyboardHeight : 0;
  if (keyboardOpen) document.documentElement.dataset.kbOpen = "1";
  else delete document.documentElement.dataset.kbOpen;

  const height = rawHeight;
  const top = rawTop;
  const width = rawWidth;
  document.documentElement.style.setProperty("--app-vv-height", `${height}px`);
  document.documentElement.style.setProperty("--app-vv-top", `${top}px`);
  document.documentElement.style.setProperty("--app-vv-width", `${width}px`);
  if (appVirtualKeyboard) {
    // Samsung Internet can expose VirtualKeyboard geometry to JS while the CSS
    // env(keyboard-inset-height) fallback stays stale or incomplete. Promote the
    // measured geometry into the shared token so dialogs and terminal controls
    // use the same keyboard inset on Chrome and Samsung.
    document.documentElement.style.setProperty("--kb-inset", `${appKeyboardInset}px`);
  } else {
    document.documentElement.style.removeProperty("--kb-inset");
  }

  const topbarEl = document.querySelector(".topbar");
  let navLeftGap = 0;
  let navBottomGap = 0;
  if (topbarEl) {
    const styles = window.getComputedStyle(topbarEl);
    if (styles.display !== "none" && styles.visibility !== "hidden") {
      const rect = topbarEl.getBoundingClientRect();
      const rectWidth = Math.max(0, Math.round(rect.width));
      const rectHeight = Math.max(0, Math.round(rect.height));
      const isRailLayout =
        rectWidth > 0 &&
        rectHeight > 0 &&
        rectHeight >= Math.max(Math.round(rectWidth * 1.25), Math.round(height * 0.5)) &&
        rectWidth <= Math.max(160, Math.round(width * 0.35));

      if (isRailLayout) {
        navLeftGap = rectWidth;
      } else if (rectHeight > 0) {
        const visibleTop = Math.max(0, Math.round(rect.top));
        const visibleBottom = Math.min(height, Math.round(rect.bottom));
        navBottomGap = Math.max(0, visibleBottom - visibleTop);
      }
    }
  }

  document.documentElement.style.setProperty("--app-nav-left-gap", `${navLeftGap}px`);
  document.documentElement.style.setProperty("--app-nav-bottom-gap", `${navBottomGap}px`);

  const layoutMode = computeLayoutMode();
  if (document.documentElement.dataset.layout !== layoutMode) {
    document.documentElement.dataset.layout = layoutMode;
  }
}

/* ── VirtualKeyboard API (Chromium: Android Chrome, Samsung Internet, Edge 94+) ─
   Opt into overlaysContent so the browser reports the *exact* on-screen keyboard
   geometry — including the suggestion / autofill toolbar strip that the
   visualViewport height alone misses on Samsung (the "172.30.1.42 / ID·비번" bar
   that was covering dialogs and the terminal input). Keyboard-aware surfaces
   then reserve `env(keyboard-inset-height)`. iOS Safari / Firefox lack the API,
   so `data-vk` stays unset, `env(keyboard-inset-*)` resolves to its 0 fallback,
   and those engines keep the visualViewport path (which already shrinks with the
   keyboard there). Unlike interactive-widget=resizes-content, overlaysContent
   does NOT resize the layout viewport, so it can't disturb the aspect-ratio
   layout mode. */
function initVirtualKeyboard() {
  try {
    const vk = navigator.virtualKeyboard;
    if (!vk) return null;
    vk.overlaysContent = true;
    document.documentElement.dataset.vk = "1";
    return vk;
  } catch (e) {
    return null;
  }
}

const appVirtualKeyboard = initVirtualKeyboard();

function setAppVirtualKeyboardOverlaysContent(enabled) {
  if (!appVirtualKeyboard) return false;
  try {
    appVirtualKeyboard.overlaysContent = !!enabled;
    if (enabled) document.documentElement.dataset.vk = "1";
    else delete document.documentElement.dataset.vk;
    updateAppViewportMetrics();
    return true;
  } catch (e) {
    return false;
  }
}

window.CarrotViewport = {
  ...(window.CarrotViewport || {}),
  setVirtualKeyboardOverlaysContent: setAppVirtualKeyboardOverlaysContent,
  updateMetrics: updateAppViewportMetrics,
};

/* overlaysContent=true also means the browser no longer auto-scrolls a focused
   input above the keyboard (it isn't resizing anything). Dialogs, the terminal
   and the search panel anchor themselves above the keyboard, but plain inline
   inputs (e.g. the profile-name field) would sit behind it. This safety net
   scrolls any other focused editable element into the visible area above the
   keyboard — so keyboard handling is consistent everywhere in the app. */
function isEditableTarget(el) {
  if (!el) return false;
  const tag = el.tagName;
  return tag === "INPUT" || tag === "TEXTAREA" || el.isContentEditable === true;
}

function ensureFocusedInputVisible(el) {
  if (!appVirtualKeyboard || !isEditableTarget(el)) return;
  if (el.closest(".app-dialog, .page--terminal, .setting-search-panel")) return;
  const kb = appKeyboardInset;
  if (kb <= 0) return;
  const rect = el.getBoundingClientRect();
  const visibleBottom = (window.innerHeight || 0) - kb - 24;
  if (rect.bottom > visibleBottom) {
    try {
      el.scrollIntoView({ block: "center", behavior: "smooth" });
    } catch (e) {}
  }
}

if (appVirtualKeyboard) {
  document.addEventListener(
    "focusin",
    (e) => {
      const el = e.target;
      setTimeout(() => ensureFocusedInputVisible(el), 250);
    },
    { passive: true },
  );
  appVirtualKeyboard.addEventListener(
    "geometrychange",
    () => ensureFocusedInputVisible(document.activeElement),
    { passive: true },
  );
}

function bindAppViewportObservers() {
  if (appViewportMetricsBound) return;
  appViewportMetricsBound = true;

  const handleLayout = () => requestAnimationFrame(updateAppViewportMetrics);
  updateAppViewportMetrics();
  window.addEventListener("resize", handleLayout, { passive: true });
  window.addEventListener("orientationchange", handleLayout, { passive: true });
  document.addEventListener("fullscreenchange", handleLayout);
  document.addEventListener("webkitfullscreenchange", handleLayout);
  if (window.visualViewport) {
    window.visualViewport.addEventListener("resize", handleLayout, { passive: true });
    window.visualViewport.addEventListener("scroll", handleLayout, { passive: true });
  }
  if (appVirtualKeyboard) {
    appVirtualKeyboard.addEventListener("geometrychange", handleLayout, { passive: true });
  }
}

bindAppViewportObservers();


function syncDriveHudLayout() {
  driveHudLayoutRaf = 0;
  window.DriveVisionHudContent?.resize?.();
}

function scheduleDriveHudLayout() {
  if (driveHudLayoutRaf) return;
  driveHudLayoutRaf = requestAnimationFrame(syncDriveHudLayout);
}

function bindDriveHudLayoutObservers() {
  if (driveHudLayoutObserversBound) return;
  driveHudLayoutObserversBound = true;

  const handleLayout = () => scheduleDriveHudLayout();
  window.addEventListener("resize", handleLayout, { passive: true });
  window.addEventListener("orientationchange", handleLayout, { passive: true });
  window.addEventListener("carrot:pagechange", handleLayout);
  if (window.visualViewport) {
    window.visualViewport.addEventListener("resize", handleLayout, { passive: true });
    window.visualViewport.addEventListener("scroll", handleLayout, { passive: true });
  }

  scheduleDriveHudLayout();
}

bindDriveHudLayoutObservers();
