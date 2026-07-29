"use strict";

(function () {
  const ENTER_MAX_WIDTH = 450;
  const EXIT_MIN_WIDTH = 520;
  const MIN_HEIGHT = 96;
  const COMPACT_MAX_HEIGHT = 760;
  const LARGE_CANVAS_MIN = 700;
  const PARAM = "mini_hud";

  let active = false;
  let bound = false;
  let rafId = 0;
  let overlayObserver = null;
  let lastDecisionLogKey = "";

  function readRequestMode() {
    const search = new URLSearchParams(window.location.search);
    const value = search.get(PARAM);
    if (value === "0" || value === "off") return "off";
    if (value === "1" || value === "force") return "force";
    if (value === "auto") return "auto";
    return "detect";
  }

  // Web setting gate. Only the default "detect" path is gated; explicit URL
  // params (off/force/auto) stay honored as overrides.
  function settingEnabled() {
    const settings = window.CarrotWebSettingsState || {};
    const value = Object.prototype.hasOwnProperty.call(settings, "mini_hud_enabled")
      ? settings.mini_hud_enabled
      : true;
    if (typeof value === "string") return ["1", "true", "yes", "on"].includes(value.trim().toLowerCase());
    return Boolean(value);
  }

  function viewportMetrics() {
    const viewport = window.visualViewport;
    const root = document.documentElement;
    const layoutWidth = Math.max(0, Math.round(root?.clientWidth || window.innerWidth || 0));
    const layoutHeight = Math.max(0, Math.round(root?.clientHeight || window.innerHeight || 0));
    const visualWidth = Math.max(0, Math.round(viewport?.width || window.innerWidth || layoutWidth || 0));
    const visualHeight = Math.max(0, Math.round(viewport?.height || window.innerHeight || layoutHeight || 0));
    return {
      width: layoutWidth,
      height: Math.min(layoutHeight || visualHeight, visualHeight || layoutHeight),
      layoutWidth,
      layoutHeight,
      visualWidth,
      visualHeight,
    };
  }

  function currentPage() {
    return document.body?.dataset.page || document.documentElement?.dataset.page || "";
  }

  function isRendered(el) {
    if (!el || el.hidden || el.closest?.("[hidden]")) return false;
    const style = window.getComputedStyle(el);
    if (style.display === "none" || style.visibility === "hidden") return false;
    return el.getClientRects().length > 0;
  }

  function hasBlockingOverlay() {
    if (document.body?.classList.contains("dialog-open")) return true;

    const fixedOverlays = [
      "appDialog",
      "appBranchPicker",
      "appCarPicker",
      "settingSearchPanel",
    ];
    if (fixedOverlays.some((id) => isRendered(document.getElementById(id)))) return true;

    const modalSelector = [
      '[role="dialog"][aria-modal="true"]',
      ".training-guide-modal",
      ".dashcam-player-overlay.is-open",
    ].join(",");
    return Array.from(document.querySelectorAll(modalSelector)).some(isRendered);
  }

  function windowedEvidence(metrics) {
    if (metrics.width <= 280) return { detected: true, reason: "ultra-narrow" };

    const screenWidth = Math.max(Number(window.screen?.availWidth) || 0, Number(window.screen?.width) || 0);
    const screenHeight = Math.max(Number(window.screen?.availHeight) || 0, Number(window.screen?.height) || 0);
    const outerWidth = Number(window.outerWidth) || 0;

    const narrowerThanScreen = screenWidth > 0 && metrics.width <= screenWidth * 0.78;
    const narrowerThanWindow = outerWidth > 0 && metrics.width <= outerWidth * 0.80;
    const compactOnLargeCanvas =
      metrics.width <= ENTER_MAX_WIDTH &&
      metrics.height <= COMPACT_MAX_HEIGHT &&
      (screenWidth >= LARGE_CANVAS_MIN || outerWidth >= LARGE_CANVAS_MIN);

    if (narrowerThanScreen) return { detected: true, reason: "narrower-than-screen" };
    if (narrowerThanWindow) return { detected: true, reason: "narrower-than-window" };
    if (compactOnLargeCanvas) return { detected: true, reason: "compact-on-large-canvas" };
    return {
      detected: false,
      reason: "no-horizontal-windowing",
      screen: { width: screenWidth, height: screenHeight },
      outer: { width: outerWidth, height: Number(window.outerHeight) || 0 },
    };
  }

  function hasWindowedEvidence(metrics) {
    return windowedEvidence(metrics).detected;
  }

  function evaluateActivation() {
    const requestMode = readRequestMode();
    const metrics = viewportMetrics();
    const page = currentPage();
    const overlayOpen = hasBlockingOverlay();
    const windowed = windowedEvidence(metrics);
    const result = {
      active: false,
      reason: "idle",
      requestMode,
      page,
      overlayOpen,
      metrics,
      windowed,
      thresholds: {
        enterMaxWidth: ENTER_MAX_WIDTH,
        exitMinWidth: EXIT_MIN_WIDTH,
        minHeight: MIN_HEIGHT,
      },
    };

    if (requestMode === "off") {
      result.reason = "request-off";
      return result;
    }
    if (requestMode === "force") {
      result.active = true;
      result.reason = "request-force";
      return result;
    }

    if (requestMode === "detect" && !settingEnabled()) {
      result.reason = "setting-off";
      return result;
    }

    if (overlayOpen) {
      result.reason = "overlay-open";
      return result;
    }

    if (metrics.height < MIN_HEIGHT) {
      result.reason = "height-too-small";
      return result;
    }

    const eligible = requestMode === "auto" || windowed.detected;
    if (!eligible) {
      result.reason = "not-windowed";
      return result;
    }

    if (active) {
      result.active = metrics.width < EXIT_MIN_WIDTH;
      result.reason = result.active ? "hold-below-exit-width" : "exit-width";
      return result;
    }

    result.active = metrics.width <= ENTER_MAX_WIDTH;
    result.reason = result.active ? "enter-width" : "above-enter-width";
    return result;
  }

  function shouldActivate() {
    return evaluateActivation().active;
  }

  function logDecision(next, reason, evaluation) {
    const tone = next ? "active" : "idle";
    console.log(`[mini_hud] ${tone}`, {
      event: reason,
      decision: evaluation.reason,
      requestMode: evaluation.requestMode,
      page: evaluation.page,
      overlayOpen: evaluation.overlayOpen,
      metrics: evaluation.metrics,
      windowed: evaluation.windowed,
      thresholds: evaluation.thresholds,
    });
  }

  function apply(evaluation, reason) {
    const next = Boolean(evaluation?.active);
    const metrics = evaluation?.metrics || {};
    const logKey = [
      next ? "1" : "0",
      evaluation?.reason || "",
      evaluation?.requestMode || "",
      evaluation?.overlayOpen ? "overlay" : "clear",
      evaluation?.windowed?.reason || "",
      metrics.width || 0,
      metrics.height || 0,
    ].join("|");
    if (active === next) {
      if (logKey !== lastDecisionLogKey) {
        lastDecisionLogKey = logKey;
        logDecision(next, `stable:${reason}`, evaluation);
      }
      return;
    }
    active = next;
    lastDecisionLogKey = logKey;

    const root = document.getElementById("carrotMiniHud");
    if (root) root.hidden = !active;

    if (active) document.documentElement.dataset.carrotMiniHud = "1";
    else delete document.documentElement.dataset.carrotMiniHud;

    logDecision(active, reason, evaluation);

    window.dispatchEvent(new CustomEvent("carrot:minihudchange", {
      detail: { active, reason, decision: evaluation.reason, page: evaluation.page, overlayOpen: evaluation.overlayOpen, metrics: evaluation.metrics },
    }));
  }

  function sync(reason = "sync") {
    apply(evaluateActivation(), reason);
  }

  function schedule(reason) {
    if (rafId) return;
    rafId = requestAnimationFrame(() => {
      rafId = 0;
      sync(reason);
    });
  }

  function bind() {
    if (bound) return;
    bound = true;

    const onResize = () => schedule("resize");
    window.addEventListener("resize", onResize, { passive: true });
    window.addEventListener("orientationchange", onResize, { passive: true });
    window.addEventListener("carrot:pagechange", () => schedule("pagechange"));
    window.addEventListener("carrot:websettingschange", (event) => {
      if (!event?.detail || event.detail.key === "mini_hud_enabled") schedule("websettingschange");
    });
    if (window.visualViewport) {
      window.visualViewport.addEventListener("resize", onResize, { passive: true });
    }
    if (window.MutationObserver && document.body) {
      overlayObserver = new MutationObserver(() => schedule("overlay"));
      overlayObserver.observe(document.body, {
        attributes: true,
        attributeFilter: ["class", "hidden", "aria-hidden"],
        childList: true,
        subtree: true,
      });
    }
    sync("init");
  }

  window.CarrotMiniHudMode = {
    bind,
    sync,
    isActive: () => active,
    requestMode: readRequestMode,
    thresholds: Object.freeze({ enterMaxWidth: ENTER_MAX_WIDTH, exitMinWidth: EXIT_MIN_WIDTH, minHeight: MIN_HEIGHT }),
    hasWindowedEvidence,
    windowedEvidence,
    hasBlockingOverlay,
    currentPage,
    evaluateActivation,
  };
})();
