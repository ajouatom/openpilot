"use strict";

(function () {
  const root = document.getElementById("carrotMiniHud");
  if (!root) return;

  const elements = {
    cpu: root.querySelector("[data-mini-hud-cpu]"),
    source: root.querySelector("[data-mini-hud-source]"),
    limitZone: root.querySelector("[data-mini-hud-limit-zone]"),
    limit: root.querySelector("[data-mini-hud-limit]"),
    limitLabel: root.querySelector("[data-mini-hud-limit-label]"),
    limitBadge: root.querySelector("[data-mini-hud-limit-badge]"),
    alertZone: root.querySelector("[data-mini-hud-alert-zone]"),
    countdown: root.querySelector("[data-mini-hud-countdown]"),
    distance: root.querySelector("[data-mini-hud-distance]"),
    alert: root.querySelector("[data-mini-hud-alert]"),
    stockZone: root.querySelector("[data-mini-hud-stock-zone]"),
    driveMode: root.querySelector("[data-mini-hud-drive-mode]"),
    driveModeFrame: root.querySelector(".carrot-mini-hud__drive-mode"),
    gearDrive: root.querySelector(".carrot-mini-hud__gear small"),
    gear: root.querySelector("[data-mini-hud-gear]"),
    speed: root.querySelector("[data-mini-hud-speed]"),
    setSpeed: root.querySelector("[data-mini-hud-set-speed]"),
    speedCell: root.querySelector(".carrot-mini-hud__speed"),
    setSpeedCell: root.querySelector(".carrot-mini-hud__set-speed"),
    main: root.querySelector(".carrot-mini-hud__main"),
  };

  const canvas = document.createElement("canvas");
  const context = canvas.getContext("2d");
  let latestModel = null;
  let layoutRaf = 0;
  let resizeObserver = null;

  function clamp(value, min, max) {
    return Math.min(max, Math.max(min, value));
  }

  function setText(element, value) {
    if (!element) return;
    const text = value == null ? "" : String(value);
    if (element.textContent !== text) element.textContent = text;
  }

  function setHidden(element, hidden) {
    if (element) element.hidden = Boolean(hidden);
  }

  function measure(text, size, family = 'system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif') {
    if (!context) return String(text).length * size * 0.62;
    context.font = `900 ${size}px ${family}`;
    return context.measureText(String(text)).width;
  }

  function contentWidth(element, fallback) {
    if (!element) return fallback;
    const rect = element.getBoundingClientRect();
    const style = getComputedStyle(element);
    const padding = (parseFloat(style.paddingLeft) || 0) + (parseFloat(style.paddingRight) || 0);
    const inner = rect.width - padding;
    // A grid/flex cell can measure ~0 for a frame right as Compact HUD activates
    // (layout not resolved yet). Using that 0 collapsed the auto-fit font to 1px
    // and it stuck. Fall back to the proportional estimate instead.
    return inner > 4 ? inner : fallback;
  }

  function render(model) {
    if (!model) return;
    latestModel = model;

    const stock = model.source === "stock";
    const limitVisible = !stock && model.roadLimit && model.roadLimit !== "--";
    const alertVisible = !stock && Boolean(model.alert?.visible);

    root.dataset.source = model.source;
    root.dataset.alertKind = model.alert?.kind || "none";
    root.classList.toggle("is-three-digit", String(model.speed).length >= 3 || String(model.setSpeed).length >= 3);

    setText(elements.cpu, `CPU:${model.cpu == null ? "--" : model.cpu}°`);
    setText(elements.source, model.source.toUpperCase());
    setText(elements.speed, model.speed);
    setText(elements.setSpeed, model.setSpeed);

    setHidden(elements.limitZone, !limitVisible);
    setText(elements.limit, model.roadLimit);
    setText(elements.limitLabel, model.alert?.section ? "구간단속" : "");
    setHidden(elements.limitLabel, !model.alert?.section);
    setText(elements.limitBadge, model.alert?.badge || "");
    setHidden(elements.limitBadge, !model.alert?.badge);

    setHidden(elements.alertZone, !alertVisible);
    setText(elements.countdown, model.alert?.countdown || "");
    setHidden(elements.countdown, !model.alert?.countdown);
    setText(elements.distance, model.alert?.distance || "");
    setHidden(elements.distance, !model.alert?.distance);
    setText(elements.alert, model.alert?.name || "");

    setHidden(elements.stockZone, !stock);
    setText(elements.driveMode, model.driveMode?.name || "NORMAL");
    if (elements.driveModeFrame) elements.driveModeFrame.dataset.miniHudDriveKind = model.driveMode?.kind || "normal";
    setText(elements.gear, model.gearStep || "--");
    setHidden(elements.gearDrive, model.gearStep == null);

    scheduleLayout();
  }

  function applyLayout() {
    layoutRaf = 0;
    if (!window.CarrotMiniHudMode?.isActive?.() || root.hidden) return;

    const rect = root.getBoundingClientRect();
    // Degenerate frame during activation — skip so we never bake a 1px font from
    // a not-yet-laid-out root; a later resize/render tick re-runs the layout.
    if (rect.width < 40 || rect.height < 40) {
      scheduleLayout();
      return;
    }
    const width = Math.max(1, rect.width);
    // Width-referenced scale, exactly like the preview (reference width 404).
    const layoutScale = clamp(width / 404, 0.46, 1);

    const topCells = root.querySelectorAll(".carrot-mini-hud__top-cell");
    const tempCellWidth = contentWidth(topCells[0], width * 0.50);
    const sourceCellWidth = contentWidth(topCells[1], width * 0.50);
    const tempText = elements.cpu?.textContent || "CPU:--°";
    const sourceText = elements.source?.textContent || "STOCK";
    const tempUnit = Math.max(1, measure(tempText, 100) / 100);
    const sourceUnit = Math.max(1, measure(sourceText, 100) / 100);
    const tempFont = Math.max(1, Math.min(32 * layoutScale, tempCellWidth * 0.90 / tempUnit));
    const sourceFont = Math.max(1, Math.min(32 * layoutScale, sourceCellWidth * 0.90 / sourceUnit));

    const speedText = elements.speed?.textContent || "--";
    const cruiseText = elements.setSpeed?.textContent || "--";
    const speedColWidth = contentWidth(elements.speedCell, width * 0.70);
    const cruiseColWidth = contentWidth(elements.setSpeedCell, width * 0.30);
    const baseSpeedUnit = Math.max(1, measure("72", 100) / 100);
    const baseCruiseUnit = Math.max(1, measure("80", 100) / 100);
    const speedSize = Math.max(1, Math.min(220, speedColWidth / baseSpeedUnit));
    const cruiseSize = Math.max(1, Math.min(104, cruiseColWidth / baseCruiseUnit));
    const speedFamily = getComputedStyle(elements.speed).fontFamily;
    const cruiseFamily = getComputedStyle(elements.setSpeed).fontFamily;
    const speedRendered = Math.max(1, measure(speedText, speedSize, speedFamily));
    const cruiseRendered = Math.max(1, measure(cruiseText, cruiseSize, cruiseFamily));
    const speedScaleX = speedText.length > 2 ? Math.min(1, speedColWidth * 0.98 / speedRendered) : 1;
    const cruiseScaleX = cruiseText.length > 2 ? Math.min(1, cruiseColWidth * 0.98 / cruiseRendered) : 1;

    // Limit / detail / stock fonts are measured from their own zone rects, exactly
    // like the preview's syncHudScale (not the whole main region).
    const limitRect = elements.limitZone?.getBoundingClientRect?.();
    const limitAreaWidth = limitRect?.width || width * 0.52;
    const limitAreaHeight = limitRect?.height || width * 0.55;
    const limitSize = Math.max(42, Math.min(220, limitAreaWidth * 0.90, limitAreaHeight * 0.86));
    const limitInnerWidth = Math.max(1, limitSize * 0.66);
    const limitNumUnit = Math.max(1, measure(elements.limit?.textContent || "110", 100) / 100);
    const limitLabelUnit = Math.max(1, measure(elements.limitLabel?.textContent || "", 100) / 100);
    const limitBadgeUnit = Math.max(1, measure(elements.limitBadge?.textContent || "", 100) / 100);
    const limitFont = Math.max(14, Math.min(72, limitSize * 0.40, limitInnerWidth / limitNumUnit));
    const limitLabelFont = Math.max(7, Math.min(26, limitSize * 0.142, limitInnerWidth / limitLabelUnit));
    const badgeFont = Math.max(10, Math.min(34, limitSize * 0.19, (limitSize * 0.80) / limitBadgeUnit));

    const detailRect = elements.alertZone?.getBoundingClientRect?.();
    const detailWidth = detailRect?.width || width * 0.48;
    const detailHeight = detailRect?.height || width * 0.45;
    const detailUnit = Math.max(
      measure(elements.countdown?.textContent || "", 100) / 100,
      measure(elements.distance?.textContent || "", 100) / 100,
      measure(elements.alert?.textContent || "", 100) / 100,
      1,
    );
    const detailFont = Math.max(1, Math.min(46, detailWidth * 0.88 / detailUnit, detailHeight * 0.165));

    const stockRect = elements.stockZone?.getBoundingClientRect?.();
    const stockWidth = stockRect?.width || width;
    const stockHeight = stockRect?.height || width * 0.60;
    const modeUnit = Math.max(1, measure("NORMAL", 100, getComputedStyle(elements.driveMode).fontFamily) / 100);
    const modeFrameWidth = stockWidth * 0.48 * 0.92;
    const modeFont = Math.max(1, Math.min(80, modeFrameWidth / (modeUnit + 0.36), stockHeight * 0.28));
    const gearUnit = Math.max(1, measure(elements.gear?.textContent || "4", 100) / 100);
    const gearDriveUnit = Math.max(0, measure("D", 100) * 0.34 / 100);
    const gearWidth = Math.max(1, stockWidth * 0.52 * 0.88 - 4 * layoutScale);
    const gearFont = Math.max(1, Math.min(180, gearWidth / (gearUnit + gearDriveUnit), stockHeight * 0.50));

    const style = root.style;
    style.setProperty("--mini-ui-scale", layoutScale.toFixed(4));
    style.setProperty("--mini-top-font", `${Math.min(tempFont, sourceFont).toFixed(1)}px`);
    style.setProperty("--mini-speed-font", `${speedSize.toFixed(1)}px`);
    style.setProperty("--mini-set-font", `${cruiseSize.toFixed(1)}px`);
    style.setProperty("--mini-speed-scale-x", speedScaleX.toFixed(4));
    style.setProperty("--mini-set-scale-x", cruiseScaleX.toFixed(4));
    style.setProperty("--mini-limit-size", `${limitSize.toFixed(1)}px`);
    style.setProperty("--mini-limit-font", `${limitFont.toFixed(1)}px`);
    style.setProperty("--mini-limit-label-font", `${limitLabelFont.toFixed(1)}px`);
    style.setProperty("--mini-badge-font", `${badgeFont.toFixed(1)}px`);
    style.setProperty("--mini-alert-font", `${detailFont.toFixed(1)}px`);
    style.setProperty("--mini-mode-font", `${modeFont.toFixed(1)}px`);
    style.setProperty("--mini-gear-font", `${gearFont.toFixed(1)}px`);
  }

  function scheduleLayout() {
    if (layoutRaf) return;
    layoutRaf = requestAnimationFrame(applyLayout);
  }

  function init() {
    window.CarrotMiniHudMode?.bind?.();
    const onResize = () => scheduleLayout();
    window.addEventListener("resize", onResize, { passive: true });
    window.addEventListener("orientationchange", onResize, { passive: true });
    if (window.visualViewport) {
      window.visualViewport.addEventListener("resize", onResize, { passive: true });
    }
    window.addEventListener("carrot:minihudchange", (event) => {
      if (event?.detail?.active) {
        if (latestModel) render(latestModel);
        scheduleLayout();
      }
    });
    if (typeof ResizeObserver === "function") {
      resizeObserver = new ResizeObserver(scheduleLayout);
      resizeObserver.observe(root);
    }
  }

  window.CarrotMiniHud = { init, render, update: render, relayout: scheduleLayout };
  init();
})();
