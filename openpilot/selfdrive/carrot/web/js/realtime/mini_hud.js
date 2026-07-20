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
    alertBadge: root.querySelector("[data-mini-hud-alert-badge]"),
    stockMode: root.querySelector("[data-mini-hud-stock-mode]"),
    driveMode: root.querySelector("[data-mini-hud-drive-mode]"),
    driveModeFrame: root.querySelector(".carrot-mini-hud__drive-mode"),
    alertZone: root.querySelector("[data-mini-hud-alert-zone]"),
    countdownZone: root.querySelector("[data-mini-hud-countdown-zone]"),
    countdownLabel: root.querySelector("[data-mini-hud-countdown-zone] .carrot-mini-hud__chip-label"),
    countdown: root.querySelector("[data-mini-hud-countdown]"),
    distanceZone: root.querySelector("[data-mini-hud-distance-zone]"),
    distanceLabel: root.querySelector("[data-mini-hud-distance-zone] .carrot-mini-hud__chip-label"),
    distance: root.querySelector("[data-mini-hud-distance]"),
    gapZone: root.querySelector("[data-mini-hud-gap-zone]"),
    gapLabel: root.querySelector("[data-mini-hud-gap-zone] .carrot-mini-hud__chip-label"),
    gap: root.querySelector("[data-mini-hud-gap]"),
    gapSignal: root.querySelector("[data-mini-hud-gap-signal]"),
    tempZone: root.querySelector("[data-mini-hud-temp-zone]"),
    tempLabel: root.querySelector("[data-mini-hud-temp-label]"),
    tempSpeed: root.querySelector("[data-mini-hud-temp-speed]"),
    gearBadge: root.querySelector("[data-mini-hud-gear-badge]"),
    gearLabel: root.querySelector("[data-mini-hud-gear-label]"),
    gear: root.querySelector("[data-mini-hud-gear]"),
    speed: root.querySelector("[data-mini-hud-speed]"),
    setSpeed: root.querySelector("[data-mini-hud-set-speed]"),
    speedCell: root.querySelector(".carrot-mini-hud__speed"),
    setSpeedCell: root.querySelector(".carrot-mini-hud__set-speed"),
    main: root.querySelector(".carrot-mini-hud__main"),
  };

  const canvas = document.createElement("canvas");
  const context = canvas.getContext("2d");
  const TEMPERATURE_UNIT_KEY = "carrot.miniHud.temperatureUnit.v1";
  const LAYOUT_RETRY_MS = 250;
  let latestModel = null;
  let layoutRaf = 0;
  let layoutRetryTimer = 0;
  let resizeObserver = null;
  let temperatureUnit = loadTemperatureUnit();
  // Detail-row labels (TIM / DST / GAP / temp source) are ALWAYS shown at full
  // length. The label chip already reserves a fixed width (--runtime-mini-alert-label-width
  // = detailFont * DETAIL_LABEL_UNIT), so a 3-char label costs no extra room versus
  // a 1-char one — the earlier "collapse to one letter on narrow layouts" behavior
  // only left the reserved chip half-empty, so it was dropped.
  const DETAIL_LABEL_LENGTH = 3;

  function loadTemperatureUnit() {
    try {
      return localStorage.getItem(TEMPERATURE_UNIT_KEY) === "f" ? "f" : "c";
    } catch (_) {
      return "c";
    }
  }

  function saveTemperatureUnit() {
    try {
      localStorage.setItem(TEMPERATURE_UNIT_KEY, temperatureUnit);
    } catch (_) {
      // Storage can be unavailable in private or restricted browser contexts.
    }
  }

  function cpuTemperatureText(celsius) {
    if (celsius == null || celsius === "") return `CPU:--°${temperatureUnit.toUpperCase()}`;
    const value = Number(celsius);
    if (!Number.isFinite(value)) return `CPU:--°${temperatureUnit.toUpperCase()}`;
    const display = temperatureUnit === "f" ? (value * 9 / 5) + 32 : value;
    return `CPU:${Math.round(display)}°${temperatureUnit.toUpperCase()}`;
  }

  function syncCpuTemperature() {
    setText(elements.cpu, cpuTemperatureText(latestModel?.cpu));
    if (!elements.cpu) return;
    const current = temperatureUnit === "f" ? "Fahrenheit" : "Celsius";
    const next = temperatureUnit === "f" ? "Celsius" : "Fahrenheit";
    elements.cpu.setAttribute("aria-label", `CPU temperature in ${current}. Activate to use ${next}.`);
  }

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

  function setRowEmpty(element, empty) {
    if (element) element.dataset.miniHudEmpty = empty ? "1" : "0";
  }

  function shortLabel(value, fallback = "", maxLength = 5) {
    const label = String(value || fallback || "").trim().toUpperCase();
    return label.slice(0, Math.max(1, maxLength));
  }

  function currentLang() {
    // Cross-IIFE safe: i18n.js writes the active code onto <html lang>. hud_card's
    // own currentLang() is module-private, so we read the DOM instead of it.
    const raw = String(document.documentElement.lang || "en").toLowerCase();
    return raw.split(/[-_]/)[0] || "en";
  }

  function uiText(key, fallback) {
    const table = window.CarrotTranslations?.strings?.[currentLang()] || {};
    const value = table[key];
    return typeof value === "string" && value ? value : fallback;
  }

  function localizedDriveMode(model) {
    const kind = String(model?.driveMode?.kind || "normal").toLowerCase();
    // driveModes is published globally by translations/registry.js
    // (normal/eco/safe/sport → 일반/연비/안전/고속 등). Mirror hud_card's big-HUD
    // label so the compact HUD is localized the same way instead of raw English.
    const table = window.CarrotTranslations?.driveModes || {};
    const lang = currentLang();
    const labels = table[lang] || table.en || {};
    return labels[kind] || model?.driveMode?.name || (kind ? kind.toUpperCase() : "NORMAL");
  }

  function syncDetailLabels(model, maxLength = DETAIL_LABEL_LENGTH) {
    setText(elements.countdownLabel, "TIM");
    setText(elements.distanceLabel, "DST");
    setText(elements.gapLabel, "GAP");
    const tempSource = model?.source === "stock" ? "TMP" : model?.temp?.label;
    setText(elements.tempLabel, shortLabel(
      tempSource,
      model?.alert?.name || model?.source || "SRC",
      maxLength,
    ));
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

  function contentHeight(element, fallback) {
    if (!element) return fallback;
    const rect = element.getBoundingClientRect();
    const style = getComputedStyle(element);
    const padding = (parseFloat(style.paddingTop) || 0) + (parseFloat(style.paddingBottom) || 0);
    const inner = rect.height - padding;
    // Same degenerate-frame guard as contentWidth (see above).
    return inner > 4 ? inner : fallback;
  }

  function render(model) {
    if (!model) return;
    latestModel = model;

    const stock = model.source === "stock";
    const limitVisible = !stock && model.roadLimit && model.roadLimit !== "--";
    const alertVisible = !stock && Boolean(model.alert?.visible);

    root.dataset.source = model.source;
    root.dataset.limitStyle = model.limitStyle || "kr";
    root.dataset.alertKind = model.alert?.kind || "none";
    root.classList.toggle("is-three-digit", String(model.speed).length >= 3 || String(model.setSpeed).length >= 3);

    syncCpuTemperature();
    setText(elements.source, model.source.toUpperCase());
    setText(elements.speed, model.speed);
    setText(elements.setSpeed, model.setSpeed);

    setHidden(elements.limitZone, !limitVisible);
    setText(elements.limit, model.roadLimit);
    setText(elements.limitLabel, model.alert?.section ? uiText("mini_hud_section_limit", "SECTION") : "");
    setHidden(elements.limitLabel, !model.alert?.section);
    setText(elements.limitBadge, model.alert?.badge || "");
    setHidden(elements.limitBadge, !model.alert?.badge);

    const alertBadgeKind = limitVisible && alertVisible && ["camera", "police"].includes(model.alert?.kind)
      ? model.alert.kind
      : "";
    if (elements.alertBadge && alertBadgeKind) {
      const source = `/assets/alert_${alertBadgeKind}.svg`;
      if (elements.alertBadge.getAttribute("src") !== source) elements.alertBadge.setAttribute("src", source);
      elements.alertBadge.dataset.kind = alertBadgeKind;
    }
    setHidden(elements.alertBadge, !alertBadgeKind);

    setHidden(elements.stockMode, !stock);
    setText(elements.driveMode, localizedDriveMode(model));
    if (elements.driveModeFrame) elements.driveModeFrame.dataset.miniHudDriveKind = model.driveMode?.kind || "normal";

    const tempVisible = !stock && Boolean(model.temp?.visible);
    setHidden(elements.alertZone, false);
    syncDetailLabels(model);
    setText(elements.countdown, model.alert?.countdown || "--");
    setHidden(elements.countdownZone, false);
    setRowEmpty(elements.countdownZone, !model.alert?.countdown);
    setText(elements.distance, model.alert?.distance || "--");
    setHidden(elements.distanceZone, false);
    setRowEmpty(elements.distanceZone, !model.alert?.distance);
    setText(elements.gap, model.gap || "--");
    if (elements.gapSignal) elements.gapSignal.dataset.level = String(clamp(Number(model.gap) || 0, 0, 4));
    setHidden(elements.gapZone, false);
    setRowEmpty(elements.gapZone, !model.gap);
    setText(elements.tempSpeed, tempVisible ? model.temp?.speed : "--");
    setHidden(elements.tempZone, false);
    setRowEmpty(elements.tempZone, !tempVisible);
    if (elements.tempZone) elements.tempZone.dataset.miniHudTempDecel = model.temp?.decel ? "1" : "0";

    const gearStep = model.gearStep == null ? "" : String(model.gearStep);
    const gearLabel = String(model.gear || (gearStep ? "D" : "")).trim().toUpperCase();
    const gearKnown = Boolean(gearLabel || gearStep);
    setText(elements.gearLabel, gearLabel);
    setHidden(elements.gearLabel, !gearLabel);
    setText(elements.gear, gearStep || (gearKnown ? "" : "--"));
    setHidden(elements.gear, gearKnown && !gearStep);
    setHidden(elements.gearBadge, false);
    if (elements.gearBadge) elements.gearBadge.dataset.miniHudGearKnown = gearKnown ? "1" : "0";

    scheduleLayout();
  }

  function applyLayout() {
    layoutRaf = 0;
    if (!window.CarrotMiniHudMode?.isActive?.() || root.hidden) return;

    const rect = root.getBoundingClientRect();
    // Degenerate frame during activation — skip so we never bake a 1px font from
    // a not-yet-laid-out root; a later resize/render tick re-runs the layout.
    if (rect.width < 40 || rect.height < 40) {
      scheduleLayoutRetry();
      return;
    }
    const width = Math.max(1, rect.width);
    // Scale the whole compact surface from both axes. Short multi-window layouts
    // must shrink the same tokens as narrow layouts instead of overflowing.
    const layoutScale = clamp(Math.min(width / 404, rect.height / 650), 0.46, 1);
    const style = root.style;
    style.setProperty("--runtime-mini-ui-scale", layoutScale.toFixed(4));
    style.setProperty("--runtime-mini-detail-height", `${(220 * layoutScale).toFixed(1)}px`);

    const topCells = root.querySelectorAll(".carrot-mini-hud__top-cell");
    const tempCellWidth = contentWidth(topCells[0], width * 0.50);
    const sourceCellWidth = contentWidth(topCells[1], width * 0.50);
    const tempText = elements.cpu?.textContent || "CPU:--°C";
    const sourceText = elements.source?.textContent || "STOCK";
    const tempUnit = Math.max(1, measure(tempText, 100) / 100);
    const sourceUnit = Math.max(1, measure(sourceText, 100) / 100);
    const tempFont = Math.max(1, Math.min(32 * layoutScale, tempCellWidth * 0.90 / tempUnit));
    const sourceFont = Math.max(1, Math.min(32 * layoutScale, sourceCellWidth * 0.90 / sourceUnit));

    const speedText = elements.speed?.textContent || "--";
    const cruiseText = elements.setSpeed?.textContent || "--";
    const speedRowWidth = contentWidth(elements.speedCell?.parentElement, width);
    const speedGap = clamp(width * 0.030, 8 * layoutScale, 18 * layoutScale);
    const speedGroupWidth = Math.max(1, Math.min(speedRowWidth * 0.96, width * 0.94));
    const speedGroupLeft = Math.max(0, (speedRowWidth - speedGroupWidth) * 0.5);
    // Also bound by the speed-ROW height so digits don't overflow (and clip
    // against the fixed-inset root) on short viewports. Measure the row, not the
    // cell: the cell is `align-items:end` grid content whose height IS the glyph
    // line-box, so feeding it back into the font size ping-pongs the size. The
    // row is a fixed 34% flex-basis and stays stable regardless of font.
    const speedRowHeight = contentHeight(elements.speedCell?.parentElement, rect.height * 0.30);
    const speedFamily = getComputedStyle(elements.speed).fontFamily;
    const cruiseFamily = getComputedStyle(elements.setSpeed).fontFamily;
    const baseSpeedSlotText = "88";
    const baseCruiseSlotText = "88";
    const baseSpeedUnit = Math.max(1, measure(baseSpeedSlotText, 100, speedFamily) / 100);
    const baseCruiseUnit = Math.max(1, measure(baseCruiseSlotText, 100, cruiseFamily) / 100);
    const stableCruiseSize = Math.max(1, Math.min(
      124,
      ((speedGroupWidth - speedGap) * 0.32) / baseCruiseUnit,
      speedRowHeight * 0.66,
    ));
    const gearBottom = stableCruiseSize * 0.92;
    const gearFont = stableCruiseSize * 0.30;
    const gearHeight = stableCruiseSize * 0.52;
    const cruiseShare = 0.32;
    const speedShare = 1 - cruiseShare;
    const speedMaxWidth = Math.max(1, (speedGroupWidth - speedGap) * speedShare);
    const cruiseMaxWidth = Math.max(1, (speedGroupWidth - speedGap) * cruiseShare);
    const gearLeft = speedGroupLeft + speedMaxWidth + speedGap;
    const speedSize = Math.max(1, Math.min(240, speedMaxWidth / baseSpeedUnit, speedRowHeight * 1.10));
    const cruiseSize = Math.max(1, Math.min(124, cruiseMaxWidth / baseCruiseUnit, speedRowHeight * 0.66));
    const speedRendered = Math.max(1, measure(speedText, speedSize, speedFamily));
    const cruiseRendered = Math.max(1, measure(cruiseText, cruiseSize, cruiseFamily));
    const speedColWidth = speedMaxWidth;
    const cruiseColWidth = cruiseMaxWidth;
    const speedScaleX = speedText.length > 2 ? Math.min(1, speedMaxWidth * 0.98 / speedRendered) : 1;
    const cruiseScaleX = cruiseText.length > 2 ? Math.min(1, cruiseMaxWidth * 0.98 / cruiseRendered) : 1;

    // Limit / detail / stock fonts are measured from their own zone rects, exactly
    // like the preview's syncHudScale (not the whole main region).
    const limitRect = elements.limitZone?.getBoundingClientRect?.();
    const mainRect = elements.main?.getBoundingClientRect?.();
    const limitAreaWidth = limitRect?.width || width * 0.52;
    const limitAreaHeight = limitRect?.height || mainRect?.height || width * 0.55;
    const limitSize = Math.max(46, Math.min(250, limitAreaWidth * 0.98, limitAreaHeight * 0.94));
    const limitInnerWidth = Math.max(1, limitSize * 0.66);
    const limitNumUnit = Math.max(1, measure(elements.limit?.textContent || "110", 100) / 100);
    const limitLabelUnit = Math.max(1, measure(elements.limitLabel?.textContent || "", 100) / 100);
    const limitBadgeUnit = Math.max(1, measure(elements.limitBadge?.textContent || "", 100) / 100);
    const limitFont = Math.max(14, Math.min(78, limitSize * 0.40, limitInnerWidth / limitNumUnit));
    const limitLabelFont = Math.max(7, Math.min(28, limitSize * 0.142, limitInnerWidth / limitLabelUnit));
    const badgeFont = Math.max(10, Math.min(38, limitSize * 0.19, (limitSize * 0.80) / limitBadgeUnit));

    // US MUTCD rectangle: the number fills a wider/taller area than the circle,
    // with a stacked SPEED/LIMIT caption above it (sized off the same limitSize).
    const isUsLimit = root.dataset.limitStyle === "us";
    const usInnerWidth = Math.max(1, limitSize * 0.82 * 0.82);
    const usCaptionUnit = Math.max(1, measure("LIMIT", 100) / 100);
    const limitNumberFont = isUsLimit
      ? Math.max(16, Math.min(108, limitSize * 0.52, usInnerWidth / limitNumUnit))
      : limitFont;
    const limitCaptionFont = Math.max(7, Math.min(34, limitSize * 0.145, usInnerWidth / usCaptionUnit));

    const detailRect = elements.alertZone?.getBoundingClientRect?.();
    const detailWidth = contentWidth(elements.alertZone, width * 0.48);
    const detailHeight = contentHeight(elements.alertZone, detailRect?.height || width * 0.45);
    syncDetailLabels(latestModel);
    // Keep the detail area as fixed rows; empty rows stay in the scale budget.
    const chipCount = 4;
    const detailLabelUnit = 2.24;
    const detailValueUnit = 3.40;
    const detailLineUnit = detailValueUnit + detailLabelUnit + 0.72;
    const detailFontH = (detailHeight / chipCount) * 0.76;
    const detailFontW = (detailWidth * 0.90) / detailLineUnit;
    const detailFont = Math.max(16 * layoutScale, Math.min(78, detailFontW, detailFontH));

    const modeFamily = getComputedStyle(elements.driveMode).fontFamily;
    const modeUnit = Math.max(1, measure("NORMAL", 100, modeFamily) / 100);
    const modeSize = limitSize;
    const modeFont = Math.max(1, Math.min(72, modeSize * 0.80 / modeUnit, modeSize * 0.25));

    style.setProperty("--runtime-mini-top-font", `${Math.min(tempFont, sourceFont).toFixed(1)}px`);
    style.setProperty("--runtime-mini-speed-slot-width", `${speedColWidth.toFixed(1)}px`);
    style.setProperty("--runtime-mini-set-slot-width", `${cruiseColWidth.toFixed(1)}px`);
    style.setProperty("--runtime-mini-speed-gap", `${speedGap.toFixed(1)}px`);
    style.setProperty("--runtime-mini-gear-left", `${gearLeft.toFixed(1)}px`);
    style.setProperty("--runtime-mini-gear-bottom", `${gearBottom.toFixed(1)}px`);
    style.setProperty("--runtime-mini-gear-font", `${gearFont.toFixed(1)}px`);
    style.setProperty("--runtime-mini-gear-height", `${gearHeight.toFixed(1)}px`);
    style.setProperty("--runtime-mini-speed-font", `${speedSize.toFixed(1)}px`);
    style.setProperty("--runtime-mini-set-font", `${cruiseSize.toFixed(1)}px`);
    style.setProperty("--runtime-mini-speed-scale-x", speedScaleX.toFixed(4));
    style.setProperty("--runtime-mini-set-scale-x", cruiseScaleX.toFixed(4));
    style.setProperty("--runtime-mini-limit-size", `${limitSize.toFixed(1)}px`);
    style.setProperty("--runtime-mini-limit-font", `${limitNumberFont.toFixed(1)}px`);
    style.setProperty("--runtime-mini-limit-caption-font", `${limitCaptionFont.toFixed(1)}px`);
    style.setProperty("--runtime-mini-limit-label-font", `${limitLabelFont.toFixed(1)}px`);
    style.setProperty("--runtime-mini-badge-font", `${badgeFont.toFixed(1)}px`);
    style.setProperty("--runtime-mini-alert-font", `${detailFont.toFixed(1)}px`);
    // Auto-fit the detail label instead of a small fixed ratio. The chip reserves
    // detailLabelUnit(2.24)x the value width, but a 3-char label only needs ~2
    // units, so we can size the label up until it fills the chip — capped at 0.78x
    // the value so it stays clearly a label yet reads as a HUD on the smallest
    // windows (the earlier fixed 0.54x was the "still too small" complaint).
    const labelChipInner = detailFont * detailLabelUnit * 0.88;
    const labelUnit = Math.max(1, measure("GAP", 100) / 100);
    const labelFont = Math.max(1, Math.min(detailFont * 0.78, labelChipInner / labelUnit));
    style.setProperty("--runtime-mini-alert-label-font", `${labelFont.toFixed(1)}px`);
    style.setProperty("--runtime-mini-alert-label-width", `${(detailFont * detailLabelUnit).toFixed(1)}px`);
    style.setProperty("--runtime-mini-mode-size", `${modeSize.toFixed(1)}px`);
    style.setProperty("--runtime-mini-mode-font", `${modeFont.toFixed(1)}px`);
  }

  function scheduleLayout() {
    if (layoutRaf) return;
    layoutRaf = requestAnimationFrame(applyLayout);
  }

  // A degenerate root rect can persist (viewport narrower than the 40px probe,
  // or a frame that never resolves). Re-arming on rAF then spins a core at
  // 60fps forever with nothing to measure, so back off to a slow poll instead.
  function scheduleLayoutRetry() {
    if (layoutRaf || layoutRetryTimer) return;
    layoutRetryTimer = setTimeout(() => {
      layoutRetryTimer = 0;
      scheduleLayout();
    }, LAYOUT_RETRY_MS);
  }

  function init() {
    window.CarrotMiniHudMode?.bind?.();
    elements.cpu?.addEventListener("click", (event) => {
      event.stopPropagation();
      temperatureUnit = temperatureUnit === "c" ? "f" : "c";
      saveTemperatureUnit();
      syncCpuTemperature();
      scheduleLayout();
    });
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
    // Re-localize the drive-mode label when the user switches language.
    window.addEventListener("carrot:languagechange", () => {
      if (latestModel) render(latestModel);
    });
    if (typeof ResizeObserver === "function") {
      resizeObserver = new ResizeObserver(scheduleLayout);
      resizeObserver.observe(root);
    }
  }

  window.CarrotMiniHud = { init, render, update: render, relayout: scheduleLayout };
  init();
})();
