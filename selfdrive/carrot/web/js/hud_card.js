/* Adaptive driving HUD
 * - Reuses CarrotLink homePreview surface as the primary web HUD layout.
 * - Mount position (inline vs overlay) is handled separately from the visual surface.
 */
(function () {
  function $(id) { return document.getElementById(id); }
  function clamp(v, min, max) { return Math.min(max, Math.max(min, v)); }
  function setText(id, v) {
    const el = $(id);
    if (!el) return;
    const s = v == null ? "" : String(v);
    if (el.textContent !== s) el.textContent = s;
  }

  const SURFACE_HOME = "homePreview";
  const SURFACE_INLINE = "driveInline";
  const SURFACE_OVERLAY = "driveOverlay";
  const STRONG_TEXT_SHADOW = "0 1.4px 3.6px rgba(0,0,0,0.94), 0 0 1.2px rgba(0,0,0,0.62)";
  const DRIVE_MODE_TEXT = {
    en: {
      normal: "Normal",
      eco: "Eco",
      safe: "Safe",
      sport: "Fast",
      fast: "Fast",
    },
    ko: {
      normal: "일반",
      eco: "에코",
      safe: "안전",
      sport: "고속",
      fast: "고속",
    },
    zh: {
      normal: "普通",
      eco: "经济",
      safe: "安全",
      sport: "高速",
      fast: "高速",
    },
    ja: {
      normal: "通常",
      eco: "エコ",
      safe: "安全",
      sport: "高速",
      fast: "高速",
    },
    fr: {
      normal: "Normal",
      eco: "Eco",
      safe: "Securite",
      sport: "Rapide",
      fast: "Rapide",
    },
  };
  const HUD_LABELS = {
    en: { speed: "Speed", setSpeed: "Set Speed", temp: "TEMP", gear: "GEAR", limit: "LIMIT" },
    ko: { speed: "현재속도", setSpeed: "설정속도", temp: "TEMP", gear: "GEAR", limit: "LIMIT" },
    zh: { speed: "当前速度", setSpeed: "设定速度", temp: "TEMP", gear: "GEAR", limit: "LIMIT" },
    ja: { speed: "速度", setSpeed: "設定速度", temp: "TEMP", gear: "GEAR", limit: "LIMIT" },
    fr: { speed: "Vitesse", setSpeed: "Vitesse reglee", temp: "TEMP", gear: "GEAR", limit: "LIMIT" },
  };
  const HUD_AUX_ROTATE_MS = 1600;
  const HUD_AUX_ICON_PATHS = {
    disk: "M4 6a2 2 0 0 1 2-2h12a2 2 0 0 1 2 2v12a2 2 0 0 1-2 2H6a2 2 0 0 1-2-2zm2 0v12h12V6zm2 2h8v2H8zm0 4h5v2H8zm7 3.5a1 1 0 1 0 0 2 1 1 0 0 0 0-2",
    volt: "M11 21h-1l1-7H8.5a.5.5 0 0 1-.39-.81L13 3h1l-1 7h2.5a.5.5 0 0 1 .39.81z",
  };

  let hudLayoutBound = false;
  let hudLayoutRaf = 0;
  let lastHudLabelLang = "";
  let lastHudBarsCount = -1;
  let lastHudTempSignature = "";
  let hudAuxMetricTimer = 0;
  let hudAuxMetricKind = "disk";
  let lastStableGear = "U";
  let lastStableGearStep = null;
  let hudAuxMetricState = {
    diskPct: null,
    voltageV: null,
  };

  function getHudCard() {
    return $("driveHudCard");
  }

  function getHudRoot() {
    return $("hudRoot");
  }

  function getHudAuxValueEl() {
    return $("hudDiskVal");
  }

  function getHudAuxIconPathEl() {
    return $("hudAuxIconPath");
  }

  function isOverlayMount() {
    const page = document.body?.dataset?.page || "carrot";
    return page === "carrot" && window.matchMedia("(orientation: landscape)").matches;
  }

  function getHudSurface() {
    const page = document.body?.dataset?.page || "carrot";
    if (page !== "carrot") return SURFACE_HOME;
    return window.matchMedia("(orientation: landscape)").matches ? SURFACE_OVERLAY : SURFACE_INLINE;
  }

  function getPreferredAspectRatioForWindow(windowClass, wide) {
    if (wide) {
      switch (windowClass) {
        case "compact": return 1.24;
        case "medium": return 1.30;
        case "expanded": return 1.36;
        case "large": return 1.42;
        default: return 1.46;
      }
    }

    switch (windowClass) {
      case "compact": return 0.92;
      case "medium": return 0.96;
      case "expanded": return 1.00;
      case "large": return 1.04;
      default: return 1.06;
    }
  }

  function getHudWindowClass(width) {
    if (width < 600) return "compact";
    if (width < 840) return "medium";
    if (width < 1200) return "expanded";
    if (width < 1600) return "large";
    return "extraLarge";
  }

  function currentLang() {
    return typeof LANG === "string" && LANG ? LANG : "en";
  }

  function getHudLabels() {
    return HUD_LABELS[currentLang()] || HUD_LABELS.en;
  }

  function getBandHeight(density) {
    switch (density) {
      case "micro": return 34;
      case "compact": return 40;
      case "regular": return 44;
      default: return 48;
    }
  }

  function getBandTypography(profile, surface) {
    let metricScale;
    let bottomScale;

    switch (profile.density) {
      case "micro":
        metricScale = 1.84;
        bottomScale = 1.34;
        break;
      case "compact":
        metricScale = 1.78;
        bottomScale = 1.30;
        break;
      case "regular":
        metricScale = 1.72;
        bottomScale = 1.26;
        break;
      default:
        metricScale = 1.66;
        bottomScale = 1.22;
        break;
    }

    if (surface === SURFACE_OVERLAY) {
      metricScale = Math.max(1.42, (metricScale - 0.02) * 0.90);
      bottomScale = Math.max(1.02, (bottomScale - 0.02) * 0.90);
    }

    return {
      metricScale,
      bottomScale,
    };
  }

  function getSurfaceTarget() {
    if (isOverlayMount()) return $("carrotStage");
    return $("carrotHudDock");
  }

  function mountHudToSurface(surface) {
    const card = getHudCard();
    const target = getSurfaceTarget();
    if (!card || !target) return;

    if (card.parentElement !== target) {
      target.appendChild(card);
    }

    card.classList.remove("driveHudCard--inline", "driveHudCard--overlay");
    card.classList.add(isOverlayMount() ? "driveHudCard--overlay" : "driveHudCard--inline");
  }

  function getHudConstraints(surface) {
    const vv = window.visualViewport;
    const viewportWidth = Math.max(320, Math.round(vv?.width || window.innerWidth || 0));
    const viewportHeight = Math.max(320, Math.round(vv?.height || window.innerHeight || 0));
    const isPortrait = viewportHeight > viewportWidth;
    const overlayMount = isOverlayMount();
    const target = getSurfaceTarget();
    const rect = target?.getBoundingClientRect?.() || null;
    const width = rect?.width || viewportWidth;
    const height = rect?.height || viewportHeight;
    const windowClass = getHudWindowClass(viewportWidth);

    if (surface === SURFACE_OVERLAY) {
      const drawWidth = width;
      const drawHeight = height;
      const base = Math.min(drawWidth, drawHeight);
      const overlaySizeBoost = 1.10;
      const ratio = (() => {
        switch (windowClass) {
          case "compact": return 0.23;
          case "medium": return 0.225;
          case "expanded": return 0.22;
          case "large": return 0.215;
          default: return 0.21;
        }
      })();
      const minSize = (() => {
        switch (windowClass) {
          case "compact": return 150;
          case "medium": return 162;
          case "expanded": return 174;
          case "large": return 186;
          default: return 198;
        }
      })() * overlaySizeBoost;
      const maxSize = (() => {
        switch (windowClass) {
          case "compact": return 246;
          case "medium": return 262;
          case "expanded": return 278;
          case "large": return 294;
          default: return 310;
        }
      })() * overlaySizeBoost;
      const viewportCap = drawHeight * 0.374;
      const upperBound = Math.max(minSize, Math.min(maxSize, viewportCap));
      const overlayHeight = clamp(base * ratio * overlaySizeBoost, minSize, upperBound);
      const overlayWidth = Math.min(
        Math.max(320, drawWidth - 32),
        overlayHeight * getPreferredAspectRatioForWindow(windowClass, true),
      );
      return {
        width: overlayWidth,
        height: overlayHeight,
      };
    }

    if (surface === SURFACE_INLINE) {
      const horizontalInset = (() => {
        switch (windowClass) {
          case "compact": return 12;
          case "medium": return 14;
          case "expanded": return 16;
          default: return 18;
        }
      })();
      const usableWidth = Math.max(1, viewportWidth - (horizontalInset * 2));
      const desiredHeight = usableWidth / getPreferredAspectRatioForWindow(windowClass, true);
      const minHeight = (() => {
        switch (windowClass) {
          case "compact": return 190;
          case "medium": return 204;
          case "expanded": return 216;
          case "large": return 228;
          default: return 236;
        }
      })();
      const maxHeight = (() => {
        switch (windowClass) {
          case "compact": return Math.min(430, viewportHeight * 0.42);
          case "medium": return Math.min(450, viewportHeight * 0.41);
          case "expanded": return Math.min(470, viewportHeight * 0.40);
          case "large": return Math.min(490, viewportHeight * 0.39);
          default: return Math.min(510, viewportHeight * 0.38);
        }
      })();
      return {
        width: viewportWidth,
        height: clamp(desiredHeight, minHeight, maxHeight),
      };
    }

    if (surface === SURFACE_HOME) {
      if (overlayMount) {
        return {
          width: clamp(width - 36, 560, 980),
          height: clamp(height - 36, 320, 620),
        };
      }
      const viewportAwareHudCap = isPortrait
        ? clamp(viewportHeight * 0.46, 280, 420)
        : clamp(viewportHeight * 0.32, 220, 520);
      const homeHudHeightCap = (() => {
        switch (windowClass) {
          case "compact": return clamp(viewportAwareHudCap, isPortrait ? 280 : 220, isPortrait ? 380 : 360);
          case "medium": return clamp(viewportAwareHudCap, isPortrait ? 300 : 240, isPortrait ? 400 : 390);
          case "expanded": return clamp(viewportAwareHudCap, isPortrait ? 320 : 260, isPortrait ? 420 : 420);
          case "large": return clamp(viewportAwareHudCap, 280, 450);
          default: return clamp(viewportAwareHudCap, 300, 480);
        }
      })();
      return {
        width: Math.min(Math.max(isPortrait ? 320 : 280, width), viewportWidth - (isPortrait ? 8 : 24)),
        height: homeHudHeightCap,
      };
    }

    return {
      width: clamp((rect?.width || viewportWidth) * 0.42, 320, 980),
      height: clamp((rect?.height || viewportHeight) * 0.36, 210, 360),
    };
  }

  function buildHudProfile(width, height, surface) {
    const shortest = Math.min(width, height);
    const aspect = width / Math.max(height, 1);
    const heightBudget = surface === SURFACE_OVERLAY
      ? height * 0.76
      : surface === SURFACE_INLINE
        ? height * 0.84
        : height;
    const heightWeightedShortest = Math.min(shortest, heightBudget);
    const density = heightWeightedShortest < 190
      ? "micro"
      : heightWeightedShortest < 280
        ? "compact"
        : heightWeightedShortest < 420
          ? "regular"
          : "spacious";
    const wideThreshold = surface === SURFACE_OVERLAY
      ? 1.12
      : surface === SURFACE_INLINE
        ? 1.18
        : 1.26;
    const wide = aspect >= wideThreshold;

    if (density === "micro") {
      return {
        density,
        wide: false,
        preferredAspectRatio: surface === SURFACE_OVERLAY ? 1.02 : surface === SURFACE_INLINE ? 0.98 : 1.24,
        borderRadius: 18,
        dockInset: surface === SURFACE_OVERLAY ? 4 : surface === SURFACE_HOME ? 2 : 6,
        padding: surface === SURFACE_HOME ? 10 : 13,
        sectionGap: surface === SURFACE_HOME ? 8 : 11,
        metricGap: surface === SURFACE_HOME ? 7 : 9,
        speedFontSize: 74,
        primaryValueFontSize: 34,
        secondaryValueFontSize: 24,
        labelFontSize: 13.5,
        chipFontSize: 12.5,
        gearFontSize: 38,
        maxWidth: surface === SURFACE_OVERLAY ? 542 : surface === SURFACE_INLINE ? 404 : 360,
      };
    }

    if (density === "compact") {
      return {
        density,
        wide,
        preferredAspectRatio: surface === SURFACE_OVERLAY
          ? (wide ? 1.02 : 0.94)
          : surface === SURFACE_INLINE
            ? (wide ? 0.96 : 0.80)
            : (wide ? 1.34 : 1.24),
        borderRadius: 20,
        dockInset: surface === SURFACE_OVERLAY ? 6 : surface === SURFACE_HOME ? 3 : 8,
        padding: surface === SURFACE_HOME ? 12 : 16,
        sectionGap: surface === SURFACE_HOME ? 10 : 13,
        metricGap: surface === SURFACE_HOME ? 8 : 10,
        speedFontSize: 90,
        primaryValueFontSize: 38,
        secondaryValueFontSize: 26,
        labelFontSize: 14.5,
        chipFontSize: 13.5,
        gearFontSize: 46,
        maxWidth: surface === SURFACE_OVERLAY ? 648 : surface === SURFACE_INLINE ? 476 : 428,
      };
    }

    if (density === "regular") {
      return {
        density,
        wide,
        preferredAspectRatio: surface === SURFACE_OVERLAY
          ? (wide ? 1.08 : 0.98)
          : surface === SURFACE_INLINE
            ? (wide ? 1.00 : 0.84)
            : (wide ? 1.44 : 1.30),
        borderRadius: 24,
        dockInset: surface === SURFACE_OVERLAY ? 8 : surface === SURFACE_HOME ? 4 : 10,
        padding: surface === SURFACE_HOME ? 14 : 20,
        sectionGap: surface === SURFACE_HOME ? 12 : 15,
        metricGap: surface === SURFACE_HOME ? 9 : 11,
        speedFontSize: 110,
        primaryValueFontSize: 45,
        secondaryValueFontSize: 31,
        labelFontSize: 15.5,
        chipFontSize: 14.5,
        gearFontSize: 56,
        maxWidth: surface === SURFACE_OVERLAY ? 804 : surface === SURFACE_INLINE ? 604 : 536,
      };
    }

    return {
      density: "spacious",
      wide,
      preferredAspectRatio: surface === SURFACE_OVERLAY
        ? (wide ? 1.26 : 1.02)
        : surface === SURFACE_INLINE
          ? (wide ? 1.16 : 0.90)
          : (wide ? 1.56 : 1.38),
      borderRadius: 28,
      dockInset: surface === SURFACE_OVERLAY ? 10 : surface === SURFACE_HOME ? 5 : 12,
      padding: surface === SURFACE_HOME ? 16 : 24,
      sectionGap: surface === SURFACE_HOME ? 14 : 18,
      metricGap: surface === SURFACE_HOME ? 10 : 13,
      speedFontSize: 128,
      primaryValueFontSize: 52,
      secondaryValueFontSize: 36,
      labelFontSize: 16.5,
      chipFontSize: 15.5,
      gearFontSize: 68,
      maxWidth: surface === SURFACE_OVERLAY ? 984 : surface === SURFACE_INLINE ? 734 : 668,
    };
  }

  let _hudProfileSig = "";

  function getHudSupportLayout(surface, profile, panelWidth, panelHeight) {
    if (surface === SURFACE_OVERLAY || profile.density === "micro" || panelWidth < 320 || panelHeight < 176) {
      return "veryCompact";
    }
    if (profile.density === "compact" || panelWidth < 430 || panelHeight < 260) {
      return "compact";
    }
    return "regular";
  }

  function applyHudProfile() {
    const root = getHudRoot();
    const card = getHudCard();
    if (!root || !card) return;

    const surface = getHudSurface();
    const constraints = getHudConstraints(surface);
    const sig = `${surface}|${Math.round(constraints.width)}|${Math.round(constraints.height)}`;
    if (_hudProfileSig === sig) return;
    _hudProfileSig = sig;

    mountHudToSurface(surface);

    let profile = buildHudProfile(constraints.width, constraints.height, surface);
    if (surface === SURFACE_OVERLAY) {
      const overlayScale = clamp(Math.min(constraints.width / 340, constraints.height / 210), 0.92, 1.02);
      profile = {
        ...profile,
        density: "micro",
        wide: false,
        borderRadius: 18,
        dockInset: 4,
        padding: Math.max(8, Math.round(profile.padding * 0.64)),
        sectionGap: Math.max(6, Math.round(profile.sectionGap * 0.56)),
        metricGap: Math.max(5, Math.round(profile.metricGap * 0.56)),
        speedFontSize: Math.round(profile.speedFontSize * 0.84 * overlayScale),
        primaryValueFontSize: Math.round(profile.primaryValueFontSize * 0.80 * overlayScale),
        secondaryValueFontSize: Math.round(profile.secondaryValueFontSize * 0.82 * overlayScale),
        labelFontSize: Math.round(profile.labelFontSize * 0.92 * overlayScale * 10) / 10,
        chipFontSize: Math.round(profile.chipFontSize * overlayScale * 10) / 10,
        gearFontSize: Math.round(profile.gearFontSize * 0.74 * overlayScale),
        maxWidth: Math.min(profile.maxWidth, 340),
      };
    }
    const bandTypography = getBandTypography(profile, surface);
    const style = root.style;
    const portraitInline = surface === SURFACE_INLINE && !isOverlayMount();
    const exactSizedSurface = surface === SURFACE_INLINE || surface === SURFACE_OVERLAY;
    const panelWidth = exactSizedSurface
      ? Math.round(constraints.width)
      : surface === SURFACE_HOME
        ? Math.min(constraints.width, profile.maxWidth, constraints.height * profile.preferredAspectRatio)
        : Math.min(constraints.width, profile.maxWidth);
    const panelHeight = exactSizedSurface
      ? Math.round(constraints.height)
      : Math.round(panelWidth / Math.max(profile.preferredAspectRatio, 0.1));

    card.dataset.surface = surface;
    root.dataset.surface = surface;
    root.dataset.density = profile.density;
    root.dataset.wide = profile.wide ? "1" : "0";
    root.dataset.windowClass = getHudWindowClass(constraints.width);
    root.dataset.supportLayout = getHudSupportLayout(surface, profile, panelWidth, panelHeight);

    style.setProperty("--hud-aspect", String(profile.preferredAspectRatio));
    style.setProperty("--hud-radius", `${portraitInline ? 0 : profile.borderRadius}px`);
    style.setProperty("--hud-padding", `${profile.padding}px`);
    style.setProperty("--hud-section-gap", `${profile.sectionGap}px`);
    style.setProperty("--hud-metric-gap", `${profile.metricGap}px`);
    style.setProperty("--hud-speed-font", `${profile.speedFontSize}px`);
    style.setProperty("--hud-primary-font", `${profile.primaryValueFontSize}px`);
    style.setProperty("--hud-secondary-font", `${profile.secondaryValueFontSize}px`);
    style.setProperty("--hud-label-font", `${profile.labelFontSize}px`);
    style.setProperty("--hud-chip-font", `${profile.chipFontSize}px`);
    style.setProperty("--hud-gear-font", `${profile.gearFontSize}px`);
    style.setProperty("--hud-band-height", `${getBandHeight(profile.density)}px`);
    style.setProperty("--hud-metric-value-scale", bandTypography.metricScale.toFixed(2));
    style.setProperty("--hud-bottom-segment-scale", bandTypography.bottomScale.toFixed(2));
    style.setProperty("--hud-max-width", `${Math.round(profile.maxWidth)}px`);
    style.setProperty("--hud-dock-inset", `${profile.dockInset}px`);
    style.setProperty("--hud-text-shadow-strong", STRONG_TEXT_SHADOW);
    style.setProperty("--hud-card-width", `${Math.round(panelWidth)}px`);
    style.setProperty("--hud-card-height", `${panelHeight}px`);
    document.documentElement.style.setProperty("--carrot-dock-panel-height", `${Math.round(panelHeight + (portraitInline ? 0 : profile.dockInset * 2))}px`);
    document.documentElement.style.setProperty("--carrot-dock-panel-width", `${Math.round(panelWidth)}px`);
  }

  function scheduleHudProfileApply() {
    if (hudLayoutRaf) return;
    hudLayoutRaf = requestAnimationFrame(() => {
      hudLayoutRaf = 0;
      applyHudProfile();
    });
  }

  function bindHudLayout() {
    if (hudLayoutBound) return;
    hudLayoutBound = true;

    const handleLayout = () => scheduleHudProfileApply();
    window.addEventListener("resize", handleLayout, { passive: true });
    window.addEventListener("orientationchange", handleLayout, { passive: true });
    window.addEventListener("carrot:pagechange", handleLayout);
    if (window.visualViewport) {
      window.visualViewport.addEventListener("resize", handleLayout, { passive: true });
      window.visualViewport.addEventListener("scroll", handleLayout, { passive: true });
    }
    if (typeof ResizeObserver === "function") {
      const observer = new ResizeObserver(handleLayout);
      const carrotDock = $("carrotHudDock");
      const carrotStage = $("carrotStage");
      if (carrotDock) observer.observe(carrotDock);
      if (carrotStage) observer.observe(carrotStage);
    }
    scheduleHudProfileApply();
  }

  function setSignalDot(kind) {
    const el = $("hudSignalDot");
    if (!el) return;
    const state = String(kind || "off").toLowerCase();
    if (el.dataset.state !== state) el.dataset.state = state;
  }

  function setDriveMode(name, kind) {
    const el = $("hudDriveMode");
    if (!el) return;
    const normalized = String(kind || "").toLowerCase();
    const lang = currentLang();
    const registryLabels = typeof DRIVE_MODES === "object" ? DRIVE_MODES[lang] || DRIVE_MODES.en : null;
    const labels = Object.assign({}, DRIVE_MODE_TEXT.en, DRIVE_MODE_TEXT[lang] || {}, registryLabels || {});
    const translated = labels[normalized] || name || labels.normal;
    if (el.textContent !== translated) el.textContent = translated;
    if (el.dataset.kind !== (normalized || "normal")) el.dataset.kind = normalized || "normal";
  }

  function isMetricDisplay(value) {
    return value == null ? true : Boolean(value);
  }

  function formatDisplaySpeedKph(speedKph, isMetric) {
    if (speedKph == null || !isFinite(speedKph)) return "--";
    const displaySpeed = isMetricDisplay(isMetric) ? Number(speedKph) : Number(speedKph) * 0.621371;
    return `${Math.round(displaySpeed)}`;
  }

  function setRoadLimit(speedKph, over, blink, isMetric) {
    const el = $("hudRoadLimitDisplay");
    if (!el) return;
    const limitLabel = getHudLabels().limit;
    const text = `${limitLabel} ${formatDisplaySpeedKph(speedKph, isMetric)}`;
    if (el.textContent !== text) el.textContent = text;
    const overStr = over ? "1" : "0";
    if (el.dataset.over !== overStr) el.dataset.over = overStr;
    setCameraBlink(blink);
  }

  function setCameraBlink(on) {
    const root = getHudRoot();
    if (!root) return;
    const v = on ? "1" : "0";
    if (root.dataset.cameraBlink !== v) root.dataset.cameraBlink = v;
  }

  function setConnectivity(text) {
    setText("hudConnectivity", text && String(text).trim() ? String(text).trim() : "--");
  }

  function setBars(n) {
    const wrap = $("hudBars");
    if (!wrap) return;
    const bars = wrap.querySelectorAll(".hudGapBar");
    const count = clamp(Number(n) || 0, 0, bars.length);
    if (lastHudBarsCount === count) return;
    lastHudBarsCount = count;
    bars.forEach((bar, index) => {
      bar.classList.toggle("is-on", index < count);
    });
  }

  function setGapNum(n) {
    const display = n == null || !isFinite(n) ? "(--)" : `(${Math.round(n)})`;
    setText("hudGapNum", display);
  }

  function setTemp(temp, isMetric) {
    const reasonEl = $("hudTempReason");
    const speedEl = $("hudTempSpeed");
    if (!reasonEl || !speedEl) return;
    const lang = currentLang();

    if (!temp || temp.speed == null || !isFinite(temp.speed)) {
      const nextSignature = `${lang}|idle`;
      if (lastHudTempSignature === nextSignature) return;
      lastHudTempSignature = nextSignature;
      const labels = getHudLabels();
      if (reasonEl.textContent !== labels.temp) reasonEl.textContent = labels.temp;
      if (speedEl.textContent !== "--") speedEl.textContent = "--";
      if (reasonEl.style.color !== "rgba(255,255,255,0.84)") reasonEl.style.color = "rgba(255,255,255,0.84)";
      if (speedEl.style.color !== "rgba(255,255,255,0.88)") speedEl.style.color = "rgba(255,255,255,0.88)";
      return;
    }

    const reason = String(temp.source || getHudLabels().temp).trim();
    const color = temp.is_decel ? "#FFC94A" : "#34C96E";
    const speedText = formatDisplaySpeedKph(temp.speed, isMetric);
    const nextSignature = `${lang}|${reason || "TEMP"}|${speedText}|${color}`;
    if (lastHudTempSignature === nextSignature) return;
    lastHudTempSignature = nextSignature;
    if (reasonEl.textContent !== (reason || "TEMP")) reasonEl.textContent = reason || "TEMP";
    reasonEl.title = reason || "TEMP";
    if (speedEl.textContent !== speedText) speedEl.textContent = speedText;
    if (reasonEl.style.color !== color) reasonEl.style.color = color;
    if (speedEl.style.color !== color) speedEl.style.color = color;
  }

  function setSpeed(vEgoKph, isMetric) {
    setText("hudSpeed", formatDisplaySpeedKph(vEgoKph, isMetric));
  }

  function setSetSpeed(vSetKph, isMetric) {
    setText("hudSetSpeed", formatDisplaySpeedKph(vSetKph, isMetric));
  }

  function setGear(txt, gearStep) {
    const el = $("hudGear");
    const gearStepEl = $("hudGearStep");
    if (!el) return;
    const value = String(txt || "U").trim().toUpperCase() || "U";
    const unknown = value === "U" || value === "X";
    if (!unknown) lastStableGear = value;
    const stableKnown = lastStableGear && lastStableGear !== "U" && lastStableGear !== "X";
    const display = unknown ? (stableKnown ? lastStableGear : "\u2013") : value;
    if (el.textContent !== display) el.textContent = display;
    const unknownStr = display === "\u2013" ? "1" : "0";
    if (el.dataset.unknown !== unknownStr) el.dataset.unknown = unknownStr;

    if (gearStepEl) {
      const numericGearStep = Number(gearStep);
      const resolvedGearStep = Number.isFinite(numericGearStep) && numericGearStep > 0
        ? Math.round(numericGearStep)
        : (display === "D" && Number.isFinite(lastStableGearStep) ? lastStableGearStep : null);
      const showGearStep = display === "D" && Number.isFinite(resolvedGearStep) && resolvedGearStep > 0;
      if (showGearStep) {
        const nextText = `${resolvedGearStep}`;
        if (gearStepEl.textContent !== nextText) gearStepEl.textContent = nextText;
        gearStepEl.hidden = false;
        lastStableGearStep = resolvedGearStep;
      } else {
        gearStepEl.hidden = true;
        if (display !== "D") lastStableGearStep = null;
      }
      gearStepEl.dataset.active = showGearStep ? "1" : "0";
    }
  }

  function hasFiniteAuxMetric(value) {
    return value != null && isFinite(value);
  }

  function formatHudAuxMetric(kind, value) {
    if (!hasFiniteAuxMetric(value)) {
      return kind === "disk" ? "--%" : "--.-V";
    }
    return kind === "disk"
      ? `${Number(value).toFixed(0)}%`
      : `${Number(value).toFixed(1)}V`;
  }

  function renderHudAuxMetric() {
    const valueEl = getHudAuxValueEl();
    const iconPathEl = getHudAuxIconPathEl();
    if (!valueEl) return;

    const hasDisk = hasFiniteAuxMetric(hudAuxMetricState.diskPct);
    const hasVolt = hasFiniteAuxMetric(hudAuxMetricState.voltageV);
    if (!hasDisk && hasVolt) hudAuxMetricKind = "volt";
    else if (hasDisk && !hasVolt) hudAuxMetricKind = "disk";

    const kind = hudAuxMetricKind === "volt" ? "volt" : "disk";
    const nextValue = kind === "disk" ? hudAuxMetricState.diskPct : hudAuxMetricState.voltageV;
    const text = formatHudAuxMetric(kind, nextValue);
    if (valueEl.textContent !== text) valueEl.textContent = text;

    if (iconPathEl) {
      const nextPath = HUD_AUX_ICON_PATHS[kind];
      if (iconPathEl.getAttribute("d") !== nextPath) iconPathEl.setAttribute("d", nextPath);
    }
    valueEl.dataset.metric = kind;
  }

  function ensureHudAuxMetricTimer() {
    if (hudAuxMetricTimer) return;
    hudAuxMetricTimer = window.setInterval(() => {
      const hasDisk = hasFiniteAuxMetric(hudAuxMetricState.diskPct);
      const hasVolt = hasFiniteAuxMetric(hudAuxMetricState.voltageV);
      if (hasDisk && hasVolt) {
        hudAuxMetricKind = hudAuxMetricKind === "disk" ? "volt" : "disk";
      } else if (hasVolt) {
        hudAuxMetricKind = "volt";
      } else {
        hudAuxMetricKind = "disk";
      }
      renderHudAuxMetric();
    }, HUD_AUX_ROTATE_MS);
  }

  function setMetrics(cpuTempC, memPct, diskPct, voltageV) {
    setText("hudCpuVal", cpuTempC == null || !isFinite(cpuTempC) ? "--°" : `${cpuTempC.toFixed(0)}°`);
    setText("hudMemVal", memPct == null || !isFinite(memPct) ? "--%" : `${memPct.toFixed(0)}%`);
    hudAuxMetricState = {
      diskPct: hasFiniteAuxMetric(diskPct) ? Number(diskPct) : null,
      voltageV: hasFiniteAuxMetric(voltageV) ? Number(voltageV) : null,
    };
    renderHudAuxMetric();
  }

  function syncStaticHudText(force = false) {
    const lang = currentLang();
    if (!force && lastHudLabelLang === lang) return;
    lastHudLabelLang = lang;
    const labels = getHudLabels();
    setText("hudSpeedLabel", labels.speed);
    setText("hudSetSpeedLabel", labels.setSpeed);
    setText("hudGearLabel", labels.gear);
  }

  const DrivingHud = {
    init() {
      bindHudLayout();
      syncStaticHudText(true);
      ensureHudAuxMetricTimer();
      setMetrics(null, null, null, null);
      setSpeed(null);
      setSetSpeed(null);
      setTemp(null);
      setBars(0);
      setGapNum(null);
      setGear("U", null);
      setSignalDot("off");
      setDriveMode("", "normal");
      setRoadLimit(null, false);
      setConnectivity("--");
    },

    update(payload) {
      if (!payload) return;
      const isMetric = isMetricDisplay(payload.isMetric);
      syncStaticHudText();
      setMetrics(payload.cpuTempC, payload.memPct, payload.diskPct, payload.voltageV);
      setSpeed(payload.vEgoKph, isMetric);
      setSetSpeed(payload.vSetKph, isMetric);
      setTemp(payload.temp, isMetric);
      setBars(payload.tfBars != null ? payload.tfBars : payload.tfGap);
      setGapNum(payload.tfGap);
      setGear(payload.gear, payload.gearStep);
      setSignalDot(payload.tlight || "off");
      if (payload.driveMode) setDriveMode(payload.driveMode.name, payload.driveMode.kind);
      else setDriveMode("", "normal");
      setRoadLimit(payload.speedLimitKph, payload.speedLimitOver, payload.speedLimitBlink, isMetric);
      setConnectivity(payload.apm);
    },

    relayout() {
      scheduleHudProfileApply();
    },

    renderText() {
      syncStaticHudText(true);
      setDriveMode("", "normal");
      setRoadLimit(null, false);
    },
  };

  window.DrivingHud = DrivingHud;
})();
