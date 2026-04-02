/* Adaptive driving HUD
 * - Uses CarrotLink drive surfaces directly:
 *   driveInline / driveOverlay
 * - Same data model, different mount target + layout profile per surface
 */
(function () {
  function $(id) { return document.getElementById(id); }
  function clamp(v, min, max) { return Math.min(max, Math.max(min, v)); }
  function setText(id, v) {
    const el = $(id);
    if (el) el.textContent = v == null ? "" : String(v);
  }

  const SURFACE_INLINE = "driveInline";
  const SURFACE_OVERLAY = "driveOverlay";
  const STRONG_TEXT_SHADOW = "0 1.4px 3.6px rgba(0,0,0,0.94), 0 0 1.2px rgba(0,0,0,0.62)";
  const DRIVE_MODE_TEXT = {
    normal: "일반",
    eco: "에코",
    safe: "안전",
    sport: "고속",
    fast: "고속",
  };

  let hudLayoutBound = false;
  let hudLayoutRaf = 0;

  function getHudCard() {
    return $("driveHudCard");
  }

  function getHudRoot() {
    return $("hudRoot");
  }

  function getHudSurface() {
    const page = document.body?.dataset?.page || "carrot";
    const landscape = window.matchMedia("(orientation: landscape)").matches;
    if (page === "carrot") return landscape ? SURFACE_OVERLAY : SURFACE_INLINE;
    return SURFACE_INLINE;
  }

  function getHudWindowClass(width) {
    if (width < 600) return "compact";
    if (width < 840) return "medium";
    if (width < 1200) return "expanded";
    if (width < 1600) return "large";
    return "extraLarge";
  }

  function getBandHeight(density) {
    switch (density) {
      case "micro": return 30;
      case "compact": return 34;
      case "regular": return 38;
      default: return 42;
    }
  }

  function getSurfaceTarget(surface) {
    if (surface === SURFACE_INLINE) return $("carrotHudDock");
    return $("carrotStage");
  }

  function mountHudToSurface(surface) {
    const card = getHudCard();
    const target = getSurfaceTarget(surface);
    if (!card || !target) return;

    if (card.parentElement !== target) {
      target.appendChild(card);
    }

    card.classList.remove("driveHudCard--inline", "driveHudCard--overlay");
    if (surface === SURFACE_INLINE) card.classList.add("driveHudCard--inline");
    else card.classList.add("driveHudCard--overlay");
  }

  function getHudConstraints(surface) {
    const vv = window.visualViewport;
    const viewportWidth = Math.max(320, Math.round(vv?.width || window.innerWidth || 0));
    const viewportHeight = Math.max(320, Math.round(vv?.height || window.innerHeight || 0));
    const target = getSurfaceTarget(surface);
    const rect = target?.getBoundingClientRect?.() || null;
    const width = rect?.width || viewportWidth;
    const height = rect?.height || viewportHeight;

    if (surface === SURFACE_INLINE) {
      return {
        width: clamp(width - 4, 304, 734),
        height: clamp(height || viewportHeight * 0.30, 190, 440),
      };
    }

    return {
      width: clamp((rect?.width || viewportWidth) * 0.34, 260, 820),
      height: clamp((rect?.height || viewportHeight) * 0.30, 184, 304),
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
        dockInset: surface === SURFACE_OVERLAY ? 4 : 6,
        padding: 13,
        sectionGap: 11,
        metricGap: 9,
        speedFontSize: 74,
        primaryValueFontSize: 34,
        secondaryValueFontSize: 24,
        labelFontSize: 13.5,
        chipFontSize: 12.5,
        gearFontSize: 38,
        maxWidth: surface === SURFACE_OVERLAY ? 452 : 404,
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
        dockInset: surface === SURFACE_OVERLAY ? 6 : 8,
        padding: 16,
        sectionGap: 13,
        metricGap: 10,
        speedFontSize: 90,
        primaryValueFontSize: 38,
        secondaryValueFontSize: 26,
        labelFontSize: 14.5,
        chipFontSize: 13.5,
        gearFontSize: 46,
        maxWidth: surface === SURFACE_OVERLAY ? 540 : 476,
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
        dockInset: surface === SURFACE_OVERLAY ? 8 : 10,
        padding: 20,
        sectionGap: 15,
        metricGap: 11,
        speedFontSize: 110,
        primaryValueFontSize: 45,
        secondaryValueFontSize: 31,
        labelFontSize: 15.5,
        chipFontSize: 14.5,
        gearFontSize: 56,
        maxWidth: surface === SURFACE_OVERLAY ? 670 : 604,
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
      dockInset: surface === SURFACE_OVERLAY ? 10 : 12,
      padding: 24,
      sectionGap: 18,
      metricGap: 13,
      speedFontSize: 128,
      primaryValueFontSize: 52,
      secondaryValueFontSize: 36,
      labelFontSize: 16.5,
      chipFontSize: 15.5,
      gearFontSize: 68,
      maxWidth: surface === SURFACE_OVERLAY ? 820 : 734,
    };
  }

  function applyHudProfile() {
    const root = getHudRoot();
    const card = getHudCard();
    if (!root || !card) return;

    const surface = getHudSurface();
    mountHudToSurface(surface);

    const constraints = getHudConstraints(surface);
    const profile = buildHudProfile(constraints.width, constraints.height, surface);
    const style = root.style;

    card.dataset.surface = surface;
    root.dataset.surface = surface;
    root.dataset.density = profile.density;
    root.dataset.wide = profile.wide ? "1" : "0";
    root.dataset.windowClass = getHudWindowClass(constraints.width);

    style.setProperty("--hud-aspect", String(profile.preferredAspectRatio));
    style.setProperty("--hud-radius", `${profile.borderRadius}px`);
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
    style.setProperty("--hud-max-width", `${Math.round(profile.maxWidth)}px`);
    style.setProperty("--hud-dock-inset", `${profile.dockInset}px`);
    style.setProperty("--hud-text-shadow-strong", STRONG_TEXT_SHADOW);
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
    el.dataset.state = state;
  }

  function setDriveMode(name, kind) {
    const el = $("hudDriveMode");
    if (!el) return;
    const normalized = String(kind || "").toLowerCase();
    const translated = DRIVE_MODE_TEXT[normalized] || name || DRIVE_MODE_TEXT.normal;
    el.textContent = translated;
    el.dataset.kind = normalized || "normal";
  }

  function setRoadLimit(speedKph, over) {
    const el = $("hudRoadLimitDisplay");
    if (!el) return;
    const text = speedKph == null || !isFinite(speedKph) ? "LIMIT --" : `LIMIT ${Math.round(speedKph)}`;
    el.textContent = text;
    el.dataset.over = over ? "1" : "0";
  }

  function setConnectivity(text) {
    setText("hudConnectivity", text && String(text).trim() ? String(text).trim() : "--");
  }

  function setBars(n) {
    const wrap = $("hudBars");
    if (!wrap) return;
    const bars = wrap.querySelectorAll(".hudGapBar");
    const count = clamp(Number(n) || 0, 0, bars.length);
    bars.forEach((bar, index) => {
      bar.classList.toggle("is-on", index < count);
    });
  }

  function setGapNum(n) {
    const display = n == null || !isFinite(n) ? "(--)" : `(${Math.round(n)})`;
    setText("hudGapNum", display);
  }

  function setTemp(temp) {
    const reasonEl = $("hudTempReason");
    const speedEl = $("hudTempSpeed");
    if (!reasonEl || !speedEl) return;

    if (!temp || temp.speed == null || !isFinite(temp.speed)) {
      reasonEl.textContent = "TEMP";
      speedEl.textContent = "--";
      reasonEl.style.color = "rgba(255,255,255,0.84)";
      speedEl.style.color = "rgba(255,255,255,0.88)";
      return;
    }

    const reason = String(temp.source || "TEMP").trim();
    const color = temp.is_decel ? "#FFC94A" : "#34C96E";
    reasonEl.textContent = reason || "TEMP";
    speedEl.textContent = `${Math.round(temp.speed)}`;
    reasonEl.style.color = color;
    speedEl.style.color = color;
  }

  function setSpeed(vEgoKph) {
    setText("hudSpeed", vEgoKph == null || !isFinite(vEgoKph) ? "--" : `${Math.round(vEgoKph)}`);
  }

  function setSetSpeed(vSetKph) {
    setText("hudSetSpeed", vSetKph == null || !isFinite(vSetKph) ? "--" : `${Math.round(vSetKph)}`);
  }

  function setGear(txt) {
    const el = $("hudGear");
    if (!el) return;
    const value = String(txt || "U").trim().toUpperCase() || "U";
    const unknown = value === "U" || value === "X";
    el.textContent = unknown ? "–" : value;
    el.dataset.unknown = unknown ? "1" : "0";
  }

  function setMetrics(cpuTempC, memPct, diskPct) {
    setText("hudCpuVal", cpuTempC == null || !isFinite(cpuTempC) ? "--°" : `${cpuTempC.toFixed(0)}°`);
    setText("hudMemVal", memPct == null || !isFinite(memPct) ? "--%" : `${memPct.toFixed(0)}%`);
    setText("hudDiskVal", diskPct == null || !isFinite(diskPct) ? "--.-V" : `${Number(diskPct).toFixed(1)}V`);
  }

  const DrivingHud = {
    init() {
      bindHudLayout();
      setMetrics(null, null, null);
      setSpeed(null);
      setSetSpeed(null);
      setTemp(null);
      setBars(0);
      setGapNum(null);
      setGear("U");
      setSignalDot("off");
      setDriveMode("", "normal");
      setRoadLimit(null, false);
      setConnectivity("--");
    },

    update(payload) {
      if (!payload) return;
      scheduleHudProfileApply();
      setMetrics(payload.cpuTempC, payload.memPct, payload.diskPct);
      setSpeed(payload.vEgoKph);
      setSetSpeed(payload.vSetKph);
      setTemp(payload.temp);
      setBars(payload.tfBars != null ? payload.tfBars : payload.tfGap);
      setGapNum(payload.tfGap);
      setGear(payload.gear);
      setSignalDot(payload.tlight || "off");
      if (payload.driveMode) setDriveMode(payload.driveMode.name, payload.driveMode.kind);
      else setDriveMode("", "normal");
      setRoadLimit(payload.speedLimitKph, payload.speedLimitOver);
      setConnectivity(payload.apm);
    },

    relayout() {
      scheduleHudProfileApply();
    },
  };

  window.DrivingHud = DrivingHud;
})();
