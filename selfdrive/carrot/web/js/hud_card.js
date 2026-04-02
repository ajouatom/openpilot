/* Adaptive driving HUD
 * - Exposes: window.DrivingHud.init(), window.DrivingHud.update(payload)
 * - Mirrors CarrotLink HUD layout profiles for homePreview / driveInline / driveOverlay
 */
(function () {
  function $(id) { return document.getElementById(id); }
  function setText(id, v) { const el = $(id); if (el) el.textContent = (v == null ? "" : String(v)); }
  function show(id, on) { const el = $(id); if (el) el.style.display = on ? "" : "none"; }
  function clamp(v, min, max) { return Math.min(max, Math.max(min, v)); }

  const SURFACE_HOME = "homePreview";
  const SURFACE_INLINE = "driveInline";
  const SURFACE_OVERLAY = "driveOverlay";

  let hudLayoutBound = false;
  let hudLayoutRaf = 0;

  function getHudRoot() {
    return $("hudRoot");
  }

  function getHudSurface() {
    const page = document.body?.dataset?.page || "home";
    const coarseLandscape = window.matchMedia("(orientation: landscape) and (max-height: 560px) and (pointer: coarse)").matches;
    if (page === "carrot") return coarseLandscape ? SURFACE_OVERLAY : SURFACE_INLINE;
    return SURFACE_HOME;
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

  function getHudConstraints(surface) {
    const vv = window.visualViewport;
    const viewportWidth = Math.max(320, Math.round(vv?.width || window.innerWidth || 0));
    const viewportHeight = Math.max(320, Math.round(vv?.height || window.innerHeight || 0));
    const navRect = document.querySelector(".topbar")?.getBoundingClientRect();
    const navTop = navRect ? navRect.top : (window.innerHeight - 72);
    const utilityRect = document.querySelector("#pageHome .home-utility")?.getBoundingClientRect();
    const rootRect = getHudRoot()?.getBoundingClientRect();

    if (surface === SURFACE_HOME) {
      return {
        width: clamp(viewportWidth - 28, 280, 428),
        height: clamp(navTop - ((utilityRect?.bottom || 0) + 24), 180, Math.max(180, viewportHeight * 0.44)),
      };
    }

    if (surface === SURFACE_INLINE) {
      return {
        width: clamp(viewportWidth - 12, 304, 476),
        height: clamp(viewportHeight * 0.42, 190, 470),
      };
    }

    return {
      width: clamp(rootRect?.width || (viewportWidth * 0.34), 260, 540),
      height: clamp(rootRect?.height || (viewportHeight * 0.30), 184, 304),
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
    };
  }

  function applyHudProfile() {
    const root = getHudRoot();
    if (!root) return;

    const surface = getHudSurface();
    const constraints = getHudConstraints(surface);
    const profile = buildHudProfile(constraints.width, constraints.height, surface);
    const style = root.style;
    const bandHeight = getBandHeight(profile.density);

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
    style.setProperty("--hud-band-height", `${bandHeight}px`);

    if (surface === SURFACE_INLINE) {
      const portraitHudHeight = Math.round(constraints.width / Math.max(profile.preferredAspectRatio, 0.76));
      document.body.style.setProperty("--carrot-portrait-hud-height", `${portraitHudHeight}px`);
    } else {
      document.body.style.removeProperty("--carrot-portrait-hud-height");
    }
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
    scheduleHudProfileApply();
  }

  function setSignalDot(kind) {
    const el = $("hudSignalDot");
    if (!el) return;
    if (kind === "red") el.style.background = "#ff5c5c";
    else if (kind === "green") el.style.background = "#34c96e";
    else if (kind === "yellow") el.style.background = "#ffc94a";
    else el.style.background = "rgba(255,255,255,0.22)";
  }

  function setGear(txt) {
    const el = $("hudGear");
    if (!el) return;
    el.textContent = txt || "U";
    const t = String(txt || "U").toUpperCase();
    if (t === "R") el.style.color = "#ffc94a";
    else if (t === "N") el.style.color = "#7ec8ff";
    else if (t === "U") el.style.color = "rgba(255,255,255,0.72)";
    else el.style.color = "#ffffff";
  }

  function setDriveMode(name, kind) {
    const el = $("hudDriveMode");
    if (!el) return;

    let modeName = name;
    if (window.DRIVE_MODES && window.LANG) {
      modeName = window.DRIVE_MODES[window.LANG][kind] || name;
    }

    el.textContent = modeName || (window.DRIVE_MODES && window.LANG ? window.DRIVE_MODES[window.LANG].normal : "Normal");
    el.classList.remove("mode_normal", "mode_eco", "mode_safe", "mode_sport");
    if (kind === "eco") el.classList.add("mode_eco");
    else if (kind === "safe") el.classList.add("mode_safe");
    else if (kind === "sport") el.classList.add("mode_sport");
    else el.classList.add("mode_normal");
  }

  function setGps(ok) {
    const el = $("hudGps");
    if (!el) return;
    el.classList.toggle("off", !ok);
  }

  function setRoadLimit(speedKph, over) {
    const box = $("hudRoadLimitVal");
    setText("hudRoadLimitVal", (speedKph == null || !isFinite(speedKph)) ? "--" : Math.round(speedKph));
    if (box) box.classList.toggle("over", !!over);
  }

  function setBars(n) {
    const wrap = $("hudBars");
    if (!wrap) return;
    const bars = wrap.querySelectorAll(".hudBar");
    const k = Math.max(0, Math.min(bars.length, Number(n) || 0));
    const start = bars.length - k;
    bars.forEach((b, i) => b.classList.toggle("on", i >= start));
  }

  function setGapNum(n) {
    const el = $("hudGapNum");
    if (!el) return;
    el.textContent = (n == null || !isFinite(n)) ? "-" : String(Math.round(n));
  }

  function setTemp(temp) {
    if (!temp || temp.speed == null || !isFinite(temp.speed) || !temp.source) return;
    $("hudTempReason").textContent = String(temp.source);
    $("hudTempSpeed").textContent = String(Math.round(temp.speed));

    const isDecel = !!temp.is_decel;
    const color = isDecel ? "#ffc94a" : "#34c96e";
    $("hudTempReason").style.color = color;
    $("hudTempSpeed").style.color = color;
  }

  function setSpeed(vEgoKph) {
    const el = $("hudSpeed");
    if (!el) return;
    el.textContent = (vEgoKph == null || !isFinite(vEgoKph)) ? "--" : String(Math.round(vEgoKph));
  }

  function setSetSpeed(vSetKph) {
    const el = $("hudSetSpeed");
    if (!el) return;
    el.textContent = (vSetKph == null || !isFinite(vSetKph)) ? "--" : String(Math.round(vSetKph));
  }

  function setRedDot(on) {
    show("hudRedDot", !!on);
  }

  function setSys(cpuTempC, memPct, diskPct, diskLabel) {
    setText("hudCpuVal", (cpuTempC == null || !isFinite(cpuTempC)) ? "--°C" : `${cpuTempC.toFixed(0)}°C`);
    setText("hudMemVal", (memPct == null || !isFinite(memPct)) ? "--%" : `${memPct.toFixed(0)}%`);
    if (diskPct == null || !isFinite(diskPct)) setText("hudDiskVal", "--V");
    else setText("hudDiskVal", `${Number(diskPct).toFixed(1)}V`);
    if (diskLabel) setText("hudDiskLabel", diskLabel);
  }

  const DrivingHud = {
    init() {
      bindHudLayout();
      scheduleHudProfileApply();
      setBars(0);
      setSignalDot("off");
      setGps(false);
      setDriveMode("", "normal");
      setRoadLimit(null, false);
      setGapNum(null);
      setGear("U");
      setRedDot(false);
      setTemp(null);
    },

    update(p) {
      if (!p) return;
      scheduleHudProfileApply();
      setSys(p.cpuTempC, p.memPct, p.diskPct, p.diskLabel);
      setSpeed(p.vEgoKph);
      setSetSpeed(p.vSetKph);
      setSignalDot(p.tlight || "off");
      setRedDot(p.redDot);
      setTemp(p.temp);
      setGapNum(p.tfGap);
      setBars(p.tfBars != null ? p.tfBars : p.tfGap);
      setGear(p.gear);
      setGps(!!p.gpsOk);
      if (p.driveMode) setDriveMode(p.driveMode.name, p.driveMode.kind);
      setRoadLimit(p.speedLimitKph, p.speedLimitOver);
    },

    relayout() {
      scheduleHudProfileApply();
    },
  };

  window.DrivingHud = DrivingHud;
})();
