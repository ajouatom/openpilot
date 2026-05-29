(function () {
  "use strict";

  // Carrot Nav HUD V2 — small top-center card with rich slots.
  //
  // Reads window.CarrotLiveRuntimeState.services.carrotMan and renders into
  // #carrotNavHud directly (no iframe, no network, no Kakao API). Independent
  // of carrot_map.js by design so it runs with or without the minimap.
  //
  // Main card modes:
  //   turn — turn arrow + dist + road
  //   idle — current road name + speed limit only
  //
  // Side card:
  //   camera / speed-limit details, independent of the main guidance card.
  //
  // Attachments (conditional, mode-independent):
  //   ATC badge       — atcType "prepare" / "go"
  //   Countdown chip  — xTurnCountDown / xSpdCountDown <= 10s
  //   Speed limit     — nRoadLimitSpeed (idle/turn)
  //   Hint badge      — trafficState (idle, red signal)

  // Hysteresis thresholds (enter vs stay).
  const SHOW_TURN_MAX_M = 1500;
  const HIDE_TURN_MAX_M = 2000;
  const SHOW_SDI_MAX_M = 800;
  const HIDE_SDI_MAX_M = 1100;

  // Stale cache: how long to keep showing the last payload when the feed
  // goes quiet. Long enough to ride out UDP drops / navi phase gaps.
  const HIDE_AFTER_STALE_MS = 10000;

  // Once visible, keep the card up at least this long. Prevents
  // threshold-flicker and short data gaps from blinking the card.
  const MIN_VISIBLE_MS = 4000;

  // Countdown chip threshold (xTurnCountDown / xSpdCountDown in seconds).
  const COUNTDOWN_SHOW_SEC = 10;

  // Refresh throttle (DOM updates max ~5Hz).
  const REFRESH_THROTTLE_MS = 200;

  // Turn-info code -> glyph + label. Matches selfdrive/ui/carrot.cc
  // TurnInfoDrawer::drawTurnInfoHud() switch statement.
  const TURN_ICONS = {
    1: "left",
    2: "right",
    3: "lane-left",
    4: "lane-right",
    5: "left",
    6: "toll",
    7: "uturn",
    8: "finish",
  };
  const TURN_LABELS = {
    1: "좌회전",
    2: "우회전",
    3: "좌 차로변경",
    4: "우 차로변경",
    5: "좌회전",
    6: "통행료",
    7: "유턴",
    8: "목적지",
  };

  // trafficState enum (carrot_serv): 0=none, 1=stopped(red), 2=go(green).
  // Conservative — only show "red" hint. "green" is noise.
  const TRAFFIC_RED = 1;
  const TRAFFIC_GREEN = 2;

  function finiteNumber(value) {
    const n = Number(value);
    return Number.isFinite(n) ? n : null;
  }

  function clamp(value, min, max) {
    return Math.max(min, Math.min(max, value));
  }

  function normalizeBool(value) {
    if (typeof value === "string") {
      return ["1", "true", "yes", "on"].includes(value.trim().toLowerCase());
    }
    return Boolean(value);
  }

  function getSetting(key, fallback) {
    const settings = window.CarrotWebSettingsState || {};
    return Object.prototype.hasOwnProperty.call(settings, key) ? settings[key] : fallback;
  }

  function isCarrotPageActive() {
    return document.body?.dataset?.page === "carrot";
  }

  function isVisionActive() {
    if (typeof window.isCarrotVisionActive === "function") return window.isCarrotVisionActive();
    return Boolean(window.CarrotVisionState?.active);
  }

  function isLandscape() {
    if (typeof window.matchMedia === "function") {
      try {
        return window.matchMedia("(orientation: landscape)").matches;
      } catch {}
    }
    return Number(window.innerWidth || 0) >= Number(window.innerHeight || 0);
  }

  function formatDistance(meters) {
    const m = finiteNumber(meters);
    if (m === null || m <= 0) return "";
    if (m < 950) return `${Math.round(m / 10) * 10}m`;
    const km = m / 1000;
    return km < 10 ? `${km.toFixed(1)}km` : `${Math.round(km)}km`;
  }

  function formatDuration(seconds) {
    const s = finiteNumber(seconds);
    if (s === null || s <= 0) return "";
    const mins = Math.round(s / 60);
    if (mins < 60) return `${mins}분`;
    const h = Math.floor(mins / 60);
    const rest = mins % 60;
    return rest ? `${h}시간 ${rest}분` : `${h}시간`;
  }

  // Computes "HH:MM" arrival time from now + remaining seconds.
  function formatEtaClock(remainingSec) {
    const s = finiteNumber(remainingSec);
    if (s === null || s <= 0) return "";
    const eta = new Date(Date.now() + s * 1000);
    const hh = String(eta.getHours()).padStart(2, "0");
    const mm = String(eta.getMinutes()).padStart(2, "0");
    return `${hh}:${mm}`;
  }

  function buildMeta(cm) {
    const goalDist = formatDistance(cm.nGoPosDist);
    const goalTime = formatDuration(cm.nGoPosTime);
    const etaClock = formatEtaClock(cm.nGoPosTime);
    const parts = [];
    if (goalDist) parts.push(goalDist);
    if (goalTime) parts.push(goalTime);
    if (etaClock) parts.push(`도착 ${etaClock}`);
    return parts.join(" · ");
  }

  function navIconSvg(kind) {
    const attrs = 'class="carrot-nav-hud__icon-svg" viewBox="0 0 48 48" aria-hidden="true" focusable="false"';
    const pathAttrs = 'fill="none" stroke="currentColor" stroke-width="6.5" stroke-linecap="round" stroke-linejoin="round"';
    switch (kind) {
      case "straight":
        return `<svg ${attrs}><path ${pathAttrs} d="M24 39V10"/><path ${pathAttrs} d="M14 20 24 10l10 10"/></svg>`;
      case "left":
        return `<svg ${attrs}><path ${pathAttrs} d="M34 38V26c0-7-5-12-12-12H13"/><path ${pathAttrs} d="M21 6l-8 8 8 8"/></svg>`;
      case "right":
        return `<svg ${attrs}><path ${pathAttrs} d="M14 38V26c0-7 5-12 12-12h9"/><path ${pathAttrs} d="M27 6l8 8-8 8"/></svg>`;
      case "lane-left":
        return `<svg ${attrs}><path ${pathAttrs} d="M31 39V12"/><path ${pathAttrs} d="M18 36V24c0-6 4-10 10-10h5"/><path ${pathAttrs} d="M26 6l8 8-8 8"/></svg>`;
      case "lane-right":
        return `<svg ${attrs}><path ${pathAttrs} d="M17 39V12"/><path ${pathAttrs} d="M30 36V24c0-6-4-10-10-10h-5"/><path ${pathAttrs} d="M22 6l-8 8 8 8"/></svg>`;
      case "uturn":
        return `<svg ${attrs}><path ${pathAttrs} d="M33 39V22c0-7-5-12-12-12h-5"/><path ${pathAttrs} d="M23 3l-8 7 8 8"/></svg>`;
      case "finish":
        return `<svg ${attrs}><path ${pathAttrs} d="M15 39V9"/><path fill="currentColor" d="M18 9h18l-4 7 4 7H18z"/></svg>`;
      default:
        return "";
    }
  }

  class CarrotNavHud {
    constructor() {
      this.root = document.getElementById("carrotNavHud");
      this.mainEl = this.root?.querySelector("[data-nav-hud-main]") || null;
      this.sideEl = this.root?.querySelector("[data-nav-hud-side]") || null;
      this.sideSignEl = this.root?.querySelector("[data-nav-hud-side-sign]") || null;
      this.sideDistEl = this.root?.querySelector("[data-nav-hud-side-dist]") || null;
      this.sideLabelEl = this.root?.querySelector("[data-nav-hud-side-label]") || null;
      this.sideCountdownEl = this.root?.querySelector("[data-nav-hud-side-countdown]") || null;
      this.atcEl = this.root?.querySelector("[data-nav-hud-atc]") || null;
      this.iconEl = this.root?.querySelector("[data-nav-hud-icon]") || null;
      this.distEl = this.root?.querySelector("[data-nav-hud-dist]") || null;
      this.adviceEl = this.root?.querySelector("[data-nav-hud-advice]") || null;
      this.countdownEl = this.root?.querySelector("[data-nav-hud-countdown]") || null;
      this.roadEl = this.root?.querySelector("[data-nav-hud-road]") || null;
      this.limitEl = this.root?.querySelector("[data-nav-hud-limit]") || null;
      this.metaEl = this.root?.querySelector("[data-nav-hud-meta]") || null;
      this.hintEl = this.root?.querySelector("[data-nav-hud-hint]") || null;

      this.lastSig = "";
      this.lastRenderAt = 0;
      this.lastDataAt = 0;
      this.visible = false;
      this.visibleSinceMs = 0;
      this.lastMode = "";
      this.hideTimer = 0;
      this.resizeObserver = null;
      this.viewportLayout = null;

      this.sync = this.sync.bind(this);
      this.tick = this.tick.bind(this);
      this.handleVisibility = this.handleVisibility.bind(this);
    }

    init() {
      if (!this.root) return;
      window.addEventListener("carrot:pagechange", this.sync);
      window.addEventListener("carrot:visionchange", this.sync);
      window.addEventListener("carrot:websettingschange", this.sync);
      window.addEventListener("carrot:render-request", this.tick);
      window.addEventListener("carrot:viewportlayout", (event) => {
        this.viewportLayout = event.detail || null;
        this.sync();
      });
      window.addEventListener("resize", this.sync);
      window.addEventListener("orientationchange", this.sync);
      document.addEventListener("visibilitychange", this.handleVisibility);
      const stage = document.getElementById("carrotStage");
      if (stage && typeof ResizeObserver === "function") {
        this.resizeObserver = new ResizeObserver(this.sync);
        this.resizeObserver.observe(stage);
      }
      this.sync();
    }

    enabled() {
      return normalizeBool(getSetting("nav_hud_enabled", true));
    }

    shouldRun() {
      if (!this.enabled()) return false;
      if (!isCarrotPageActive()) return false;
      if (!isVisionActive()) return false;
      if (!isLandscape()) return false;
      if (document.visibilityState === "hidden") return false;
      return true;
    }

    handleVisibility() {
      if (document.visibilityState !== "visible") {
        this.hide({ force: true });
        return;
      }
      this.sync();
    }

    sync() {
      if (!this.shouldRun()) {
        this.hide({ force: true });
        return;
      }
      this.updateLayout(!this.sideEl?.hidden);
      this.tick();
    }

    updateLayout(sideVisible = false) {
      if (!this.root) return;
      const stage = document.getElementById("carrotStage");
      const stageRect = stage?.getBoundingClientRect?.();
      const stageWidth = stage?.clientWidth || stageRect?.width || window.innerWidth || 0;
      const stageHeight = stage?.clientHeight || stageRect?.height || window.innerHeight || 0;
      if (!stageWidth || !stageHeight) return;

      const edge = clamp(stageWidth * 0.012, 10, 18);
      const viewport = this.viewportLayout || {};
      const viewportLeft = clamp(finiteNumber(viewport.left) ?? 0, 0, stageWidth);
      const viewportTop = clamp(finiteNumber(viewport.top) ?? 0, 0, stageHeight);
      const viewportWidth = clamp(finiteNumber(viewport.width) ?? stageWidth, 1, stageWidth);
      const top = Math.round(clamp(viewportTop + (stageHeight <= 500 ? 36 : 38), edge, stageHeight - 96));
      let gap = clamp(stageWidth * 0.010, 10, 14);
      let sideWidth = sideVisible ? clamp(stageWidth * 0.145, 156, 204) : 0;
      let mainWidth = clamp(stageWidth * 0.31, 318, 430);
      const groupWidth = () => (sideVisible ? sideWidth + gap : 0) + mainWidth;
      const available = Math.max(260, stageWidth - edge * 2);
      if (mainWidth > available) {
        mainWidth = clamp(available, 280, mainWidth);
      }
      const mainLeft = Math.round(clamp(
        viewportLeft + (viewportWidth - mainWidth) / 2,
        edge,
        Math.max(edge, stageWidth - edge - mainWidth),
      ));
      let sideLeft = Math.round(mainLeft - gap - sideWidth);
      if (sideVisible && sideLeft < edge) {
        sideWidth = clamp(mainLeft - gap - edge, 132, sideWidth);
        sideLeft = Math.round(mainLeft - gap - sideWidth);
      }

      const compact = sideVisible && (sideLeft <= edge + 1 || groupWidth() > stageWidth * 0.68);
      const cardHeight = compact ? 78 : clamp(stageHeight * 0.12, 82, 92);
      if (compact) this.root.dataset.layout = "compact";
      else delete this.root.dataset.layout;
      this.root.dataset.sideVisible = sideVisible ? "1" : "0";

      this.root.style.setProperty("--nav-top", `${top}px`);
      this.root.style.setProperty("--nav-main-width", `${Math.round(mainWidth)}px`);
      this.root.style.setProperty("--nav-side-width", `${Math.round(Math.max(0, sideWidth))}px`);
      this.root.style.setProperty("--nav-gap", `${Math.round(gap)}px`);
      this.root.style.setProperty("--nav-card-height", `${Math.round(cardHeight)}px`);
      this.root.style.setProperty("--nav-main-left", `${mainLeft}px`);
      this.root.style.setProperty("--nav-side-left", `${Math.max(edge, sideLeft)}px`);
    }

    readCarrotMan() {
      const runtimeState = window.CarrotLiveRuntimeState;
      if (!runtimeState?.ok) return null;
      const services = runtimeState.services || {};
      const cm = services.carrotMan;
      if (!cm || typeof cm !== "object") return null;
      this.lastDataAt = finiteNumber(runtimeState.fetchedAtMs) || Date.now();
      return cm;
    }

    pickPayload(cm) {
      // Active flag — match carrot.cc: activeCarrot > 1 means actively guiding.
      const active = (finiteNumber(cm.activeCarrot) ?? 0) > 1;
      if (!active) return null;

      const sdiDist = finiteNumber(cm.xSpdDist) ?? 0;
      const sdiType = finiteNumber(cm.xSpdType) ?? -1;
      const sdiLimit = finiteNumber(cm.xSpdLimit) ?? 0;
      const sdiDescr = String(cm.szSdiDescr || "").trim();
      const sdiCountdown = finiteNumber(cm.xSpdCountDown) ?? 0;

      const turnInfo = finiteNumber(cm.xTurnInfo) ?? -1;
      const turnDist = finiteNumber(cm.xDistToTurn) ?? 0;
      const turnCountdown = finiteNumber(cm.xTurnCountDown) ?? 0;
      const roadLimit = finiteNumber(cm.nRoadLimitSpeed) ?? 0;
      const roadName = String(cm.szPosRoadName || "").trim();
      const tbtRoad = String(cm.szTBTMainText || "").trim();
      const atcType = String(cm.atcType || "").trim().toLowerCase();
      const trafficState = finiteNumber(cm.trafficState) ?? 0;
      const meta = buildMeta(cm);

      // Shared attachments — computed once, slotted per mode below.
      const atcBadge = (atcType === "prepare" || atcType === "go")
        ? (atcType === "prepare" ? "차로변경 준비" : "차로변경 중")
        : "";
      const limitText = roadLimit > 0 ? String(roadLimit) : "";

      // Hysteresis: if a mode is already visible, stay until the wider HIDE_*.
      const sdiThreshold = (this.visible && this.root?.dataset.sideKind === "camera") ? HIDE_SDI_MAX_M : SHOW_SDI_MAX_M;
      const turnThreshold = (this.visible && this.lastMode === "turn") ? HIDE_TURN_MAX_M : SHOW_TURN_MAX_M;

      let side = null;
      if (sdiDist > 0 && sdiDist <= sdiThreshold && sdiType > 0) {
        const countdown = (sdiCountdown > 0 && sdiCountdown <= COUNTDOWN_SHOW_SEC) ? `${sdiCountdown}초` : "";
        side = {
          kind: "camera",
          sign: sdiLimit > 0 ? String(sdiLimit) : "!",
          dist: formatDistance(sdiDist),
          countdown,
          label: sdiDescr || (sdiLimit > 0 ? `${sdiLimit} 제한 단속` : "단속 구간"),
        };
      } else if (limitText) {
        side = {
          kind: "limit",
          sign: limitText,
          dist: "",
          countdown: "",
          label: "제한속도",
        };
      }

      // Main card: turn guidance has priority, but speed/camera no longer
      // replaces the whole card. That keeps the top-center layout stable.
      if (turnInfo > 0 && turnDist > 0 && turnDist <= turnThreshold) {
        const countdown = (turnCountdown > 0 && turnCountdown <= COUNTDOWN_SHOW_SEC) ? `${turnCountdown}초` : "";
        return {
          mode: "turn",
          icon: turnInfo === 6 ? "TG" : "",
          iconKind: TURN_ICONS[turnInfo] || "straight",
          dist: formatDistance(turnDist),
          advice: "",
          countdown,
          road: tbtRoad || TURN_LABELS[turnInfo] || "안내",
          limit: "",
          meta,
          atc: atcBadge,
          side,
          hint: "",
          hintTone: "",
        };
      }

      // Idle — road name / route summary. Keep it stable while activeCarrot ≥ 2.
      let hint = "";
      let hintTone = "";
      if (trafficState === TRAFFIC_RED) {
        hint = "신호 대기";
        hintTone = "red";
      } else if (trafficState === TRAFFIC_GREEN) {
        // Suppress green by default to reduce noise; uncomment to enable.
        // hint = "출발"; hintTone = "green";
      }

      return {
        mode: "idle",
        icon: "",
        iconKind: "straight",
        dist: "",
        advice: "",
        countdown: "",
        road: roadName || tbtRoad || "주행 중",
        limit: "",
        meta,
        atc: atcBadge,
        side,
        hint,
        hintTone,
      };
    }

    tick() {
      if (!this.shouldRun()) {
        this.hide({ force: true });
        return;
      }
      const now = Date.now();
      if (now - this.lastRenderAt < REFRESH_THROTTLE_MS) return;

      const cm = this.readCarrotMan();
      if (!cm) {
        if (this.lastDataAt && now - this.lastDataAt > HIDE_AFTER_STALE_MS) this.hide();
        return;
      }

      const payload = this.pickPayload(cm);
      if (!payload) {
        this.hide();
        return;
      }
      this.updateLayout(Boolean(payload.side));

      const sig = [
        payload.mode, payload.icon, payload.dist, payload.advice, payload.countdown,
        payload.iconKind,
        payload.road, payload.limit, payload.meta, payload.atc,
        payload.side?.kind, payload.side?.sign, payload.side?.dist, payload.side?.label, payload.side?.countdown,
        payload.hint, payload.hintTone,
      ].join("|");
      if (sig === this.lastSig && this.visible) return;
      this.lastSig = sig;
      this.lastRenderAt = now;
      this.render(payload);
    }

    render(payload) {
      if (!this.root) return;
      this.updateLayout(Boolean(payload.side));
      this.root.dataset.mainMode = payload.mode;
      this.root.dataset.mode = payload.mode;
      this.lastMode = payload.mode;
      if (payload.atc) {
        // Distinguish prepare vs go via raw atcType — we already mapped to label
        // text, but the CSS hook reads data-atc. Approximate: any non-empty
        // means active; CSS treats "prepare"|"go" identically (green outline).
        this.root.dataset.atc = payload.atc.includes("중") ? "go" : "prepare";
      } else {
        delete this.root.dataset.atc;
      }

      if (this.iconEl) {
        const iconSvg = navIconSvg(payload.iconKind);
        if (iconSvg) this.iconEl.innerHTML = iconSvg;
        else this.iconEl.textContent = payload.icon || "";
      }
      if (this.distEl) this.distEl.textContent = payload.dist;
      if (this.adviceEl) {
        this.adviceEl.textContent = payload.advice;
        this.adviceEl.hidden = !payload.advice;
      }
      if (this.countdownEl) {
        this.countdownEl.textContent = payload.countdown;
        this.countdownEl.hidden = !payload.countdown;
      }
      if (this.roadEl) this.roadEl.textContent = payload.road;
      if (this.limitEl) {
        this.limitEl.textContent = payload.limit;
        this.limitEl.hidden = !payload.limit;
      }
      if (this.metaEl) {
        this.metaEl.textContent = payload.meta;
        this.metaEl.hidden = !payload.meta;
      }
      if (this.atcEl) {
        this.atcEl.textContent = payload.atc;
        this.atcEl.hidden = !payload.atc;
      }
      if (this.hintEl) {
        this.hintEl.textContent = payload.hint;
        this.hintEl.hidden = !payload.hint;
        if (payload.hintTone) this.hintEl.dataset.tone = payload.hintTone;
        else delete this.hintEl.dataset.tone;
      }
      this.renderSide(payload.side);
      this.show();
    }

    renderSide(side) {
      if (!this.root || !this.sideEl) return;
      if (!side) {
        this.sideEl.hidden = true;
        delete this.root.dataset.sideKind;
        delete this.sideEl.dataset.kind;
        return;
      }
      this.root.dataset.sideKind = side.kind || "";
      this.sideEl.dataset.kind = side.kind || "";
      if (this.sideSignEl) this.sideSignEl.textContent = side.sign || "";
      if (this.sideDistEl) {
        this.sideDistEl.textContent = side.dist || "";
        this.sideDistEl.hidden = !side.dist;
      }
      if (this.sideLabelEl) this.sideLabelEl.textContent = side.label || "";
      if (this.sideCountdownEl) {
        this.sideCountdownEl.textContent = side.countdown || "";
        this.sideCountdownEl.hidden = !side.countdown;
      }
      this.sideEl.hidden = false;
    }

    show() {
      if (!this.root || this.visible) return;
      if (this.hideTimer) {
        window.clearTimeout(this.hideTimer);
        this.hideTimer = 0;
      }
      this.root.hidden = false;
      window.requestAnimationFrame(() => {
        window.requestAnimationFrame(() => {
          this.root.classList.add("is-visible");
        });
      });
      this.visible = true;
      this.visibleSinceMs = Date.now();
    }

    hide({ force = false } = {}) {
      if (!this.root) return;
      if (!this.visible && this.root.hidden) return;
      if (!force && this.visible && this.visibleSinceMs > 0) {
        const aliveMs = Date.now() - this.visibleSinceMs;
        if (aliveMs < MIN_VISIBLE_MS) return;
      }
      this.root.classList.remove("is-visible");
      this.visible = false;
      this.visibleSinceMs = 0;
      this.lastSig = "";
      delete this.root.dataset.atc;
      delete this.root.dataset.sideKind;
      const finalize = () => {
        if (this.visible || !this.root) return;
        this.root.hidden = true;
        if (this.sideEl) this.sideEl.hidden = true;
        this.hideTimer = 0;
      };
      if (force) {
        finalize();
      } else {
        if (this.hideTimer) window.clearTimeout(this.hideTimer);
        this.hideTimer = window.setTimeout(finalize, 260);
      }
    }
  }

  const instance = new CarrotNavHud();
  window.CarrotNavHud = instance;

  if (document.readyState === "loading") {
    window.addEventListener("DOMContentLoaded", () => instance.init(), { once: true });
  } else {
    instance.init();
  }
})();
