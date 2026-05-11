(function () {
  const CARROT_VISION_PHASE = Object.freeze({
    UNAVAILABLE: "unavailable",
    INACTIVE: "inactive",
    STARTING: "starting",
    RTC_CONNECTING: "rtc-connecting",
    TRACK_WAITING: "track-waiting",
    FIRST_FRAME_WAITING: "first-frame-waiting",
    READY: "ready",
    RECOVERING: "recovering",
    FAILED: "failed",
  });
  const CARROT_VISION_PHASE_SET = new Set(Object.values(CARROT_VISION_PHASE));
  const CARROT_VISION_STATE = {
    active: false,
    available: false,
    phase: CARROT_VISION_PHASE.UNAVAILABLE,
    statusText: "",
    detailText: "",
    disabledMessage: getUIText("vision_unavailable_hint", "Available when DisableDM is 2."),
    reason: "init",
    updatedAtMs: Date.now(),
    rtc: {
      state: "idle",
      pcLabel: "none",
      trackSeen: false,
      liveTrack: false,
      pending: false,
    },
    raw: {
      hud: "idle",
      overlay: "idle",
    },
  };

  function isCarrotVisionActive() {
    return Boolean(CARROT_VISION_STATE.active);
  }

  function syncLegacyCarrotVisionFlags() {
    window.CARROT_VISION_ACTIVE = Boolean(CARROT_VISION_STATE.active);
    window.CARROT_VISION_AVAILABLE = Boolean(CARROT_VISION_STATE.available);
    window.CARROT_VISION_DISABLED_MESSAGE = CARROT_VISION_STATE.disabledMessage || "";
  }

  function getCarrotVisionPhaseStatusText(phase) {
    switch (phase) {
      case CARROT_VISION_PHASE.UNAVAILABLE:
        return CARROT_VISION_STATE.disabledMessage || getUIText("vision_unavailable_hint", "Available when DisableDM is 2.");
      case CARROT_VISION_PHASE.INACTIVE:
        return getUIText("start_vision_hint", "Tap the start button to enable drive vision.");
      case CARROT_VISION_PHASE.STARTING:
      case CARROT_VISION_PHASE.RTC_CONNECTING:
        return getUIText("connecting", "Connecting...");
      case CARROT_VISION_PHASE.TRACK_WAITING:
        return getUIText("connected_waiting_track", "Connected, waiting track...");
      case CARROT_VISION_PHASE.FIRST_FRAME_WAITING:
        return getUIText("waiting_road_stream", "Waiting road camera stream...");
      case CARROT_VISION_PHASE.READY:
        return getUIText("connected", "Connected");
      case CARROT_VISION_PHASE.RECOVERING:
        return getUIText("reconnecting", "Reconnecting...");
      case CARROT_VISION_PHASE.FAILED:
        return getUIText("disable_dm_check_failed", "Could not check DisableDM status.");
      default:
        return "";
    }
  }

  function getCarrotVisionPhaseDetailText(phase) {
    switch (phase) {
      case CARROT_VISION_PHASE.UNAVAILABLE:
        return getUIText("vision_step_unavailable", "Enable DisableDM 2 to use Carrot Vision.");
      case CARROT_VISION_PHASE.INACTIVE:
        return getUIText("vision_step_inactive", "Ready to start.");
      case CARROT_VISION_PHASE.STARTING:
        return getUIText("vision_step_starting", "Preparing camera and overlay streams.");
      case CARROT_VISION_PHASE.RTC_CONNECTING:
        return getUIText("vision_step_rtc_connecting", "Opening road camera WebRTC stream.");
      case CARROT_VISION_PHASE.TRACK_WAITING:
        return getUIText("vision_step_track_waiting", "Stream connected. Waiting for video track.");
      case CARROT_VISION_PHASE.FIRST_FRAME_WAITING:
        return getUIText("vision_step_first_frame", "Video track received. Waiting for first frame.");
      case CARROT_VISION_PHASE.READY:
        return getUIText("vision_step_ready", "Camera and overlay are live.");
      case CARROT_VISION_PHASE.RECOVERING:
        return getUIText("vision_step_recovering", "Refreshing the stream connection.");
      case CARROT_VISION_PHASE.FAILED:
        return getUIText("vision_step_failed", "Connection check failed. Retrying when available.");
      default:
        return "";
    }
  }

  function publishCarrotVisionState(detail = {}) {
    syncLegacyCarrotVisionFlags();
    window.CarrotVisionState = CARROT_VISION_STATE;
    window.dispatchEvent(new CustomEvent("carrot:visionstatechange", {
      detail: {
        state: CARROT_VISION_STATE,
        ...detail,
      },
    }));
  }

  function setCarrotVisionState(patch = {}, detail = {}) {
    const next = { ...patch };
    if (next.rtc == null) delete next.rtc;
    if (next.raw == null) delete next.raw;
    if (next.rtc && typeof next.rtc === "object") {
      Object.assign(CARROT_VISION_STATE.rtc, next.rtc);
      delete next.rtc;
    }
    if (next.raw && typeof next.raw === "object") {
      Object.assign(CARROT_VISION_STATE.raw, next.raw);
      delete next.raw;
    }
    Object.assign(CARROT_VISION_STATE, next);
    CARROT_VISION_STATE.active = Boolean(CARROT_VISION_STATE.active);
    CARROT_VISION_STATE.available = Boolean(CARROT_VISION_STATE.available);
    CARROT_VISION_STATE.updatedAtMs = Date.now();
    if (detail.silent) {
      syncLegacyCarrotVisionFlags();
      window.CarrotVisionState = CARROT_VISION_STATE;
      return;
    }
    publishCarrotVisionState(detail);
  }

  function setRtcStatus(statusText) {
    if (typeof window.rtcStatusSet === "function") {
      window.rtcStatusSet(statusText);
    }
  }

  function requestVisionRender(detail = {}) {
    if (typeof window.requestCarrotVisionRender === "function") {
      window.requestCarrotVisionRender(detail);
    }
  }

  function emitVisionChange(active) {
    if (typeof window.emitCarrotVisionChange === "function") {
      window.emitCarrotVisionChange(active);
    } else {
      window.dispatchEvent(new CustomEvent("carrot:visionchange", { detail: { active: Boolean(active) } }));
    }
  }

  function setCarrotVisionPhase(phase, detail = {}) {
    const nextPhase = CARROT_VISION_PHASE_SET.has(phase) ? phase : CARROT_VISION_PHASE.FAILED;
    const statusText = Object.prototype.hasOwnProperty.call(detail, "statusText")
      ? String(detail.statusText || "")
      : getCarrotVisionPhaseStatusText(nextPhase);
    const detailText = Object.prototype.hasOwnProperty.call(detail, "detailText")
      ? String(detail.detailText || "")
      : getCarrotVisionPhaseDetailText(nextPhase);
    const nextReason = detail.reason || nextPhase;
    if (
      CARROT_VISION_STATE.phase === nextPhase &&
      CARROT_VISION_STATE.statusText === statusText &&
      CARROT_VISION_STATE.detailText === detailText &&
      CARROT_VISION_STATE.reason === nextReason &&
      detail.rtc == null &&
      detail.raw == null
    ) {
      if (detail.updateRtcStatus !== false && statusText) setRtcStatus(statusText);
      return;
    }
    setCarrotVisionState({
      phase: nextPhase,
      statusText,
      detailText,
      reason: nextReason,
      rtc: detail.rtc || undefined,
      raw: detail.raw || undefined,
    }, { phase: nextPhase, reason: nextReason, silent: detail.silent });
    if (detail.updateRtcStatus !== false && statusText) setRtcStatus(statusText);
    if (detail.render !== false) requestVisionRender({ phase: nextPhase });
  }

  function setCarrotVisionAvailable(available, detail = {}) {
    const nextAvailable = Boolean(available);
    const wasActive = CARROT_VISION_STATE.active;
    const disabledMessage = nextAvailable ? "" : (detail.disabledMessage || getUIText("vision_unavailable_hint", "Available when DisableDM is 2."));
    const nextPhase = detail.phase || (nextAvailable
      ? (CARROT_VISION_STATE.active ? CARROT_VISION_STATE.phase : CARROT_VISION_PHASE.INACTIVE)
      : CARROT_VISION_PHASE.UNAVAILABLE);
    setCarrotVisionState({
      available: nextAvailable,
      active: nextAvailable ? CARROT_VISION_STATE.active : false,
      disabledMessage,
    }, { reason: detail.reason || "availability" });
    setCarrotVisionPhase(nextPhase, {
      reason: detail.reason || "availability",
      statusText: detail.statusText,
      detailText: detail.detailText,
      updateRtcStatus: detail.updateRtcStatus,
      render: detail.render,
    });
    if (wasActive && !CARROT_VISION_STATE.active) emitVisionChange(false);
  }

  function setCarrotVisionActive(active, detail = {}) {
    const nextActive = Boolean(active);
    const nextPhase = detail.phase || (nextActive
      ? CARROT_VISION_PHASE.STARTING
      : (CARROT_VISION_STATE.available ? CARROT_VISION_PHASE.INACTIVE : CARROT_VISION_PHASE.UNAVAILABLE));
    const changed = CARROT_VISION_STATE.active !== nextActive;
    setCarrotVisionState({
      active: nextActive,
    }, { reason: detail.reason || "active" });
    setCarrotVisionPhase(nextPhase, {
      reason: detail.reason || "active",
      statusText: detail.statusText,
      detailText: detail.detailText,
      updateRtcStatus: detail.updateRtcStatus,
      render: detail.render,
    });
    if (changed) emitVisionChange(nextActive);
  }

  function relocalizeCarrotVisionState() {
    const phase = CARROT_VISION_STATE.phase || CARROT_VISION_PHASE.UNAVAILABLE;
    const disabledMessage = CARROT_VISION_STATE.available
      ? ""
      : getUIText("vision_unavailable_hint", "Available when DisableDM is 2.");
    if (disabledMessage) CARROT_VISION_STATE.disabledMessage = disabledMessage;
    setCarrotVisionState({
      statusText: getCarrotVisionPhaseStatusText(phase),
      detailText: getCarrotVisionPhaseDetailText(phase),
      disabledMessage: CARROT_VISION_STATE.disabledMessage,
    }, { reason: "language" });
    if (CARROT_VISION_STATE.statusText) setRtcStatus(CARROT_VISION_STATE.statusText);
    requestVisionRender({ phase });
  }

  window.CarrotVisionPhase = CARROT_VISION_PHASE;
  window.CarrotVisionState = CARROT_VISION_STATE;
  window.CarrotVisionSetPhase = setCarrotVisionPhase;
  window.CarrotVisionSetState = setCarrotVisionState;
  window.CarrotVisionSetAvailable = setCarrotVisionAvailable;
  window.CarrotVisionSetActive = setCarrotVisionActive;
  window.CarrotVisionRelocalize = relocalizeCarrotVisionState;
  window.isCarrotVisionActive = isCarrotVisionActive;

  syncLegacyCarrotVisionFlags();
  window.addEventListener("carrot:languagechange", relocalizeCarrotVisionState);
})();
