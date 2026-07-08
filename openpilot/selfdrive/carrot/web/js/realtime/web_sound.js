"use strict";

(function () {
  const STORAGE_KEY = "carrot.webSound.enabled.v1";
  const VOLUME_STORAGE_KEY = "carrot.webSound.volume.v1";
  const RECONNECT_MS = 1500;
  const AudioContextClass = window.AudioContext || window.webkitAudioContext;
  const button = document.getElementById("btnWebSound");

  const SOUND_MAP = Object.freeze({
    1: { file: "engage.wav", once: true, engageVolume: true },
    2: { file: "disengage.wav", once: true, engageVolume: true },
    3: { file: "refuse.wav", once: true },
    4: { file: "warning_soft.wav" },
    5: { file: "warning_immediate.wav" },
    6: { file: "prompt.wav", once: true },
    7: { file: "prompt.wav" },
    8: { file: "prompt_distracted.wav" },
    9: { file: "audio_turn.wav" },
    10: { file: "tici_engaged.wav" },
    11: { file: "tici_disengaged.wav" },
    12: { file: "traffic_sign_green.wav" },
    13: { file: "traffic_sign_changed.wav" },
    14: { file: "audio_lane_change.wav" },
    15: { file: "audio_stopping.wav" },
    16: { file: "audio_auto_hold.wav" },
    17: { file: "audio_engage.wav" },
    18: { file: "audio_disengage.wav" },
    19: { file: "audio_traffic_error.wav" },
    20: { file: "audio_car_watchout.wav" },
    21: { file: "audio_speed_down.wav" },
    22: { file: "audio_stopstop.wav" },
    23: { file: "reverse_gear.wav", once: true, engageVolume: true },
    24: { file: "audio_1.wav" },
    25: { file: "audio_2.wav" },
    26: { file: "audio_3.wav" },
    27: { file: "audio_4.wav" },
    28: { file: "audio_5.wav" },
    29: { file: "audio_6.wav" },
    30: { file: "audio_7.wav" },
    31: { file: "audio_8.wav" },
    32: { file: "audio_9.wav" },
    33: { file: "audio_10.wav" },
  });

  let enabled = loadEnabled();
  let webVolume = loadWebVolume();
  let context = null;
  let socket = null;
  let reconnectTimer = null;
  let current = null;
  let pending = null;
  let playRequest = 0;
  let lastAlert = null;
  let lastCountdown = null;
  let volume = 1;
  let engageVolume = 1;
  let tizi = false;
  let activeSoundDirectory = bootstrapSoundDirectory();
  const bufferCache = new Map();
  // Decoded buffers keyed by "dir/file", populated as loadBuffer resolves. Lets
  // playAlert take a synchronous fast path (play in the same tick as the WS
  // message) when the context is already running and the sound is decoded.
  const readyBuffers = new Map();

  function loadEnabled() {
    try {
      return localStorage.getItem(STORAGE_KEY) === "1";
    } catch (_) {
      return false;
    }
  }

  function saveEnabled(value) {
    try {
      localStorage.setItem(STORAGE_KEY, value ? "1" : "0");
    } catch (_) {}
  }

  function loadWebVolume() {
    try {
      const stored = localStorage.getItem(VOLUME_STORAGE_KEY);
      if (stored == null || stored === "") return 1;
      const value = Number(stored);
      return Number.isFinite(value) ? Math.max(0, Math.min(1, value)) : 1;
    } catch (_) {
      return 1;
    }
  }

  function setWebVolume(value) {
    webVolume = Math.max(0, Math.min(1, Number(value) || 0));
    try { localStorage.setItem(VOLUME_STORAGE_KEY, String(webVolume)); } catch (_) {}
    if (current?.gain && context) {
      // Independent of device Settings > Sound volume — Web slider only.
      current.gain.gain.setValueAtTime(Math.max(0, Math.min(1, webVolume)), context.currentTime);
    }
    return webVolume;
  }

  function bootstrapSoundDirectory() {
    const configured = String(window.__CARROT_BOOTSTRAP__?.soundLanguage || "auto").trim();
    const device = String(window.__CARROT_BOOTSTRAP__?.deviceLanguage || "en").trim();
    let normalized = (configured && configured.toLowerCase() !== "auto" ? configured : device)
      .replaceAll("_", "-")
      .toLowerCase();
    if (normalized.startsWith("main-")) normalized = normalized.slice(5);
    if (normalized === "ko" || normalized.startsWith("ko-")) return "sounds";
    if (normalized === "zh-chs" || normalized === "zh-hans" || normalized.startsWith("zh")) return "sounds_chs";
    return "sounds_eng";
  }

  function soundDirectory() {
    return activeSoundDirectory;
  }

  function setButtonState() {
    if (!button) return;
    button.classList.toggle("is-active", enabled);
    button.textContent = getUIText("web_sound_button", "Sound");
    const label = enabled
      ? getUIText("web_sound_enabled", "Web sound on")
      : getUIText("web_sound_disabled", "Web sound off");
    button.setAttribute("aria-label", label);
    button.title = label;
  }

  function ensureContext() {
    if (!context && AudioContextClass) {
      // "interactive" (also the spec default) asks the browser for the lowest
      // reliable output latency — important since the alert must land as close to
      // the device's own sound as possible. Older prefixed constructors may reject
      // the options object, so fall back to the bare form.
      try {
        context = new AudioContextClass({ latencyHint: "interactive" });
      } catch (_) {
        context = new AudioContextClass();
      }
    }
    return context;
  }

  async function unlockAudio() {
    const audioContext = ensureContext();
    if (!audioContext) throw new Error(getUIText("web_sound_unsupported", "Audio playback is not supported by this browser."));
    if (audioContext.state === "suspended") await audioContext.resume();
    return audioContext.state === "running";
  }

  async function loadBuffer(file) {
    const audioContext = ensureContext();
    if (!audioContext) throw new Error("AudioContext unavailable");
    const key = `${soundDirectory()}/${file}`;
    if (!bufferCache.has(key)) {
      bufferCache.set(key, (async () => {
        const response = await fetch(`/sound-assets/${encodeURIComponent(soundDirectory())}/${encodeURIComponent(file)}`, { cache: "force-cache" });
        if (!response.ok) throw new Error(`sound asset HTTP ${response.status}`);
        const decoded = await audioContext.decodeAudioData(await response.arrayBuffer());
        readyBuffers.set(key, decoded);
        return decoded;
      })().catch((error) => {
        bufferCache.delete(key);
        readyBuffers.delete(key);
        throw error;
      }));
    }
    return bufferCache.get(key);
  }

  function preloadSounds() {
    const files = new Set(Object.values(SOUND_MAP).map((spec) => spec.file));
    if (tizi) files.add("engage_tizi.wav");
    if (tizi) files.add("disengage_tizi.wav");
    return Promise.allSettled(Array.from(files, (file) => loadBuffer(file)));
  }

  function stopCurrent(immediate = true) {
    if (immediate) {
      playRequest += 1;
      pending = null;
    } else if (pending) {
      pending.forceOneShot = true;
    }
    if (!current) return;
    if (!immediate) {
      if (current.loop) {
        current.source.loop = false;
        current.loop = false;
        current.released = true;
      }
      return;
    }
    try { current.source.stop(); } catch (_) {}
    current = null;
  }

  function resolveSoundFile(alertNum, spec) {
    if (tizi && alertNum === 1) return "engage_tizi.wav";
    if (tizi && alertNum === 2) return "disengage_tizi.wav";
    return spec.file;
  }

  function startBuffer(alertNum, buffer, resolvedOneShot) {
    stopCurrent(true);
    const source = context.createBufferSource();
    const gain = context.createGain();
    source.buffer = buffer;
    source.loop = !resolvedOneShot;
    // Web playback volume is intentionally independent of the device's
    // Settings > Sound volume (SoundVolumeAdjust / SoundVolumeAdjustEngage): only
    // the in-dialog Web volume slider scales what the browser plays.
    gain.gain.value = Math.max(0, Math.min(1, webVolume));
    source.connect(gain);
    gain.connect(context.destination);
    current = { source, gain, alert: alertNum, loop: source.loop, released: false };
    source.onended = () => {
      if (current?.source === source) current = null;
    };
    source.start();
    return current;
  }

  async function playAlert(alert, options = {}) {
    const preview = Boolean(options.preview);
    if (!enabled && !preview) return;
    const alertNum = Number(alert);
    const spec = SOUND_MAP[alertNum];
    if (!spec) return;
    const oneShot = Boolean(options.oneShot || spec.once);
    const request = ++playRequest;
    pending = { request, alert: alertNum, oneShot, forceOneShot: false };
    const file = resolveSoundFile(alertNum, spec);

    // Fast path: warm context + already-decoded buffer → start in this same tick
    // with no await hops, so the alert fires the moment the WS message lands
    // (the extra ~50ms server poll was the real lag; this keeps the client tight).
    if (context?.state === "running") {
      const readyBuffer = readyBuffers.get(`${soundDirectory()}/${file}`);
      if (readyBuffer) {
        try {
          const playback = startBuffer(alertNum, readyBuffer, oneShot);
          if (pending?.request === request) pending = null;
          return playback;
        } catch (error) {
          console.warn("[web sound] fast-path failed", error);
        }
      }
    }

    try {
      await unlockAudio();
      const buffer = await loadBuffer(file);
      if ((!enabled && !preview) || request !== playRequest) return;
      const resolvedOneShot = oneShot || Boolean(pending?.request === request && pending.forceOneShot);
      return startBuffer(alertNum, buffer, resolvedOneShot);
    } catch (error) {
      console.warn("[web sound] playback failed", error);
      return null;
    } finally {
      if (pending?.request === request) pending = null;
    }
  }

  // Deduplicated sound list for the dialog's sample picker: one entry per file,
  // labelled by the file stem (engage.wav → "engage"). Language-independent.
  function sampleEntries() {
    const seen = new Set();
    const entries = [];
    Object.keys(SOUND_MAP)
      .map(Number)
      .sort((a, b) => a - b)
      .forEach((num) => {
        const file = SOUND_MAP[num]?.file;
        if (!file || seen.has(file)) return;
        seen.add(file);
        entries.push({ alert: num, label: file.replace(/\.wav$/i, "") });
      });
    return entries;
  }

  function countdownAlert(countdown) {
    if (countdown === 0) return 11;
    if (countdown === 11) return 8;
    if (countdown >= 1 && countdown <= 10) return 23 + countdown;
    return 0;
  }

  function handleSoundState(state) {
    if (!enabled || state?.type !== "soundState") return;
    volume = Number.isFinite(Number(state.volume)) ? Number(state.volume) : 1;
    engageVolume = Number.isFinite(Number(state.engageVolume)) ? Number(state.engageVolume) : 1;
    tizi = Boolean(state.tizi);
    const previousSoundDirectory = activeSoundDirectory;
    if (["sounds", "sounds_eng", "sounds_chs"].includes(state.soundDirectory)) {
      activeSoundDirectory = state.soundDirectory;
    }
    const directoryChanged = previousSoundDirectory !== activeSoundDirectory;

    const alert = Number(state.alert) || 0;
    const countdown = Number.isFinite(Number(state.countdown)) ? Number(state.countdown) : 100;
    const alertChanged = alert !== lastAlert;
    if (alertChanged) {
      if (alert > 0) {
        const sameCurrentAlert = current?.alert === alert && !current.released;
        const samePendingAlert = pending?.alert === alert;
        if (!sameCurrentAlert && !samePendingAlert) void playAlert(alert);
      } else if (lastAlert > 0) {
        stopCurrent(false);
      }
      lastAlert = alert;
    }

    if (alert === 0 && countdown !== lastCountdown) {
      const derivedAlert = countdownAlert(countdown);
      const initialZero = lastCountdown == null && countdown === 0;
      if (derivedAlert > 0 && !initialZero) void playAlert(derivedAlert, { oneShot: true });
      lastCountdown = countdown;
    }
    if (directoryChanged && context?.state === "running") {
      void preloadSounds();
      if (!alertChanged && current?.alert > 0) {
        void playAlert(current.alert, { oneShot: !current.loop });
      }
    }
  }

  function socketUrl() {
    const protocol = location.protocol === "https:" ? "wss:" : "ws:";
    return `${protocol}//${location.host}/ws/web_sound`;
  }

  function clearReconnect() {
    if (!reconnectTimer) return;
    clearTimeout(reconnectTimer);
    reconnectTimer = null;
  }

  function scheduleReconnect() {
    clearReconnect();
    if (!enabled) return;
    reconnectTimer = window.setTimeout(connect, RECONNECT_MS);
  }

  function connect() {
    clearReconnect();
    if (!enabled || socket || !location.host) return;
    lastAlert = null;
    lastCountdown = null;
    const ws = new WebSocket(socketUrl());
    socket = ws;
    ws.onmessage = (event) => {
      try { handleSoundState(JSON.parse(event.data)); } catch (_) {}
    };
    ws.onerror = () => {
      try { ws.close(); } catch (_) {}
    };
    ws.onclose = () => {
      if (socket === ws) socket = null;
      scheduleReconnect();
    };
  }

  function disconnect() {
    clearReconnect();
    const ws = socket;
    socket = null;
    if (ws) {
      ws.onclose = null;
      try { ws.close(); } catch (_) {}
    }
    lastAlert = null;
    lastCountdown = null;
  }

  async function setEnabled(value, fromGesture = false) {
    enabled = Boolean(value);
    saveEnabled(enabled);
    setButtonState();
    if (!enabled) {
      disconnect();
      stopCurrent(true);
      return false;
    }
    if (fromGesture) {
      await unlockAudio();
      void preloadSounds();
    }
    connect();
    return true;
  }

  function dialogHtml() {
    return `
      <div class="web-sound-dialog">
        <p class="web-sound-dialog__description">${getUIText(
          "web_sound_description",
          "This feature plays driving alerts on the connected phone or browser for clone devices that cannot play sound.",
        )}</p>
        <label class="web-sound-dialog__toggle">
          <span>${getUIText("web_sound_toggle", "Play sounds in this browser")}</span>
          <span class="c-switch c-switch--sm">
            <input class="c-switch__input" type="checkbox" data-web-sound-toggle ${enabled ? "checked" : ""} />
            <span class="c-switch__track" aria-hidden="true"></span>
          </span>
        </label>
        <label class="web-sound-dialog__volume">
          <span class="web-sound-dialog__volume-head">
            <span>${getUIText("web_sound_volume", "Web volume")}</span>
            <output data-web-sound-volume-value>${Math.round(webVolume * 100)}%</output>
          </span>
          <input class="web-sound-dialog__volume-slider" type="range" min="0" max="100" step="5" value="${Math.round(webVolume * 100)}" data-web-sound-volume />
        </label>
        <div class="web-sound-dialog__sample">
          <span class="web-sound-dialog__sample-head">${getUIText("web_sound_sample", "Play sample")}</span>
          <div class="web-sound-dialog__sample-grid" role="listbox" aria-label="${getUIText("web_sound_sample", "Play sample")}">
            ${sampleEntries().map((entry, index) => `<button type="button" class="web-sound-dialog__sample-option${index === 0 ? " is-current" : ""}" role="option" aria-selected="${index === 0 ? "true" : "false"}" data-web-sound-sample-option data-alert="${entry.alert}">${entry.label}</button>`).join("")}
          </div>
          <div class="web-sound-dialog__sample-actions">
            <button type="button" class="btn web-sound-dialog__sample-play" data-web-sound-sample-play>
              <span class="web-sound-dialog__sample-play-icon" aria-hidden="true">▶</span>${getUIText("web_sound_sample_play", "Play")}
            </button>
          </div>
        </div>
      </div>`;
  }

  function openDialog() {
    const dialogPromise = appAlert("", {
      title: getUIText("web_sound_title", "Web sound"),
      html: true,
      messageHtml: dialogHtml(),
      confirmLabel: getUIText("close", "Close"),
    });
    if (typeof appDialog !== "undefined" && appDialog) appDialog.classList.add("app-dialog--web-sound");
    window.setTimeout(() => {
      const input = document.querySelector("[data-web-sound-toggle]");
      const volumeInput = document.querySelector("[data-web-sound-volume]");
      const volumeValue = document.querySelector("[data-web-sound-volume-value]");
      input?.addEventListener("change", () => {
        setEnabled(input.checked, true).catch((error) => {
          input.checked = false;
          void setEnabled(false);
          if (typeof showAppToast === "function") showAppToast(error?.message || String(error), { tone: "error" });
        });
      });
      volumeInput?.addEventListener("input", () => {
        const percent = Math.max(0, Math.min(100, Number(volumeInput.value) || 0));
        setWebVolume(percent / 100);
        if (volumeValue) volumeValue.textContent = `${percent}%`;
      });
      const sampleOptions = Array.from(document.querySelectorAll("[data-web-sound-sample-option]"));
      const samplePlay = document.querySelector("[data-web-sound-sample-play]");
      let selectedSampleAlert = sampleOptions.length ? Number(sampleOptions[0].dataset.alert) || 0 : 0;
      sampleOptions.forEach((option) => {
        option.addEventListener("click", () => {
          // Selection only — the sample auditions when Play is pressed, not here.
          selectedSampleAlert = Number(option.dataset.alert) || 0;
          sampleOptions.forEach((other) => {
            const isSelected = other === option;
            other.classList.toggle("is-current", isSelected);
            other.setAttribute("aria-selected", isSelected ? "true" : "false");
          });
        });
      });
      samplePlay?.addEventListener("click", () => {
        if (!selectedSampleAlert) return;
        // preview:true so samples audition even when the live web-sound toggle is
        // off; the click itself is the gesture that unlocks the AudioContext.
        Promise.resolve(playAlert(selectedSampleAlert, { oneShot: true, preview: true })).then((playback) => {
          if (!playback && typeof showAppToast === "function") {
            showAppToast(getUIText("web_sound_sample_failed", "Could not play sample"), { tone: "error" });
          }
        });
      });
    }, 0);
    dialogPromise.finally(() => {
      if (typeof appDialog !== "undefined" && appDialog) appDialog.classList.remove("app-dialog--web-sound");
    });
  }

  async function testAlert(alert = 1) {
    if (!enabled) throw new Error(getUIText("web_sound_disabled", "Web sound off"));
    const value = Number(alert);
    if (!SOUND_MAP[value]) throw new Error(`Unknown AudibleAlert: ${alert}`);
    await unlockAudio();
    const playback = await playAlert(value, { oneShot: true });
    if (!playback) throw new Error(`Failed to play AudibleAlert: ${alert}`);
    if (playback?.source) {
      await new Promise((resolve) => {
        const timeout = window.setTimeout(resolve, Math.max(250, playback.source.buffer.duration * 1000 + 150));
        playback.source.addEventListener("ended", () => {
          clearTimeout(timeout);
          resolve();
        }, { once: true });
      });
    }
    return { alert: value, file: SOUND_MAP[value].file, soundDirectory: soundDirectory() };
  }

  async function testCountdown(countdown = 5) {
    const value = Number(countdown);
    const alert = countdownAlert(value);
    if (!alert) throw new Error(`Unsupported countdown: ${countdown}`);
    return testAlert(alert);
  }

  function status() {
    return {
      enabled,
      audioState: context?.state || "not-created",
      socketState: socket ? socket.readyState : WebSocket.CLOSED,
      connected: socket?.readyState === WebSocket.OPEN,
      currentAlert: current?.alert ?? null,
      pendingAlert: pending?.alert ?? null,
      soundDirectory: soundDirectory(),
      volume,
      engageVolume,
      webVolume,
      // Estimated browser audio output latency (ms) — the residual client-side lag
      // once the server poll is tightened; useful to verify realtime tuning.
      outputLatencyMs: context && Number.isFinite(context.outputLatency) ? Math.round(context.outputLatency * 1000) : null,
      baseLatencyMs: context && Number.isFinite(context.baseLatency) ? Math.round(context.baseLatency * 1000) : null,
    };
  }

  button?.addEventListener("click", openDialog);
  document.addEventListener("pointerdown", () => {
    if (!enabled || context?.state === "running") return;
    void unlockAudio()
      .then(() => {
        void preloadSounds();
        if (!current && lastAlert > 0) void playAlert(lastAlert);
      })
      .catch(() => {});
  }, { capture: true });
  window.addEventListener("carrot:languagechange", setButtonState);
  window.addEventListener("online", () => { if (enabled) connect(); });
  window.addEventListener("beforeunload", disconnect);

  setButtonState();
  if (enabled) connect();

  window.CarrotWebSound = Object.freeze({
    isEnabled: () => enabled,
    setEnabled,
    openDialog,
    status,
    setVolume: setWebVolume,
    stop: () => {
      stopCurrent(true);
      return status();
    },
    test: testAlert,
    testCountdown,
    preview: (alert) => playAlert(alert, { oneShot: true, preview: true }),
  });
})();
