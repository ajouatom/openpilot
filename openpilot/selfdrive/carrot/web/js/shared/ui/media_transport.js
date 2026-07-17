"use strict";

function carrotMediaAutoHideDelay() {
  return window.matchMedia?.("(pointer: coarse)")?.matches ? 1500 : 1200;
}

const CARROT_MEDIA_ACTION_PATHS = Object.freeze({
  previous: "M11 10h3v16h-3zm4 8 12-8v16z",
  play: "M13 9v18l14-9z",
  pause: "M12 9h4v18h-4zm8 0h4v18h-4z",
  next: "M22 10h3v16h-3zM9 10l12 8-12 8z",
});

function setCarrotMediaActionState(button, state, options = {}) {
  if (!button) return;
  const normalizedState = Object.hasOwn(CARROT_MEDIA_ACTION_PATHS, state) ? state : "play";
  const path = button.querySelector("path");
  if (path) path.setAttribute("d", CARROT_MEDIA_ACTION_PATHS[normalizedState]);
  button.dataset.state = normalizedState;
  const label = options.label || button.dataset[`${normalizedState}Label`] || normalizedState;
  button.setAttribute("aria-label", String(label));
  button.title = String(label);
}

function createCarrotMediaActionButton(action, options = {}) {
  const normalizedAction = Object.hasOwn(CARROT_MEDIA_ACTION_PATHS, action) ? action : "play";
  const button = document.createElement("button");
  button.type = "button";
  button.className = `carrot-media-action-button carrot-media-action-button--${normalizedAction}`;
  button.disabled = Boolean(options.disabled);
  button.dataset.playLabel = String(options.playLabel || options.label || "play");
  button.dataset.pauseLabel = String(options.pauseLabel || "pause");

  const svg = document.createElementNS("http://www.w3.org/2000/svg", "svg");
  svg.setAttribute("viewBox", "0 0 36 36");
  svg.setAttribute("aria-hidden", "true");
  const path = document.createElementNS("http://www.w3.org/2000/svg", "path");
  svg.appendChild(path);
  button.appendChild(svg);
  setCarrotMediaActionState(button, normalizedAction, { label: options.label || normalizedAction });

  if (typeof options.onActivate === "function") {
    button.addEventListener("click", (event) => {
      event.stopPropagation();
      options.onActivate(event);
    });
  }
  return button;
}

function createCarrotMediaSegmentButton(direction, options = {}) {
  const normalizedDirection = direction === "previous" ? "previous" : "next";
  const button = document.createElement("button");
  button.type = "button";
  button.className = `plyr__control carrot-media-icon-button carrot-media-icon-button--${normalizedDirection}`;
  button.setAttribute("aria-label", String(options.label || normalizedDirection));
  button.title = String(options.label || normalizedDirection);
  button.disabled = Boolean(options.disabled);

  const svg = document.createElementNS("http://www.w3.org/2000/svg", "svg");
  svg.setAttribute("viewBox", "0 0 36 36");
  svg.setAttribute("aria-hidden", "true");
  const path = document.createElementNS("http://www.w3.org/2000/svg", "path");
  path.setAttribute("d", normalizedDirection === "previous"
    ? "M11 10h3v16h-3zm4 8 12-8v16z"
    : "M22 10h3v16h-3zM9 10l12 8-12 8z");
  svg.appendChild(path);
  button.appendChild(svg);
  if (typeof options.onActivate === "function") {
    button.addEventListener("click", (event) => {
      event.stopPropagation();
      options.onActivate(event);
    });
  }
  return button;
}

const CARROT_REPLAY_EVENT_ICON_PATHS = Object.freeze({
  control: "M12 3a9 9 0 1 0 0 18 9 9 0 0 0 0-18Zm0 6.5a2.5 2.5 0 1 0 0 5 2.5 2.5 0 0 0 0-5ZM3.8 10.5h5.9m4.6 0h5.9M12 14.5V21M9.8 13.7 6.2 18m8-4.3 3.6 4.3",
  driver: "M12 4.5a3.5 3.5 0 1 0 0 7 3.5 3.5 0 0 0 0-7ZM5 20c.7-4.3 3.1-6.5 7-6.5s6.3 2.2 7 6.5",
  vehicle: "M4 17v-6l2.2-5h11.6l2.2 5v6m-16-5h16M7.5 17.5a1.5 1.5 0 1 0 0 3 1.5 1.5 0 0 0 0-3Zm9 0a1.5 1.5 0 1 0 0 3 1.5 1.5 0 0 0 0-3Z",
  carrot: "M6 18a2 2 0 1 0 0 4 2 2 0 0 0 0-4Zm12-16a2 2 0 1 0 0 4 2 2 0 0 0 0-4ZM8 20c8 0 1-16 8-16",
  warning: "M12 3 2.8 20h18.4L12 3Zm0 6v5m0 3v.1",
  turn: "M5 20v-4c0-5.5 3.5-9 9-9h5m-3.5-3L19 7l-3.5 3",
  nav: "M12 21s6-5.4 6-11a6 6 0 1 0-12 0c0 5.6 6 11 6 11Zm0-13.5a2.5 2.5 0 1 0 0 5 2.5 2.5 0 0 0 0-5Z",
  event: "M12 3a9 9 0 1 0 0 18 9 9 0 0 0 0-18Zm0 5v5m0 3v.1",
});

function createCarrotReplayEventIcon(category) {
  const normalizedCategory = Object.hasOwn(CARROT_REPLAY_EVENT_ICON_PATHS, category) ? category : "event";
  const svg = document.createElementNS("http://www.w3.org/2000/svg", "svg");
  svg.setAttribute("viewBox", "0 0 24 24");
  svg.setAttribute("aria-hidden", "true");
  svg.setAttribute("focusable", "false");
  svg.setAttribute("fill", "none");
  svg.setAttribute("stroke", "currentColor");
  svg.setAttribute("stroke-width", "1.8");
  svg.setAttribute("stroke-linecap", "round");
  svg.setAttribute("stroke-linejoin", "round");
  svg.dataset.eventIcon = normalizedCategory;
  const path = document.createElementNS("http://www.w3.org/2000/svg", "path");
  path.setAttribute("d", CARROT_REPLAY_EVENT_ICON_PATHS[normalizedCategory]);
  svg.appendChild(path);
  return svg;
}

window.CarrotMediaTransport = Object.freeze({
  autoHideDelay: carrotMediaAutoHideDelay,
  createActionButton: createCarrotMediaActionButton,
  createEventIcon: createCarrotReplayEventIcon,
  createSegmentButton: createCarrotMediaSegmentButton,
  setActionState: setCarrotMediaActionState,
  create(options = {}) {
    const media = options.media;
    const root = options.root;
    const playButton = options.playButton;
    const speedButton = options.speedButton;
    const seek = options.seek;
    const currentTime = options.currentTime;
    const duration = options.duration;
    const status = options.status;
    const closeButton = options.closeButton;
    const filmstrip = options.filmstrip;
    const fallbackThumbnail = options.fallbackThumbnail;
    const scrubPreview = options.scrubPreview;
    const scrubImage = options.scrubImage;
    const scrubTime = options.scrubTime;
    const scrubEventInfo = options.scrubEventInfo;
    const scrubEventIcon = options.scrubEventIcon;
    const scrubEventTitle = options.scrubEventTitle;
    const scrubEventMeta = options.scrubEventMeta;
    const scrubEvents = options.scrubEvents;
    const scrubEventsCanvas = options.scrubEventsCanvas;
    const requestedSpeeds = Array.isArray(options.speeds)
      ? options.speeds.filter((value) => Number.isFinite(value) && value > 0)
      : [];
    const speeds = requestedSpeeds.length ? requestedSpeeds : [0.5, 1, 2];
    let speedIndex = Math.max(0, speeds.indexOf(1));
    let seekFrame = null;
    let previewToken = 0;
    let previewMedia = null;
    let previewFrames = [];
    let scrubHideTimer = null;
    let previewPinnedTimer = null;
    let scrubbing = false;
    let scrubStartClientX = null;
    let previewPinned = false;
    let previewGenerating = false;
    let previewEventWindow = null;
    let previewSeekTimer = null;
    let previewSeekToken = 0;
    let previewCaptureCanvas = null;
    let previewVisible = false;

    if (!media || !root || !playButton || !speedButton || !seek) return null;

    const translate = (key, fallback) => (
      typeof options.text === "function" ? options.text(key, fallback) : fallback
    );
    const canInteract = () => (
      typeof options.canInteract === "function" ? Boolean(options.canInteract()) : true
    );
    const mediaDuration = () => {
      const value = typeof options.getDuration === "function"
        ? Number(options.getDuration())
        : Number(media.duration);
      return Number.isFinite(value) && value > 0 ? value : 0;
    };
    const formatTime = (seconds) => {
      const value = Math.max(0, Number(seconds) || 0);
      const minutes = Math.floor(value / 60);
      const rest = Math.floor(value % 60);
      return `${minutes}:${String(rest).padStart(2, "0")}`;
    };
    const previewIdleDelay = () => carrotMediaAutoHideDelay() + 800;

    function notifyInteraction() {
      options.onInteract?.();
    }

    function disposePreviewMedia() {
      previewSeekToken += 1;
      if (previewSeekTimer != null) window.clearTimeout(previewSeekTimer);
      previewSeekTimer = null;
      if (!previewMedia) return;
      try { previewMedia.pause(); } catch {}
      previewMedia.removeAttribute("src");
      try { previewMedia.load(); } catch {}
      previewMedia = null;
    }

    function setPreviewPinned(pinned, releaseDelayMs = 0) {
      if (previewPinnedTimer != null) window.clearTimeout(previewPinnedTimer);
      previewPinnedTimer = null;
      previewPinned = Boolean(pinned);
      scrubPreview?.classList.toggle("is-pinned", previewPinned);
      if (previewPinned && releaseDelayMs > 0) {
        previewPinnedTimer = window.setTimeout(() => {
          previewPinnedTimer = null;
          previewPinned = false;
          scrubPreview?.classList.remove("is-pinned");
          hideScrubPreview(0, true);
        }, releaseDelayMs);
      }
    }

    function setPreviewVisible(visible) {
      const next = Boolean(visible);
      if (previewVisible === next) return;
      previewVisible = next;
      options.onPreviewVisibilityChange?.(next);
    }

    function hideScrubPreview(delayMs = 0, force = false) {
      if (!scrubPreview) return;
      if (force) setPreviewPinned(false);
      if (previewPinned && !force) return;
      if (scrubHideTimer != null) window.clearTimeout(scrubHideTimer);
      scrubHideTimer = null;
      const hide = () => {
        scrubHideTimer = null;
        previewSeekToken += 1;
        if (previewSeekTimer != null) window.clearTimeout(previewSeekTimer);
        previewSeekTimer = null;
        scrubPreview.hidden = true;
        scrubPreview.setAttribute("aria-hidden", "true");
        if (scrubEventInfo) scrubEventInfo.hidden = true;
        setPreviewVisible(false);
      };
      if (delayMs > 0) scrubHideTimer = window.setTimeout(hide, delayMs);
      else hide();
    }

    function previewFrameAt(seconds) {
      if (!previewFrames.length) return null;
      let selected = previewFrames[0];
      let distance = Math.abs(selected.time - seconds);
      for (let index = 1; index < previewFrames.length; index += 1) {
        const candidateDistance = Math.abs(previewFrames[index].time - seconds);
        if (candidateDistance >= distance) continue;
        selected = previewFrames[index];
        distance = candidateDistance;
      }
      return selected;
    }

    function categoryColor(category) {
      const variables = {
        control: "--carrot-replay-insights-control",
        driver: "--carrot-replay-insights-driver",
        vehicle: "--carrot-replay-insights-vehicle",
        carrot: "--carrot-replay-insights-carrot",
        warning: "--carrot-replay-insights-warning",
        turn: "--carrot-replay-insights-turn",
        nav: "--carrot-replay-insights-nav",
      };
      const property = variables[category] || "--carrot-replay-insights-event";
      return window.getComputedStyle?.(scrubEvents || root).getPropertyValue(property).trim() || "#ffad66";
    }

    function renderScrubEvents(centerSeconds) {
      if (!scrubEvents || !scrubEventsCanvas || typeof options.getTimelineEvents !== "function") return;
      const eventWindow = options.getTimelineEvents(centerSeconds, 6);
      if (!eventWindow || !(eventWindow.endSeconds > eventWindow.startSeconds)) {
        previewEventWindow = null;
        scrubEvents.hidden = true;
        return;
      }
      previewEventWindow = eventWindow;
      scrubEvents.hidden = false;
      const width = Math.max(1, scrubEvents.clientWidth || scrubPreview?.clientWidth || 180);
      const height = Math.max(1, scrubEvents.clientHeight || 18);
      const ratio = Math.max(1, Math.min(3, Number(window.devicePixelRatio) || 1));
      scrubEventsCanvas.width = Math.round(width * ratio);
      scrubEventsCanvas.height = Math.round(height * ratio);
      const context = scrubEventsCanvas.getContext("2d", { alpha: true });
      if (!context) return;
      context.setTransform(ratio, 0, 0, ratio, 0, 0);
      context.clearRect(0, 0, width, height);
      context.strokeStyle = "rgba(255, 255, 255, .24)";
      context.lineWidth = 1;
      context.beginPath();
      context.moveTo(0, height - 4.5);
      context.lineTo(width, height - 4.5);
      context.stroke();
      const duration = eventWindow.endSeconds - eventWindow.startSeconds;
      for (const event of eventWindow.events || []) {
        const x = Math.max(1, Math.min(width - 1, (event.timeSeconds - eventWindow.startSeconds) / duration * width));
        context.strokeStyle = categoryColor(event.category);
        context.lineWidth = event.category === "warning" ? 3 : 2;
        context.beginPath();
        context.moveTo(x, event.category === "warning" ? 2 : 6);
        context.lineTo(x, height - 3);
        context.stroke();
      }
      const centerRatio = Math.max(0, Math.min(1, (centerSeconds - eventWindow.startSeconds) / duration));
      scrubEvents.style.setProperty("--carrot-replay-scrub-event-cursor", `${(centerRatio * 100).toFixed(2)}%`);
    }

    function renderScrubEventInfo(seconds) {
      if (!scrubEventInfo || typeof options.getTimelineEvent !== "function") return;
      const event = options.getTimelineEvent(seconds, 3);
      if (!event) {
        scrubEventInfo.hidden = true;
        return;
      }
      scrubEventInfo.dataset.category = event.category || "event";
      if (scrubEventIcon) scrubEventIcon.replaceChildren(createCarrotReplayEventIcon(event.category));
      if (scrubEventTitle) scrubEventTitle.textContent = event.title || "";
      if (scrubEventMeta) scrubEventMeta.textContent = event.meta || "";
      scrubEventInfo.hidden = false;
    }

    function timelinePosition(event) {
      const rect = seek.getBoundingClientRect();
      if (rect.width <= 0) return null;
      const pointerX = Number(event?.clientX);
      const ratio = Number.isFinite(pointerX)
        ? Math.max(0, Math.min(1, (pointerX - rect.left) / rect.width))
        : Math.max(0, Math.min(1, Number(seek.value || 0) / 1000));
      return { rect, ratio, seconds: ratio * mediaDuration() };
    }

    function captureVideoFrame(target, canvas = null) {
      if (!target || Number(target.videoWidth) <= 0 || Number(target.videoHeight) <= 0) return "";
      const output = canvas || document.createElement("canvas");
      output.width = 192;
      output.height = 108;
      const context = output.getContext("2d", { alpha: false });
      if (!context) return "";
      const videoWidth = Math.max(1, Number(target.videoWidth));
      const videoHeight = Math.max(1, Number(target.videoHeight));
      const scale = Math.max(output.width / videoWidth, output.height / videoHeight);
      const sourceWidth = output.width / scale;
      const sourceHeight = output.height / scale;
      const sourceX = Math.max(0, (videoWidth - sourceWidth) / 2);
      const sourceY = Math.max(0, (videoHeight - sourceHeight) / 2);
      context.drawImage(target, sourceX, sourceY, sourceWidth, sourceHeight, 0, 0, output.width, output.height);
      return output.toDataURL("image/jpeg", 0.62);
    }

    function requestExactPreview(seconds) {
      if (!previewMedia || previewGenerating || !scrubImage) return;
      const request = ++previewSeekToken;
      if (previewSeekTimer != null) window.clearTimeout(previewSeekTimer);
      previewSeekTimer = window.setTimeout(async () => {
        previewSeekTimer = null;
        const target = Math.max(0, Math.min(Math.max(0, mediaDuration() - 0.03), Number(seconds) || 0));
        try {
          if (Math.abs(Number(previewMedia.currentTime || 0) - target) > 0.025) {
            previewMedia.currentTime = target;
            await waitForMediaEvent(previewMedia, "seeked", 1400, previewToken);
          }
          if (request !== previewSeekToken || !scrubPreview || scrubPreview.hidden) return;
          previewCaptureCanvas ||= document.createElement("canvas");
          const image = captureVideoFrame(previewMedia, previewCaptureCanvas);
          if (image) scrubImage.src = image;
        } catch {}
      }, 90);
    }

    function showScrubPreviewAt(seconds, renderEvents = true) {
      if (!scrubPreview || !scrubImage || !scrubTime) return;
      const frame = previewFrameAt(seconds);
      if (frame?.src) scrubImage.src = frame.src;
      else if (fallbackThumbnail?.src) scrubImage.src = fallbackThumbnail.src;
      else return;
      scrubTime.textContent = formatTime(seconds);
      scrubPreview.hidden = false;
      scrubPreview.setAttribute("aria-hidden", "false");
      setPreviewVisible(true);
      renderScrubEventInfo(seconds);
      if (renderEvents) renderScrubEvents(seconds);
      requestExactPreview(seconds);
      if (previewPinned) setPreviewPinned(true, carrotMediaAutoHideDelay());
      else hideScrubPreview(previewIdleDelay());
      notifyInteraction();
    }

    function showScrubPreview(event) {
      if (!scrubPreview || !scrubImage || !scrubTime || seek.disabled) return;
      const position = timelinePosition(event);
      if (!position) return;
      const { rect, ratio, seconds } = position;
      if (event?.pointerType === "touch") setPreviewPinned(true, carrotMediaAutoHideDelay());
      showScrubPreviewAt(seconds);
      const halfWidth = Math.max(58, (scrubPreview.offsetWidth || 136) / 2);
      const left = Math.max(halfWidth, Math.min(rect.width - halfWidth, ratio * rect.width));
      root.style.setProperty("--carrot-replay-preview-left", `${left.toFixed(1)}px`);
    }

    function captureFallbackFrame() {
      if (!fallbackThumbnail || fallbackThumbnail.src || previewFrames.length || previewGenerating) return false;
      try {
        const image = captureVideoFrame(media);
        if (!image) return false;
        setFallbackThumbnail(image);
        return true;
      } catch {
        return false;
      }
    }

    function waitForMediaEvent(target, eventName, timeoutMs, token) {
      return new Promise((resolve, reject) => {
        let timer = null;
        const cleanup = () => {
          if (timer != null) window.clearTimeout(timer);
          target.removeEventListener(eventName, onReady);
          target.removeEventListener("error", onError);
        };
        const onReady = () => {
          cleanup();
          if (token !== previewToken) {
            const error = new Error("Preview canceled");
            error.name = "AbortError";
            reject(error);
          } else resolve();
        };
        const onError = () => {
          cleanup();
          reject(new Error("Preview video failed"));
        };
        target.addEventListener(eventName, onReady, { once: true });
        target.addEventListener("error", onError, { once: true });
        timer = window.setTimeout(() => {
          cleanup();
          reject(new Error(`Preview ${eventName} timeout`));
        }, timeoutMs);
      });
    }

    function renderFilmstrip(frames) {
      if (!filmstrip || !frames.length) return;
      const fragment = document.createDocumentFragment();
      for (const frame of frames) {
        const image = document.createElement("img");
        image.src = frame.src;
        image.alt = "";
        image.draggable = false;
        fragment.appendChild(image);
      }
      filmstrip.replaceChildren(fragment);
      previewFrames = frames;
    }

    function setFallbackThumbnail(url) {
      previewFrames = [];
      if (!fallbackThumbnail || !filmstrip) return;
      filmstrip.replaceChildren(fallbackThumbnail);
      fallbackThumbnail.classList.remove("is-ready");
      if (url) fallbackThumbnail.src = String(url);
      else fallbackThumbnail.removeAttribute("src");
    }

    async function generateFilmstrip(source, options = {}) {
      const src = typeof source === "object" ? String(source?.url || "") : String(source || "");
      if (!src || !filmstrip) return [];
      const token = ++previewToken;
      disposePreviewMedia();
      const preview = document.createElement("video");
      previewMedia = preview;
      previewGenerating = true;
      preview.muted = true;
      preview.playsInline = true;
      preview.preload = "auto";
      preview.src = src;
      let completed = false;

      try {
        if (source && typeof source === "object" && source.ready) await source.ready;
        if (preview.readyState < 1) await waitForMediaEvent(preview, "loadedmetadata", 8000, token);
        if (token !== previewToken) return [];
        const total = Number(options.duration) > 0 ? Number(options.duration) : Number(preview.duration);
        if (!Number.isFinite(total) || total <= 0) return [];
        const durationCount = Math.ceil(total / 2);
        const widthCount = Math.ceil((filmstrip.clientWidth || 560) / 32);
        const count = Math.max(12, Math.min(32, Math.max(durationCount, widthCount)));
        const canvas = document.createElement("canvas");
        canvas.width = 192;
        canvas.height = 108;
        const frames = [];

        for (let index = 0; index < count; index += 1) {
          if (token !== previewToken) return [];
          const time = Math.min(Math.max(0, total - 0.05), ((index + 0.5) / count) * total);
          if (Math.abs(Number(preview.currentTime || 0) - time) > 0.03) {
            preview.currentTime = time;
            await waitForMediaEvent(preview, "seeked", 3200, token);
          }
          const image = captureVideoFrame(preview, canvas);
          if (!image) continue;
          frames.push({ time, src: image });
          previewFrames = [...frames];
          if (frames.length === 1 || frames.length % 4 === 0 || index === count - 1) renderFilmstrip(frames);
        }

        if (token === previewToken) renderFilmstrip(frames);
        completed = frames.length > 0;
        return frames;
      } catch (error) {
        if (error?.name !== "AbortError") console.debug("[replay] preview filmstrip unavailable", error);
        return [];
      } finally {
        if (token === previewToken) previewGenerating = false;
        if (previewMedia === preview && (!options.keepAlive || !completed)) disposePreviewMedia();
      }
    }

    function sync() {
      const total = mediaDuration();
      const elapsed = Math.min(total || Infinity, Math.max(0, Number(media.currentTime) || 0));
      const progress = total > 0 ? Math.max(0, Math.min(1, elapsed / total)) : 0;
      const interactive = canInteract();
      root.style.setProperty("--carrot-replay-progress", `${(progress * 100).toFixed(2)}%`);
      if (currentTime) currentTime.textContent = formatTime(elapsed);
      if (duration) duration.textContent = formatTime(total);
      seek.disabled = !interactive || total <= 0;
      if (!seek.matches(":active")) {
        seek.value = String(total > 0 ? Math.round((elapsed / total) * 1000) : 0);
      }

      const paused = media.paused || media.ended;
      const playState = paused ? "play" : "pause";
      const playLabel = paused
        ? translate("replay_play", "Play replay")
        : translate("replay_pause", "Pause replay");
      playButton.disabled = !interactive;
      setCarrotMediaActionState(playButton, playState, { label: playLabel });
      playButton.setAttribute("aria-pressed", String(!paused));

      const currentRateIndex = speeds.indexOf(Number(media.playbackRate));
      if (currentRateIndex >= 0) speedIndex = currentRateIndex;
      speedButton.disabled = !interactive;
      speedButton.textContent = `${speeds[speedIndex]}×`;
      speedButton.setAttribute("aria-label", translate("replay_speed", "Replay speed"));
      if (status && typeof options.getStatus === "function") {
        status.textContent = String(options.getStatus() || "");
      }
    }

    function syncLabels() {
      if (closeButton) {
        closeButton.textContent = translate("replay_close_button", "Close");
        closeButton.setAttribute("aria-label", translate("replay_close", "Close replay"));
      }
      seek.setAttribute("aria-label", translate("replay_position", "Replay position"));
      scrubEvents?.setAttribute("aria-label", translate("replay_nearby_events", "Nearby replay events"));
      sync();
    }

    function setRate(rate = 1) {
      const requested = Number(rate);
      const index = speeds.indexOf(requested);
      speedIndex = index >= 0 ? index : Math.max(0, speeds.indexOf(1));
      media.playbackRate = speeds[speedIndex];
      sync();
    }

    function reset() {
      if (seekFrame != null) window.cancelAnimationFrame(seekFrame);
      seekFrame = null;
      speedIndex = Math.max(0, speeds.indexOf(1));
      root.style.setProperty("--carrot-replay-progress", "0%");
      seek.value = "0";
      previewToken += 1;
      previewFrames = [];
      previewEventWindow = null;
      previewGenerating = false;
      scrubbing = false;
      disposePreviewMedia();
      setPreviewPinned(false);
      hideScrubPreview(0, true);
      if (scrubEvents) scrubEvents.hidden = true;
      if (scrubEventInfo) scrubEventInfo.hidden = true;
      setFallbackThumbnail("");
      sync();
    }

    function scrubEventPosition(event) {
      if (!scrubEvents || !previewEventWindow) return null;
      const rect = scrubEvents.getBoundingClientRect();
      if (rect.width <= 0) return null;
      const ratio = Math.max(0, Math.min(1, (Number(event?.clientX) - rect.left) / rect.width));
      return {
        rect,
        ratio,
        seconds: previewEventWindow.startSeconds
          + ratio * (previewEventWindow.endSeconds - previewEventWindow.startSeconds),
      };
    }

    function showLocalPreview(event) {
      const position = scrubEventPosition(event);
      if (!position) return null;
      scrubEvents.style.setProperty("--carrot-replay-scrub-event-cursor", `${(position.ratio * 100).toFixed(2)}%`);
      showScrubPreviewAt(position.seconds, false);
      return position;
    }

    function commitLocalPreview(event) {
      const position = showLocalPreview(event);
      if (!position || !previewEventWindow) return;
      let nearest = null;
      let distance = Infinity;
      for (const candidate of previewEventWindow.events || []) {
        const candidateDistance = Math.abs(Number(candidate.timeSeconds) - position.seconds);
        if (candidateDistance >= distance) continue;
        nearest = candidate;
        distance = candidateDistance;
      }
      const tolerancePx = event?.pointerType === "touch" ? 16 : 9;
      const toleranceSeconds = (previewEventWindow.endSeconds - previewEventWindow.startSeconds)
        * tolerancePx / Math.max(1, position.rect.width);
      if (nearest && distance <= toleranceSeconds) options.onTimelineEventSelect?.(nearest.id);
      else options.onTimelinePreviewSeek?.(position.seconds);
      setPreviewPinned(false);
      hideScrubPreview(0, true);
    }

    const onRootClick = (event) => event.stopPropagation();
    const onClose = () => options.onClose?.();
    const onPlay = () => {
      if (!canInteract()) return;
      notifyInteraction();
      if (media.ended) media.currentTime = 0;
      if (media.paused) media.play().catch(() => sync());
      else media.pause();
    };
    const onSpeed = () => {
      if (!canInteract()) return;
      notifyInteraction();
      speedIndex = (speedIndex + 1) % speeds.length;
      media.playbackRate = speeds[speedIndex];
      sync();
    };
    const onSeek = () => {
      if (!canInteract()) return;
      showScrubPreview();
      if (seekFrame != null) window.cancelAnimationFrame(seekFrame);
      seekFrame = window.requestAnimationFrame(() => {
        seekFrame = null;
        const total = mediaDuration();
        if (total <= 0) return;
        media.currentTime = (Number(seek.value || 0) / 1000) * total;
        options.onSeek?.(media.currentTime);
        sync();
      });
    };
    const onScrubStart = (event) => {
      if (!canInteract()) return;
      if (!scrubbing) {
        scrubbing = true;
        scrubStartClientX = Number.isFinite(Number(event?.clientX)) ? Number(event.clientX) : null;
        options.onScrubStart?.();
      }
      showScrubPreview(event);
      try { seek.setPointerCapture?.(event.pointerId); } catch {}
    };
    const onScrubEnd = (event, previewDelayMs = 0) => {
      const wasScrubbing = scrubbing;
      scrubbing = false;
      try {
        if (seek.hasPointerCapture?.(event?.pointerId)) seek.releasePointerCapture(event.pointerId);
      } catch {}
      if (wasScrubbing) {
        options.onScrubEnd?.();
        const endClientX = Number(event?.clientX);
        const movement = scrubStartClientX == null || !Number.isFinite(endClientX)
          ? 0
          : Math.abs(endClientX - scrubStartClientX);
        const position = timelinePosition(event);
        if (event?.type === "pointerup" && movement <= 8 && position) {
          options.onTimelineCommit?.(position.seconds, {
            pointerType: String(event.pointerType || "mouse"),
            ratio: position.ratio,
          });
        }
      }
      scrubStartClientX = null;
      if (event?.pointerType === "touch" && !scrubPreview?.hidden) {
        setPreviewPinned(true, carrotMediaAutoHideDelay());
      }
      hideScrubPreview(previewDelayMs);
    };
    const onMediaChange = () => sync();

    root.addEventListener("click", onRootClick);
    closeButton?.addEventListener("click", onClose);
    playButton.addEventListener("click", onPlay);
    speedButton.addEventListener("click", onSpeed);
    seek.addEventListener("input", onSeek);
    seek.addEventListener("pointerenter", showScrubPreview);
    seek.addEventListener("pointermove", showScrubPreview);
    seek.addEventListener("pointerdown", onScrubStart);
    seek.addEventListener("pointerleave", () => hideScrubPreview(240));
    seek.addEventListener("pointerup", (event) => onScrubEnd(event, 650));
    seek.addEventListener("pointercancel", (event) => onScrubEnd(event));
    seek.addEventListener("lostpointercapture", (event) => onScrubEnd(event, 650));
    scrubPreview?.addEventListener("pointerenter", () => {
      if (previewPinned) setPreviewPinned(true, carrotMediaAutoHideDelay());
      else hideScrubPreview(previewIdleDelay());
    });
    scrubPreview?.addEventListener("pointermove", () => {
      if (previewPinned) setPreviewPinned(true, carrotMediaAutoHideDelay());
      else hideScrubPreview(previewIdleDelay());
    });
    scrubPreview?.addEventListener("pointerleave", () => hideScrubPreview(240));
    scrubPreview?.addEventListener("pointerdown", (event) => {
      event.stopPropagation();
      if (event.pointerType === "touch") setPreviewPinned(true, carrotMediaAutoHideDelay());
    });
    scrubPreview?.addEventListener("click", (event) => event.stopPropagation());
    scrubEvents?.addEventListener("pointerdown", (event) => {
      event.preventDefault();
      event.stopPropagation();
      setPreviewPinned(true, carrotMediaAutoHideDelay());
      try { scrubEvents.setPointerCapture?.(event.pointerId); } catch {}
      showLocalPreview(event);
    });
    scrubEvents?.addEventListener("pointermove", (event) => {
      if (event.pointerType !== "mouse" && !scrubEvents.hasPointerCapture?.(event.pointerId)) return;
      if (scrubEvents.hasPointerCapture?.(event.pointerId)) {
        setPreviewPinned(true, carrotMediaAutoHideDelay());
      }
      showLocalPreview(event);
    });
    scrubEvents?.addEventListener("pointerup", (event) => {
      event.preventDefault();
      event.stopPropagation();
      commitLocalPreview(event);
      try {
        if (scrubEvents.hasPointerCapture?.(event.pointerId)) scrubEvents.releasePointerCapture(event.pointerId);
      } catch {}
    });
    document.addEventListener("pointerdown", (event) => {
      if (!previewPinned || scrubPreview?.contains(event.target) || event.target === seek) return;
      setPreviewPinned(false);
      hideScrubPreview(0, true);
    });
    window.addEventListener("blur", () => hideScrubPreview(0, true));
    document.addEventListener("visibilitychange", () => {
      if (document.hidden) hideScrubPreview(0, true);
    });
    ["loadedmetadata", "durationchange", "play", "pause", "ended", "ratechange", "timeupdate", "seeking", "seeked"].forEach((eventName) => {
      media.addEventListener(eventName, onMediaChange);
    });

    return Object.freeze({
      generateFilmstrip,
      captureFallbackFrame,
      reset,
      setFallbackThumbnail,
      setRate,
      sync,
      syncLabels,
    });
  },
});
