(function initNavStatusBadges() {
  "use strict";

  const homeButton = document.getElementById("btnHome");
  const terminalButton = document.getElementById("btnTerminal");
  const badgeGroup = document.getElementById("navHomeStatusBadges");
  const terminalBadgeGroup = document.getElementById("navTerminalStatusBadges");
  const recordBadge = document.getElementById("navRecordStatusBadge");
  const liveBadge = document.getElementById("navYouTubeStatusBadge");
  const remoteBadge = document.getElementById("navRemoteStatusBadge");
  if (!homeButton || !terminalButton || !badgeGroup || !terminalBadgeGroup || !recordBadge || !liveBadge || !remoteBadge) return;

  const POLL_INTERVAL_MS = 2000;
  const STALE_AFTER_MS = 8000;
  let youtubeLive = false;
  let remoteSupport = false;
  let lastSuccessAt = 0;
  let lastSupportSuccessAt = 0;
  let pollPending = false;
  let supportPollPending = false;
  let pollTimer = null;

  function syncBadges() {
    if (homeButton.classList.contains("youtube-live") !== youtubeLive) {
      homeButton.classList.toggle("youtube-live", youtubeLive);
    }
    const recording = homeButton.classList.contains("recording")
      && homeButton.dataset.recordBadge === "REC";
    recordBadge.hidden = !recording;
    liveBadge.hidden = !youtubeLive;
    badgeGroup.classList.toggle("is-visible", recording || youtubeLive);

    terminalButton.classList.toggle("remote-support", remoteSupport);
    remoteBadge.hidden = !remoteSupport;
    terminalBadgeGroup.classList.toggle("is-visible", remoteSupport);
  }

  async function pollYouTubeStatus() {
    if (pollPending || document.hidden) return;
    pollPending = true;
    try {
      const status = await getJson("/api/youtube_live/status");
      lastSuccessAt = Date.now();
      youtubeLive = status?.state === "live" && status?.running === true;
      syncBadges();
    } catch (_) {
      if (!lastSuccessAt || Date.now() - lastSuccessAt >= STALE_AFTER_MS) {
        youtubeLive = false;
        syncBadges();
      }
    } finally {
      pollPending = false;
    }
  }

  async function pollSupportStatus() {
    if (supportPollPending || document.hidden) return;
    supportPollPending = true;
    try {
      const status = await getJson("/api/support_terminal/status");
      lastSupportSuccessAt = Date.now();
      remoteSupport = Boolean(status?.active);
      syncBadges();
    } catch (_) {
      if (!lastSupportSuccessAt || Date.now() - lastSupportSuccessAt >= STALE_AFTER_MS) {
        remoteSupport = false;
        syncBadges();
      }
    } finally {
      supportPollPending = false;
    }
  }

  function startPolling() {
    if (pollTimer !== null) return;
    pollYouTubeStatus();
    pollSupportStatus();
    pollTimer = window.setInterval(() => {
      pollYouTubeStatus();
      pollSupportStatus();
    }, POLL_INTERVAL_MS);
  }

  function stopPolling() {
    if (pollTimer === null) return;
    window.clearInterval(pollTimer);
    pollTimer = null;
  }

  new MutationObserver(syncBadges).observe(homeButton, {
    attributes: true,
    attributeFilter: ["class", "data-record-badge"],
  });

  document.addEventListener("visibilitychange", () => {
    if (document.hidden) stopPolling();
    else startPolling();
  });
  window.addEventListener("online", pollYouTubeStatus);
  window.addEventListener("online", pollSupportStatus);
  window.addEventListener("carrot:support-terminal-status", (event) => {
    remoteSupport = Boolean(event?.detail?.active);
    lastSupportSuccessAt = Date.now();
    syncBadges();
  });
  window.addEventListener("pageshow", startPolling);
  window.addEventListener("pagehide", stopPolling);

  syncBadges();
  startPolling();
})();
