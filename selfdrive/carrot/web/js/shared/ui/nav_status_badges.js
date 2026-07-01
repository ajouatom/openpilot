(function initNavStatusBadges() {
  "use strict";

  const homeButton = document.getElementById("btnHome");
  const badgeGroup = document.getElementById("navHomeStatusBadges");
  const recordBadge = document.getElementById("navRecordStatusBadge");
  const liveBadge = document.getElementById("navYouTubeStatusBadge");
  if (!homeButton || !badgeGroup || !recordBadge || !liveBadge) return;

  const POLL_INTERVAL_MS = 2000;
  const STALE_AFTER_MS = 8000;
  let youtubeLive = false;
  let lastSuccessAt = 0;
  let pollPending = false;
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

  function startPolling() {
    if (pollTimer !== null) return;
    pollYouTubeStatus();
    pollTimer = window.setInterval(pollYouTubeStatus, POLL_INTERVAL_MS);
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
  window.addEventListener("pageshow", startPolling);
  window.addEventListener("pagehide", stopPolling);

  syncBadges();
  startPolling();
})();
