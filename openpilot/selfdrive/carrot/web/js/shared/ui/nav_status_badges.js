(function initNavStatusBadges() {
  "use strict";

  const homeButton = document.getElementById("btnHome");
  const terminalButton = document.getElementById("btnTerminal");
  const badgeGroup = document.getElementById("navHomeStatusBadges");
  const terminalBadgeGroup = document.getElementById("navTerminalStatusBadges");
  const recordBadge = document.getElementById("navRecordStatusBadge");
  const remoteBadge = document.getElementById("navRemoteStatusBadge");
  if (!homeButton || !terminalButton || !badgeGroup || !terminalBadgeGroup || !recordBadge || !remoteBadge) return;

  const POLL_INTERVAL_MS = 2000;
  const STALE_AFTER_MS = 8000;
  let remoteSupport = false;
  let lastSupportSuccessAt = 0;
  let supportPollPending = false;
  let pollTimer = null;

  function syncBadges() {
    const recording = homeButton.classList.contains("recording")
      && homeButton.dataset.recordBadge === "REC";
    recordBadge.hidden = !recording;
    badgeGroup.classList.toggle("is-visible", recording);

    terminalButton.classList.toggle("remote-support", remoteSupport);
    remoteBadge.hidden = !remoteSupport;
    terminalBadgeGroup.classList.toggle("is-visible", remoteSupport);
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
    pollSupportStatus();
    pollTimer = window.setInterval(pollSupportStatus, POLL_INTERVAL_MS);
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
