"use strict";

(() => {
  const shell = document.querySelector(".logs-shell");
  const dashcamPanel = document.getElementById("logsDashcamPanel");
  const screenPanel = document.getElementById("logsScreenPanel");
  const dashcamScroller = document.getElementById("dashcamRoutes");
  const screenScroller = document.getElementById("screenrecordVideos");
  const scrollers = [dashcamScroller, screenScroller].filter(Boolean);
  if (!shell || !scrollers.length) return;

  let syncFrame = 0;

  function activeScroller() {
    if (dashcamPanel && !dashcamPanel.hidden) return dashcamScroller;
    if (screenPanel && !screenPanel.hidden) return screenScroller;
    return scrollers.find((scroller) => scroller.offsetWidth > 0) || null;
  }

  function syncScrollbarGutter() {
    syncFrame = 0;
    const scroller = activeScroller();
    if (!scroller || scroller.offsetWidth <= 0) return;
    const style = window.getComputedStyle(scroller);
    const borderWidth = (
      (Number.parseFloat(style.borderLeftWidth) || 0)
      + (Number.parseFloat(style.borderRightWidth) || 0)
    );
    const gutter = Math.max(0, Math.round(scroller.offsetWidth - scroller.clientWidth - borderWidth));
    shell.style.setProperty("--logs-scrollbar-gutter", `${gutter}px`);
  }

  function scheduleSync() {
    if (syncFrame) return;
    syncFrame = window.requestAnimationFrame(syncScrollbarGutter);
  }

  if (typeof ResizeObserver === "function") {
    const resizeObserver = new ResizeObserver(scheduleSync);
    resizeObserver.observe(shell);
    scrollers.forEach((scroller) => resizeObserver.observe(scroller));
  }

  if (typeof MutationObserver === "function") {
    const tabObserver = new MutationObserver(scheduleSync);
    if (dashcamPanel) tabObserver.observe(dashcamPanel, { attributes: true, attributeFilter: ["hidden"] });
    if (screenPanel) tabObserver.observe(screenPanel, { attributes: true, attributeFilter: ["hidden"] });
  }

  window.addEventListener("resize", scheduleSync, { passive: true });
  window.addEventListener("carrot:pagechange", scheduleSync);
  scheduleSync();
})();
