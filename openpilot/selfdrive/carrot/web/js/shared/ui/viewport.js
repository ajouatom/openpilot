"use strict";

let appViewportMetricsBound = false;
const driveHudCardEl = document.getElementById("driveHudCard");
let driveHudLayoutObserversBound = false;
let driveHudLayoutRaf = 0;


function updateAppViewportMetrics() {
  const vv = window.visualViewport;
  const height = Math.max(320, Math.round(vv?.height || window.innerHeight || 0));
  const top = Math.max(0, Math.round(vv?.offsetTop || 0));
  const width = Math.max(320, Math.round(vv?.width || window.innerWidth || 0));
  document.documentElement.style.setProperty("--app-vv-height", `${height}px`);
  document.documentElement.style.setProperty("--app-vv-top", `${top}px`);
  document.documentElement.style.setProperty("--app-vv-width", `${width}px`);

  const topbarEl = document.querySelector(".topbar");
  let navLeftGap = 0;
  let navBottomGap = 0;
  if (topbarEl) {
    const styles = window.getComputedStyle(topbarEl);
    if (styles.display !== "none" && styles.visibility !== "hidden") {
      const rect = topbarEl.getBoundingClientRect();
      const rectWidth = Math.max(0, Math.round(rect.width));
      const rectHeight = Math.max(0, Math.round(rect.height));
      const isRailLayout =
        rectWidth > 0 &&
        rectHeight > 0 &&
        rectHeight >= Math.max(Math.round(rectWidth * 1.25), Math.round(height * 0.5)) &&
        rectWidth <= Math.max(160, Math.round(width * 0.35));

      if (isRailLayout) {
        navLeftGap = rectWidth;
      } else if (rectHeight > 0) {
        const visibleTop = Math.max(0, Math.round(rect.top));
        const visibleBottom = Math.min(height, Math.round(rect.bottom));
        navBottomGap = Math.max(0, visibleBottom - visibleTop);
      }
    }
  }

  document.documentElement.style.setProperty("--app-nav-left-gap", `${navLeftGap}px`);
  document.documentElement.style.setProperty("--app-nav-bottom-gap", `${navBottomGap}px`);
}

function bindAppViewportObservers() {
  if (appViewportMetricsBound) return;
  appViewportMetricsBound = true;

  const handleLayout = () => requestAnimationFrame(updateAppViewportMetrics);
  updateAppViewportMetrics();
  window.addEventListener("resize", handleLayout, { passive: true });
  window.addEventListener("orientationchange", handleLayout, { passive: true });
  document.addEventListener("fullscreenchange", handleLayout);
  document.addEventListener("webkitfullscreenchange", handleLayout);
  if (window.visualViewport) {
    window.visualViewport.addEventListener("resize", handleLayout, { passive: true });
    window.visualViewport.addEventListener("scroll", handleLayout, { passive: true });
  }
}

bindAppViewportObservers();


function syncDriveHudLayout() {
  driveHudLayoutRaf = 0;
  if (!driveHudCardEl || !window.DrivingHud) return;
  window.DrivingHud.relayout();
}

function scheduleDriveHudLayout() {
  if (driveHudLayoutRaf) return;
  driveHudLayoutRaf = requestAnimationFrame(syncDriveHudLayout);
}

function bindDriveHudLayoutObservers() {
  if (driveHudLayoutObserversBound) return;
  driveHudLayoutObserversBound = true;

  const handleLayout = () => scheduleDriveHudLayout();
  window.addEventListener("resize", handleLayout, { passive: true });
  window.addEventListener("orientationchange", handleLayout, { passive: true });
  window.addEventListener("carrot:pagechange", handleLayout);
  if (window.visualViewport) {
    window.visualViewport.addEventListener("resize", handleLayout, { passive: true });
    window.visualViewport.addEventListener("scroll", handleLayout, { passive: true });
  }

  scheduleDriveHudLayout();
}

bindDriveHudLayoutObservers();
