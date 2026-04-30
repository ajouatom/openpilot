"use strict";

// Cross-page long-running activity state: nav badges + unload guard.

const appActivities = new Map();
let appActivitySeq = 0;

const APP_ACTIVITY_SCOPES = {
  tools: () => btnTools,
  logs: () => btnLogs,
};

function getAppActivityButton(scope) {
  const getter = APP_ACTIVITY_SCOPES[scope];
  return typeof getter === "function" ? getter() : null;
}

function getAppActivityEntries(scope = null) {
  return Array.from(appActivities.values()).filter((entry) => !scope || entry.scope === scope);
}

function hasBlockingAppActivity() {
  return getAppActivityEntries().some((entry) => entry.blockUnload !== false);
}

function syncAppActivityNav() {
  Object.keys(APP_ACTIVITY_SCOPES).forEach((scope) => {
    const button = getAppActivityButton(scope);
    if (!button) return;

    const entries = getAppActivityEntries(scope);
    const active = entries.length > 0;
    const label = entries[entries.length - 1]?.label || getUIText("working", "Working");
    if (button.classList.contains("is-working") !== active) {
      button.classList.toggle("is-working", active);
    }
    if (button.hasAttribute("aria-busy") !== active) {
      button.toggleAttribute("aria-busy", active);
    }

    if (active) {
      if (button.dataset.workBadge !== "") button.dataset.workBadge = "";
      if (button.title !== label) button.title = label;
    } else {
      if (button.hasAttribute("data-work-badge")) button.removeAttribute("data-work-badge");
      if (button.hasAttribute("title")) button.removeAttribute("title");
    }
  });
}

function notifyAppActivityChange() {
  syncAppActivityNav();
  window.dispatchEvent(new CustomEvent("carrot:activitychange", {
    detail: {
      active: appActivities.size > 0,
      blocking: hasBlockingAppActivity(),
      activities: getAppActivityEntries(),
    },
  }));
}

function beginAppActivity(scope, label = "", options = {}) {
  const id = `activity-${++appActivitySeq}`;
  appActivities.set(id, {
    id,
    scope,
    label: String(label || getUIText("working", "Working")),
    blockUnload: options.blockUnload !== false,
    startedAt: Date.now(),
  });
  notifyAppActivityChange();
  return id;
}

function endAppActivity(id) {
  if (!id || !appActivities.has(id)) return false;
  appActivities.delete(id);
  notifyAppActivityChange();
  return true;
}

function appActivityBeforeUnload(event) {
  if (!hasBlockingAppActivity()) return undefined;
  event.preventDefault();
  event.returnValue = "";
  return "";
}

window.addEventListener("beforeunload", appActivityBeforeUnload);
window.addEventListener("carrot:languagechange", syncAppActivityNav);
