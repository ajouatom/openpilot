"use strict";

import { loadWebSettings } from "./state.js";
import {
  findWebSettingsItem,
  getWebSettingValue,
  renderWebSettingsDialogHtml,
  setActiveWebSettingsGroup,
  setWebSettingValue,
  syncWebSettingsDialog,
  webSettingsText,
} from "./render.js";

// Web settings dialog controller. Schema, state, and rendering live beside
// this module; this file only opens the dialog and wires
// row events to the setters.

let webSettingsViewportSyncBound = false;
let lastAutoUpdateErrorEventId = "";

function ensureActiveWebSettingsGroupVisible() {
  const nav = document.querySelector(".app-dialog--web-settings .web-settings-nav");
  const active = nav?.querySelector(".web-settings-nav__item.is-active");
  if (!nav || !active) return false;
  const navRect = nav.getBoundingClientRect();
  const activeRect = active.getBoundingClientRect();
  let left = 0;
  let top = 0;
  if (activeRect.left < navRect.left) left = activeRect.left - navRect.left;
  else if (activeRect.right > navRect.right) left = activeRect.right - navRect.right;
  if (activeRect.top < navRect.top) top = activeRect.top - navRect.top;
  else if (activeRect.bottom > navRect.bottom) top = activeRect.bottom - navRect.bottom;
  if (Math.abs(left) < 1 && Math.abs(top) < 1) return false;
  if (typeof nav.scrollBy === "function") nav.scrollBy({ left, top, behavior: "auto" });
  else {
    nav.scrollLeft += left;
    nav.scrollTop += top;
  }
  return true;
}

function scheduleActiveWebSettingsGroupVisibility() {
  window.requestAnimationFrame(ensureActiveWebSettingsGroupVisible);
}

function bindWebSettingsDialogEvents() {
  document.querySelectorAll("[data-web-settings-group]").forEach((button) => {
    if (button.dataset.webSettingsBound === "1") return;
    button.dataset.webSettingsBound = "1";
    button.addEventListener("click", () => {
      setActiveWebSettingsGroup(button.dataset.webSettingsGroup || "general");
      syncWebSettingsDialog();
      scheduleActiveWebSettingsGroupVisibility();
    });
  });

  document.querySelectorAll("[data-web-setting]").forEach((row) => {
    if (row.dataset.webSettingBound === "1") return;
    row.dataset.webSettingBound = "1";
    const item = findWebSettingsItem(row.dataset.webSetting);
    const input = row.querySelector(".c-switch__input");
    const select = row.querySelector(".web-settings-select");
    if (!item) return;
    if (input) {
      input.addEventListener("change", () => {
        setWebSettingValue(item, input.checked)
          .then(() => {
            if (item.id === "auto_update_git_pull" && input.checked) {
              globalThis.CarrotToolsRuntime?.refreshGitPullStatus?.({ force: true, ttlMs: 0 }).catch(() => {});
            }
          })
          .catch((err) => {
            input.checked = Boolean(getWebSettingValue(item));
            if (typeof showAppToast === "function") {
              showAppToast(err?.message || String(err), { tone: "error" });
            }
          });
      });
    }
    if (select) {
      select.addEventListener("change", () => {
        setWebSettingValue(item, select.value).catch((err) => {
          select.value = String(getWebSettingValue(item));
          if (typeof showAppToast === "function") {
            showAppToast(err?.message || String(err), { tone: "error" });
          }
        });
      });
    }
  });
  globalThis.WebSettingsComponents?.bind?.(document);
  scheduleActiveWebSettingsGroupVisibility();
  if (!webSettingsViewportSyncBound) {
    webSettingsViewportSyncBound = true;
    for (const eventName of ["resize", "orientationchange", "carrot:viewportlayout"]) {
      window.addEventListener(eventName, scheduleActiveWebSettingsGroupVisibility, { passive: true });
    }
  }
}

async function openWebSettingsDialog() {
  // Refresh on every open so the preview and Drive workspace both consume the
  // current device-backed layout, including changes made from another client.
  await loadWebSettings(true).catch(() => {});
  const dialogPromise = appAlert("", {
    title: webSettingsText("web_settings"),
    html: true,
    messageHtml: renderWebSettingsDialogHtml(),
  });
  if (typeof appDialog !== "undefined" && appDialog) {
    appDialog.classList.add("app-dialog--web-settings");
  }
  window.setTimeout(bindWebSettingsDialogEvents, 0);
  dialogPromise.finally(() => {
    if (typeof appDialog !== "undefined" && appDialog) {
      appDialog.classList.remove("app-dialog--web-settings");
    }
  });
  return dialogPromise;
}

function handleWebAutoUpdateStatus(status = {}) {
  // Auto update now runs device-side in carrot_server (services/auto_update.py),
  // so it works without the web open and won't double-pull. Surface persistent
  // device-side failures once per event when a browser is available again.
  const update = status?.auto_update || {};
  const updateStatus = String(update.status || "");
  if (!["error", "reboot_blocked"].includes(updateStatus)) return;

  const eventId = String(update.event_id || `${updateStatus}:${update.target_head || ""}:${update.error_code || ""}`);
  if (!eventId || eventId === lastAutoUpdateErrorEventId || typeof showAppToast !== "function") return;
  lastAutoUpdateErrorEventId = eventId;

  const title = updateStatus === "reboot_blocked"
    ? webSettingsText("web_auto_update_reboot_blocked")
    : webSettingsText("web_auto_update_failed");
  const detail = String(update.error || update.error_code || "").trim();
  showAppToast(detail ? `${title}: ${detail}` : title, { tone: "error", duration: 8000 });
}

window.openWebSettingsDialog = openWebSettingsDialog;
window.handleWebAutoUpdateStatus = handleWebAutoUpdateStatus;

export { openWebSettingsDialog, handleWebAutoUpdateStatus };
