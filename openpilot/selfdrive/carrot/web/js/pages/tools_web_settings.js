"use strict";

// Web settings dialog controller. Schema lives in tools_web_settings_schema.js,
// state/API in tools_web_settings_state.js, rendering in
// tools_web_settings_render.js — this module only opens the dialog and wires
// row events to the setters.

function bindWebSettingsDialogEvents() {
  document.querySelectorAll("[data-web-settings-group]").forEach((button) => {
    if (button.dataset.webSettingsBound === "1") return;
    button.dataset.webSettingsBound = "1";
    button.addEventListener("click", () => {
      activeWebSettingsGroup = button.dataset.webSettingsGroup || "general";
      syncWebSettingsDialog();
    });
  });

  document.querySelectorAll("[data-web-setting]").forEach((row) => {
    if (row.dataset.webSettingBound === "1") return;
    row.dataset.webSettingBound = "1";
    const item = WEB_SETTINGS_GROUPS.flatMap((group) => group.items).find((entry) => entry.id === row.dataset.webSetting);
    const input = row.querySelector(".c-switch__input");
    const select = row.querySelector(".web-settings-select");
    if (!item) return;
    if (input) {
      input.addEventListener("change", () => {
        setWebSettingValue(item, input.checked)
          .then(() => {
            if (item.id === "auto_update_git_pull" && input.checked && typeof refreshGitPullStatus === "function") {
              refreshGitPullStatus({ force: true, ttlMs: 0 }).catch(() => {});
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
}

async function openWebSettingsDialog() {
  await loadWebSettings().catch(() => {});
  const dialogPromise = appAlert("", {
    title: getUIText("web_settings", "Web Settings"),
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
  // so it works without the web open and won't double-pull. Intentionally a
  // no-op here; kept because tools.js still calls it on git-status updates.
  void status;
}

window.openWebSettingsDialog = openWebSettingsDialog;
window.handleWebAutoUpdateStatus = handleWebAutoUpdateStatus;
