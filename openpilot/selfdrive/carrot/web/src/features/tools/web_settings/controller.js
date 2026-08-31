"use strict";

import { WEB_SETTINGS_GROUPS } from "./schema.js";
import { getWebSettingByKey, loadWebSettings } from "./state.js";
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
    const textInput = row.querySelector(".web-settings-input");
    if (textInput) {
      textInput.addEventListener("change", () => {
        setWebSettingValue(item, textInput.value)
          .then(() => {
            textInput.value = String(getWebSettingValue(item) ?? "");
          })
          .catch((err) => {
            textInput.value = String(getWebSettingValue(item) ?? "");
            if (typeof showAppToast === "function") {
              showAppToast(err?.message || String(err), { tone: "error" });
            }
          });
      });
      textInput.addEventListener("keydown", (ev) => {
        if (ev.key === "Enter") textInput.blur();
      });
    }
  });

  document.querySelectorAll("[data-web-setting-action]").forEach((row) => {
    if (row.dataset.webSettingActionBound === "1") return;
    row.dataset.webSettingActionBound = "1";
    const button = row.querySelector(".web-settings-action");
    if (!button) return;
    if (row.dataset.webSettingAction === "toss_connection_test") {
      button.addEventListener("click", async () => {
        button.disabled = true;
        button.dataset.busy = "1";
        try {
          await runTossConnectionTest();
        } finally {
          delete button.dataset.busy;
          syncWebSettingsActionStates();
        }
      });
    }
  });

  syncWebSettingsActionStates();
  globalThis.WebSettingsComponents?.bind?.(document);
  scheduleActiveWebSettingsGroupVisibility();
  if (!webSettingsViewportSyncBound) {
    webSettingsViewportSyncBound = true;
    for (const eventName of ["resize", "orientationchange", "carrot:viewportlayout"]) {
      window.addEventListener(eventName, scheduleActiveWebSettingsGroupVisibility, { passive: true });
    }
  }
}

function syncWebSettingsActionStates() {
  document.querySelectorAll("[data-web-setting-action]").forEach((row) => {
    const item = WEB_SETTINGS_GROUPS.flatMap((group) => group.items).find((entry) => entry.id === row.dataset.webSettingAction);
    if (!item || typeof item.enabledWhen !== "function") return;
    const button = row.querySelector(".web-settings-action");
    if (!button || button.dataset.busy === "1") return;
    const enabled = Boolean(item.enabledWhen());
    button.disabled = !enabled;
    button.title = enabled ? "" : getUIText(item.disabledHintKey, item.defaultDisabledHint || "");
  });
}

window.addEventListener("carrot:websettingschange", syncWebSettingsActionStates);

/* ── Toss connection test popup ─────────────────────────── */
const TOSS_TEST_STEPS = [
  { id: "config", labelKey: "web_toss_test_step_config", defaultLabel: "Check settings" },
  { id: "connect", labelKey: "web_toss_test_step_connect", defaultLabel: "Contact server" },
  { id: "auth", labelKey: "web_toss_test_step_auth", defaultLabel: "Verify token & status" },
];

function closeTossTestPopup() {
  document.getElementById("tossTestPop")?.remove();
}

function openTossTestPopup() {
  closeTossTestPopup();
  const title = getUIText("web_toss_test_title", "Toss server connection test");
  const pop = document.createElement("div");
  pop.id = "tossTestPop";
  pop.className = "toss-test-pop";
  pop.innerHTML = `
    <div class="toss-test-pop__card" role="dialog" aria-live="polite" aria-label="${escapeHtml(title)}">
      <div class="toss-test-pop__title">${escapeHtml(title)}</div>
      <ul class="toss-test-pop__steps">
        ${TOSS_TEST_STEPS.map((step) => `
          <li class="toss-test-step" data-toss-step="${step.id}" data-state="pending">
            <span class="toss-test-step__icon" aria-hidden="true"></span>
            <span class="toss-test-step__label">${escapeHtml(getUIText(step.labelKey, step.defaultLabel))}</span>
            <span class="toss-test-step__note"></span>
          </li>`).join("")}
      </ul>
      <div class="toss-test-pop__result" hidden></div>
      <button type="button" class="toss-test-pop__close">${escapeHtml(getUIText("web_toss_test_close", "Close"))}</button>
    </div>`;
  pop.querySelector(".toss-test-pop__close").addEventListener("click", closeTossTestPopup);
  pop.addEventListener("click", (ev) => {
    if (ev.target === pop && pop.classList.contains("is-done")) closeTossTestPopup();
  });
  document.body.appendChild(pop);
  return pop;
}

function setTossTestStep(pop, stepId, state, note = "") {
  const row = pop.querySelector(`[data-toss-step="${stepId}"]`);
  if (!row) return;
  row.dataset.state = state; // pending | running | ok | fail | skip
  const icon = row.querySelector(".toss-test-step__icon");
  if (icon) icon.textContent = state === "ok" ? "✓" : state === "fail" ? "✕" : state === "skip" ? "–" : "";
  const noteEl = row.querySelector(".toss-test-step__note");
  if (noteEl) noteEl.textContent = note;
}

function finishTossTest(pop, ok, message) {
  const result = pop.querySelector(".toss-test-pop__result");
  if (result) {
    result.hidden = false;
    result.textContent = message;
    result.classList.toggle("is-ok", ok);
    result.classList.toggle("is-fail", !ok);
  }
  pop.classList.add("is-done");
}

async function runTossConnectionTest() {
  const pop = openTossTestPopup();

  setTossTestStep(pop, "config", "running");
  const url = String(getWebSettingByKey("toss_upload_url", "") || "").trim();
  const token = String(getWebSettingByKey("toss_upload_token", "") || "").trim();
  if (!url || !token) {
    const reason = !url
      ? getUIText("web_toss_test_no_url", "Toss server URL is not set")
      : getUIText("web_toss_test_no_token", "Access token is not set");
    setTossTestStep(pop, "config", "fail", reason);
    setTossTestStep(pop, "connect", "skip");
    setTossTestStep(pop, "auth", "skip");
    finishTossTest(pop, false, getUIText("web_toss_connection_failed", "Toss server connection failed"));
    return;
  }
  setTossTestStep(pop, "config", "ok", url.replace(/^https?:\/\//i, ""));

  setTossTestStep(pop, "connect", "running");
  try {
    const payload = await postJson("/api/dashcam/upload/test", {});
    if (!pop.isConnected) return;
    const elapsed = Number(payload?.elapsed_ms);
    const statusNote = `HTTP ${payload?.status ?? 200}${Number.isFinite(elapsed) ? ` · ${elapsed}ms` : ""}`;
    setTossTestStep(pop, "connect", "ok");
    setTossTestStep(pop, "auth", "ok", statusNote);
    finishTossTest(pop, true, getUIText("web_toss_connection_ok", "Toss server connection OK"));
  } catch (err) {
    if (!pop.isConnected) return;
    const remoteStatus = Number(err?.payload?.status);
    if (Number.isFinite(remoteStatus) && remoteStatus > 0) {
      // The toss server answered over HTTPS, so the connection itself is fine —
      // the failure is the token or the server-side health state.
      setTossTestStep(pop, "connect", "ok");
      const authFailed = remoteStatus === 401 || remoteStatus === 403;
      setTossTestStep(pop, "auth", "fail", authFailed
        ? getUIText("web_toss_test_auth_failed", "Authentication failed — check the token")
        : `HTTP ${remoteStatus}`);
    } else {
      setTossTestStep(pop, "connect", "fail", err?.message || String(err));
      setTossTestStep(pop, "auth", "skip");
    }
    finishTossTest(pop, false, `${getUIText("web_toss_connection_failed", "Toss server connection failed")}: ${err?.message || err}`);
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
