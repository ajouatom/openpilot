"use strict";

// Web-only settings dialog. Server-backed settings belong to the device, while
// last-page history remains browser-local by design.

const WEB_SETTINGS_GROUPS = [
  {
    id: "general",
    labelKey: "web_settings_general",
    defaultLabel: "General",
    items: [
      {
        id: "auto_update_git_pull",
        type: "toggle",
        titleKey: "web_auto_update",
        defaultTitle: "Auto update",
        descKey: "web_auto_update_desc",
        defaultDesc: "Automatically run git pull when updates are available. This will not reboot.",
      },
    ],
  },
  {
    id: "display",
    labelKey: "web_settings_display",
    defaultLabel: "Display",
    items: [
      {
        id: "start_page",
        type: "select",
        titleKey: "web_start_page",
        defaultTitle: "Start menu",
        descKey: "web_start_page_desc",
        defaultDesc: "Choose which menu opens first when Carrot Web loads.",
        options: [
          { value: "last", labelKey: "web_start_page_last", defaultLabel: "Last tab" },
          { value: "carrot", labelKey: "home", defaultLabel: "Drive" },
          { value: "setting", labelKey: "setting", defaultLabel: "Setting" },
          { value: "tools", labelKey: "tools", defaultLabel: "Tools" },
          { value: "logs", labelKey: "logs", defaultLabel: "Logs" },
          { value: "terminal", labelKey: "terminal", defaultLabel: "Terminal" },
        ],
      },
    ],
  },
];

const WEB_LAST_PAGE_KEY = "carrot_web_last_page";
const WEB_AUTO_UPDATE_COOLDOWN_MS = 10 * 60 * 1000;
const WEB_PRIMARY_PAGES = new Set(["carrot", "setting", "tools", "logs", "terminal"]);
const WEB_SETTING_DEFAULTS = {
  auto_update_git_pull: false,
  start_page: "last",
  web_language: "",
};

const webSettingsState = { ...WEB_SETTING_DEFAULTS };
window.CarrotWebSettingsState = webSettingsState;

let activeWebSettingsGroup = "general";
let webSettingsLoaded = false;
let webSettingsLoadPromise = null;
let webAutoUpdateInFlight = false;
let webAutoUpdateLastAttempt = 0;

function normalizeWebSettingValue(key, value) {
  if (key === "auto_update_git_pull") {
    if (typeof value === "string") return ["1", "true", "yes", "on"].includes(value.trim().toLowerCase());
    return Boolean(value);
  }
  if (key === "start_page") {
    const page = String(value || "").trim().toLowerCase();
    return page === "last" || WEB_PRIMARY_PAGES.has(page) ? page : WEB_SETTING_DEFAULTS.start_page;
  }
  if (key === "web_language") {
    if (typeof normalizeLangCode === "function") return normalizeLangCode(value);
    const lang = String(value || "").trim().toLowerCase();
    return ["en", "ko", "zh", "ja", "fr"].includes(lang) ? lang : "";
  }
  return value;
}

function applyWebSettings(settings = {}) {
  Object.keys(WEB_SETTING_DEFAULTS).forEach((key) => {
    const value = Object.prototype.hasOwnProperty.call(settings, key) ? settings[key] : WEB_SETTING_DEFAULTS[key];
    webSettingsState[key] = normalizeWebSettingValue(key, value);
  });
  return webSettingsState;
}

applyWebSettings(window.__CARROT_BOOTSTRAP__?.webSettings || {});

async function requestWebSettings(method = "GET", body = null) {
  const options = { method };
  if (body) {
    options.headers = { "Content-Type": "application/json" };
    options.body = JSON.stringify(body);
  }
  if (typeof requestJson === "function") {
    return requestJson("/api/web_settings", options);
  }
  const response = await fetch("/api/web_settings", options);
  const payload = await response.json();
  if (!response.ok || payload?.ok === false) {
    throw new Error(payload?.error || `HTTP ${response.status}`);
  }
  return payload;
}

async function loadWebSettings(force = false) {
  if (!force && webSettingsLoaded) return webSettingsState;
  if (webSettingsLoadPromise) return webSettingsLoadPromise;
  webSettingsLoadPromise = requestWebSettings("GET")
    .then((payload) => {
      webSettingsLoaded = true;
      return applyWebSettings(payload?.settings || {});
    })
    .finally(() => {
      webSettingsLoadPromise = null;
    });
  return webSettingsLoadPromise;
}

function getWebSettingByKey(key, fallback = undefined) {
  if (!Object.prototype.hasOwnProperty.call(WEB_SETTING_DEFAULTS, key)) return fallback;
  return webSettingsState[key] ?? fallback ?? WEB_SETTING_DEFAULTS[key];
}

async function setWebSettingByKey(key, value) {
  if (!Object.prototype.hasOwnProperty.call(WEB_SETTING_DEFAULTS, key)) return undefined;
  const previous = webSettingsState[key];
  const next = normalizeWebSettingValue(key, value);
  webSettingsState[key] = next;
  try {
    const payload = await requestWebSettings("POST", { [key]: next });
    applyWebSettings(payload?.settings || { [key]: next });
    window.dispatchEvent(new CustomEvent("carrot:websettingschange", {
      detail: { key, value: webSettingsState[key], settings: { ...webSettingsState } },
    }));
  } catch (err) {
    webSettingsState[key] = previous;
    throw err;
  }
  return webSettingsState[key];
}

function getWebSettingValue(item) {
  return getWebSettingByKey(item.id, WEB_SETTING_DEFAULTS[item.id]);
}

function setWebSettingValue(item, value) {
  return setWebSettingByKey(item.id, value);
}

function getWebStartPage() {
  const setting = getWebStartPageSetting();
  if (setting !== "last") return setting;
  try {
    const lastPage = localStorage.getItem(WEB_LAST_PAGE_KEY);
    return WEB_PRIMARY_PAGES.has(lastPage) ? lastPage : "carrot";
  } catch {
    return "carrot";
  }
}

function getWebStartPageSetting() {
  return normalizeWebSettingValue("start_page", webSettingsState.start_page);
}

function setWebStartPage(value) {
  return setWebSettingByKey("start_page", value);
}

function recordWebLastPage(page) {
  if (!WEB_PRIMARY_PAGES.has(page)) return;
  try {
    localStorage.setItem(WEB_LAST_PAGE_KEY, page);
  } catch {}
}

function renderWebSettingsItem(item) {
  const title = getUIText(item.titleKey, item.defaultTitle);
  const desc = getUIText(item.descKey, item.defaultDesc);
  if (item.type === "toggle") {
    const checked = getWebSettingValue(item);
    return `
      <label class="web-settings-row web-settings-row--toggle" data-web-setting="${escapeHtml(item.id)}">
        <span class="web-settings-row__copy">
          <span class="web-settings-row__title">${escapeHtml(title)}</span>
          <span class="web-settings-row__desc">${escapeHtml(desc)}</span>
        </span>
        <span class="web-settings-switch">
          <input class="web-settings-switch__input" type="checkbox" ${checked ? "checked" : ""} />
          <span class="web-settings-switch__slider" aria-hidden="true"></span>
        </span>
      </label>`;
  }

  if (item.type === "select") {
    const value = getWebSettingValue(item);
    const options = (item.options || []).map((option) => `
      <option value="${escapeHtml(option.value)}" ${option.value === value ? "selected" : ""}>
        ${escapeHtml(getUIText(option.labelKey, option.defaultLabel))}
      </option>
    `).join("");
    return `
      <label class="web-settings-row web-settings-row--select" data-web-setting="${escapeHtml(item.id)}">
        <span class="web-settings-row__copy">
          <span class="web-settings-row__title">${escapeHtml(title)}</span>
          <span class="web-settings-row__desc">${escapeHtml(desc)}</span>
        </span>
        <select class="web-settings-select" aria-label="${escapeHtml(title)}">
          ${options}
        </select>
      </label>`;
  }

  return `
    <div class="web-settings-row">
      <div class="web-settings-row__title">${escapeHtml(title)}</div>
      ${desc ? `<div class="web-settings-row__desc">${escapeHtml(desc)}</div>` : ""}
    </div>`;
}

function renderWebSettingsGroup(group) {
  const title = getUIText(group.labelKey, group.defaultLabel);
  const body = group.items.length
    ? group.items.map(renderWebSettingsItem).join("")
    : `<div class="web-settings-empty">${escapeHtml(getUIText("web_settings_empty", "No general web settings yet."))}</div>`;

  return `
    <section class="web-settings-group" data-web-settings-panel="${escapeHtml(group.id)}" ${group.id === activeWebSettingsGroup ? "" : "hidden"}>
      <div class="web-settings-group__title">${escapeHtml(title)}</div>
      <div class="web-settings-group__body">${body}</div>
    </section>`;
}

function renderWebSettingsDialogHtml() {
  const groups = WEB_SETTINGS_GROUPS.map((group) => {
    const active = group.id === activeWebSettingsGroup;
    return `
      <button type="button" class="web-settings-nav__item ${active ? "is-active" : ""}" data-web-settings-group="${escapeHtml(group.id)}" aria-pressed="${active ? "true" : "false"}">
        ${escapeHtml(getUIText(group.labelKey, group.defaultLabel))}
      </button>`;
  }).join("");

  return `
    <div class="web-settings-dialog">
      <nav class="web-settings-nav" aria-label="${escapeHtml(getUIText("web_settings", "Web Settings"))}">
        ${groups}
      </nav>
      <div class="web-settings-panels">
        ${WEB_SETTINGS_GROUPS.map(renderWebSettingsGroup).join("")}
      </div>
    </div>`;
}

function syncWebSettingsDialog() {
  document.querySelectorAll("[data-web-settings-group]").forEach((button) => {
    const active = button.dataset.webSettingsGroup === activeWebSettingsGroup;
    button.classList.toggle("is-active", active);
    button.setAttribute("aria-pressed", active ? "true" : "false");
  });
  document.querySelectorAll("[data-web-settings-panel]").forEach((panel) => {
    panel.hidden = panel.dataset.webSettingsPanel !== activeWebSettingsGroup;
  });
}

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
    const input = row.querySelector(".web-settings-switch__input");
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

async function runWebAutoUpdateGitPull(status = {}) {
  if (!getWebSettingByKey("auto_update_git_pull", false)) return;
  if (webAutoUpdateInFlight) return;
  if (document.hidden) return;
  const behind = Math.max(0, Number(status.behind || 0));
  if (behind <= 0) return;
  if (Date.now() - webAutoUpdateLastAttempt < WEB_AUTO_UPDATE_COOLDOWN_MS) return;

  webAutoUpdateInFlight = true;
  webAutoUpdateLastAttempt = Date.now();
  try {
    if (typeof toolsLogNotice === "function") {
      toolsLogNotice(getUIText("web_auto_update_running", "Auto update: running git pull."), { label: "auto update" });
    }
    if (typeof runTool === "function") {
      await runTool("git_pull");
    } else {
      await postJson("/api/tools", { action: "git_pull" });
    }
    if (typeof refreshToolsMetaInfo === "function") await refreshToolsMetaInfo();
    if (typeof refreshGitPullStatus === "function") await refreshGitPullStatus({ force: true, ttlMs: 0 });
    if (typeof toolsLogNotice === "function") {
      toolsLogNotice(getUIText("web_auto_update_done", "Auto update complete. Reboot was not requested."), { label: "auto update" });
    }
  } catch (err) {
    if (typeof showAppToast === "function") {
      showAppToast(getUIText("web_auto_update_failed", "Auto update failed"), { tone: "error" });
    }
    if (typeof toolsLogNotice === "function") {
      toolsLogNotice(err?.message || String(err), { label: "auto update", meta: false });
    }
  } finally {
    webAutoUpdateInFlight = false;
  }
}

function handleWebAutoUpdateStatus(status = {}) {
  runWebAutoUpdateGitPull(status).catch((err) => console.error("[WebSettings]", err));
}

window.loadWebSettings = loadWebSettings;
window.getWebSettingByKey = getWebSettingByKey;
window.setWebSettingByKey = setWebSettingByKey;
window.getWebStartPage = getWebStartPage;
window.getWebStartPageSetting = getWebStartPageSetting;
window.setWebStartPage = setWebStartPage;
window.recordWebLastPage = recordWebLastPage;
window.openWebSettingsDialog = openWebSettingsDialog;
window.handleWebAutoUpdateStatus = handleWebAutoUpdateStatus;
