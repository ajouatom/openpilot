"use strict";

// Translation registry — loaded by translations/registry.js + ko/en/zh/ja/fr.js
const TRANSLATION_REGISTRY = window.CarrotTranslations || { packs: {}, order: ["en", "ko", "zh"] };
const UI_STRINGS = TRANSLATION_REGISTRY.strings || {};
const ACTION_LABELS = TRANSLATION_REGISTRY.actionLabels || {};
const ERROR_MESSAGES = TRANSLATION_REGISTRY.errorMessages || {};
const DRIVE_MODES = TRANSLATION_REGISTRY.driveModes || {};

let LANG = "en";


function normalizeLangCode(raw) {
  const value = String(raw || "").trim().toLowerCase();
  const packs = window.CarrotTranslations?.packs || {};
  if (packs[value]) return value;
  const deviceAliases = {
    main_ko: "ko",
    main_en: "en",
    "main_zh-chs": "zh",
    "main_zh-cht": "zh",
    main_ja: "ja",
    main_fr: "fr",
  };
  if (deviceAliases[value]) return deviceAliases[value];
  const withoutMainPrefix = value.replace(/^main[_-]/, "");
  if (packs[withoutMainPrefix]) return withoutMainPrefix;
  if (value.startsWith("ko")) return "ko";
  if (value.startsWith("zh")) return "zh";
  if (value.startsWith("ja")) return "ja";
  if (value.startsWith("fr")) return "fr";
  if (value.startsWith("en")) return "en";
  return "";
}

function detectDefaultLang() {
  try {
    const stored = normalizeLangCode(localStorage.getItem(LANG_STORAGE_KEY));
    if (stored) return stored;
  } catch {}

  const browserLangs = Array.isArray(navigator.languages) && navigator.languages.length
    ? navigator.languages
    : [navigator.language, navigator.userLanguage];
  for (const candidate of browserLangs) {
    const normalized = normalizeLangCode(candidate);
    if (normalized) return normalized;
  }
  return "en";
}

LANG = detectDefaultLang();

function hasStoredWebLanguage() {
  try {
    return Boolean(normalizeLangCode(localStorage.getItem(LANG_STORAGE_KEY)));
  } catch {
    return false;
  }
}


/* ── Friendly error / action label lookup ─────────────────── */
function friendlyError(json) {
  if (!json) return null;
  const code = json.error_code;
  const detail = json.error_detail || "";
  const langMap = ERROR_MESSAGES[LANG] || ERROR_MESSAGES.en;
  if (code && langMap[code]) return langMap[code](detail);
  // fallback: try to make raw errors more readable
  const raw = json.error || "";
  if (raw.startsWith("git subcommand not allowed:")) {
    const sub = raw.replace("git subcommand not allowed:", "").trim();
    return (ERROR_MESSAGES[LANG] || ERROR_MESSAGES.en).GIT_CMD_NOT_ALLOWED(sub);
  }
  if (raw.startsWith("not allowed:")) {
    const cmd = raw.replace("not allowed:", "").trim();
    return (ERROR_MESSAGES[LANG] || ERROR_MESSAGES.en).CMD_NOT_ALLOWED(cmd);
  }
  if (raw === "timeout") return (ERROR_MESSAGES[LANG] || ERROR_MESSAGES.en).CMD_TIMEOUT();
  if (raw === "bad mode") return (ERROR_MESSAGES[LANG] || ERROR_MESSAGES.en).INVALID_RESET_MODE();
  if (raw === "missing branch") return (ERROR_MESSAGES[LANG] || ERROR_MESSAGES.en).MISSING_BRANCH();
  return raw || null;
}

function getActionLabel(action) {
  const labels = (ACTION_LABELS[LANG] || ACTION_LABELS.en)[action];
  return labels || { running: action + "...", done: action, failed: action };
}


/* ── Text rendering helpers ──────────────────────────────── */
function setNavText(id, txt) {
  const el = document.getElementById(id);
  if (!el) return;
  // The label is the last <span> child
  const spans = el.querySelectorAll(":scope > span");
  if (spans.length >= 2) spans[spans.length - 1].textContent = txt;
  else el.textContent = txt;
}

function setText(id, txt) {
  const el = document.getElementById(id);
  if (el) el.textContent = txt;
}

function updateLangLabel() {
  const main = langLabel?.querySelector(".lang-label__main");
  const sub = langLabel?.querySelector(".lang-label__sub");
  const emoji = LANG_EMOJI[LANG] || "🌐";
  const pack = TRANSLATION_REGISTRY.getPack?.(LANG) || {};
  const languageName = pack.name || LANG.toUpperCase();
  const nativeName = pack.nativeName || languageName;
  if (langLabel) {
    if (main && sub) {
      main.textContent = emoji;
      sub.textContent = "";
      sub.hidden = true;
    } else {
      langLabel.textContent = emoji;
    }
  }

  if (btnLang) {
    const text = `${getUIText("language", getUIText("lang", "Language"))} (${languageName})`;
    btnLang.setAttribute("aria-label", text);
    btnLang.title = text;
  }
  if (btnSettingLang) {
    const label = `${getUIText("language", getUIText("lang", "Language"))} · ${nativeName}`;
    btnSettingLang.textContent = label;
    btnSettingLang.title = label;
  }
  document.documentElement.lang = LANG;
}

function formatUIText(text, vars = {}) {
  let out = String(text ?? "");
  Object.entries(vars || {}).forEach(([key, value]) => {
    out = out.replaceAll(`{${key}}`, String(value));
  });
  return out;
}

function getUIText(key, fallback = "", vars = null) {
  const value = UI_STRINGS[LANG]?.[key] ?? UI_STRINGS.en?.[key] ?? UI_STRINGS.ko?.[key] ?? fallback;
  return vars ? formatUIText(value, vars) : value;
}


/* ── Page-wide UI text rendering ─────────────────────────── */
function renderUIText() {
  const s = UI_STRINGS[LANG];
  if (!s) return;
  document.title = "CarrotPilot";

  // Nav bar (nested spans — set last child text)
  setNavText("btnHome", s.home);
  setNavText("btnSetting", s.setting);
  setNavText("btnTools", s.tools);
  setNavText("btnLogs", s.logs);
  setNavText("btnTerminal", s.terminal);
  setText("btnQuickLinkWeb", "CarrotMan");

  setText("carrotTitle", "CarrotPilot");

  // Car Select
  setText("carTitle", s.car_select);
  setText("btnBackCar", s.back);
  setText("makersTitle", s.makers);
  setText("modelTitle", s.models);

  // Setting
  setText("settingTitleText", s.setting);
  setText("settingCarEyebrow", s.car_select);
  setText("btnBackGroups", s.back);
  setText("groupsTitle", s.groups);
  setText("itemsTitle", s.items);

  // Tools
  setText("toolsTitle", s.tools);
  setText("gitCommandsTitle", "Git Commands");
  setText("userSystemTitle", "User / System");
  setText("toolsQuickLinkTitle", "Link");
  setText("userSettingsTitle", "Settings");
  setText("btnDeviceInfo", getUIText("carrot_info", "Carrot Info"));
  setText("btnGitRemote", "change repository");
  setText("btnGitBranch", "change branch");
  setText("btnGitAddRemote", "add remote");
  setText("btnGitResetRepo", "reset repo");
  setText("btnDeviceLang", "Device Lang");
  setText("btnResetCalib", "Reset Calib");
  setText("btnSendTmuxLog", "capture tmux");
  setText("btnSendTmuxServerLog", "send tmux");
  setText("btnInstallRequired", "install flask");
  setText("btnDeleteVideos", "delete all videos");
  setText("btnDeleteLogs", "delete all logs");
  setText("btnRebuildAll", "Rebuild All");
  setText("btnReboot", "Reboot");
  setText("btnBackupSettings", "Backup");
  setText("btnRestoreSettings", "Restore");
  setText("btnCopySettings", "Copy");
  setText("btnViewSettings", "View");
  setText("sysCmdTitle", getUIText("section_sys_cmd", "System Command"));
  setText("sysCmdHelp", getUIText("sys_cmd_help", "Allowed: pull, status, branch, log, git ..., df, free, uptime"));
  setText("outputTitle", getUIText("section_output", "Output"));
  setText("terminalTitle", s.terminal);
  setText("terminalSessionMeta", "/data/openpilot");
  setText("btnTerminalCtrlC", s.terminal_ctrl_c);
  setText("btnTerminalClear", s.terminal_clear);
  setText("btnTerminalReconnect", s.terminal_reconnect);
  setText("btnTerminalSend", s.terminal_send);
  setText("logsDashcamTitle", s.logs_dashcam || "Dashcam");
  setText("logsScreenTitle", s.logs_screenrecord || "Screen Record");
  setText("btnStartVision", `▶ ${s.start_vision || "Start Drive Vision"}`);
  if (typeof applyRecordFabState === "function") applyRecordFabState();
  if (window.HomeDrive && typeof window.HomeDrive.renderText === "function") {
    window.HomeDrive.renderText();
  }
  const terminalInput = document.getElementById("terminalInput");
  if (terminalInput) terminalInput.placeholder = "";
  setText("settingSearchTitle", s.setting_search);
  if (settingSearchInput) settingSearchInput.placeholder = s.setting_search_placeholder || "";
  if (settingSearchMeta && (!settingSearchInput || !settingSearchInput.value.trim())) {
    settingSearchMeta.textContent = s.setting_search_idle || "";
  }
  if (btnSettingSearch) {
    btnSettingSearch.setAttribute("aria-label", s.setting_search || "Search Settings");
    btnSettingSearch.title = s.setting_search || "Search Settings";
  }
  if (btnSettingSearchSubmit) {
    btnSettingSearchSubmit.setAttribute("aria-label", s.setting_search || "Search Settings");
    btnSettingSearchSubmit.title = s.setting_search || "Search Settings";
  }
  if (typeof renderSettingSearchResults === "function" && settingSearchPanel && !settingSearchPanel.hidden) {
    renderSettingSearchResults(settingSearchInput?.value || "");
  }
  setText("appBranchPickerTitle", s.branch_select);
  setText("appBranchPickerClose", s.close);
  setText("appCarPickerTitle", s.car_select);
  setText("appCarPickerClose", s.cancel);
  updateLangLabel();
  syncHomeUtilityButtons();
  if (window.DrivingHud && typeof window.DrivingHud.renderText === "function") {
    window.DrivingHud.renderText();
  }
  renderQuickLinkUI();
}


/* ── Language switching ──────────────────────────────────── */
function setWebLanguage(lang, options = {}) {
  const normalized = normalizeLangCode(lang);
  if (!normalized || !UI_STRINGS[normalized]) return false;
  const persist = options.persist !== false;
  const render = options.render !== false;
  const dispatch = options.dispatch !== false;
  LANG = normalized;
  if (persist) {
    try {
      localStorage.setItem(LANG_STORAGE_KEY, LANG);
    } catch {}
  }

  updateLangLabel();

  if (!render) return true;

  renderUIText();
  if (typeof loadRecordState === "function") loadRecordState().catch(() => {});
  if (typeof rerenderPageLangUi === "function") rerenderPageLangUi();

  if (SETTINGS && !(typeof getCurrentSettingTab === "function" && getCurrentSettingTab() === "device")) {
    if (typeof rebuildSettingSearchEntries === "function") rebuildSettingSearchEntries();
    if (typeof renderGroups === "function") renderGroups({ animateGroups: false });
    if (typeof renderSettingSubnav === "function") renderSettingSubnav();
    if (CURRENT_GROUP && typeof renderItems === "function") {
      const currentTop = typeof getSettingItemsScrollTop === "function"
        ? getSettingItemsScrollTop()
        : 0;
      renderItems(CURRENT_GROUP, { scrollMode: "restore", scrollTop: currentTop, animateItems: false });
    }
  }
  if (dispatch) {
    window.dispatchEvent(new CustomEvent("carrot:languagechange", { detail: { lang: LANG } }));
  }
  return true;
}

async function syncWebLanguageFromDeviceDefault() {
  if (hasStoredWebLanguage()) return false;
  try {
    const res = await fetch("/api/device_info", { cache: "no-store" });
    if (!res.ok) return false;
    const info = await res.json();
    const lang = normalizeLangCode(info?.language);
    if (!lang || lang === LANG) return false;
    return setWebLanguage(lang, { persist: false, render: false, dispatch: false });
  } catch {
    return false;
  }
}

function toggleLang() {
  const order = (TRANSLATION_REGISTRY.order || ["en", "ko", "zh"]).filter((lang) => UI_STRINGS[lang]);
  const currentIndex = Math.max(0, order.indexOf(LANG));
  const next = order[(currentIndex + 1) % order.length] || "en";
  setWebLanguage(next);
}

if (btnLang) btnLang.onclick = () => toggleLang();
if (btnSettingLang) btnSettingLang.onclick = () => toggleLang();
