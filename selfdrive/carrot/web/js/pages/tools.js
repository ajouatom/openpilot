"use strict";

// Tools page — meta info, output console, action runners, initToolsPage which
// binds all the tool buttons. (Branch picker modal + branch utilities live in
// pages/branch.js since they are also used by the standalone Branch page.)

let toolsMetaLoadPromise = null;
let toolsMetaLoadedAt = 0;
let toolsMetaLastValues = null;
let gitPullStatusPromise = null;
let gitPullStatusLoadedAt = 0;
let gitPullStatusTimer = null;
let gitPullStatusVisibilityBound = false;
let uiWarmupTimer = null;

const GIT_PULL_STATUS_CLIENT_INTERVAL_MS = 10000;

let toolsOutHistory = "";
let toolsOutCurrentBlock = "";
let toolsLogAttentionTimer = null;

function normalizeToolsOutText(s) {
  return String(s ?? "").replace(/\s+$/, "");
}

function scrollToolsLogToBottom(delay = 0) {
  window.setTimeout(() => {
    const out = document.getElementById("toolsOut");
    if (!out) return;
    out.scrollTop = out.scrollHeight;
  }, delay);
}

function pulseToolsLogPanel() {
  const page = document.getElementById("pageTools");
  if (!page || page.classList.contains("tools-log-expanded")) return;
  page.classList.add("tools-log-attention");
  if (toolsLogAttentionTimer) window.clearTimeout(toolsLogAttentionTimer);
  toolsLogAttentionTimer = window.setTimeout(() => {
    page.classList.remove("tools-log-attention");
    toolsLogAttentionTimer = null;
    scrollToolsLogToBottom();
    scrollToolsLogToBottom(280);
  }, 3200);
  scrollToolsLogToBottom(280);
}

function setToolsLogExpanded(expanded) {
  const page = document.getElementById("pageTools");
  if (!page) return;
  page.classList.toggle("tools-log-expanded", expanded);
  document.getElementById("toolsOut")?.setAttribute("aria-expanded", expanded ? "true" : "false");
  if (expanded) {
    page.classList.remove("tools-log-attention");
    if (toolsLogAttentionTimer) {
      window.clearTimeout(toolsLogAttentionTimer);
      toolsLogAttentionTimer = null;
    }
  }
  scrollToolsLogToBottom();
  scrollToolsLogToBottom(280);
}

function renderToolsOut() {
  const out = document.getElementById("toolsOut");
  if (!out) return;
  const historyText = normalizeToolsOutText(toolsOutHistory);
  const currentText = normalizeToolsOutText(toolsOutCurrentBlock);

  if (!historyText && !currentText) {
    out.textContent = " ";
  } else {
    const frag = document.createDocumentFragment();

    if (historyText) {
      const historyBlock = document.createElement("span");
      historyBlock.className = "tools-console-log__history";
      historyBlock.textContent = historyText;
      frag.appendChild(historyBlock);
    }

    if (historyText && currentText) {
      frag.appendChild(document.createTextNode("\n\n"));
    }

    if (currentText) {
      const currentBlock = document.createElement("span");
      currentBlock.className = "tools-console-log__current";
      currentBlock.textContent = currentText;
      frag.appendChild(currentBlock);
    }

    out.replaceChildren(frag);
  }

  requestAnimationFrame(() => scrollToolsLogToBottom());
  scrollToolsLogToBottom(280);
}

function toolsOutSet(s) {
  toolsOutCurrentBlock = normalizeToolsOutText(s);
  renderToolsOut();
  pulseToolsLogPanel();
}

function toolsOutAppend(s) {
  const next = normalizeToolsOutText(s);
  if (!next) return;
  toolsOutHistory = toolsOutHistory ? `${toolsOutHistory}\n\n${next}` : next;
  renderToolsOut();
  pulseToolsLogPanel();
}

function toolsOutCommitCurrent() {
  if (!toolsOutCurrentBlock) return;
  toolsOutAppend(toolsOutCurrentBlock);
  toolsOutCurrentBlock = "";
  renderToolsOut();
}

function toolsLogNotice(message, options = {}) {
  const text = normalizeToolsOutText(message);
  if (!text) return;
  const label = options.label ? String(options.label).trim() : "notice";
  toolsOutCommitCurrent();
  toolsOutAppend(`[${label}]\n${text}`);
  if (options.meta !== false) toolsMetaSet(text.split("\n")[0]);
  if (options.clearProgress !== false) toolsProgressSet(null, { active: false });
}

function renderGitPullStatus(status = {}) {
  const button = document.getElementById("btnGitPull");
  const badge = document.getElementById("gitPullBadge");
  const navButton = (typeof btnTools !== "undefined" && btnTools) ? btnTools : document.getElementById("btnTools");
  if (!button || !badge) return;

  const behind = Math.max(0, Number(status.behind || 0));
  const state = String(status.state || "");
  const hasError = Boolean(state && state !== "ok");
  const hasUpdates = behind > 0;
  const label = hasUpdates ? (behind > 99 ? "99+" : String(behind)) : (hasError ? "X" : "✓");
  button.classList.toggle("has-updates", hasUpdates);
  button.classList.toggle("is-current", !hasUpdates && !hasError);
  button.classList.toggle("has-git-error", !hasUpdates && hasError);
  badge.hidden = false;
  badge.textContent = label;
  badge.dataset.state = hasUpdates ? "updates" : (hasError ? "error" : "current");

  if (navButton) {
    navButton.classList.toggle("has-git-updates", hasUpdates);
    if (hasUpdates) navButton.dataset.gitBehind = label;
    else navButton.removeAttribute("data-git-behind");
  }

  if (hasUpdates) {
    const upstream = String(status.upstream || "").trim();
    const suffix = upstream ? ` (${upstream})` : "";
    button.title = `${behind} commits available${suffix}`;
  } else if (hasError) {
    button.title = status.error || status.fetch_error || "git status unavailable";
  } else {
    button.title = "Up to date";
  }
}

async function refreshGitPullStatus(options = {}) {
  const force = options.force === true;
  const ttlMs = Number.isFinite(options.ttlMs) ? options.ttlMs : 60000;
  if (!force && gitPullStatusPromise) return gitPullStatusPromise;
  if (!force && hasFreshPageData(gitPullStatusLoadedAt, ttlMs)) return null;

  const url = `/api/tools/git_status${force ? "?force=1" : ""}`;
  gitPullStatusPromise = getJson(url)
    .then((status) => {
      gitPullStatusLoadedAt = Date.now();
      renderGitPullStatus(status);
      return status;
    })
    .catch((error) => {
      console.log("[GitStatus] check failed:", error);
      renderGitPullStatus({ behind: 0 });
      return null;
    })
    .finally(() => {
      gitPullStatusPromise = null;
    });
  return gitPullStatusPromise;
}

function startGitPullStatusPolling() {
  if (!gitPullStatusVisibilityBound) {
    gitPullStatusVisibilityBound = true;
    document.addEventListener("visibilitychange", () => {
      if (!document.hidden) refreshGitPullStatus({ force: true, ttlMs: 0 }).catch(() => {});
    });
  }

  if (gitPullStatusTimer) return;
  gitPullStatusTimer = window.setInterval(() => {
    if (!document.hidden) refreshGitPullStatus({ ttlMs: 0 }).catch(() => {});
  }, GIT_PULL_STATUS_CLIENT_INTERVAL_MS);
}

function getToolCommandPreview(action, payload = {}) {
  switch (action) {
    case "shell_cmd": return String(payload.cmd || "").trim() || "command";
    case "git_pull": return "git pull";
    case "git_sync": return "git sync";
    case "git_reset": return `git reset --${payload.mode || "hard"} ${payload.target || "HEAD"}`.trim();
    case "git_checkout": return `git checkout ${payload.branch || ""}`.trim();
    case "git_branch_list": return "change branch";
    case "git_remote_add": return `git remote add/set-url ${payload.name || ""}`.trim();
    case "send_tmux_log": return "capture tmux";
    case "server_tmux_log": return "send tmux";
    case "install_required": return "install flask";
    case "delete_all_videos": return "delete all videos";
    case "delete_all_logs": return "delete all logs";
    case "rebuild_all": return "rebuild all";
    case "backup_settings": return "backup settings";
    case "reboot": return "reboot";
    case "git_reset_repo_fetch": return "reset repo (fetch)";
    case "git_reset_repo_checkout": return `reset repo (checkout ${payload.branch || "unknown"})`;
    case "reset_calib": return "reset calib";
    default: return action;
  }
}

let toolsMetaStatusText = "";
let toolsMetaInfoText = "";
let toolsMetaInfoDialogText = "";
let toolsLanguageMenuOpen = false;

function getAvailableWebLanguages() {
  const registry = window.CarrotTranslations || {};
  const order = Array.isArray(registry.order) ? registry.order : ["ko", "en", "zh"];
  return order
    .map((lang) => {
      const pack = registry.getPack?.(lang) || registry.packs?.[lang] || {};
      const strings = UI_STRINGS?.[lang];
      if (!strings) return null;
      return {
        lang,
        name: pack.name || lang.toUpperCase(),
        nativeName: pack.nativeName || pack.name || lang.toUpperCase(),
        shortName: pack.shortName || lang.toUpperCase(),
      };
    })
    .filter(Boolean);
}

function closeToolsLanguageMenu() {
  if (!toolsLanguageMenuOpen) return;
  toolsLanguageMenuOpen = false;
  renderToolsMeta();
}

function setToolsLanguageMenuPosition(anchor) {
  if (!anchor || typeof anchor.getBoundingClientRect !== "function") return;
  const rect = anchor.getBoundingClientRect();
  const top = Math.max(12, Math.round(rect.bottom + 10));
  const right = Math.max(12, Math.round((window.innerWidth || document.documentElement.clientWidth || 0) - rect.right));
  document.documentElement.style.setProperty("--tools-lang-menu-panel-top", `${top}px`);
  document.documentElement.style.setProperty("--tools-lang-menu-panel-right", `${right}px`);
}

function bindToolsLanguageMenuDismiss() {
  if (document.body.dataset.toolsLangDismissBound === "1") return;
  document.body.dataset.toolsLangDismissBound = "1";
  document.addEventListener("click", (event) => {
    if (!toolsLanguageMenuOpen) return;
    if (event.target instanceof Element && event.target.closest(".tools-lang-menu, .tools-lang-menu__panel")) return;
    closeToolsLanguageMenu();
  });
  document.addEventListener("keydown", (event) => {
    if (event.key === "Escape") closeToolsLanguageMenu();
  });
}

function renderToolsMeta() {
  const meta = document.getElementById("toolsMeta");
  if (!meta) return;
  bindToolsLanguageMenuDismiss();

  meta.textContent = "";

  const statusEl = document.createElement("span");
  statusEl.className = "tools-meta__status";
  statusEl.textContent = toolsMetaStatusText || "-";
  meta.appendChild(statusEl);

  const actionsEl = document.createElement("div");
  actionsEl.className = "tools-meta__actions";

  const languages = getAvailableWebLanguages();
  const current = languages.find((item) => item.lang === LANG) || languages[0];
  const langWrap = document.createElement("div");
  langWrap.className = "tools-lang-menu";

  const langBtn = document.createElement("button");
  langBtn.type = "button";
  langBtn.className = "tools-lang-menu__button";
  langBtn.setAttribute("aria-haspopup", "menu");
  langBtn.setAttribute("aria-expanded", toolsLanguageMenuOpen ? "true" : "false");
  langBtn.innerHTML = `
    <span class="tools-lang-menu__globe" aria-hidden="true">
      <svg viewBox="0 0 24 24" focusable="false">
        <path fill="currentColor" d="M12 2a10 10 0 1 0 0 20 10 10 0 0 0 0-20m6.93 9h-3.01a15.7 15.7 0 0 0-1.08-5.05A8.04 8.04 0 0 1 18.93 11M12 4.04c.62.9 1.55 2.86 1.9 6.96h-3.8c.35-4.1 1.28-6.06 1.9-6.96M4.07 13h3.01c.16 1.97.55 3.73 1.08 5.05A8.04 8.04 0 0 1 4.07 13m3.01-2H4.07a8.04 8.04 0 0 1 4.09-5.05A15.7 15.7 0 0 0 7.08 11M12 19.96c-.62-.9-1.55-2.86-1.9-6.96h3.8c-.35 4.1-1.28 6.06-1.9 6.96m2.84-1.91c.53-1.32.92-3.08 1.08-5.05h3.01a8.04 8.04 0 0 1-4.09 5.05"/>
      </svg>
    </span>
    <span class="tools-lang-menu__label">${escapeHtml(getUIText("language", getUIText("lang", "Language")))}</span>
  `;
  langBtn.addEventListener("click", (event) => {
    event.stopPropagation();
    if (!toolsLanguageMenuOpen) setToolsLanguageMenuPosition(langBtn);
    toolsLanguageMenuOpen = !toolsLanguageMenuOpen;
    renderToolsMeta();
  });
  langWrap.appendChild(langBtn);

  if (toolsLanguageMenuOpen) {
    const panel = document.createElement("div");
    panel.className = "tools-lang-menu__panel";
    panel.setAttribute("role", "menu");
    const currentLabel = current?.name || LANG.toUpperCase();
    panel.innerHTML = `
      <div class="tools-lang-menu__current">${escapeHtml(getUIText("current_language", "Current language"))}: ${escapeHtml(currentLabel)}</div>
      <div class="tools-lang-menu__divider" aria-hidden="true"></div>
    `;
    languages.forEach((item) => {
      const option = document.createElement("button");
      option.type = "button";
      option.className = "tools-lang-menu__item";
      option.setAttribute("role", "menuitemradio");
      option.setAttribute("aria-checked", item.lang === LANG ? "true" : "false");
      option.innerHTML = `
        <span>${escapeHtml(item.name)}</span>
      `;
      option.addEventListener("click", (event) => {
        event.stopPropagation();
        if (typeof setWebLanguage === "function") setWebLanguage(item.lang);
        toolsLanguageMenuOpen = false;
        renderToolsMeta();
      });
      panel.appendChild(option);
    });
    langWrap.appendChild(panel);
  }
  actionsEl.appendChild(langWrap);

  meta.appendChild(actionsEl);
}

function toolsMetaSet(s) {
  toolsMetaStatusText = String(s || "");
  renderToolsMeta();
}

function buildToolsMetaInfo(values = {}) {
  const branch = String(values.GitBranch || "").trim();
  const commit = String(values.GitCommit || "").trim();
  const dongleId = String(values.DongleId || "").trim();
  const serial = String(values.HardwareSerial || "").trim();
  const parts = [];
  if (branch) parts.push(branch);
  if (commit) parts.push(commit.slice(0, 7));
  if (dongleId) parts.push(dongleId);
  if (serial) parts.push(serial);
  return parts.join("  ·  ");
}

function formatToolsMetaDate(value) {
  let raw = String(value || "").replace(/['"]/g, "").trim();
  if (!raw) return "";

  // Support "1730000000 2024-10-27..."
  const parts = raw.split(" ");
  if (parts.length > 1 && /^\d{10,}$/.test(parts[0])) {
    const d = new Date(parseInt(parts[0], 10) * 1000);
    if (!isNaN(d)) return `${d.getFullYear()}.${String(d.getMonth() + 1).padStart(2, "0")}.${String(d.getDate()).padStart(2, "0")}`;
  }

  const m = raw.match(/^(\d{4})[-./](\d{2})[-./](\d{2})/);
  if (m) return `${m[1]}.${m[2]}.${m[3]}`;
  const ts = Date.parse(raw);
  if (!Number.isFinite(ts)) return "";
  const d = new Date(ts);
  const yyyy = d.getFullYear();
  const mm = String(d.getMonth() + 1).padStart(2, "0");
  const dd = String(d.getDate()).padStart(2, "0");
  return `${yyyy}.${mm}.${dd}`;
}

function formatToolsMetaDateTime(value) {
  const raw = String(value || "").trim();
  if (!raw) return "";

  let ms = NaN;
  if (/^\d+$/.test(raw)) {
    const num = Number(raw);
    if (Number.isFinite(num)) ms = raw.length >= 13 ? num : num * 1000;
  } else {
    ms = Date.parse(raw);
  }
  if (!Number.isFinite(ms)) return "";

  const d = new Date(ms);
  const yyyy = d.getFullYear();
  const mm = String(d.getMonth() + 1).padStart(2, "0");
  const dd = String(d.getDate()).padStart(2, "0");
  const hh = String(d.getHours()).padStart(2, "0");
  const min = String(d.getMinutes()).padStart(2, "0");
  const ss = String(d.getSeconds()).padStart(2, "0");
  return `${yyyy}.${mm}.${dd} ${hh}:${min}:${ss}`;
}

function buildToolsMetaInfoDialog(values = {}) {
  const branch = String(values.GitBranch || "").trim();
  const commit = String(values.GitCommit || "").trim();
  const commitDate = formatToolsMetaDate(values.GitCommitDate);
  const dongleId = String(values.DongleId || "").trim();
  const serial = String(values.HardwareSerial || "").trim();
  const gitPullTime = formatToolsMetaDateTime(values.GitPullTime);
  const labels = {
    branch: getUIText("branch", "Branch"),
    commit: getUIText("commit", "Commit"),
    deviceType: getUIText("device_type", "Device"),
    dongle: getUIText("dongle_id", "Dongle ID"),
    serial: getUIText("serial", "Serial"),
    gitPull: getUIText("recent_update", "Recent update"),
    position: getUIText("position", "Position"),
  };
  const htmlEscape = typeof escapeHtml === "function"
    ? escapeHtml
    : (value) => String(value)
      .replaceAll("&", "&amp;")
      .replaceAll("<", "&lt;")
      .replaceAll(">", "&gt;")
      .replaceAll('"', "&quot;")
      .replaceAll("'", "&#039;");
  const lines = [];
  const remote = String(values.GitRemote || "").trim();
  const shortRemote = remote ? remote.replace(/^https?:\/\/[^/]+\//, "").replace(/\.git$/, "") : "";
  const branchText = branch + (shortRemote ? ` (${shortRemote})` : "");
  if (branchText) lines.push(`<div class="app-dialog__metaLine">${htmlEscape(labels.branch)}: ${htmlEscape(branchText)}</div>`);
  if (commit) {
    const commitText = `${commit.slice(0, 7)}${commitDate ? ` (${commitDate})` : ""}`;
    lines.push(`<div class="app-dialog__metaLine">${htmlEscape(labels.commit)}: ${htmlEscape(commitText)}</div>`);
  }

  if (dongleId) lines.push(`<div class="app-dialog__metaLine">${htmlEscape(labels.dongle)}: ${htmlEscape(dongleId)}</div>`);
  if (serial) lines.push(`<div class="app-dialog__metaLine">${htmlEscape(labels.serial)}: ${htmlEscape(serial)}</div>`);
  const position = String(values.DevicePosition || "").trim();
  lines.push(`<div class="app-dialog__metaLine">${htmlEscape(labels.position)}: ${htmlEscape(position)}</div>`);
  if (gitPullTime) {
    lines.push(`<div class="app-dialog__metaSubtle">${htmlEscape(labels.gitPull)}: ${htmlEscape(gitPullTime)}</div>`);
  }
  return `<div class="app-dialog__metaList" id="toolsMetaListContent">${lines.join("")}</div>`;
}

function buildToolsMetaPlainText(values = {}) {
  const branch = String(values.GitBranch || "").trim();
  const commit = String(values.GitCommit || "").trim();
  const commitDate = formatToolsMetaDate(values.GitCommitDate);
  const dongleId = String(values.DongleId || "").trim();
  const serial = String(values.HardwareSerial || "").trim();
  const gitPullTime = formatToolsMetaDateTime(values.GitPullTime);
  const remote = String(values.GitRemote || "").trim();
  const shortRemote = remote ? remote.replace(/^https?:\/\/[^/]+\//, "").replace(/\.git$/, "") : "";
  const branchText = branch + (shortRemote ? ` (${shortRemote})` : "");
  const position = String(values.DevicePosition || "").trim();
  const lines = [];
  if (branchText) lines.push(`Branch: ${branchText}`);
  if (commit) lines.push(`Commit: ${commit.slice(0, 7)}${commitDate ? ` (${commitDate})` : ""}`);
  if (dongleId) lines.push(`DongleID: ${dongleId}`);
  if (serial) lines.push(`Serial: ${serial}`);
  if (position) lines.push(`Position: ${position}`);
  if (gitPullTime) lines.push(`Updated: ${gitPullTime}`);
  return lines.join("\n");
}

function rerenderPageLangUi() {
  renderToolsMeta();
  refreshToolsMetaInfo().catch(() => {});
  if (CURRENT_PAGE === "logs") {
    renderDashcamRoutes({ animate: false });
    renderScreenrecordVideos?.({ animate: false });
  }

  const terminalMetaEl = document.getElementById("terminalMeta");
  if (!terminalMetaEl) return;

  const current = String(terminalMetaEl.textContent || "").trim();
  const terminalStates = [
    ["connecting", "connecting..."],
    ["connected", "connected"],
    ["reconnecting", "reconnecting..."],
    ["terminal_ready", "tmux ready"],
    ["terminal_disconnected", "disconnected"],
    ["terminal_unavailable", "terminal unavailable"],
    ["terminal_offline", "terminal offline"],
  ];

  const langOrder = window.CarrotTranslations?.order || ["ko", "en", "zh"];
  for (const [key, fallback] of terminalStates) {
    const variants = langOrder
      .map((langKey) => UI_STRINGS[langKey]?.[key] || fallback)
      .filter(Boolean);
    if (variants.includes(current)) {
      terminalMetaEl.textContent = getUIText(key, fallback);
      break;
    }
  }
}

async function refreshToolsMetaInfo(options = {}) {
  const force = options.force === true;
  const silent = options.silent === true;
  const ttlMs = Number.isFinite(options.ttlMs) ? options.ttlMs : PAGE_DATA_TTL_MS;
  if (typeof bulkGet !== "function") return;
  if (!force && toolsMetaLoadPromise) return toolsMetaLoadPromise;
  if (!force && hasFreshPageData(toolsMetaLoadedAt, ttlMs)) {
    if (!silent || CURRENT_PAGE === "tools") renderToolsMeta();
    return {
      text: toolsMetaInfoText,
      dialog: toolsMetaInfoDialogText,
    };
  }

  toolsMetaLoadPromise = (async () => {
    const values = await bulkGet(["GitBranch", "GitCommit", "GitCommitDate", "GitRemote", "DeviceType", "DongleId", "HardwareSerial", "GitPullTime", "DevicePosition"]);
    toolsMetaInfoText = buildToolsMetaInfo(values);
    toolsMetaInfoDialogText = buildToolsMetaInfoDialog(values);
    toolsMetaLoadedAt = Date.now();
    toolsMetaLastValues = values;
    if (!silent || CURRENT_PAGE === "tools") renderToolsMeta();
    return values;
  })().finally(() => {
    toolsMetaLoadPromise = null;
  });

  return toolsMetaLoadPromise;
}

async function syncDeviceLanguageOnce() {
  if (typeof bulkGet !== "function" || typeof setParam !== "function") return;
  try {
    const SYNC_KEY = "carrot_device_lang_synced";
    if (localStorage.getItem(SYNC_KEY) === "1") return;

    const values = await bulkGet(["LanguageSetting"]);
    const currentLang = String(values["LanguageSetting"] || "").trim();

    const browserLang = (navigator.language || navigator.userLanguage || "en").toLowerCase();
    let targetParam = "main_en";
    if (browserLang.startsWith("ko")) targetParam = "main_ko";
    else if (browserLang.startsWith("zh")) targetParam = browserLang.includes("tw") || browserLang.includes("hk") ? "main_zh-CHT" : "main_zh-CHS";
    else if (browserLang.startsWith("ja")) targetParam = "main_ja";
    else if (browserLang.startsWith("de")) targetParam = "main_de";
    else if (browserLang.startsWith("fr")) targetParam = "main_fr";
    else if (browserLang.startsWith("es")) targetParam = "main_es";
    else if (browserLang.startsWith("pt")) targetParam = "main_pt-BR";
    else if (browserLang.startsWith("tr")) targetParam = "main_tr";
    else if (browserLang.startsWith("ar")) targetParam = "main_ar";
    else if (browserLang.startsWith("th")) targetParam = "main_th";

    if (currentLang !== targetParam) {
      await setParam("LanguageSetting", targetParam);
      localStorage.setItem(SYNC_KEY, "1");
      // show notification after a short delay so the page finishes loading
      setTimeout(() => {
        openAppDialog({
          mode: "alert",
          title: getUIText("device_lang", "Device Language"),
          message: getUIText("device_lang_changed", "Device language has been changed.\nPlease reboot the device to apply."),
        });
      }, 800);
      return;
    }
    localStorage.setItem(SYNC_KEY, "1");
  } catch (e) {
    console.log("Language sync failed:", e);
  }
}

function runUiWarmup() {
  return Promise.allSettled([
    loadCurrentCar({ resetRetry: false, ttlMs: PAGE_DATA_TTL_MS }),
    loadRecordState({ ttlMs: PAGE_DATA_TTL_MS }),
    refreshToolsMetaInfo({ silent: true, ttlMs: PAGE_DATA_TTL_MS }),
    refreshGitPullStatus({ ttlMs: 600000 }),
    typeof updateQuickLink === "function" ? updateQuickLink({ silent: true, ttlMs: PAGE_DATA_TTL_MS }) : Promise.resolve(),
    loadSettings({ background: true }),
    ensureCarsLoaded(),
  ]);
}

function scheduleUiWarmup(delay = 140) {
  if (uiWarmupTimer) return;
  uiWarmupTimer = window.setTimeout(() => {
    uiWarmupTimer = null;
    requestIdleTask(() => {
      runUiWarmup().catch(() => {});
    });
  }, Math.max(0, delay));
}

if (document.readyState === "complete") {
  scheduleUiWarmup(120);
  startGitPullStatusPolling();
} else {
  window.addEventListener("load", () => {
    scheduleUiWarmup(120);
    startGitPullStatusPolling();
  }, { once: true });
}

function toolsProgressSet(percent = null, opts = {}) {
  const host = document.getElementById("toolsProgress");
  const bar = document.getElementById("toolsProgressBar");
  if (!host || !bar) return;

  const active = opts.active !== false;
  const indeterminate = Boolean(opts.indeterminate);
  if (!active) {
    host.hidden = true;
    host.classList.remove("is-indeterminate");
    bar.style.width = "0%";
    return;
  }

  host.hidden = false;
  host.classList.toggle("is-indeterminate", indeterminate);

  const hasPercent = Number.isFinite(percent);
  const safePercent = hasPercent ? Math.max(4, Math.min(100, Number(percent))) : 28;
  bar.style.width = `${safePercent}%`;
}

async function postJson(url, bodyObj) {
  const r = await fetch(url, {
    method: "POST",
    headers: {"Content-Type": "application/json"},
    body: JSON.stringify(bodyObj || {})
  });
  const j = await r.json().catch(() => ({}));
  if (!r.ok || !j.ok) {
    const msg = friendlyError(j) || j.error || ("HTTP " + r.status);
    throw new Error(msg);
  }
  return j;
}

async function getJson(url) {
  const r = await fetch(url);
  const j = await r.json().catch(() => ({}));
  if (!r.ok || !j.ok) {
    const msg = friendlyError(j) || j.error || ("HTTP " + r.status);
    throw new Error(msg);
  }
  return j;
}

function waitMs(ms) {
  return new Promise((resolve) => window.setTimeout(resolve, ms));
}

let activeToolRunToken = 0;

function updateToolsRunningState(labels, snapshot) {
  const message = String(snapshot?.message || labels.running || "");
  const stepCurrent = Number(snapshot?.step_current || 0);
  const stepTotal = Number(snapshot?.step_total || 0);
  const progressValue = Number(snapshot?.progress);
  const hasProgress = Number.isFinite(progressValue) && progressValue > 0;

  let metaText = message || labels.running;
  if (hasProgress && progressValue < 100) {
    metaText = `${metaText} · ${Math.round(progressValue)}%`;
  } else if (stepTotal > 1 && stepCurrent > 0) {
    metaText = `${metaText} · ${stepCurrent}/${stepTotal}`;
  }

  toolsMetaSet(metaText);
  toolsProgressSet(hasProgress ? progressValue : null, {
    active: true,
    indeterminate: !hasProgress || progressValue >= 100,
  });
}

async function runTool(action, payload) {
  const labels = getActionLabel(action);
  const runToken = ++activeToolRunToken;
  const commandPreview = getToolCommandPreview(action, payload || {});
  const activityId = typeof beginAppActivity === "function"
    ? beginAppActivity("tools", labels.running || commandPreview)
    : null;

  try {
    toolsMetaSet(labels.running);
    toolsOutCommitCurrent();
    toolsOutSet(`> ${commandPreview}\n${labels.running}`);
    toolsProgressSet(null, { active: true, indeterminate: true });

    const started = await postJson("/api/tools/start", { action, ...(payload || {}) });
    const jobId = started.job_id;
    let snapshot = null;

    while (runToken === activeToolRunToken) {
      snapshot = await getJson(`/api/tools/job?id=${encodeURIComponent(jobId)}`);

      if (snapshot.log != null) {
        const body = normalizeToolsOutText(snapshot.log) || labels.running;
        toolsOutSet(`> ${commandPreview}\n${body}`);
      }

      if (!snapshot.done) {
        updateToolsRunningState(labels, snapshot);
        await waitMs(320);
        continue;
      }

      const result = snapshot.result || snapshot;
      if (!result.ok) {
        const errMsg = friendlyError(result) || result.error || snapshot.error || labels.failed;
        toolsOutSet(`> ${commandPreview}\n${normalizeToolsOutText(snapshot?.log) || errMsg}`);
        toolsOutCommitCurrent();
        toolsMetaSet(labels.failed);
        toolsProgressSet(null, { active: false });
        throw new Error(errMsg);
      }

      const finalBody = normalizeToolsOutText(result.out ?? snapshot.log) || labels.done;
      toolsOutSet(`> ${commandPreview}\n${finalBody}`);
      toolsOutCommitCurrent();
      toolsMetaSet(labels.done);
      toolsProgressSet(100, { active: true, indeterminate: false });
      window.setTimeout(() => {
        if (activeToolRunToken === runToken) toolsProgressSet(null, { active: false });
      }, 900);
      return result;
    }

    throw new Error("tool run cancelled");
  } finally {
    if (activityId && typeof endAppActivity === "function") endAppActivity(activityId);
  }
}

function didGitPullUpdate(result) {
  const body = normalizeToolsOutText(result?.out || result?.log || "");
  if (!body) return false;
  const lower = body.toLowerCase();
  if (lower.includes("already up to date") || lower.includes("already up-to-date")) {
    return false;
  }
  return (
    lower.includes("fast-forward") ||
    lower.includes("merge made by") ||
    lower.includes("updating ") ||
    /[0-9]+\s+files?\s+changed/.test(lower) ||
    lower.includes("create mode ") ||
    lower.includes("delete mode ") ||
    lower.includes("rewrite ")
  );
}

async function confirmText(msg, placeholder = "") {
  const v = await appPrompt(msg, {
    title: UI_STRINGS[LANG].input_title || "Input",
    defaultValue: placeholder,
    placeholder,
  });
  if (v === null) return null;
  return String(v).trim();
}

function showError(action, error) {
  const labels = getActionLabel(action);
  const title = labels.failed;
  const msg = (typeof error === "object" && error.message) ? error.message : String(error);
  toolsMetaSet(title);
  toolsProgressSet(null, { active: false });
  toolsLogNotice(msg, { label: action, meta: false });
}

let branchPickerCloseTimer = null;
function initToolsPage() {
  const bindOnce = (id, fn) => {
    const el = document.getElementById(id);
    if (!el || el.dataset.bound === "1") return;
    el.dataset.bound = "1";
    el.onclick = fn;
  };

  const bindNodeOnce = (node, key, fn, eventName = "click") => {
    if (!node || node.dataset[key] === "1") return;
    node.dataset[key] = "1";
    node.addEventListener(eventName, fn);
  };

  const initToolsGroups = () => {
    const groups = Array.from(document.querySelectorAll("#pageTools .tools-group"));
    const applyToolsStagger = () => {
      const items = Array.from(document.querySelectorAll(
        "#pageTools .tools-scroll-stack > .row-wrap:first-child, #pageTools .tools-group, #pageTools .tools-group.is-open .tools-group__body > *"
      )).filter((node) => !node.hidden && !node.classList.contains("hidden"));
      items.forEach((node, index) => {
        node.classList.add("ui-stagger-item");
        node.style.setProperty("--i", String(index));
      });
    };

    groups.forEach((group) => {
      const toggle = group.querySelector(".tools-group__toggle");
      const body = group.querySelector(".tools-group__body");
      if (!toggle || !body) return;

      const groupName = group.dataset.toolsGroup;
      const savedState = localStorage.getItem("tools_group_" + groupName);
      const shouldOpen = savedState !== null ? savedState === "true" : true;

      group.classList.toggle("is-open", shouldOpen);
      body.hidden = !shouldOpen;
      body.classList.toggle("hidden", !shouldOpen);
      toggle.setAttribute("aria-expanded", shouldOpen ? "true" : "false");

      bindNodeOnce(toggle, "toolsGroupToggle", () => {
        const nextOpen = body.hidden;
        body.hidden = !nextOpen;
        body.classList.toggle("hidden", !nextOpen);
        group.classList.toggle("is-open", nextOpen);
        toggle.setAttribute("aria-expanded", nextOpen ? "true" : "false");
        localStorage.setItem("tools_group_" + groupName, nextOpen ? "true" : "false");
        applyToolsStagger();
      });
    });
    applyToolsStagger();
  };

  const initToolsLogPanel = () => {
    const out = document.getElementById("toolsOut");
    if (!out || out.dataset.toolsLogBound === "1") return;
    out.dataset.toolsLogBound = "1";
    out.setAttribute("role", "button");
    out.setAttribute("tabindex", "0");
    out.setAttribute("aria-expanded", "false");
    out.setAttribute("aria-label", getUIText("toggle_log_panel", "Expand or collapse log panel"));
    out.addEventListener("click", () => {
      const page = document.getElementById("pageTools");
      setToolsLogExpanded(!page?.classList.contains("tools-log-expanded"));
    });
    out.addEventListener("keydown", (ev) => {
      if (ev.key !== "Enter" && ev.key !== " ") return;
      ev.preventDefault();
      const page = document.getElementById("pageTools");
      setToolsLogExpanded(!page?.classList.contains("tools-log-expanded"));
    });
  };

  const runSystemCommand = async () => {
    const inp = document.getElementById("sysCmdInput");
    const cmd = (inp?.value || "").trim();
    if (!cmd) return;

    try {
      await runTool("shell_cmd", { cmd });
    } catch (e) {
      showError("shell_cmd", e);
    }
  };

  toolsMetaSet(UI_STRINGS[LANG].ready || "Ready");
  toolsProgressSet(null, { active: false });
  refreshToolsMetaInfo().catch(() => {});
  refreshGitPullStatus({ force: true }).catch(() => {});
  initToolsGroups();
  initToolsLogPanel();

  bindOnce("btnDeviceInfo", async () => {
    let title = getUIText("device_info", "Device Info");
    
    try {
      if (!toolsMetaLastValues && !toolsMetaLoadPromise) {
        await refreshToolsMetaInfo({ ttlMs: 3600000 });
      }
      const values = toolsMetaLoadPromise ? await toolsMetaLoadPromise : toolsMetaLastValues;
      if (values) {
        const deviceType = String(values.DeviceType || "").trim();
        if (deviceType) {
          const deviceFriendly = { tici: "c3", tizi: "c3x", mici: "c4" };
          const friendly = deviceFriendly[deviceType] || deviceType;
          const label = friendly !== deviceType ? `${friendly}/${deviceType}` : deviceType;
          title += `(${label})`;
        }
      }
    } catch (e) {}

    appAlert(toolsMetaInfoDialogText || toolsMetaInfoText, {
      title,
      html: true,
      messageHtml: toolsMetaInfoDialogText,
      copyText: buildToolsMetaPlainText(toolsMetaLastValues || {}),
    });
  });

  bindOnce("btnGitPull", async () => {
    try {
      const result = await runTool("git_pull");
      await refreshToolsMetaInfo();
      await refreshGitPullStatus({ force: true });
      if (!didGitPullUpdate(result)) return;
      if (await appConfirm(UI_STRINGS[LANG].confirm_reboot || "Reboot now?", {
        title: UI_STRINGS[LANG].reboot || "Reboot",
      })) {
        await runTool("reboot");
      }
    } catch (e) {
      showError("git_pull", e);
    }
  });

  bindOnce("btnGitSync", async () => {
    if (!await appConfirm(UI_STRINGS[LANG].git_sync_confirm || "Run git sync?", { title: "git sync" })) return;
    try {
      await runTool("git_sync");
      await refreshGitPullStatus({ force: true });
    } catch (e) {
      showError("git_sync", e);
    }
  });

  async function runGitResetMode(mode) {
    const safeMode = String(mode || "hard").trim().toLowerCase();
    const title = `git reset --${safeMode}`;
    const message = UI_STRINGS[LANG].git_reset_confirm || "Run git reset?";
    if (!await appConfirm(`${message}\n\nHEAD`, { title })) return;
    try {
      await runTool("git_reset", { mode: safeMode, target: "HEAD" });
    } catch (e) {
      showError("git_reset", e);
    }
  }

  bindOnce("btnGitReset", async () => {
    const mode = await openAppDialog({
      mode: "choice",
      title: "git reset",
      message: getUIText("git_reset_head_prompt", "Select reset mode based on HEAD."),
      cancelLabel: UI_STRINGS[LANG].cancel || "Cancel",
      choices: [
        { label: "reset hard", value: "hard", danger: true },
        { label: "reset mixed", value: "mixed" },
        { label: "reset soft", value: "soft" },
      ],
    });
    if (!mode) return;
    await runGitResetMode(mode);
  });

  bindOnce("btnGitRemote", async () => {
    const title = getUIText("git_remote_title", "Change Repository");
    let defaultUrl = "";
    try {
      const v = await bulkGet(["GitRemote"]);
      if (v && v.GitRemote) defaultUrl = String(v.GitRemote).trim();
    } catch (e) {}

    const msg = getUIText(
      "git_remote_prompt",
      "Current: {url}\n\nEnter new GitHub repository URL.\n(This will overwrite the current connection)",
      { url: defaultUrl }
    );
    
    const newUrl = await appPrompt(msg, { title, defaultValue: defaultUrl });
    if (!newUrl || newUrl.trim() === "" || newUrl.trim() === defaultUrl) return;

    try {
      const waitMsg = getUIText("git_remote_fetching", "Fetching repository data.\nThis may take a few minutes for new repositories.\nPlease wait...");
      toolsLogNotice(waitMsg, { label: "change repository" });
      await runTool("git_remote_set", { url: newUrl.trim() });
      await refreshToolsMetaInfo();
      await refreshGitPullStatus({ force: true });
      const successMsg = getUIText("git_remote_success", "Repository changed successfully.\nClick [change branch] to select a branch.");
      toolsLogNotice(successMsg, { label: "change repository" });
    } catch (e) {
      showError("change repository", e);
    }
  });
  bindOnce("btnGitBranch", async () => {
    await loadBranchesAndShow();
  });

  bindOnce("btnGitAddRemote", async () => {
    const title = getUIText("git_add_remote_title", "Add/Update Remote");
    const nameInput = await appPrompt(
      getUIText("git_add_remote_name_prompt", "Enter remote name (e.g. remote)"),
      { title, placeholder: "remote" }
    );
    if (!nameInput || !nameInput.trim()) return;
    const remoteName = nameInput.trim();

    const urlInput = await appPrompt(
      getUIText("git_add_remote_url_prompt", "Enter URL for '{name}'", { name: remoteName }),
      { title, placeholder: "https://github.com/user/repo" }
    );
    if (!urlInput || !urlInput.trim()) return;

    try {
      await runTool("git_remote_add", { name: remoteName, url: urlInput.trim() });
      await refreshGitPullStatus({ force: true });
      toolsLogNotice(getUIText("git_add_remote_done", "Remote '{name}' added/updated", { name: remoteName }), { label: "git_remote_add" });
    } catch (e) {
      showError("git_remote_add", e);
    }
  });

  bindOnce("btnGitLog", async () => {
    try {
      // 1. 터미널 출력
      await runTool("git_log", { count: 20 });

      // 2. 팝업 UI 표시를 위한 데이터 다시 로드
      const res = await postJson("/api/tools", { action: "git_log", count: 20 });
      if (!res.ok) throw new Error(res.error || "Failed to load git log");
      const commits = res.commits || [];
      const currentCommit = res.current_commit || "";
      if (!commits.length) {
        toolsLogNotice("No commits found", { label: "git_log" });
        return;
      }

      const selected = await openAppDialog({
        mode: "choice",
        title: "git log",
        message: getUIText("git_log_checkout_prompt", "Select commit to checkout"),
        cancelLabel: UI_STRINGS[LANG].cancel || "Cancel",
        choices: commits.map(c => {
          const isCurrent = currentCommit && c.hash.startsWith(currentCommit);
          const badgeHtml = isCurrent
            ? ` <span class="app-branch-picker__badge">${getUIText("branch_current", "Current")}</span>`
            : "";
          return {
            labelHtml: `<span class="app-branch-picker__label"><span style="color:#4caf50;font-weight:700;font-family:monospace;margin-right:8px;">${escapeHtml(c.hash)}</span>${escapeHtml(c.message)}</span>${badgeHtml}`,
            value: c.hash,
            className: isCurrent ? "is-current" : "",
          };
        }),
      });
      if (!selected) return;

      const confirmMsg = `${getUIText("git_log_checkout_confirm", "Checkout this commit?")}\n\n${selected}`;
      if (!await appConfirm(confirmMsg, { title: "git checkout" })) return;

      const resetRes = await postJson("/api/tools", { action: "git_reset", mode: "hard", target: selected });
      if (!resetRes.ok) throw new Error(resetRes.error || "Reset failed");
      
      toolsLogNotice(getUIText("git_log_checkout_done", "Checkout complete"), { label: "git_log" });
      await refreshToolsMetaInfo();
      await refreshGitPullStatus({ force: true });
    } catch (e) {
      showError("git_log", e);
    }
  });

  bindOnce("btnGitResetRepo", async () => {
    const title = getUIText("git_reset_repo_title", "Reset Repository");
    const msg = getUIText(
      "git_reset_repo_confirm",
      "Warning: This will remove origin and re-add 'ajouatom/openpilot'.\nAll local changes will be lost. Proceed?"
    );
    
    if (!await appConfirm(msg, { title, danger: true })) return;

    try {
      // Phase 1: fetch remote and get branch list
      const fetchResult = await runTool("git_reset_repo_fetch");
      const branches = fetchResult.branches || [];
      if (!branches.length) {
        toolsLogNotice(getUIText("git_reset_repo_no_branches", "No branches found"), { label: "git_reset_repo" });
        return;
      }

      // Phase 2: let user pick a branch
      const selected = await openAppDialog({
        mode: "choice",
        title: getUIText("branch_select", "Select Branch"),
        message: getUIText("git_reset_repo_branch_message", "Select branch to reset to"),
        cancelLabel: UI_STRINGS[LANG].cancel || "Cancel",
        choices: branches.map(b => ({ label: b, value: b })),
      });
      if (!selected) return;

      // Phase 3: checkout selected branch
      await runTool("git_reset_repo_checkout", { branch: selected });
      toolsLogNotice(getUIText("git_reset_repo_done", "Reset to '{branch}' complete", { branch: selected }), { label: "git_reset_repo" });
      await refreshToolsMetaInfo();
      await refreshGitPullStatus({ force: true });

      if (await appConfirm(UI_STRINGS[LANG].confirm_reboot || "Reboot now?", {
        title: UI_STRINGS[LANG].reboot || "Reboot",
      })) {
        await runTool("reboot");
      }
    } catch (e) {
      showError("git_reset_repo", e);
    }
  });

  bindOnce("btnResetCalib", async () => {
    const title = getUIText("reset_calib_title", "ReCalibration");
    const msg = getUIText("reset_calib_confirm", "Are you sure you want to reset calibration?\nDevice will reboot automatically.");
    if (!await appConfirm(msg, { title })) return;
    try {
      await runTool("reset_calib");
    } catch (e) {
      showError("reset_calib", e);
    }
  });

  bindOnce("btnDeviceLang", async () => {
    const choices = [
      { label: "한국어", value: "main_ko" },
      { label: "English", value: "main_en" },
      { label: "中文(简体)", value: "main_zh-CHS" },
      { label: "中文(繁體)", value: "main_zh-CHT" },
      { label: "日本語", value: "main_ja" },
      { label: "Deutsch", value: "main_de" },
      { label: "Français", value: "main_fr" },
      { label: "Português", value: "main_pt-BR" },
      { label: "Español", value: "main_es" },
      { label: "Türkçe", value: "main_tr" },
      { label: "العربية", value: "main_ar" },
      { label: "ไทย", value: "main_th" },
    ];
    const val = await openAppDialog({
      mode: "choice",
      title: getUIText("device_lang", "Device Language"),
      message: getUIText("device_lang_select_prompt", "Select language for the device UI"),
      cancelLabel: UI_STRINGS[LANG]?.cancel || "Cancel",
      choices
    });
    if (!val) return;
    try {
      await setParam("LanguageSetting", val);
      const rebootMsg = getUIText("setting_changed_reboot", "Setting changed. Reboot now?");
      if (await appConfirm(rebootMsg, { title: getUIText("reboot", "Reboot") })) {
        await runTool("reboot");
      }
    } catch (e) {
      showError("shell_cmd", e);
    }
  });


  bindOnce("btnSendTmuxLog", async () => {
    try {
      const j = await runTool("send_tmux_log");
      if (j.file) {
        window.location.href = j.file;
      }
    } catch (e) {
      showError("send_tmux_log", e);
    }
  });

  bindOnce("btnSendTmuxServerLog", async () => {
    try {
      const j = await runTool("server_tmux_log");
      if (j.file) {
        window.location.href = j.file;
      }
    } catch (e) {
      showError("server_tmux_log", e);
    }
  });

  bindOnce("btnInstallRequired", async () => {
    try {
      const j = await runTool("install_required");

      let summary = "";
      if (j.results && Array.isArray(j.results)) {
        const lines = j.results.map(r => `${r.package}: ${r.status}`);
        summary = lines.join("\n");
      }
      if (summary.trim()) toolsOutAppend(summary);

      if (j.need_reboot) {
        const yes = await appConfirm(UI_STRINGS[LANG].confirm_reboot_after_install, {
          title: UI_STRINGS[LANG].reboot || "Reboot",
        });
        if (yes) {
          await runTool("reboot");
        }
      }
    } catch (e) {
      showError("install_required", e);
    }
  });

  bindOnce("btnDeleteVideos", async () => {
    if (!await appConfirm(UI_STRINGS[LANG].delete_videos_confirm || "Delete ALL videos?", { title: "delete videos" })) return;
    try {
      await runTool("delete_all_videos");
    } catch (e) {
      showError("delete_all_videos", e);
    }
  });

  bindOnce("btnDeleteLogs", async () => {
    if (!await appConfirm(UI_STRINGS[LANG].delete_logs_confirm || "Delete ALL logs?", { title: "delete logs" })) return;
    try {
      await runTool("delete_all_logs");
    } catch (e) {
      showError("delete_all_logs", e);
    }
  });

  bindOnce("btnRebuildAll", async () => {
    if (!await appConfirm(UI_STRINGS[LANG].rebuild_confirm || "Rebuild all?", { title: "Rebuild All" })) return;
    try {
      await runTool("rebuild_all");
    } catch (e) {
      showError("rebuild_all", e);
    }
  });

  bindOnce("btnBackupSettings", async () => {
    try {
      const j = await runTool("backup_settings");
      if (j.file) window.location.href = j.file;
    } catch (e) {
      showError("backup_settings", e);
    }
  });

  bindOnce("btnRestoreSettings", async () => {
    const inp = document.createElement("input");
    inp.type = "file";
    inp.accept = "application/json,text/plain,*/*";
    inp.style.display = "none";

    inp.onchange = async () => {
      if (!inp.files || !inp.files[0]) return;

      if (!await appConfirm(UI_STRINGS[LANG].restore_confirm || "Restore settings from file?", {
        title: UI_STRINGS[LANG].restore || "Restore",
      })) {
        return;
      }

      let activityId = null;
      try {
        const labels = getActionLabel("backup_settings");
        activityId = typeof beginAppActivity === "function"
          ? beginAppActivity("tools", labels.running || "restore settings")
          : null;
        toolsMetaSet(labels.running);
        toolsOutCommitCurrent();
        toolsOutSet(`> restore settings\n${labels.running}`);
        toolsProgressSet(null, { active: true, indeterminate: true });

        const fd = new FormData();
        fd.append("file", inp.files[0]);

        const r = await fetch("/api/params_restore", { method: "POST", body: fd });
        const j = await r.json().catch(() => ({}));
        if (!r.ok || !j.ok) throw new Error(friendlyError(j) || j.error || ("HTTP " + r.status));

        toolsMetaSet(labels.done);
        toolsProgressSet(100, { active: true, indeterminate: false });
        toolsOutSet(`> restore settings\n${JSON.stringify(j.result, null, 2)}`);
        toolsOutCommitCurrent();
        if (activityId && typeof endAppActivity === "function") {
          endAppActivity(activityId);
          activityId = null;
        }

        if (await appConfirm(UI_STRINGS[LANG].restore_done_reboot || "Restore done.\nReboot now?", {
          title: UI_STRINGS[LANG].reboot || "Reboot",
        })) {
          await runTool("reboot");
        }
      } catch (e) {
        showError("backup_settings", e);
      } finally {
        if (activityId && typeof endAppActivity === "function") endAppActivity(activityId);
        window.setTimeout(() => toolsProgressSet(null, { active: false }), 900);
        inp.remove();
      }
    };

    document.body.appendChild(inp);
    inp.click();
  });

  function getAllSettingNames(settingsObj) {
    if (!settingsObj || !settingsObj.items_by_group) return [];
    const names = [];
    Object.values(settingsObj.items_by_group).forEach(list => {
      list.forEach(item => names.push(item.name));
    });
    return names;
  }

  async function buildSettingsJsonText() {
    if (!SETTINGS || !SETTINGS.items_by_group) {
      const r = await fetch("/api/settings");
      const j = await r.json();
      if (j.ok) SETTINGS = j;
    }
    if (!SETTINGS || !SETTINGS.items_by_group) {
      return { text: "", count: 0 };
    }
    const allNames = getAllSettingNames(SETTINGS);
    const values = await bulkGet(allNames);
    const orderedValues = {};
    allNames.forEach((name) => {
      orderedValues[name] = values[name] ?? "";
    });
    return {
      text: JSON.stringify(orderedValues, null, 2),
      count: allNames.length,
    };
  }

  bindOnce("btnCopySettings", async () => {
    try {
      const { text, count } = await buildSettingsJsonText();
      if (!text) {
        toolsLogNotice(getUIText("settings_not_loaded", "Settings not loaded"), { label: "copy settings" });
        return;
      }
      copyToClipboard(text);
      toolsLogNotice(getUIText("copy_settings_done", "{count} params copied", { count }), { label: "copy settings" });
    } catch (e) {
      showError("copy settings", e);
    }
  });

  bindOnce("btnViewSettings", async () => {
    try {
      const { text, count } = await buildSettingsJsonText();
      if (!text) {
        toolsLogNotice(getUIText("settings_not_loaded", "Settings not loaded"), { label: "view settings" });
        return;
      }
      appAlert(text, {
        title: getUIText("settings_title", "Settings ({count} params)", { count }),
        copyText: text,
      });
    } catch (e) {
      showError("view settings", e);
    }
  });

  bindOnce("btnReboot", async () => {
    if (!await appConfirm(UI_STRINGS[LANG].confirm_reboot || "Reboot now?", {
      title: UI_STRINGS[LANG].reboot || "Reboot",
    })) return;
    try {
      await runTool("reboot");
    } catch (e) {
      showError("reboot", e);
    }
  });

  const sysCmdForm = document.getElementById("sysCmdForm");
  const sysCmdInput = document.getElementById("sysCmdInput");
  bindNodeOnce(sysCmdForm, "submitBound", (ev) => {
    ev.preventDefault();
    runSystemCommand();
  }, "submit");

  const btnSysCmdInfo = document.getElementById("btnSysCmdInfo");
  const sysCmdHelpPanel = document.getElementById("sysCmdHelpPanel");
  const maybeShowSysCmdHint = () => {
    if (!sysCmdInput || !sysCmdHelpPanel || !sysCmdHelpPanel.hidden) return;
    const now = Date.now();
    const cooldownUntil = Number(sysCmdInput.dataset.hintCooldownUntil || "0");
    if (now < cooldownUntil) return;
    sysCmdInput.dataset.hintCooldownUntil = String(now + 3200);
    showAppToast(getUIText("sys_cmd_help", "Allowed: pull, status, branch, log, git ..., df, free, uptime"), {
      tone: "hint",
      duration: 1700,
    });
  };

  bindNodeOnce(sysCmdInput, "hintFocusBound", () => {
    if (!sysCmdInput.value.trim()) maybeShowSysCmdHint();
  }, "focus");

  bindNodeOnce(sysCmdInput, "hintInputBound", () => {
    if (sysCmdInput.value.trim().length === 1) maybeShowSysCmdHint();
  }, "input");

  bindNodeOnce(btnSysCmdInfo, "toggleBound", () => {
    if (!sysCmdHelpPanel) return;
    const nextOpen = sysCmdHelpPanel.hidden;
    sysCmdHelpPanel.hidden = !nextOpen;
    btnSysCmdInfo.setAttribute("aria-expanded", nextOpen ? "true" : "false");
  });
}

