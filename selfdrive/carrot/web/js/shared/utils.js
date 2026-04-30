"use strict";

// State that the rest of the app shares as plain globals.
// Page logic mutates these — they live here because P8 has no dedicated state file.
let SETTINGS = null;
let CURRENT_GROUP = null;
let CARS = null;                 // { makers: {Hyundai:[...], Genesis:[...]} }
let CURRENT_MAKER = null;
let BRANCHES = [];
let CURRENT_BRANCH_NAME = "";

// Quick link state
let QUICK_LINK_URL = QUICK_LINK_FIXED_URL;
let QUICK_LINK_STATUS = "loading";
let QUICK_LINK_MESSAGE = "";
let quickLinkLoadPromise = null;
let quickLinkLoadedAt = 0;
let quickLinkActionTimer = null;


const PAGE_DATA_TTL_MS = 15000;

function hasFreshPageData(lastLoadedAt, ttlMs = PAGE_DATA_TTL_MS) {
  return lastLoadedAt > 0 && (Date.now() - lastLoadedAt) < ttlMs;
}

function requestIdleTask(callback, timeout = 900) {
  if (typeof window.requestIdleCallback === "function") {
    return window.requestIdleCallback(callback, { timeout });
  }
  return window.setTimeout(callback, 1);
}


function escapeHtml(s) {
  return String(s)
    .replaceAll("&", "&amp;")
    .replaceAll("<", "&lt;")
    .replaceAll(">", "&gt;")
    .replaceAll('"', "&quot;")
    .replaceAll("'", "&#039;");
}

function formatItemText(p, keyKo, keyEn, fallback = "") {
  if (LANG === "zh") return (p["c" + keyEn.slice(1)] || p[keyEn] || p[keyKo] || fallback);
  if (LANG === "ko") return (p[keyKo] || p[keyEn] || fallback);
  return (p[keyEn] || p[keyKo] || fallback);
}

function clamp(v, mn, mx) {
  if (Number.isFinite(mn) && v < mn) return mn;
  if (Number.isFinite(mx) && v > mx) return mx;
  return v;
}

function copyToClipboard(text) {
  if (navigator.clipboard && window.isSecureContext) {
    navigator.clipboard.writeText(text).catch(() => {});
    return;
  }
  const ta = document.createElement("textarea");
  ta.value = text;
  ta.style.cssText = "position:fixed;top:-9999px;opacity:0";
  document.body.appendChild(ta);
  ta.focus();
  ta.select();
  try { document.execCommand("copy"); } catch {}
  document.body.removeChild(ta);
}


/* ── Quick Link helpers ──────────────────────────────── */
function syncHomeUtilityButtons() {
  return;
}

function flashQuickLinkActionLabel(label, duration = 1400) {
  if (!btnSaveQuickLink) return;
  if (quickLinkActionTimer) clearTimeout(quickLinkActionTimer);
  btnSaveQuickLink.textContent = label;
  quickLinkActionTimer = window.setTimeout(() => {
    quickLinkActionTimer = null;
    syncHomeUtilityButtons();
  }, duration);
}

function renderQuickLinkUI() {
  const hasUrl = Boolean(QUICK_LINK_URL);
  const emptyMessage = QUICK_LINK_MESSAGE || getUIText("quick_link_empty", "GithubUsername not set");
  const loadingMessage = getUIText("connecting", "Connecting...");
  const errorMessage = QUICK_LINK_MESSAGE || getUIText("error", "Error");
  const inlineText = hasUrl
    ? QUICK_LINK_URL
    : (
      QUICK_LINK_STATUS === "loading"
        ? loadingMessage
        : (QUICK_LINK_STATUS === "error" ? errorMessage : emptyMessage)
    );

  if (quickLink) {
    if (hasUrl) {
      quickLink.href = QUICK_LINK_URL;
      quickLink.textContent = inlineText;
      quickLink.removeAttribute("aria-disabled");
    } else {
      quickLink.removeAttribute("href");
      quickLink.setAttribute("aria-disabled", "true");
      quickLink.textContent = inlineText;
    }
  }

  if (btnSaveQuickLink) {
    btnSaveQuickLink.disabled = !hasUrl;
    btnSaveQuickLink.setAttribute("aria-disabled", hasUrl ? "false" : "true");
  }

  if (btnQuickLinkWeb) {
    if (hasUrl) {
      btnQuickLinkWeb.href = QUICK_LINK_URL;
      btnQuickLinkWeb.setAttribute("aria-disabled", "false");
    } else {
      btnQuickLinkWeb.removeAttribute("href");
      btnQuickLinkWeb.setAttribute("aria-disabled", "true");
    }
  }
}

function setServerStateStatus() {}

async function updateQuickLink(options = {}) {
  const silent = options.silent === true;
  QUICK_LINK_URL = QUICK_LINK_FIXED_URL;
  QUICK_LINK_STATUS = "ready";
  QUICK_LINK_MESSAGE = "";
  quickLinkLoadPromise = null;
  quickLinkLoadedAt = Date.now();
  if (!silent || CURRENT_PAGE === "tools") renderQuickLinkUI();
  return QUICK_LINK_URL;
}

async function openQuickLink() {
  QUICK_LINK_URL = QUICK_LINK_FIXED_URL;
  renderQuickLinkUI();
  const msg = `${getUIText("open_carrotman_confirm", "Open {name}?", { name: "CarrotMan" })}\n\n${QUICK_LINK_FIXED_URL}`;
  const ok = await appConfirm(msg, { title: "CarrotMan" });
  if (!ok) return;
  window.open(QUICK_LINK_FIXED_URL, "_blank", "noopener");
}
