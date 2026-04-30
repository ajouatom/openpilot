"use strict";

// Terminal page — tmux WebSocket client.

/* ---------- Terminal ---------- */
const terminalMetaEl = document.getElementById("terminalMeta");
const terminalSessionMetaEl = document.getElementById("terminalSessionMeta");
const terminalPageEl = document.getElementById("pageTerminal");
const terminalScreenEl = document.getElementById("terminalScreen");
const terminalOutputEl = document.getElementById("terminalOutput");
const terminalFormEl = document.getElementById("terminalForm");
const terminalInputEl = document.getElementById("terminalInput");
const btnTerminalCtrlCEl = document.getElementById("btnTerminalCtrlC");
const btnTerminalClearEl = document.getElementById("btnTerminalClear");
const btnTerminalReconnectEl = document.getElementById("btnTerminalReconnect");

let terminalWs = null;
let terminalReconnectTimer = null;
let terminalPageActive = false;
let terminalSessionName = "carrot-web";
let terminalLastScreen = "";
let terminalLayoutBound = false;
let terminalFollowOutput = true;
let terminalCurrentCwd = "/data/openpilot";
let terminalScrollRaf = 0;

function setTerminalMeta(text) {
  if (terminalMetaEl) terminalMetaEl.textContent = String(text || "");
}

function setTerminalSessionMeta(cwd = terminalCurrentCwd) {
  if (!terminalSessionMetaEl) return;
  terminalSessionMetaEl.textContent = String(cwd || "/data/openpilot");
}

function setTerminalSessionInfo(session = terminalSessionName) {
  terminalSessionName = session || terminalSessionName;
  setTerminalSessionMeta();
}

function sanitizeTerminalScreen(text) {
  let nextText = String(text || " ");
  const headLimit = Math.min(nextText.length, 640);
  const head = nextText.slice(0, headLimit);
  const sanitizedHead = head.replace(
    /[^\n]*\$\s*cd(?:\s+\/data\/openpilot)?\n(?:\/data\/openpilot\n)?(?=[^\n]*:\/data\/openpilot\$)/,
    "",
  );

  if (sanitizedHead !== head) {
    nextText = sanitizedHead + nextText.slice(headLimit);
    nextText = nextText.replace(/^\n+/, "");
  }

  const lines = nextText.replace(/\r/g, "").split("\n");
  while (lines.length > 1 && !lines[lines.length - 1].trim()) {
    lines.pop();
  }
  nextText = lines.join("\n");

  if (!nextText.trim()) return " ";
  return nextText;
}

function extractTerminalCwd(text) {
  const lines = String(text || "").split("\n");
  for (let i = lines.length - 1; i >= 0; i -= 1) {
    const line = lines[i].trim();
    if (!line) continue;
    const match = line.match(/^[^\s:@]+@[^\s:]+:(.+?)[#$]\s*$/);
    if (match) return match[1].trim();
  }
  return "";
}

function renderTerminalScreenMarkup(text) {
  return String(text || " ")
    .split("\n")
    .map((line) => {
      const match = line.match(/^([^\s:@]+@[^\s:]+)(?=:[^$]*\$ ?)/);
      if (!match) return escapeHtml(line);
      const promptHost = match[1];
      return `<span class="terminal-output__promptHost">${escapeHtml(promptHost)}</span>${escapeHtml(line.slice(promptHost.length))}`;
    })
    .join("\n");
}

function isTerminalPinnedToBottom() {
  if (!terminalScreenEl) return true;
  return (getTerminalBottomDistance() < 28);
}

function getTerminalBottomDistance() {
  if (!terminalScreenEl) return 0;
  return Math.max(0, terminalScreenEl.scrollHeight - terminalScreenEl.scrollTop - terminalScreenEl.clientHeight);
}

function getTerminalBottomScrollTop() {
  if (!terminalScreenEl) return 0;
  return Math.max(0, terminalScreenEl.scrollHeight - terminalScreenEl.clientHeight);
}

function pinTerminalToBottom(options = {}) {
  if (!terminalScreenEl) return;
  const { immediate = false } = options;
  const apply = () => {
    terminalScreenEl.scrollTop = getTerminalBottomScrollTop();
    updateTerminalOverflowState();
  };
  if (terminalScrollRaf) {
    cancelAnimationFrame(terminalScrollRaf);
    terminalScrollRaf = 0;
  }
  if (immediate) apply();
  terminalScrollRaf = requestAnimationFrame(() => {
    terminalScrollRaf = requestAnimationFrame(() => {
      terminalScrollRaf = 0;
      apply();
    });
  });
}

function updateTerminalOverflowState() {
  if (!terminalScreenEl || !terminalOutputEl) return;
  const overflowX = (terminalOutputEl.scrollWidth - terminalScreenEl.clientWidth) > 20;
  const atRight = (terminalScreenEl.scrollWidth - terminalScreenEl.scrollLeft - terminalScreenEl.clientWidth) < 8;
  terminalScreenEl.classList.toggle("is-x-overflow", overflowX && !atRight);
}

function clearTerminalViewport() {
  terminalLastScreen = "";
  if (terminalOutputEl) terminalOutputEl.innerHTML = "";
  updateTerminalOverflowState();
  pinTerminalToBottom({ immediate: true });
}

function setTerminalScreen(text, forceStick = false) {
  if (!terminalOutputEl) return;
  const nextText = sanitizeTerminalScreen(text);
  if (nextText === terminalLastScreen) return;

  const bottomDistance = getTerminalBottomDistance();
  const previousScrollLeft = terminalScreenEl?.scrollLeft || 0;
  const shouldStick = forceStick || terminalFollowOutput || isTerminalPinnedToBottom();
  terminalLastScreen = nextText;
  const nextCwd = extractTerminalCwd(nextText);
  if (nextCwd && nextCwd !== terminalCurrentCwd) {
    terminalCurrentCwd = nextCwd;
    setTerminalSessionMeta(nextCwd);
  }
  terminalOutputEl.innerHTML = renderTerminalScreenMarkup(nextText);
  requestAnimationFrame(() => {
    updateTerminalOverflowState();
    if (shouldStick) {
      pinTerminalToBottom();
      return;
    }
    if (terminalScreenEl) {
      terminalScreenEl.scrollTop = Math.max(0, terminalScreenEl.scrollHeight - terminalScreenEl.clientHeight - bottomDistance);
      terminalScreenEl.scrollLeft = previousScrollLeft;
    }
  });
}

function runTerminalLocalAlias(line) {
  const key = String.fromCharCode(119, 104, 101, 114, 101, 105, 115, 109, 121, 99, 97, 114, 114, 111, 116);
  if (String(line || "").trim().toLowerCase() !== key) return false;

  const msg = String.fromCharCode(45817, 44540, 33, 33);
  const evt = String.fromCharCode(99, 97, 114, 114, 111, 116, 58, 114, 117, 110, 58, 52, 48, 52);
  const base = terminalLastScreen && terminalLastScreen.trim()
    ? `${terminalLastScreen.replace(/\s+$/g, "")}\n`
    : "";
  terminalFollowOutput = true;
  setTerminalScreen(`${base}${msg}`, true);

  window.dispatchEvent(new CustomEvent(evt, {
    detail: { [String.fromCharCode(113)]: 1 },
  }));

  if (terminalInputEl) terminalInputEl.value = "";
  return true;
}

function clearTerminalReconnectTimer() {
  if (terminalReconnectTimer) {
    clearTimeout(terminalReconnectTimer);
    terminalReconnectTimer = null;
  }
}

function updateTerminalToastAnchor() {
  if (!terminalFormEl || document.body?.dataset?.page !== "terminal") {
    document.documentElement.style.removeProperty("--terminal-toast-bottom");
    document.documentElement.style.removeProperty("--terminal-toast-left");
    document.documentElement.style.removeProperty("--terminal-toast-width");
    return;
  }

  const rect = terminalFormEl.getBoundingClientRect();
  if (!rect.width || !rect.height) {
    document.documentElement.style.removeProperty("--terminal-toast-bottom");
    document.documentElement.style.removeProperty("--terminal-toast-left");
    document.documentElement.style.removeProperty("--terminal-toast-width");
    return;
  }
  const gap = 10;
  const vv = window.visualViewport;
  const viewportBottom = vv ? (vv.offsetTop + vv.height) : (window.innerHeight || 0);
  const offset = Math.max(0, Math.round(viewportBottom - rect.top + gap));
  document.documentElement.style.setProperty("--terminal-toast-bottom", `${offset}px`);
  document.documentElement.style.setProperty("--terminal-toast-left", `${Math.round(rect.left)}px`);
  document.documentElement.style.setProperty("--terminal-toast-width", `${Math.round(rect.width)}px`);
}

function updateTerminalViewportMetrics() {
  updateAppViewportMetrics();
  const vv = window.visualViewport;
  const landscapeRail = typeof isLandscapeRailMode === "function" && isLandscapeRailMode();
  const layoutHeight = Math.max(320, Math.round(window.innerHeight || vv?.height || 0));
  const height = Math.max(320, Math.round(vv?.height || window.innerHeight || 0));
  const top = Math.max(0, Math.round(vv?.offsetTop || 0));
  const keyboardInset = Math.max(0, Math.round(layoutHeight - height - top));
  const keyboardOpen = !landscapeRail && keyboardInset > 120;
  const desktopBottomNav = window.matchMedia?.("(min-width: 769px)")?.matches;
  const restingBottomGap = landscapeRail
    ? `calc(14px + env(safe-area-inset-bottom, 0px))`
    : `calc(var(--nav-bar-height${desktopBottomNav ? "-desktop" : ""}) + env(safe-area-inset-bottom, 0px))`;
  const restingFormBottom = landscapeRail
    ? `max(10px, env(safe-area-inset-bottom, 0px))`
    : `calc(8px + env(safe-area-inset-bottom, 0px))`;
  document.documentElement.style.setProperty("--terminal-vv-height", `${height}px`);
  document.documentElement.style.setProperty("--terminal-vv-top", `${top}px`);
  const layoutStyle = terminalPageEl?.style || document.documentElement.style;
  layoutStyle.setProperty(
    "--terminal-bottom-gap",
    keyboardOpen
      ? `calc(6px + env(safe-area-inset-bottom, 0px))`
      : restingBottomGap,
  );
  layoutStyle.setProperty(
    "--terminal-form-bottom",
    keyboardOpen
      ? `calc(6px + env(safe-area-inset-bottom, 0px))`
      : restingFormBottom,
  );
  document.documentElement.classList.toggle("terminal-keyboard-open", keyboardOpen);
}

function bindTerminalLayoutObservers() {
  if (terminalLayoutBound) return;
  terminalLayoutBound = true;

  const handleLayout = () => requestAnimationFrame(() => {
    updateTerminalViewportMetrics();
    updateTerminalToastAnchor();
    updateTerminalOverflowState();
    if (terminalFollowOutput) pinTerminalToBottom();
  });
  window.addEventListener("resize", handleLayout, { passive: true });
  window.addEventListener("orientationchange", handleLayout, { passive: true });
  if (window.visualViewport) {
    window.visualViewport.addEventListener("resize", handleLayout, { passive: true });
    window.visualViewport.addEventListener("scroll", handleLayout, { passive: true });
  }
}

function closeTerminalSocket() {
  if (!terminalWs) return;
  const ws = terminalWs;
  terminalWs = null;
  try {
    ws.onopen = null;
    ws.onmessage = null;
    ws.onclose = null;
    ws.onerror = null;
    if (ws.readyState === WebSocket.OPEN || ws.readyState === WebSocket.CONNECTING) ws.close();
  } catch (e) {
    console.log("[Terminal] ws close failed:", e);
  }
}

function getTerminalWsUrl() {
  const proto = location.protocol === "https:" ? "wss" : "ws";
  return `${proto}://${location.host}/ws/terminal?session=${encodeURIComponent(terminalSessionName)}`;
}

function scheduleTerminalReconnect(delay = 1200) {
  clearTerminalReconnectTimer();
  if (!terminalPageActive) return;
  setTerminalMeta(getUIText("reconnecting", "reconnecting..."));
  terminalReconnectTimer = window.setTimeout(() => {
    terminalReconnectTimer = null;
    connectTerminal();
  }, delay);
}

function sendTerminalPacket(payload, options = {}) {
  const { quiet = false } = options;
  if (!terminalWs || terminalWs.readyState !== WebSocket.OPEN) {
    if (!quiet) showAppToast(getUIText("terminal_offline", "terminal offline"), { tone: "error" });
    return false;
  }

  try {
    terminalWs.send(JSON.stringify(payload));
    return true;
  } catch (e) {
    if (!quiet) showAppToast(e.message || "Terminal send failed", { tone: "error" });
    return false;
  }
}

function sendTerminalControl(action, options = {}) {
  return sendTerminalPacket({ type: "control", action }, options);
}

function connectTerminal(force = false) {
  clearTerminalReconnectTimer();

  if (terminalWs && (terminalWs.readyState === WebSocket.OPEN || terminalWs.readyState === WebSocket.CONNECTING)) {
    if (!force) return;
    closeTerminalSocket();
  }

  setTerminalMeta(getUIText("connecting", "connecting..."));

  let ws;
  try {
    ws = new WebSocket(getTerminalWsUrl());
  } catch (e) {
    setTerminalMeta(e.message || getUIText("terminal_unavailable", "terminal unavailable"));
    scheduleTerminalReconnect(1600);
    return;
  }

  terminalWs = ws;

  ws.onopen = () => {
    if (terminalWs !== ws) return;
    setTerminalMeta(getUIText("connecting", "connecting..."));
  };

  ws.onmessage = (ev) => {
    if (terminalWs !== ws) return;

    let data;
    try {
      data = JSON.parse(ev.data);
    } catch (e) {
      return;
    }

    if (data.type === "meta") {
      setTerminalSessionInfo(data.session || terminalSessionName);
      setTerminalMeta(data.created ? getUIText("terminal_ready", "tmux ready") : getUIText("connected", "connected"));
      return;
    }

    if (data.type === "screen") {
      setTerminalScreen(data.text, false);
      if (terminalMetaEl && terminalMetaEl.textContent === getUIText("connecting", "connecting...")) {
        setTerminalMeta(getUIText("connected", "connected"));
      }
      return;
    }

    if (data.type === "error") {
      const errorText = String(data.error || getUIText("error", "Error"));
      setTerminalMeta(errorText);
      showAppToast(errorText, { tone: "error" });
    }
  };

  ws.onclose = () => {
    if (terminalWs !== ws) return;
    terminalWs = null;
    if (!terminalPageActive) return;
    setTerminalMeta(getUIText("terminal_disconnected", "disconnected"));
    scheduleTerminalReconnect(1250);
  };

  ws.onerror = () => {
    if (terminalWs !== ws) return;
    setTerminalMeta(getUIText("terminal_unavailable", "terminal unavailable"));
  };
}

function initTerminalBindings() {
  const bindNodeOnce = (node, key, fn, eventName = "click") => {
    if (!node || node.dataset[key] === "1") return;
    node.dataset[key] = "1";
    node.addEventListener(eventName, fn);
  };

  bindTerminalLayoutObservers();

  if (terminalInputEl) {
    terminalInputEl.autocomplete = "off";
    terminalInputEl.autocapitalize = "none";
    terminalInputEl.spellcheck = false;
    terminalInputEl.setAttribute("autocorrect", "off");
    terminalInputEl.setAttribute("enterkeyhint", "send");
  }

  bindNodeOnce(terminalScreenEl, "scrollBound", () => {
    terminalFollowOutput = isTerminalPinnedToBottom();
    updateTerminalOverflowState();
  }, "scroll");

  bindNodeOnce(terminalFormEl, "submitBound", (ev) => {
    ev.preventDefault();
    const line = (terminalInputEl?.value || "").trim();
    if (!line) return;
    if (runTerminalLocalAlias(line)) return;
    terminalFollowOutput = isTerminalPinnedToBottom();
    if (sendTerminalPacket({ type: "input", data: line })) {
      terminalInputEl.value = "";
    }
  }, "submit");

  bindNodeOnce(btnTerminalCtrlCEl, "clickBound", () => {
    terminalFollowOutput = isTerminalPinnedToBottom();
    sendTerminalControl("ctrl_c");
  });

  bindNodeOnce(btnTerminalClearEl, "clickBound", () => {
    terminalFollowOutput = true;
    clearTerminalViewport();
    sendTerminalControl("clear");
  });

  bindNodeOnce(btnTerminalReconnectEl, "clickBound", () => {
    terminalFollowOutput = isTerminalPinnedToBottom();
    const metaText = String(terminalMetaEl?.textContent || "");
    const blockedByPassword = terminalLastScreen.includes("Password:");
    const blockedByStartup = metaText.includes("returned non-zero exit status");
    if ((blockedByPassword || blockedByStartup) && sendTerminalControl("new_session", { quiet: true })) {
      setTerminalMeta(getUIText("connecting", "connecting..."));
      return;
    }
    connectTerminal(true);
  });
}

function initTerminalPage() {
  terminalPageActive = true;
  terminalFollowOutput = true;
  terminalCurrentCwd = "/data/openpilot";
  initTerminalBindings();
  setTerminalSessionMeta();
  updateTerminalViewportMetrics();
  if (!terminalLastScreen) setTerminalScreen(" ", true);
  requestAnimationFrame(updateTerminalToastAnchor);
  requestAnimationFrame(updateTerminalOverflowState);
  window.setTimeout(updateTerminalToastAnchor, 90);
  connectTerminal(false);
}

function teardownTerminalPage() {
  terminalPageActive = false;
  clearTerminalReconnectTimer();
  closeTerminalSocket();
  document.documentElement.style.removeProperty("--terminal-vv-height");
  document.documentElement.style.removeProperty("--terminal-vv-top");
  const layoutStyle = terminalPageEl?.style || document.documentElement.style;
  layoutStyle.removeProperty("--terminal-bottom-gap");
  layoutStyle.removeProperty("--terminal-form-bottom");
  document.documentElement.style.removeProperty("--terminal-toast-bottom");
  document.documentElement.style.removeProperty("--terminal-toast-left");
  document.documentElement.style.removeProperty("--terminal-toast-width");
  document.documentElement.classList.remove("terminal-keyboard-open");
}
