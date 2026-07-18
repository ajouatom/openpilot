"use strict";

// Terminal page tmux WebSocket client.

/* ---------- Terminal ---------- */
const terminalMetaEl = document.getElementById("terminalMeta");
const terminalSessionMetaEl = document.getElementById("terminalSessionMeta");
const terminalPageEl = document.getElementById("pageTerminal");
const terminalScreenEl = document.getElementById("terminalScreen");
const terminalOutputEl = document.getElementById("terminalOutput");
const terminalKeysEl = document.getElementById("terminalKeys");
const btnTerminalCtrlCEl = document.getElementById("btnTerminalCtrlC");
const btnTerminalClearEl = document.getElementById("btnTerminalClear");
const btnTerminalReconnectEl = document.getElementById("btnTerminalReconnect");
const terminalXtermEl = document.getElementById("terminalXterm");

let terminalWs = null;
let terminalReconnectTimer = null;
let terminalResetPending = false;
let terminalPageActive = false;
let terminalSessionName = "";
let terminalLastScreen = "";
let terminalPtyBuffer = "";
let terminalLayoutBound = false;
let terminalFollowOutput = true;
let terminalCurrentCwd = "/data/openpilot";
let terminalScrollRaf = 0;
const terminalUsePty = true;

// Real terminal emulation via xterm.js (grid renderer): interprets cursor
// moves / clears / colors / alternate-screen, so full-screen TUIs (btop, vim,
// nested tmux) render correctly instead of the naive append-only fallback.
// Falls back to the legacy <pre> renderer if xterm.js failed to load.
let terminalXterm = null;
let terminalXtermActive = false;
let terminalXtermResizeObserver = null;
let terminalXtermFitRaf = 0;
let terminalKeyboardFitTimers = [];
let terminalRendererAddon = null;
let terminalRendererKind = "dom";
let terminalRendererDpr = 0;
let terminalRendererVerifiedDpr = 0;
let terminalRendererScaleCheckTimer = 0;
let terminalDprMediaQuery = null;
let terminalOuterScrollRaf = 0;
let terminalOuterScrollTimer = 0;
let terminalOuterScrollLockUntil = 0;
let terminalSuppressScrollUntil = 0;
let terminalCtrlSticky = false;
let terminalSuppressDataDepth = 0;
let terminalLayoutRaf = 0;
let terminalKeysTouchStart = null;
let terminalXtermTouchStartY = null;
let terminalLastSizeKey = "";
const TERMINAL_GRID_COLS = 100;
const TERMINAL_GRID_ROWS = 30;

function enterTerminalKeyboardMode() {
  window.CarrotViewport?.setVirtualKeyboardOverlaysContent?.(true);
  window.CarrotViewport?.updateMetrics?.();
}

function leaveTerminalKeyboardMode() {
  window.CarrotViewport?.setVirtualKeyboardOverlaysContent?.(true);
  window.CarrotViewport?.updateMetrics?.();
}

// Samsung Internet can scroll the outer, horizontally-pannable xterm host to
// reveal xterm's focused helper textarea when the native keyboard opens. That
// moves the whole rendered grid upward while xterm's own buffer stays correct.
// The host deliberately has no vertical scrolling contract, so keep it at 0.
function lockTerminalOuterVerticalScroll() {
  if (Date.now() > terminalOuterScrollLockUntil) return;
  if (terminalXtermEl && terminalXtermEl.scrollTop !== 0) terminalXtermEl.scrollTop = 0;
}

function scheduleTerminalOuterVerticalScrollLock() {
  terminalOuterScrollLockUntil = Date.now() + 800;
  lockTerminalOuterVerticalScroll();
  if (terminalOuterScrollRaf) cancelAnimationFrame(terminalOuterScrollRaf);
  if (terminalOuterScrollTimer) clearTimeout(terminalOuterScrollTimer);
  terminalOuterScrollRaf = requestAnimationFrame(() => {
    terminalOuterScrollRaf = 0;
    lockTerminalOuterVerticalScroll();
    requestAnimationFrame(lockTerminalOuterVerticalScroll);
  });
  terminalOuterScrollTimer = window.setTimeout(() => {
    terminalOuterScrollTimer = 0;
    lockTerminalOuterVerticalScroll();
  }, 620);
}

// Output can arrive while a touch scroll is still in progress. Mark an actual
// vertical drag immediately so a queued layout/output pin cannot pull the
// viewport back to the bottom before xterm emits its scroll event.
function trackTerminalXtermTouch(event) {
  const touch = event.touches?.[0];
  if (!touch || terminalXtermTouchStartY == null) return;
  if (Math.abs(touch.clientY - terminalXtermTouchStartY) < 6) return;
  terminalFollowOutput = false;
  terminalSuppressScrollUntil = 0;
}

function pinTerminalCursorToBottom() {
  if (!terminalXtermActive || !terminalXterm || !terminalFollowOutput) return;
  try {
    terminalSuppressScrollUntil = Date.now() + 350;
    terminalXterm.scrollToBottom();
  } catch (e) {
    try {
      terminalXterm.scrollToBottom();
    } catch (ignore) {}
  }
}

// Raw escape sequences for the on-screen key bar (Esc/Tab/arrows) so touch
// devices that have no physical Esc/Ctrl/arrow keys can still drive
// interactive programs (vim, btop, less) that the shell input box cannot.
const TERMINAL_KEY_SEQ = {
  esc: "\x1b",
  tab: "\t",
  ctrl_c: "\x03",
  ctrl_d: "\x04",
  // tmux detach: AGNOS prefix is backtick, so ` then d (used to leave `tmux a`).
  detach: "\x60d",
  home: "\x1b[H",
  end: "\x1b[F",
  page_up: "\x1b[5~",
  page_down: "\x1b[6~",
  up: "\x1b[A",
  down: "\x1b[B",
  right: "\x1b[C",
  left: "\x1b[D",
};

const terminalTextDecoder = (typeof TextDecoder === "function") ? new TextDecoder("utf-8") : null;
const TERMINAL_WEB_ACTION_PREFIX = "[[CARROT_WEB_ACTION:";
const TERMINAL_WEB_ACTION_SUFFIX = "]]";
let terminalWebActionCarry = "";

function runTerminalWebAction(action) {
  if (action !== "web-intro") return;
  if (!terminalPageActive || document.visibilityState === "hidden") return;
  if (typeof window.CarrotIntroShell?.open !== "function") {
    showAppToast("인트로를 불러오지 못했습니다.", { tone: "error" });
    return;
  }
  window.CarrotIntroShell.open({ preview: true });
}

/* Custom commands can request a browser-only action without adding another
   server state/polling channel. The marker is removed before xterm renders it;
   history replay is consumed but deliberately never dispatched again. */
function consumeTerminalWebActions(chunk, { dispatch = true } = {}) {
  let text = terminalWebActionCarry + String(chunk || "");
  terminalWebActionCarry = "";
  let output = "";

  while (text) {
    const start = text.indexOf(TERMINAL_WEB_ACTION_PREFIX);
    if (start < 0) {
      let carryLength = 0;
      const max = Math.min(text.length, TERMINAL_WEB_ACTION_PREFIX.length - 1);
      for (let length = max; length > 0; length -= 1) {
        if (text.endsWith(TERMINAL_WEB_ACTION_PREFIX.slice(0, length))) {
          carryLength = length;
          break;
        }
      }
      if (carryLength) {
        output += text.slice(0, -carryLength);
        terminalWebActionCarry = text.slice(-carryLength);
      } else {
        output += text;
      }
      break;
    }

    output += text.slice(0, start);
    const end = text.indexOf(TERMINAL_WEB_ACTION_SUFFIX, start + TERMINAL_WEB_ACTION_PREFIX.length);
    if (end < 0) {
      terminalWebActionCarry = text.slice(start);
      break;
    }

    const action = text.slice(start + TERMINAL_WEB_ACTION_PREFIX.length, end).trim();
    if (dispatch) window.setTimeout(() => runTerminalWebAction(action), 0);
    text = text.slice(end + TERMINAL_WEB_ACTION_SUFFIX.length);
  }

  return output;
}

function base64ToBytes(b64) {
  const bin = atob(String(b64 || ""));
  const len = bin.length;
  const bytes = new Uint8Array(len);
  for (let i = 0; i < len; i += 1) bytes[i] = bin.charCodeAt(i);
  return bytes;
}

function readCssVar(name, fallback) {
  try {
    const v = getComputedStyle(document.documentElement).getPropertyValue(name).trim();
    return v || fallback;
  } catch (e) {
    return fallback;
  }
}

function terminalXtermSupported() {
  return !!(terminalXtermEl
    && typeof window.Terminal === "function");
}

// Scale the font so the fixed 100-column grid fills the width on any screen
// (a phone gets a smaller font, a desktop a larger one). Uses xterm's own cell
// measurement (FitAddon) instead of an estimated advance ratio: columns are
// inversely proportional to font size, so one linear step lands on 100 cols.
// Clamped so it stays legible — a very narrow phone floors and the container
// pans horizontally (.terminal-xterm overflow-x / touch pan-x).
// Fixed, readable cell size (auto-scaling to fill the width made it too small).
// The 100-column grid keeps a consistent wrap width; when it is wider than a
// narrow screen the container pans horizontally instead of shrinking the text.
function terminalFontSize() {
  const w = window.innerWidth || 800;
  if (w <= 380) return 11;
  if (w <= 640) return 12;
  return 13;
}

// The host paints an opaque dark surface, so xterm never needs a see-through
// background here. Reading the host's resolved color keeps the grid identical
// in light/dark while letting us run xterm opaque — required because the WebGL
// renderer thins glyphs badly whenever allowTransparency is on (xtermjs #4212).
function terminalOpaqueBackground() {
  try {
    const bg = terminalXtermEl ? getComputedStyle(terminalXtermEl).backgroundColor : "";
    if (bg && bg !== "transparent" && !/rgba?\([^)]*,\s*0\s*\)\s*$/i.test(bg)) return bg;
  } catch (e) {}
  return readCssVar("--md-surface", "#0b0f14");
}

// The default DOM renderer re-renders every visible row (<div> textContent) on
// each scroll frame, which stutters on the device — worst on rows with heavy
// ANSI styling (many colored spans). Paint to a single GPU surface instead:
// WebGL first, Canvas as a fallback, and keep the DOM renderer if neither addon
// activates (no regression). Each step logs its outcome so "why is it still the
// DOM renderer" is answerable from the console.
// The UMD builds expose the addon global as a namespace object
// (window.WebglAddon === { WebglAddon: class }), not the class itself, so a
// plain `new window.WebglAddon()` fails. Resolve the constructor from either
// shape (direct class, or `.<name>` on the namespace).
function resolveTerminalAddon(name) {
  const g = window[name];
  if (typeof g === "function") return g;
  if (g && typeof g[name] === "function") return g[name];
  return null;
}

function currentTerminalDevicePixelRatio() {
  const dpr = Number(window.devicePixelRatio);
  return Number.isFinite(dpr) && dpr > 0 ? dpr : 1;
}

function rememberTerminalRenderer(addon, kind) {
  terminalRendererAddon = addon || null;
  terminalRendererKind = kind;
  terminalRendererDpr = currentTerminalDevicePixelRatio();
}

function disposeTerminalRendererAddon() {
  const addon = terminalRendererAddon;
  terminalRendererAddon = null;
  terminalRendererKind = "dom";
  if (!addon || typeof addon.dispose !== "function") return;
  try { addon.dispose(); } catch {}
}

function attachTerminalCanvasRenderer(term) {
  let addon = null;
  try {
    const Canvas = resolveTerminalAddon("CanvasAddon");
    if (!Canvas) {
      console.log("[Terminal] CanvasAddon not loaded");
      return false;
    }
    addon = new Canvas();
    term.loadAddon(addon);
    rememberTerminalRenderer(addon, "canvas");
    console.log("[Terminal] renderer: canvas");
    return true;
  } catch (e) {
    try { addon?.dispose?.(); } catch {}
    console.log("[Terminal] canvas renderer failed:", (e && e.message) || e);
    return false;
  }
}

function attachTerminalRenderer(term) {
  // Attempt the WebGL addon directly rather than pre-probing a detached canvas
  // with getContext('webgl2') — several mobile browsers false-negative that
  // probe for unattached canvases, which was silently dropping us to the slow
  // DOM renderer even though WebGL2 works for the real, attached terminal.
  let addon = null;
  try {
    const Webgl = resolveTerminalAddon("WebglAddon");
    if (Webgl) {
      addon = new Webgl();
      if (typeof addon.onContextLoss === "function") {
        addon.onContextLoss(() => {
          if (terminalRendererAddon !== addon) return;
          disposeTerminalRendererAddon();
          if (!attachTerminalCanvasRenderer(term)) rememberTerminalRenderer(null, "dom");
          term.refresh(0, Math.max(0, term.rows - 1));
        });
      }
      term.loadAddon(addon);
      rememberTerminalRenderer(addon, "webgl");
      console.log("[Terminal] renderer: webgl");
      return;
    }
    console.log("[Terminal] WebglAddon not loaded");
  } catch (e) {
    try { addon?.dispose?.(); } catch {}
    console.log("[Terminal] webgl renderer failed:", (e && e.message) || e);
  }
  if (!attachTerminalCanvasRenderer(term)) {
    rememberTerminalRenderer(null, "dom");
    console.log("[Terminal] renderer: dom (fallback)");
  }
}

function resetTerminalRendererForDpr(term, reason) {
  const previousKind = terminalRendererKind;
  const previousDpr = terminalRendererDpr;
  disposeTerminalRendererAddon();
  attachTerminalRenderer(term);
  console.log(
    `[Terminal] renderer scale reset (${reason}): ${previousKind} ${previousDpr} -> ${terminalRendererKind} ${terminalRendererDpr}`,
  );
}

function syncTerminalRendererDpr(term) {
  const dpr = currentTerminalDevicePixelRatio();
  if (!terminalRendererDpr) {
    terminalRendererDpr = dpr;
    return;
  }
  if (Math.abs(dpr - terminalRendererDpr) < 0.01) return;
  if (!terminalPageActive) return;
  terminalRendererVerifiedDpr = 0;
  resetTerminalRendererForDpr(term, "devicePixelRatio changed");
}

function terminalRendererCanvasScaleMismatch() {
  if (!terminalRendererAddon || !terminalXtermEl) return false;
  const dpr = currentTerminalDevicePixelRatio();
  const canvases = terminalXtermEl.querySelectorAll(".xterm-screen canvas");
  for (const canvas of canvases) {
    const cssWidth = Number.parseFloat(canvas.style.width) || canvas.getBoundingClientRect().width;
    if (cssWidth < 1 || canvas.width < 1) continue;
    const backingStoreRatio = canvas.width / cssWidth;
    if (Math.abs(backingStoreRatio - dpr) > 0.15) return true;
  }
  return false;
}

function scheduleTerminalRendererScaleCheck() {
  if (Math.abs(currentTerminalDevicePixelRatio() - terminalRendererVerifiedDpr) < 0.01) return;
  if (terminalRendererScaleCheckTimer) clearTimeout(terminalRendererScaleCheckTimer);
  terminalRendererScaleCheckTimer = window.setTimeout(() => {
    terminalRendererScaleCheckTimer = 0;
    if (!terminalPageActive || !terminalXtermActive || !terminalXterm || !terminalRendererAddon) return;
    const dpr = currentTerminalDevicePixelRatio();
    if (Math.abs(dpr - terminalRendererVerifiedDpr) < 0.01) return;
    terminalRendererVerifiedDpr = dpr;
    if (!terminalRendererCanvasScaleMismatch()) return;
    resetTerminalRendererForDpr(terminalXterm, "canvas backing-store mismatch");
    scheduleTerminalFit();
  }, 120);
}

function bindTerminalDprObserver(onChange) {
  if (typeof window.matchMedia !== "function") return;
  if (terminalDprMediaQuery) {
    if (typeof terminalDprMediaQuery.media.removeEventListener === "function") {
      terminalDprMediaQuery.media.removeEventListener("change", terminalDprMediaQuery.listener);
    } else {
      terminalDprMediaQuery.media.removeListener?.(terminalDprMediaQuery.listener);
    }
  }
  const media = window.matchMedia(`(resolution: ${currentTerminalDevicePixelRatio()}dppx)`);
  const listener = () => {
    bindTerminalDprObserver(onChange);
    onChange();
  };
  if (typeof media.addEventListener === "function") media.addEventListener("change", listener);
  else media.addListener?.(listener);
  terminalDprMediaQuery = { media, listener };
}

function ensureTerminalXterm() {
  if (terminalXterm) return terminalXterm;
  if (!terminalXtermSupported()) return null;
  const term = new window.Terminal({
    fontFamily: readCssVar("--font-mono", "ui-monospace, \"Roboto Mono\", Menlo, Consolas, monospace"),
    fontSize: terminalFontSize(),
    lineHeight: 1.25,
    cursorBlink: true,
    scrollback: 5000,
    convertEol: false,
    allowProposedApi: true,
    allowTransparency: false,
    theme: {
      background: terminalOpaqueBackground(),
      foreground: readCssVar("--md-on-surface", "#e6e9ef"),
      cursor: readCssVar("--md-primary", "#7ee0a0"),
      cursorAccent: readCssVar("--md-surface", "#0b0f14"),
      selectionBackground: "rgba(120,160,255,0.35)",
    },
  });
  term.open(terminalXtermEl);
  attachTerminalRenderer(term);
  term.element?.addEventListener("touchstart", (event) => {
    terminalXtermTouchStartY = event.touches?.[0]?.clientY ?? null;
  }, { capture: true, passive: true });
  term.element?.addEventListener("touchmove", trackTerminalXtermTouch, { capture: true, passive: true });
  term.element?.addEventListener("touchend", () => { terminalXtermTouchStartY = null; }, { capture: true, passive: true });
  term.element?.addEventListener("touchcancel", () => { terminalXtermTouchStartY = null; }, { capture: true, passive: true });
  // Keystrokes typed directly into the grid drive interactive programs.
  term.onData((data) => {
    if (terminalSuppressDataDepth > 0) return;
    terminalFollowOutput = true;
    if (sendTerminalPacket({ type: "raw", data: applyTerminalCtrl(data) }, { quiet: true })) {
      window.CarrotSupportTerminal?.reportHostRawTyping?.(data);
    }
  });
  term.onScroll((viewportY) => {
    if (Date.now() < terminalSuppressScrollUntil) return;
    const buffer = term.buffer?.active;
    terminalFollowOutput = !buffer || viewportY >= (buffer.baseY | 0);
  });
  term.onResize(({ cols, rows }) => {
    terminalLastSizeKey = `${cols | 0}x${rows | 0}`;
    scheduleTerminalOuterVerticalScrollLock();
  });
  term.textarea?.addEventListener("focus", scheduleTerminalOuterVerticalScrollLock, { passive: true });
  terminalXterm = term;
  // Re-fit whenever the host actually gets/changes size (page show, orientation,
  // keyboard). Columns stay locked at 100; rows track the container height.
  if (typeof ResizeObserver === "function" && !terminalXtermResizeObserver) {
    terminalXtermResizeObserver = new ResizeObserver(() => {
      if (terminalPageActive) refreshTerminalLayout();
      else scheduleTerminalFit();
    });
    terminalXtermResizeObserver.observe(terminalXtermEl);
  }
  return term;
}

function writeTerminalXterm(data, options = {}) {
  if (!terminalXtermActive || !terminalXterm) return;
  const { suppressData = false } = options;
  if (!suppressData) {
    terminalXterm.write(data);
    return;
  }

  terminalSuppressDataDepth += 1;
  let done = false;
  const release = () => {
    if (done) return;
    done = true;
    terminalSuppressDataDepth = Math.max(0, terminalSuppressDataDepth - 1);
  };
  try {
    terminalXterm.write(data, () => {
      release();
    });
    window.setTimeout(release, 500);
  } catch (e) {
    release();
    throw e;
  }
}

function scheduleTerminalFit() {
  if (terminalXtermFitRaf) cancelAnimationFrame(terminalXtermFitRaf);
  terminalXtermFitRaf = requestAnimationFrame(() => {
    terminalXtermFitRaf = 0;
    fitTerminalXterm();
  });
}

// The keyboard opening/closing resizes the container in steps (and Samsung
// reports the geometry late); re-fit a few times as it settles so the grid ends
// up matching the final height instead of a transient one (empty band bug).
function scheduleKeyboardSettleFit() {
  if (!terminalXtermActive) return;
  terminalKeyboardFitTimers.forEach((t) => clearTimeout(t));
  terminalKeyboardFitTimers = [0, 130, 320, 600].map((ms) => setTimeout(() => fitTerminalXterm(), ms));
}

function activateTerminalXterm() {
  if (!terminalXtermSupported()) return false;
  // Reveal the grid host before opening so xterm can measure its cell size
  // (a display:none container yields no dimensions).
  if (terminalScreenEl) terminalScreenEl.hidden = true;
  if (terminalXtermEl) terminalXtermEl.hidden = false;
  const term = ensureTerminalXterm();
  if (!term) {
    if (terminalScreenEl) terminalScreenEl.hidden = false;
    if (terminalXtermEl) terminalXtermEl.hidden = true;
    return false;
  }
  terminalXtermActive = true;
  fitTerminalXterm();
  // Container may still be settling right after the page is shown, and xterm's
  // renderer measures the cell a frame or two after open; re-fit on the next
  // frames and once more slightly later so the grid fills the whole area.
  requestAnimationFrame(() => requestAnimationFrame(() => fitTerminalXterm()));
  setTimeout(() => fitTerminalXterm(), 180);
  return true;
}

// Rows follow the container height so the grid fills the screen (no empty band
// below it) while columns stay locked at 100. The cell height is measured from
// the actually-rendered .xterm-screen (its height / current rows) in CSS pixels
// — this is DPR-safe. Reading xterm's internal renderService cell instead could
// report DEVICE pixels and undercount rows by the DPR, which is what left the
// big empty band under the terminal when the keyboard opened on high-DPR phones.
function terminalGridRows() {
  try {
    const host = terminalXtermEl;
    const outerHeight = host ? host.getBoundingClientRect().height : 0;
    // The grid must only consume the content box.  Measuring the outer flex
    // item includes its terminal padding, which lets the last row paint into
    // the intended breathing room above the touch-key bar.
    const hostStyle = host ? getComputedStyle(host) : null;
    const verticalPadding = hostStyle
      ? (parseFloat(hostStyle.paddingTop) || 0) + (parseFloat(hostStyle.paddingBottom) || 0)
      : 0;
    const h = Math.max(0, outerHeight - verticalPadding);
    if (h > 0) {
      let cellH = 0;
      const screen = host.querySelector(".xterm-screen");
      const curRows = terminalXterm && terminalXterm.rows;
      if (screen && curRows > 0) {
        const sh = screen.getBoundingClientRect().height;
        if (sh > 0) cellH = sh / curRows;
      }
      if (!(cellH > 0)) {
        const fs = (terminalXterm && terminalXterm.options && terminalXterm.options.fontSize) || 13;
        cellH = fs * 1.25;
      }
      return Math.max(6, Math.floor(h / cellH));
    }
  } catch (e) {
    /* not laid out yet */
  }
  return TERMINAL_GRID_ROWS;
}

function fitTerminalXterm() {
  if (!terminalXtermActive || !terminalXterm) return;
  try {
    syncTerminalRendererDpr(terminalXterm);
    const fs = terminalFontSize();
    if (terminalXterm && terminalXterm.options && terminalXterm.options.fontSize !== fs) {
      terminalXterm.options.fontSize = fs;
    }
    const rows = terminalGridRows();
    if (terminalXterm.cols !== TERMINAL_GRID_COLS || terminalXterm.rows !== rows) {
      terminalXterm.resize(TERMINAL_GRID_COLS, rows);
      // Tell the shared PTY the new row count (columns stay locked at 100) so
      // full-screen apps (btop/vim) draw to the full height too.
      sendTerminalPacket({ type: "resize", cols: TERMINAL_GRID_COLS, rows }, { quiet: true });
      requestAnimationFrame(() => {
        pinTerminalCursorToBottom();
        scheduleTerminalOuterVerticalScrollLock();
      });
    } else {
      terminalXterm.refresh(0, rows - 1);
      pinTerminalCursorToBottom();
      scheduleTerminalOuterVerticalScrollLock();
    }
    scheduleTerminalRendererScaleCheck();
  } catch (e) {
    /* container not laid out yet */
  }
}

function setTerminalCtrlSticky(on) {
  terminalCtrlSticky = !!on;
  const btn = document.querySelector('.terminal-key[data-key="ctrl"]');
  if (btn) btn.classList.toggle("is-active", terminalCtrlSticky);
}

// When the sticky Ctrl key is armed, fold the next single character into its
// control code (Ctrl-C, Ctrl-D, Ctrl-[, ...), then disarm.
function applyTerminalCtrl(data) {
  if (!terminalCtrlSticky || String(data).length !== 1) return data;
  const code = String(data).toUpperCase().charCodeAt(0);
  setTerminalCtrlSticky(false);
  if (code >= 64 && code <= 95) return String.fromCharCode(code - 64);
  return data;
}

function sendTerminalKey(key) {
  if (key === "ctrl") {
    setTerminalCtrlSticky(!terminalCtrlSticky);
    if (terminalXtermActive && terminalXterm) terminalXterm.focus();
    return;
  }
  const seq = TERMINAL_KEY_SEQ[key];
  if (seq == null) return;
  terminalFollowOutput = true;
  const out = applyTerminalCtrl(seq);
  if (sendTerminalPacket({ type: "raw", data: out }, { quiet: true })) {
    if (terminalXtermActive && terminalXterm) terminalXterm.focus();
  }
}

function currentTerminalSize() {
  const rows = (terminalXtermActive && terminalXterm && terminalXterm.rows) || terminalGridRows();
  return { cols: TERMINAL_GRID_COLS, rows };
}

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

// The web terminal runs `::` meta commands by typing a fixed CLI bridge into
// tmux, so tmux echoes the raw `python3 -m ...cli --line <cmd>` invocation.
// Replace that echo with our own friendly "running command" line.
const TERMINAL_META_ECHO_RE = /(?:env\s+\S*PYTHONPATH=\S+\s+)?python3 -m selfdrive\.carrot\.server\.terminal_commands\.cli --line (.*)$/gm;

function rewriteTerminalMetaEcho(text) {
  return String(text || "").replace(TERMINAL_META_ECHO_RE, (match, raw) => {
    let arg = String(raw || "").trim();
    if (arg.length >= 2 &&
        ((arg[0] === "'" && arg[arg.length - 1] === "'") ||
         (arg[0] === '"' && arg[arg.length - 1] === '"'))) {
      arg = arg.slice(1, -1);
    }
    const label = getUIText("terminal_meta_running", "Carrot command");
    return `> ${label}: ::${arg}`;
  });
}

function sanitizeTerminalScreen(text) {
  let nextText = rewriteTerminalMetaEcho(String(text || " "));
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

function stripTerminalAnsi(text) {
  return String(text || "")
    .replace(/\x1b\][^\x07]*(?:\x07|\x1b\\)/g, "")
    .replace(/\x1b\[[0-?]*[ -/]*[@-~]/g, "")
    .replace(/\x1b[()][A-Za-z0-9]/g, "")
    // Drop leftover C0 control chars (bell, stray ESC, etc.) so they don't
    // render as boxes. Keep \b (0x08), \t (0x09), \n (0x0a), \r (0x0d).
    .replace(/[\x00-\x07\x0b\x0c\x0e-\x1f]/g, "");
}

function appendTerminalPtyOutput(chunk) {
  const text = stripTerminalAnsi(chunk).replace(/\r\n/g, "\n");
  if (!text) return;
  const out = terminalPtyBuffer || "";
  // Split the committed lines from the line currently under the cursor so that
  // carriage-return / backspace overwrite *within* the line the way a real
  // terminal does (e.g. git/pip progress bars), instead of dropping text.
  const lastNl = out.lastIndexOf("\n");
  let head = lastNl >= 0 ? out.slice(0, lastNl + 1) : "";
  let line = lastNl >= 0 ? out.slice(lastNl + 1) : out;
  let col = line.length;
  for (const ch of Array.from(text)) {
    if (ch === "\n") {
      head += line + "\n";
      line = "";
      col = 0;
    } else if (ch === "\r") {
      col = 0;
    } else if (ch === "\b") {
      if (col > 0) col -= 1;
    } else {
      if (col < line.length) {
        line = line.slice(0, col) + ch + line.slice(col + 1);
      } else {
        if (col > line.length) line = line.padEnd(col, " ");
        line += ch;
      }
      col += 1;
    }
  }
  const merged = head + line;
  const lines = merged.split("\n");
  terminalPtyBuffer = lines.length > 600 ? lines.slice(-600).join("\n") : merged;
  setTerminalScreen(terminalPtyBuffer || " ", false);
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
  terminalPtyBuffer = "";
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

function clearTerminalReconnectTimer() {
  if (terminalReconnectTimer) {
    clearTimeout(terminalReconnectTimer);
    terminalReconnectTimer = null;
  }
}

function updateTerminalToastAnchor() {
  const anchorEl = terminalKeysEl;
  if (!anchorEl || document.body?.dataset?.page !== "terminal") {
    document.documentElement.style.removeProperty("--terminal-toast-bottom");
    document.documentElement.style.removeProperty("--terminal-toast-left");
    document.documentElement.style.removeProperty("--terminal-toast-width");
    return;
  }

  const rect = anchorEl.getBoundingClientRect();
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
}

function bindTerminalLayoutObservers() {
  if (terminalLayoutBound) return;
  terminalLayoutBound = true;

  const handleLayout = (options = {}) => {
    const { resizeTerminal = true } = options;
    if (terminalLayoutRaf) cancelAnimationFrame(terminalLayoutRaf);
    terminalLayoutRaf = requestAnimationFrame(() => {
      terminalLayoutRaf = 0;
      updateTerminalViewportMetrics();
      updateTerminalToastAnchor();
      updateTerminalOverflowState();
      if (resizeTerminal) sendTerminalResize();
      if (!terminalXtermActive && terminalFollowOutput) pinTerminalToBottom();
    });
  };
  const handleResizeLayout = () => {
    handleLayout({ resizeTerminal: true });
  };
  const handleViewportScroll = () => {
    if (document.body?.dataset?.page === "terminal" && document.documentElement.dataset.kbOpen === "1") {
      updateTerminalToastAnchor();
      return;
    }
    handleLayout({ resizeTerminal: false });
  };

  handleResizeLayout();
  bindTerminalDprObserver(handleResizeLayout);
  window.addEventListener("resize", handleResizeLayout, { passive: true });
  window.addEventListener("orientationchange", handleResizeLayout, { passive: true });
  window.addEventListener("pageshow", handleResizeLayout, { passive: true });
  // Keyboard-driven layout changes also get extra settle-time re-fits.
  const handleKeyboardLayout = () => {
    handleResizeLayout();
    scheduleKeyboardSettleFit();
  };
  if (window.visualViewport) {
    window.visualViewport.addEventListener("resize", handleKeyboardLayout, { passive: true });
    window.visualViewport.addEventListener("scroll", handleViewportScroll, { passive: true });
  }
  // VK API mode: the keyboard show/hide fires geometrychange, not a
  // visualViewport resize (the visual viewport no longer moves).
  if (navigator.virtualKeyboard) {
    navigator.virtualKeyboard.addEventListener("geometrychange", handleKeyboardLayout, { passive: true });
  }
  document.addEventListener("visibilitychange", () => {
    if (!document.hidden && document.body?.dataset?.page === "terminal") handleResizeLayout();
  }, { passive: true });
  document.addEventListener("focusin", () => {
    if (document.body?.dataset?.page === "terminal") handleResizeLayout();
  }, { passive: true });
  document.addEventListener("focusout", () => {
    if (document.body?.dataset?.page === "terminal") window.setTimeout(handleResizeLayout, 80);
  }, { passive: true });
}

function scheduleTerminalSettledLayouts() {
  window.setTimeout(refreshTerminalLayout, 80);
  window.setTimeout(refreshTerminalLayout, 260);
  window.setTimeout(refreshTerminalLayout, 700);
}

function refreshTerminalLayout() {
  requestAnimationFrame(() => {
    updateTerminalViewportMetrics();
    updateTerminalToastAnchor();
    updateTerminalOverflowState();
    sendTerminalResize();
    if (!terminalXtermActive && terminalFollowOutput) pinTerminalToBottom();
  });
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
  const size = currentTerminalSize();
  const path = terminalUsePty ? "/ws/terminal_pty" : "/ws/terminal";
  const params = new URLSearchParams({
    cols: String(size.cols),
    rows: String(size.rows),
  });
  if (terminalSessionName) params.set("session", terminalSessionName);
  if (terminalResetPending) {
    // One-shot: the Reconnect button ends the current session and starts fresh.
    params.set("reset", "1");
    terminalResetPending = false;
  }
  return `${proto}://${location.host}${path}?${params.toString()}`;
}

function sendTerminalResize() {
  if (terminalXtermActive) fitTerminalXterm();
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
  if (terminalXtermActive && terminalXterm) terminalXterm.reset();
  else if (terminalUsePty) clearTerminalViewport();

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
      setTerminalMeta(data.mode === "pty"
        ? getUIText("connected", "connected")
        : (data.created ? getUIText("terminal_ready", "tmux ready") : getUIText("connected", "connected")));
      if (terminalXtermActive && terminalXterm) {
        fitTerminalXterm();
        terminalXterm.focus();
      }
      return;
    }

    if (data.type === "pty_output") {
      const bytes = data.b64 != null ? base64ToBytes(data.b64) : null;
      const decoded = bytes
        ? (terminalTextDecoder ? terminalTextDecoder.decode(bytes, { stream: true }) : data.text || "")
        : (data.text || "");
      const output = consumeTerminalWebActions(decoded, { dispatch: data.replay !== true });
      if (data.replay === true) terminalWebActionCarry = "";
      if (terminalXtermActive && terminalXterm) {
        if (output) writeTerminalXterm(output, { suppressData: data.replay === true });
      } else {
        appendTerminalPtyOutput(output);
      }
      if (terminalMetaEl && terminalMetaEl.textContent === getUIText("connecting", "connecting...")) {
        setTerminalMeta(getUIText("connected", "connected"));
      }
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

  bindNodeOnce(terminalScreenEl, "scrollBound", () => {
    terminalFollowOutput = isTerminalPinnedToBottom();
    updateTerminalOverflowState();
  }, "scroll");

  bindNodeOnce(terminalXtermEl, "outerScrollBound", () => {
    if (terminalXtermEl?.scrollTop) lockTerminalOuterVerticalScroll();
  }, "scroll");

  bindNodeOnce(btnTerminalCtrlCEl, "clickBound", () => {
    terminalFollowOutput = isTerminalPinnedToBottom();
    sendTerminalControl("ctrl_c");
  });

  bindNodeOnce(btnTerminalClearEl, "clickBound", () => {
    terminalFollowOutput = true;
    if (terminalXtermActive && terminalXterm) terminalXterm.clear();
    else clearTerminalViewport();
    sendTerminalControl("clear");
  });

  bindNodeOnce(btnTerminalReconnectEl, "clickBound", () => {
    terminalFollowOutput = true;
    // Reconnect = end the session and start a fresh one (not just re-attach).
    terminalResetPending = true;
    connectTerminal(true);
  });

  // On-screen key bar (Esc/Ctrl/Tab/arrows) for touch devices. mousedown
  // preventDefault keeps focus on the grid so physical/virtual typing that
  // follows still lands in the terminal.
  if (terminalKeysEl && terminalKeysEl.dataset.keysBound !== "1") {
    terminalKeysEl.dataset.keysBound = "1";
    terminalKeysEl.addEventListener("touchstart", (ev) => {
      const touch = ev.touches && ev.touches[0];
      terminalKeysTouchStart = touch ? { x: touch.clientX, y: touch.clientY } : null;
    }, { passive: true });
    terminalKeysEl.addEventListener("touchmove", (ev) => {
      if (!terminalKeysTouchStart) return;
      const touch = ev.touches && ev.touches[0];
      if (!touch) return;
      const dx = Math.abs(touch.clientX - terminalKeysTouchStart.x);
      const dy = Math.abs(touch.clientY - terminalKeysTouchStart.y);
      if (dy > dx + 4) ev.preventDefault();
    }, { passive: false });
    terminalKeysEl.addEventListener("touchend", () => {
      terminalKeysTouchStart = null;
    }, { passive: true });
    terminalKeysEl.addEventListener("touchcancel", () => {
      terminalKeysTouchStart = null;
    }, { passive: true });
    terminalKeysEl.querySelectorAll(".terminal-key").forEach((btn) => {
      btn.addEventListener("mousedown", (ev) => ev.preventDefault());
      btn.addEventListener("click", () => sendTerminalKey(btn.dataset.key));
    });
  }

  // Click anywhere on the grid host focuses the terminal so a physical (PC)
  // keyboard drives it directly. Esc/Ctrl/arrows are handled natively by xterm.
  if (terminalXtermEl && terminalXtermEl.dataset.focusBound !== "1") {
    terminalXtermEl.dataset.focusBound = "1";
    terminalXtermEl.addEventListener("mousedown", () => {
      if (terminalXtermActive && terminalXterm) {
        requestAnimationFrame(() => terminalXterm.focus());
      }
    });
  }
}

function initTerminalPage() {
  enterTerminalKeyboardMode();
  terminalPageActive = true;
  terminalFollowOutput = true;
  terminalCurrentCwd = "/data/openpilot";
  initTerminalBindings();
  activateTerminalXterm();
  setTerminalSessionMeta();
  if (!terminalXtermActive && !terminalLastScreen) setTerminalScreen(" ", true);
  refreshTerminalLayout();
  scheduleTerminalSettledLayouts();
  connectTerminal(false);
  window.CarrotSupportTerminal?.init?.();
}

function teardownTerminalPage() {
  terminalPageActive = false;
  leaveTerminalKeyboardMode();
  window.CarrotSupportTerminal?.teardown?.();
  clearTerminalReconnectTimer();
  closeTerminalSocket();
  if (terminalLayoutRaf) {
    cancelAnimationFrame(terminalLayoutRaf);
    terminalLayoutRaf = 0;
  }
  terminalKeyboardFitTimers.forEach((t) => clearTimeout(t));
  terminalKeyboardFitTimers = [];
  document.documentElement.style.removeProperty("--terminal-toast-bottom");
  document.documentElement.style.removeProperty("--terminal-toast-left");
  document.documentElement.style.removeProperty("--terminal-toast-width");
  terminalKeysTouchStart = null;
}

const CarrotTerminalRuntime = Object.freeze({
  init: initTerminalPage,
  teardown: teardownTerminalPage,
});

Object.assign(globalThis, {
  CarrotTerminalRuntime,
  initTerminalPage,
  teardownTerminalPage,
});

export { CarrotTerminalRuntime, initTerminalPage, teardownTerminalPage };
