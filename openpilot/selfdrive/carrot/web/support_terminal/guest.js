"use strict";

(function () {
  const sessionId = document.documentElement.dataset.sessionId || location.pathname.split("/").pop() || "";
  const els = {
    meta: document.getElementById("terminalMeta"),
    session: document.getElementById("terminalSessionMeta"),
    permission: document.getElementById("permissionChip"),
    auth: document.getElementById("authForm"),
    authStatus: document.getElementById("authStatus"),
    pin: document.getElementById("pinInput"),
    connect: document.getElementById("connectBtn"),
    xterm: document.getElementById("terminalXterm"),
    typingHost: document.getElementById("hostTypingHost"),
    notice: document.getElementById("commandNotice"),
    keys: document.getElementById("terminalKeys"),
    ctrlC: document.getElementById("btnCtrlC"),
    clear: document.getElementById("btnClear"),
    disconnect: document.getElementById("btnDisconnect"),
  };

  const strings = {
    en: {
      title: "Carrot Remote Terminal", pin: "PIN required", connect: "Connect", connecting: "Connecting...",
      connected: "connected", disconnected: "Disconnected", unavailable: "Session unavailable",
      approveEach: "Approve each · {seconds}s", allowAll: "Full control", viewOnly: "View only", hostAway: "Host is not viewing terminal",
      waiting: "Waiting for owner approval", approved: "Approved · sending to terminal", running: "Sent to terminal",
      rejected: "Command rejected", expired: "Command approval expired", failed: "Failed to send command",
      disconnect: "Disconnect", closed: "Remote connection closed", inputDenied: "Owner approval is required",
      expires: "Connected · {time}", unlimited: "unlimited",
      consoleBlocked: "Developer tools detected - enter PIN again",
      consoleWarningTitle: "Warning", consoleWarningBody: "Console use is prohibited on this page.",
      hostActor: "Host", typing: "Typing",
    },
    ko: {
      title: "Carrot 원격 터미널", pin: "PIN 필요", connect: "연결", connecting: "연결 중...",
      connected: "연결됨", disconnected: "연결 끊김", unavailable: "사용할 수 없는 세션입니다",
      approveEach: "명령별 승인 · {seconds}초", allowAll: "전체 제어", viewOnly: "보기 전용", hostAway: "호스트가 터미널을 보고 있지 않습니다",
      waiting: "소유자 승인 대기 중", approved: "승인됨 · 터미널로 전송 중", running: "터미널로 전송됨",
      rejected: "명령이 거절되었습니다", expired: "명령 승인이 만료되었습니다", failed: "명령 전송 실패",
      disconnect: "연결 종료", closed: "원격 연결이 종료되었습니다", inputDenied: "소유자 승인이 필요합니다",
      expires: "연결됨 · {time}", unlimited: "무제한",
      consoleBlocked: "개발자 도구 감지됨 - PIN을 다시 입력하세요",
      consoleWarningTitle: "경고", consoleWarningBody: "콘솔 사용이 금지되어 있습니다.",
      hostActor: "호스트", typing: "입력 중",
    },
    zh: {
      title: "Carrot 远程终端", pin: "需要 PIN", connect: "连接", connecting: "正在连接...",
      connected: "已连接", disconnected: "已断开", unavailable: "会话不可用",
      approveEach: "逐条批准 · {seconds}秒", allowAll: "完全控制", viewOnly: "仅查看", hostAway: "车主未查看终端",
      waiting: "等待车主批准", approved: "已批准 · 正在发送", running: "已发送到终端",
      rejected: "命令已拒绝", expired: "命令批准已过期", failed: "命令发送失败",
      disconnect: "断开连接", closed: "远程连接已关闭", inputDenied: "需要车主批准",
      expires: "已连接 · {time}", unlimited: "无限",
      consoleBlocked: "检测到开发者工具 - 请重新输入 PIN",
      consoleWarningTitle: "警告", consoleWarningBody: "此页面禁止使用控制台。",
      hostActor: "主机", typing: "正在输入",
    },
  };

  // The remote support guest page is only ever used by the (Korean) admin, so
  // force Korean here. The multi-locale strings map is kept intact for reuse.
  const language = "ko";
  const t = (key, vars = null) => {
    let value = strings[language][key] || strings.en[key] || key;
    if (vars) Object.entries(vars).forEach(([name, replacement]) => { value = value.replaceAll(`{${name}}`, String(replacement)); });
    return value;
  };

  const state = {
    ws: null,
    authed: false,
    allowAll: false,
    controlGranted: false,
    commandTimeoutSeconds: 30,
    followOutput: true,
    ownerPresent: false,
    sessionClosed: false,
    terminal: null,
    ctrlSticky: false,
    rows: 30,
    cols: 100,
    remainingDeadlineMs: 0,
    remainingTimer: 0,
    noticeTimer: 0,
    pinRaf: 0,
    viewerFitRaf: 0,
    outerScrollRaf: 0,
    outerScrollTimer: 0,
    outerScrollLockUntil: 0,
    viewerResizeObserver: null,
    suppressScrollUntil: 0,
    touchStartY: null,
    consoleBlocked: false,
    rawTypingBuffer: "",
    rawTypingTimer: 0,
    approvalInput: "",
    approvalCursor: 0,
    approvalRenderedInput: "",
    approvalRenderedCursor: 0,
    hostTypingActive: false,
    hostTypingText: "",
    hostTypingTimer: 0,
    hostTypingRaf: 0,
    typingIndicator: null,
  };

  const KEY_SEQ = {
    esc: "\x1b", tab: "\t", ctrl_c: "\x03", ctrl_d: "\x04",
    // AGNOS configures tmux's prefix as backtick, then `d detaches it.
    detach: "\x60d",
    home: "\x1b[H", end: "\x1b[F", page_up: "\x1b[5~", page_down: "\x1b[6~",
    up: "\x1b[A", down: "\x1b[B", right: "\x1b[C", left: "\x1b[D",
  };

  // xterm generates these replies itself when an application asks about the
  // terminal type or its colours. The host is the canonical terminal for the
  // shared PTY, so a second reply from a guest can arrive after tmux detached
  // and become literal shell input (for example: `rgb:ffff/ffff/ffff`).
  const TERMINAL_QUERY_RESPONSE_RE = /^(?:\x1b\[(?:\?|>)[0-9;]*c|\x1b\[\d+;\d+R|\x1b\](?:4;\d+|10|11|12);rgb:[0-9a-f/]+(?:\x07|\x1b\\))+$/i;

  function socketReady() {
    return Boolean(state.ws && state.ws.readyState === WebSocket.OPEN);
  }

  function isTerminalQueryResponse(data) {
    return TERMINAL_QUERY_RESPONSE_RE.test(String(data || ""));
  }

  function applyPrintableTypingInput(buffer, data) {
    let next = String(buffer || "");
    let changed = false;
    let submitted = false;
    const input = String(data || "");

    for (let index = 0; index < input.length; index += 1) {
      const char = input[index];
      if (char === "\x1b") {
        const kind = input[index + 1];
        if (kind === "[") {
          index += 2;
          while (index < input.length) {
            const code = input.charCodeAt(index);
            if (code >= 0x40 && code <= 0x7e) break;
            index += 1;
          }
        } else if (kind === "]") {
          index += 2;
          while (index < input.length) {
            if (input[index] === "\x07") break;
            if (input[index] === "\x1b" && input[index + 1] === "\\") {
              index += 1;
              break;
            }
            index += 1;
          }
        } else {
          index += 1;
        }
        continue;
      }
      if (char === "\r" || char === "\n") {
        if (next) changed = true;
        next = "";
        submitted = true;
        continue;
      }
      if (char === "\x7f" || char === "\b") {
        const trimmed = next.slice(0, -1);
        changed ||= trimmed !== next;
        next = trimmed;
        continue;
      }
      if (char >= " " && char !== "\x7f") {
        next = (next + char).slice(-160);
        changed = true;
      }
    }
    return { text: next, changed, submitted };
  }

  function base64ToBytes(value) {
    const binary = atob(String(value || ""));
    const bytes = new Uint8Array(binary.length);
    for (let i = 0; i < binary.length; i += 1) bytes[i] = binary.charCodeAt(i);
    return bytes;
  }

  function terminalFontSize() {
    const width = window.innerWidth || 800;
    if (width <= 380) return 11;
    if (width <= 640) return 12;
    return 13;
  }

  function readCssVar(name, fallback) {
    try {
      return getComputedStyle(document.documentElement).getPropertyValue(name).trim() || fallback;
    } catch (_) {
      return fallback;
    }
  }

  function viewerRowCount() {
    const hostHeight = els.xterm?.getBoundingClientRect().height || 0;
    if (!(hostHeight > 0)) return state.rows;
    // terminal-xterm has shared host padding.  Keep it outside xterm's grid
    // so the final terminal row cannot consume the lower breathing room.
    const hostStyle = getComputedStyle(els.xterm);
    const verticalPadding = (parseFloat(hostStyle.paddingTop) || 0)
      + (parseFloat(hostStyle.paddingBottom) || 0);
    const contentHeight = Math.max(0, hostHeight - verticalPadding);
    const screen = els.xterm?.querySelector(".xterm-screen");
    const renderedRows = state.terminal?.rows || 0;
    const renderedHeight = screen?.getBoundingClientRect().height || 0;
    const cellHeight = renderedRows > 0 && renderedHeight > 0
      ? renderedHeight / renderedRows
      : terminalFontSize() * 1.25;
    return Math.max(6, Math.floor(contentHeight / Math.max(cellHeight, 1)));
  }

  function fitViewerTerminal() {
    const terminal = state.terminal;
    if (!terminal) return;
    const rows = viewerRowCount();
    if (terminal.cols !== state.cols || terminal.rows !== rows) {
      terminal.resize(state.cols, rows);
    }
    if (state.followOutput) scheduleTerminalPin();
    scheduleViewerOuterVerticalScrollLock();
  }

  function scheduleViewerFit() {
    if (state.viewerFitRaf) cancelAnimationFrame(state.viewerFitRaf);
    state.viewerFitRaf = requestAnimationFrame(() => {
      state.viewerFitRaf = 0;
      fitViewerTerminal();
    });
  }

  function lockViewerOuterVerticalScroll() {
    if (Date.now() > state.outerScrollLockUntil) return;
    if (els.xterm && els.xterm.scrollTop !== 0) els.xterm.scrollTop = 0;
  }

  function scheduleViewerOuterVerticalScrollLock() {
    state.outerScrollLockUntil = Date.now() + 800;
    lockViewerOuterVerticalScroll();
    if (state.outerScrollRaf) cancelAnimationFrame(state.outerScrollRaf);
    if (state.outerScrollTimer) clearTimeout(state.outerScrollTimer);
    state.outerScrollRaf = requestAnimationFrame(() => {
      state.outerScrollRaf = 0;
      lockViewerOuterVerticalScroll();
      requestAnimationFrame(lockViewerOuterVerticalScroll);
    });
    state.outerScrollTimer = window.setTimeout(() => {
      state.outerScrollTimer = 0;
      lockViewerOuterVerticalScroll();
    }, 620);
  }

  // A real vertical drag takes precedence over pending output/layout pins.
  // This lets a guest inspect terminal history even while new output arrives.
  function trackViewerTerminalTouch(event) {
    const touch = event.touches?.[0];
    if (!touch || state.touchStartY == null) return;
    if (Math.abs(touch.clientY - state.touchStartY) < 6) return;
    state.followOutput = false;
    state.suppressScrollUntil = 0;
  }

  function pinTerminalToBottom() {
    const terminal = state.terminal;
    if (!terminal || !state.followOutput) return;
    state.suppressScrollUntil = Date.now() + 350;
    terminal.scrollToBottom();
  }

  function scheduleTerminalPin() {
    if (!state.followOutput || !state.terminal) return;
    if (state.pinRaf) return;
    state.pinRaf = requestAnimationFrame(() => {
      state.pinRaf = 0;
      pinTerminalToBottom();
    });
  }

  // The host paints an opaque dark surface, so xterm runs opaque here. WebGL
  // thins glyphs badly with allowTransparency on (xtermjs #4212), so we read the
  // resolved host color and keep transparency off.
  function opaqueBackground() {
    try {
      const bg = els.xterm ? getComputedStyle(els.xterm).backgroundColor : "";
      if (bg && bg !== "transparent" && !/rgba?\([^)]*,\s*0\s*\)\s*$/i.test(bg)) return bg;
    } catch (_) {}
    return readCssVar("--md-surface", "#0b0f14");
  }

  // Prefer a GPU renderer so scrollback is smooth: the default DOM renderer
  // re-renders row <div>s on every scroll frame. WebGL first, Canvas fallback,
  // then the built-in DOM renderer. Logs the outcome so a fallback is visible.
  // The UMD builds expose the addon global as a namespace object
  // (window.WebglAddon === { WebglAddon: class }), not the class, so resolve the
  // constructor from either shape.
  function resolveAddon(name) {
    const g = window[name];
    if (typeof g === "function") return g;
    if (g && typeof g[name] === "function") return g[name];
    return null;
  }

  function attachCanvasRenderer(terminal) {
    try {
      const Canvas = resolveAddon("CanvasAddon");
      if (!Canvas) {
        console.log("[Guest] CanvasAddon not loaded");
        return false;
      }
      terminal.loadAddon(new Canvas());
      console.log("[Guest] renderer: canvas");
      return true;
    } catch (e) {
      console.log("[Guest] canvas renderer failed:", (e && e.message) || e);
      return false;
    }
  }

  // Try the WebGL addon directly; pre-probing getContext('webgl2') on a detached
  // canvas false-negatives on some mobile browsers and was silently forcing DOM.
  function attachRenderer(terminal) {
    try {
      const Webgl = resolveAddon("WebglAddon");
      if (Webgl) {
        const addon = new Webgl();
        if (typeof addon.onContextLoss === "function") {
          addon.onContextLoss(() => {
            try { addon.dispose(); } catch (_) {}
            attachCanvasRenderer(terminal);
          });
        }
        terminal.loadAddon(addon);
        console.log("[Guest] renderer: webgl");
        return;
      }
      console.log("[Guest] WebglAddon not loaded");
    } catch (e) {
      console.log("[Guest] webgl renderer failed:", (e && e.message) || e);
    }
    if (!attachCanvasRenderer(terminal)) console.log("[Guest] renderer: dom (fallback)");
  }

  function ensureTerminal() {
    if (state.terminal) return state.terminal;
    if (typeof window.Terminal !== "function") throw new Error("xterm unavailable");
    const terminal = new window.Terminal({
      fontFamily: readCssVar("--font-mono", 'ui-monospace, "Roboto Mono", Menlo, Consolas, monospace'),
      fontSize: terminalFontSize(),
      lineHeight: 1.25,
      cursorBlink: true,
      scrollback: 5000,
      convertEol: false,
      allowProposedApi: true,
      allowTransparency: false,
      disableStdin: true,
      theme: {
        background: opaqueBackground(),
        foreground: readCssVar("--md-on-surface", "#e6e9ef"),
        cursor: readCssVar("--md-primary", "#7ee0a0"),
        cursorAccent: readCssVar("--md-surface", "#0b0f14"),
        selectionBackground: "rgba(120,160,255,0.35)",
      },
    });
    terminal.open(els.xterm);
    attachRenderer(terminal);
    terminal.resize(state.cols, state.rows);
    terminal.textarea?.addEventListener("focus", scheduleViewerOuterVerticalScrollLock, { passive: true });
    terminal.element?.addEventListener("touchstart", (event) => {
      state.touchStartY = event.touches?.[0]?.clientY ?? null;
    }, { capture: true, passive: true });
    terminal.element?.addEventListener("touchmove", trackViewerTerminalTouch, { capture: true, passive: true });
    terminal.element?.addEventListener("touchend", () => { state.touchStartY = null; }, { capture: true, passive: true });
    terminal.element?.addEventListener("touchcancel", () => { state.touchStartY = null; }, { capture: true, passive: true });
    terminal.onResize(scheduleViewerOuterVerticalScrollLock);
    terminal.onData((data) => {
      if (isTerminalQueryResponse(data)) return;
      if (canControlRaw()) {
        send({ type: "raw", data: applyStickyCtrl(data) });
        reportRawTyping(data);
      } else if (canComposeApproval()) {
        handleApprovalInput(data);
      }
    });
    terminal.onScroll((viewportY) => {
      if (Date.now() < state.suppressScrollUntil) return;
      const buffer = terminal.buffer?.active;
      state.followOutput = !buffer || viewportY >= (buffer.baseY | 0);
    });
    state.terminal = terminal;
    if (typeof ResizeObserver === "function" && !state.viewerResizeObserver) {
      state.viewerResizeObserver = new ResizeObserver(scheduleViewerFit);
      state.viewerResizeObserver.observe(els.xterm);
    }
    scheduleViewerFit();
    return terminal;
  }

  function setTerminalSize(cols, rows) {
    state.cols = Math.max(20, Number(cols || 100));
    state.rows = Math.max(8, Number(rows || 30));
    const terminal = ensureTerminal();
    if (terminal.cols !== state.cols || terminal.rows !== state.rows) terminal.resize(state.cols, state.rows);
    terminal.options.fontSize = terminalFontSize();
    terminal.refresh(0, terminal.rows - 1);
    scheduleViewerFit();
  }

  function canControlRaw() {
    return state.authed && state.allowAll && state.controlGranted && state.ownerPresent && socketReady();
  }

  function canComposeApproval() {
    return state.authed && !state.allowAll && state.ownerPresent && socketReady();
  }

  function inputChars(value) {
    return Array.from(String(value || ""));
  }

  function cellWidth(value) {
    let width = 0;
    for (const char of inputChars(value)) {
      const code = char.codePointAt(0) || 0;
      if ((code >= 0x300 && code <= 0x36f) || (code >= 0xfe00 && code <= 0xfe0f)) continue;
      width += code >= 0x1100 && (
        code <= 0x115f || code === 0x2329 || code === 0x232a ||
        (code >= 0x2e80 && code <= 0xa4cf) || (code >= 0xac00 && code <= 0xd7a3) ||
        (code >= 0xf900 && code <= 0xfaff) || (code >= 0xfe10 && code <= 0xfe6f) ||
        (code >= 0xff00 && code <= 0xff60) || (code >= 0xffe0 && code <= 0xffe6) ||
        (code >= 0x1f300 && code <= 0x1faff)
      ) ? 2 : 1;
    }
    return width;
  }

  function approvalRenderSequence(clearOnly = false) {
    const rendered = inputChars(state.approvalRenderedInput);
    const renderedCursor = Math.min(state.approvalRenderedCursor, rendered.length);
    const back = cellWidth(rendered.slice(0, renderedCursor).join(""));
    let sequence = `${back ? `\x1b[${back}D` : ""}\x1b[0K`;
    if (!clearOnly) {
      const chars = inputChars(state.approvalInput);
      const cursor = Math.min(state.approvalCursor, chars.length);
      sequence += chars.join("");
      const tail = cellWidth(chars.slice(cursor).join(""));
      if (tail) sequence += `\x1b[${tail}D`;
      state.approvalRenderedInput = state.approvalInput;
      state.approvalRenderedCursor = cursor;
    } else {
      state.approvalRenderedInput = "";
      state.approvalRenderedCursor = 0;
    }
    return sequence;
  }

  function renderApprovalInput(reportTyping = true) {
    state.terminal?.write(approvalRenderSequence());
    if (reportTyping) reportCommandTyping(state.approvalInput);
  }

  function clearApprovalInput(resetDraft = true) {
    if (state.approvalRenderedInput) state.terminal?.write(approvalRenderSequence(true));
    if (resetDraft) {
      state.approvalInput = "";
      state.approvalCursor = 0;
      reportCommandTyping("");
    }
  }

  function handleApprovalInput(data) {
    if (data.length > 1 && !data.startsWith("\x1b") && /[\r\n]/.test(data)) {
      for (const char of inputChars(data)) handleApprovalInput(char);
      return;
    }
    if (data === "\r" || data === "\n") {
      const line = state.approvalInput.trim();
      clearApprovalInput();
      if (line && send({ type: "input", data: line })) setNotice(t("waiting"));
      return;
    }
    if (data === "\x03") {
      clearApprovalInput();
      send({ type: "control", action: "ctrl_c" });
      return;
    }
    const chars = inputChars(state.approvalInput);
    if (data === "\x7f" || data === "\b") {
      if (state.approvalCursor > 0) chars.splice(--state.approvalCursor, 1);
    } else if (data === "\x1b[D") {
      state.approvalCursor = Math.max(0, state.approvalCursor - 1);
    } else if (data === "\x1b[C") {
      state.approvalCursor = Math.min(chars.length, state.approvalCursor + 1);
    } else if (data === "\x1b[H" || data === "\x1b[1~") {
      state.approvalCursor = 0;
    } else if (data === "\x1b[F" || data === "\x1b[4~") {
      state.approvalCursor = chars.length;
    } else if (data === "\x1b[3~") {
      if (state.approvalCursor < chars.length) chars.splice(state.approvalCursor, 1);
    } else if (data === "\t") {
      chars.splice(state.approvalCursor, 0, " ", " ");
      state.approvalCursor += 2;
    } else if (![...data].some((char) => char < " " || char === "\x7f")) {
      const inserted = inputChars(data);
      chars.splice(state.approvalCursor, 0, ...inserted);
      state.approvalCursor += inserted.length;
    } else {
      return;
    }
    state.approvalInput = chars.join("").slice(0, 4000);
    state.approvalCursor = Math.min(state.approvalCursor, inputChars(state.approvalInput).length);
    renderApprovalInput();
  }

  function syncInputMode() {
    const rawEnabled = canControlRaw();
    const approvalEnabled = canComposeApproval();
    const controlsEnabled = state.authed && state.ownerPresent && socketReady()
      && (!state.allowAll || state.controlGranted);
    if (state.terminal) state.terminal.options.disableStdin = !(rawEnabled || approvalEnabled);
    // Ctrl+C and Clear are approval-aware control actions. Keep them in the
    // shared key bar even for approve-each sessions; raw keys only appear when
    // this guest has the same full terminal control as the host.
    els.keys.hidden = !state.authed || !state.ownerPresent || !socketReady();
    els.keys.querySelectorAll("[data-key]").forEach((button) => {
      button.hidden = !rawEnabled;
    });
    els.ctrlC.disabled = !controlsEnabled;
    els.clear.disabled = !controlsEnabled;
    els.disconnect.disabled = !state.authed;
    els.permission.hidden = !state.authed;
    els.permission.textContent = state.allowAll
      ? t(state.controlGranted ? "allowAll" : "viewOnly")
      : t("approveEach", { seconds: state.commandTimeoutSeconds });
    if (!state.ownerPresent && state.authed) setNotice(t("hostAway"));
    else if (els.notice.textContent === t("hostAway")) setNotice("");
    if (rawEnabled || approvalEnabled) state.terminal?.focus();
  }

  function send(payload) {
    if (!socketReady()) return false;
    state.ws.send(JSON.stringify(payload));
    return true;
  }

  // In allow-all mode the guest types straight into the shared PTY (raw), so the
  // host's dialog otherwise shows no "guest is typing" preview. Mirror the raw
  // keystrokes into a small line buffer and forward it as typing events so the
  // owner can see what the controller is entering, matching the old line-input
  // behaviour. Enter clears the buffer; idle stops the indicator.
  function reportRawTyping(data) {
    const result = applyPrintableTypingInput(state.rawTypingBuffer, data);
    state.rawTypingBuffer = result.text;
    if (state.rawTypingTimer) clearTimeout(state.rawTypingTimer);
    if (result.submitted) {
      state.rawTypingTimer = 0;
      send({ type: "typing", active: false, text: "" });
      return;
    }
    if (!result.changed && !state.rawTypingBuffer) return;
    send({ type: "typing", active: true, text: state.rawTypingBuffer });
    state.rawTypingTimer = window.setTimeout(() => {
      state.rawTypingTimer = 0;
      send({ type: "typing", active: false, text: "" });
    }, 1400);
  }

  function ensureHostTypingOverlay() {
    const host = els.typingHost;
    if (!host || !els.xterm) return null;
    if (els.xterm.querySelector(".xterm") && host.parentElement !== els.xterm) els.xterm.append(host);
    return host;
  }

  function scheduleHostTypingRender() {
    if (state.hostTypingRaf) return;
    state.hostTypingRaf = requestAnimationFrame(() => {
      state.hostTypingRaf = 0;
      const host = ensureHostTypingOverlay();
      if (!host) return;
      if (!state.typingIndicator && window.CarrotTerminalTypingIndicator) {
        state.typingIndicator = window.CarrotTerminalTypingIndicator.create(host);
      }
      state.typingIndicator?.update({
        active: state.hostTypingActive,
        actor: t("hostActor"),
        text: state.hostTypingText,
        emptyLabel: t("typing"),
      });
    });
  }

  function showHostTyping(active, text = "") {
    state.hostTypingActive = Boolean(active);
    state.hostTypingText = state.hostTypingActive ? String(text || "").slice(0, 160) : "";
    if (state.hostTypingTimer) clearTimeout(state.hostTypingTimer);
    if (state.hostTypingActive) {
      state.hostTypingTimer = window.setTimeout(() => {
        state.hostTypingTimer = 0;
        state.hostTypingActive = false;
        state.hostTypingText = "";
        scheduleHostTypingRender();
      }, 2200);
    }
    scheduleHostTypingRender();
  }

  function setNotice(message, durationMs = 0) {
    if (state.noticeTimer) clearTimeout(state.noticeTimer);
    state.noticeTimer = 0;
    els.notice.textContent = String(message || "");
    els.notice.hidden = !message;
    if (message && durationMs > 0) {
      state.noticeTimer = window.setTimeout(() => setNotice(""), durationMs);
    }
  }

  // Anchor the PIN overlay to the browser's actual visible rectangle. This
  // avoids device-specific dvh/keyboard resize behaviour and keeps the panel at
  // the exact horizontal and vertical center before and after the IME opens.
  function syncAuthViewport() {
    const vv = window.visualViewport;
    const root = document.documentElement;
    const left = vv ? vv.offsetLeft : 0;
    const top = vv ? vv.offsetTop : 0;
    const width = vv ? vv.width : (window.innerWidth || document.documentElement.clientWidth || 0);
    const height = vv ? vv.height : (window.innerHeight || document.documentElement.clientHeight || 0);
    root.style.setProperty("--auth-vv-left", `${Math.round(left)}px`);
    root.style.setProperty("--auth-vv-top", `${Math.round(top)}px`);
    root.style.setProperty("--auth-vv-width", `${Math.round(width)}px`);
    root.style.setProperty("--auth-vv-height", `${Math.round(height)}px`);
  }

  function reportCommandTyping(text) {
    const value = String(text || "").slice(0, 160);
    if (state.rawTypingTimer) clearTimeout(state.rawTypingTimer);
    state.rawTypingTimer = 0;
    if (!value) {
      send({ type: "typing", active: false, text: "" });
      return;
    }
    send({ type: "typing", active: true, text: value });
    state.rawTypingTimer = window.setTimeout(() => {
      state.rawTypingTimer = 0;
      send({ type: "typing", active: false, text: "" });
    }, 1400);
  }

  function showAuth(message = t("pin")) {
    els.auth.hidden = false;
    els.auth.classList.remove("is-hiding");
    els.authStatus.textContent = message;
    syncAuthViewport();
    requestAnimationFrame(() => els.pin.focus());
  }

  function hideAuth() {
    els.auth.classList.add("is-hiding");
    window.setTimeout(() => { if (els.auth.classList.contains("is-hiding")) els.auth.hidden = true; }, 190);
  }

  function closeSocket() {
    const ws = state.ws;
    state.ws = null;
    if (!ws) return;
    try { if (ws.readyState === WebSocket.OPEN || ws.readyState === WebSocket.CONNECTING) ws.close(); } catch (_) {}
  }

  function resetToPin(message, clearTerminal = false) {
    state.authed = false;
    state.allowAll = false;
    state.controlGranted = false;
    state.ownerPresent = false;
    state.remainingDeadlineMs = 0;
    if (state.remainingTimer) clearInterval(state.remainingTimer);
    state.remainingTimer = 0;
    els.pin.value = "";
    clearApprovalInput();
    if (state.rawTypingTimer) clearTimeout(state.rawTypingTimer);
    state.rawTypingTimer = 0;
    els.session.textContent = message || t("disconnected");
    setNotice("");
    if (clearTerminal) state.terminal?.reset();
    syncInputMode();
    showAuth(message || t("pin"));
  }

  function formatRemaining(seconds) {
    if (seconds == null) return t("unlimited");
    const total = Math.max(0, Math.ceil(seconds));
    const minutes = Math.floor(total / 60);
    const remainder = total % 60;
    return minutes > 0 ? `${minutes}:${String(remainder).padStart(2, "0")}` : `${remainder}s`;
  }

  function updateRemaining() {
    if (!state.authed) return;
    const seconds = state.remainingDeadlineMs ? Math.max(0, Math.ceil((state.remainingDeadlineMs - Date.now()) / 1000)) : null;
    els.meta.textContent = state.ownerPresent ? t("expires", { time: formatRemaining(seconds) }) : t("hostAway");
    if (seconds === 0) {
      state.sessionClosed = true;
      closeSocket();
      resetToPin(t("closed"));
    }
  }

  function startRemaining(expiresIn) {
    if (state.remainingTimer) clearInterval(state.remainingTimer);
    state.remainingDeadlineMs = expiresIn == null ? 0 : Date.now() + Math.max(0, Number(expiresIn || 0)) * 1000;
    updateRemaining();
    state.remainingTimer = window.setInterval(updateRemaining, 1000);
  }

  // Toggle the "console detected" lockout UI: disable the PIN field + connect
  // button so the page cannot even be authenticated, and surface the warning.
  function setConsoleLockUI(locked) {
    if (els.pin) els.pin.disabled = locked;
    if (els.connect) els.connect.disabled = locked;
    els.auth?.classList.toggle("is-console-blocked", locked);
  }

  // Best-effort developer-tools guard for the remote support page. A large gap
  // between the outer and inner window size is the classic signal that a docked
  // devtools panel is open. When detected we tear the session down entirely: any
  // live socket is closed, the terminal is wiped, the PIN screen is shown with a
  // warning, and PIN entry itself is disabled until devtools is closed. It is a
  // deterrent, not a hard security boundary — pairs with the page CSP + PIN gate.
  function blockForConsole() {
    if (state.consoleBlocked) return;
    state.consoleBlocked = true;
    state.sessionClosed = true;
    closeSocket();
    resetToPin(t("consoleBlocked"), true);
    setConsoleLockUI(true);
  }

  function isDevToolsLikelyOpen() {
    const widthGap = window.outerWidth > 0 ? Math.abs(window.outerWidth - window.innerWidth) : 0;
    const heightGap = window.outerHeight > 0 ? Math.abs(window.outerHeight - window.innerHeight) : 0;
    const touchDevice = Number(navigator.maxTouchPoints || 0) > 0;
    // Normal desktop Chrome can reserve close to 200px for its own tab/address
    // bars, while a mobile IME can legitimately consume much more height. A
    // docked devtools pane is materially larger; only use the height signal on
    // non-touch layouts so opening a phone keyboard never locks the PIN/input.
    return widthGap > 260 || (!touchDevice && heightGap > 260);
  }

  function checkDevToolsOpen() {
    if (isDevToolsLikelyOpen()) {
      blockForConsole();
      return true;
    }
    if (state.consoleBlocked) {
      state.consoleBlocked = false;
      setConsoleLockUI(false);
      els.authStatus.textContent = t("pin");
    }
    return false;
  }

  function showConsoleWarning() {
    try {
      console.log(`%c${t("consoleWarningTitle")}`, "color:#ff4d4f;font-size:42px;font-weight:900;");
      console.log(`%c${t("consoleWarningBody")}`, "color:#ffb86b;font-size:16px;font-weight:700;");
    } catch (_) {}
  }

  function connect(pin) {
    if (state.consoleBlocked || checkDevToolsOpen()) return;
    closeSocket();
    state.sessionClosed = false;
    state.terminal?.reset();
    els.authStatus.textContent = t("connecting");
    const proto = location.protocol === "https:" ? "wss" : "ws";
    const ws = new WebSocket(`${proto}://${location.host}/ws/support_terminal/${encodeURIComponent(sessionId)}`);
    state.ws = ws;
    ws.onopen = () => {
      if (state.ws !== ws) return;
      if (state.consoleBlocked || checkDevToolsOpen()) return;
      send({ type: "auth", pin });
    };
    ws.onmessage = (event) => {
      if (state.ws !== ws) return;
      let data;
      try { data = JSON.parse(event.data); } catch (_) { return; }
      handleMessage(data);
    };
    ws.onclose = () => {
      if (state.ws !== ws) return;
      state.ws = null;
      if (!state.sessionClosed) resetToPin(t("disconnected"));
    };
    ws.onerror = () => { if (!state.authed) els.authStatus.textContent = t("unavailable"); };
  }

  function handleMessage(data) {
    if (data.type === "auth_ok") {
      if (state.consoleBlocked || checkDevToolsOpen()) return;
      state.authed = true;
      state.allowAll = data.permission_mode === "allow_all";
      state.controlGranted = Boolean(data.control_granted);
      state.commandTimeoutSeconds = Number(data.command_timeout_seconds || 30);
      state.ownerPresent = Boolean(data.owner_present);
      ensureTerminal();
      hideAuth();
      startRemaining(data.expires_in == null ? null : Number(data.expires_in || 0));
      syncInputMode();
      return;
    }
    if (data.type === "auth_failed") {
      els.authStatus.textContent = `${t("pin")} · ${Number(data.remaining || 0)}`;
      return;
    }
    if (data.type === "meta") {
      els.session.textContent = `${data.user || "comma"}@${data.session || "login-shell"}`;
      setTerminalSize(data.cols, data.rows);
      return;
    }
    if (data.type === "pty_output") {
      const terminal = ensureTerminal();
      const output = data.b64 != null ? base64ToBytes(data.b64) : String(data.text || "");
      const shouldFollow = Boolean(data.replay) || state.followOutput;
      if (data.replay) state.followOutput = true;
      const redrawApproval = Boolean(state.approvalRenderedInput);
      if (redrawApproval) terminal.write(approvalRenderSequence(true));
      terminal.write(output, () => {
        if (redrawApproval && state.approvalInput) renderApprovalInput(false);
        if (shouldFollow && state.followOutput) scheduleTerminalPin();
      });
      return;
    }
    if (data.type === "pty_resize") {
      setTerminalSize(data.cols, data.rows);
      return;
    }
    if (data.type === "host_typing") {
      showHostTyping(Boolean(data.active), String(data.text || ""));
      return;
    }
    if (data.type === "owner_presence") {
      state.ownerPresent = Boolean(data.active);
      updateRemaining();
      syncInputMode();
      return;
    }
    if (data.type === "control_role") {
      state.controlGranted = Boolean(data.granted);
      syncInputMode();
      return;
    }
    if (data.type === "owner_absent") {
      state.ownerPresent = false;
      updateRemaining();
      syncInputMode();
      return;
    }
    const notices = {
      command_waiting_approval: ["waiting", 0], command_approved: ["approved", 1600],
      command_running: ["running", 1800], command_rejected: ["rejected", 2200],
      command_expired: ["expired", 2200], command_failed: ["failed", 2600], input_denied: ["inputDenied", 2200],
    };
    if (notices[data.type]) {
      const [key, duration] = notices[data.type];
      setNotice(t(key), duration);
      return;
    }
    if (data.type === "session_closed" || data.type === "pty_exit") {
      state.sessionClosed = true;
      closeSocket();
      resetToPin(t("closed"));
      return;
    }
    if (data.type === "error") setNotice(data.message || t("unavailable"), 3000);
  }

  function setCtrlSticky(enabled) {
    state.ctrlSticky = Boolean(enabled);
    const button = els.keys.querySelector('[data-key="ctrl"]');
    button?.classList.toggle("is-active", state.ctrlSticky);
  }

  function applyStickyCtrl(data) {
    if (!state.ctrlSticky || String(data).length !== 1) return data;
    const code = String(data).toUpperCase().charCodeAt(0);
    setCtrlSticky(false);
    return code >= 64 && code <= 95 ? String.fromCharCode(code - 64) : data;
  }

  function sendKey(key) {
    if (key === "ctrl") {
      setCtrlSticky(!state.ctrlSticky);
      state.terminal?.focus();
      return;
    }
    const sequence = KEY_SEQ[key];
    if (sequence != null && canControlRaw()) send({ type: "raw", data: applyStickyCtrl(sequence) });
    state.terminal?.focus();
  }

  els.auth.addEventListener("submit", (event) => {
    event.preventDefault();
    if (state.consoleBlocked || checkDevToolsOpen()) return;
    const pin = els.pin.value.trim();
    if (pin) connect(pin);
  });
  els.ctrlC.addEventListener("click", () => send({ type: "control", action: "ctrl_c" }));
  els.clear.addEventListener("click", () => send({ type: "control", action: "clear" }));
  els.disconnect.addEventListener("click", () => {
    if (!send({ type: "close_session" })) return;
    els.disconnect.disabled = true;
    setNotice(t("closed"));
  });
  els.keys.addEventListener("click", (event) => {
    const button = event.target.closest("[data-key]");
    if (button) sendKey(button.dataset.key || "");
  });
  els.xterm.addEventListener("scroll", () => {
    if (els.xterm.scrollTop) lockViewerOuterVerticalScroll();
  }, { passive: true });
  window.addEventListener("resize", () => {
    checkDevToolsOpen();
    syncAuthViewport();
    if (!state.terminal) return;
    state.terminal.options.fontSize = terminalFontSize();
    scheduleViewerFit();
  });
  // Poll as a fallback for devtools opened without a resize (e.g. undocked).
  window.setInterval(checkDevToolsOpen, 1200);
  if (window.visualViewport) {
    window.visualViewport.addEventListener("resize", syncAuthViewport, { passive: true });
    window.visualViewport.addEventListener("scroll", syncAuthViewport, { passive: true });
  }
  els.pin?.addEventListener("focus", syncAuthViewport, { passive: true });
  syncAuthViewport();

  els.meta.textContent = t("title");
  els.authStatus.textContent = t("pin");
  els.connect.textContent = t("connect");
  els.disconnect.textContent = t("disconnect");
  showConsoleWarning();
  ensureTerminal();
  syncInputMode();
  showAuth();
  // Catch devtools that is already open at load, before any PIN can be entered.
  checkDevToolsOpen();
})();
