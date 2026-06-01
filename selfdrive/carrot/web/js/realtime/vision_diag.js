(function () {
  "use strict";

  // Carrot Vision diagnostic recorder.
  //
  // Silently records RTC phase transitions + periodic WebRTC stats into a ring
  // buffer (persisted to localStorage so it survives reloads / reconnects), and
  // exposes a phone-friendly export (download .txt + copy to clipboard) via a
  // small "LOG" button on the drive page. Built so a tester can capture
  // "why did it drop" data during a once-a-day drive WITHOUT DevTools.
  //
  // Fully self-contained: only reads window.CarrotRtcPerf / CarrotVisionState
  // and listens to carrot:visionstatechange — no edits to the RTC logic.

  const STORAGE_KEY = "carrot_vision_diag_log_v1";
  const MAX_ENTRIES = 1500;          // ~50 min at the 2s stats cadence
  const STATS_INTERVAL_MS = 2000;
  const PERSIST_THROTTLE_MS = 4000;

  let entries = [];
  let lastPersist = 0;
  let lastPhaseSig = "";

  // Restore a prior buffer (survives the reconnect/reload that often follows a drop).
  try {
    const raw = window.localStorage && window.localStorage.getItem(STORAGE_KEY);
    if (raw) {
      const parsed = JSON.parse(raw);
      if (Array.isArray(parsed)) entries = parsed.slice(-MAX_ENTRIES);
    }
  } catch (_) {}

  function persist(force) {
    const t = Date.now();
    if (!force && t - lastPersist < PERSIST_THROTTLE_MS) return;
    lastPersist = t;
    try {
      window.localStorage && window.localStorage.setItem(STORAGE_KEY, JSON.stringify(entries.slice(-MAX_ENTRIES)));
    } catch (_) {}
  }

  function push(entry) {
    entry.t = Date.now();
    entries.push(entry);
    if (entries.length > MAX_ENTRIES) entries.splice(0, entries.length - MAX_ENTRIES);
    persist(false);
  }

  function record(type, data) {
    push(Object.assign({ type }, data || {}));
  }

  function num(v, d) {
    return (typeof v === "number" && isFinite(v)) ? Number(v.toFixed(d == null ? 1 : d)) : null;
  }

  function snapshotStats() {
    if (!document.body || document.body.dataset.page !== "carrot") return;
    const st = window.CarrotVisionState || {};
    if (!st.active) return;
    const p = window.CarrotRtcPerf || {};
    const inb = p.inbound || {};
    const net = p.network || {};
    const vid = p.video || {};
    push({
      type: "stat",
      ph: st.phase,
      cs: st.controlState,
      conn: p.connectionState,
      ice: p.iceConnectionState,
      loss: num(net.lossPct, 1),
      jit: num(net.jitterMs, 0),
      br: num(net.bitrateMbps, 2),
      frm: inb.framesDecoded != null ? inb.framesDecoded : null,
      key: inb.keyFramesDecoded != null ? inb.keyFramesDecoded : null,
      lost: inb.packetsLost != null ? inb.packetsLost : null,
      ct: num(vid.currentTime, 1),
      rs: vid.readyState != null ? vid.readyState : null,
      err: p.error || "",
    });
  }

  function fmtTime(ms) {
    const base = entries.length ? entries[0].t : ms;
    const s = Math.max(0, (ms - base) / 1000);
    const mm = Math.floor(s / 60);
    const ss = (s % 60).toFixed(1);
    return String(mm).padStart(2, "0") + ":" + (ss.length < 4 ? "0" + ss : ss);
  }

  function dump() {
    const conn = navigator.connection || {};
    const lines = [
      "# Carrot Vision diagnostic log",
      "# exported: " + new Date().toISOString(),
      "# ua: " + navigator.userAgent,
      "# viewport: " + window.innerWidth + "x" + window.innerHeight + " dpr=" + window.devicePixelRatio,
      "# net: type=" + (conn.effectiveType || "?") + " downlink=" + (conn.downlink != null ? conn.downlink : "?") + "Mbps rtt=" + (conn.rtt != null ? conn.rtt : "?") + "ms",
      "# entries: " + entries.length,
      "",
    ];
    for (const e of entries) {
      const ts = fmtTime(e.t);
      if (e.type === "phase") {
        lines.push("[" + ts + "] PHASE " + e.ph + " (" + e.cs + ") reason=" + (e.reason || ""));
      } else if (e.type === "stat") {
        lines.push("[" + ts + "] STAT  ph=" + e.ph + " conn=" + e.conn + " ice=" + e.ice +
          " loss=" + e.loss + "% jit=" + e.jit + "ms br=" + e.br + "M" +
          " frm=" + e.frm + " key=" + e.key + " lost=" + e.lost + " ct=" + e.ct + " rs=" + e.rs +
          (e.err ? " err=" + e.err : ""));
      } else {
        lines.push("[" + ts + "] " + e.type + " " + JSON.stringify(e));
      }
    }
    return lines.join("\n");
  }

  function copy(text) {
    const t = typeof text === "string" ? text : dump();
    try {
      if (navigator.clipboard && navigator.clipboard.writeText) {
        navigator.clipboard.writeText(t).catch(function () {});
        return;
      }
    } catch (_) {}
    try {
      const ta = document.createElement("textarea");
      ta.value = t;
      ta.style.position = "fixed";
      ta.style.opacity = "0";
      document.body.appendChild(ta);
      ta.select();
      document.execCommand("copy");
      ta.remove();
    } catch (_) {}
  }

  function download() {
    persist(true);
    const text = dump();
    try {
      const blob = new Blob([text], { type: "text/plain" });
      const url = URL.createObjectURL(blob);
      const a = document.createElement("a");
      a.href = url;
      a.download = "carrot_vision_diag_" + new Date().toISOString().replace(/[:.]/g, "-") + ".txt";
      document.body.appendChild(a);
      a.click();
      setTimeout(function () { try { URL.revokeObjectURL(url); a.remove(); } catch (_) {} }, 1000);
    } catch (_) {}
    copy(text);  // also stash on the clipboard so they can paste directly
  }

  function clear() {
    entries = [];
    lastPhaseSig = "";
    try { window.localStorage && window.localStorage.removeItem(STORAGE_KEY); } catch (_) {}
  }

  // --- capture ---
  // Phase / recovery transitions (recovery sets phase=recovering with a reason).
  // De-dupe so the many same-phase state updates don't spam the log.
  window.addEventListener("carrot:visionstatechange", function (ev) {
    const st = (ev && ev.detail && ev.detail.state) || window.CarrotVisionState;
    if (!st) return;
    const sig = (st.phase || "") + "|" + (st.reason || "");
    if (sig === lastPhaseSig) return;
    lastPhaseSig = sig;
    record("phase", { ph: st.phase, cs: st.controlState, reason: st.reason || "" });
  });

  setInterval(snapshotStats, STATS_INTERVAL_MS);
  window.addEventListener("beforeunload", function () { persist(true); });

  // --- phone-friendly export button on the drive page ---
  function ensureButton() {
    if (!document.body || document.getElementById("carrotDiagBtn")) return;
    const btn = document.createElement("button");
    btn.id = "carrotDiagBtn";
    btn.type = "button";
    btn.textContent = "LOG⤓";
    btn.setAttribute("aria-label", "Download Carrot Vision diagnostic log");
    const s = btn.style;
    s.position = "fixed";
    s.left = "8px";
    s.top = "8px";          // top-left — clear of the HUD card (bottom-left) and controls (bottom-right)
    s.zIndex = "99999";
    s.padding = "5px 9px";
    s.fontSize = "11px";
    s.fontWeight = "700";
    s.fontFamily = "ui-monospace, monospace";
    s.lineHeight = "1";
    s.color = "rgba(255,255,255,0.9)";
    s.background = "rgba(0,0,0,0.45)";
    s.border = "1px solid rgba(255,255,255,0.25)";
    s.borderRadius = "8px";
    s.backdropFilter = "blur(6px)";
    s.webkitBackdropFilter = "blur(6px)";
    s.opacity = "0.5";
    s.cursor = "pointer";
    btn.addEventListener("click", function (e) {
      e.stopPropagation();
      download();
      btn.textContent = "SAVED";
      setTimeout(function () { btn.textContent = "LOG⤓"; }, 1200);
    });
    document.body.appendChild(btn);
    syncButton();
  }

  function syncButton() {
    const btn = document.getElementById("carrotDiagBtn");
    if (!btn) return;
    btn.style.display = (document.body && document.body.dataset.page === "carrot") ? "block" : "none";
  }

  window.addEventListener("carrot:pagechange", syncButton);
  if (document.readyState === "loading") {
    window.addEventListener("DOMContentLoaded", ensureButton);
  } else {
    ensureButton();
  }

  window.CarrotVisionDiag = {
    record: record,
    dump: dump,
    download: download,
    copy: copy,
    clear: clear,
    get entries() { return entries; },
  };
})();
