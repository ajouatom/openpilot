(function () {
  "use strict";

  // Carrot Vision diagnostic recorder (enriched).
  //
  // Silently records, into a ring buffer persisted to localStorage:
  //   PHASE   - RTC phase / recovery transitions (deduped)
  //   STAT    - periodic WebRTC + <video> numbers, with framesReceived/Decoded
  //             deltas so a decode stall ("data in, 0 frames out") is obvious
  //   CODEC   - negotiated codec / H264 profile / decoder implementation (on change)
  //   PATH    - transport protocol + ICE candidate types + negotiated resolution (on change)
  //   VINFO   - <video> readyState / networkState / MediaError (on change)
  //   VEVENT  - real <video> element events (loadedmetadata/playing/waiting/error/...)
  //   VIS     - tab visibility changes (a hidden tab can freeze the decoder)
  //   NETCHG  - navigator.connection changes (effectiveType/downlink/rtt)
  //   PERFERR - getStats() errors
  //   RTCTRACE- WebRTC control-plane trace events emitted by vision_rtc.js
  //   LIFECYCLE - page leave/return and forced reconnect lifecycle events
  //
  // dump() prepends an AUTO SUMMARY (verdict) so a tester can read the conclusion
  // without scrolling 200 lines. Phone-friendly export via a small "LOG" button
  // (download .txt + copy to clipboard) — no DevTools needed.
  //
  // Self-contained: reads window.CarrotRtcPerf / CarrotVisionState, listens to
  // carrot:visionstatechange, and binds to the live <video> element. No edits to
  // the RTC control logic.

  const STORAGE_KEY = "carrot_vision_diag_log_v2";
  const MAX_ENTRIES = 3000;
  const MAX_CONSOLE_ENTRIES = 1200;
  const MAX_CONSOLE_ARG_CHARS = 1600;
  const PERSIST_THROTTLE_MS = 4000;
  const STAT_FAST_MS = 1000;     // while connecting / reconnecting / pre-first-frame
  const STAT_SLOW_MS = 2500;     // once decoding steadily

  let entries = [];
  let lastPersist = 0;
  let lastPhaseSig = "";
  let prevFrm = null;
  let prevRecv = null;
  let prevCt = null;
  let prevDvf = null;
  let everDecoded = false;
  let lastCodecSig = "";
  let lastPathSig = "";
  let lastVinfoSig = "";
  let lastErr = "";
  let statsTimer = null;
  let consoleEntries = [];

  function stringifyConsoleArg(value) {
    try {
      if (value instanceof Error) {
        return value.stack || value.message || String(value);
      }
      if (typeof value === "string") return value;
      if (value == null || typeof value === "number" || typeof value === "boolean") return String(value);
      return JSON.stringify(value);
    } catch (_) {
      try { return String(value); } catch (__) { return "[unprintable]"; }
    }
  }

  function recordConsole(level, args) {
    try {
      const parts = Array.prototype.slice.call(args || []).map(function (arg) {
        const text = stringifyConsoleArg(arg);
        return text.length > MAX_CONSOLE_ARG_CHARS ? text.slice(0, MAX_CONSOLE_ARG_CHARS) + "..." : text;
      });
      consoleEntries.push({
        t: Date.now(),
        level: level,
        message: parts.join(" "),
      });
      if (consoleEntries.length > MAX_CONSOLE_ENTRIES) {
        consoleEntries.splice(0, consoleEntries.length - MAX_CONSOLE_ENTRIES);
      }
    } catch (_) {}
  }

  function hookConsole() {
    if (!window.console || window.console.__carrotVisionDiagHooked) return;
    ["log", "info", "warn", "error", "debug"].forEach(function (level) {
      const original = window.console[level];
      if (typeof original !== "function") return;
      window.console[level] = function () {
        recordConsole(level, arguments);
        return original.apply(this, arguments);
      };
    });
    window.console.__carrotVisionDiagHooked = true;
  }

  hookConsole();

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

  function n(v) { return (typeof v === "number" && isFinite(v)) ? v : null; }

  // 0x42=Baseline 0x4D=Main 0x58=Extended 0x64=High ... + constrained-baseline flag.
  function h264Profile(fmtp) {
    const m = /profile-level-id=([0-9a-fA-F]{6})/.exec(fmtp || "");
    if (!m) return "";
    const pli = m[1].toUpperCase();
    const prof = pli.slice(0, 2);
    const constraints = parseInt(pli.slice(2, 4), 16) || 0;
    const level = (parseInt(pli.slice(4, 6), 16) || 0) / 10;
    const map = { "42": "Baseline", "4D": "Main", "58": "Extended", "64": "High", "6E": "High10", "7A": "High422", "F4": "High444" };
    let name = map[prof] || ("0x" + prof);
    if (prof === "42" && (constraints & 0x40)) name = "ConstrainedBaseline";
    return name + " L" + level.toFixed(1) + " (" + pli + ")";
  }

  // --- video element binding (re-binds when the element is recreated on reconnect) ---
  function getVideo() {
    return document.getElementById("carrotRoadVideo") || document.getElementById("rtcVideo");
  }

  const VEVENTS = [
    "loadedmetadata", "loadeddata", "canplay", "canplaythrough", "play", "playing",
    "pause", "waiting", "stalled", "suspend", "emptied", "ended", "ratechange", "resize", "error",
  ];

  function ensureVideoBound() {
    const v = getVideo();
    if (!v || v.__carrotDiagBound) return;
    v.__carrotDiagBound = true;
    VEVENTS.forEach(function (name) {
      v.addEventListener(name, function () {
        const e = { name: name, w: v.videoWidth || 0, h: v.videoHeight || 0, rs: v.readyState, ct: num(v.currentTime, 1) };
        if (name === "error" && v.error) { e.code = v.error.code; e.msg = String(v.error.message || ""); }
        record("vevent", e);
      }, { passive: true });
    });
  }

  function emitChangeLines(st, p, inb, net, vid) {
    const codecSig = (p.codec || "") + "|" + (inb.decoderImplementation || "") + "|" + (p.codecParams || "");
    if (codecSig !== lastCodecSig && (p.codec || inb.decoderImplementation || p.codecParams)) {
      lastCodecSig = codecSig;
      record("codec", { mime: p.codec || "", decoder: inb.decoderImplementation || "", fmtp: p.codecParams || "", profile: h264Profile(p.codecParams) });
    }
    const pathSig = (net.protocol || "") + "|" + (net.localCandidateType || "") + "|" + (net.remoteCandidateType || "") + "|" + (net.resolutionLabel || "");
    if (pathSig !== lastPathSig && (net.protocol || net.resolutionLabel)) {
      lastPathSig = pathSig;
      record("path", { proto: net.protocol || "", local: net.localCandidateType || "", remote: net.remoteCandidateType || "", res: net.resolutionLabel || "", avail: num(net.availableIncomingMbps, 2) });
    }
    const vinfoSig = vid.readyState + "|" + vid.networkState + "|" + (vid.errorCode || 0) + "|" + vid.paused;
    if (vinfoSig !== lastVinfoSig) {
      lastVinfoSig = vinfoSig;
      record("vinfo", { rs: n(vid.readyState), ns: n(vid.networkState), code: vid.errorCode || 0, msg: vid.errorMessage || "", paused: typeof vid.paused === "boolean" ? vid.paused : null });
    }
    const err = p.error || "";
    if (err !== lastErr) { lastErr = err; if (err) record("perferr", { msg: err }); }
  }

  function snapshotStats() {
    if (!document.body || document.body.dataset.page !== "carrot") return;
    ensureVideoBound();
    const st = window.CarrotVisionState || {};
    if (!st.active) return;
    const p = window.CarrotRtcPerf || {};
    const inb = p.inbound || {};
    const net = p.network || {};
    const vid = p.video || {};

    emitChangeLines(st, p, inb, net, vid);

    const frm = n(inb.framesDecoded);
    const recv = n(inb.framesReceived);
    const frmD = (frm != null && prevFrm != null) ? Math.max(0, frm - prevFrm) : null;
    const recvD = (recv != null && prevRecv != null) ? Math.max(0, recv - prevRecv) : null;
    const ct = num(vid.currentTime, 1);
    const dvf = n(vid.droppedVideoFrames);
    const ctD = (ct != null && prevCt != null) ? num(Math.max(0, ct - prevCt), 1) : null;
    const dvfD = (dvf != null && prevDvf != null) ? Math.max(0, dvf - prevDvf) : null;
    prevFrm = frm; prevRecv = recv;
    prevCt = ct; prevDvf = dvf;
    if (frm != null && frm > 0) everDecoded = true;

    push({
      type: "stat",
      ph: st.phase, cs: st.controlState, conn: p.connectionState, ice: p.iceConnectionState,
      recv: recv, recvD: recvD, frm: frm, frmD: frmD, key: n(inb.keyFramesDecoded),
      drop: n(inb.framesDropped), fps: num(inb.framesPerSecond, 1),
      dw: n(inb.frameWidth), dh: n(inb.frameHeight),
      loss: num(net.lossPct, 1), lost: n(inb.packetsLost),
      jit: num(net.jitterMs, 0), rtt: num(net.rttMs, 0),
      br: num(net.bitrateMbps, 2), avail: num(net.availableIncomingMbps, 2),
      nack: n(inb.nackCount), pli: n(inb.pliCount), fir: n(inb.firCount),
      frz: n(inb.freezeCount), frzMs: num(inb.totalFreezesDuration, 1),
      ct: ct, ctD: ctD, rs: n(vid.readyState), ns: n(vid.networkState),
      paused: typeof vid.paused === "boolean" ? vid.paused : null,
      dvf: dvf, dvfD: dvfD, cvf: n(vid.corruptedVideoFrames),
    });
  }

  function statInterval() {
    const st = window.CarrotVisionState || {};
    const cs = st.controlState;
    if (cs === "connecting" || cs === "reconnecting" || !everDecoded) return STAT_FAST_MS;
    return STAT_SLOW_MS;
  }

  function statsTick() {
    try { snapshotStats(); } catch (_) {}
    statsTimer = setTimeout(statsTick, statInterval());
  }

  function fmtTime(ms) {
    const base = entries.length ? entries[0].t : ms;
    const s = Math.max(0, (ms - base) / 1000);
    const mm = Math.floor(s / 60);
    const ss = (s % 60).toFixed(1);
    return String(mm).padStart(2, "0") + ":" + (ss.length < 4 ? "0" + ss : ss);
  }

  function summary() {
    const stats = entries.filter((e) => e.type === "stat");
    const codecs = entries.filter((e) => e.type === "codec");
    let maxFrm = 0, maxRecv = 0, maxKey = 0, peakLoss = 0, maxFrz = 0, brSum = 0, brN = 0;
    let reachedReady = false, everPlayed = false, decRes = "";
    for (const s of stats) {
      if (typeof s.frm === "number") maxFrm = Math.max(maxFrm, s.frm);
      if (typeof s.recv === "number") maxRecv = Math.max(maxRecv, s.recv);
      if (typeof s.key === "number") maxKey = Math.max(maxKey, s.key);
      if (typeof s.rs === "number" && s.rs >= 2) reachedReady = true;
      if (typeof s.ct === "number" && s.ct > 0) everPlayed = true;
      if (typeof s.loss === "number") peakLoss = Math.max(peakLoss, s.loss);
      if (typeof s.frz === "number") maxFrz = Math.max(maxFrz, s.frz);
      if (typeof s.br === "number" && s.br > 0) { brSum += s.br; brN++; }
      if (typeof s.dw === "number" && s.dw > 0 && typeof s.dh === "number" && s.dh > 0) decRes = s.dw + "x" + s.dh;
    }
    let videoError = false;
    for (const e of entries) {
      if (e.type === "vevent" && e.name === "error") videoError = true;
      if (e.type === "vinfo" && e.code) videoError = true;
    }
    const reconnects = entries.filter((e) => e.type === "phase" && e.cs === "reconnecting").length;
    const starts = entries.filter((e) => e.type === "phase" && /user start/.test(e.reason || "")).length;
    const decoders = [...new Set(codecs.map((c) => c.decoder).filter(Boolean))];
    const profiles = [...new Set(codecs.map((c) => c.profile).filter(Boolean))];
    const mimes = [...new Set(codecs.map((c) => c.mime).filter(Boolean))];
    const decodedEver = maxFrm > 0, receivedEver = maxRecv > 0;

    let verdict;
    if (receivedEver && !decodedEver)
      verdict = "DECODE STALL — frames RECEIVED (" + maxRecv + ") but framesDecoded stayed 0. Viewer never decoded a renderable frame -> likely codec/profile/hardware-decoder incompatibility.";
    else if (decodedEver && !reachedReady)
      verdict = "DECODED frames but <video> never reached readyState>=2 -> render/attach problem, not decode.";
    else if (peakLoss >= 5)
      verdict = "PACKET LOSS elevated (peak " + peakLoss.toFixed(1) + "%) -> network path is the prime suspect.";
    else if (decodedEver && reachedReady)
      verdict = "Video DID play (frames decoded, readyState>=2). For any mid-stream drops, inspect freeze/loss spikes below.";
    else
      verdict = "Inconclusive — a media flow likely never established (no frames received).";

    return [
      "# ===== AUTO SUMMARY =====",
      "# verdict: " + verdict,
      "# framesReceived(max)=" + maxRecv + "  framesDecoded(max)=" + maxFrm + "  keyFrames(max)=" + maxKey,
      "# reachedReadyState>=2=" + reachedReady + "  everPlayed(ct>0)=" + everPlayed + "  decodedResolution=" + (decRes || "never"),
      "# peakLoss=" + peakLoss.toFixed(1) + "%  avgBitrate=" + (brN ? (brSum / brN).toFixed(2) : "?") + "Mbps  maxFreezeCount=" + maxFrz,
      "# starts=" + starts + "  reconnects=" + reconnects + "  videoElementError=" + videoError,
      "# codec=" + (mimes.join(",") || "?") + "  profile=" + (profiles.join(",") || "?") + "  decoder=" + (decoders.join(",") || "?"),
      "# ========================",
      "",
    ];
  }

  function koreanHealthSummary() {
    const stats = entries.filter((e) => e.type === "stat");
    const latest = stats.length ? stats[stats.length - 1] : null;
    const codec = entries.filter((e) => e.type === "codec").slice(-1)[0] || {};
    const path = entries.filter((e) => e.type === "path").slice(-1)[0] || {};
    let verdict = "아직 판단할 통계가 충분하지 않습니다.";
    if (latest) {
      const connected = /connected|completed/i.test(String(latest.conn || "")) || /connected|completed/i.test(String(latest.ice || ""));
      const recv = Number(latest.recv || 0);
      const frm = Number(latest.frm || 0);
      const loss = Number(latest.loss || 0);
      if (recv > 0 && frm > 0) {
        verdict = "영상 수신과 디코드가 정상 진행 중입니다.";
      } else if (recv > 0 && frm <= 0) {
        verdict = "영상 RTP는 수신되지만 브라우저 디코드/렌더가 진행되지 않습니다.";
      } else if (connected && recv <= 0) {
        verdict = "WebRTC 연결은 되었지만 영상 RTP/첫 프레임이 아직 들어오지 않았습니다.";
      } else if (loss >= 5) {
        verdict = "패킷 손실이 높아 네트워크 경로 문제가 의심됩니다.";
      }
    }
    return [
      "# ===== 네트워크 건강 요약 =====",
      "# 판정: " + verdict,
      "# 상태: " + (latest ? `${latest.ph}/${latest.cs}` : "?"),
      "# ICE/연결: " + (latest ? `${latest.conn}/${latest.ice}` : "?"),
      "# 경로: " + ((path.proto || "?") + " " + (path.local || "?") + " -> " + (path.remote || "?")),
      "# 해상도/FPS: " + (latest ? `${latest.dw || "?"}x${latest.dh || "?"} / ${latest.fps || "?"}fps` : "?"),
      "# 프레임: 수신 " + (latest?.recv ?? "?") + " / 디코드 " + (latest?.frm ?? "?") + " / 키프레임 " + (latest?.key ?? "?"),
      "# 네트워크: RTT " + (latest?.rtt ?? "?") + "ms / 지터 " + (latest?.jit ?? "?") + "ms / 손실 " + (latest?.loss ?? "?") + "% / 비트레이트 " + (latest?.br ?? "?") + "Mbps",
      "# 코덱: " + (codec.mime || "?") + " / " + (codec.profile || "?"),
      "# 프리즈: " + (latest?.frz ?? "?") + "회 / " + (latest?.frzMs ?? "?") + "ms",
      "# ============================",
      "",
    ];
  }

  function fmtLine(e) {
    const ts = fmtTime(e.t);
    switch (e.type) {
      case "phase":
        return "[" + ts + "] PHASE " + e.ph + " (" + e.cs + ") reason=" + (e.reason || "");
      case "stat":
        return "[" + ts + "] STAT " + e.ph + "/" + e.cs + " " + e.conn + "/" + e.ice +
          " | recv=" + e.recv + "Δ" + e.recvD + " frm=" + e.frm + "Δ" + e.frmD + " key=" + e.key +
          " drop=" + e.drop + " fps=" + e.fps + " dec=" + e.dw + "x" + e.dh +
          " | loss=" + e.loss + "% lost=" + e.lost + " jit=" + e.jit + "ms rtt=" + e.rtt + "ms br=" + e.br + "M avail=" + e.avail + "M" +
          " nack=" + e.nack + " pli=" + e.pli + " fir=" + e.fir +
          " | ct=" + e.ct + "?" + e.ctD + " rs=" + e.rs + " ns=" + e.ns + " paused=" + e.paused + " dvf=" + e.dvf + "?" + e.dvfD + " cvf=" + e.cvf +
          " frz=" + e.frz + "/" + e.frzMs + "ms";
      case "codec":
        return "[" + ts + "] CODEC mime=" + e.mime + " profile=" + (e.profile || "?") + " decoder=" + (e.decoder || "?") + " fmtp=\"" + (e.fmtp || "") + "\"";
      case "path":
        return "[" + ts + "] PATH proto=" + e.proto + " " + e.local + "->" + e.remote + " res=" + (e.res || "?") + " avail=" + e.avail + "M";
      case "vinfo":
        return "[" + ts + "] VINFO rs=" + e.rs + " ns=" + e.ns + " paused=" + e.paused + (e.code ? " ERROR " + e.code + ":" + (e.msg || "") : "");
      case "vevent":
        return "[" + ts + "] VEVENT " + e.name + " " + e.w + "x" + e.h + " rs=" + e.rs + " ct=" + e.ct + (e.code ? " ERROR " + e.code + ":" + (e.msg || "") : "");
      case "vis":
        return "[" + ts + "] VIS " + e.state;
      case "netchg":
        return "[" + ts + "] NETCHG type=" + e.netType + " downlink=" + e.downlink + "Mbps rtt=" + e.rtt + "ms";
      case "perferr":
        return "[" + ts + "] PERFERR " + e.msg;
      case "rtctrace":
        return "[" + ts + "] RTCTRACE " + e.event + " pc=" + (e.pc || "?") +
          " conn=" + (e.conn || "?") + "/" + (e.ice || "?") +
          " frm=" + e.framesDecoded + " key=" + e.keyFramesDecoded +
          " br=" + e.bitrateMbps + "M rtt=" + e.rttMs + "ms" +
          " rs=" + e.readyState + " track=" + (e.trackState || "?") +
          (e.stallSamples != null ? " stallSamples=" + e.stallSamples : "") +
          (e.receivedProgress != null ? " rtpProgress=" + e.receivedProgress : "") +
          (e.reason ? " reason=" + e.reason : "") +
          (e.timeoutMs ? " timeoutMs=" + e.timeoutMs : "");
      case "lifecycle":
        return "[" + ts + "] LIFECYCLE " + e.event +
          (e.page ? " page=" + e.page : "") +
          (e.reason ? " reason=" + e.reason : "") +
          (e.visionActive != null ? " visionActive=" + e.visionActive : "") +
          (e.forceFetch != null ? " forceFetch=" + e.forceFetch : "");
      default:
        return "[" + ts + "] " + e.type + " " + JSON.stringify(e);
    }
  }

  function safeJson(value) {
    try {
      return JSON.stringify(value, null, 2);
    } catch (error) {
      return JSON.stringify({ error: error?.message || String(error), fallback: String(value) }, null, 2);
    }
  }

  function jsonSection(title, value) {
    return [
      "",
      "# ===== " + title + " =====",
      safeJson(value),
      "# ===== END " + title + " =====",
    ].join("\n");
  }

  function dump() {
    const conn = navigator.connection || {};
    const head = [
      "# Carrot Vision diagnostic log",
      "# exported: " + new Date().toISOString(),
      "# ua: " + navigator.userAgent,
      "# viewport: " + window.innerWidth + "x" + window.innerHeight + " dpr=" + window.devicePixelRatio,
      "# net: type=" + (conn.effectiveType || "?") + " downlink=" + (conn.downlink != null ? conn.downlink : "?") + "Mbps rtt=" + (conn.rtt != null ? conn.rtt : "?") + "ms",
      "# entries: " + entries.length,
      "",
    ];
    const body = entries.map(fmtLine);
    return head.concat(summary(), koreanHealthSummary(), body).join("\n");
  }

  async function collectBrowserRawSnapshot() {
    const snapshot = {
      capturedAtMs: Date.now(),
      capturedAtIso: new Date().toISOString(),
      document: {
        visibilityState: document.visibilityState,
        hidden: document.hidden,
        url: window.location.href,
        referrer: document.referrer || "",
        fullscreenElement: Boolean(document.fullscreenElement),
        pictureInPictureElement: Boolean(document.pictureInPictureElement),
      },
      navigator: {
        userAgent: navigator.userAgent,
        platform: navigator.platform || "",
        language: navigator.language || "",
        languages: navigator.languages || [],
        hardwareConcurrency: navigator.hardwareConcurrency ?? null,
        deviceMemory: navigator.deviceMemory ?? null,
        maxTouchPoints: navigator.maxTouchPoints ?? null,
        cookieEnabled: navigator.cookieEnabled ?? null,
        onLine: navigator.onLine ?? null,
        connection: navigator.connection ? {
          effectiveType: navigator.connection.effectiveType || "",
          downlink: navigator.connection.downlink ?? null,
          rtt: navigator.connection.rtt ?? null,
          saveData: navigator.connection.saveData ?? null,
        } : null,
      },
      screen: {
        width: window.screen?.width ?? null,
        height: window.screen?.height ?? null,
        availWidth: window.screen?.availWidth ?? null,
        availHeight: window.screen?.availHeight ?? null,
        orientation: window.screen?.orientation ? {
          type: window.screen.orientation.type || "",
          angle: window.screen.orientation.angle ?? null,
        } : null,
        innerWidth: window.innerWidth,
        innerHeight: window.innerHeight,
        outerWidth: window.outerWidth,
        outerHeight: window.outerHeight,
        devicePixelRatio: window.devicePixelRatio,
      },
      performance: {
        timeOrigin: performance.timeOrigin ?? null,
        now: performance.now ? performance.now() : null,
        memory: performance.memory ? {
          jsHeapSizeLimit: performance.memory.jsHeapSizeLimit ?? null,
          totalJSHeapSize: performance.memory.totalJSHeapSize ?? null,
          usedJSHeapSize: performance.memory.usedJSHeapSize ?? null,
        } : null,
        navigation: performance.getEntriesByType ? performance.getEntriesByType("navigation").map(function (entry) {
          return {
            type: entry.type || "",
            startTime: entry.startTime,
            domContentLoadedEventEnd: entry.domContentLoadedEventEnd,
            loadEventEnd: entry.loadEventEnd,
            transferSize: entry.transferSize,
            encodedBodySize: entry.encodedBodySize,
            decodedBodySize: entry.decodedBodySize,
          };
        }) : [],
      },
      state: {
        carrotVision: window.CarrotVisionState || null,
        rtcPerf: window.CarrotRtcPerf || null,
        visionTest: window.CarrotVisionTestState || null,
      },
      rtc: null,
      rawStatsHistory: null,
      errors: [],
    };
    try {
      if (typeof window.rtcDiagnosticSnapshot === "function") {
        snapshot.rtc = await window.rtcDiagnosticSnapshot();
      } else if (window.CarrotVisionRtc && typeof window.CarrotVisionRtc.diagnosticSnapshot === "function") {
        snapshot.rtc = await window.CarrotVisionRtc.diagnosticSnapshot();
      }
    } catch (error) {
      snapshot.errors.push({ source: "rtcDiagnosticSnapshot", message: error?.message || String(error) });
    }
    try {
      if (window.CarrotVisionRtc && typeof window.CarrotVisionRtc.rawStatsHistory === "function") {
        snapshot.rawStatsHistory = window.CarrotVisionRtc.rawStatsHistory();
      } else if (typeof window.rtcRawStatsHistory === "function") {
        snapshot.rawStatsHistory = window.rtcRawStatsHistory();
      }
    } catch (error) {
      snapshot.errors.push({ source: "rtcRawStatsHistory", message: error?.message || String(error) });
    }
    return snapshot;
  }

  async function fetchServerSnapshot() {
    const controller = typeof AbortController === "function" ? new AbortController() : null;
    const timer = controller ? setTimeout(function () { controller.abort(); }, 8000) : null;
    try {
      const response = await fetch("/api/vision_diag/server_snapshot", {
        cache: "no-store",
        signal: controller ? controller.signal : undefined,
      });
      const body = await response.json().catch(function () { return null; });
      return {
        ok: response.ok,
        status: response.status,
        body,
      };
    } catch (error) {
      return {
        ok: false,
        error: error?.message || String(error),
      };
    } finally {
      if (timer) clearTimeout(timer);
    }
  }

  async function dumpBundle() {
    persist(true);
    const browserRaw = await collectBrowserRawSnapshot();
    const serverSnapshot = await fetchServerSnapshot();
    return [
      dump(),
      jsonSection("BROWSER RAW SNAPSHOT", browserRaw),
      jsonSection("COMMA SERVER SNAPSHOT", serverSnapshot),
    ].join("\n");
  }

  function dumpConsole() {
    return [
      "# Carrot Vision browser console",
      "# exported: " + new Date().toISOString(),
      "# ua: " + navigator.userAgent,
      "# entries: " + consoleEntries.length,
      jsonSection("BROWSER CONSOLE", consoleEntries.slice(-MAX_CONSOLE_ENTRIES)),
    ].join("\n");
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

  async function download() {
    persist(true);
    const text = await dumpBundle();
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

  async function uploadDiscord() {
    persist(true);
    const text = await dumpBundle();
    const consoleText = dumpConsole();
    const filename = "carrot_vision_diag_" + new Date().toISOString().replace(/[:.]/g, "-") + ".txt";
    const consoleFilename = filename.replace(/\.txt$/, "_console.txt");
    const response = await fetch("/api/vision_diag/upload_discord", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({
        source: "web",
        filename: filename,
        bundle: text,
        consoleFilename: consoleFilename,
        console: consoleText,
      }),
    });
    const body = await response.json().catch(function () { return null; });
    if (!response.ok || !body || !body.ok) {
      const detail = body?.discord?.error || body?.error || response.statusText || "upload failed";
      throw new Error(detail);
    }
    return body;
  }

  function clear() {
    entries = [];
    lastPhaseSig = "";
    prevFrm = null; prevRecv = null; prevCt = null; prevDvf = null; everDecoded = false;
    lastCodecSig = ""; lastPathSig = ""; lastVinfoSig = ""; lastErr = "";
    try { window.localStorage && window.localStorage.removeItem(STORAGE_KEY); } catch (_) {}
  }

  // --- capture ---
  window.addEventListener("carrot:visionstatechange", function (ev) {
    const st = (ev && ev.detail && ev.detail.state) || window.CarrotVisionState;
    if (!st) return;
    ensureVideoBound();
    const sig = (st.phase || "") + "|" + (st.reason || "");
    if (sig === lastPhaseSig) return;
    lastPhaseSig = sig;
    record("phase", { ph: st.phase, cs: st.controlState, reason: st.reason || "" });
  });

  window.addEventListener("carrot:rtctrace", function (ev) {
    const detail = ev && ev.detail;
    if (!detail || typeof detail !== "object") return;
    record("rtctrace", detail);
  });

  window.addEventListener("carrot:visionlifecycle", function (ev) {
    const detail = ev && ev.detail;
    if (!detail || typeof detail !== "object") return;
    record("lifecycle", detail);
  });

  document.addEventListener("visibilitychange", function () {
    record("vis", { state: document.visibilityState });
  });

  window.addEventListener("error", function (event) {
    recordConsole("window.error", [
      event?.message || "error",
      event?.filename || "",
      event?.lineno || "",
      event?.colno || "",
      event?.error?.stack || event?.error || "",
    ]);
  });

  window.addEventListener("unhandledrejection", function (event) {
    recordConsole("unhandledrejection", [
      event?.reason?.stack || event?.reason?.message || event?.reason || "",
    ]);
  });

  const netInfo = navigator.connection;
  if (netInfo && typeof netInfo.addEventListener === "function") {
    netInfo.addEventListener("change", function () {
      record("netchg", {
        netType: netInfo.effectiveType || "?",
        downlink: netInfo.downlink != null ? netInfo.downlink : null,
        rtt: netInfo.rtt != null ? netInfo.rtt : null,
      });
    });
  }

  statsTick();
  window.addEventListener("beforeunload", function () { persist(true); });

  // --- phone-friendly export button on the drive page ---
  function ensureButton() {
    if (!document.body || document.getElementById("carrotRtcPerfLogBtn") || document.getElementById("carrotDiagBtn")) return;
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
    dumpBundle: dumpBundle,
    dumpConsole: dumpConsole,
    download: download,
    uploadDiscord: uploadDiscord,
    copy: copy,
    clear: clear,
    get entries() { return entries; },
  };
})();
