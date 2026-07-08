"use strict";

// Auto-Tuner page — web port of the on-device AutoTunerHistoryPanel / GraphWidget
// (selfdrive/ui/qt/offroad/settings.cc). Portrait-first layout for the C4 web app.
//
// Data source (read via /api/params_bulk, writes via /api/param_set):
//   CarrotLearningHistory — change-history array (drives the graph + cards)
//   CarrotLearningData    — cumulative stats blob (diagnostics: phase5~9, band_sec)
//   CarrotLearningActive / CarrotTunerApplyLat / CarrotTunerApplyLong — toggles
//
// No chart dependency: the multi-series graph is drawn on a <canvas>, porting the
// QPainter algorithm 1:1 (30-point timeline, large-scale params hidden unless soloed,
// tap a node to inspect that snapshot).

const TUNER_PARAM_KEYS = [
  "CarrotLearningHistory",
  "CarrotLearningData",
  "CarrotLearningActive",
  "CarrotTunerApplyLat",
  "CarrotTunerApplyLong",
];
const TUNER_CHART_LIMIT = 50;
const TUNER_LARGE_SCALE = 300.0; // |value| above this → hidden unless that param is soloed

// Stable color palette assigned to series in sorted-key order.
const TUNER_COLORS = [
  "#4fc3f7", "#81c784", "#ffb74d", "#e57373", "#ba68c8", "#4db6ac",
  "#fff176", "#a1887f", "#7986cb", "#f06292", "#90a4ae", "#aed581",
  "#64b5f6", "#ff8a65", "#9575cd", "#4dd0e1",
];

const tunerState = {
  built: false,
  history: [],          // parsed array, stored order (newest first)
  data: null,           // parsed CarrotLearningData
  active: false,
  applyLat: false,
  applyLong: false,
  timestamps: [],       // oldest → newest
  series: {},           // param → [values aligned to timestamps]
  colors: {},           // param → color
  categories: {},       // param → group label
  selectedParam: null,  // soloed param (null = all)
  selectedIndex: -1,    // tapped timestamp index (-1 = none)
  activeCategory: null, // category filter (null = all)
  loading: false,
};

function tT(key, fallback) {
  try {
    const s = (typeof UI_STRINGS !== "undefined") && UI_STRINGS[LANG] && UI_STRINGS[LANG][key];
    return s || fallback;
  } catch (e) {
    return fallback;
  }
}

function tunerSafeParse(raw, fallback) {
  if (raw === undefined || raw === null) return fallback;
  if (typeof raw !== "string") return raw;
  const s = raw.trim();
  if (!s) return fallback;
  try {
    return JSON.parse(s);
  } catch (e) {
    return fallback;
  }
}

function tunerTruthy(v) {
  if (typeof v === "boolean") return v;
  if (typeof v === "number") return v !== 0;
  if (typeof v === "string") return ["1", "true", "on", "yes"].includes(v.trim().toLowerCase());
  return false;
}

// ── Entry point (called from runPageEnter) ───────────────────────────────
function initTunerPage() {
  const page = document.getElementById("pageTuner");
  if (!page) return;
  // Force the container visible. Some transition paths in the landscape/split
  // layout leave #pageTuner hidden/display:none even when it's the current page,
  // which is why the scaffold built but nothing showed.
  page.hidden = false;
  page.removeAttribute("aria-hidden");
  if (page.style.display === "none") page.style.display = "";
  // (Re)build the scaffold whenever .tuner-wrap is absent — the built flag alone
  // is unreliable because the page container can be cleared by transitions.
  if (!page.querySelector(".tuner-wrap")) {
    tunerBuildScaffold(page);
    tunerState.built = true;
  }
  tunerRefresh();
}

// Backup invocation. The runPageEnter() hook can fail to reach the tuner branch
// in the landscape/split layout (an earlier step in commitPageChange may abort
// before runPageEnter), leaving the page blank. carrot:pagechange is dispatched
// for every page switch right before runPageEnter, so binding here guarantees the
// page initializes. initTunerPage() is idempotent (built flag + refresh).
if (typeof window !== "undefined") {
  window.addEventListener("carrot:pagechange", (ev) => {
    if (ev && ev.detail && ev.detail.page === "tuner") {
      try { initTunerPage(); } catch (e) { console.error("[tuner] init failed", e); }
    }
  });
}

function tunerBuildScaffold(page) {
  page.innerHTML = `
    <div class="tuner-wrap">
      <div class="tuner-header">
        <h2 class="tuner-title">${tT("tuner_title", "Auto-Tuner")}</h2>
        <button id="tunerRefreshBtn" class="tuner-icon-btn" type="button" aria-label="${tT("tuner_refresh", "Refresh")}">&#x21bb;</button>
      </div>
      <div id="tunerChips" class="tuner-chips"></div>
      <div class="tuner-actions">
        <button id="tunerParamResetBtn" class="tuner-action-btn tuner-action-btn--danger" type="button">${tT("tuner_param_reset", "Parameter Init. Reset")}</button>
        <button id="tunerClearLogsBtn" class="tuner-action-btn tuner-action-btn--warn" type="button">${tT("tuner_clear_logs", "Clear All Logs")}</button>
      </div>
      <div id="tunerSummary" class="tuner-summary"></div>
      <div class="tuner-card tuner-graph-card">
        <div id="tunerGraphTitle" class="tuner-section-title">${tT("tuner_trend", "Parameter trend (last 30)")}</div>
        <div class="tuner-canvas-wrap">
          <canvas id="tunerCanvas" class="tuner-canvas"></canvas>
        </div>
        <div id="tunerAxis" class="tuner-axis"></div>
        <div id="tunerSnapshot" class="tuner-snapshot" hidden></div>
      </div>
      <div id="tunerCategories" class="tuner-filters"></div>
      <div id="tunerLegend" class="tuner-legend"></div>
      <div id="tunerDiag" class="tuner-diag"></div>
      <div id="tunerCardsTitle" class="tuner-section-title">${tT("tuner_history", "Change history")}</div>
      <div id="tunerCards" class="tuner-cards"></div>
    </div>`;

  const refreshBtn = page.querySelector("#tunerRefreshBtn");
  if (refreshBtn) refreshBtn.addEventListener("click", () => tunerRefresh());

  const canvas = page.querySelector("#tunerCanvas");
  if (canvas) canvas.addEventListener("click", (ev) => tunerOnCanvasClick(ev));

  const resetBtn = page.querySelector("#tunerParamResetBtn");
  if (resetBtn) resetBtn.addEventListener("click", () => tunerParamReset());
  const clearBtn = page.querySelector("#tunerClearLogsBtn");
  if (clearBtn) clearBtn.addEventListener("click", () => tunerClearLogs());

  window.addEventListener("resize", () => {
    if (!document.getElementById("pageTuner")?.hidden) tunerRenderGraph();
  });
}

// ── Data load ─────────────────────────────────────────────────────────────
async function tunerRefresh() {
  if (typeof bulkGet !== "function") return;
  tunerState.loading = true;
  let values = {};
  try {
    values = await bulkGet(TUNER_PARAM_KEYS);
  } catch (e) {
    values = {};
  }
  tunerState.loading = false;

  const hist = tunerSafeParse(values.CarrotLearningHistory, []);
  tunerState.history = Array.isArray(hist) ? hist : [];
  tunerState.data = tunerSafeParse(values.CarrotLearningData, null);
  tunerState.active = tunerTruthy(values.CarrotLearningActive);
  tunerState.applyLat = tunerTruthy(values.CarrotTunerApplyLat);
  tunerState.applyLong = tunerTruthy(values.CarrotTunerApplyLong);

  tunerComputeSeries();
  tunerRenderChips();
  tunerRenderSummary();
  tunerRenderCategories();
  tunerRenderLegend();
  tunerRenderGraph();
  tunerRenderDiagnostics();
  tunerRenderCards();
}

// Port of refreshHistory()'s timeline build: forward-fill each param's recommended
// value across the (oldest→newest) timeline, seeding pre-first points with 'current'.
function tunerComputeSeries() {
  const hist = tunerState.history;
  const nPoints = Math.min(hist.length, TUNER_CHART_LIMIT);

  const timestamps = [];
  const entries = [];
  for (let i = nPoints - 1; i >= 0; i--) {
    const item = hist[i] || {};
    timestamps.push(item.timestamp || "");
    entries.push(item);
  }

  // Gather all params + remember each param's category (group label).
  const categories = {};
  const paramSet = new Set();
  for (const entry of entries) {
    const changes = entry.changes || {};
    for (const group of Object.keys(changes)) {
      const gItems = changes[group] || {};
      for (const key of Object.keys(gItems)) {
        paramSet.add(key);
        categories[key] = group;
      }
    }
  }

  const series = {};
  const lastValues = {};
  for (const p of paramSet) series[p] = [];

  for (let t = 0; t < nPoints; t++) {
    const changes = entries[t].changes || {};
    const currentChanges = {};
    for (const group of Object.keys(changes)) {
      const gItems = changes[group] || {};
      for (const key of Object.keys(gItems)) {
        const rec = Number(gItems[key]?.recommended);
        if (Number.isFinite(rec)) currentChanges[key] = rec;
      }
    }
    for (const param of paramSet) {
      if (param in currentChanges) {
        lastValues[param] = currentChanges[param];
        series[param].push(currentChanges[param]);
      } else if (param in lastValues) {
        series[param].push(lastValues[param]);
      } else {
        // Seed with the 'current' of the first future occurrence.
        let initial = 0.0;
        for (let ft = t; ft < nPoints; ft++) {
          const fChanges = entries[ft].changes || {};
          let found = false;
          for (const group of Object.keys(fChanges)) {
            const gItems = fChanges[group] || {};
            if (param in gItems) {
              const cur = Number(gItems[param]?.current);
              if (Number.isFinite(cur)) initial = cur;
              found = true;
              break;
            }
          }
          if (found) break;
        }
        lastValues[param] = initial;
        series[param].push(initial);
      }
    }
  }

  // Assign stable colors by sorted key.
  const colors = {};
  const sortedKeys = Array.from(paramSet).sort();
  sortedKeys.forEach((p, idx) => { colors[p] = TUNER_COLORS[idx % TUNER_COLORS.length]; });

  tunerState.timestamps = timestamps;
  tunerState.series = series;
  tunerState.colors = colors;
  tunerState.categories = categories;
  if (tunerState.selectedParam && !paramSet.has(tunerState.selectedParam)) tunerState.selectedParam = null;
  if (tunerState.selectedIndex >= nPoints) tunerState.selectedIndex = -1;
}

// ── Status chips (Active / Apply LAT / Apply LONG) ──────────────────────────
function tunerRenderChips() {
  const host = document.getElementById("tunerChips");
  if (!host) return;
  host.innerHTML = "";

  const chips = [
    { key: "CarrotLearningActive", on: tunerState.active, kind: "active",
      onLabel: tT("tuner_learn_on", "Learning ON"), offLabel: tT("tuner_learn_off", "Learning OFF") },
    { key: "CarrotTunerApplyLat", on: tunerState.applyLat, kind: "lat",
      onLabel: tT("tuner_apply_lat_on", "LAT ON"), offLabel: tT("tuner_apply_lat_off", "LAT OFF") },
    { key: "CarrotTunerApplyLong", on: tunerState.applyLong, kind: "long",
      onLabel: tT("tuner_apply_long_on", "LONG ON"), offLabel: tT("tuner_apply_long_off", "LONG OFF") },
  ];

  for (const c of chips) {
    const btn = document.createElement("button");
    btn.type = "button";
    btn.className = "tuner-chip" + (c.on ? " is-on" : " is-off");
    btn.textContent = c.on ? c.onLabel : c.offLabel;
    btn.addEventListener("click", () => tunerToggleParam(c.key, c.on));
    host.appendChild(btn);
  }
}

// Learning(master) ↔ LAT/LONG 연동 규칙 (C4 등 단말 메뉴가 없는 차량 지원):
//   • Learning ON  → LAT ON, LONG ON 자동 동반
//   • Learning OFF → LAT OFF, LONG OFF 자동 동반
//   • Learning OFF 상태에서 LAT 또는 LONG 중 하나라도 ON → Learning ON
//   • LAT/LONG 모두 OFF가 되면 → Learning OFF
async function tunerToggleParam(key, currentlyOn) {
  if (typeof setParam !== "function") return;
  const next = currentlyOn ? 0 : 1;
  const onFlag = !currentlyOn;
  try {
    if (key === "CarrotLearningActive") {
      // 마스터 스위치: LAT/LONG을 함께 켜고/끈다
      await setParam("CarrotLearningActive", next);
      await setParam("CarrotTunerApplyLat", next);
      await setParam("CarrotTunerApplyLong", next);
      tunerState.active = onFlag;
      tunerState.applyLat = onFlag;
      tunerState.applyLong = onFlag;
    } else {
      // 개별 LAT/LONG 토글
      await setParam(key, next);
      if (key === "CarrotTunerApplyLat") tunerState.applyLat = onFlag;
      else if (key === "CarrotTunerApplyLong") tunerState.applyLong = onFlag;
      // Learning은 LAT/LONG 상태를 따라감: 하나라도 ON이면 ON, 모두 OFF면 OFF
      const anyOn = tunerState.applyLat || tunerState.applyLong;
      if (anyOn !== tunerState.active) {
        await setParam("CarrotLearningActive", anyOn ? 1 : 0);
        tunerState.active = anyOn;
      }
    }
    tunerRenderChips();
  } catch (e) {
    if (typeof showAppToast === "function") showAppToast(tT("tuner_save_failed", "Save failed"), { kind: "error" });
  }
}

// Parameter Init. Reset — mirrors the on-device "Factory Reset": sets the
// CarrotTunerFactoryReset flag; the carrot_learning process restores every tuner
// param to its install default and deletes learning data/history.
async function tunerParamReset() {
  if (typeof setParam !== "function") return;
  const confirmFn = (typeof appConfirm === "function") ? appConfirm : async () => window.confirm("Reset?");
  const ok = await confirmFn(
    tT("tuner_reset_confirm", "Restore all auto-tuned parameters to defaults and delete learning data/history?"),
    { title: tT("tuner_param_reset", "Parameter Init. Reset") },
  );
  if (!ok) return;
  try {
    await setParam("CarrotTunerFactoryReset", 1);
    if (typeof showAppToast === "function") showAppToast(tT("tuner_reset_done", "Parameters restored to defaults"));
    setTimeout(() => tunerRefresh(), 900);
  } catch (e) {
    if (typeof showAppToast === "function") showAppToast(tT("tuner_save_failed", "Save failed"), { kind: "error" });
  }
}

// Clear All Logs — clears accumulated learning data and the change history.
async function tunerClearLogs() {
  if (typeof setParam !== "function") return;
  const confirmFn = (typeof appConfirm === "function") ? appConfirm : async () => window.confirm("Clear?");
  const ok = await confirmFn(
    tT("tuner_clear_confirm", "Delete all learning data and change history?"),
    { title: tT("tuner_clear_logs", "Clear All Logs") },
  );
  if (!ok) return;
  try {
    await setParam("CarrotLearningClear", 1);
    await setParam("CarrotLearningHistory", "[]");
    if (typeof showAppToast === "function") showAppToast(tT("tuner_clear_done", "Logs cleared"));
    setTimeout(() => tunerRefresh(), 500);
  } catch (e) {
    if (typeof showAppToast === "function") showAppToast(tT("tuner_save_failed", "Save failed"), { kind: "error" });
  }
}

// ── Summary line ────────────────────────────────────────────────────────────
function tunerRenderSummary() {
  const host = document.getElementById("tunerSummary");
  if (!host) return;
  const changeCount = tunerState.history.length;

  let drivenSec = 0;
  const d = tunerState.data;
  if (d && Array.isArray(d.band_sec)) drivenSec = d.band_sec.reduce((a, b) => a + (Number(b) || 0), 0);

  let latestText = "";
  const latest = tunerState.history[0];
  if (latest && latest.changes) {
    const parts = [];
    for (const group of Object.keys(latest.changes)) {
      const gItems = latest.changes[group] || {};
      for (const key of Object.keys(gItems)) {
        const it = gItems[key] || {};
        parts.push(`${key} ${it.current ?? "?"}→${it.recommended ?? "?"}`);
      }
    }
    if (parts.length) latestText = parts.slice(0, 2).join(", ");
  }

  const driveLabel = tT("tuner_driven", "Driven");
  const changesLabel = tT("tuner_changes", "changes");
  const latestLabel = tT("tuner_latest", "Latest");
  host.innerHTML = `
    <div class="tuner-summary-line">${driveLabel} ${tunerFmtDuration(drivenSec)} · ${changeCount} ${changesLabel}</div>
    ${latestText ? `<div class="tuner-summary-latest">${latestLabel} ▸ ${tunerEsc(latestText)}</div>` : ""}`;
}

function tunerFmtDuration(sec) {
  sec = Math.round(sec || 0);
  if (sec < 60) return `${sec}s`;
  const m = Math.floor(sec / 60);
  const s = sec % 60;
  if (m < 60) return `${m}m ${s}s`;
  const h = Math.floor(m / 60);
  return `${h}h ${m % 60}m`;
}

// ── Category filter chips ───────────────────────────────────────────────────
function tunerRenderCategories() {
  const host = document.getElementById("tunerCategories");
  if (!host) return;
  host.innerHTML = "";

  const cats = [];
  const seen = new Set();
  for (const p of Object.keys(tunerState.categories)) {
    const c = tunerState.categories[p];
    if (!seen.has(c)) { seen.add(c); cats.push(c); }
  }
  if (cats.length <= 1) return; // nothing to filter

  const makeChip = (label, value) => {
    const btn = document.createElement("button");
    btn.type = "button";
    btn.className = "tuner-filter-chip" + (tunerState.activeCategory === value ? " is-active" : "");
    btn.textContent = label;
    btn.addEventListener("click", () => {
      tunerState.activeCategory = (tunerState.activeCategory === value) ? null : value;
      tunerState.selectedParam = null;
      tunerRenderCategories();
      tunerRenderLegend();
      tunerRenderGraph();
    });
    return btn;
  };

  host.appendChild(makeChip(tT("tuner_all", "All"), null));
  for (const c of cats) host.appendChild(makeChip(tunerCatShort(c), c));
}

// "가속 (Acceleration)" → "가속" / "Acceleration" depending on lang heuristic.
function tunerCatShort(c) {
  if (!c) return c;
  const m = c.match(/^\s*([^(]+?)\s*\(([^)]+)\)\s*$/);
  if (!m) return c;
  return (typeof LANG !== "undefined" && LANG === "ko") ? m[1].trim() : m[2].trim();
}

// ── Legend chips (one per visible param; tap to solo) ───────────────────────
function tunerVisibleParams() {
  let params = Object.keys(tunerState.series);
  if (tunerState.activeCategory) {
    params = params.filter((p) => tunerState.categories[p] === tunerState.activeCategory);
  }
  return params.sort();
}

function tunerRenderLegend() {
  const host = document.getElementById("tunerLegend");
  if (!host) return;
  host.innerHTML = "";
  for (const p of tunerVisibleParams()) {
    const btn = document.createElement("button");
    btn.type = "button";
    const soloed = tunerState.selectedParam === p;
    btn.className = "tuner-legend-chip" + (soloed ? " is-solo" : "");
    btn.innerHTML = `<span class="tuner-dot" style="background:${tunerState.colors[p]}"></span>${tunerEsc(p)}`;
    btn.addEventListener("click", () => {
      tunerState.selectedParam = soloed ? null : p;
      tunerRenderLegend();
      tunerRenderGraph();
    });
    host.appendChild(btn);
  }
}

// ── Canvas graph (port of AutoTunerGraphWidget::paintEvent) ──────────────────
function tunerGraphLayout(canvas) {
  const wrap = canvas.parentElement;
  const cssW = Math.max(240, wrap ? wrap.clientWidth : 320);
  const cssH = 300;
  const dpr = window.devicePixelRatio || 1;
  canvas.style.width = cssW + "px";
  canvas.style.height = cssH + "px";
  canvas.width = Math.round(cssW * dpr);
  canvas.height = Math.round(cssH * dpr);
  const ctx = canvas.getContext("2d");
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  return { ctx, cssW, cssH };
}

function tunerRenderGraph() {
  const canvas = document.getElementById("tunerCanvas");
  if (!canvas) return;
  const { ctx, cssW, cssH } = tunerGraphLayout(canvas);

  ctx.fillStyle = "#11161d";
  ctx.fillRect(0, 0, cssW, cssH);

  const ts = tunerState.timestamps;
  const axis = document.getElementById("tunerAxis");
  const snap = document.getElementById("tunerSnapshot");

  if (!ts.length) {
    ctx.fillStyle = "#7a8694";
    ctx.font = "16px " + tunerFont();
    ctx.textAlign = "center";
    ctx.textBaseline = "middle";
    ctx.fillText(tT("tuner_no_data", "No historical data to display"), cssW / 2, cssH / 2);
    if (axis) axis.textContent = "";
    if (snap) snap.hidden = true;
    return;
  }

  const mL = 32, mR = 12, mT = 16, mB = 22;
  const gx = mL, gy = mT, gw = cssW - mL - mR, gh = cssH - mT - mB;
  let stepsX = ts.length - 1;
  if (stepsX < 1) stepsX = 1;

  const params = tunerVisibleParams();
  const isLargeScale = (p) => tunerState.series[p].some((v) => Math.abs(v) > TUNER_LARGE_SCALE);
  const excluded = (p) => isLargeScale(p) && p !== tunerState.selectedParam;
  const drawn = params.filter((p) => !(tunerState.selectedParam ? p !== tunerState.selectedParam : excluded(p)));

  // global min/max over drawn params
  let gmin = 0, gmax = 0, first = true;
  for (const p of drawn) {
    for (const v of tunerState.series[p]) {
      if (first) { gmin = gmax = v; first = false; }
      else { if (v < gmin) gmin = v; if (v > gmax) gmax = v; }
    }
  }
  if (first) { gmin = 0; gmax = 1; }
  if (gmin === gmax) { gmin -= 1; gmax += 1; }
  const pad = (gmax - gmin) * 0.12;
  gmin -= pad; gmax += pad;

  const yOf = (v) => gy + gh - ((v - gmin) / (gmax - gmin)) * gh;
  const xOf = (i) => gx + (i * gw) / stepsX;

  // grid
  ctx.strokeStyle = "#222a34";
  ctx.lineWidth = 1;
  for (let i = 0; i <= stepsX; i++) {
    const x = xOf(i);
    ctx.beginPath(); ctx.moveTo(x, gy); ctx.lineTo(x, gy + gh); ctx.stroke();
  }
  const stepsY = 4;
  ctx.fillStyle = "#6b7785";
  ctx.font = "11px " + tunerFont();
  ctx.textAlign = "right";
  ctx.textBaseline = "middle";
  for (let i = 0; i <= stepsY; i++) {
    const y = gy + (i * gh) / stepsY;
    ctx.beginPath(); ctx.moveTo(gx, y); ctx.lineTo(gx + gw, y); ctx.stroke();
    const val = gmax - (i * (gmax - gmin)) / stepsY;
    ctx.fillText(tunerFmtNum(val), gx - 4, y);
  }

  // selected vertical cursor
  if (tunerState.selectedIndex >= 0 && tunerState.selectedIndex < ts.length) {
    const x = xOf(tunerState.selectedIndex);
    ctx.strokeStyle = "rgba(255,255,255,0.5)";
    ctx.setLineDash([4, 4]);
    ctx.beginPath(); ctx.moveTo(x, gy); ctx.lineTo(x, gy + gh); ctx.stroke();
    ctx.setLineDash([]);
  }

  // series lines + nodes
  for (const p of drawn) {
    const vals = tunerState.series[p];
    ctx.strokeStyle = tunerState.colors[p];
    ctx.lineWidth = (p === tunerState.selectedParam) ? 3 : 2;
    ctx.beginPath();
    vals.forEach((v, i) => {
      const x = xOf(i), y = yOf(v);
      if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
    });
    ctx.stroke();
    if (tunerState.selectedIndex >= 0) {
      const i = tunerState.selectedIndex;
      ctx.fillStyle = tunerState.colors[p];
      ctx.beginPath(); ctx.arc(xOf(i), yOf(vals[i]), 3.5, 0, Math.PI * 2); ctx.fill();
    }
  }

  // x-axis labels (first / last timestamp)
  if (axis) {
    const a = tunerShortTs(ts[0]);
    const b = tunerShortTs(ts[ts.length - 1]);
    axis.innerHTML = `<span>${tunerEsc(a)}</span><span>${tunerEsc(b)}</span>`;
  }

  tunerRenderSnapshot();
}

function tunerRenderSnapshot() {
  const snap = document.getElementById("tunerSnapshot");
  if (!snap) return;
  const i = tunerState.selectedIndex;
  if (i < 0 || i >= tunerState.timestamps.length) { snap.hidden = true; snap.innerHTML = ""; return; }

  // The entry that produced this timeline point (timeline is oldest→newest).
  const nPoints = Math.min(tunerState.history.length, TUNER_CHART_LIMIT);
  const entry = tunerState.history[nPoints - 1 - i];
  const rows = [];
  if (entry && entry.changes) {
    for (const group of Object.keys(entry.changes)) {
      const gItems = entry.changes[group] || {};
      for (const key of Object.keys(gItems)) {
        const it = gItems[key] || {};
        rows.push(`<div class="tuner-snap-row"><span class="tuner-snap-k">${tunerEsc(key)}</span><span class="tuner-snap-v">${it.current ?? "?"} → ${it.recommended ?? "?"}</span></div>`);
      }
    }
  }
  snap.hidden = false;
  snap.innerHTML =
    `<div class="tuner-snap-title">▣ ${tunerEsc(tunerState.timestamps[i])}</div>` +
    (rows.length ? rows.join("") : `<div class="tuner-snap-row">${tT("tuner_no_change", "no change")}</div>`);
}

function tunerOnCanvasClick(ev) {
  const ts = tunerState.timestamps;
  if (!ts.length) return;
  const canvas = ev.currentTarget;
  const rect = canvas.getBoundingClientRect();
  const cssW = rect.width;
  const mL = 32, mR = 12;
  const gx = mL, gw = cssW - mL - mR;
  let stepsX = ts.length - 1;
  if (stepsX < 1) stepsX = 1;
  const clickX = ev.clientX - rect.left;

  let closest = 0, minDist = Infinity;
  for (let i = 0; i < ts.length; i++) {
    const nodeX = gx + (i * gw) / stepsX;
    const dist = Math.abs(clickX - nodeX);
    if (dist < minDist) { minDist = dist; closest = i; }
  }
  tunerState.selectedIndex = (minDist < 40) ? ((tunerState.selectedIndex === closest) ? -1 : closest) : -1;
  tunerRenderGraph();
}

// ── Diagnostics (CarrotLearningData → collapsible) ──────────────────────────
function tunerRenderDiagnostics() {
  const host = document.getElementById("tunerDiag");
  if (!host) return;
  const d = tunerState.data;
  if (!d) { host.innerHTML = ""; return; }

  const sections = [];

  const p8 = d.phase8;
  if (p8 && p8.long_samples > 0) {
    const lag = p8.long_lag_count / p8.long_samples;
    const over = p8.long_overshoot_count / p8.long_samples;
    const meanErr = p8.long_err_sum / p8.long_samples;
    sections.push({
      title: tT("tuner_diag_long", "Longitudinal (phase8)"),
      rows: [
        [tT("tuner_diag_err", "Mean track error"), `${meanErr.toFixed(2)} m/s²`, null],
        [tT("tuner_diag_lag", "Lag"), `${p8.long_lag_count}/${p8.long_samples} (${Math.round(lag * 100)}%)`, lag],
        [tT("tuner_diag_over", "Overshoot"), `${p8.long_overshoot_count}/${p8.long_samples} (${Math.round(over * 100)}%)`, over],
      ],
    });
  }

  const p7 = d.phase7;
  if (p7 && p7.stop_events > 0) {
    sections.push({
      title: tT("tuner_diag_stop", "Stops (phase7)"),
      rows: [
        [tT("tuner_diag_stop_events", "Stop events"), `${p7.stop_events}`, null],
        [tT("tuner_diag_harsh", "Harsh stops"), `${p7.stop_harsh_count || 0}`, null],
      ],
    });
  }

  if (!sections.length) { host.innerHTML = ""; return; }

  host.innerHTML = sections.map((sec) => `
    <details class="tuner-diag-section" open>
      <summary>${tunerEsc(sec.title)}</summary>
      <div class="tuner-diag-rows">
        ${sec.rows.map(([k, v, ratio]) => `
          <div class="tuner-diag-row">
            <span class="tuner-diag-k">${tunerEsc(k)}</span>
            <span class="tuner-diag-v">${tunerEsc(v)}</span>
            ${ratio == null ? "" : `<span class="tuner-diag-bar"><i style="width:${Math.min(100, Math.round(ratio * 100))}%"></i></span>`}
          </div>`).join("")}
      </div>
    </details>`).join("");
}

// ── History cards (+ restore) ───────────────────────────────────────────────
function tunerRenderCards() {
  const host = document.getElementById("tunerCards");
  if (!host) return;
  host.innerHTML = "";

  if (!tunerState.history.length) {
    host.innerHTML = `<div class="tuner-empty">${tT("tuner_no_data", "No historical data to display")}</div>`;
    return;
  }

  for (const entry of tunerState.history) {
    const card = document.createElement("div");
    card.className = "tuner-card tuner-hist-card";

    let catLabel = "";
    const changeRows = [];
    for (const group of Object.keys(entry.changes || {})) {
      if (!catLabel) catLabel = tunerCatShort(group);
      const gItems = entry.changes[group] || {};
      for (const key of Object.keys(gItems)) {
        const it = gItems[key] || {};
        const reason = it.band_kph ? `<div class="tuner-hist-reason">${tunerEsc(String(it.band_kph))}</div>` : "";
        changeRows.push(`
          <div class="tuner-hist-change">
            <span class="tuner-hist-key">${tunerEsc(key)}</span>
            <span class="tuner-hist-delta">${it.current ?? "?"} <i>→</i> ${it.recommended ?? "?"}</span>
          </div>${reason}`);
      }
    }

    card.innerHTML = `
      <div class="tuner-hist-top">
        <span class="tuner-hist-ts">${tunerEsc(entry.timestamp || "")}</span>
        ${catLabel ? `<span class="tuner-hist-cat">${tunerEsc(catLabel)}</span>` : ""}
      </div>
      ${changeRows.join("")}
      <div class="tuner-hist-actions">
        <button type="button" class="tuner-restore-btn" data-id="${tunerEsc(entry.id || "")}">${tT("tuner_restore", "Restore")}</button>
      </div>`;

    const btn = card.querySelector(".tuner-restore-btn");
    if (btn) btn.addEventListener("click", () => tunerRestore(entry.id));
    host.appendChild(card);
  }
}

// Port of restoreItem(): write each changed param's 'current', then drop the entry.
async function tunerRestore(id) {
  if (!id || typeof setParam !== "function") return;
  const confirmFn = (typeof appConfirm === "function") ? appConfirm : async () => window.confirm("Restore?");
  const ok = await confirmFn(
    tT("tuner_restore_confirm", "Restore the parameters to this state?"),
    { title: tT("tuner_restore", "Restore") },
  );
  if (!ok) return;

  const entry = tunerState.history.find((e) => e.id === id);
  if (!entry) return;

  try {
    for (const group of Object.keys(entry.changes || {})) {
      const gItems = entry.changes[group] || {};
      for (const key of Object.keys(gItems)) {
        const prev = Number(gItems[key]?.current);
        if (Number.isFinite(prev)) await setParam(key, Math.round(prev));
      }
    }
    const remaining = tunerState.history.filter((e) => e.id !== id);
    await setParam("CarrotLearningHistory", JSON.stringify(remaining));
    if (typeof showAppToast === "function") showAppToast(tT("tuner_restored", "Restored to previous values"));
    await tunerRefresh();
  } catch (e) {
    if (typeof showAppToast === "function") showAppToast(tT("tuner_save_failed", "Save failed"), { kind: "error" });
  }
}

// ── small utils ─────────────────────────────────────────────────────────────
function tunerEsc(s) {
  return String(s == null ? "" : s)
    .replace(/&/g, "&amp;").replace(/</g, "&lt;").replace(/>/g, "&gt;")
    .replace(/"/g, "&quot;").replace(/'/g, "&#39;");
}
function tunerFmtNum(v) {
  if (Math.abs(v) >= 1000) return Math.round(v).toString();
  if (Math.abs(v) >= 100) return v.toFixed(0);
  return v.toFixed(Math.abs(v) < 10 ? 1 : 0);
}
function tunerShortTs(ts) {
  if (!ts) return "";
  // "2026-06-25 17:47" → "06-25 17:47"
  const m = String(ts).match(/(\d{2})-(\d{2})\s+(\d{2}:\d{2})/);
  return m ? `${m[1]}-${m[2]} ${m[3]}` : String(ts);
}
function tunerFont() {
  return 'Roboto, system-ui, -apple-system, sans-serif';
}
