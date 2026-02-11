const DEBUG_UI = false;

let SETTINGS = null;
let CURRENT_GROUP = null;
let LANG = "ko"; // "ko" | "en"

let UNIT_CYCLE = [1, 2, 5, 10, 50, 100];
const UNIT_INDEX = {}; // per name

// Car select data
let CARS = null;                 // { makers: {Hyundai:[...], Genesis:[...]} }
let CURRENT_MAKER = null;

const btnHome = document.getElementById("btnHome");
const btnSetting = document.getElementById("btnSetting");
const btnFleet = document.getElementById("btnFleet");
const btnLang = document.getElementById("btnLang");
const langLabel = document.getElementById("langLabel");
const btnTools = document.getElementById("btnTools");
const btnToolsBack = document.getElementById("btnToolsBack");

btnTools.onclick = () => showPage("tools");
btnToolsBack.onclick = () => showPage("home");

const btnChangeCar = document.getElementById("btnChangeCar");
const curCarLabelCar = document.getElementById("curCarLabelCar");
const curCarLabelSetting = document.getElementById("curCarLabelSetting");

// Setting screens
const settingTitle = document.getElementById("settingTitle");
const btnBackGroups = document.getElementById("btnBackGroups");
const screenGroups = document.getElementById("settingScreenGroups");
const screenItems = document.getElementById("settingScreenItems");
const itemsTitle = document.getElementById("itemsTitle");

// Car screens
const carTitle = document.getElementById("carTitle");
const btnBackCar = document.getElementById("btnBackCar");
const carMeta = document.getElementById("carMeta");
const carScreenMakers = document.getElementById("carScreenMakers");
const carScreenModels = document.getElementById("carScreenModels");
const makerList = document.getElementById("makerList");
const modelList = document.getElementById("modelList");
const modelTitle = document.getElementById("modelTitle");
const modelMeta = document.getElementById("modelMeta");

btnHome.onclick = () => showPage("home", true);
btnSetting.onclick = () => showPage("setting", true);

btnFleet.onclick = () => {
  const ip = location.hostname;
  const url = `http://${ip}:8082/`;
  window.open(url, "_blank", "noopener");
};

btnLang.onclick = () => toggleLang();

btnChangeCar.onclick = () => showPage("car", true);
btnBackCar.onclick = () => history.back();
carTitle.onclick = () => history.back();
modelTitle.onclick = () => showCarScreen("makers"); // 모델화면에서 타이틀 눌러 makers로

// Branch select
let BRANCHES = [];
const branchTitle = document.getElementById("branchTitle");
const btnBackBranch = document.getElementById("btnBackBranch");
const branchMeta = document.getElementById("branchMeta");
const branchList = document.getElementById("branchList");

// Quick Link
const quickLink = document.getElementById("quickLink");

btnBackBranch.onclick = () => history.back();
branchTitle.onclick = () => history.back();

function showPage(page, pushHistory = false) {
  document.getElementById("pageHome").style.display = (page === "home") ? "" : "none";
  document.getElementById("pageSetting").style.display = (page === "setting") ? "" : "none";
  document.getElementById("pageCar").style.display = (page === "car") ? "" : "none";
  document.getElementById("pageTools").style.display = (page === "tools") ? "" : "none";
  document.getElementById("pageBranch").style.display = (page === "branch") ? "" : "none";

  btnHome.classList.toggle("active", page === "home");
  btnSetting.classList.toggle("active", page === "setting");

  if (page === "home") {
    loadCurrentCar().catch(() => {});
    updateQuickLink().catch(() => {});
  }

  if (page === "setting") {
    showSettingScreen("groups", false);
    if (!SETTINGS) loadSettings();
  }

  if (page === "car") {
    showCarScreen("makers", false);
    if (!CARS) loadCars();
  }
  if (page === "tools") {
    initToolsPage();
  }

  const state =
    (page === "home") ? { page: "home" } :
    (page === "setting") ? { page: "setting", screen: "groups", group: null } :
    (page === "car") ? { page: "car", screen: "makers", maker: null } :
    (page === "tools") ? { page: "tools" } :
    (page === "branch") ? { page: "branch" } :
    { page: "home" };

  if (pushHistory) history.pushState(state, "");
  else history.replaceState(state, "");
}

/* ---------- screen transitions (Setting) ---------- */
function showSettingScreen(which, pushHistory = false) {
  const isGroups = (which === "groups");
  const showEl = isGroups ? screenGroups : screenItems;
  const hideEl = isGroups ? screenItems : screenGroups;

  btnBackGroups.style.display = isGroups ? "none" : "";
  settingTitle.textContent = isGroups ? "Setting" : ("Setting - " + (CURRENT_GROUP || ""));

  showEl.style.display = "";
  requestAnimationFrame(() => showEl.classList.remove("hidden"));

  hideEl.classList.add("hidden");
  setTimeout(() => { hideEl.style.display = "none"; }, 170);

  if (pushHistory) {
    history.pushState({ page: "setting", screen: which, group: CURRENT_GROUP || null }, "");
  }
}

btnBackGroups.onclick = () => history.back();
settingTitle.onclick = () => history.back();
itemsTitle.onclick = () => history.back();

/* ---------- screen transitions (Car) ---------- */
function showCarScreen(which, pushHistory = false) {
  const isMakers = (which === "makers");
  const showEl = isMakers ? carScreenMakers : carScreenModels;
  const hideEl = isMakers ? carScreenModels : carScreenMakers;

  showEl.style.display = "";
  requestAnimationFrame(() => showEl.classList.remove("hidden"));

  hideEl.classList.add("hidden");
  setTimeout(() => { hideEl.style.display = "none"; }, 170);

  if (pushHistory) {
    history.pushState({ page: "car", screen: which, maker: CURRENT_MAKER || null }, "");
  }
}

function toggleLang() {
  LANG = (LANG === "ko") ? "en" : "ko";
  langLabel.textContent = (LANG === "ko") ? "KO" : "EN";
  if (SETTINGS) {
    renderGroups();
    if (CURRENT_GROUP) renderItems(CURRENT_GROUP);
  }
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
  if (LANG === "ko") return (p[keyKo] ?? fallback);
  return (p[keyEn] ?? p[keyKo] ?? fallback);
}

function clamp(v, mn, mx) {
  if (Number.isFinite(mn) && v < mn) return mn;
  if (Number.isFinite(mx) && v > mx) return mx;
  return v;
}

/* ---------- Params helpers ---------- */
async function bulkGet(names) {
  const q = encodeURIComponent(names.join(","));
  const r = await fetch("/api/params_bulk?names=" + q);
  const j = await r.json();
  if (!j.ok) throw new Error(j.error || "bulk failed");
  return j.values || {};
}

async function setParam(name, value) {
  const r = await fetch("/api/param_set", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ name, value })
  });
  const j = await r.json();
  if (!j.ok) throw new Error(j.error || "set failed");
  return true;
}

/* ---------- Home: current car ---------- */
async function loadCurrentCar() {
  try {
    const values = await bulkGet(["CarSelected3"]);
    const v = values["CarSelected3"];
    curCarLabelCar.textContent = (v && String(v).trim().length) ? String(v) : "-";
    curCarLabelSetting.textContent = (v && String(v).trim().length) ? String(v) : "-";
  } catch (e) {
    curCarLabelCar.textContent = "-";
    curCarLabelSetting.textContent = "-";
  }
}

/* ---------- Cars: load list + maker/model UI ---------- */
async function loadCars() {
  carMeta.textContent = "loading...";
  makerList.innerHTML = "";
  modelList.innerHTML = "";
  CURRENT_MAKER = null;
  showCarScreen("makers", false);

  const r = await fetch("/api/cars");
  const j = await r.json();
  if (!j.ok) {
    carMeta.textContent = "Failed: " + (j.error || "unknown");
    return;
  }
  CARS = j; // { ok:true, sources:[...], makers:{Hyundai:[...],Genesis:[...]} ... }

  const sources = (j.sources || []).join(", ");
  carMeta.textContent = sources ? ("sources: " + sources) : "ok";

  renderMakers();
}

function renderMakers() {
  makerList.innerHTML = "";
  const makers = CARS && CARS.makers ? Object.keys(CARS.makers) : [];
  makers.sort((a, b) => a.localeCompare(b));

  for (const mk of makers) {
    const arr = CARS.makers[mk] || [];
    const b = document.createElement("button");
    b.className = "btn groupBtn";
    b.textContent = `${mk} (${arr.length})`;
    b.onclick = () => {
      CURRENT_MAKER = mk;
      renderModels(mk);
      showCarScreen("models", true);
    };
    makerList.appendChild(b);
  }
}

function renderModels(maker) {
  modelList.innerHTML = "";
  const arr = (CARS.makers && CARS.makers[maker]) ? CARS.makers[maker] : [];
  modelTitle.textContent = maker;
  modelMeta.textContent = `${arr.length} models`;

  // 긴 목록이니까 버튼 폭/탭 편하게: groupBtn 재사용
  for (const fullLine of arr) {
    // fullLine 예: "Hyundai Grandeur 2018-19"
    // CarSelected3에는 maker를 빼고 넣어야 함 → "Grandeur 2018-19"
    const modelOnly = stripMaker(fullLine, maker);

    const b = document.createElement("button");
    b.className = "btn groupBtn";
    b.textContent = modelOnly;
    b.onclick = () => onSelectCar(maker, modelOnly, fullLine);
    modelList.appendChild(b);
  }
}

function stripMaker(fullLine, maker) {
  // maker + 공백을 1번만 제거
  const prefix = maker + " ";
  if (fullLine.startsWith(prefix)) return fullLine.slice(prefix.length).trim();
  // 혹시 "Hyundai"가 아닌 다른 표기면 fallback: 첫 단어 제거
  const sp = fullLine.split(" ");
  if (sp.length >= 2) return sp.slice(1).join(" ").trim();
  return fullLine.trim();
}

async function onSelectCar(maker, modelOnly, fullLine) {
  const ok = confirm(`Select this car?\n\n${maker} ${modelOnly}\n\nThis will set CarSelected3 = "${modelOnly}".`);
  if (!ok) return;

  try {
    await setParam("CarSelected3", fullLine);
  } catch (e) {
    alert("Failed to set CarSelected3: " + e.message);
    return;
  }

  // Home 표시 업데이트
  curCarLabelCar.textContent = modelOnly;
  curCarLabelSetting.textContent = modelOnly;

  const rb = confirm("Reboot now?");
  if (!rb) {
    alert("Selected. Reboot later to apply.");
    return;
  }

  try {
    const r = await fetch("/api/reboot", { method: "POST" });
    const j = await r.json();
    if (!j.ok) throw new Error(j.error || "reboot failed");
    alert("Rebooting...");
  } catch (e) {
    alert("Reboot failed: " + e.message);
  }
}

/* ---------- Settings ---------- */
async function loadSettings() {
  const meta = document.getElementById("settingsMeta");
  meta.textContent = "loading...";

  const r = await fetch("/api/settings");
  const j = await r.json();
  if (!j.ok) {
    meta.textContent = "Failed: " + (j.error || "unknown");
    return;
  }

  SETTINGS = j;
  UNIT_CYCLE = j.unit_cycle || UNIT_CYCLE;

  meta.textContent = `path: ${j.path} | has_params: ${j.has_params} | type_api: ${j.has_param_type}`;

  if (!DEBUG_UI) {
    meta.style.display = "none";
    const gm = document.getElementById("groupMeta");
    if (gm) gm.style.display = "none";
    const cm = document.getElementById("carMeta");
    if (cm) cm.style.display = "none";
  }

  renderGroups();
  CURRENT_GROUP = null;
  showSettingScreen("groups", false);
}

function renderGroups() {
  const box = document.getElementById("groupList");
  box.innerHTML = "";

  (SETTINGS.groups || []).forEach(g => {
    const label = (LANG === "ko") ? g.group : (g.egroup || g.group);
    const b = document.createElement("button");
    b.className = "btn groupBtn";
    b.textContent = `${label} (${g.count})`;
    b.onclick = () => selectGroup(g.group);
    box.appendChild(b);
  });
}

function selectGroup(group) {
  CURRENT_GROUP = group;
  showSettingScreen("items", true);
  renderItems(group);
}

async function renderItems(group) {
  const meta = document.getElementById("groupMeta");
  const itemsBox = document.getElementById("items");
  itemsBox.innerHTML = "";

  const list = SETTINGS.items_by_group[group] || [];
  if (meta) meta.textContent = `${group} / ${list.length}`;
  settingTitle.textContent = "Setting - " + group;

  const names = list.map(p => p.name);
  let values = {};
  try {
    values = await bulkGet(names);
  } catch (e) {
    values = {};
  }

  for (const p of list) {
    const name = p.name;
    if (!(name in UNIT_INDEX)) UNIT_INDEX[name] = 0;

    const title = formatItemText(p, "title", "etitle", "");
    const descr = formatItemText(p, "descr", "edescr", "");

    const el = document.createElement("div");
    el.className = "setting";

    const top = document.createElement("div");
    top.className = "settingTop";

    const left = document.createElement("div");
    left.innerHTML = `
      <div class="title">${escapeHtml(title)}</div>
      <div class="name">${escapeHtml(name)}</div>
      <div class="muted" style="margin-top:6px;">
        min=${p.min}, max=${p.max}, default=${p.default}
      </div>
    `;

    const ctrl = document.createElement("div");
    ctrl.className = "ctrl";

    const btnMinus = document.createElement("button");
    btnMinus.className = "smallBtn";
    btnMinus.textContent = "-";

    const val = document.createElement("div");
    val.className = "pill val";

    const btnPlus = document.createElement("button");
    btnPlus.className = "smallBtn";
    btnPlus.textContent = "+";

    const unitBtn = document.createElement("button");
    unitBtn.className = "smallBtn";
    unitBtn.textContent = "unit: " + UNIT_CYCLE[UNIT_INDEX[name]];

    unitBtn.onclick = () => {
      UNIT_INDEX[name] = (UNIT_INDEX[name] + 1) % UNIT_CYCLE.length;
      unitBtn.textContent = "unit: " + UNIT_CYCLE[UNIT_INDEX[name]];
    };

    ctrl.appendChild(btnMinus);
    ctrl.appendChild(val);
    ctrl.appendChild(btnPlus);
    ctrl.appendChild(unitBtn);

    top.appendChild(left);
    top.appendChild(ctrl);

    const d = document.createElement("div");
    d.className = "descr";
    d.textContent = descr;

    el.appendChild(top);
    el.appendChild(d);
    itemsBox.appendChild(el);

    // initial value
    const cur = (name in values) ? values[name] : p.default;
    val.textContent = String(cur);

    async function applyDelta(sign) {
      const step = UNIT_CYCLE[UNIT_INDEX[name]];
      let curv = Number(val.textContent);
      if (Number.isNaN(curv)) curv = Number(p.default);

      let next = curv + sign * step;
      next = clamp(next, Number(p.min), Number(p.max));

      if (Number.isInteger(p.min) && Number.isInteger(p.max) && Number.isInteger(step)) {
        next = Math.round(next);
      }

      try {
        await setParam(name, next);
        val.textContent = String(next);
      } catch (e) {
        alert("set failed: " + e.message);
      }
    }

    btnMinus.onclick = () => applyDelta(-1);
    btnPlus.onclick = () => applyDelta(+1);
  }
}

/* ---------- Home WS state ---------- */
function wsConnect() {
  const wsProto = (location.protocol === "https:") ? "wss" : "ws";
  const ws = new WebSocket(wsProto + "://" + location.host + "/ws/state");
  const box = document.getElementById("stateBox");
  ws.onopen = () => box.textContent = "connected";
  ws.onmessage = (ev) => {
    try {
      const j = JSON.parse(ev.data);
      box.textContent = JSON.stringify(j, null, 2);
    } catch (e) {
      box.textContent = ev.data;
    }
  };
  ws.onclose = () => {
    box.textContent = "disconnected (reconnecting...)";
    setTimeout(wsConnect, 1000);
  };
}
wsConnect();

/* ---------- Back key / history ---------- */
history.replaceState({ page: "home" }, "");

window.addEventListener("popstate", async (ev) => {
  const st = ev.state || { page: "home" };

  if (st.page === "home") {
    CURRENT_GROUP = null;
    CURRENT_MAKER = null;
    showPage("home", false);
    return;
  }

  if (st.page === "setting") {
    showPage("setting", false);
    const screen = st.screen || "groups";
    CURRENT_GROUP = st.group || null;

    if (screen === "items" && CURRENT_GROUP) {
      showSettingScreen("items", false);
      renderItems(CURRENT_GROUP);
    } else {
      showSettingScreen("groups", false);
    }
    return;
  }

  if (st.page === "car") {
    showPage("car", false);
    if (!CARS) await loadCars();

    const screen = st.screen || "makers";
    CURRENT_MAKER = st.maker || null;

    if (screen === "models" && CURRENT_MAKER) {
      renderModels(CURRENT_MAKER);
      showCarScreen("models", false);
    } else {
      showCarScreen("makers", false);
    }
    return;
  }

  if (st.page == "tools") {
    showPage("tools", false);
    return;
  }

  if (st.page === "branch") {
    showPage("branch", false);
    // 브랜치 목록이 없으면 다시 로드
    if (!BRANCHES || !BRANCHES.length) {
      loadBranchesAndShow().catch(() => {});
    }
    return;
  }

});

function toolsOutSet(s) {
  const out = document.getElementById("toolsOut");
  if (out) out.textContent = String(s);
}

function toolsMetaSet(s) {
  const meta = document.getElementById("toolsMeta");
  if (meta) meta.textContent = String(s);
}

async function postJson(url, bodyObj) {
  const r = await fetch(url, {
    method: "POST",
    headers: {"Content-Type": "application/json"},
    body: JSON.stringify(bodyObj || {})
  });
  const j = await r.json().catch(() => ({}));
  if (!r.ok || !j.ok) throw new Error(j.error || ("HTTP " + r.status));
  return j;
}

async function runTool(action, payload) {
  toolsMetaSet("running: " + action);
  toolsOutSet("...");

  // 서버에서 { ok:true, out:"...", rc:0 } 이런 형태로 주면 가장 좋음
  const j = await postJson("/api/tools", { action, ...(payload || {}) });

  toolsMetaSet("done: " + action);
  if (j.out != null) {
    toolsOutSet(j.out);
  } else {
    toolsOutSet(JSON.stringify(j, null, 2));
  }

  return j;
}

function confirmText(msg, placeholder = "") {
  const v = prompt(msg, placeholder);
  if (v === null) return null;
  return String(v).trim();
}


function initToolsPage() {
  // 버튼 바인딩 (한 번만)
  const bindOnce = (id, fn) => {
    const el = document.getElementById(id);
    if (!el || el.dataset.bound === "1") return;
    el.dataset.bound = "1";
    el.onclick = fn;
  };

  toolsMetaSet("ready");

  bindOnce("btnGitPull", async () => {
    try {
      await runTool("git_pull");
    } catch (e) {
      toolsMetaSet("error");
      toolsOutSet("git pull failed: " + e.message);
      alert(e.message);
    }
  });

  bindOnce("btnGitSync", async () => {
    if (!confirm("Run git sync?")) return;
    try {
      await runTool("git_sync");
    } catch (e) {
      toolsMetaSet("error");
      toolsOutSet("git sync failed: " + e.message);
      alert(e.message);
    }
  });

  bindOnce("btnGitReset", async () => {
    if (!confirm("Run git reset? (DANGEROUS)")) return;

    // 옵션 필요하면 prompt로 받기
    // 예: hard / soft, target
    const mode = confirmText("reset mode? (hard/soft/mixed)", "hard");
    if (!mode) return;

    const target = confirmText("reset target? (e.g. HEAD~1 or origin/master)", "HEAD");
    if (!target) return;

    try {
      await runTool("git_reset", { mode, target });
    } catch (e) {
      toolsMetaSet("error");
      toolsOutSet("git reset failed: " + e.message);
      alert(e.message);
    }
  });
  bindOnce("btnGitBranch", async () => {
    await loadBranchesAndShow();
  });


  bindOnce("btnSendTmuxLog", async () => {
    try {
      const j = await runTool("send_tmux_log");

      if (j.file) {
        window.location.href = j.file;
      }
    } catch (e) {
      toolsMetaSet("error");
      toolsOutSet("send tmux log failed: " + e.message);
      alert(e.message);
    }
  });

  bindOnce("btnDeleteVideos", async () => {
    if (!confirm("Delete ALL videos? (DANGEROUS)")) return;
    try {
      await runTool("delete_all_videos");
    } catch (e) {
      toolsMetaSet("error");
      toolsOutSet("delete videos failed: " + e.message);
      alert(e.message);
    }
  });

  bindOnce("btnDeleteLogs", async () => {
    if (!confirm("Delete ALL logs? (DANGEROUS)")) return;
    try {
      await runTool("delete_all_logs");
    } catch (e) {
      toolsMetaSet("error");
      toolsOutSet("delete logs failed: " + e.message);
      alert(e.message);
    }
  });

  bindOnce("btnBackupSettings", async () => {
    try {
      const j = await runTool("backup_settings");
      if (j.file) window.location.href = j.file; //  다운로드
    } catch (e) {
      toolsMetaSet("error");
      toolsOutSet("backup failed: " + e.message);
      alert(e.message);
    }
  });

  bindOnce("btnRestoreSettings", async () => {
    const inp = document.getElementById("restoreFile");
    if (!inp || !inp.files || !inp.files[0]) {
      alert("Select a backup json file first.");
      return;
    }

    if (!confirm("Restore settings from file?\n\nThis will overwrite many Params values.")) return;

    try {
      toolsMetaSet("uploading...");
      toolsOutSet("...");

      const fd = new FormData();
      fd.append("file", inp.files[0]);

      const r = await fetch("/api/params_restore", { method: "POST", body: fd });
      const j = await r.json().catch(() => ({}));
      if (!r.ok || !j.ok) throw new Error(j.error || ("HTTP " + r.status));

      toolsMetaSet("restore done");
      toolsOutSet(JSON.stringify(j.result, null, 2));

      if (confirm("Restore done.\nReboot now?")) {
        await runTool("reboot");
        toolsMetaSet("rebooting...");
        toolsOutSet("reboot requested");
      }
    } catch (e) {
      toolsMetaSet("error");
      toolsOutSet("restore failed: " + e.message);
      alert(e.message);
    }
  });

  bindOnce("btnReboot", async () => {
    if (!confirm("Reboot now?")) return;
    try {
      // 네가 이미 만든 /api/reboot를 쓸 거면 이걸로 바꿔도 됨:
      // await postJson("/api/reboot", {});
      await runTool("reboot");
      toolsMetaSet("rebooting...");
      toolsOutSet("reboot requested");
    } catch (e) {
      toolsMetaSet("error");
      toolsOutSet("reboot failed: " + e.message);
      alert(e.message);
    }
  });

  bindOnce("btnSysCmdRun", async () => {
    const inp = document.getElementById("sysCmdInput");
    const cmd = (inp?.value || "").trim();
    if (!cmd) return;

    toolsOutSet("running: " + cmd + "\n");

    try {
      const j = await runTool("shell_cmd", { cmd });
      // j.out에 stdout/stderr 합친 결과
      toolsOutSet(j.out || "(no output)");
    } catch (e) {
      toolsOutSet("error: " + e.message);
      alert(e.message);
    }
  });
}

async function loadBranchesAndShow() {
  showPage("branch", true);
  if (!branchMeta || !branchList) {
    alert("Branch DOM missing (branchMeta / branchList)");
    return;
  }
  branchMeta.textContent = "loading...";
  branchList.innerHTML = "";
  BRANCHES = [];

  try {
    const j = await runTool("git_branch_list");
    BRANCHES = j.branches || [];
    branchMeta.textContent = `${BRANCHES.length} branches`;

    renderBranchList();
  } catch (e) {
    branchMeta.textContent = "Failed: " + e.message;
  }
}

function renderBranchList() {
  branchList.innerHTML = "";

  for (const br of BRANCHES) {
    const b = document.createElement("button");
    b.className = "btn groupBtn";
    b.textContent = br;
    b.onclick = () => onSelectBranch(br);
    branchList.appendChild(b);
  }
}

async function onSelectBranch(branch) {
  if (!confirm(`Checkout branch?\n\n${branch}\n\nContinue?`)) return;

  try {
    await runTool("git_checkout", { branch });
    alert("Branch changed.");
  } catch (e) {
    alert("Checkout failed: " + e.message);
    return;
  }

  const rb = confirm("Reboot now?");
  if (!rb) return;

  try {
    await runTool("reboot"); // 또는 /api/reboot
    alert("Rebooting...");
  } catch (e) {
    alert("Reboot failed: " + e.message);
  }
}




// ===== WebRTC (auto) =====
let RTC_PC = null;
let RTC_RETRY_T = null;

function rtcStatusSet(s) {
  const el = document.getElementById("rtcStatus");
  if (el) el.textContent = String(s);
}

function rtcCancelRetry() {
  if (RTC_RETRY_T) {
    clearTimeout(RTC_RETRY_T);
    RTC_RETRY_T = null;
  }
}

async function rtcDisconnect() {
  rtcCancelRetry(); // 추가
  try { if (RTC_PC) RTC_PC.close(); } catch {}
  RTC_PC = null;
  const v = document.getElementById("rtcVideo");
  if (v) { v.srcObject = null; v.style.display = "none"; }
  const rtcCard = document.getElementById("rtcCard");
  rtcCard.style.display = "none";

  // HUD auto dock handled by hudAutoDock()
  //await carWsDisconnect();
}

function rtcScheduleRetry(ms = 2000) {
  rtcCancelRetry(); // 항상 새로 잡는다
  RTC_RETRY_T = setTimeout(async () => {
    RTC_RETRY_T = null;
    await rtcConnectOnce().catch(() => {});
  }, ms);
}

async function waitIceComplete(pc, timeoutMs = 8000) {
  if (pc.iceGatheringState === "complete") return;
  await new Promise((resolve) => {
    const t = setTimeout(resolve, timeoutMs);
    function onchg() {
      if (pc.iceGatheringState === "complete") {
        pc.removeEventListener("icegatheringstatechange", onchg);
        clearTimeout(t);
        resolve();
      }
    }
    pc.addEventListener("icegatheringstatechange", onchg);
  });
}

let RTC_WAIT_TRACK_T = null;

function rtcArmTrackTimeout(ms = 5000) {
  if (RTC_WAIT_TRACK_T) clearTimeout(RTC_WAIT_TRACK_T);
  RTC_WAIT_TRACK_T = setTimeout(async () => {
    RTC_WAIT_TRACK_T = null;
    rtcStatusSet("no track, retry...");
    await rtcDisconnect();
    rtcScheduleRetry(1000);
  }, ms);
}

function rtcDisarmTrackTimeout() {
  if (RTC_WAIT_TRACK_T) {
    clearTimeout(RTC_WAIT_TRACK_T);
    RTC_WAIT_TRACK_T = null;
  }
}

async function rtcConnectOnce() {
  if (RTC_PC && (RTC_PC.connectionState === "connected" || RTC_PC.connectionState === "connecting")) return;

  try {
    await rtcDisconnect();
    rtcStatusSet("connecting...");

    const pc = new RTCPeerConnection({
      iceServers: [],
      sdpSemantics: "unified-plan",
      iceCandidatePoolSize: 1
    });
    RTC_PC = pc;

    const v = document.getElementById("rtcVideo");
    if (v) { v.muted = true; v.playsInline = true; }

    const dbg = (...a) => console.log("[RTC]", ...a);

    pc.addTransceiver("video", { direction: "recvonly" });

    pc.ontrack = async (ev) => {
      const rtcCard = document.getElementById("rtcCard");
      const v = document.getElementById("rtcVideo");
      if (!v) return;

      let stream = ev.streams && ev.streams[0];
      if (!stream) {
        stream = new MediaStream([ev.track]);
      }

      v.srcObject = stream;
      v.style.display = "block";
      rtcCard.style.display = "block";
      try { await v.play(); } catch(e) { console.log("[RTC] play() failed", e); }
      rtcStatusSet("track: " + ev.track.kind);
      rtcDisarmTrackTimeout();

      hudAutoDock();
      carWsConnect();
    };

    pc.onconnectionstatechange = () => {
      const st = pc.connectionState;
      dbg("connectionState:", st);
      rtcStatusSet("conn: " + st);
      if (st === "failed" || st === "disconnected" || st === "closed") {
        rtcDisconnect();
        rtcScheduleRetry(2000);
      }
    };

    pc.oniceconnectionstatechange = () => {
      const st = pc.iceConnectionState;
      dbg("iceConnectionState:", st);
      rtcStatusSet("ice: " + st);
      if (st === "failed" || st === "disconnected" || st === "closed") {
        rtcDisconnect();
        rtcScheduleRetry(2000);
      }
    };

    // offer
    const offer = await pc.createOffer();
    await pc.setLocalDescription(offer);

    await waitIceComplete(pc, 8000);

    const url = "/stream";   
    const body = {
      sdp: pc.localDescription.sdp,
      cameras: ["road"],
      bridge_services_in: [],
      bridge_services_out: [],
    };

    const r = await fetch(url, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(body),
    });

    if (!r.ok) {
      const t = await r.text().catch(() => "");
      throw new Error("stream http " + r.status + " " + t);
    }

    const ans = await r.json();
    if (!ans || !ans.sdp) throw new Error("bad answer");

    await pc.setRemoteDescription({ type: ans.type || "answer", sdp: ans.sdp });

    rtcStatusSet("connected (waiting track...)");
    rtcArmTrackTimeout(6000);

  } catch (e) {
    rtcStatusSet("error: " + e.message);
    await rtcDisconnect();        //  실패 시 깨끗이 정리
    rtcScheduleRetry(2000);       //  여기서도 무조건 재시도
    throw e;
  }
}

async function waitServerReady(timeoutMs = 8000) {
  const t0 = Date.now();
  while (Date.now() - t0 < timeoutMs) {
    try {
      // 서버 살아있는지만 확인 (가벼운 API)
      const r = await fetch("/api/settings", { cache: "no-store" });
      if (r.ok) return true;
    } catch {}
    await new Promise(res => setTimeout(res, 300));
  }
  return false;
}

function rtcInitAuto() {
  (async () => {
    rtcStatusSet("waiting server...");
    await waitServerReady(8000);   // 실패해도 계속 진행
    await rtcConnectOnce().catch(() => {});
  })();

  document.addEventListener("visibilitychange", () => {
    if (!document.hidden) rtcConnectOnce().catch(() => {});
  });
}
const btnRtcFs = document.getElementById("btnRtcFs");
const rtcVideoEl = document.getElementById("rtcVideo");
const rtcWrap = document.getElementById("rtcWrap");

// 유저 제스처에서만 호출되도록: 버튼 클릭 / 비디오 탭 이벤트에서만 실행
async function rtcToggleFullscreen() {
  const target = rtcWrap || rtcVideoEl;

  // 이미 풀스크린이면 종료
  const fsEl = document.fullscreenElement || document.webkitFullscreenElement;
  if (fsEl) {
    if (document.exitFullscreen) await document.exitFullscreen().catch(()=>{});
    else if (document.webkitExitFullscreen) document.webkitExitFullscreen();
    return;
  }

  // 1) 표준 Fullscreen API (대부분의 크롬/안드/데스크탑)
  if (target.requestFullscreen) {
    await target.requestFullscreen().catch(()=>{});
    return;
  }

  // 2) Safari (일부는 webkitRequestFullscreen)
  if (target.webkitRequestFullscreen) {
    target.webkitRequestFullscreen();
    return;
  }

  // 3) iOS Safari: video 전용 전체화면 (가장 잘 먹힘)
  //    (주의: iOS는 inline 재생/정책 때문에 이 방법이 더 안정적)
  if (target.webkitEnterFullscreen) {
    target.webkitEnterFullscreen();
    return;
  }

  alert("Fullscreen not supported on this browser.");
}

// 버튼
if (btnRtcFs) btnRtcFs.onclick = rtcToggleFullscreen;

// 비디오 탭(원하면)
if (rtcVideoEl) {
  rtcVideoEl.style.cursor = "pointer";
  rtcVideoEl.addEventListener("click", rtcToggleFullscreen);
}

let CAR_WS = null;
let CAR_WS_RETRY_T = null;

function carWsScheduleReconnect(ms = 1000) {
  if (CAR_WS_RETRY_T) return;
  CAR_WS_RETRY_T = setTimeout(() => {
    CAR_WS_RETRY_T = null;
    carWsConnect();
  }, ms);
}

// ===== Driving HUD docking (card <-> WebRTC overlay) =====
function hudDock(mode /* "card"|"top"|"bl" */) {
  const hudRoot = document.getElementById("hudRoot");
  const card = document.getElementById("driveHudCard");
  const host = document.getElementById("hudOverlayHost");
  if (!hudRoot || !card || !host) return;

  host.classList.remove("dock_top","dock_bl");
  host.style.display = "none";

  if (mode === "top" || mode === "bl") {
    host.classList.add(mode === "bl" ? "dock_bl" : "dock_top");
    host.style.display = "";
    if (hudRoot.parentElement !== host) host.appendChild(hudRoot);
    card.style.display = "none";
  } else {
    if (hudRoot.parentElement !== card) card.appendChild(hudRoot);
    card.style.display = "";
  }
}

function hudAutoDock() {
  const rtcVideo = document.getElementById("rtcVideo");
  const rtcCard = document.getElementById("rtcCard");
  const host = document.getElementById("hudOverlayHost");
  if (!rtcVideo || !rtcCard || !host) return;

  const videoVisible = rtcCard.style.display !== "none" && rtcVideo.style.display !== "none";
  if (!videoVisible) { hudDock("card"); return; }

  const fs = document.fullscreenElement === rtcVideo;
  const landscape = window.innerWidth >= window.innerHeight;

  if (fs && landscape) hudDock("bl");
  else hudDock("top");
}

function drivingHudUpdateFromCarPayload(j) {
  if (!window.DrivingHud) {
    console.log("[HUD] update none");
    return;
  }

  const vEgoKph = (typeof j.vEgo === "number" && isFinite(j.vEgo)) ? j.vEgo * 3.6 : null;

  const payload = {
    cpuTempC: j.cpuTempC,
    memPct: j.memPct,
    diskPct: j.diskPct,
    diskLabel: j.diskLabel,
    vEgoKph,
    vSetKph: j.vSetKph,
    temp: j.temp,
    redDot: j.redDot,
    tlight: j.tlight,
    tfGap: j.tfGap,
    tfBars: j.tfBars,
    gear: j.gear,
    gpsOk: j.gpsOk,
    driveMode: j.driveMode,
    speedLimitKph: j.speedLimitKph,
    speedLimitOver: j.speedLimitOver,
    apm: j.apm,
  };

  window.DrivingHud.update(payload);
}
function carWsConnect() {
  // 이미 살아있으면 패스
  if (CAR_WS && (CAR_WS.readyState === WebSocket.OPEN || CAR_WS.readyState === WebSocket.CONNECTING)) return;

  const wsProto = (location.protocol === "https:") ? "wss" : "ws";
  CAR_WS = new WebSocket(wsProto + "://" + location.host + "/ws/carstate");

  CAR_WS.onopen = () => {
    console.log("[CAR_WS] open");
  };

  CAR_WS.onmessage = (ev) => {
    try {
      const j = JSON.parse(ev.data);
      // console.log("[CAR_WS] msg keys:", Object.keys(j || {}));
      // console.log("[CAR_WS] vEgo:", j?.vEgo, "type:", typeof j?.vEgo);
      drivingHudUpdateFromCarPayload(j);
      hudAutoDock();
    } catch (e) {
      console.log("[CAR_WS] bad msg", e, ev.data);
    }
  };

  CAR_WS.onerror = (e) => {
    console.log("[CAR_WS] error", e);
  };

  CAR_WS.onclose = () => {
    console.log("[CAR_WS] close -> reconnect");
    CAR_WS = null;
    carWsScheduleReconnect(1000);
  };
}

async function carWsDisconnect() {
  if (CAR_WS_RETRY_T) { clearTimeout(CAR_WS_RETRY_T); CAR_WS_RETRY_T = null; }
  try { if (CAR_WS) CAR_WS.close(); } catch {}
  CAR_WS = null;
}

async function updateQuickLink() {
  const el = document.getElementById("quickLink");
  if (!el) return;

  try {
    const v = await bulkGet(["GithubUsername"]);
    const githubId = (v["GithubUsername"] || "").trim();

    if (!githubId) {
      el.style.display = "";
      el.textContent = "GithubUsername empty (bulkGet ok)";
      return;
    }

    const url = `https://shind0.synology.me/carrot/go/?id=${encodeURIComponent(githubId)}`;
    el.href = url;
    el.textContent = url;
    el.style.display = "";
  } catch (e) {
    el.style.display = "";
    el.removeAttribute("href");
    el.textContent = "QuickLink error: " + (e?.message || e);
    console.log("[QuickLink] failed:", e);
  }
}






function startAll() {
  showPage("home", false);
  rtcInitAuto();
  updateQuickLink().catch(() => {});

  if (window.DrivingHud) {
    window.DrivingHud.init();
  }

  // start car telemetry WS (10Hz)
  carWsConnect();

  // keep HUD dock state in sync
  window.addEventListener("resize", hudAutoDock);
  document.addEventListener("fullscreenchange", hudAutoDock);
  setInterval(hudAutoDock, 800);
}



if (document.readyState === "loading") {
  window.addEventListener("DOMContentLoaded", startAll);
} else {
  startAll();
}

