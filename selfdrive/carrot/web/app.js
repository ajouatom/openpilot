const DEBUG_UI = false;

let SETTINGS = null;
let CURRENT_GROUP = null;
let LANG = "ko"; // "ko" | "en" | "zh"

const UI_STRINGS = {
  ko: {
    home: "홈",
    setting: "설정",
    tools: "도구",
    fleet: "플릿",
    lang: "언어",
    server_state: "서버 상태",
    quick_link: "퀵 링크",
    car_select: "차량 선택",
    makers: "제조사",
    models: "모델",
    groups: "그룹",
    items: "항목",
    back: "뒤로",
    change: "변경",
    git_commands: "Git 명령",
    user_system: "사용자 / 시스템",
    reboot: "재부팅",
    backup: "백업",
    restore: "복구",
    apply: "적용",
    confirm_car: "이 차량을 선택하시겠습니까?",
    confirm_reboot: "지금 재부팅하시겠습니까?",
    reboot_later: "선택되었습니다. 적용하려면 나중에 재부팅하세요.",
    rebooting: "재부팅 중...",
    git_sync_confirm: "Git sync를 실행하시겠습니까?",
    git_reset_confirm: "Git reset을 실행하시겠습니까? (위험)",
    delete_videos_confirm: "모든 비디오를 삭제하시겠습니까? (위험)",
    delete_logs_confirm: "모든 로그를 삭제하시겠습니까? (위험)",
    select_backup_file: "먼저 백업 json 파일을 선택하세요.",
    restore_confirm: "파일에서 설정을 복구하시겠습니까?\n\n많은 Params 값이 덮어씌워집니다.",
    restore_done_reboot: "복구가 완료되었습니다.\n지금 재부팅하시겠습니까?",
    checkout_confirm: "브랜치를 체크아웃하시겠습니까?",
    branch_changed: "브랜치가 변경되었습니다.",
    quick_link_hint: "* 길게 눌러 링크저장",
    git_hint: "* reset/branch는 위험할 수 있으니 confirm 뜹니다.",
    sys_hint: "* delete/reboot는 confirm 후 실행합니다.",
    restore_hint: "* restore 후 reboot 권장",
    failed_set_car: "차량 선택 저장 실패: ",
    reboot_failed: "재부팅 실패: ",
    set_failed: "설정 실패: ",
    branch_dom_missing: "브랜치 DOM 요소를 찾을 수 없습니다.",
    fullscreen_not_supported: "이 브라우저는 전체화면을 지원하지 않습니다.",
  },
  en: {
    home: "Home",
    setting: "Setting",
    tools: "Tools",
    fleet: "Fleet",
    lang: "Lang",
    server_state: "Server State",
    quick_link: "Quick Link",
    car_select: "Car Select",
    makers: "Makers",
    models: "Models",
    groups: "Groups",
    items: "Items",
    back: "Back",
    change: "Change",
    git_commands: "Git Commands",
    user_system: "User / System",
    reboot: "Reboot",
    backup: "Backup",
    restore: "Restore",
    apply: "Apply",
    confirm_car: "Select this car?",
    confirm_reboot: "Reboot now?",
    reboot_later: "Selected. Reboot later to apply.",
    rebooting: "Rebooting...",
    git_sync_confirm: "Run git sync?",
    git_reset_confirm: "Run git reset? (DANGEROUS)",
    delete_videos_confirm: "Delete ALL videos? (DANGEROUS)",
    delete_logs_confirm: "Delete ALL logs? (DANGEROUS)",
    select_backup_file: "Select a backup json file first.",
    restore_confirm: "Restore settings from file?\n\nThis will overwrite many Params values.",
    restore_done_reboot: "Restore done.\nReboot now?",
    checkout_confirm: "Checkout branch?",
    branch_changed: "Branch changed.",
    quick_link_hint: "* Long press to save link",
    git_hint: "* Reset/branch will prompt for confirmation.",
    sys_hint: "* Delete/reboot will prompt for confirmation.",
    restore_hint: "* Reboot recommended after restore.",
    failed_set_car: "Failed to set car: ",
    reboot_failed: "Reboot failed: ",
    set_failed: "Set failed: ",
    branch_dom_missing: "Branch DOM elements missing.",
    fullscreen_not_supported: "Fullscreen not supported on this browser.",
  },
  zh: {
    home: "首页",
    setting: "设置",
    tools: "工具",
    fleet: "车队",
    lang: "语言",
    server_state: "服务器状态",
    quick_link: "快速链接",
    car_select: "车辆选择",
    makers: "制造商",
    models: "车型",
    groups: "分组",
    items: "项",
    back: "返回",
    change: "修改",
    git_commands: "Git 命令",
    user_system: "用户 / 系统",
    reboot: "重启",
    backup: "备份",
    restore: "还原",
    apply: "应用",
    confirm_car: "选择此车辆吗？",
    confirm_reboot: "现在重启吗？",
    reboot_later: "已选择。请稍后重启以应用更改。",
    rebooting: "正在重启...",
    git_sync_confirm: "执行 Git 同步吗？",
    git_reset_confirm: "执行 Git 重置吗？（危险）",
    delete_videos_confirm: "删除所有视频吗？（危险）",
    delete_logs_confirm: "删除所有日志吗？（危险）",
    select_backup_file: "请先选择一个备份 JSON 文件。",
    restore_confirm: "从文件还原设置吗？\n\n这将覆盖许多参数值。",
    restore_done_reboot: "还原完成。\n现在重启吗？",
    checkout_confirm: "切换分支吗？",
    branch_changed: "分支已切换。",
    quick_link_hint: "* 长按保存链接",
    git_hint: "* 重置/分支操作会弹出确认提示。",
    sys_hint: "* 删除/重启操作会弹出确认提示。",
    restore_hint: "* 还原后建议重启。",
    failed_set_car: "保存车辆选择失败: ",
    reboot_failed: "重启失败: ",
    set_failed: "设置失败: ",
    branch_dom_missing: "找不到分支 DOM 元素。",
    fullscreen_not_supported: "此浏览器不支持全屏。",
  }
};

const DRIVE_MODES = {
  ko: { normal: "일반", eco: "연비", safe: "안전", sport: "고속" },
  en: { normal: "Normal", eco: "Eco", safe: "Safe", sport: "Sport" },
  zh: { normal: "标准", eco: "经济", safe: "安全", sport: "运动" }
};

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
modelTitle.onclick = () => showCarScreen("makers"); // ��ȭ�鿡�� Ÿ��Ʋ ���� makers��

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
  if (LANG === "ko") LANG = "en";
  else if (LANG === "en") LANG = "zh";
  else LANG = "ko";

  langLabel.textContent = LANG.toUpperCase();

  // Update static UI text
  renderUIText();

  if (SETTINGS) {
    renderGroups();
    if (CURRENT_GROUP) renderItems(CURRENT_GROUP);
  }
}

function renderUIText() {
  const s = UI_STRINGS[LANG];
  if (!s) return;

  setText("btnHome", s.home);
  setText("btnSetting", s.setting);
  setText("btnTools", s.tools);
  setText("btnFleet", s.fleet);
  // langLabel is handled in toggleLang

  // Home
  setText("homeTitle", s.home);
  setText("serverStateTitle", s.server_state);
  setText("quickLinkTitle", s.quick_link);

  // Car Select
  setText("carTitle", s.car_select);
  setText("btnBackCar", s.back);
  setText("makersTitle", s.makers);
  setText("modelTitle", s.models);

  // Setting
  setText("settingTitleText", s.setting); // Use a specific ID if needed
  setText("btnBackGroups", s.back);
  setText("btnChangeCar", s.change);
  setText("groupsTitle", s.groups);
  setText("itemsTitle", s.items);

  // Tools
  setText("toolsTitle", s.tools);
  setText("btnToolsBack", s.back);
  setText("gitCommandsTitle", s.git_commands);
  setText("userSystemTitle", s.user_system);
  setText("btnReboot", s.reboot);
  setText("btnBackupSettings", s.backup);
  setText("btnRestoreSettings", s.restore);

  setText("quickLinkHint", s.quick_link_hint);
  setText("gitHint", s.git_hint);
  setText("sysHint", s.sys_hint);
  setText("restoreHint", s.restore_hint);
}

function setText(id, txt) {
  const el = document.getElementById(id);
  if (el) el.textContent = txt;
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

  // �� ����̴ϱ� ��ư ��/�� ���ϰ�: groupBtn ����
  for (const fullLine of arr) {
    // fullLine ��: "Hyundai Grandeur 2018-19"
    // CarSelected3���� maker�� ���� �־�� �� �� "Grandeur 2018-19"
    const modelOnly = stripMaker(fullLine, maker);

    const b = document.createElement("button");
    b.className = "btn groupBtn";
    b.textContent = modelOnly;
    b.onclick = () => onSelectCar(maker, modelOnly, fullLine);
    modelList.appendChild(b);
  }
}

function stripMaker(fullLine, maker) {
  // maker + ������ 1���� ����
  const prefix = maker + " ";
  if (fullLine.startsWith(prefix)) return fullLine.slice(prefix.length).trim();
  // Ȥ�� "Hyundai"�� �ƴ� �ٸ� ǥ��� fallback: ù �ܾ� ����
  const sp = fullLine.split(" ");
  if (sp.length >= 2) return sp.slice(1).join(" ").trim();
  return fullLine.trim();
}

async function onSelectCar(maker, modelOnly, fullLine) {
  const msg = (UI_STRINGS[LANG].confirm_car || "Select this car?") + `\n\n${maker} ${modelOnly}\n\nThis will set CarSelected3 = "${modelOnly}".`;
  if (!confirm(msg)) return;

  try {
    await setParam("CarSelected3", fullLine);
  } catch (e) {
    alert((UI_STRINGS[LANG].failed_set_car || "Failed to set car: ") + e.message);
    return;
  }

  // Home ǥ�� ������Ʈ
  curCarLabelCar.textContent = modelOnly;
  curCarLabelSetting.textContent = modelOnly;

  const rb = confirm(UI_STRINGS[LANG].confirm_reboot || "Reboot now?");
  if (!rb) {
    alert(UI_STRINGS[LANG].reboot_later || "Selected. Reboot later to apply.");
    return;
  }

  try {
    const r = await fetch("/api/reboot", { method: "POST" });
    const j = await r.json();
    if (!j.ok) throw new Error(j.error || "reboot failed");
    alert(UI_STRINGS[LANG].rebooting || "Rebooting...");
  } catch (e) {
    alert((UI_STRINGS[LANG].reboot_failed || "Reboot failed: ") + e.message);
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
    let label = g.group;
    if (LANG === "zh") label = g.cgroup || g.egroup || g.group;
    else if (LANG === "en") label = g.egroup || g.group;

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
        alert((UI_STRINGS[LANG].set_failed || "set failed: ") + e.message);
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
    // �귣ġ ����� ������ �ٽ� �ε�
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

  // �������� { ok:true, out:"...", rc:0 } �̷� ���·� �ָ� ���� ����
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
  // ��ư ���ε� (�� ����)
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
    if (!confirm(UI_STRINGS[LANG].git_sync_confirm || "Run git sync?")) return;
    try {
      await runTool("git_sync");
    } catch (e) {
      toolsMetaSet("error");
      toolsOutSet("git sync failed: " + e.message);
      alert(e.message);
    }
  });

  bindOnce("btnGitReset", async () => {
    if (!confirm(UI_STRINGS[LANG].git_reset_confirm || "Run git reset? (DANGEROUS)")) return;

    // �ɼ� �ʿ��ϸ� prompt�� �ޱ�
    // ��: hard / soft, target
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
    if (!confirm(UI_STRINGS[LANG].delete_videos_confirm || "Delete ALL videos? (DANGEROUS)")) return;
    try {
      await runTool("delete_all_videos");
    } catch (e) {
      toolsMetaSet("error");
      toolsOutSet("delete videos failed: " + e.message);
      alert(e.message);
    }
  });

  bindOnce("btnDeleteLogs", async () => {
    if (!confirm(UI_STRINGS[LANG].delete_logs_confirm || "Delete ALL logs? (DANGEROUS)")) return;
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
      if (j.file) window.location.href = j.file; //  �ٿ�ε�
    } catch (e) {
      toolsMetaSet("error");
      toolsOutSet("backup failed: " + e.message);
      alert(e.message);
    }
  });

  bindOnce("btnRestoreSettings", async () => {
    const inp = document.getElementById("restoreFile");
    if (!inp || !inp.files || !inp.files[0]) {
      alert(UI_STRINGS[LANG].select_backup_file || "Select a backup json file first.");
      return;
    }

    if (!confirm(UI_STRINGS[LANG].restore_confirm || "Restore settings from file?\n\nThis will overwrite many Params values.")) return;

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

      if (confirm(UI_STRINGS[LANG].restore_done_reboot || "Restore done.\nReboot now?")) {
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
    if (!confirm(UI_STRINGS[LANG].confirm_reboot || "Reboot now?")) return;
    try {
      // �װ� �̹� ���� /api/reboot�� �� �Ÿ� �̰ɷ� �ٲ㵵 ��:
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
      // j.out�� stdout/stderr ��ģ ���
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
    alert(UI_STRINGS[LANG].branch_dom_missing || "Branch DOM missing");
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
  if (!confirm((UI_STRINGS[LANG].checkout_confirm || "Checkout branch?") + `\n\n${branch}\n\nContinue?`)) return;

  try {
    await runTool("git_checkout", { branch });
    alert(UI_STRINGS[LANG].branch_changed || "Branch changed.");
  } catch (e) {
    alert((UI_STRINGS[LANG].set_failed || "Checkout failed: ") + e.message);
    return;
  }

  const rb = confirm(UI_STRINGS[LANG].confirm_reboot || "Reboot now?");
  if (!rb) return;

  try {
    await runTool("reboot"); // 또는 /api/reboot
    alert(UI_STRINGS[LANG].rebooting || "Rebooting...");
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
  rtcCancelRetry(); // �߰�
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
  rtcCancelRetry(); // �׻� ���� ��´�
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
    await rtcDisconnect();        //  ���� �� ������ ����
    rtcScheduleRetry(2000);       //  ���⼭�� ������ ��õ�
    throw e;
  }
}

async function waitServerReady(timeoutMs = 8000) {
  const t0 = Date.now();
  while (Date.now() - t0 < timeoutMs) {
    try {
      // ���� ����ִ����� Ȯ�� (������ API)
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
    await waitServerReady(8000);   // �����ص� ��� ����
    await rtcConnectOnce().catch(() => {});
  })();

  document.addEventListener("visibilitychange", () => {
    if (!document.hidden) rtcConnectOnce().catch(() => {});
  });
}
const btnRtcFs = document.getElementById("btnRtcFs");
const rtcVideoEl = document.getElementById("rtcVideo");
const rtcWrap = document.getElementById("rtcWrap");

// ���� ����ó������ ȣ��ǵ���: ��ư Ŭ�� / ���� �� �̺�Ʈ������ ����
async function rtcToggleFullscreen() {
  const target = rtcWrap || rtcVideoEl;

  // �̹� Ǯ��ũ���̸� ����
  const fsEl = document.fullscreenElement || document.webkitFullscreenElement;
  if (fsEl) {
    if (document.exitFullscreen) await document.exitFullscreen().catch(()=>{});
    else if (document.webkitExitFullscreen) document.webkitExitFullscreen();
    return;
  }

  // 1) ǥ�� Fullscreen API (��κ��� ũ��/�ȵ�/����ũž)
  if (target.requestFullscreen) {
    await target.requestFullscreen().catch(()=>{});
    return;
  }

  // 2) Safari (�Ϻδ� webkitRequestFullscreen)
  if (target.webkitRequestFullscreen) {
    target.webkitRequestFullscreen();
    return;
  }

  // 3) iOS Safari: video ���� ��üȭ�� (���� �� ����)
  //    (����: iOS�� inline ���/��å ������ �� ����� �� ������)
  if (target.webkitEnterFullscreen) {
    target.webkitEnterFullscreen();
    return;
  }

  alert(UI_STRINGS[LANG].fullscreen_not_supported || "Fullscreen not supported on this browser.");
}

// ��ư
if (btnRtcFs) btnRtcFs.onclick = rtcToggleFullscreen;

// ���� ��(���ϸ�)
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
  // �̹� ��������� �н�
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
  renderUIText();
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

