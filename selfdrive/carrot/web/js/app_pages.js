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

async function loadRecordState() {
  try {
    const values = await bulkGet(["ScreenRecord"]);
    const v = values["ScreenRecord"];

    const isOn =
      v === true ||
      v === 1 ||
      v === "1" ||
      v === "true" ||
      v === "True";

    btnRecordToggle.classList.toggle("active", isOn);
    btnRecordToggle.textContent = isOn
      ? (UI_STRINGS[LANG].record_on || UI_STRINGS[LANG].record || "Recording")
      : (UI_STRINGS[LANG].record_off || UI_STRINGS[LANG].record || "Idle");
  } catch (e) {
    btnRecordToggle.classList.remove("active");
    btnRecordToggle.textContent = UI_STRINGS[LANG].record_off || UI_STRINGS[LANG].record || "Record";
  }
}
async function toggleRecord() {
  try {
    const values = await bulkGet(["ScreenRecord"]);
    const v = values["ScreenRecord"];

    const isOn =
      v === true ||
      v === 1 ||
      v === "1" ||
      v === "true" ||
      v === "True";

    await setParam("ScreenRecord", !isOn);
    await loadRecordState();
  } catch (e) {
    alert((UI_STRINGS[LANG].record || "Failed to toggle record: ") + e.message);
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
  CARS = j;

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

  for (const fullLine of arr) {
    const modelOnly = stripMaker(fullLine, maker);

    const b = document.createElement("button");
    b.className = "btn groupBtn";
    b.textContent = modelOnly;
    b.onclick = () => onSelectCar(maker, modelOnly, fullLine);
    modelList.appendChild(b);
  }
}

function stripMaker(fullLine, maker) {
  const prefix = maker + " ";
  if (fullLine.startsWith(prefix)) return fullLine.slice(prefix.length).trim();
  const sp = fullLine.split(" ");
  if (sp.length >= 2) return sp.slice(1).join(" ").trim();
  return fullLine.trim();
}

async function onSelectCar(maker, modelOnly, fullLine) {
  const msg = (UI_STRINGS[LANG].confirm_car || "Select this car?") + `\n\n${maker} ${modelOnly}`;
  if (!confirm(msg)) return;

  try {
    await setParam("CarSelected3", fullLine);
  } catch (e) {
    alert((UI_STRINGS[LANG].failed_set_car || "Failed to set car: ") + e.message);
    return;
  }

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
  renderSettingSubnav();
  CURRENT_GROUP = null;
  showSettingScreen("groups", false);
}

function renderGroups() {
  const box = document.getElementById("groupList");
  box.innerHTML = "";

  (SETTINGS.groups || []).forEach(g => {
    const label = getSettingGroupLabel(g.group);

    const b = document.createElement("button");
    b.className = "btn groupBtn";
    b.textContent = `${label} (${g.count})`;
    b.onclick = () => selectGroup(g.group);
    box.appendChild(b);
  });
}

function getSettingGroupMeta(group) {
  const groups = SETTINGS?.groups || [];
  return groups.find((entry) => entry.group === group) || null;
}

function getSettingGroupLabel(group) {
  const meta = getSettingGroupMeta(group);
  if (!meta) return group;
  if (LANG === "zh") return meta.cgroup || meta.egroup || meta.group;
  if (LANG === "en") return meta.egroup || meta.group;
  return meta.group;
}

let settingSubnavSettleTimer = null;
let settingSubnavProgrammaticScroll = false;
let settingSubnavFocusTimer = null;

function updateSettingSubnavLayoutState() {
  if (!settingSubnav || !settingSubnavWrap) return;

  const maxScrollLeft = Math.max(settingSubnav.scrollWidth - settingSubnav.clientWidth, 0);
  const isScrollable = maxScrollLeft > 4;
  settingSubnavWrap.classList.toggle("is-scrollable", isScrollable);
}

function getCenteredSettingSubnavGroup() {
  if (!settingSubnav) return null;
  const tabs = Array.from(settingSubnav.querySelectorAll(".setting-subnav__tab"));
  if (!tabs.length) return null;

  const viewport = settingSubnav.getBoundingClientRect();
  const centerX = viewport.left + (viewport.width / 2);
  let bestGroup = null;
  let bestDistance = Number.POSITIVE_INFINITY;

  tabs.forEach((tab) => {
    const rect = tab.getBoundingClientRect();
    const tabCenter = rect.left + (rect.width / 2);
    const distance = Math.abs(tabCenter - centerX);
    if (distance < bestDistance) {
      bestDistance = distance;
      bestGroup = tab.dataset.group || null;
    }
  });

  return bestGroup;
}

function centerActiveSettingSubnavTab(behavior = "smooth") {
  if (!settingSubnav) return;
  const activeTab = settingSubnav.querySelector(".setting-subnav__tab.is-active");
  if (activeTab) {
    const maxScrollLeft = Math.max(settingSubnav.scrollWidth - settingSubnav.clientWidth, 0);
    const targetLeft = activeTab.offsetLeft - ((settingSubnav.clientWidth - activeTab.offsetWidth) / 2);
    const nextLeft = Math.max(0, Math.min(targetLeft, maxScrollLeft));
    settingSubnavProgrammaticScroll = true;
    settingSubnav.scrollTo({ left: nextLeft, behavior });
    window.setTimeout(() => {
      settingSubnavProgrammaticScroll = false;
      updateSettingSubnavLayoutState();
    }, behavior === "smooth" ? 260 : 80);
  }
  updateSettingSubnavLayoutState();
}

function scheduleSettingSubnavFocus() {
  if (settingSubnavFocusTimer) clearTimeout(settingSubnavFocusTimer);

  requestAnimationFrame(() => centerActiveSettingSubnavTab("auto"));
  settingSubnavFocusTimer = window.setTimeout(() => {
    centerActiveSettingSubnavTab("auto");
    settingSubnavFocusTimer = window.setTimeout(() => {
      centerActiveSettingSubnavTab("auto");
      settingSubnavFocusTimer = null;
    }, 180);
  }, 60);
}

function renderSettingSubnav() {
  if (!settingSubnav) return;
  settingSubnav.innerHTML = "";

  const groups = SETTINGS?.groups || [];
  groups.forEach((entry) => {
    const button = document.createElement("button");
    button.className = "setting-subnav__tab";
    if (entry.group === CURRENT_GROUP) button.classList.add("is-active");
    button.dataset.group = entry.group;
    button.textContent = getSettingGroupLabel(entry.group);
    button.type = "button";
    button.onclick = () => selectGroup(entry.group, screenItems?.style.display === "none");
    settingSubnav.appendChild(button);
  });

  scheduleSettingSubnavFocus();
}

if (settingSubnav) {
  settingSubnav.addEventListener("scroll", () => {
    updateSettingSubnavLayoutState();
    if (settingSubnavProgrammaticScroll) return;

    if (settingSubnavSettleTimer) clearTimeout(settingSubnavSettleTimer);
    settingSubnavSettleTimer = window.setTimeout(() => {
      settingSubnavSettleTimer = null;
      const centeredGroup = getCenteredSettingSubnavGroup();
      if (!centeredGroup) return;
      if (centeredGroup !== CURRENT_GROUP) {
        selectGroup(centeredGroup, false);
        return;
      }
      centerActiveSettingSubnavTab("smooth");
    }, 120);
  }, { passive: true });
  window.addEventListener("resize", () => requestAnimationFrame(updateSettingSubnavLayoutState));
}

function selectGroup(group, pushHistory = true) {
  CURRENT_GROUP = group;
  showSettingScreen("items", pushHistory);
  if (!pushHistory) {
    history.replaceState({ page: "setting", screen: "items", group: CURRENT_GROUP || null }, "");
  }
  renderItems(group);
}

async function renderItems(group) {
  const meta = document.getElementById("groupMeta");
  const itemsBox = document.getElementById("items");
  itemsBox.innerHTML = "";
  renderSettingSubnav();

  const list = SETTINGS.items_by_group[group] || [];
  if (meta) meta.textContent = `${group} / ${list.length}`;
  const groupLabel = getSettingGroupLabel(group);
  settingTitle.textContent = (UI_STRINGS[LANG].setting || "Setting") + " - " + groupLabel;
  if (itemsTitle) itemsTitle.textContent = groupLabel;

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
      <div class="muted mt-sm">
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
    unitBtn.textContent = "x" + UNIT_CYCLE[UNIT_INDEX[name]];

    unitBtn.onclick = () => {
      UNIT_INDEX[name] = (UNIT_INDEX[name] + 1) % UNIT_CYCLE.length;
      unitBtn.textContent = "x" + UNIT_CYCLE[UNIT_INDEX[name]];
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
  if (!r.ok || !j.ok) {
    const msg = friendlyError(j) || j.error || ("HTTP " + r.status);
    throw new Error(msg);
  }
  return j;
}

async function runTool(action, payload) {
  const labels = getActionLabel(action);

  toolsMetaSet(labels.running);
  toolsOutSet("...");

  const j = await postJson("/api/tools", { action, ...(payload || {}) });

  if (j.out != null) {
    toolsOutSet(j.out);
  } else {
    toolsOutSet(JSON.stringify(j, null, 2));
  }

  if (!j.ok) {
    toolsMetaSet(labels.failed);
    throw new Error(friendlyError(j) || j.out || labels.failed);
  }

  toolsMetaSet(labels.done);
  return j;
}

function confirmText(msg, placeholder = "") {
  const v = prompt(msg, placeholder);
  if (v === null) return null;
  return String(v).trim();
}

function showError(action, error) {
  const labels = getActionLabel(action);
  const title = labels.failed;
  const msg = (typeof error === "object" && error.message) ? error.message : String(error);
  alert(`${title}\n\n${msg}`);
}


function initToolsPage() {
  const bindOnce = (id, fn) => {
    const el = document.getElementById(id);
    if (!el || el.dataset.bound === "1") return;
    el.dataset.bound = "1";
    el.onclick = fn;
  };

  toolsMetaSet(UI_STRINGS[LANG].ready || "Ready");

  bindOnce("btnGitPull", async () => {
    try {
      await runTool("git_pull");
    } catch (e) {
      showError("git_pull", e);
    }
  });

  bindOnce("btnGitSync", async () => {
    if (!confirm(UI_STRINGS[LANG].git_sync_confirm || "Run git sync?")) return;
    try {
      await runTool("git_sync");
    } catch (e) {
      showError("git_sync", e);
    }
  });

  bindOnce("btnGitReset", async () => {
    if (!confirm(UI_STRINGS[LANG].git_reset_confirm || "Run git reset?")) return;

    const mode = confirmText(UI_STRINGS[LANG].git_reset_mode_prompt || "reset mode? (hard/soft/mixed)", "hard");
    if (!mode) return;

    const target = confirmText(UI_STRINGS[LANG].git_reset_target_prompt || "reset target?", "HEAD");
    if (!target) return;

    try {
      await runTool("git_reset", { mode, target });
    } catch (e) {
      showError("git_reset", e);
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

      let msg = j.out || j.error || "";
      if (j.results && Array.isArray(j.results)) {
        const lines = j.results.map(r => `${r.package}: ${r.status}`);
        msg += "\n" + lines.join("\n");
      }
      toolsOutSet(msg);

      if (j.need_reboot) {
        const yes = confirm(UI_STRINGS[LANG].confirm_reboot_after_install);
        if (yes) {
          const r = await runTool("reboot");
          toolsOutSet((msg + "\n\n" + (r.out || "")).trim());
        }
      }
    } catch (e) {
      showError("install_required", e);
    }
  });

  bindOnce("btnDeleteVideos", async () => {
    if (!confirm(UI_STRINGS[LANG].delete_videos_confirm || "Delete ALL videos?")) return;
    try {
      await runTool("delete_all_videos");
    } catch (e) {
      showError("delete_all_videos", e);
    }
  });

  bindOnce("btnDeleteLogs", async () => {
    if (!confirm(UI_STRINGS[LANG].delete_logs_confirm || "Delete ALL logs?")) return;
    try {
      await runTool("delete_all_logs");
    } catch (e) {
      showError("delete_all_logs", e);
    }
  });

  bindOnce("btnRebuildAll", async () => {
    if (!confirm(UI_STRINGS[LANG].rebuild_confirm || "Rebuild all?")) return;
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
    inp.accept = "application/json";
    inp.style.display = "none";

    inp.onchange = async () => {
      if (!inp.files || !inp.files[0]) return;

      if (!confirm(UI_STRINGS[LANG].restore_confirm || "Restore settings from file?")) {
        return;
      }

      try {
        const labels = getActionLabel("backup_settings");
        toolsMetaSet(labels.running);
        toolsOutSet("...");

        const fd = new FormData();
        fd.append("file", inp.files[0]);

        const r = await fetch("/api/params_restore", { method: "POST", body: fd });
        const j = await r.json().catch(() => ({}));
        if (!r.ok || !j.ok) throw new Error(friendlyError(j) || j.error || ("HTTP " + r.status));

        toolsMetaSet(labels.done);
        toolsOutSet(JSON.stringify(j.result, null, 2));

        if (confirm(UI_STRINGS[LANG].restore_done_reboot || "Restore done.\nReboot now?")) {
          const rebootRes = await runTool("reboot");
          toolsOutSet(rebootRes.out || "");
        }
      } catch (e) {
        showError("backup_settings", e);
      } finally {
        inp.remove();
      }
    };

    document.body.appendChild(inp);
    inp.click();
  });

  bindOnce("btnReboot", async () => {
    if (!confirm(UI_STRINGS[LANG].confirm_reboot || "Reboot now?")) return;
    try {
      await runTool("reboot");
    } catch (e) {
      showError("reboot", e);
    }
  });

  bindOnce("btnSysCmdRun", async () => {
    const inp = document.getElementById("sysCmdInput");
    const cmd = (inp?.value || "").trim();
    if (!cmd) return;

    try {
      const j = await runTool("shell_cmd", { cmd });
      toolsOutSet(j.out || "(no output)");
    } catch (e) {
      showError("shell_cmd", e);
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
    branchMeta.textContent = e.message;
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
  if (!confirm((UI_STRINGS[LANG].checkout_confirm || "Switch to this branch?") + `\n\n${branch}`)) return;

  try {
    await runTool("git_checkout", { branch });
    alert(UI_STRINGS[LANG].branch_changed || "Branch changed.");
  } catch (e) {
    showError("git_checkout", e);
    return;
  }

  const rb = confirm(UI_STRINGS[LANG].confirm_reboot || "Reboot now?");
  if (!rb) return;

  try {
    await runTool("reboot");
    alert(UI_STRINGS[LANG].rebooting || "Rebooting...");
  } catch (e) {
    showError("reboot", e);
  }
}
