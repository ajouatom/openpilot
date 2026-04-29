"use strict";

// Branch picker modal + standalone Branch page.
// State: ORIGIN_USERNAME, BRANCH_REMOTE_NAMES, BRANCH_REMOTE_OWNERS, BRANCH_GROUP_OPEN.

let ORIGIN_USERNAME = "origin";
let BRANCH_REMOTE_NAMES = ["origin"];
let BRANCH_REMOTE_OWNERS = Object.create(null);
let BRANCH_GROUP_OPEN = Object.create(null);


function openBranchPicker() {
  if (!appBranchPicker) return false;
  if (branchPickerCloseTimer) {
    clearTimeout(branchPickerCloseTimer);
    branchPickerCloseTimer = null;
  }
  appBranchPicker.hidden = false;
  syncModalBodyLock();
  requestAnimationFrame(() => {
    appBranchPicker.classList.add("is-open");
  });
  return true;
}

function closeBranchPicker(immediate = false) {
  if (!appBranchPicker || appBranchPicker.hidden) return;
  appBranchPicker.classList.remove("is-open");

  if (branchPickerCloseTimer) {
    clearTimeout(branchPickerCloseTimer);
    branchPickerCloseTimer = null;
  }

  const finishClose = () => {
    appBranchPicker.hidden = true;
    syncModalBodyLock();
  };

  if (immediate) {
    finishClose();
    return;
  }

  branchPickerCloseTimer = window.setTimeout(() => {
    branchPickerCloseTimer = null;
    finishClose();
  }, 180);
}

if (appBranchPickerBackdrop) appBranchPickerBackdrop.onclick = () => closeBranchPicker();
if (appBranchPickerClose) appBranchPickerClose.onclick = () => closeBranchPicker();

function parseGitHubOwner(url) {
  const text = String(url || "").trim();
  if (!text) return "";
  const httpsMatch = text.match(/github\.com\/([^\/:\s]+)\//);
  if (httpsMatch) return httpsMatch[1];
  const sshMatch = text.match(/github\.com[:\/]([^\/:\s]+)\//);
  return sshMatch ? sshMatch[1] : "";
}

function resetBranchRemoteContext() {
  ORIGIN_USERNAME = "origin";
  BRANCH_REMOTE_NAMES = ["origin"];
  BRANCH_REMOTE_OWNERS = Object.create(null);
}

function syncBranchRemoteContext(result = {}) {
  const remotes = Array.isArray(result.remotes) ? result.remotes.map((r) => String(r || "").trim()).filter(Boolean) : [];
  BRANCH_REMOTE_NAMES = remotes.length ? remotes : ["origin"];
  BRANCH_REMOTE_OWNERS = Object.create(null);

  const remoteUrls = result.remote_urls && typeof result.remote_urls === "object" ? result.remote_urls : {};
  for (const [name, url] of Object.entries(remoteUrls)) {
    const owner = parseGitHubOwner(url);
    if (owner) BRANCH_REMOTE_OWNERS[name] = owner;
  }

  if (BRANCH_REMOTE_OWNERS.origin) {
    ORIGIN_USERNAME = BRANCH_REMOTE_OWNERS.origin;
  }
}

function normalizeBranchItems(result = {}) {
  const items = Array.isArray(result.branch_items) ? result.branch_items : [];
  if (items.length) {
    return items
      .map((item) => {
        const kind = item && item.kind === "remote" ? "remote" : "local";
        const remote = String(item?.remote || "").trim();
        const name = String(item?.name || item?.label || item?.ref || "").trim();
        const ref = String(item?.ref || (kind === "remote" && remote && name ? `${remote}/${name}` : name)).trim();
        if (!ref || !name) return null;
        return {
          id: String(item?.id || `${kind}:${remote}:${name}`),
          kind,
          ref,
          remote,
          name,
          label: String(item?.label || name).trim() || name,
        };
      })
      .filter(Boolean);
  }

  const refs = Array.isArray(result.branches) ? result.branches : [];
  return refs
    .map((ref) => ({ kind: "legacy", ref: String(ref || "").trim() }))
    .filter((item) => item.ref);
}

function getBranchDisplayInfo(entry) {
  if (entry && typeof entry === "object" && entry.kind !== "legacy") {
    const kind = entry.kind === "remote" ? "remote" : "local";
    const ref = String(entry.ref || "").trim();
    const name = String(entry.name || entry.label || ref).trim();

    if (kind === "local") {
      return {
        ref,
        label: name,
        groupKey: "local",
        groupTitle: "local",
        groupRank: 0,
        kind: "local",
        name,
        checkoutPayload: { branch: ref, kind: "local", name },
      };
    }

    const remoteName = String(entry.remote || "").trim();
    const owner = BRANCH_REMOTE_OWNERS[remoteName] || (remoteName === "origin" ? ORIGIN_USERNAME : remoteName);
    return {
      ref,
      label: String(entry.label || name).trim() || name,
      groupKey: `remote:${remoteName}:${owner}`,
      groupTitle: remoteName === "origin" ? `${remoteName} / ${owner}` : `remote / ${owner}`,
      groupRank: remoteName === "origin" ? 1 : 2,
      kind: "remote",
      remote: remoteName,
      name,
      checkoutPayload: { branch: ref, kind: "remote", remote: remoteName, name },
    };
  }

  const raw = String(entry?.ref || entry || "").trim();
  const parts = raw.split("/").filter(Boolean);
  const first = parts[0] || "";
  const isRemoteRef = parts.length >= 2 && BRANCH_REMOTE_NAMES.includes(first);

  if (!isRemoteRef) {
    return {
      ref: raw,
      label: raw,
      groupKey: "local",
      groupTitle: "local",
      groupRank: 0,
      kind: "local",
      name: raw,
      checkoutPayload: { branch: raw },
    };
  }

  const remoteName = first;
  const rest = parts.slice(1);

  const owner = BRANCH_REMOTE_OWNERS[remoteName] || (remoteName === "origin" ? ORIGIN_USERNAME : remoteName);
  return {
    ref: raw,
    label: rest.join("/") || raw,
    groupKey: `remote:${remoteName}:${owner}`,
    groupTitle: remoteName === "origin" ? `${remoteName} / ${owner}` : `remote / ${owner}`,
    groupRank: remoteName === "origin" ? 1 : 2,
    kind: "remote",
    remote: remoteName,
    name: rest.join("/") || raw,
    checkoutPayload: { branch: raw },
  };
}

function isCurrentBranchItem(info) {
  return Boolean(info && info.kind === "local" && CURRENT_BRANCH_NAME && info.name === CURRENT_BRANCH_NAME);
}

function getBranchGroups() {
  const groups = new Map();
  for (const br of BRANCHES) {
    const info = getBranchDisplayInfo(br);
    if (!info.ref || !info.label) continue;

    let group = groups.get(info.groupKey);
    if (!group) {
      group = {
        key: info.groupKey,
        title: info.groupTitle,
        rank: info.groupRank,
        items: [],
        hasCurrent: false,
      };
      groups.set(info.groupKey, group);
    }

    const current = isCurrentBranchItem(info);
    group.hasCurrent = group.hasCurrent || current;
    group.items.push({ ...info, current });
  }

  return Array.from(groups.values()).sort((a, b) => {
    if (a.rank !== b.rank) return a.rank - b.rank;
    return a.title.localeCompare(b.title);
  });
}

function isBranchGroupOpen(group) {
  if (Object.prototype.hasOwnProperty.call(BRANCH_GROUP_OPEN, group.key)) {
    return Boolean(BRANCH_GROUP_OPEN[group.key]);
  }
  return group.key === "local" || group.rank === 1 || group.hasCurrent;
}

function branchCountLabel(count) {
  return count === 1 ? "1 branch" : `${count} branches`;
}

async function loadBranchesAndShow() {
  if (!appBranchPickerMeta || !appBranchPickerList || !openBranchPicker()) {
    toolsLogNotice(UI_STRINGS[LANG].branch_dom_missing || "Branch DOM missing", { label: "git_branch_list" });
    return;
  }
  appBranchPickerMeta.textContent = getUIText("loading", "Loading...");
  appBranchPickerList.innerHTML = "";
  BRANCHES = [];
  CURRENT_BRANCH_NAME = "";
  resetBranchRemoteContext();

  try {
    const v = await bulkGet(["GitRemote"]);
    if (v && v.GitRemote) {
      const owner = parseGitHubOwner(v.GitRemote);
      if (owner) ORIGIN_USERNAME = owner;
    }
  } catch(e) {}

  try {
    const j = await runTool("git_branch_list");
    syncBranchRemoteContext(j);
    BRANCHES = normalizeBranchItems(j);
    CURRENT_BRANCH_NAME = (j.current_branch || "").trim();
    appBranchPickerMeta.textContent = `${BRANCHES.length} branches`;

    renderBranchList();
  } catch (e) {
    appBranchPickerMeta.textContent = e.message;
  }
}

function renderBranchList() {
  if (!appBranchPickerList) return;
  appBranchPickerList.innerHTML = "";

  if (!BRANCHES.length) {
    const empty = document.createElement("div");
    empty.className = "muted";
    empty.textContent = "-";
    appBranchPickerList.appendChild(empty);
    return;
  }

  for (const group of getBranchGroups()) {
    const open = isBranchGroupOpen(group);
    const section = document.createElement("div");
    section.className = "app-branch-picker__group";
    section.classList.toggle("is-open", open);

    const head = document.createElement("button");
    head.className = "app-branch-picker__groupHead";
    head.type = "button";
    head.setAttribute("aria-expanded", open ? "true" : "false");

    const icon = document.createElement("span");
    icon.className = "app-branch-picker__groupIcon";
    icon.textContent = open ? "▼" : "▶";
    head.appendChild(icon);

    const title = document.createElement("span");
    title.className = "app-branch-picker__groupTitle";
    title.textContent = group.title;
    head.appendChild(title);

    const count = document.createElement("span");
    count.className = "app-branch-picker__groupCount";
    count.textContent = branchCountLabel(group.items.length);
    head.appendChild(count);

    head.onclick = () => {
      BRANCH_GROUP_OPEN[group.key] = !open;
      renderBranchList();
    };
    section.appendChild(head);

    if (open) {
      const items = document.createElement("div");
      items.className = "app-branch-picker__groupItems";

      for (const item of group.items) {
        const b = document.createElement("button");
        b.className = "btn groupBtn app-branch-picker__item";
        if (item.current) {
          b.classList.add("is-current");
        }
        b.title = item.ref;

        const label = document.createElement("span");
        label.className = "app-branch-picker__label";
        label.textContent = item.label;
        b.appendChild(label);

        if (item.current) {
          const badge = document.createElement("span");
          badge.className = "app-branch-picker__badge";
          badge.textContent = getUIText("branch_current", "Current");
          b.appendChild(badge);
        }

        b.onclick = () => onSelectBranch(item);
        items.appendChild(b);
      }

      section.appendChild(items);
    }

    appBranchPickerList.appendChild(section);
  }
}

async function onSelectBranch(item) {
  const branch = String(item?.ref || item || "").trim();
  closeBranchPicker(true);
  if (!await appConfirm((UI_STRINGS[LANG].checkout_confirm || "Switch to this branch?") + `\n\n${branch}`, {
    title: "git checkout",
  })) return;

  try {
    await runTool("git_checkout", item?.checkoutPayload || { branch });
    toolsLogNotice(UI_STRINGS[LANG].branch_changed || "Branch changed.", { label: "git_checkout" });
  } catch (e) {
    showError("git_checkout", e);
    return;
  }

  const rb = await appConfirm(UI_STRINGS[LANG].confirm_reboot || "Reboot now?", {
    title: UI_STRINGS[LANG].reboot || "Reboot",
  });
  if (!rb) return;

  try {
    await runTool("reboot");
    toolsLogNotice(UI_STRINGS[LANG].rebooting || "Rebooting...", { label: "reboot" });
  } catch (e) {
    showError("reboot", e);
  }
}

/* ---------- Logs / Dashcam ---------- */
const dashcamState = {
  initialized: false,
  loading: false,
  routes: [],
  expanded: new Set(),
  selected: new Set(),
  refreshTimer: null,
  scrollBusy: false,
  scrollTimer: null,
  loadSeq: 0,
  layoutBound: false,
  layoutTimer: null,
  landscape: null,
  signature: "",
};

const screenrecordState = {
  initialized: false,
  loading: false,
  videos: [],
  loadSeq: 0,
  signature: "",
};

let logsActiveTab = "dashcam";
const logsScrollTops = { dashcam: 0, screen: 0 };
let logsLazyImageObserver = null;

