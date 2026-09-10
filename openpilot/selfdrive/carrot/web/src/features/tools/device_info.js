const EMPTY_VALUE = "-";

function uiText(key, fallback) {
  return typeof globalThis.getUIText === "function" ? globalThis.getUIText(key, fallback) : fallback;
}

function escapeHtml(value) {
  return String(value ?? "")
    .replaceAll("&", "&amp;")
    .replaceAll("<", "&lt;")
    .replaceAll(">", "&gt;")
    .replaceAll('"', "&quot;")
    .replaceAll("'", "&#039;");
}

function text(value) {
  return String(value ?? "").trim();
}

function formatDate(value, withTime = false) {
  const raw = text(value).replace(/['"]/g, "");
  if (!raw) return "";
  let timestamp = Number.NaN;
  if (/^\d+$/.test(raw)) {
    const number = Number(raw);
    timestamp = Number.isFinite(number) ? (raw.length >= 13 ? number : number * 1000) : Number.NaN;
  } else {
    const firstPart = raw.split(" ")[0];
    if (/^\d{10,}$/.test(firstPart)) timestamp = Number(firstPart) * 1000;
    else timestamp = Date.parse(raw);
  }
  if (!Number.isFinite(timestamp)) {
    const match = raw.match(/^(\d{4})[-./](\d{2})[-./](\d{2})/);
    return match ? `${match[1]}.${match[2]}.${match[3]}` : "";
  }
  const date = new Date(timestamp);
  const datePart = `${date.getFullYear()}.${String(date.getMonth() + 1).padStart(2, "0")}.${String(date.getDate()).padStart(2, "0")}`;
  if (!withTime) return datePart;
  return `${datePart} ${String(date.getHours()).padStart(2, "0")}:${String(date.getMinutes()).padStart(2, "0")}`;
}

function friendlyDeviceType(value) {
  const raw = text(value).toLowerCase();
  const friendly = { tici: "c3", tizi: "c3x", mici: "c4" }[raw] || raw || "unknown";
  return friendly === raw ? raw || "unknown" : `${friendly} / ${raw}`;
}

function maskImei(value) {
  const imei = text(value);
  if (imei.length <= 7) return imei;
  return `${imei.slice(0, 4)}••••••${imei.slice(-4)}`;
}

function normalizeInfo(payload = {}) {
  const identity = payload.identity && typeof payload.identity === "object" ? payload.identity : {};
  const software = payload.software && typeof payload.software === "object" ? payload.software : {};
  const connectivity = payload.connectivity && typeof payload.connectivity === "object" ? payload.connectivity : {};
  return Object.freeze({
    identity: Object.freeze({
      deviceType: text(identity.device_type),
      imei: text(identity.imei),
      imeiAvailable: identity.imei_available === true && Boolean(text(identity.imei)),
      dongleId: text(identity.dongle_id),
      hardwareSerial: text(identity.hardware_serial),
      position: text(identity.position),
    }),
    software: Object.freeze({
      branch: text(software.branch),
      commit: text(software.commit),
      commitDate: text(software.commit_date),
      lastUpdate: text(software.last_update),
    }),
    connectivity: Object.freeze({
      carrier: text(connectivity.carrier),
      technology: text(connectivity.technology),
      state: text(connectivity.state),
      simState: text(connectivity.sim_state),
      modemVersion: text(connectivity.modem_version),
    }),
    runtime: Object.freeze({
      bootTime: text(payload.runtime?.boot_time),
    }),
  });
}

function row(label, value, options = {}) {
  const display = text(value) || options.empty || EMPTY_VALUE;
  const valueClass = options.code ? " tools-device-info__value--code" : "";
  const copy = options.copy
    ? `<button type="button" class="tools-device-info__copy" data-tools-device-info-copy="imei">${escapeHtml(options.copy)}</button>`
    : "";
  return `<div class="tools-device-info__row"><div class="tools-device-info__rowCopy"><span class="tools-device-info__label">${escapeHtml(label)}</span><span class="tools-device-info__value${valueClass}">${escapeHtml(display)}</span></div>${copy}</div>`;
}

function section(title, rows) {
  if (!rows.length) return "";
  return `<section class="tools-device-info__section"><h3 class="tools-device-info__sectionTitle">${escapeHtml(title)}</h3><div class="tools-device-info__list">${rows.join("")}</div></section>`;
}

export function buildToolsDeviceInfoDialog(payload = {}) {
  const info = normalizeInfo(payload);
  const device = friendlyDeviceType(info.identity.deviceType);
  const imei = info.identity.imei;
  const commitDate = formatDate(info.software.commitDate);
  const commit = info.software.commit ? `${info.software.commit.slice(0, 7)}${commitDate ? ` · ${commitDate}` : ""}` : "";
  const bootTime = formatDate(info.runtime.bootTime, true);
  const connection = [info.connectivity.technology, info.connectivity.carrier, info.connectivity.state]
    .filter(Boolean)
    .join(" · ");
  const identityRows = [
    row(uiText("imei", "IMEI"), imei, {
      code: true,
      empty: uiText("imei_unavailable", "Unavailable"),
      copy: info.identity.imeiAvailable ? uiText("copy_imei", "Copy IMEI") : "",
    }),
    row(uiText("dongle_id", "Dongle ID"), info.identity.dongleId, { code: true }),
    row(uiText("serial", "Serial"), info.identity.hardwareSerial, { code: true }),
    row(uiText("position", "Position"), info.identity.position),
  ];
  const softwareRows = [
    row(uiText("branch", "Branch"), info.software.branch),
    row(uiText("commit", "Commit"), commit, { code: true }),
    row(uiText("recent_update", "Recent update"), formatDate(info.software.lastUpdate, true)),
  ];
  const connectivityRows = [
    connection ? row(uiText("connection", "Connection"), connection) : "",
    info.connectivity.simState ? row(uiText("sim_status", "SIM status"), info.connectivity.simState) : "",
    info.connectivity.modemVersion ? row(uiText("modem_version", "Modem firmware"), info.connectivity.modemVersion, { code: true }) : "",
  ].filter(Boolean);
  const supportLines = [
    `${uiText("device_type", "Device")}: ${device}`,
    `IMEI: ${imei ? maskImei(imei) : uiText("imei_unavailable", "Unavailable")}`,
    info.identity.dongleId && `Dongle ID: ${info.identity.dongleId}`,
    info.identity.hardwareSerial && `Serial: ${info.identity.hardwareSerial}`,
    info.identity.position && `${uiText("position", "Position")}: ${info.identity.position}`,
    bootTime && `${uiText("boot_time", "Boot time")}: ${bootTime}`,
    info.software.branch && `${uiText("branch", "Branch")}: ${info.software.branch}`,
    commit && `${uiText("commit", "Commit")}: ${commit}`,
    connection && `${uiText("connection", "Connection")}: ${connection}`,
  ].filter(Boolean).join("\n");
  const heroDetail = bootTime
    ? `${uiText("boot_time", "Booted")} ${bootTime}`
    : connection || uiText("device_info_summary", "Device identity and software status");
  return Object.freeze({
    title: `${uiText("carrot_info", "Carrot Info")} · ${device}`,
    html: `<article class="tools-device-info"><header class="tools-device-info__hero"><span class="tools-device-info__mark" aria-hidden="true">${escapeHtml(device.split(" ")[0].toUpperCase())}</span><div><p class="tools-device-info__device">${escapeHtml(device)}</p><p class="tools-device-info__summary">${escapeHtml(heroDetail)}</p></div></header>${section(uiText("device_identity", "Device identity"), identityRows)}${section(uiText("software", "Software"), softwareRows)}${section(uiText("network", "Network"), connectivityRows)}</article>`,
    supportCopyText: supportLines,
    imeiCopyText: imei,
  });
}

export function legacyToolsDeviceInfo(values = {}) {
  return {
    identity: {
      device_type: values.DeviceType,
      imei: "",
      imei_available: false,
      dongle_id: values.DongleId,
      hardware_serial: values.HardwareSerial,
      position: values.DevicePosition,
    },
    software: {
      branch: values.GitBranch,
      commit: values.GitCommit,
      commit_date: values.GitCommitDate,
      last_update: values.GitPullTime,
    },
    connectivity: {},
    runtime: {
      boot_time: "",
    },
  };
}

export function bindToolsDeviceInfoCopy(documentRoot, imei) {
  const copyButton = documentRoot?.querySelector?.("[data-tools-device-info-copy=imei]");
  if (!copyButton || !imei) return;
  copyButton.addEventListener("click", () => {
    globalThis.copyToClipboard?.(imei);
    globalThis.showAppToast?.(uiText("copied", "Copied"), { tone: "success" });
  }, { once: true });
}
