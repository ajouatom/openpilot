"use strict";

let toolsQrCameraStream = null;
let toolsQrScanTimer = null;
let toolsQrScanFinishTimer = null;
let toolsQrAlignedFrames = 0;

function toolsQrText(key, fallback, vars = null) {
  return typeof getUIText === "function" ? getUIText(key, fallback, vars) : fallback;
}

function toolsQrEscape(value) {
  return String(value ?? "")
    .replaceAll("&", "&amp;")
    .replaceAll("<", "&lt;")
    .replaceAll(">", "&gt;")
    .replaceAll('"', "&quot;")
    .replaceAll("'", "&#039;");
}

function toolsQrSetDialogClass(enabled) {
  if (typeof appDialog === "undefined" || !appDialog) return;
  appDialog.classList.toggle("app-dialog--tools-qr", Boolean(enabled));
}

function toolsQrToast(key, fallback, vars = null, options = null) {
  const message = toolsQrText(key, fallback, vars);
  if (typeof showAppToast === "function") showAppToast(message, options || undefined);
  return message;
}

function toolsQrStopCamera() {
  if (toolsQrScanTimer) {
    cancelAnimationFrame(toolsQrScanTimer);
    toolsQrScanTimer = null;
  }
  if (toolsQrScanFinishTimer) {
    clearTimeout(toolsQrScanFinishTimer);
    toolsQrScanFinishTimer = null;
  }
  toolsQrAlignedFrames = 0;
  if (toolsQrCameraStream) {
    toolsQrCameraStream.getTracks().forEach((track) => track.stop());
    toolsQrCameraStream = null;
  }
}

function toolsQrSetCameraState(cameraWrap, state) {
  if (!cameraWrap) return;
  cameraWrap.dataset.scanState = state || "idle";
}

function toolsQrGuideRect(width, height) {
  const side = Math.min(width, height) * 0.74;
  return {
    left: (width - side) / 2,
    top: (height - side) / 2,
    right: (width + side) / 2,
    bottom: (height + side) / 2,
    width: side,
    height: side,
  };
}

function toolsQrCodeBounds(code) {
  const points = [
    code?.location?.topLeftCorner,
    code?.location?.topRightCorner,
    code?.location?.bottomRightCorner,
    code?.location?.bottomLeftCorner,
  ].filter((point) => Number.isFinite(Number(point?.x)) && Number.isFinite(Number(point?.y)));
  if (points.length < 4) return null;
  const xs = points.map((point) => Number(point.x));
  const ys = points.map((point) => Number(point.y));
  const left = Math.min(...xs);
  const right = Math.max(...xs);
  const top = Math.min(...ys);
  const bottom = Math.max(...ys);
  return {
    left,
    top,
    right,
    bottom,
    width: right - left,
    height: bottom - top,
    centerX: (left + right) / 2,
    centerY: (top + bottom) / 2,
  };
}

function toolsQrIsCodeAligned(code, width, height) {
  const guide = toolsQrGuideRect(width, height);
  const bounds = toolsQrCodeBounds(code);
  if (!bounds || bounds.width <= 0 || bounds.height <= 0) return false;
  const slackX = guide.width * 0.18;
  const slackY = guide.height * 0.18;
  const centerInGuide =
    bounds.centerX >= guide.left &&
    bounds.centerX <= guide.right &&
    bounds.centerY >= guide.top &&
    bounds.centerY <= guide.bottom;
  const mostlyInside =
    bounds.left >= guide.left - slackX &&
    bounds.right <= guide.right + slackX &&
    bounds.top >= guide.top - slackY &&
    bounds.bottom <= guide.bottom + slackY;
  const readableSize =
    bounds.width >= guide.width * 0.34 &&
    bounds.height >= guide.height * 0.34 &&
    bounds.width <= guide.width * 1.18 &&
    bounds.height <= guide.height * 1.18;
  return centerInGuide && mostlyInside && readableSize;
}

function toolsQrMake(payload) {
  if (typeof qrcode !== "function") {
    throw new Error("QR library not loaded");
  }
  const qr = qrcode(0, "L");
  const text = String(payload || "");
  qr.addData(payload, text.startsWith("CQR3:") || text.startsWith("CQR4:") ? "Alphanumeric" : "Byte");
  qr.make();
  return qr;
}

function toolsQrMakeSvg(payload) {
  return toolsQrMake(payload).createSvgTag(2, 4);
}

function toolsQrMakePngDataUrl(payload) {
  const qr = toolsQrMake(payload);
  const moduleCount = qr.getModuleCount();
  const cellSize = 4;
  const margin = 4;
  const size = (moduleCount + margin * 2) * cellSize;
  const canvas = document.createElement("canvas");
  canvas.width = size;
  canvas.height = size;
  const ctx = canvas.getContext("2d");
  ctx.fillStyle = "#ffffff";
  ctx.fillRect(0, 0, size, size);
  ctx.fillStyle = "#000000";
  for (let row = 0; row < moduleCount; row += 1) {
    for (let col = 0; col < moduleCount; col += 1) {
      if (qr.isDark(row, col)) {
        ctx.fillRect((col + margin) * cellSize, (row + margin) * cellSize, cellSize, cellSize);
      }
    }
  }
  return canvas.toDataURL("image/png");
}

function toolsQrSafeFilePart(value) {
  const text = String(value || "").trim() || "carrot";
  return text
    .replace(/[\\/:*?"<>|]+/g, "-")
    .replace(/\s+/g, "-")
    .replace(/-+/g, "-")
    .replace(/^-|-$/g, "")
    .slice(0, 64) || "carrot";
}

function toolsQrDateStamp(date = new Date()) {
  const yyyy = String(date.getFullYear());
  const mm = String(date.getMonth() + 1).padStart(2, "0");
  const dd = String(date.getDate()).padStart(2, "0");
  return `${yyyy}${mm}${dd}`;
}

async function toolsQrBackupFileName() {
  try {
    const values = await bulkGet(["GitBranch"]);
    return `${toolsQrSafeFilePart(values.GitBranch)}-${toolsQrDateStamp()}.png`;
  } catch (e) {
    return `carrot-${toolsQrDateStamp()}.png`;
  }
}

async function toolsQrEnsureDependency() {
  const status = await getJson("/api/params_qr_dependency");
  console.info("[carrot][qr-dependency]", status);
  if (status.installed) return status;

  toolsQrToast("qr_configuring", "Configuring QR feature...");
  const result = await postJson("/api/params_qr_dependency/ensure", {});
  console.info("[carrot][qr-dependency]", result);
  if (!result.ok || !result.installed) {
    throw new Error(result.error || toolsQrText("qr_config_failed", "QR feature could not be configured."));
  }
  toolsQrToast("qr_config_done", "QR feature is ready.");
  return result;
}

function toolsQrDownloadDataUrl(dataUrl, filename) {
  const link = document.createElement("a");
  link.href = dataUrl;
  link.download = filename;
  document.body.appendChild(link);
  link.click();
  link.remove();
}

function toolsQrDecodeImageElement(img) {
  const canvas = document.createElement("canvas");
  const scale = Math.min(1, 1600 / Math.max(img.naturalWidth || img.width, img.naturalHeight || img.height, 1));
  canvas.width = Math.max(1, Math.floor((img.naturalWidth || img.width) * scale));
  canvas.height = Math.max(1, Math.floor((img.naturalHeight || img.height) * scale));
  const ctx = canvas.getContext("2d", { willReadFrequently: true });
  ctx.drawImage(img, 0, 0, canvas.width, canvas.height);
  const imageData = ctx.getImageData(0, 0, canvas.width, canvas.height);
  const code = typeof jsQR === "function" ? jsQR(imageData.data, canvas.width, canvas.height) : null;
  return code?.data || "";
}

function toolsQrPreviewImageFile(file, preview, statusNode) {
  if (!file) return;
  const reader = new FileReader();
  reader.onload = () => {
    const img = new Image();
    img.onload = () => {
      const decoded = toolsQrDecodeImageElement(img);
      if (decoded) preview(decoded);
      else statusNode.textContent = toolsQrText("qr_restore_decode_failed", "QR code was not found.");
    };
    img.onerror = () => {
      statusNode.textContent = toolsQrText("qr_restore_decode_failed", "QR code was not found.");
    };
    img.src = String(reader.result || "");
  };
  reader.readAsDataURL(file);
}

function toolsQrSummaryHtml(summary = {}) {
  const item = (key, labelKey, fallback) => `
    <span class="tools-qr-chip">
      <strong>${Number(summary[key] || 0)}</strong>
      <span>${toolsQrEscape(toolsQrText(labelKey, fallback))}</span>
    </span>
  `;
  return `
    <div class="tools-qr-summary">
      ${item("changed", "qr_restore_changed", "changed")}
      ${item("same", "qr_restore_same", "same")}
      ${item("skipped", "qr_restore_skipped", "skipped")}
      ${item("invalid", "qr_restore_invalid", "invalid")}
    </div>
  `;
}

function toolsQrDiffHtml(preview) {
  const entries = Array.isArray(preview?.entries) ? preview.entries : [];
  const changed = entries.filter((entry) => entry.apply || entry.status === "changed");
  const shown = changed.slice(0, 80);
  const hiddenCount = Math.max(0, changed.length - shown.length);
  const currentLabel = toolsQrText("qr_restore_current_value", "Current");
  const backupLabel = toolsQrText("qr_restore_backup_value", "Restore");
  const changedLabel = toolsQrText("qr_restore_changed", "Changed");
  const rows = shown.map((entry) => `
    <div class="tools-qr-diff__row">
      <div class="tools-qr-diff__head">
        <div class="tools-qr-diff__key">${toolsQrEscape(entry.key)}</div>
        <span class="tools-qr-diff__status">${toolsQrEscape(changedLabel)}</span>
      </div>
      <div class="tools-qr-diff__compare">
        <div class="tools-qr-diff__value tools-qr-diff__value--old">
          <span>${toolsQrEscape(currentLabel)}</span>
          <code>${toolsQrEscape(entry.current)}</code>
        </div>
        <div class="tools-qr-diff__arrow" aria-hidden="true">&gt;</div>
        <div class="tools-qr-diff__value tools-qr-diff__value--new">
          <span>${toolsQrEscape(backupLabel)}</span>
          <code>${toolsQrEscape(entry.value)}</code>
        </div>
      </div>
    </div>
  `).join("");

  if (!changed.length) {
    return `
      ${toolsQrSummaryHtml(preview?.summary)}
      <div class="tools-qr-empty">${toolsQrEscape(toolsQrText("qr_restore_no_changes", "No changes to apply."))}</div>
    `;
  }

  return `
    ${toolsQrSummaryHtml(preview?.summary)}
    <div class="tools-qr-diff__list">${rows}</div>
    ${hiddenCount ? `<div class="tools-qr-more">${toolsQrEscape(toolsQrText("qr_restore_more", "{count} more changes hidden", { count: hiddenCount }))}</div>` : ""}
  `;
}

async function toolsQrShowBackup() {
  try {
    await toolsQrEnsureDependency();
    const j = await getJson("/api/params_qr_backup");
    const format = j.format || String(j.payload || "").split(/[.:]/, 1)[0] || "unknown";
    console.info("[carrot][qr-backup]", {
      format,
      version: j.version,
      count: j.count,
      payloadChars: j.payload_chars,
      rawBytes: j.json_bytes,
      compressedBytes: j.compressed_bytes,
      ecc: j.ecc,
    });
    const svg = toolsQrMakeSvg(j.payload);
    const pngDataUrl = toolsQrMakePngDataUrl(j.payload);
    const fileName = await toolsQrBackupFileName();
    const title = toolsQrText("qr_backup_title", "QR Backup");
    const html = `
      <div class="tools-qr-backup">
        <div class="tools-qr-code" aria-label="${toolsQrEscape(title)}">${svg}</div>
      </div>
    `;
    const promise = openAppDialog({
      mode: "confirm",
      title,
      html: true,
      messageHtml: html,
      confirmLabel: toolsQrText("download", "Download"),
      cancelLabel: toolsQrText("cancel", "Cancel"),
    });
    toolsQrSetDialogClass(true);
    promise.finally(() => toolsQrSetDialogClass(false));
    const ok = await promise;
    if (ok) toolsQrDownloadDataUrl(pngDataUrl, fileName);
  } catch (e) {
    showError("qr backup", e);
  }
}

async function toolsQrPreviewPayload(payload, diffNode, confirmButton, statusNode) {
  const trimmed = String(payload || "").trim();
  if (!trimmed) return;
  statusNode.textContent = toolsQrText("qr_restore_previewing", "Checking backup...");
  confirmButton.disabled = true;
  const j = await postJson("/api/params_restore_preview", { payload: trimmed });
  diffNode.innerHTML = toolsQrDiffHtml(j.preview);
  const selected = Number(j.preview?.summary?.selected || 0);
  confirmButton.disabled = selected <= 0;
  statusNode.textContent = selected > 0
    ? toolsQrText("qr_restore_ready", "{count} changes ready", { count: selected })
    : toolsQrText("qr_restore_no_changes", "No changes to apply.");
  return trimmed;
}

function toolsQrBindRestoreDialog(state) {
  const imageBtn = document.getElementById("toolsQrImageBtn");
  const imageInput = document.getElementById("toolsQrImageInput");
  const cameraBtn = document.getElementById("toolsQrCameraBtn");
  const cameraWrap = document.getElementById("toolsQrCameraWrap");
  const video = document.getElementById("toolsQrVideo");
  const canvas = document.getElementById("toolsQrCanvas");
  const statusNode = document.getElementById("toolsQrStatus");
  const diffNode = document.getElementById("toolsQrDiff");
  const confirmButton = typeof appDialogConfirm !== "undefined" ? appDialogConfirm : null;
  if (!imageBtn || !imageInput || !cameraBtn || !cameraWrap || !video || !canvas || !statusNode || !diffNode || !confirmButton) {
    return;
  }

  confirmButton.disabled = true;
  const cameraAvailable = Boolean(window.isSecureContext && navigator.mediaDevices?.getUserMedia);
  if (!cameraAvailable) {
    cameraBtn.disabled = true;
    cameraBtn.classList.add("is-disabled");
    cameraBtn.textContent = toolsQrText("qr_restore_camera_disabled", "Camera unavailable");
    cameraBtn.setAttribute("aria-label", toolsQrText("qr_restore_https_required", "Open this page with HTTPS to use the camera."));
    cameraBtn.title = toolsQrText("qr_restore_https_required", "Open this page with HTTPS to use the camera.");
  }

  const preview = async (payload) => {
    try {
      state.payload = await toolsQrPreviewPayload(payload, diffNode, confirmButton, statusNode);
    } catch (e) {
      state.payload = "";
      confirmButton.disabled = true;
      statusNode.textContent = e?.message || toolsQrText("qr_restore_preview_failed", "Failed to read backup.");
      diffNode.innerHTML = "";
    }
  };

  imageBtn.onclick = () => imageInput.click();
  imageInput.onchange = () => {
    toolsQrPreviewImageFile(imageInput.files?.[0], preview, statusNode);
    imageInput.value = "";
  };

  cameraBtn.onclick = async () => {
    if (!cameraAvailable) return;
    if (toolsQrCameraStream) {
      toolsQrStopCamera();
      cameraWrap.hidden = true;
      toolsQrSetCameraState(cameraWrap, "idle");
      cameraBtn.textContent = toolsQrText("qr_restore_camera", "Camera");
      return;
    }

    try {
      if (!window.isSecureContext) {
        statusNode.textContent = toolsQrText("qr_restore_https_required", "Open this page with HTTPS to use the camera.");
        return;
      }
      if (!navigator.mediaDevices?.getUserMedia) {
        statusNode.textContent = toolsQrText("qr_restore_camera_unsupported", "Camera stream is not supported by this browser.");
        return;
      }
      toolsQrCameraStream = await navigator.mediaDevices.getUserMedia({
        video: { facingMode: { ideal: "environment" } },
        audio: false,
      });
      video.srcObject = toolsQrCameraStream;
      video.setAttribute("playsinline", "true");
      await video.play();
      cameraWrap.hidden = false;
      toolsQrSetCameraState(cameraWrap, "searching");
      cameraBtn.textContent = toolsQrText("qr_restore_stop_camera", "Stop Camera");
      statusNode.textContent = toolsQrText("qr_restore_scan_hint", "Point the camera at the QR code.");

      const scan = () => {
        if (!toolsQrCameraStream || video.readyState < 2) {
          toolsQrScanTimer = requestAnimationFrame(scan);
          return;
        }
        canvas.width = video.videoWidth || 640;
        canvas.height = video.videoHeight || 480;
        const ctx = canvas.getContext("2d", { willReadFrequently: true });
        ctx.drawImage(video, 0, 0, canvas.width, canvas.height);
        const imageData = ctx.getImageData(0, 0, canvas.width, canvas.height);
        const code = typeof jsQR === "function" ? jsQR(imageData.data, canvas.width, canvas.height) : null;
        if (code?.data) {
          if (toolsQrIsCodeAligned(code, canvas.width, canvas.height)) {
            toolsQrAlignedFrames += 1;
            toolsQrSetCameraState(cameraWrap, "aligned");
            statusNode.textContent = toolsQrText("qr_restore_scan_aligned", "QR code detected. Hold still...");
            if (toolsQrAlignedFrames >= 2) {
              toolsQrSetCameraState(cameraWrap, "locked");
              statusNode.textContent = toolsQrText("qr_restore_scan_locked", "QR code captured.");
              toolsQrScanFinishTimer = window.setTimeout(() => {
                toolsQrScanFinishTimer = null;
                toolsQrStopCamera();
                cameraWrap.hidden = true;
                toolsQrSetCameraState(cameraWrap, "idle");
                cameraBtn.textContent = toolsQrText("qr_restore_camera", "Camera");
                preview(code.data);
              }, 160);
              return;
            }
          } else {
            toolsQrAlignedFrames = 0;
            toolsQrSetCameraState(cameraWrap, "detected");
            statusNode.textContent = toolsQrText("qr_restore_scan_detected", "Move the QR code into the guide box.");
          }
        } else {
          toolsQrAlignedFrames = 0;
          toolsQrSetCameraState(cameraWrap, "searching");
        }
        toolsQrScanTimer = requestAnimationFrame(scan);
      };
      toolsQrScanTimer = requestAnimationFrame(scan);
    } catch (e) {
      toolsQrStopCamera();
      cameraWrap.hidden = true;
      toolsQrSetCameraState(cameraWrap, "idle");
      cameraBtn.textContent = toolsQrText("qr_restore_camera", "Camera");
      statusNode.textContent = e?.message || toolsQrText("qr_restore_camera_failed", "Camera could not be opened.");
    }
  };
}

async function toolsQrShowRestore() {
  const state = { payload: "" };
  const html = `
    <div class="tools-qr-restore">
      <div class="tools-qr-actions">
        <button id="toolsQrImageBtn" class="btn tools-qr-action-btn" type="button">${toolsQrEscape(toolsQrText("qr_restore_upload", "Image"))}</button>
        <button id="toolsQrCameraBtn" class="btn tools-qr-action-btn tools-qr-action-btn--camera" type="button">${toolsQrEscape(toolsQrText("qr_restore_camera", "Camera"))}</button>
      </div>
      <input id="toolsQrImageInput" type="file" accept="image/*" hidden />
      <div id="toolsQrCameraWrap" class="tools-qr-camera" hidden>
        <video id="toolsQrVideo" muted playsinline></video>
        <div class="tools-qr-camera__overlay" aria-hidden="true">
          <div class="tools-qr-camera__guide">
            <span class="tools-qr-camera__corner tools-qr-camera__corner--tl"></span>
            <span class="tools-qr-camera__corner tools-qr-camera__corner--tr"></span>
            <span class="tools-qr-camera__corner tools-qr-camera__corner--bl"></span>
            <span class="tools-qr-camera__corner tools-qr-camera__corner--br"></span>
          </div>
        </div>
        <canvas id="toolsQrCanvas" hidden></canvas>
      </div>
      <div id="toolsQrStatus" class="tools-qr-status">${toolsQrEscape(toolsQrText("qr_restore_hint", "Scan or select a QR backup image before restoring."))}</div>
      <div id="toolsQrDiff" class="tools-qr-diff"></div>
    </div>
  `;

  const promise = openAppDialog({
    mode: "confirm",
    title: toolsQrText("qr_restore_title", "QR Restore"),
    html: true,
    messageHtml: html,
    confirmLabel: toolsQrText("qr_restore_apply", "Apply"),
    cancelLabel: toolsQrText("cancel", "Cancel"),
  });
  toolsQrSetDialogClass(true);
  window.setTimeout(() => toolsQrBindRestoreDialog(state), 0);
  const ok = await promise.finally(() => {
    toolsQrStopCamera();
    toolsQrSetDialogClass(false);
  });
  if (!ok || !state.payload) return;

  try {
    const j = await postJson("/api/params_restore_json", { payload: state.payload });
    const count = Number(j.result?.ok_cnt || 0);
    const failed = new Set((j.result?.fails || []).map((entry) => String(entry?.key || "")).filter(Boolean));
    const restoredValues = {};
    (j.preview?.entries || []).forEach((entry) => {
      if (!entry?.apply || failed.has(String(entry.key))) return;
      restoredValues[entry.key] = entry.value;
    });
    if (Object.keys(restoredValues).length) {
      window.dispatchEvent(new CustomEvent("carrot:paramsrestored", {
        detail: { source: "qr_restore", values: restoredValues },
      }));
      Object.entries(restoredValues).forEach(([name, value]) => {
        window.dispatchEvent(new CustomEvent("carrot:paramchange", {
          detail: { name, value, source: "qr_restore" },
        }));
      });
    }
    toolsLogNotice(JSON.stringify(j.result, null, 2), { label: "qr restore", meta: false });
    if (typeof showAppToast === "function") {
      showAppToast(toolsQrText("qr_restore_applied", "{count} params restored", { count }));
    }
    if (count > 0 && await appConfirm(UI_STRINGS[LANG].restore_done_reboot || "Restore done.\nReboot now?", {
      title: UI_STRINGS[LANG].reboot || "Reboot",
    })) {
      await runTool("reboot");
    }
  } catch (e) {
    showError("qr restore", e);
  }
}

function initToolsSettingsQr() {
  const bind = (id, fn) => {
    const el = document.getElementById(id);
    if (!el || el.dataset.qrBound === "1") return;
    el.dataset.qrBound = "1";
    el.addEventListener("click", fn);
  };
  bind("btnQrBackupSettings", toolsQrShowBackup);
  bind("btnQrRestoreSettings", toolsQrShowRestore);
}

document.addEventListener("DOMContentLoaded", initToolsSettingsQr);
if (document.readyState !== "loading") initToolsSettingsQr();
