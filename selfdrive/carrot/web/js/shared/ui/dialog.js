"use strict";

let appToastSerial = 0;
let activeAppToast = null;
let appToastHideTimer = null;
let appToastRemoveTimer = null;
let activeAppDialog = null;
let appDialogSerial = 0;


function syncModalBodyLock() {
  const hasOpenDialog =
    Boolean(appDialog && !appDialog.hidden) ||
    Boolean(appBranchPicker && !appBranchPicker.hidden) ||
    Boolean(appCarPicker && !appCarPicker.hidden) ||
    Boolean(settingSearchPanel && !settingSearchPanel.hidden);
  document.body.classList.toggle("dialog-open", hasOpenDialog);
}

function showAppToast(message, opts = {}) {
  if (!appToastHost || !message) return;

  const tone = opts.tone || "default";
  const duration = opts.duration ?? 2600;
  let toast = activeAppToast;
  if (!toast || !toast.isConnected) {
    toast = document.createElement("div");
    appToastHost.innerHTML = "";
    appToastHost.appendChild(toast);
    activeAppToast = toast;
  }

  toast.className = "app-toast";
  if (tone && tone !== "default") toast.classList.add(`is-${tone}`);
  toast.textContent = String(message);

  if (appToastHideTimer) {
    clearTimeout(appToastHideTimer);
    appToastHideTimer = null;
  }
  if (appToastRemoveTimer) {
    clearTimeout(appToastRemoveTimer);
    appToastRemoveTimer = null;
  }

  appToastSerial += 1;
  const toastSerial = appToastSerial;
  requestAnimationFrame(() => {
    if (!activeAppToast || toastSerial !== appToastSerial) return;
    activeAppToast.classList.add("is-visible");
  });

  appToastHideTimer = window.setTimeout(() => {
    if (!activeAppToast || toastSerial !== appToastSerial) return;
    activeAppToast.classList.remove("is-visible");
    appToastHideTimer = null;
    appToastRemoveTimer = window.setTimeout(() => {
      if (!activeAppToast || toastSerial !== appToastSerial) return;
      activeAppToast.remove();
      activeAppToast = null;
      appToastRemoveTimer = null;
    }, 180);
  }, duration);
}


/* ── Dialog (alert / confirm / prompt / choice) ─────────── */
function resolveAppDialog(result) {
  if (!activeAppDialog || !appDialog) return;

  const state = activeAppDialog;
  activeAppDialog = null;
  const dialogSerial = state.serial;
  appDialog.classList.remove("is-open");

  window.setTimeout(() => {
    if (dialogSerial !== appDialogSerial) {
      state.resolve(result);
      return;
    }
    appDialog.hidden = true;
    syncModalBodyLock();
    if (appDialogChoices) {
      appDialogChoices.hidden = true;
      appDialogChoices.innerHTML = "";
    }
    if (appDialogInputWrap) appDialogInputWrap.hidden = true;
    if (appDialogInput) {
      appDialogInput.value = "";
      appDialogInput.placeholder = "";
    }
    if (state.lastFocus && typeof state.lastFocus.focus === "function") {
      state.lastFocus.focus();
    }
    state.resolve(result);
  }, 180);
}

function cancelAppDialog() {
  if (!activeAppDialog) return;
  const result = activeAppDialog.mode === "prompt" || activeAppDialog.mode === "choice"
    ? null
    : false;
  resolveAppDialog(result);
}

function confirmAppDialog() {
  if (!activeAppDialog) return;
  const result = activeAppDialog.mode === "prompt"
    ? (appDialogInput ? appDialogInput.value : "")
    : true;
  resolveAppDialog(result);
}

function openAppDialog(options = {}) {
  if (!appDialog || !appDialogTitle || !appDialogBody || !appDialogConfirm || !appDialogCancel) {
    if (options.mode === "prompt") return Promise.resolve(null);
    return Promise.resolve(options.mode === "alert");
  }

  if (activeAppDialog) cancelAppDialog();

  const mode = options.mode || "alert";
  const title =
    options.title ||
    (mode === "confirm"
      ? getUIText("confirm_title", "Confirm")
      : mode === "prompt"
        ? getUIText("input_title", "Input")
        : getUIText("notice", "Notice"));
  const message = options.message || "";
  const messageHtml = options.messageHtml || "";
  const useHtml = Boolean(options.html);
  const confirmLabel = options.confirmLabel || getUIText("ok", "OK");
  const cancelLabel = options.cancelLabel || getUIText("cancel", "Cancel");
  const choices = Array.isArray(options.choices)
    ? options.choices.filter((choice) => choice && (choice.label || choice.labelHtml))
    : [];
  const hasChoices = choices.length > 0;
  const isChoice = mode === "choice" || hasChoices;
  const showCancel = mode !== "alert";

  appDialogTitle.textContent = title;
  if (useHtml) appDialogBody.innerHTML = String(messageHtml || message);
  else appDialogBody.textContent = String(message);
  appDialogBody.style.flex = hasChoices ? "0 0 auto" : "1 1 auto";
  appDialogConfirm.textContent = confirmLabel;
  appDialogCancel.textContent = cancelLabel;
  appDialogConfirm.disabled = false;
  appDialogCancel.disabled = false;
  appDialogCancel.hidden = !showCancel;
  appDialogCancel.setAttribute("aria-hidden", showCancel ? "false" : "true");
  appDialogConfirm.hidden = isChoice;
  appDialogConfirm.setAttribute("aria-hidden", isChoice ? "true" : "false");

  const copyText = options.copyText || "";
  if (appDialogCopy) {
    appDialogCopy.hidden = !copyText;
    appDialogCopy.textContent = options.copyLabel || getUIText("copy", "Copy");
    appDialogCopy.onclick = copyText ? () => {
      copyToClipboard(copyText);
      alert(getUIText("copied", "Copied"));
    } : null;
  }

  if (appDialogChoices) {
    appDialogChoices.innerHTML = "";
    appDialogChoices.hidden = !hasChoices;
    for (const choice of choices) {
      const button = document.createElement("button");
      button.type = "button";
      let btnClass = choice.danger
        ? "btn btn--danger app-dialog__choiceBtn"
        : "btn app-dialog__choiceBtn";
      if (choice.className) btnClass += " " + choice.className;
      button.className = btnClass;
      if (choice.labelHtml) {
        button.innerHTML = choice.labelHtml;
      } else {
        button.textContent = String(choice.label);
      }
      button.style.cssText = "text-align:left; overflow:hidden; text-overflow:ellipsis; white-space:nowrap;";
      button.addEventListener("click", () => resolveAppDialog(choice.value));
      appDialogChoices.appendChild(button);
    }
  }

  if (appDialogInputWrap && appDialogInput) {
    const isPrompt = mode === "prompt";
    appDialogInputWrap.hidden = !isPrompt;
    appDialogInput.value = options.defaultValue ?? "";
    appDialogInput.placeholder = options.placeholder || "";
  }

  return new Promise((resolve) => {
    const dialogSerial = ++appDialogSerial;
    activeAppDialog = {
      resolve,
      mode,
      serial: dialogSerial,
      lastFocus: document.activeElement instanceof HTMLElement ? document.activeElement : null,
    };

    appDialog.hidden = false;
    syncModalBodyLock();

    requestAnimationFrame(() => {
      appDialog.classList.add("is-open");
      if (mode === "prompt" && appDialogInput) {
        appDialogInput.focus();
        appDialogInput.select();
      } else if (hasChoices && appDialogChoices) {
        const firstChoice = appDialogChoices.querySelector("button");
        if (firstChoice && typeof firstChoice.focus === "function") firstChoice.focus();
      } else if (mode === "choice" && appDialogCancel) {
        appDialogCancel.focus();
      } else {
        appDialogConfirm.focus();
      }
    });
  });
}

function appAlert(message, opts = {}) {
  return openAppDialog({
    mode: "alert",
    title: opts.title,
    message,
    messageHtml: opts.messageHtml,
    html: opts.html,
    confirmLabel: opts.confirmLabel,
    copyText: opts.copyText,
  });
}

function appConfirm(message, opts = {}) {
  return openAppDialog({
    mode: "confirm",
    title: opts.title,
    message,
    confirmLabel: opts.confirmLabel,
    cancelLabel: opts.cancelLabel,
  });
}

function appPrompt(message, opts = {}) {
  return openAppDialog({
    mode: "prompt",
    title: opts.title,
    message,
    confirmLabel: opts.confirmLabel,
    cancelLabel: opts.cancelLabel,
    defaultValue: opts.defaultValue,
    placeholder: opts.placeholder,
  });
}

if (appDialogBackdrop) appDialogBackdrop.onclick = cancelAppDialog;
if (appDialogCancel) appDialogCancel.onclick = cancelAppDialog;
if (appDialogConfirm) appDialogConfirm.onclick = confirmAppDialog;

document.addEventListener("keydown", (ev) => {
  if (!activeAppDialog) return;

  if (ev.key === "Escape") {
    ev.preventDefault();
    if (activeAppDialog.mode === "alert") resolveAppDialog(true);
    else cancelAppDialog();
    return;
  }

  if (ev.key === "Enter" && !ev.shiftKey) {
    const targetTag = ev.target?.tagName;
    if (targetTag === "TEXTAREA") return;
    ev.preventDefault();
    confirmAppDialog();
  }
});
