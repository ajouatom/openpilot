"use strict";

let appToastSerial = 0;
let activeAppToast = null;
let appToastHideTimer = null;
let appToastRemoveTimer = null;
let activeAppDialog = null;
let appDialogSerial = 0;

const APP_DIALOG_VARIANT_CLASSES = [
  "app-dialog--choice",
  "app-dialog--choice-list",
  "app-dialog--choice-grid",
  "app-dialog--choice-value-grid",
];


function syncModalBodyLock() {
  const hasOpenDialog =
    Boolean(appDialog && !appDialog.hidden) ||
    Boolean(appBranchPicker && !appBranchPicker.hidden) ||
    Boolean(appCarPicker && !appCarPicker.hidden) ||
    Boolean(settingSearchPanel && !settingSearchPanel.hidden);
  document.body.classList.toggle("dialog-open", hasOpenDialog);
}

// Tone → icon glyph + style class. Centralized here so every toast across the
// app (showAppToast is the single entry point) renders the same toast surface.
const APP_TOAST_TONE_GLYPH = {
  success: "✓", // ✓
  error: "✕",   // ✕
  warn: "!",
  offline: "!",
  info: "i",
  hint: "i",
  default: "i",
};
const APP_TOAST_TONE_CLASS = {
  success: "is-success",
  error: "is-error",
  warn: "is-warn",
  offline: "is-warn",
  info: "is-info",
  hint: "is-hint",
};

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

  const toneClass = APP_TOAST_TONE_CLASS[tone] || "";
  toast.className = toneClass ? `app-toast ${toneClass}` : "app-toast";

  const icon = document.createElement("span");
  icon.className = "app-toast__icon";
  icon.setAttribute("aria-hidden", "true");
  icon.textContent = APP_TOAST_TONE_GLYPH[tone] || APP_TOAST_TONE_GLYPH.default;

  const msg = document.createElement("div");
  msg.className = "app-toast__msg";
  msg.textContent = String(message);

  toast.replaceChildren(icon, msg);

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
    }, 220);
  }, duration);
}


/* ── Dialog (alert / confirm / prompt / choice) ─────────── */
function resetAppDialogPresentation() {
  if (appDialog) appDialog.classList.remove(...APP_DIALOG_VARIANT_CLASSES);
  if (appDialogChoices) {
    appDialogChoices.className = "app-dialog__choices";
    appDialogChoices.style.removeProperty("--app-dialog-choice-columns");
  }
}

function appDialogChoiceText(choice) {
  if (!choice || choice.labelHtml) return "";
  return String(choice.label ?? "").trim();
}

function inferAppDialogChoiceLayout(choices, options = {}) {
  const explicit = String(options.choiceLayout || options.choiceKind || "").trim();
  if (explicit === "grid" || explicit === "value-grid" || explicit === "values") return "value-grid";
  if (explicit === "list" || explicit === "action-list" || explicit === "actions") return "list";

  const shortValueChoices = choices.length > 4 && choices.every((choice) => {
    const text = appDialogChoiceText(choice);
    return text && text.length <= 5 && !choice.danger && /^[+-]?(?:\d+|\d+\.\d+|[A-Za-z]{1,4})$/.test(text);
  });
  return shortValueChoices ? "value-grid" : "list";
}

function appDialogChoiceColumns(count, options = {}) {
  const explicit = Number(options.choiceColumns || options.columns);
  if (Number.isInteger(explicit) && explicit >= 2 && explicit <= 6) return explicit;
  if (count <= 4) return Math.max(2, count);
  if (count <= 25) return 5;
  return 4;
}

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
    resetAppDialogPresentation();
    if (appDialogChoices) {
      appDialogChoices.hidden = true;
      appDialogChoices.innerHTML = "";
    }
    if (appDialogDefault) {
      appDialogDefault.hidden = true;
      appDialogDefault.onclick = null;
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
  const defaultActionLabel = options.defaultActionLabel || "";
  const hasDefaultAction = mode === "prompt" && Boolean(defaultActionLabel);
  const choices = Array.isArray(options.choices)
    ? options.choices.filter((choice) => choice && (choice.label != null || choice.labelHtml))
    : [];
  const hasChoices = choices.length > 0;
  const isChoice = mode === "choice" || hasChoices;
  const choiceLayout = hasChoices ? inferAppDialogChoiceLayout(choices, options) : "";
  const showCancel = mode !== "alert" && options.showCancel !== false;

  resetAppDialogPresentation();
  if (appDialog && hasChoices) {
    appDialog.classList.add("app-dialog--choice");
    appDialog.classList.add(choiceLayout === "value-grid" ? "app-dialog--choice-grid" : "app-dialog--choice-list");
    if (choiceLayout === "value-grid") appDialog.classList.add("app-dialog--choice-value-grid");
  }

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
  if (appDialogDefault) {
    appDialogDefault.hidden = !hasDefaultAction;
    appDialogDefault.textContent = defaultActionLabel;
    appDialogDefault.disabled = false;
    appDialogDefault.onclick = hasDefaultAction
      ? () => resolveAppDialog(options.defaultActionValue ?? "")
      : null;
  }

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
    appDialogChoices.className = `app-dialog__choices app-dialog__choices--${choiceLayout || "list"}`;
    if (choiceLayout === "value-grid") {
      appDialogChoices.style.setProperty("--app-dialog-choice-columns", String(appDialogChoiceColumns(choices.length, options)));
    } else {
      appDialogChoices.style.removeProperty("--app-dialog-choice-columns");
    }
    for (const choice of choices) {
      const button = document.createElement("button");
      button.type = "button";
      let btnClass = choice.danger
        ? "btn btn--danger app-dialog__choiceBtn"
        : "btn app-dialog__choiceBtn";
      btnClass += choiceLayout === "value-grid"
        ? " app-dialog__choiceBtn--value"
        : " app-dialog__choiceBtn--action";
      if (choice.current || choice.selected) btnClass += " is-current";
      if (choice.className) btnClass += " " + choice.className;
      button.className = btnClass;
      if (choice.labelHtml) {
        button.innerHTML = choice.labelHtml;
      } else {
        button.textContent = String(choice.label);
      }
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
        const currentChoice = appDialogChoices.querySelector(".is-current");
        const firstChoice = currentChoice || appDialogChoices.querySelector("button");
        if (firstChoice && typeof firstChoice.focus === "function") {
          firstChoice.focus({ preventScroll: Boolean(currentChoice) });
          if (currentChoice && typeof currentChoice.scrollIntoView === "function") {
            currentChoice.scrollIntoView({ block: "center", inline: "nearest" });
          }
        }
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
    defaultActionLabel: opts.defaultActionLabel,
    defaultActionValue: opts.defaultActionValue,
    showCancel: opts.showCancel,
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
