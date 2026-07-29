import { createFocusTrap as createDefaultFocusTrap } from "./focus_trap.js";
import {
  createAppProgressView,
  normalizeAppProgressState,
} from "../progress/progress.js";

export { normalizeAppProgressState };

export const APP_DIALOG_CLOSE_DELAY = 180;

export const APP_DIALOG_VARIANT_CLASSES = Object.freeze([
  "app-dialog--choice",
  "app-dialog--choice-list",
  "app-dialog--choice-grid",
  "app-dialog--choice-value-grid",
  "app-dialog--form",
  "app-dialog--input",
  "app-dialog--progress",
]);

const MODAL_SURFACE_IDS = Object.freeze([
  "appDialog",
  "appBranchPicker",
  "appCarPicker",
  "settingSearchPanel",
]);

function choiceText(choice) {
  if (!choice || choice.labelHtml) return "";
  return String(choice.label ?? "").trim();
}

function choiceHeading(choice) {
  if (!choice || typeof choice !== "object") return "";
  return String(choice.heading ?? "").trim();
}

function isChoiceItem(choice) {
  return Boolean(choice) && !choiceHeading(choice) && (choice.label != null || Boolean(choice.labelHtml));
}

// Split choices into sections. A `{ heading }` entry opens a section and the
// choices after it belong to it, so an ungrouped list stays a single section
// with no heading and renders exactly as before.
export function appDialogChoiceGroups(choices) {
  const groups = [];
  for (const choice of Array.isArray(choices) ? choices : []) {
    const heading = choiceHeading(choice);
    if (heading) {
      groups.push({ heading, items: [] });
      continue;
    }
    if (!isChoiceItem(choice)) continue;
    if (!groups.length) groups.push({ heading: "", items: [] });
    groups[groups.length - 1].items.push(choice);
  }
  return groups.filter((group) => group.items.length > 0);
}

export function inferAppDialogChoiceLayout(choices, options = {}) {
  const explicit = String(options.choiceLayout || options.choiceKind || "").trim();
  if (explicit === "grid" || explicit === "value-grid" || explicit === "values") return "value-grid";
  if (explicit === "list" || explicit === "action-list" || explicit === "actions") return "list";

  const shortValueChoices = choices.length > 4 && choices.every((choice) => {
    const text = choiceText(choice);
    return text && text.length <= 5 && !choice.danger
      && /^[+-]?(?:\d+|\d+\.\d+|[A-Za-z]{1,4})$/.test(text);
  });
  return shortValueChoices ? "value-grid" : "list";
}

export function appDialogChoiceLayout(groups, options = {}) {
  const items = Array.isArray(groups) ? groups.flatMap((group) => group.items || []) : [];
  if (!items.length) return "";
  // Section headings need stacked rows; a value grid has no room for them.
  if (groups.some((group) => Boolean(group?.heading))) return "list";
  return inferAppDialogChoiceLayout(items, options);
}

export function appDialogChoiceColumns(count, options = {}) {
  const explicit = Number(options.choiceColumns || options.columns);
  if (Number.isInteger(explicit) && explicit >= 2 && explicit <= 6) return explicit;
  if (count <= 4) return Math.max(2, count);
  if (count <= 25) return 5;
  return 4;
}

function controllerEnvironment(environment) {
  const target = environment.target ?? globalThis;
  const documentRoot = environment.document ?? target.document;
  const setTimer = environment.setTimeout
    ?? target.setTimeout?.bind(target)
    ?? globalThis.setTimeout.bind(globalThis);
  const clearTimer = environment.clearTimeout
    ?? target.clearTimeout?.bind(target)
    ?? globalThis.clearTimeout.bind(globalThis);
  const requestFrame = environment.requestAnimationFrame
    ?? target.requestAnimationFrame?.bind(target)
    ?? ((callback) => setTimer(callback, 0));
  const cancelFrame = environment.cancelAnimationFrame
    ?? target.cancelAnimationFrame?.bind(target)
    ?? clearTimer;
  return { target, documentRoot, setTimer, requestFrame, cancelFrame };
}

export function createDialogController(environment = {}) {
  const { target, documentRoot, setTimer, requestFrame, cancelFrame } = controllerEnvironment(environment);
  const element = (id) => documentRoot?.getElementById?.(id) ?? null;
  const appDialog = element("appDialog");
  const appDialogBackdrop = element("appDialogBackdrop");
  const appDialogTitle = element("appDialogTitle");
  const appDialogBody = element("appDialogBody");
  const appDialogChoices = element("appDialogChoices");
  const appDialogInputWrap = element("appDialogInputWrap");
  const appDialogInput = element("appDialogInput");
  const appDialogInputError = element("appDialogInputError");
  const appDialogCopy = element("appDialogCopy");
  const appDialogDefault = element("appDialogDefault");
  const appDialogCancel = element("appDialogCancel");
  const appDialogConfirm = element("appDialogConfirm");
  let activeDialog = null;
  let dialogSerial = 0;
  let pendingRestoreFocus = null;

  const makeFocusTrap = environment.createFocusTrap ?? ((container, options) => createDefaultFocusTrap(
    container,
    options,
    {
      document: documentRoot,
      window: target,
      requestAnimationFrame: requestFrame,
      cancelAnimationFrame: cancelFrame,
    },
  ));

  function text(key, fallback) {
    const getUIText = target.getUIText;
    return typeof getUIText === "function" ? getUIText.call(target, key, fallback) : fallback;
  }

  function syncModalBodyLock() {
    const hasOpenDialog = MODAL_SURFACE_IDS.some((id) => {
      const surface = element(id);
      return Boolean(surface && !surface.hidden);
    });
    documentRoot?.body?.classList?.toggle("dialog-open", hasOpenDialog);
  }

  function resetPresentation() {
    appDialog?.classList?.remove(...APP_DIALOG_VARIANT_CLASSES);
    if (appDialogChoices) {
      appDialogChoices.className = "app-dialog__choices";
      appDialogChoices.style.removeProperty("--app-dialog-choice-columns");
    }
  }

  function dialogCancelResult(state) {
    return state.mode === "prompt" || state.mode === "form" || state.mode === "choice" ? null : false;
  }

  function restoreFocus(state) {
    if (state.lastFocus && typeof state.lastFocus.focus === "function") state.lastFocus.focus();
  }

  function finalizeDialog(state, result) {
    if (state.serial !== dialogSerial) {
      state.resolve(result);
      return;
    }

    appDialog.hidden = true;
    syncModalBodyLock();
    resetPresentation();
    if (appDialogChoices) {
      appDialogChoices.hidden = true;
      appDialogChoices.innerHTML = "";
    }
    if (appDialogDefault) {
      appDialogDefault.hidden = true;
      appDialogDefault.onclick = null;
    }
    if (appDialogInputWrap) appDialogInputWrap.hidden = true;
    if (appDialogInputError) {
      appDialogInputError.hidden = true;
      appDialogInputError.textContent = "";
    }
    if (appDialogInput) {
      appDialogInput.disabled = false;
      appDialogInput.value = "";
      appDialogInput.placeholder = "";
      appDialogInput.type = "text";
      appDialogInput.removeAttribute("autocomplete");
      appDialogInput.removeAttribute("autocapitalize");
      appDialogInput.removeAttribute("inputmode");
      appDialogInput.removeAttribute("aria-labelledby");
      appDialogInput.spellcheck = true;
    }
    if (appDialogBody) appDialogBody.hidden = false;
    if (appDialogConfirm) appDialogConfirm.disabled = false;
    if (appDialogCancel) appDialogCancel.disabled = false;
    restoreFocus(state);
    pendingRestoreFocus = null;
    state.resolve(result);
  }

  function resolveDialogState(state, result) {
    if (!state || activeDialog !== state || state.closing) return;
    state.closing = true;
    activeDialog = null;
    pendingRestoreFocus = state.lastFocus || pendingRestoreFocus;
    if (state.openFrame != null) {
      cancelFrame(state.openFrame);
      state.openFrame = null;
    }
    state.focusTrap?.deactivate?.({ restoreFocus: false });
    appDialog?.classList?.remove("is-open");
    state.closeTimer = setTimer(() => {
      state.closeTimer = null;
      finalizeDialog(state, result);
    }, APP_DIALOG_CLOSE_DELAY);
  }

  function resolveAppDialog(result) {
    resolveDialogState(activeDialog, result);
  }

  async function cancelAppDialog() {
    const state = activeDialog;
    if (!state || state.submitting || state.canceling) return;
    if (typeof state.onCancel !== "function") {
      resolveDialogState(state, dialogCancelResult(state));
      return;
    }
    state.canceling = true;
    if (appDialogCancel) {
      appDialogCancel.disabled = true;
      appDialogCancel.textContent = state.cancelingLabel;
    }
    try {
      await state.onCancel();
      if (state.closeOnCancel && activeDialog === state && !state.closing) {
        resolveDialogState(state, dialogCancelResult(state));
      }
    } catch (error) {
      if (activeDialog !== state || state.closing) return;
      state.canceling = false;
      if (appDialogCancel) {
        appDialogCancel.disabled = false;
        appDialogCancel.textContent = state.cancelLabel;
      }
      if (typeof target.showAppToast === "function") {
        target.showAppToast(error?.message || String(error), { tone: "error", duration: 3600 });
      }
    }
  }

  async function confirmAppDialog() {
    if (!activeDialog || activeDialog.submitting) return;
    const state = activeDialog;
    const isInputMode = state.mode === "prompt" || state.mode === "form";
    const result = isInputMode ? (appDialogInput ? appDialogInput.value : "") : true;
    if (state.mode !== "form" || typeof state.onSubmit !== "function") {
      resolveDialogState(state, result);
      return;
    }

    state.submitting = true;
    if (appDialogInputError) {
      appDialogInputError.hidden = true;
      appDialogInputError.textContent = "";
    }
    if (appDialogInput) appDialogInput.disabled = true;
    if (appDialogConfirm) {
      appDialogConfirm.disabled = true;
      appDialogConfirm.textContent = state.submittingLabel;
    }
    if (appDialogCancel) appDialogCancel.disabled = true;

    try {
      await state.onSubmit(result);
      if (activeDialog === state && !state.closing) resolveDialogState(state, result);
    } catch (error) {
      if (activeDialog !== state || state.closing) return;
      state.submitting = false;
      if (appDialogInput) {
        appDialogInput.disabled = false;
        appDialogInput.focus();
      }
      if (appDialogConfirm) {
        appDialogConfirm.disabled = false;
        appDialogConfirm.textContent = state.confirmLabel;
      }
      if (appDialogCancel) appDialogCancel.disabled = false;
      if (appDialogInputError) {
        appDialogInputError.textContent = error?.message || String(error);
        appDialogInputError.hidden = false;
      }
    }
  }

  function openAppDialog(options = {}) {
    if (!appDialog || !appDialogTitle || !appDialogBody || !appDialogConfirm || !appDialogCancel) {
      if (options.mode === "prompt" || options.mode === "form") return Promise.resolve(null);
      return Promise.resolve(options.mode === "alert");
    }

    let inheritedLastFocus = pendingRestoreFocus;
    if (activeDialog) {
      inheritedLastFocus = activeDialog.lastFocus || inheritedLastFocus;
      resolveDialogState(activeDialog, dialogCancelResult(activeDialog));
    }
    pendingRestoreFocus = null;

    const mode = options.mode || "alert";
    const isForm = mode === "form";
    const title = options.title || (mode === "confirm"
      ? text("confirm_title", "Confirm")
      : mode === "prompt" || mode === "form"
        ? text("input_title", "Input")
        : text("notice", "Notice"));
    const message = options.message || "";
    const messageHtml = options.messageHtml || "";
    const useHtml = Boolean(options.html);
    const confirmLabel = options.confirmLabel || text("ok", "OK");
    const cancelLabel = options.cancelLabel || text("cancel", "Cancel");
    const defaultActionLabel = options.defaultActionLabel || "";
    const hasDefaultAction = mode === "prompt" && Boolean(defaultActionLabel);
    const choiceGroups = appDialogChoiceGroups(options.choices);
    const choices = choiceGroups.flatMap((group) => group.items);
    const hasChoices = choices.length > 0;
    const isChoice = mode === "choice" || hasChoices;
    const choiceLayout = appDialogChoiceLayout(choiceGroups, options);
    const showCancel = mode !== "alert" && options.showCancel !== false;

    resetPresentation();
    if (isForm) appDialog.classList.add("app-dialog--form");
    if (mode === "prompt" || isForm) appDialog.classList.add("app-dialog--input");
    if (hasChoices) {
      appDialog.classList.add("app-dialog--choice");
      appDialog.classList.add(choiceLayout === "value-grid" ? "app-dialog--choice-grid" : "app-dialog--choice-list");
      if (choiceLayout === "value-grid") appDialog.classList.add("app-dialog--choice-value-grid");
    }

    appDialogTitle.textContent = title;
    if (useHtml) appDialogBody.innerHTML = String(messageHtml || message);
    else appDialogBody.textContent = String(message);
    appDialogBody.hidden = isForm && !String(messageHtml || message).trim();
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
      appDialogCopy.textContent = options.copyLabel || text("copy", "Copy");
      appDialogCopy.onclick = copyText ? () => {
        const copyToClipboard = target.copyToClipboard;
        if (typeof copyToClipboard === "function") copyToClipboard.call(target, copyText);
        const alert = target.alert;
        if (typeof alert === "function") alert.call(target, text("copied", "Copied"));
      } : null;
    }

    if (appDialogChoices) {
      appDialogChoices.innerHTML = "";
      appDialogChoices.hidden = !hasChoices;
      appDialogChoices.className = `app-dialog__choices app-dialog__choices--${choiceLayout || "list"}`;
      if (choiceLayout === "value-grid") {
        appDialogChoices.style.setProperty(
          "--app-dialog-choice-columns",
          String(appDialogChoiceColumns(choices.length, options)),
        );
      } else {
        appDialogChoices.style.removeProperty("--app-dialog-choice-columns");
      }
      const createChoiceButton = (choice) => {
        const button = documentRoot.createElement("button");
        button.type = "button";
        let buttonClass = choice.danger
          ? "btn btn--danger app-dialog__choiceBtn"
          : "btn app-dialog__choiceBtn";
        buttonClass += choiceLayout === "value-grid"
          ? " app-dialog__choiceBtn--value"
          : " app-dialog__choiceBtn--action";
        if (choice.current || choice.selected) buttonClass += " is-current";
        if (choice.className) buttonClass += ` ${choice.className}`;
        button.className = buttonClass;
        if (choice.labelHtml) button.innerHTML = choice.labelHtml;
        else button.textContent = String(choice.label);
        button.addEventListener("click", () => resolveAppDialog(choice.value));
        return button;
      };

      choiceGroups.forEach((group, index) => {
        if (!group.heading) {
          for (const choice of group.items) appDialogChoices.appendChild(createChoiceButton(choice));
          return;
        }
        // A labelled section: screen readers announce the heading as the group
        // name instead of it reading as one more choice in the list.
        const section = documentRoot.createElement("div");
        section.className = "app-dialog__choiceGroup";
        section.setAttribute("role", "group");
        const headingId = `appDialogChoiceHeading${index}`;
        section.setAttribute("aria-labelledby", headingId);
        const heading = documentRoot.createElement("div");
        heading.id = headingId;
        heading.className = "app-dialog__choiceHeading";
        heading.textContent = group.heading;
        section.appendChild(heading);
        for (const choice of group.items) section.appendChild(createChoiceButton(choice));
        appDialogChoices.appendChild(section);
      });
    }

    if (appDialogInputWrap && appDialogInput) {
      const isPrompt = mode === "prompt" || isForm;
      appDialogInputWrap.hidden = !isPrompt;
      appDialogInput.disabled = false;
      appDialogInput.value = options.defaultValue ?? "";
      appDialogInput.placeholder = options.placeholder || "";
      appDialogInput.type = options.inputType === "password" ? "password" : "text";
      const autocomplete = options.autocomplete ?? (isForm ? "off" : "");
      const autocapitalize = options.autocapitalize ?? (isForm ? "none" : "");
      if (autocomplete) appDialogInput.autocomplete = autocomplete;
      else appDialogInput.removeAttribute("autocomplete");
      if (autocapitalize) appDialogInput.autocapitalize = autocapitalize;
      else appDialogInput.removeAttribute("autocapitalize");
      appDialogInput.spellcheck = options.spellcheck == null ? !isForm : options.spellcheck === true;
      if (isForm) appDialogInput.setAttribute("aria-labelledby", "appDialogTitle");
      else appDialogInput.removeAttribute("aria-labelledby");
      if (options.inputMode) appDialogInput.inputMode = options.inputMode;
      else appDialogInput.removeAttribute("inputmode");
      if (appDialogInputError) {
        appDialogInputError.hidden = true;
        appDialogInputError.textContent = "";
      }
    }

    return new Promise((resolve) => {
      const state = {
        resolve,
        mode,
        serial: ++dialogSerial,
        onSubmit: options.onSubmit,
        onCancel: options.onCancel,
        closeOnCancel: options.closeOnCancel !== false,
        submitting: false,
        canceling: false,
        closing: false,
        confirmLabel,
        cancelLabel,
        cancelingLabel: options.cancelingLabel || text("canceling", "Canceling..."),
        submittingLabel: options.submittingLabel || text("saving", "Saving..."),
        lastFocus: inheritedLastFocus || documentRoot.activeElement || null,
        focusTrap: null,
        openFrame: null,
        closeTimer: null,
      };
      activeDialog = state;
      appDialog.hidden = false;
      syncModalBodyLock();

      const trapSurface = appDialog.querySelector?.(".app-dialog__sheet") || appDialog;
      state.focusTrap = makeFocusTrap(trapSurface, {
        focusInitial: false,
        returnFocus: state.lastFocus,
      });
      state.focusTrap?.activate?.();
      state.openFrame = requestFrame(() => {
        state.openFrame = null;
        if (activeDialog !== state || state.serial !== dialogSerial || state.closing) return;
        appDialog.classList.add("is-open");
        if ((mode === "prompt" || isForm) && appDialogInput) {
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
        } else if (mode === "choice") {
          appDialogCancel.focus();
        } else {
          appDialogConfirm.focus();
        }
      });
    });
  }

  function appAlert(message, options = {}) {
    return openAppDialog({
      mode: "alert",
      title: options.title,
      message,
      messageHtml: options.messageHtml,
      html: options.html,
      confirmLabel: options.confirmLabel,
      copyText: options.copyText,
    });
  }

  function openAppProgressDialog(options = {}) {
    const completion = openAppDialog({
      mode: "choice",
      title: options.title,
      message: " ",
      cancelLabel: options.cancelLabel,
      cancelingLabel: options.cancelingLabel,
      onCancel: options.onCancel,
      closeOnCancel: false,
    });
    const state = activeDialog;
    appDialog?.classList?.add("app-dialog--progress");
    if (appDialogBody) appDialogBody.style.flex = "0 0 auto";
    const progressView = createAppProgressView(documentRoot, options);
    if (appDialogBody && progressView) appDialogBody.replaceChildren(progressView.element);

    const isCurrent = () => Boolean(state && activeDialog === state && !state.closing);
    const controller = {
      completion,
      setCancelHandler(handler) {
        if (!isCurrent()) return;
        state.onCancel = typeof handler === "function" ? handler : null;
        if (appDialogCancel) {
          appDialogCancel.hidden = !state.onCancel;
          appDialogCancel.setAttribute("aria-hidden", state.onCancel ? "false" : "true");
        }
      },
      setCanceling(active) {
        if (!isCurrent() || !appDialogCancel) return;
        state.canceling = Boolean(active);
        appDialogCancel.disabled = state.canceling;
        appDialogCancel.textContent = state.canceling ? state.cancelingLabel : state.cancelLabel;
      },
      setMessage(value) {
        if (isCurrent()) progressView?.setMessage(value);
      },
      setProgressState(value) {
        if (isCurrent()) progressView?.setProgressState(value);
      },
      setProgress(value) {
        controller.setProgressState(value);
      },
      setSummary(value) {
        if (isCurrent()) progressView?.setSummary(value);
      },
      close() {
        if (isCurrent()) resolveDialogState(state, true);
        return completion;
      },
    };
    controller.setMessage(options.message);
    controller.setProgressState(options.progressState ?? options.progress);
    controller.setSummary(options.summary);
    controller.setCancelHandler(options.onCancel);
    return Object.freeze(controller);
  }

  function appConfirm(message, options = {}) {
    return openAppDialog({
      mode: "confirm",
      title: options.title,
      message,
      messageHtml: options.messageHtml,
      html: options.html,
      confirmLabel: options.confirmLabel,
      cancelLabel: options.cancelLabel,
    });
  }

  function appPrompt(message, options = {}) {
    return openAppDialog({
      mode: "prompt",
      title: options.title,
      message,
      confirmLabel: options.confirmLabel,
      cancelLabel: options.cancelLabel,
      defaultValue: options.defaultValue,
      defaultActionLabel: options.defaultActionLabel,
      defaultActionValue: options.defaultActionValue,
      showCancel: options.showCancel,
      placeholder: options.placeholder,
    });
  }

  function appForm(message, options = {}) {
    return openAppDialog({
      mode: "form",
      title: options.title,
      message,
      defaultValue: options.defaultValue,
      placeholder: options.placeholder,
      inputType: options.inputType,
      autocomplete: options.autocomplete,
      autocapitalize: options.autocapitalize,
      inputMode: options.inputMode,
      spellcheck: options.spellcheck,
      confirmLabel: options.confirmLabel,
      cancelLabel: options.cancelLabel,
      submittingLabel: options.submittingLabel,
      onSubmit: options.onSubmit,
    });
  }

  if (appDialogBackdrop) appDialogBackdrop.onclick = cancelAppDialog;
  if (appDialogCancel) appDialogCancel.onclick = cancelAppDialog;
  if (appDialogConfirm) appDialogConfirm.onclick = confirmAppDialog;

  function onDocumentKeydown(event) {
    if (!activeDialog) return;
    if (event.key === "Escape") {
      event.preventDefault();
      if (activeDialog.mode === "alert") resolveDialogState(activeDialog, true);
      else cancelAppDialog();
      return;
    }
    if (event.key !== "Enter" || event.shiftKey || event.isComposing || event.defaultPrevented) return;
    const targetTag = event.target?.tagName;
    const nativeInteractive = targetTag === "BUTTON"
      || targetTag === "SELECT"
      || targetTag === "TEXTAREA"
      || targetTag === "A"
      || (targetTag === "INPUT" && event.target !== appDialogInput)
      || event.target?.isContentEditable;
    if (nativeInteractive) return;
    event.preventDefault();
    confirmAppDialog();
  }
  documentRoot?.addEventListener?.("keydown", onDocumentKeydown);

  return Object.freeze({
    openAppDialog,
    openAppProgressDialog,
    appAlert,
    appConfirm,
    appPrompt,
    appForm,
    syncModalBodyLock,
    cancelAppDialog,
    confirmAppDialog,
  });
}
