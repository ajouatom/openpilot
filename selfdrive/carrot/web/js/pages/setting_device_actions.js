"use strict";

function bindDeviceTabEvents(container) {
  bindDeviceDisabledControls(container);
  bindDeviceToggleRows(container);
  bindDevicePersonality(container);
  bindDeviceLanguage(container);

  bindDeviceAction(container, "btnDeviceReboot", "/api/reboot", getUIText("confirm_reboot", "Reboot now?"));
  bindDeviceAction(container, "btnDevicePoweroff", "/api/poweroff", getUIText("power_off_confirm", "Power off device?"));
  bindDeviceAction(container, "btnDeviceRecalib", "/api/recalibrate", getUIText("reset_calibration_confirm", "Reset calibration and reboot?"));

  const trainingButton = container.querySelector("#btnDeviceTraining");
  if (trainingButton) {
    trainingButton.addEventListener("click", async () => {
      const ok = await appConfirm(getUIText("review_training_confirm", "Are you sure you want to review the training guide?"), {
        title: getUIText("review_training_guide", "Review Training Guide"),
        confirmLabel: getUIText("review", "Review"),
      });
      if (ok) openTrainingGuide();
    });
  }

  const calibrationStatusButton = container.querySelector("#btnDeviceCalibrationStatus");
  if (calibrationStatusButton) {
    calibrationStatusButton.addEventListener("click", () => openCalibrationStatusModal());
  }

  const regulatoryButton = container.querySelector("#btnDeviceRegulatory");
  if (regulatoryButton) {
    regulatoryButton.addEventListener("click", () => {
      openRegulatoryInfo().catch((err) => {
        showAppToast(err.message || getUIText("regulatory_load_failed", "Failed to load regulatory information."), { tone: "error" });
      });
    });
  }

  const sshKeysButton = container.querySelector("#btnDeviceSshKeys");
  if (sshKeysButton) {
    sshKeysButton.addEventListener("click", () => handleSshKeysButton(sshKeysButton));
  }
}

function bindDeviceToggleRows(container) {
  container.querySelectorAll(".device-toggle__input").forEach((input) => {
    input.addEventListener("change", async (event) => {
      const toggle = event.target.closest(".device-toggle");
      const param = toggle?.dataset.param;
      if (!param) return;
      const confirmKey = toggle?.dataset.confirmKey || "";
      const confirmedParam = toggle?.dataset.confirmedParam || "";
      const alreadyConfirmed = toggle?.dataset.confirmed === "1";
      try {
        if (event.target.checked && confirmKey && !alreadyConfirmed) {
          const ok = await appConfirm(getUIText(confirmKey, ""), {
            title: getUIText("confirm_title", "Confirm"),
            confirmLabel: getUIText("enable", "Enable"),
          });
          if (!ok) {
            event.target.checked = false;
            return;
          }
          if (confirmedParam) {
            await setParam(confirmedParam, 1);
            toggle.dataset.confirmed = "1";
          }
        }
        await setParam(param, event.target.checked ? 1 : 0);
        if (event.target.checked && (param === "JoystickDebugMode" || param === "LongitudinalManeuverMode")) {
          const otherParam = param === "JoystickDebugMode" ? "LongitudinalManeuverMode" : "JoystickDebugMode";
          await setParam(otherParam, 0);
          const otherToggle = container.querySelector(`.device-toggle[data-param="${otherParam}"] .device-toggle__input`);
          if (otherToggle) otherToggle.checked = false;
        }
      } catch (err) {
        showAppToast(err.message || getUIText("failed", "Failed"), { tone: "error" });
        event.target.checked = !event.target.checked;
      }
    });
  });
}

function bindDevicePersonality(container) {
  const btnPersonality = container.querySelector("#btnDevicePersonality");
  if (!btnPersonality) return;
  btnPersonality.addEventListener("click", async () => {
    let current = 1;
    try {
      const values = await bulkGet(["LongitudinalPersonality"]);
      current = Number(values.LongitudinalPersonality ?? 1);
    } catch {}

    const currentIndex = PERSONALITY_OPTIONS.findIndex((entry) => entry.value === current);
    const nextOption = PERSONALITY_OPTIONS[(Math.max(0, currentIndex) + 1) % PERSONALITY_OPTIONS.length];
    try {
      await setParam("LongitudinalPersonality", nextOption.value);
      btnPersonality.textContent = getUIText(nextOption.labelKey, nextOption.defaultLabel);
    } catch (err) {
      showAppToast(err.message || getUIText("failed", "Failed"), { tone: "error" });
    }
  });
}

function bindDeviceLanguage(container) {
  const langSelect = container.querySelector("#deviceLanguageSelect");
  if (!langSelect) return;
  langSelect.addEventListener("change", async (event) => {
    try {
      await setParam("LanguageSetting", event.target.value);
      if (typeof setWebLanguage === "function") {
        setWebLanguage(event.target.value, { persist: true });
      }
      showAppToast(getUIText("device_lang_changed", "Language changed, reboot required."), { tone: "info" });
    } catch (err) {
      showAppToast(err.message || getUIText("failed", "Failed"), { tone: "error" });
    }
  });
}

function bindDeviceDisabledControls(container) {
  const showMessage = (event) => {
    event.preventDefault();
    event.stopPropagation();
    showAppToast(getUIText("device_only_control", "This can only be controlled on the device."), { tone: "info" });
  };

  container.querySelectorAll("[data-device-disabled=\"true\"]").forEach((el) => {
    el.addEventListener("click", showMessage);
    el.addEventListener("keydown", (event) => {
      if (event.key !== "Enter" && event.key !== " ") return;
      showMessage(event);
    });
  });
}

function bindDeviceAction(container, id, endpoint, confirmMessage = "") {
  const button = container.querySelector(`#${id}`);
  if (!button) return;
  button.addEventListener("click", async () => {
    if (confirmMessage && !confirm(confirmMessage)) return;
    try {
      await postJson(endpoint, {});
      showAppToast(getUIText("action_triggered", "Action triggered"), { tone: "info" });
    } catch (err) {
      showAppToast(err.message || getUIText("failed", "Failed"), { tone: "error" });
    }
  });
}

function getCalibrationStatusMessage() {
  const calib = deviceInfo?.calibration || {};
  let message = getUIText(
    "calibration_status_desc",
    "openpilot requires the device to be mounted within 4° left or right and within 5° up or 9° down. openpilot is continuously calibrating, resetting is rarely required.",
  );
  if (calib.calibrated) {
    message += "\n\n" + getUIText("calibration_position_desc", "Current position: pitch {pitch}°, yaw {yaw}°", {
      pitch: calib.pitch ?? "-",
      yaw: calib.yaw ?? "-",
    });
  } else {
    message += "\n\n" + getUIText("uncalibrated", "Uncalibrated");
  }
  return message;
}

function openCalibrationStatusModal() {
  appAlert(getCalibrationStatusMessage(), {
    title: getUIText("calibration_status", "Calibration Status"),
  });
}

async function handleSshKeysButton(button) {
  const hasKeys = button.dataset.hasKeys === "1";
  if (hasKeys) {
    button.disabled = true;
    try {
      await postJson("/api/ssh_keys", { action: "remove" });
      showAppToast(getUIText("ssh_keys_removed", "SSH keys removed"), { tone: "info" });
      await renderDeviceItems("Developer", false);
    } catch (err) {
      button.disabled = false;
      showAppToast(err.message || getUIText("failed", "Failed"), { tone: "error" });
    }
    return;
  }

  const username = await appPrompt(getUIText("ssh_github_username_prompt", "Enter your GitHub username"), {
    title: getUIText("ssh_keys", "SSH Keys"),
    confirmLabel: getUIText("add_upper", "ADD"),
  });
  const trimmed = String(username || "").trim();
  if (!trimmed) return;

  button.disabled = true;
  button.textContent = getUIText("loading", "Loading...");
  try {
    await postJson("/api/ssh_keys", { action: "add", username: trimmed });
    showAppToast(getUIText("ssh_keys_added", "SSH keys added"), { tone: "info" });
    await renderDeviceItems("Developer", false);
  } catch (err) {
    button.disabled = false;
    button.textContent = getUIText("add_upper", "ADD");
    await appAlert(err.message || getUIText("failed", "Failed"), {
      title: getUIText("ssh_keys", "SSH Keys"),
    });
  }
}

function openDeviceInfoModal(title, html) {
  const overlay = document.createElement("div");
  overlay.className = "training-guide-modal device-info-modal";
  overlay.innerHTML = `
    <div class="training-guide-modal__surface device-info-modal__surface" role="dialog" aria-modal="true" aria-label="${escapeHtml(title)}">
      <button type="button" class="training-guide-modal__close" data-device-info-close aria-label="${escapeHtml(getUIText("close", "Close"))}">×</button>
      <div class="device-info-modal__header">${escapeHtml(title)}</div>
      <div class="device-info-modal__body"></div>
    </div>`;

  const close = () => overlay.remove();
  overlay.querySelector(".device-info-modal__body").innerHTML = html || "";
  overlay.querySelector("[data-device-info-close]").addEventListener("click", close);
  overlay.addEventListener("click", (event) => {
    if (event.target === overlay) close();
  });
  document.body.appendChild(overlay);
}

async function openRegulatoryInfo() {
  const payload = await getJson("/api/regulatory");
  openDeviceInfoModal(getUIText("regulatory", "Regulatory"), payload.html || "");
}

function openTrainingGuide() {
  let index = 0;
  const overlay = document.createElement("div");
  overlay.className = "training-guide-modal";
  overlay.innerHTML = `
    <div class="training-guide-modal__surface" role="dialog" aria-modal="true" aria-label="${escapeHtml(getUIText("review_training_guide", "Review Training Guide"))}">
      <button type="button" class="training-guide-modal__close" data-training-close aria-label="${escapeHtml(getUIText("close", "Close"))}">×</button>
      <img class="training-guide-modal__image" alt="" />
      <div class="training-guide-modal__bar">
        <button type="button" class="smallBtn" data-training-prev>${escapeHtml(getUIText("back", "Back"))}</button>
        <div class="training-guide-modal__count"></div>
        <button type="button" class="smallBtn" data-training-next>${escapeHtml(getUIText("next", "Next"))}</button>
      </div>
    </div>`;

  const image = overlay.querySelector(".training-guide-modal__image");
  const count = overlay.querySelector(".training-guide-modal__count");
  const prev = overlay.querySelector("[data-training-prev]");
  const next = overlay.querySelector("[data-training-next]");

  const close = () => overlay.remove();
  const render = () => {
    image.src = `/training/step${index}.png`;
    count.textContent = `${index + 1} / ${TRAINING_STEP_COUNT}`;
    prev.disabled = index === 0;
    next.textContent = getUIText(index === TRAINING_STEP_COUNT - 1 ? "close" : "next", index === TRAINING_STEP_COUNT - 1 ? "Close" : "Next");
  };

  overlay.querySelector("[data-training-close]").addEventListener("click", close);
  prev.addEventListener("click", () => {
    index = Math.max(0, index - 1);
    render();
  });
  next.addEventListener("click", () => {
    if (index >= TRAINING_STEP_COUNT - 1) {
      close();
      return;
    }
    index += 1;
    render();
  });
  overlay.addEventListener("click", (event) => {
    if (event.target === overlay) close();
  });

  render();
  document.body.appendChild(overlay);
}
