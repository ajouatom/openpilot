"use strict";

const DEVICE_NETWORK_REFRESH_MS = 3000;

const DEVICE_GROUPS = [
  { id: "Device", labelKey: "device_group_info", defaultLabel: "Device Info" },
  { id: "Network", labelKey: "device_group_network", defaultLabel: "Network" },
  { id: "Toggles", labelKey: "device_group_toggles", defaultLabel: "Toggles" },
  { id: "Software", labelKey: "device_group_software", defaultLabel: "Software" },
  { id: "Developer", labelKey: "device_group_developer", defaultLabel: "Developer" },
];

const DEVICE_INFO_PARAMS = [
  "DeviceType",
  "DongleId",
  "HardwareSerial",
  "LanguageSetting",
  "SoftwareMenu",
];

const DEVICE_SOFTWARE_PARAMS = [
  "UpdaterCurrentDescription",
  "UpdaterState",
  "UpdateAvailable",
  "UpdaterFetchAvailable",
  "UpdateFailedCount",
  "UpdaterTargetBranch",
  "GitBranch",
  "UpdaterAvailableBranches",
  "LastUpdateTime",
  "UpdaterNewDescription",
];

const DEVICE_LANGUAGES = [
  { code: "main_en", name: "English" },
  { code: "main_ko", name: "한국어" },
  { code: "main_zh-CHS", name: "简体中文" },
  { code: "main_zh-CHT", name: "繁體中文" },
  { code: "main_ja", name: "日本語" },
  { code: "main_fr", name: "Français" },
  { code: "main_pt-BR", name: "Português" },
  { code: "main_de", name: "Deutsch" },
  { code: "main_es", name: "Español" },
  { code: "main_tr", name: "Türkçe" },
  { code: "main_th", name: "ไทย" },
  { code: "main_ar", name: "العربية" },
  { code: "main_pl", name: "Polski" },
  { code: "main_nl", name: "Nederlands" },
];

const DEVICE_TOGGLES = [
  { param: "OpenpilotEnabledToggle", labelKey: "enable_openpilot", defaultLabel: "Enable openpilot" },
  { param: "ExperimentalMode", labelKey: "experimental_mode", defaultLabel: "Experimental Mode", confirmKey: "experimental_mode_confirm", confirmedParam: "ExperimentalModeConfirmed" },
  { param: "DisengageOnAccelerator", labelKey: "disengage_on_accelerator", defaultLabel: "Disengage on Accelerator" },
  { param: "IsLdwEnabled", labelKey: "enable_ldw", defaultLabel: "Enable Lane Departure Warnings" },
  { param: "AlwaysOnDM", labelKey: "always_on_dm", defaultLabel: "Always-on DM" },
  { param: "RecordFront", labelKey: "record_front", defaultLabel: "Record and Upload Driver Camera" },
  { param: "RecordAudio", labelKey: "record_audio", defaultLabel: "Record and Upload Microphone Audio" },
  { param: "IsMetric", labelKey: "is_metric", defaultLabel: "Use Metric System" },
];

const DEVICE_DEVELOPER_TOGGLES = [
  { param: "AdbEnabled", labelKey: "enable_adb", defaultLabel: "Enable ADB", confirmKey: "adb_enable_confirm" },
  { param: "SshEnabled", labelKey: "enable_ssh", defaultLabel: "Enable SSH" },
  { param: "JoystickDebugMode", labelKey: "joystick_debug_mode", defaultLabel: "Joystick Debug Mode", disabled: true },
  { param: "LongitudinalManeuverMode", labelKey: "longitudinal_maneuver_mode", defaultLabel: "Longitudinal Maneuver Mode" },
  { param: "AlphaLongitudinalEnabled", labelKey: "alpha_longitudinal_control", defaultLabel: "openpilot Longitudinal Control (Alpha)", confirmKey: "alpha_longitudinal_confirm" },
];

const PERSONALITY_OPTIONS = [
  { value: 0, labelKey: "aggressive", defaultLabel: "Aggressive" },
  { value: 1, labelKey: "standard", defaultLabel: "Standard" },
  { value: 2, labelKey: "relaxed", defaultLabel: "Relaxed" },
  { value: 3, labelKey: "more_relaxed", defaultLabel: "MoreRelaxed" },
];

const TRAINING_STEP_COUNT = 19;

function isTruthyDeviceFlag(value) {
  return value === true || value === 1 || value === "1" || value === "true" || value === "True";
}
