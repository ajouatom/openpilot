"use strict";

// Neutral contract shared by the web-settings component and Drive Workspace.
// Persisted defaults and validation remain owned by services/web_settings.py.
globalThis.CarrotDriveLayoutSpec = (() => {
  const ORIENTATION = Object.freeze({
    HORIZONTAL: "horizontal",
    VERTICAL: "vertical",
  });
  const MODE = Object.freeze({
    SPLIT: "split",
    AREA_1: "area_1",
    AREA_2: "area_2",
  });
  const CONTENT = Object.freeze({
    VISION: "vision",
    NAVIGATION: "navigation",
  });
  const SETTINGS = Object.freeze({
    [ORIENTATION.HORIZONTAL]: Object.freeze({
      mode: "carrot_navi_horizontal_mode",
      area1Content: "carrot_navi_horizontal_area_1",
      area2Content: "carrot_navi_horizontal_area_2",
      ratio: "carrot_navi_split_ratio",
    }),
    [ORIENTATION.VERTICAL]: Object.freeze({
      mode: "carrot_navi_vertical_mode",
      area1Content: "carrot_navi_vertical_area_1",
      area2Content: "carrot_navi_vertical_area_2",
      ratio: "carrot_navi_vertical_split_ratio",
    }),
  });
  const settingKeys = Object.freeze(Object.values(SETTINGS).flatMap((entry) => Object.values(entry)));

  function normalizeOrientation(value) {
    return String(value || "").toLowerCase() === ORIENTATION.VERTICAL
      ? ORIENTATION.VERTICAL
      : ORIENTATION.HORIZONTAL;
  }

  function settingValue(settings, key) {
    if (Object.prototype.hasOwnProperty.call(settings || {}, key)) return settings[key];
    return globalThis.CarrotWebSettingDefaults?.[key];
  }

  function normalizeMode(value) {
    return Object.values(MODE).includes(value) ? value : MODE.SPLIT;
  }

  function normalizeContent(value) {
    return Object.values(CONTENT).includes(value) ? value : CONTENT.VISION;
  }

  function normalizeRatio(value) {
    const numeric = Number(value);
    const ratio = Number.isFinite(numeric) ? numeric : 0.5;
    const normalized = Math.max(0.3, Math.min(0.7, Math.round(ratio / 0.05) * 0.05));
    return Number(normalized.toFixed(2));
  }

  function otherContent(value) {
    return normalizeContent(value) === CONTENT.VISION ? CONTENT.NAVIGATION : CONTENT.VISION;
  }

  function keysFor(value) {
    return SETTINGS[normalizeOrientation(value)];
  }

  function read(settings, value) {
    const orientation = normalizeOrientation(value);
    const keys = keysFor(orientation);
    const area1Content = normalizeContent(settingValue(settings, keys.area1Content));
    const configuredArea2Content = normalizeContent(settingValue(settings, keys.area2Content));
    const area2Content = configuredArea2Content === area1Content
      ? otherContent(area1Content)
      : configuredArea2Content;
    return Object.freeze({
      orientation,
      mode: normalizeMode(settingValue(settings, keys.mode)),
      ratio: normalizeRatio(settingValue(settings, keys.ratio)),
      area1Content,
      area2Content,
    });
  }

  return Object.freeze({
    ORIENTATION,
    MODE,
    CONTENT,
    SETTINGS,
    settingKeys,
    normalizeOrientation,
    normalizeRatio,
    otherContent,
    keysFor,
    read,
  });
})();
