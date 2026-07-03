"use strict";

(function () {
  const WAZE_TYPES = new Set([100, 101]);
  const SECTION_TYPES = new Set([2, 3, 4]);
  const CAMERA_TYPES = new Set([0, 1, 7, 8, 75, 76, 101]);

  function finite(value) {
    const number = Number(value);
    return Number.isFinite(number) ? number : null;
  }

  function integer(value, fallback = null) {
    const number = finite(value);
    return number == null ? fallback : Math.round(number);
  }

  function displaySpeed(kph, isMetric) {
    const value = finite(kph);
    if (value == null || value < 0 || value >= 250) return "--";
    return String(Math.round(isMetric ? value : value * 0.621371));
  }

  function displayLimit(kph, isMetric) {
    const value = finite(kph);
    return value != null && value > 0 ? displaySpeed(value, isMetric) : "--";
  }

  function displayDistance(meters, isMetric) {
    const value = finite(meters);
    if (value == null || value <= 0) return "";
    if (!isMetric) {
      const feet = value * 3.28084;
      if (feet < 1000) return `${Math.round(feet / 10) * 10}ft`;
      return `${(feet / 5280).toFixed(feet >= 5280 ? 1 : 2)}mi`;
    }
    if (value < 1000) return `${Math.round(value)}m`;
    return `${(value / 1000).toFixed(value >= 10000 ? 1 : 2)}km`;
  }

  function alertDescriptor(type) {
    if (type === 100) return { name: "POLICE", kind: "police" };
    if (type === 22) return { name: "BUMP", kind: "bump" };
    if (SECTION_TYPES.has(type)) return { name: "SECTION", kind: "section" };
    if (CAMERA_TYPES.has(type)) return { name: "CAM", kind: "camera" };
    return { name: "ALERT", kind: "alert" };
  }

  function sourceMode(carrotMan) {
    const type = integer(carrotMan?.xSpdType, -1);
    const desiredSource = String(carrotMan?.desiredSource || "").trim().toLowerCase();
    if (WAZE_TYPES.has(type) || desiredSource === "police" || desiredSource === "waze") return "waze";
    if (type >= 0) return "nav";
    if (integer(carrotMan?.activeCarrot, 0) > 1) return "nav";
    return "stock";
  }

  function driveMode(payload) {
    const rawKind = String(payload?.driveMode?.kind || "normal").trim().toLowerCase();
    const kind = ["normal", "eco", "safe", "sport"].includes(rawKind) ? rawKind : "normal";
    return { kind, name: kind.toUpperCase() };
  }

  function build(payload, rawState, liveState) {
    const liveCarrotMan = liveState?.services?.carrotMan;
    const rawCarrotMan = rawState?.carrotMan;
    const carrotMan = liveCarrotMan && typeof liveCarrotMan === "object" ? liveCarrotMan : (rawCarrotMan || {});
    const isMetric = payload?.isMetric !== false;
    const source = sourceMode(carrotMan);
    const alertType = integer(carrotMan?.xSpdType, -1);
    const alert = alertDescriptor(alertType);
    const alertDistance = finite(carrotMan?.xSpdDist);
    const countdown = integer(carrotMan?.xSpdCountDown, 0);
    const roadLimitKph = finite(carrotMan?.nRoadLimitSpeed) ?? finite(payload?.speedLimitKph);
    const alertVisible = source !== "stock" && alertType >= 0 && ((alertDistance ?? 0) > 0 || WAZE_TYPES.has(alertType));
    const gearStep = integer(payload?.gearStep);

    return {
      source,
      isMetric,
      cpu: integer(payload?.cpuTempC),
      speed: displaySpeed(payload?.vEgoKph, isMetric),
      setSpeed: displaySpeed(payload?.vSetKph, isMetric),
      roadLimit: displayLimit(roadLimitKph, isMetric),
      alert: {
        visible: alertVisible,
        name: alertVisible ? alert.name : "",
        kind: alertVisible ? alert.kind : "none",
        distance: alertVisible ? displayDistance(alertDistance, isMetric) : "",
        countdown: alertVisible && countdown > 0 ? `${countdown}s` : "",
        badge: alertVisible && alert.kind === "section" ? displayDistance(alertDistance, isMetric) : "",
        section: alertVisible && alert.kind === "section",
      },
      driveMode: driveMode(payload),
      gearStep: gearStep != null && gearStep >= 1 && gearStep <= 7 ? gearStep : null,
    };
  }

  window.CarrotMiniHudModel = { build, displayDistance, displaySpeed };
})();
