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

  function displayGap(value) {
    const number = integer(value);
    return number != null && number >= 1 && number <= 9 ? String(number) : "";
  }

  function displayGear(value) {
    const gear = String(value || "").trim().toUpperCase();
    if (!gear || gear === "UNKNOWN") return "";
    const aliases = { PARK: "P", REVERSE: "R", NEUTRAL: "N", DRIVE: "D", SPORT: "S", LOW: "L" };
    if (aliases[gear]) return aliases[gear];
    return gear.slice(0, 2);
  }

  function alertDescriptor(type) {
    if (type === 100) return { name: "POLICE", kind: "police" };
    if (type === 22) return { name: "BUMP", kind: "bump" };
    if (SECTION_TYPES.has(type)) return { name: "SECTION", kind: "section" };
    if (CAMERA_TYPES.has(type)) return { name: "CAM", kind: "camera" };
    return { name: "ALERT", kind: "alert" };
  }

  function sourceMode(carrotMan) {
    // Waze cannot be reliably distinguished from other nav providers in normal
    // driving (CarrotMan carries no provider field; only xSpdType 100/101 and
    // desiredSource "waze"/"police" hint at it, and only during an active alert).
    // So the compact HUD merges every non-stock nav source into a single "nav".
    const type = integer(carrotMan?.xSpdType, -1);
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
    // carrotMan reaches us on two paths: the raw HUD stream (rawState, ~20Hz,
    // realtime) and the /api/live_runtime poll (liveState, slow). The raw stream
    // carries the fast-changing fields (desiredSpeed / desiredSource / xSpdType /
    // nRoadLimitSpeed / activeCarrot), so it must WIN to keep temp + road limit as
    // live as the green drive HUD (which reads raw only). Live is kept underneath
    // only to fill fields the raw schema omits — xSpdDist / xSpdCountDown, i.e. the
    // alert distance & countdown. Merging (raw over live) instead of picking one
    // was the fix for temp appearing frozen between the slow poll ticks.
    const liveCarrotMan = liveState?.services?.carrotMan;
    const rawCarrotMan = rawState?.carrotMan;
    const liveBase = liveCarrotMan && typeof liveCarrotMan === "object" ? liveCarrotMan : null;
    const rawFast = rawCarrotMan && typeof rawCarrotMan === "object" ? rawCarrotMan : null;
    const carrotMan = (liveBase || rawFast) ? { ...(liveBase || {}), ...(rawFast || {}) } : {};
    const isMetric = payload?.isMetric !== false;
    const source = sourceMode(carrotMan);
    const alertType = integer(carrotMan?.xSpdType, -1);
    const alert = alertDescriptor(alertType);
    const alertDistance = finite(carrotMan?.xSpdDist);
    const countdown = integer(carrotMan?.xSpdCountDown, 0);
    const roadLimitKph = finite(carrotMan?.nRoadLimitSpeed) ?? finite(payload?.speedLimitKph);
    const alertVisible = source !== "stock" && alertType >= 0 && ((alertDistance ?? 0) > 0 || WAZE_TYPES.has(alertType));
    const gearStep = integer(payload?.gearStep);

    // Active speed-control source line ("road 55" etc). We read desiredSource
    // straight from carrotMan (not payload.temp, which blanks the label while
    // decelerating) so the compact HUD always shows the winning source label.
    const desiredSpeed = finite(carrotMan?.desiredSpeed);
    const vSetKph = finite(payload?.vSetKph);
    const tempVisible = source !== "stock" && desiredSpeed != null && desiredSpeed > 0;

    return {
      source,
      isMetric,
      // metric → Korean/Vienna red circle, imperial → US MUTCD rectangle.
      limitStyle: isMetric ? "kr" : "us",
      // Hottest core, same reading as the vision HUD corner bar and the native
      // cluster. Falls back to the averaged field on payloads that lack it.
      cpu: integer(payload?.cpuTempMaxC ?? payload?.cpuTempC),
      speed: displaySpeed(payload?.vEgoKph, isMetric),
      setSpeed: displaySpeed(payload?.vSetKph, isMetric),
      roadLimit: displayLimit(roadLimitKph, isMetric),
      gap: displayGap(payload?.tfGap ?? payload?.tfBars),
      temp: {
        visible: tempVisible,
        label: tempVisible ? String(carrotMan?.desiredSource || "").trim() : "",
        speed: tempVisible ? displaySpeed(desiredSpeed, isMetric) : "",
        decel: tempVisible && vSetKph != null && desiredSpeed < vSetKph,
      },
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
      gear: displayGear(payload?.gear),
      gearStep: gearStep != null && gearStep >= 1 && gearStep <= 7 ? gearStep : null,
    };
  }

  window.CarrotMiniHudModel = { build, displayDistance, displaySpeed, displayGap, displayGear };
})();
