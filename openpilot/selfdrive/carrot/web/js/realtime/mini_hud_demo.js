"use strict";

// Compact-HUD view tester. Active only with ?mhud_test=1 in the URL (which also
// makes mini_hud_mode force the HUD on). It renders mock *models* straight into
// CarrotMiniHud so the layout/styling can be checked with no vehicle connected —
// tap the HUD to step to the next scenario; after a short idle it clears back to
// a plain STOCK view. Without the param the guard below no-ops immediately.
//
// Scenarios mirror the LIVE build (CarrotMiniHudModel.build) 1:1, so previews
// match what actually renders on the road:
//   • detail labels always read TIM / DST / GAP / <temp source> at full length
//     (never collapse to one letter — see mini_hud.js syncDetailLabels);
//   • the 4 detail rows re-order by source (mini_hud.css): STOCK keeps
//     countdown · distance · GAP · temp, while NAV lifts temp above GAP →
//     countdown · distance · temp · GAP;
//   • temp.label uses the real carrotMan.desiredSource tokens emitted by
//     carrot_serv.py: road / cam / police / section / bump / waze / hda / atc /
//     route / vturn / model / gas (rendered upper-cased, first 3 chars).
(function () {
  if (!new URLSearchParams(window.location.search).has("mhud_test")) return;

  // Base model matching CarrotMiniHudModel.build()'s output shape; scenarios
  // override only the fields they exercise.
  function model(over) {
    return Object.assign({
      source: "nav",
      isMetric: true,
      limitStyle: "kr",
      cpu: 58,
      speed: "0",
      setSpeed: "0",
      roadLimit: "--",
      gap: "3",
      temp: { visible: false, label: "", speed: "", decel: false },
      alert: { visible: false, name: "", kind: "none", distance: "", countdown: "", badge: "", section: false },
      driveMode: { kind: "normal", name: "NORMAL" },
      gear: "D",
      gearStep: null,
    }, over || {});
  }

  const STOCK = model({
    source: "stock", cpu: 55, speed: "48", setSpeed: "60", gap: "2", gearStep: 4,
  });

  const SCENARIOS = [
    { name: "STOCK", model: STOCK },
    { name: "LIVE / 순정", live: true, model: STOCK },
    {
      name: "NAV road (KR)",
      model: model({
        speed: "77", setSpeed: "80", roadLimit: "70", gap: "3", gearStep: 6,
        temp: { visible: true, label: "road", speed: "80", decel: false },
      }),
    },
    {
      name: "CAM alert",
      model: model({
        speed: "72", setSpeed: "80", roadLimit: "70", gap: "3",
        alert: { visible: true, name: "CAM", kind: "camera", distance: "320m", countdown: "8s", badge: "", section: false },
        temp: { visible: true, label: "cam", speed: "70", decel: true },
      }),
    },
    {
      name: "POLICE",
      model: model({
        speed: "64", setSpeed: "80", roadLimit: "70",
        alert: { visible: true, name: "POLICE", kind: "police", distance: "150m", countdown: "", badge: "", section: false },
        temp: { visible: true, label: "police", speed: "70", decel: true },
      }),
    },
    {
      name: "SECTION",
      model: model({
        speed: "78", setSpeed: "90", roadLimit: "80", gap: "4",
        alert: { visible: true, name: "SECTION", kind: "section", distance: "1.2km", countdown: "", badge: "1.2km", section: true },
        temp: { visible: true, label: "section", speed: "80", decel: true },
      }),
    },
    {
      name: "BUMP",
      model: model({
        speed: "40", setSpeed: "60", roadLimit: "50",
        alert: { visible: true, name: "BUMP", kind: "bump", distance: "60m", countdown: "", badge: "", section: false },
        temp: { visible: true, label: "bump", speed: "30", decel: true },
      }),
    },
    {
      name: "temp decel",
      model: model({
        speed: "58", setSpeed: "60", roadLimit: "60",
        temp: { visible: true, label: "vturn", speed: "45", decel: true },
      }),
    },
    {
      name: "US limit",
      model: model({
        isMetric: false, limitStyle: "us", cpu: 60, speed: "65", setSpeed: "70", roadLimit: "55",
        temp: { visible: true, label: "road", speed: "70", decel: false },
      }),
    },
    {
      name: "3-digit",
      model: model({
        speed: "105", setSpeed: "120", roadLimit: "110", gap: "2",
        temp: { visible: true, label: "road", speed: "120", decel: false },
      }),
    },
    {
      name: "NAV · all rows (TIM/DST/temp/GAP)",
      model: model({
        speed: "82", setSpeed: "90", roadLimit: "80", gap: "3",
        alert: { visible: true, name: "CAM", kind: "camera", distance: "480m", countdown: "12s", badge: "", section: false },
        temp: { visible: true, label: "road", speed: "90", decel: false },
      }),
    },
    {
      name: "ATC turn",
      model: model({
        speed: "60", setSpeed: "80", roadLimit: "70",
        temp: { visible: true, label: "atc", speed: "45", decel: true },
      }),
    },
    {
      name: "route curve",
      model: model({
        speed: "72", setSpeed: "90", roadLimit: "90",
        temp: { visible: true, label: "route", speed: "58", decel: true },
      }),
    },
    {
      name: "gas override",
      model: model({
        speed: "95", setSpeed: "80", roadLimit: "80",
        temp: { visible: true, label: "gas", speed: "95", decel: false },
      }),
    },
    {
      name: "HDA",
      model: model({
        speed: "98", setSpeed: "100", roadLimit: "100",
        temp: { visible: true, label: "hda", speed: "100", decel: false },
      }),
    },
    {
      name: "road only (no temp)",
      model: model({
        speed: "55", setSpeed: "60", roadLimit: "60",
      }),
    },
    {
      name: "US · all rows",
      model: model({
        isMetric: false, limitStyle: "us", speed: "62", setSpeed: "70", roadLimit: "55",
        alert: { visible: true, name: "CAM", kind: "camera", distance: "0.3mi", countdown: "9s", badge: "", section: false },
        temp: { visible: true, label: "road", speed: "70", decel: false },
      }),
    },
    {
      name: "US 3-digit",
      model: model({
        isMetric: false, limitStyle: "us", speed: "105", setSpeed: "110", roadLimit: "75",
        temp: { visible: true, label: "road", speed: "110", decel: false },
      }),
    },
  ];

  let tag = null;
  let idx = 0; // 0 = STOCK, shown at init/idle; first tap advances to scenario 1
  let showing = false; // a mock scenario is on screen → drop live HUD updates
  let currentModel = STOCK;
  const MODE_PREVIEWS = [
    { kind: "normal", name: "NORMAL" },
    { kind: "eco", name: "ECO" },
    { kind: "safe", name: "SAFE" },
    { kind: "sport", name: "SPORT" },
  ];

  function render(m) {
    currentModel = m;
    window.CarrotMiniHud?.render?.(m);
  }

  function cycleDriveMode(event) {
    event.preventDefault();
    event.stopPropagation();
    const currentKind = String(currentModel?.driveMode?.kind || "normal").toLowerCase();
    const currentIndex = MODE_PREVIEWS.findIndex((mode) => mode.kind === currentKind);
    const nextMode = MODE_PREVIEWS[(currentIndex + 1) % MODE_PREVIEWS.length];
    showing = true;
    render(Object.assign({}, currentModel, { source: "stock", driveMode: nextMode }));
    const t = ensureTag();
    if (t) {
      t.hidden = false;
      t.textContent = `MODE - ${nextMode.name}`;
    }
  }

  function ensureTag() {
    const root = document.getElementById("carrotMiniHud");
    if (!root) return null;
    if (!tag || !tag.isConnected) {
      tag = document.createElement("div");
      tag.className = "carrot-mini-hud__demo-tag";
      tag.hidden = true;
      root.appendChild(tag);
    }
    return tag;
  }

  // Idle timeout — release the surface back to the live feed and show plain
  // STOCK (option 가). With no vehicle connected the STOCK render just stays.
  function stop() {
    idx = 0;
    showing = false;
    render(STOCK);
    const t = ensureTag();
    if (t) t.hidden = true;
  }

  function step() {
    idx = (idx + 1) % SCENARIOS.length;
    const scenario = SCENARIOS[idx];
    showing = !scenario.live;
    render(scenario.model);
    const t = ensureTag();
    if (t) {
      t.hidden = false;
      t.textContent = `TEST ${idx + 1}/${SCENARIOS.length} · ${scenario.name}`;
    }
    if (t) t.textContent = `${scenario.live ? "LIVE" : "TEST"} ${idx + 1}/${SCENARIOS.length} - ${scenario.name}`;
  }

  function init() {
    if (!window.CarrotMiniHud) {
      setTimeout(init, 60);
      return;
    }
    // Take ownership of the surface: while a mock scenario is on screen, drop the
    // live HUD feed (vision_raw calls CarrotMiniHud.update ~10Hz) so it can't
    // overwrite the mock with the parked/STOCK live state.
    const liveUpdate = window.CarrotMiniHud.update;
    if (typeof liveUpdate === "function" && !window.CarrotMiniHud.__demoWrapped) {
      window.CarrotMiniHud.update = function (m) {
        if (showing) return;
        return liveUpdate.call(this, m);
      };
      window.CarrotMiniHud.__demoWrapped = true;
    }
    render(STOCK);
    const root = document.getElementById("carrotMiniHud");
    if (root) root.addEventListener("click", step, { passive: true });
    const driveMode = root?.querySelector(".carrot-mini-hud__drive-mode");
    if (driveMode) {
      driveMode.classList.add("is-demo-interactive");
      driveMode.setAttribute("role", "button");
      driveMode.setAttribute("tabindex", "0");
      driveMode.setAttribute("aria-label", "Preview next drive mode");
      driveMode.addEventListener("click", cycleDriveMode);
      driveMode.addEventListener("keydown", (event) => {
        if (event.key === "Enter" || event.key === " ") cycleDriveMode(event);
      });
    }
    console.log("[mini_hud] view-test mode ready — tap the HUD to step scenarios");
  }

  if (document.readyState === "loading") {
    document.addEventListener("DOMContentLoaded", init);
  } else {
    init();
  }
})();
