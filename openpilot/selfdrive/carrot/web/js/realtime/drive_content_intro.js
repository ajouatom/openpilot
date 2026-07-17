"use strict";

// Shared empty/idle surface for Drive Workspace content. Feature runtimes own
// their state and translated copy; this component only owns safe DOM and UI.
globalThis.DriveContentIntro = (() => {
  const instances = new WeakMap();

  function text(value) {
    return String(value ?? "").trim();
  }

  function create(options = {}) {
    const host = options.host;
    if (!host || typeof host.append !== "function") return null;
    const existing = instances.get(host);
    if (existing) return existing;

    const root = document.createElement("section");
    root.className = ["drive-content-intro", text(options.className)].filter(Boolean).join(" ");
    root.hidden = true;
    root.setAttribute("role", "status");
    root.setAttribute("aria-live", "polite");
    root.setAttribute("aria-atomic", "true");
    root.setAttribute("aria-hidden", "true");

    const body = document.createElement("div");
    body.className = "drive-content-intro__body";
    const label = document.createElement("span");
    label.className = "drive-content-intro__label";
    const title = document.createElement("strong");
    title.className = "drive-content-intro__title";
    const detail = document.createElement("span");
    detail.className = "drive-content-intro__detail";
    body.append(label, title, detail);
    root.append(body);
    host.append(root);

    let state = "hidden";
    let signature = "";
    let enterFrame = 0;
    let hideTimer = 0;

    function clearMotionJobs() {
      if (enterFrame) cancelAnimationFrame(enterFrame);
      if (hideTimer) clearTimeout(hideTimer);
      enterFrame = 0;
      hideTimer = 0;
    }

    function timeMs(value) {
      const item = String(value || "").trim();
      const number = Number.parseFloat(item);
      if (!Number.isFinite(number)) return 0;
      return item.endsWith("ms") ? number : number * 1000;
    }

    function transitionMs(element) {
      const style = getComputedStyle(element);
      const durations = style.transitionDuration.split(",").map(timeMs);
      const delays = style.transitionDelay.split(",").map(timeMs);
      return durations.reduce((longest, duration, index) => (
        Math.max(longest, duration + (delays[index % delays.length] || 0))
      ), 0);
    }

    function finishHide() {
      if (state !== "hidden") return;
      clearMotionJobs();
      root.hidden = true;
      delete root.dataset.motion;
      delete root.dataset.state;
    }

    function finishHideOnTransition(event) {
      if (event.target === root && event.propertyName === "opacity") finishHide();
    }

    root.addEventListener("transitionend", finishHideOnTransition);

    function show(view = {}) {
      const nextState = text(view.state) || "waiting";
      const nextLabel = text(view.label);
      const nextTitle = text(view.title);
      const nextDetail = text(view.detail);
      const nextBusy = Boolean(view.busy);
      const nextSignature = [nextState, nextLabel, nextTitle, nextDetail, nextBusy].join("\u0000");
      if (state !== "hidden" && signature === nextSignature) return false;
      const shouldEnter = root.hidden || state === "hidden";
      clearMotionJobs();
      state = nextState;
      signature = nextSignature;
      root.dataset.state = nextState;
      root.hidden = false;
      root.setAttribute("aria-hidden", "false");
      root.setAttribute("aria-busy", String(nextBusy));
      label.textContent = nextLabel;
      label.hidden = !nextLabel;
      title.textContent = nextTitle;
      title.hidden = !nextTitle;
      detail.textContent = nextDetail;
      detail.hidden = !nextDetail;
      if (shouldEnter) {
        root.dataset.motion = "entering";
        void root.offsetWidth;
        enterFrame = requestAnimationFrame(() => {
          enterFrame = 0;
          if (state !== "hidden") root.dataset.motion = "visible";
        });
      } else {
        root.dataset.motion = "visible";
      }
      return true;
    }

    function hide() {
      if (state === "hidden") return false;
      clearMotionJobs();
      state = "hidden";
      root.setAttribute("aria-hidden", "true");
      root.setAttribute("aria-busy", "false");
      root.dataset.motion = "leaving";
      const duration = Math.max(transitionMs(root), transitionMs(body));
      if (duration <= 0) finishHide();
      else hideTimer = window.setTimeout(finishHide, duration + 50);
      return true;
    }

    function snapshot() {
      return Object.freeze({ state, visible: state !== "hidden" });
    }

    function destroy() {
      clearMotionJobs();
      root.removeEventListener("transitionend", finishHideOnTransition);
      instances.delete(host);
      root.remove();
    }

    const api = Object.freeze({ show, hide, snapshot, destroy });
    instances.set(host, api);
    return api;
  }

  return Object.freeze({ create });
})();
