"use strict";

import { el, setHidden, setText } from "../dom.js";

export function createSdiAlert(doc) {
  const label = el(doc, "span", { class: "chud-sdi-label" });
  const speed = el(doc, "span", { class: "chud-sdi-speed" });
  const unit = el(doc, "span", { class: "chud-sdi-unit" });
  const distance = el(doc, "span", { class: "chud-sdi-distance" });
  const countdown = el(doc, "span", { class: "chud-sdi-countdown" });
  const root = el(doc, "div", {
    class: "chud-sdi",
    attrs: { role: "status", "aria-live": "off" },
  }, [
    label,
    el(doc, "div", { class: "chud-sdi-value" }, [speed, unit]),
    el(doc, "div", { class: "chud-sdi-meta" }, [distance, countdown]),
  ]);

  function update(data = {}) {
    const alert = data.sdiAlert;
    const type = Number(alert?.type);
    const hasAlert = alert != null && Number.isFinite(type) && type >= 0;
    setHidden(root, !hasAlert);
    if (!hasAlert) return;

    root.setAttribute("data-family", String(alert.family || "camera"));
    root.classList.toggle("is-imminent", Number(alert.countdownS) > 0 && Number(alert.countdownS) <= 10);
    setText(label, alert.label || "CAM");

    const hasSpeed = Number.isFinite(Number(alert.speedLimit)) && Number(alert.speedLimit) > 0;
    setHidden(speed, !hasSpeed);
    setHidden(unit, !hasSpeed);
    setText(speed, hasSpeed ? Math.round(Number(alert.speedLimit)) : "");
    setText(unit, hasSpeed ? String(alert.speedUnit || "") : "");

    const hasDistance = Boolean(alert.distanceText);
    const hasCountdown = Number.isFinite(Number(alert.countdownS))
      && Number(alert.countdownS) > 0
      && Number(alert.countdownS) < 100;
    setHidden(distance, !hasDistance);
    setHidden(countdown, !hasCountdown);
    setText(distance, hasDistance ? alert.distanceText : "");
    setText(countdown, hasCountdown ? `${Math.round(Number(alert.countdownS))}s` : "");

    const spoken = [
      alert.label || "CAM",
      hasSpeed ? `${Math.round(Number(alert.speedLimit))} ${alert.speedUnit || ""}` : "",
      hasDistance ? alert.distanceText : "",
      hasCountdown ? `${Math.round(Number(alert.countdownS))} seconds` : "",
    ].filter(Boolean).join(", ");
    root.setAttribute("aria-label", spoken);
  }

  update();
  return { el: root, update };
}
