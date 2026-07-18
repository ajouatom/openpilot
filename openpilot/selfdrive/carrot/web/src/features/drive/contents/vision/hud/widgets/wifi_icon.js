"use strict";

import { WIFI_URI } from "../assets.js";
import { svg } from "../dom.js";

export function createWifiIcon(doc) {
  const root = svg(doc, "svg", {
    class: "chud-wifi", viewBox: "0 0 48 48", role: "img", "aria-label": "네트워크",
  });
  root.append(svg(doc, "image", {
    href: WIFI_URI, x: 0, y: 0, width: 48, height: 48, preserveAspectRatio: "none",
  }));

  function update(data = {}) {
    root.style.opacity = data.networkConnected === false ? "0.51" : "1";
  }

  return { el: root, update };
}
