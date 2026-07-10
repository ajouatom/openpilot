"use strict";
// xterm's WebGL/Canvas addons (0.16.0 / 0.5.0) read the DEPRECATED
// navigator.platform (and navigator.userAgent) at module load time and call
// String.prototype.indexOf / includes on them. Some device browsers and
// WebViews return undefined for navigator.platform, so `undefined.indexOf(...)`
// throws before the addon registers its global (window.WebglAddon /
// window.CanvasAddon) — which silently left us on the slow DOM renderer.
//
// Define harmless empty strings when those fields are missing so the addons can
// finish loading. Must run BEFORE the addon <script> tags. A real value (any
// non-null string) is left untouched.
(function () {
  if (typeof navigator === "undefined") return;
  try {
    if (navigator.platform == null) {
      Object.defineProperty(navigator, "platform", { value: "", configurable: true });
    }
  } catch (e) { /* navigator.platform is a locked getter; nothing we can do */ }
  try {
    if (navigator.userAgent == null) {
      Object.defineProperty(navigator, "userAgent", { value: "", configurable: true });
    }
  } catch (e) { /* leave as-is */ }
})();
