"use strict";

// Confetti easter egg — random sparkle on pointer-down + carrot:run:404 trigger.
// Carries over from app_pages.js as-is (intentionally obfuscated).
(() => {
  const $ = (...v) => String.fromCharCode(...v);
  const R = window.matchMedia?.($(
    40, 112, 114, 101, 102, 101, 114, 115, 45, 114, 101, 100, 117, 99, 101, 100, 45, 109, 111, 116, 105, 111, 110, 58, 32, 114, 101, 100, 117, 99, 101, 41,
  ))?.matches;
  if (R) return;

  const U = $(47, 97, 115, 115, 101, 116, 115, 47, 105, 109, 103, 95, 99, 104, 102, 102, 114, 95, 119, 104, 101, 101, 108, 46, 112, 110, 103, 63, 118, 61, 50, 54, 48, 52, 45, 48, 49);
  const M = new Image();
  M.src = U;
  let C = null;
  let X = null;
  let P = [];
  let A = 0;
  let D = 1;
  let T = performance.now();
  let W = null;
  let Q = 0.01;
  const N = 30;
  const K = 0.81;
  const Z = 2147482600;
  const E = $(99, 97, 114, 114, 111, 116, 58, 114, 117, 110, 58, 52, 48, 52);
  const B = (l, h) => l + Math.random() * (h - l);

  function S() {
    if (C) return;
    C = document.createElement($(99, 97, 110, 118, 97, 115));
    C.setAttribute($(97, 114, 105, 97, 45, 104, 105, 100, 100, 101, 110), $(116, 114, 117, 101));
    C.style.cssText = `position:fixed;inset:0;width:100vw;height:100vh;pointer-events:none;z-index:${Z}`;
    document.body.appendChild(C);
    X = C.getContext($(
      50, 100,
    ));
    V();
    addEventListener($(114, 101, 115, 105, 122, 101), V, { passive: true });
  }

  function V() {
    if (!C || !X) return;
    D = Math.max(1, Math.min(2.5, devicePixelRatio || 1));
    const w = Math.max(1, innerWidth || 1);
    const h = Math.max(1, innerHeight || 1);
    C.width = Math.round(w * D);
    C.height = Math.round(h * D);
    X.setTransform(D, 0, 0, D, 0, 0);
  }

  function O(px, py, o) {
    const sp = Number.isFinite(o.spread) ? o.spread : 45;
    const an = Number.isFinite(o.angle) ? o.angle : 90;
    const ve = (Number.isFinite(o.startVelocity) ? o.startVelocity : 45) * K;
    const th = (an + B(-sp / 2, sp / 2)) * Math.PI / 180;
    const sc = Number.isFinite(o.scalar) ? o.scalar : 1;
    const sz = B(54, 66) * sc;
    return {
      x: px,
      y: py,
      vx: Math.cos(th) * ve * B(0.72, 1.18) + B(-1.8, 1.8),
      vy: -Math.sin(th) * ve * B(0.72, 1.18) + B(-1.2, 1.2),
      g: (Number.isFinite(o.gravity) ? o.gravity : 0.95) * K,
      dc: Number.isFinite(o.decay) ? o.decay : 0.9,
      dr: Number.isFinite(o.drift) ? o.drift : B(-0.18, 0.18),
      s: sz,
      sy: B(0.62, 1.08),
      r: B(0, Math.PI * 2),
      rs: B(-0.32, 0.32),
      w: B(0, Math.PI * 2),
      ws: B(0.08, 0.18),
      l: 0,
      ml: B(76, 132),
      al: 1,
    };
  }

  function F(px, py) {
    if (!M.complete || !M.naturalWidth) {
      W = [px, py];
      return;
    }
    S();
    const fx = [
      [0.25, { spread: 26, startVelocity: 55 }],
      [0.20, { spread: 60 }],
      [0.35, { spread: 100, decay: 0.91, scalar: 0.8 }],
      [0.10, { spread: 120, startVelocity: 25, decay: 0.92, scalar: 1.2 }],
      [0.10, { spread: 120, startVelocity: 45 }],
    ];
    let used = 0;
    fx.forEach((it, idx) => {
      const cnt = idx === fx.length - 1 ? (N - used) : Math.floor(N * it[0]);
      used += cnt;
      for (let i = 0; i < cnt; i += 1) P.push(O(px, py, it[1]));
    });
    if (!A) {
      T = performance.now();
      A = requestAnimationFrame(G);
    }
  }

  function G(now) {
    if (!X) return;
    const dt = Math.min(2.2, Math.max(0.45, (now - T) / 16.6667));
    T = now;
    X.clearRect(0, 0, innerWidth, innerHeight);

    for (let i = P.length - 1; i >= 0; i -= 1) {
      const p = P[i];
      p.l += dt;
      p.vx *= Math.pow(p.dc, dt);
      p.vy *= Math.pow(p.dc, dt);
      p.vy += p.g * dt;
      p.vx += p.dr * dt;
      p.x += p.vx * dt;
      p.y += p.vy * dt;
      p.r += p.rs * dt;
      p.w += p.ws * dt;
      p.al = Math.max(0, 1 - Math.pow(p.l / p.ml, 2.2));

      if (p.l >= p.ml || p.al <= 0 || p.y > innerHeight + 90 || p.x < -110 || p.x > innerWidth + 110) {
        P.splice(i, 1);
        continue;
      }

      X.save();
      X.globalAlpha = p.al;
      X.translate(p.x, p.y);
      X.rotate(p.r);
      X.scale(1 + Math.sin(p.w) * 0.18, p.sy);
      X.drawImage(M, -p.s / 2, -p.s / 2, p.s, p.s);
      X.restore();
    }

    if (P.length) {
      A = requestAnimationFrame(G);
    } else {
      A = 0;
      X.clearRect(0, 0, innerWidth, innerHeight);
    }
  }

  addEventListener($(112, 111, 105, 110, 116, 101, 114, 100, 111, 119, 110), (e) => {
    if (e.isPrimary === false || e.button > 0 || Math.random() >= Q) return;
    F(e.clientX, e.clientY);
  }, { passive: true });

  addEventListener(E, (e) => {
    const d = e.detail || {};
    const q = d[$(113)];
    if (Number.isFinite(q)) Q = Math.max(0, Math.min(1, q));
    if (Number.isFinite(d.x) || Number.isFinite(d.y)) {
      F(Number.isFinite(d.x) ? d.x : innerWidth / 2, Number.isFinite(d.y) ? d.y : innerHeight * 0.54);
    }
  });

  M.addEventListener($(108, 111, 97, 100), () => {
    if (!W) return;
    const p = W;
    W = null;
    F(p[0], p[1]);
  }, { once: true });
})();
