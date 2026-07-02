"use strict";

(() => {
  const stage = document.getElementById("carrotStage");
  const canvas = document.getElementById("carrotAmbientCanvas");
  if (!stage || !canvas) return;

  const ctx = canvas.getContext("2d", { alpha: true });
  if (!ctx) return;

  const reduceMotion = window.matchMedia?.("(prefers-reduced-motion: reduce)");
  let particles = [];
  let width = 0;
  let height = 0;
  let dpr = 1;
  let rafId = 0;
  let resizeObserver = null;
  let stageObserver = null;
  let bodyObserver = null;
  let stopTimer = null;

  function isStreamReady() {
    return stage.classList.contains("is-stream-ready");
  }

  function isCarrotPageVisible() {
    return document.body?.dataset?.page === "carrot" && !document.hidden;
  }

  function shouldAnimate() {
    return isCarrotPageVisible() && !isStreamReady() && !reduceMotion?.matches;
  }

  function rand(min, max) {
    return min + Math.random() * (max - min);
  }

  function resizeCanvas() {
    const rect = stage.getBoundingClientRect();
    const nextWidth = Math.max(1, Math.round(rect.width));
    const nextHeight = Math.max(1, Math.round(rect.height));
    const nextDpr = Math.min(window.devicePixelRatio || 1, 2);
    const changed = nextWidth !== width || nextHeight !== height || nextDpr !== dpr;

    width = nextWidth;
    height = nextHeight;
    dpr = nextDpr;
    canvas.width = Math.max(1, Math.round(width * dpr));
    canvas.height = Math.max(1, Math.round(height * dpr));
    canvas.style.width = `${width}px`;
    canvas.style.height = `${height}px`;
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);

    if (changed || !particles.length) seedParticles();
  }

  function seedParticles() {
    const area = width * height;
    const portrait = height > width;
    const density = portrait ? 13200 : 15000;
    const count = Math.round(Math.min(88, Math.max(portrait ? 42 : 48, area / density)));
    particles = Array.from({ length: count }, (_, index) => {
      const accent = index % 3 === 0 || Math.random() > 0.54;
      const slow = index % 7 === 0;
      return {
        x: rand(0, width),
        y: rand(0, height),
        vx: rand(-0.16, 0.16) * (slow ? 0.45 : 1),
        vy: rand(-0.13, 0.13) * (slow ? 0.45 : 1),
        r: accent ? rand(1.15, 1.9) : rand(0.85, 1.35),
        accent,
      };
    });
  }

  function moveParticle(p) {
    p.x += p.vx;
    p.y += p.vy;

    if (p.x < -24) p.x = width + 24;
    if (p.x > width + 24) p.x = -24;
    if (p.y < -24) p.y = height + 24;
    if (p.y > height + 24) p.y = -24;
  }

  function drawBackgroundMesh() {
    const centerX = width * 0.5;
    const centerY = height * 0.5;
    const longSide = Math.max(width, height);

    ctx.save();
    ctx.globalAlpha = 0.65;
    ctx.lineWidth = 1;

    const axis = [
      { angle: -0.32, alpha: 0.18, color: "255, 172, 96", scale: 0.66 },
      { angle: 0.28, alpha: 0.12, color: "150, 158, 170", scale: 0.56 },
      { angle: -0.05, alpha: 0.10, color: "255, 172, 96", scale: 0.44 },
    ];

    for (const line of axis) {
      const len = longSide * line.scale;
      const dx = Math.cos(line.angle) * len * 0.5;
      const dy = Math.sin(line.angle) * len * 0.5;
      const gradient = ctx.createLinearGradient(centerX - dx, centerY - dy, centerX + dx, centerY + dy);
      gradient.addColorStop(0, "rgba(0,0,0,0)");
      gradient.addColorStop(0.5, `rgba(${line.color}, ${line.alpha})`);
      gradient.addColorStop(1, "rgba(0,0,0,0)");
      ctx.strokeStyle = gradient;
      ctx.beginPath();
      ctx.moveTo(centerX - dx, centerY - dy);
      ctx.lineTo(centerX + dx, centerY + dy);
      ctx.stroke();
    }

    ctx.restore();
  }

  function drawFrame() {
    ctx.clearRect(0, 0, width, height);
    drawBackgroundMesh();

    const maxDistance = Math.min(188, Math.max(118, Math.min(width, height) * 0.28));
    for (let i = 0; i < particles.length; i += 1) {
      const a = particles[i];
      for (let j = i + 1; j < particles.length; j += 1) {
        const b = particles[j];
        const dx = a.x - b.x;
        const dy = a.y - b.y;
        const dist = Math.hypot(dx, dy);
        if (dist > maxDistance) continue;

        const t = 1 - dist / maxDistance;
        const warm = a.accent || b.accent;
        const alpha = t * (warm ? 0.30 : 0.18);
        ctx.strokeStyle = warm
          ? `rgba(255, 172, 96, ${alpha})`
          : `rgba(150, 158, 170, ${alpha})`;
        ctx.lineWidth = warm && t > 0.68 ? 1.15 : 1;
        ctx.beginPath();
        ctx.moveTo(a.x, a.y);
        ctx.lineTo(b.x, b.y);
        ctx.stroke();
      }
    }

    for (const p of particles) {
      ctx.fillStyle = p.accent ? "rgba(255, 184, 118, 0.88)" : "rgba(158, 168, 182, 0.56)";
      ctx.beginPath();
      ctx.arc(p.x, p.y, p.r, 0, Math.PI * 2);
      ctx.fill();

      if (shouldAnimate()) moveParticle(p);
    }
  }

  function renderLoop() {
    rafId = 0;
    resizeCanvas();
    drawFrame();
    if (shouldAnimate()) rafId = requestAnimationFrame(renderLoop);
  }

  function startLoop() {
    if (rafId) return;
    rafId = requestAnimationFrame(renderLoop);
  }

  function stopLoopSoon() {
    if (stopTimer) clearTimeout(stopTimer);
    stopTimer = window.setTimeout(() => {
      stopTimer = null;
      if (isStreamReady() && rafId) {
        cancelAnimationFrame(rafId);
        rafId = 0;
      }
    }, 420);
  }

  function syncAmbientState() {
    const hide = isStreamReady();
    stage.classList.toggle("carrot-stage--ambient-hidden", hide);
    if (hide) {
      drawFrame();
      stopLoopSoon();
      return;
    }
    startLoop();
  }

  function handleResize() {
    resizeCanvas();
    drawFrame();
  }

  if (window.ResizeObserver) {
    resizeObserver = new ResizeObserver(handleResize);
    resizeObserver.observe(stage);
  } else {
    window.addEventListener("resize", handleResize, { passive: true });
  }

  if (window.MutationObserver) {
    stageObserver = new MutationObserver(syncAmbientState);
    stageObserver.observe(stage, { attributes: true, attributeFilter: ["class"] });

    bodyObserver = new MutationObserver(syncAmbientState);
    bodyObserver.observe(document.body, { attributes: true, attributeFilter: ["data-page"] });
  }

  window.addEventListener("orientationchange", () => {
    window.setTimeout(handleResize, 120);
  }, { passive: true });
  window.addEventListener("focus", syncAmbientState);
  window.addEventListener("pageshow", syncAmbientState);
  window.addEventListener("carrot:visionchange", syncAmbientState);
  window.addEventListener("carrot:pagechange", syncAmbientState);
  document.addEventListener("visibilitychange", syncAmbientState);
  reduceMotion?.addEventListener?.("change", syncAmbientState);

  resizeCanvas();
  drawFrame();
  syncAmbientState();

  window.CarrotVisionBackground = {
    refresh: handleResize,
    destroy() {
      if (rafId) cancelAnimationFrame(rafId);
      if (stopTimer) clearTimeout(stopTimer);
      resizeObserver?.disconnect();
      stageObserver?.disconnect();
      bodyObserver?.disconnect();
    },
  };
})();
