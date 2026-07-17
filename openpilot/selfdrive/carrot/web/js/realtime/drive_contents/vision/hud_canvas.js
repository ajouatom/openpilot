"use strict";

globalThis.DriveVisionHudCanvas = (() => {
  const DEFAULT_FONT = "system-ui, -apple-system, BlinkMacSystemFont, 'Segoe UI', sans-serif";
  const GRADIENT_CACHE_MAX = 8;
  const TEXT_WIDTH_CACHE_MAX = 128;

  function clamp(value, min, max) {
    return Math.min(max, Math.max(min, value));
  }

  function shortText(value, maxLength = 88) {
    const text = String(value || "").trim();
    if (!text) return "";
    return text.length > maxLength ? `${text.slice(0, maxLength - 1)}...` : text;
  }

  function create(options = {}) {
    const stage = options.stage;
    const context = options.context;
    const fontFamily = String(options.fontFamily || DEFAULT_FONT);
    if (!stage || !context) return null;

    const gradientCache = new Map();
    const textWidthCache = new Map();

    function cachedGradient(key, factory) {
      const cached = gradientCache.get(key);
      if (cached) return cached;
      const gradient = factory();
      if (gradientCache.size >= GRADIENT_CACHE_MAX) {
        gradientCache.delete(gradientCache.keys().next().value);
      }
      gradientCache.set(key, gradient);
      return gradient;
    }

    function cachedTextWidth(font, text) {
      const label = String(text || "");
      const key = `${font}|${label}`;
      const cached = textWidthCache.get(key);
      if (cached != null) return cached;
      context.save();
      context.font = font;
      const width = context.measureText(label).width;
      context.restore();
      if (textWidthCache.size >= TEXT_WIDTH_CACHE_MAX) {
        textWidthCache.delete(textWidthCache.keys().next().value);
      }
      textWidthCache.set(key, width);
      return width;
    }

    function fitSingleLine(text, preferredSize, maxWidth, minSize = 4.5, fontWeight = 900) {
      const label = String(text || "").trim();
      if (!label) return preferredSize;
      const font = `${fontWeight} ${preferredSize}px ${fontFamily}`;
      const measured = cachedTextWidth(font, label);
      if (measured <= maxWidth || measured <= 1) return preferredSize;
      return clamp(preferredSize * ((maxWidth / measured) * 0.985), minSize, preferredSize);
    }

    function drawOutlinedText({
      text,
      x,
      y,
      color = "rgba(244, 244, 244, 0.94)",
      strokeColor = "rgba(0, 0, 0, 0.94)",
      strokeWidth = 3,
      fontSize = 24,
      fontWeight = 900,
      alignX = "left",
      alignY = "top",
      maxWidth,
    }) {
      const label = String(text || "").trim();
      if (!label) return false;
      context.save();
      context.font = `${fontWeight} ${fontSize}px ${fontFamily}`;
      context.fillStyle = color;
      context.strokeStyle = strokeColor;
      context.lineWidth = strokeWidth;
      context.lineJoin = "round";
      context.miterLimit = 2;
      context.textAlign = alignX === "center" ? "center" : alignX === "right" ? "right" : "left";
      context.textBaseline = alignY === "middle"
        ? "middle"
        : alignY === "bottom"
          ? "bottom"
          : alignY === "baselineBottom"
            ? "alphabetic"
            : "top";
      if (strokeWidth > 0) context.strokeText(label, x, y, maxWidth);
      context.fillText(label, x, y, maxWidth);
      context.restore();
      return true;
    }

    function labelAlpha(pathMode, phaseShift = 0) {
      if (pathMode < 1 || pathMode > 8) return 0.94;
      const now = globalThis.performance?.now?.() ?? Date.now();
      const wave = (Math.sin((now / 1000) * 0.78 + phaseShift) + 1) * 0.5;
      return clamp(0.14 + Math.pow(wave, 1.7) * 0.86, 0.14, 1);
    }

    function textMetrics(stageWidth, stageHeight) {
      const exact = stageWidth >= 1280 && stageHeight >= 720;
      const baseScale = Math.min(stageWidth / 1920, stageHeight / 1080);
      return {
        exact,
        scale: clamp(baseScale, 0.48, 1),
      };
    }

    function drawTopRight(stageWidth, stageHeight, viewportRect, text, pathMode) {
      if (stage.classList.contains("is-drive-workspace-compact-primary")) return false;
      const label = shortText(text, 160);
      if (!label) return false;
      const { exact, scale } = textMetrics(stageWidth, stageHeight);
      const insetX = exact ? 1.5 : clamp(2 * scale, 1, 2.5);
      const insetTop = exact ? 1.5 : clamp(2 * scale, 1, 2.5);
      const maxWidth = Math.max(120, viewportRect.width - insetX * 2);
      const fontSize = fitSingleLine(label, exact ? 24 : clamp(24 * scale, 7, 24), maxWidth, 4.5, 900);
      const alpha = labelAlpha(pathMode);
      return drawOutlinedText({
        text: label,
        x: viewportRect.right - insetX,
        y: viewportRect.top + insetTop,
        color: `rgba(244, 244, 244, ${alpha.toFixed(3)})`,
        strokeColor: `rgba(0, 0, 0, ${clamp(alpha + 0.08, 0, 1).toFixed(3)})`,
        strokeWidth: clamp(4.2 * scale, 2.8, 5.4),
        fontSize,
        fontWeight: 900,
        alignX: "right",
        alignY: "top",
        maxWidth,
      });
    }

    function drawTopLeft(stageWidth, stageHeight, viewportRect, text, pathMode) {
      if (stage.classList.contains("is-drive-workspace-compact-primary")) return false;
      if (!window.CarrotLayout?.isWide?.()) return false;
      const label = shortText(text, 160);
      if (!label) return false;
      const { exact, scale } = textMetrics(stageWidth, stageHeight);
      const insetX = exact ? 1.5 : clamp(2 * scale, 1, 2.5);
      const insetTop = exact ? 1.5 : clamp(2 * scale, 1, 2.5);
      const maxWidth = Math.max(120, viewportRect.width - insetX * 2);
      const fontSize = fitSingleLine(label, exact ? 24 : clamp(24 * scale, 7, 24), maxWidth, 4.5, 900);
      const alpha = labelAlpha(pathMode);
      return drawOutlinedText({
        text: label,
        x: viewportRect.left + insetX,
        y: viewportRect.top + insetTop,
        color: `rgba(244, 244, 244, ${alpha.toFixed(3)})`,
        strokeColor: `rgba(0, 0, 0, ${clamp(alpha + 0.08, 0, 1).toFixed(3)})`,
        strokeWidth: clamp(4.2 * scale, 2.8, 5.4),
        fontSize,
        fontWeight: 900,
        alignX: "left",
        alignY: "top",
        maxWidth,
      });
    }

    function drawLeftCenter(stageWidth, stageHeight, viewportRect, statusText, metaText, pathMode) {
      const statusLabel = shortText(statusText, 96);
      const metaLabel = shortText(metaText, 160);
      if (!statusLabel && !metaLabel) return false;
      const { exact, scale } = textMetrics(stageWidth, stageHeight);
      const insetX = exact ? 1.5 : clamp(2 * scale, 1, 2.5);
      const maxWidth = Math.max(180, viewportRect.width * 0.52);
      const alpha = labelAlpha(pathMode, Math.PI * 0.5);
      const statusFontSize = fitSingleLine(statusLabel, exact ? 24 : clamp(24 * scale, 9, 24), maxWidth, 6, 900);
      const metaFontSize = fitSingleLine(metaLabel, exact ? 20 : clamp(20 * scale, 8, 20), maxWidth, 6, 800);
      const statusY = viewportRect.centerY - (exact ? 16 : clamp(18 * scale, 10, 18));
      const metaY = viewportRect.centerY + (exact ? 14 : clamp(16 * scale, 9, 16));
      const strokeAlpha = clamp(alpha + 0.08, 0, 1).toFixed(3);
      drawOutlinedText({
        text: statusLabel,
        x: viewportRect.left + insetX,
        y: statusY,
        color: `rgba(244, 244, 244, ${alpha.toFixed(3)})`,
        strokeColor: `rgba(0, 0, 0, ${strokeAlpha})`,
        strokeWidth: clamp(4.2 * scale, 2.8, 5.4),
        fontSize: statusFontSize,
        fontWeight: 900,
        alignX: "left",
        alignY: "bottom",
        maxWidth,
      });
      drawOutlinedText({
        text: metaLabel,
        x: viewportRect.left + insetX,
        y: metaY,
        color: `rgba(236, 236, 236, ${alpha.toFixed(3)})`,
        strokeColor: `rgba(0, 0, 0, ${strokeAlpha})`,
        strokeWidth: clamp(4 * scale, 2.8, 5.2),
        fontSize: metaFontSize,
        fontWeight: 800,
        alignX: "left",
        alignY: "top",
        maxWidth,
      });
      return true;
    }

    function drawBottomCenter(stageWidth, stageHeight, viewportRect, text, pathMode) {
      const label = String(text || "").trim();
      if (!label) return false;
      const { exact, scale } = textMetrics(stageWidth, stageHeight);
      const maxWidth = Math.max(120, viewportRect.width - 4);
      const fontSize = fitSingleLine(label, exact ? 24 : clamp(24 * scale, 7, 24), maxWidth, 4.5, 900);
      const bottomInset = exact ? 1.5 : clamp(2 * scale, 1, 2.5);
      const alpha = labelAlpha(pathMode, Math.PI);
      return drawOutlinedText({
        text: label,
        x: viewportRect.centerX,
        y: viewportRect.bottom - bottomInset,
        color: `rgba(236, 236, 236, ${alpha.toFixed(3)})`,
        strokeColor: `rgba(0, 0, 0, ${clamp(alpha + 0.08, 0, 1).toFixed(3)})`,
        strokeWidth: clamp(4 * scale, 2.8, 5.2),
        fontSize,
        fontWeight: 900,
        alignX: "center",
        alignY: "baselineBottom",
        maxWidth,
      });
    }

    function drawBottomLeft(stageWidth, stageHeight, viewportRect, text, pathMode) {
      const label = shortText(text, 160);
      if (!label) return false;
      const { exact, scale } = textMetrics(stageWidth, stageHeight);
      const insetX = exact ? 1.5 : clamp(2 * scale, 1, 2.5);
      const bottomInset = exact ? 1.5 : clamp(2 * scale, 1, 2.5);
      const maxWidth = Math.max(120, viewportRect.width * 0.42);
      const fontSize = fitSingleLine(label, exact ? 24 : clamp(24 * scale, 7, 24), maxWidth, 4.5, 900);
      const alpha = labelAlpha(pathMode, Math.PI);
      return drawOutlinedText({
        text: label,
        x: viewportRect.left + insetX,
        y: viewportRect.bottom - bottomInset,
        color: `rgba(236, 236, 236, ${alpha.toFixed(3)})`,
        strokeColor: `rgba(0, 0, 0, ${clamp(alpha + 0.08, 0, 1).toFixed(3)})`,
        strokeWidth: clamp(4 * scale, 2.8, 5.2),
        fontSize,
        fontWeight: 900,
        alignX: "left",
        alignY: "baselineBottom",
        maxWidth,
      });
    }

    function drawEdgeFades(stageWidth, stageHeight) {
      const topHeight = clamp(stageHeight * 0.16, 72, 148);
      const bottomHeight = clamp(stageHeight * 0.24, 104, 212);
      context.save();
      context.globalCompositeOperation = "source-over";
      const topKey = `hud-top:${Math.round(stageWidth)}x${Math.round(stageHeight)}:${Math.round(topHeight)}`;
      const topGradient = cachedGradient(topKey, () => {
        const gradient = context.createLinearGradient(0, 0, 0, topHeight);
        gradient.addColorStop(0, "rgba(0, 0, 0, 0.64)");
        gradient.addColorStop(0.42, "rgba(0, 0, 0, 0.30)");
        gradient.addColorStop(1, "rgba(0, 0, 0, 0.00)");
        return gradient;
      });
      context.fillStyle = topGradient;
      context.fillRect(0, 0, stageWidth, topHeight);
      const bottomStart = Math.max(0, stageHeight - bottomHeight);
      const bottomKey = `hud-bottom:${Math.round(stageWidth)}x${Math.round(stageHeight)}:${Math.round(bottomStart)}:${Math.round(bottomHeight)}`;
      const bottomGradient = cachedGradient(bottomKey, () => {
        const gradient = context.createLinearGradient(0, bottomStart, 0, stageHeight);
        gradient.addColorStop(0, "rgba(0, 0, 0, 0.00)");
        gradient.addColorStop(0.42, "rgba(0, 0, 0, 0.28)");
        gradient.addColorStop(1, "rgba(0, 0, 0, 0.74)");
        return gradient;
      });
      context.fillStyle = bottomGradient;
      context.fillRect(0, bottomStart, stageWidth, bottomHeight);
      context.restore();
    }

    function clear(stageWidth, stageHeight) {
      context.clearRect(0, 0, stageWidth, stageHeight);
    }

    function reset() {
      gradientCache.clear();
      textWidthCache.clear();
    }

    function status() {
      return {
        gradientCacheSize: gradientCache.size,
        textWidthCacheSize: textWidthCache.size,
      };
    }

    return Object.freeze({
      clear,
      drawEdgeFades,
      drawTopLeft,
      drawTopRight,
      drawLeftCenter,
      drawBottomCenter,
      drawBottomLeft,
      reset,
      status,
    });
  }

  return Object.freeze({ create });
})();
