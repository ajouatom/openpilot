import {
  DPR_LIMIT,
  TBT_CURRENT_BOTTOM_TRANSPARENT_PX,
  TBT_STACK_OVERLAP_PX,
  clamp,
  finite,
  recordPresent,
  findAlphaRowBounds,
  formatDistance,
  formatClock,
  roundedRect,
} from "./overlay_primitives.js";

export function guidanceGlyph(guidance) {
  const label = `${guidance?.nearDirection || ""} ${guidance?.mainText || ""}`.toLowerCase();
  if (/유턴|u-?turn/.test(label)) return "↶";
  if (/왼|left/.test(label)) return "↰";
  if (/오른|우회|right/.test(label)) return "↱";
  return "↑";
}

export function selectCurrentGuidanceImageName(hasImage) {
  if (hasImage("tbt_current_full")) return "tbt_current_full";
  if (hasImage("tbt_current_compact")) return "tbt_current_compact";
  return "";
}

export class OverlayRenderer {
  constructor(canvas) {
    this.canvas = canvas;
    // This is a backing canvas consumed by the compositor, not a visible
    // compositor layer.  Keep it synchronized so a canvas-to-canvas copy
    // always sees the latest overlay frame on mobile browsers.
    this.context = canvas?.getContext("2d", { alpha: true }) || null;
    this.images = new Map();
    this.measureCanvas = null;
    this.measureContext = null;
    this.visibleRowCache = new Map();
    // Once the native TMAP image channel is observed, keep the whole
    // session native-first. Individual image streams are intentionally
    // sparse and may clear between guidance states; falling back per item
    // would mix the native skin with a second JSON-rendered skin.
    this.nativeImageSession = false;
    this.state = null;
    // Route summary and road-name text have a different lifetime from the
    // native TMAP media records. The store owns their freshness policy and
    // supplies the last effective JSON values independently of image clear.
    this.footerState = null;
    this.displayStatus = null;
    this.renderRequest = 0;
    this.renderSuspended = false;
  }

  setState(state) {
    this.state = state && typeof state === "object" ? state : null;
    this.requestRender();
  }

  setFooterState(state) {
    this.footerState = state && typeof state === "object" ? state : null;
    this.requestRender();
  }

  setStatus(status) {
    this.displayStatus = status && typeof status === "object" ? status : null;
    this.requestRender();
  }

  setImage(name, source, sequence = 0) {
    if (!name || !source) return;
    this.nativeImageSession = true;
    const previous = this.images.get(name);
    if (previous && Number(sequence) < previous.sequence) {
      source.close?.();
      return;
    }
    previous?.source?.close?.();
    this.images.set(name, {
      source,
      sequence: Math.max(0, Number(sequence) || 0),
      visibleRows: this.measureVisibleRows(name, source),
    });
    this.requestRender();
  }

  applyBatch(updates) {
    if (!Array.isArray(updates) || !updates.length) return false;
    let changed = false;
    for (const update of updates) {
      const name = String(update?.name || "");
      if (!name) {
        update?.source?.close?.();
        continue;
      }
      // A clear packet still proves that the native image stream exists.
      this.nativeImageSession = true;
      if (update.clear) {
        const previous = this.images.get(name);
        previous?.source?.close?.();
        changed = this.images.delete(name) || changed;
        continue;
      }
      const source = update.source;
      if (!source) continue;
      const sequence = Math.max(0, Number(update.sequence) || 0);
      const previous = this.images.get(name);
      if (previous && sequence < previous.sequence) {
        source.close?.();
        continue;
      }
      previous?.source?.close?.();
      this.images.set(name, {
        source,
        sequence,
        visibleRows: this.measureVisibleRows(name, source),
      });
      changed = true;
    }
    if (changed) this.requestRender();
    return changed;
  }

  clearImage(name) {
    const previous = this.images.get(name);
    previous?.source?.close?.();
    this.images.delete(name);
    this.requestRender();
  }

  clearImages() {
    for (const image of this.images.values()) image.source?.close?.();
    this.images.clear();
    this.nativeImageSession = false;
    this.requestRender();
  }

  reset() {
    this.clearImages();
    this.state = null;
    this.footerState = null;
    this.displayStatus = null;
    this.requestRender();
  }

  destroy() {
    if (this.renderRequest) cancelAnimationFrame(this.renderRequest);
    this.renderRequest = 0;
    for (const image of this.images.values()) image.source?.close?.();
    this.images.clear();
    this.state = null;
    this.footerState = null;
    this.displayStatus = null;
    if (this.context) this.context.clearRect(0, 0, this.canvas.width, this.canvas.height);
  }

  setRenderSuspended(suspended) {
    const next = Boolean(suspended);
    if (next === this.renderSuspended) return;
    this.renderSuspended = next;
    if (next && this.renderRequest) {
      cancelAnimationFrame(this.renderRequest);
      this.renderRequest = 0;
    } else if (!next) {
      this.requestRender();
    }
  }

  requestRender() {
    if (!this.context || this.renderSuspended || this.renderRequest) return;
    this.renderRequest = requestAnimationFrame(() => {
      this.renderRequest = 0;
      this.render();
    });
  }

  renderNow() {
    if (!this.context || this.renderSuspended) return false;
    if (this.renderRequest) cancelAnimationFrame(this.renderRequest);
    this.renderRequest = 0;
    this.render();
    return true;
  }

  image(name) {
    return this.images.get(name)?.source || null;
  }

  measureVisibleRows(name, source) {
    if (name !== "tbt_current_full" && name !== "tbt_current_compact" && name !== "tbt_next") return null;
    const sourceWidth = Math.round(finite(source?.width || source?.videoWidth));
    const sourceHeight = Math.round(finite(source?.height || source?.videoHeight));
    if (sourceWidth < 1 || sourceHeight < 1 || typeof document === "undefined" || typeof document.createElement !== "function") return null;
    const cacheKey = `${name}:${sourceWidth}x${sourceHeight}`;
    const cached = this.visibleRowCache.get(cacheKey);
    if (cached) return cached;
    try {
      if (!this.measureCanvas) {
        this.measureCanvas = document.createElement("canvas");
        this.measureContext = this.measureCanvas.getContext("2d", { alpha: true, willReadFrequently: true });
      }
      if (!this.measureContext) return null;
      this.measureCanvas.width = sourceWidth;
      this.measureCanvas.height = sourceHeight;
      this.measureContext.clearRect(0, 0, sourceWidth, sourceHeight);
      this.measureContext.drawImage(source, 0, 0, sourceWidth, sourceHeight);
      const pixels = this.measureContext.getImageData(0, 0, sourceWidth, sourceHeight).data;
      const visibleRows = findAlphaRowBounds(pixels, sourceWidth, sourceHeight);
      if (visibleRows) this.visibleRowCache.set(cacheKey, visibleRows);
      return visibleRows;
    } catch (_) {
      return null;
    }
  }

  snapshot() {
    return {
      activeImages: Array.from(this.images.keys()).sort(),
      nativeImageSession: this.nativeImageSession,
      hasState: Boolean(this.state),
      hasFooterRoute: recordPresent(this.footerState?.route),
      hasFooterVehicle: recordPresent(this.footerState?.vehicle),
      displayStatus: this.displayStatus?.mode || "",
    };
  }

  imageGeometry(name, rectangle, options = {}) {
    const image = this.images.get(name);
    const source = image?.source;
    if (!source) return null;
    const sourceWidth = finite(source.width || source.videoWidth);
    const sourceHeight = finite(source.height || source.videoHeight);
    if (sourceWidth < 1 || sourceHeight < 1 || rectangle.width < 1 || rectangle.height < 1) return null;
    const scale = Math.min(rectangle.width / sourceWidth, rectangle.height / sourceHeight);
    const width = sourceWidth * scale;
    const height = sourceHeight * scale;
    const alignX = clamp(finite(options.alignX, 0.5), 0, 1);
    const alignY = clamp(finite(options.alignY, 0.5), 0, 1);
    const x = rectangle.x + (rectangle.width - width) * alignX;
    const y = rectangle.y + (rectangle.height - height) * alignY;
    return { source, x, y, width, height, visibleRows: image?.visibleRows || null };
  }

  drawImage(name, rectangle, options = {}) {
    if (!this.context) return null;
    const geometry = this.imageGeometry(name, rectangle, options);
    if (!geometry) return null;
    try {
      this.context.drawImage(geometry.source, geometry.x, geometry.y, geometry.width, geometry.height);
      return geometry;
    } catch (_) {
      return null;
    }
  }

  drawOutlinedText(text, x, y, size, options = {}) {
    if (!this.context || !text) return;
    const context = this.context;
    context.save();
    context.font = `${options.weight || 800} ${Math.max(10, size)}px system-ui, -apple-system, sans-serif`;
    context.textAlign = options.align || "left";
    context.textBaseline = options.baseline || "alphabetic";
    context.lineJoin = "round";
    context.strokeStyle = options.stroke || "rgba(0, 0, 0, 0.88)";
    context.lineWidth = Math.max(2, size * 0.13);
    context.fillStyle = options.color || "#fff";
    const maxWidth = Number(options.maxWidth);
    if (Number.isFinite(maxWidth) && maxWidth > 0) {
      context.strokeText(String(text), x, y, maxWidth);
      context.fillText(String(text), x, y, maxWidth);
    } else {
      context.strokeText(String(text), x, y);
      context.fillText(String(text), x, y);
    }
    context.restore();
  }

  drawGuidanceFallback(width, height, padding) {
    const guidance = this.state?.guidanceCurrent;
    if (!recordPresent(guidance)) return null;
    const context = this.context;
    const boxWidth = Math.min(width * 0.62, 330);
    const boxHeight = Math.min(height * 0.22, 118);
    context.save();
    roundedRect(context, padding, padding, boxWidth, boxHeight, 14);
    context.fillStyle = "rgba(4, 33, 27, 0.88)";
    context.fill();
    context.strokeStyle = "rgba(116, 255, 210, 0.28)";
    context.lineWidth = 1;
    context.stroke();
    context.restore();
    this.drawOutlinedText(guidanceGlyph(guidance), padding + 18, padding + boxHeight * 0.52, clamp(boxHeight * 0.42, 30, 48));
    this.drawOutlinedText(formatDistance(guidance.distanceM), padding + 70, padding + boxHeight * 0.46, clamp(boxHeight * 0.32, 24, 38));
    const label = guidance.mainText || guidance.roadName || guidance.nearDirection;
    this.drawOutlinedText(label, padding + 70, padding + boxHeight * 0.78, clamp(boxHeight * 0.16, 13, 19), {
      color: "rgba(255,255,255,0.9)",
      maxWidth: boxWidth - 84,
    });
    return { x: padding, y: padding, width: boxWidth, height: boxHeight };
  }

  drawNextGuidanceFallback(x, y, width, height) {
    const guidance = this.state?.guidanceNext;
    if (!recordPresent(guidance)) return false;
    const context = this.context;
    context.save();
    roundedRect(context, x, y, width, height, 12);
    context.fillStyle = "rgba(8, 14, 20, 0.84)";
    context.fill();
    context.strokeStyle = "rgba(255, 255, 255, 0.16)";
    context.stroke();
    context.restore();
    this.drawOutlinedText(guidanceGlyph(guidance), x + 15, y + height * 0.68, height * 0.5);
    this.drawOutlinedText(formatDistance(guidance.distanceM), x + 54, y + height * 0.43, clamp(height * 0.28, 15, 22));
    this.drawOutlinedText(guidance.mainText || guidance.roadName || guidance.nearDirection, x + 54, y + height * 0.75, clamp(height * 0.2, 12, 16), {
      color: "rgba(255,255,255,0.8)",
      maxWidth: width - 65,
    });
    return true;
  }

  drawGuidanceStack(width, height, padding, compact, allowFallback = true) {
    // The compact native image is a deliberately short strip. Prefer the
    // full card at every pane width and reserve compact for source fallback.
    const currentName = selectCurrentGuidanceImageName((name) => Boolean(this.image(name)));
    const currentSlot = {
      x: padding,
      y: padding,
      width: Math.min(width * (compact ? 0.72 : 0.58), compact ? 350 : 500),
      height: Math.min(height * (compact ? 0.29 : 0.2), compact ? 160 : 112),
    };
    const currentBounds = this.drawImage(currentName, currentSlot, { alignX: 0, alignY: 0 })
      || (allowFallback ? this.drawGuidanceFallback(width, height, padding) : null);
    const currentSourceHeight = finite(currentBounds?.source?.height || currentBounds?.source?.videoHeight);
    const currentImageScale = currentSourceHeight > 0 ? currentBounds.height / currentSourceHeight : 0;
    const nextWidth = Math.min(currentBounds ? currentBounds.width * 0.58 : width * 0.38, width * 0.42, 230);
    const nextSlot = {
      x: currentBounds?.x ?? padding,
      y: 0,
      width: nextWidth,
      height: Math.min(height * 0.15, 82),
    };
    const currentVisibleBottom = currentBounds
      ? currentBounds.y + (currentBounds.visibleRows
        ? currentBounds.visibleRows.bottom * currentImageScale
        : Math.max(0, currentBounds.height - TBT_CURRENT_BOTTOM_TRANSPARENT_PX * currentImageScale))
      : padding;
    const nextGeometry = this.imageGeometry("tbt_next", nextSlot, { alignX: 0, alignY: 0 });
    const nextSourceHeight = finite(nextGeometry?.source?.height || nextGeometry?.source?.videoHeight);
    const nextImageScale = nextSourceHeight > 0 ? nextGeometry.height / nextSourceHeight : 0;
    const nextTransparentTop = nextGeometry?.visibleRows
      ? nextGeometry.visibleRows.top * nextImageScale
      : 0;
    nextSlot.y = currentBounds
      ? currentVisibleBottom - nextTransparentTop - TBT_STACK_OVERLAP_PX
      : padding;
    const nextBounds = this.drawImage("tbt_next", nextSlot, { alignX: 0, alignY: 0 });
    if (!nextBounds && allowFallback) this.drawNextGuidanceFallback(nextSlot.x, nextSlot.y, nextSlot.width, nextSlot.height);
    return { current: currentBounds, next: nextBounds };
  }

  drawSafetyFallback(width, height, padding) {
    const speed = this.state?.speed;
    if (!recordPresent(speed) || (!speed.roadLimitValid && !speed.sectionActive && !speed.sdiPresent)) return;
    const context = this.context;
    const radius = clamp(Math.min(width, height) * 0.055, 24, 38);
    const x = padding + radius;
    const y = height * 0.66;
    if (speed.sectionActive) {
      const panelWidth = Math.min(width * 0.36, 230);
      const panelHeight = Math.min(height * 0.2, 104);
      const panelY = y - radius - 9;
      context.save();
      roundedRect(context, padding, panelY, panelWidth, panelHeight, 14);
      context.fillStyle = "rgba(7, 12, 18, 0.88)";
      context.fill();
      context.strokeStyle = speed.sectionOffRoute ? "#ff9b45" : "rgba(255,255,255,0.18)";
      context.stroke();
      roundedRect(context, padding + 10, panelY + panelHeight - 14, panelWidth - 20, 5, 3);
      context.fillStyle = "rgba(255,255,255,0.16)";
      context.fill();
      const progress = clamp(finite(speed.sectionProgress), 0, 1);
      if (progress > 0) {
        roundedRect(context, padding + 10, panelY + panelHeight - 14, (panelWidth - 20) * progress, 5, 3);
        context.fillStyle = speed.sectionSuspended ? "#ffb54a" : "#47d79a";
        context.fill();
      }
      context.restore();
      const sectionLabel = speed.sectionSuspended ? "구간단속 일시정지" : (speed.sectionOffRoute ? "구간 이탈" : "구간단속");
      const textX = padding + radius * 2 + 12;
      this.drawOutlinedText(sectionLabel, textX, panelY + 31, 13, {
        color: "rgba(255,255,255,.82)",
        maxWidth: panelWidth - radius * 2 - 22,
      });
      this.drawOutlinedText(`평균 ${Math.round(finite(speed.sectionAverageKph))} · ${formatDistance(speed.sectionRemainingDistanceM)}`, textX, panelY + 58, 14, {
        maxWidth: panelWidth - radius * 2 - 22,
      });
    }
    const limit = speed.roadLimitKph || speed.sdiSpeedLimitKph || speed.sectionSpeedLimitKph;
    if (!limit) return;
    context.save();
    context.beginPath();
    context.arc(x, y, radius, 0, Math.PI * 2);
    context.fillStyle = "#fff";
    context.fill();
    context.lineWidth = Math.max(5, radius * 0.18);
    context.strokeStyle = "#ef3340";
    context.stroke();
    context.restore();
    this.drawOutlinedText(String(limit), x, y + radius * 0.25, radius * 0.8, {
      align: "center",
      color: "#12151a",
      stroke: "rgba(255,255,255,0)",
    });
    if (speed.sdiPresent && speed.sdiDistanceM > 0) {
      this.drawOutlinedText(formatDistance(speed.sdiDistanceM), x, y + radius + 23, 15, { align: "center" });
    }
  }

  drawTrafficFallback(width, padding) {
    const signal = this.state?.trafficSignal;
    if (!recordPresent(signal) || !signal.visible) return;
    const values = [
      [signal.redValid, signal.redOn, signal.redRemainSec, "#ff4b55", "●"],
      [signal.leftValid, signal.leftOn, signal.leftRemainSec, "#43df83", "←"],
      [signal.greenValid, signal.greenOn, signal.greenRemainSec, "#43df83", "↑"],
      [signal.rightValid, signal.rightOn, signal.rightRemainSec, "#43df83", "→"],
      [signal.uturnValid, signal.uturnOn, signal.uturnRemainSec, "#43df83", "↶"],
    ].filter((item) => item[0]);
    if (!values.length) return;
    const context = this.context;
    const boxWidth = values.length * 38 + 18;
    const x = width - padding - boxWidth;
    const y = padding;
    context.save();
    roundedRect(context, x, y, boxWidth, 50, 14);
    context.fillStyle = "rgba(7, 12, 18, 0.86)";
    context.fill();
    values.forEach((item, index) => {
      const centerX = x + 28 + index * 38;
      context.beginPath();
      context.arc(centerX, y + 18, 10, 0, Math.PI * 2);
      context.fillStyle = item[1] ? item[3] : "rgba(255,255,255,0.16)";
      context.fill();
      this.drawOutlinedText(item[4], centerX, y + 23, 13, { align: "center", color: "#fff", stroke: "rgba(0,0,0,.55)" });
      if (item[2] > 0) this.drawOutlinedText(String(item[2]), centerX, y + 43, 12, { align: "center" });
    });
    context.restore();
    const counter = signal.uiCounterValid ? signal.uiCounterRemainSec : 0;
    const detail = [signal.distanceM > 0 ? formatDistance(signal.distanceM) : "", counter > 0 ? `${counter}초` : ""].filter(Boolean).join(" · ");
    if (detail) this.drawOutlinedText(detail, width - padding, y + 68, 13, { align: "right", color: "rgba(255,255,255,.82)" });
  }

  drawLaneFallback(width, height) {
    const lane = recordPresent(this.state?.laneCurrent)
      ? this.state.laneCurrent
      : (Array.isArray(this.state?.laneAhead) ? this.state.laneAhead.find(recordPresent) : null);
    if (!lane || !lane.visible || lane.count < 1) return false;
    const context = this.context;
    const count = clamp(Math.round(finite(lane.count)), 1, 8);
    const panelWidth = Math.min(width * 0.48, 300);
    const panelHeight = Math.min(height * 0.17, 92);
    const x = (width - panelWidth) * 0.5;
    const y = height - panelHeight - 8;
    const gap = 4;
    const laneWidth = (panelWidth - 20 - gap * (count - 1)) / count;
    context.save();
    roundedRect(context, x, y, panelWidth, panelHeight, 13);
    context.fillStyle = "rgba(5, 9, 14, 0.78)";
    context.fill();
    for (let index = 0; index < count; index += 1) {
      const selected = index === Number(lane.currentLane) || Number(lane.available?.[index]) > 0;
      const laneX = x + 10 + index * (laneWidth + gap);
      roundedRect(context, laneX, y + 12, laneWidth, panelHeight - 30, 7);
      context.fillStyle = selected ? "rgba(73, 218, 157, 0.82)" : "rgba(255,255,255,0.13)";
      context.fill();
      this.drawOutlinedText("↑", laneX + laneWidth * 0.5, y + panelHeight * 0.57, clamp(laneWidth * 0.72, 15, 28), { align: "center" });
    }
    context.restore();
    if (lane.distanceM > 0) this.drawOutlinedText(formatDistance(lane.distanceM), width * 0.5, y + panelHeight - 5, 12, { align: "center" });
    return true;
  }

  drawCrossroadFallback(width, height, padding) {
    const crossroad = this.state?.crossroad;
    if (!recordPresent(crossroad) || !crossroad.visible) return false;
    const boxWidth = Math.min(width * 0.28, 160);
    const boxHeight = 50;
    const x = width - padding - boxWidth;
    const y = height * 0.2;
    const context = this.context;
    context.save();
    roundedRect(context, x, y, boxWidth, boxHeight, 13);
    context.fillStyle = "rgba(13, 19, 27, .88)";
    context.fill();
    context.strokeStyle = "rgba(103, 190, 255, .45)";
    context.stroke();
    context.restore();
    this.drawOutlinedText("교차로", x + 12, y + 21, 14, { color: "rgba(255,255,255,.72)" });
    this.drawOutlinedText(formatDistance(crossroad.distanceM), x + 12, y + 42, 18, { maxWidth: boxWidth - 24 });
    return true;
  }

  drawCrossroadOverlay(width, height, padding, compact, allowFallback = true) {
    const candidates = ["crossroad_expanded", "crossroad_minimized"];
    for (const name of candidates) {
      if (!this.image(name)) continue;
      const expanded = name === "crossroad_expanded";
      const slot = expanded ? {
        x: width * (compact ? 0.46 : 0.42),
        y: Math.max(padding, height * (compact ? 0.2 : 0.16)),
        width: Math.max(1, width * (compact ? 0.54 : 0.58) - padding),
        height: height * (compact ? 0.58 : 0.62),
      } : {
        x: width * 0.7,
        y: Math.max(padding, height * 0.16),
        width: Math.max(1, width * 0.3 - padding),
        height: height * 0.48,
      };
      const bounds = this.drawImage(name, slot, { alignX: 1, alignY: 0.5 });
      if (bounds) return { name, bounds };
    }
    if (allowFallback) this.drawCrossroadFallback(width, height, padding);
    return null;
  }

  drawNavigationStatus(width, height, padding, allowNavigationFallback = true) {
    if (allowNavigationFallback && this.state?.navigationStatus?.offRoute) {
      const boxWidth = Math.min(width * 0.38, 220);
      const x = (width - boxWidth) * 0.5;
      const context = this.context;
      context.save();
      roundedRect(context, x, padding, boxWidth, 38, 19);
      context.fillStyle = "rgba(151, 72, 14, .9)";
      context.fill();
      context.restore();
      this.drawOutlinedText("경로 이탈 · 재탐색 중", width * 0.5, padding + 25, 14, { align: "center" });
    }
    const status = this.displayStatus;
    if (!status?.mode || status.mode === "live") return;
    const labels = {
      disconnected: ["내비 연결 끊김", "TMAP 수신을 기다리는 중"],
      stalled: ["지도 영상 멈춤", "수신 복구를 시도하는 중"],
      waiting: ["지도 영상 대기 중", "첫 화면을 기다리는 중"],
    };
    const [title, detail] = labels[status.mode] || [String(status.title || "내비 상태 확인"), String(status.detail || "")];
    const panelWidth = Math.min(width * 0.56, 330);
    const panelHeight = 92;
    const x = (width - panelWidth) * 0.5;
    const y = (height - panelHeight) * 0.5;
    const context = this.context;
    context.save();
    roundedRect(context, x, y, panelWidth, panelHeight, 16);
    context.fillStyle = "rgba(5, 9, 14, .9)";
    context.fill();
    context.strokeStyle = status.mode === "stalled" ? "#ffb34d" : "rgba(255,255,255,.24)";
    context.stroke();
    context.restore();
    this.drawOutlinedText(title, width * 0.5, y + 38, 20, { align: "center", color: status.mode === "stalled" ? "#ffbd68" : "#fff" });
    this.drawOutlinedText(detail, width * 0.5, y + 66, 13, { align: "center", color: "rgba(255,255,255,.68)" });
  }

  drawRouteProgress(width, height, padding) {
    const route = this.state?.route;
    if (!recordPresent(route) || finite(route.totalDistanceM) <= 0) return;
    const progressWidth = Math.min(width * 0.42, 260);
    const progress = clamp(finite(route.movedDistanceM) / finite(route.totalDistanceM), 0, 1);
    const context = this.context;
    context.save();
    roundedRect(context, padding, height - 7, progressWidth, 3, 2);
    context.fillStyle = "rgba(255,255,255,.2)";
    context.fill();
    if (progress > 0) {
      roundedRect(context, padding, height - 7, progressWidth * progress, 3, 2);
      context.fillStyle = "#4ad89c";
      context.fill();
    }
    context.restore();
  }

  render() {
    const context = this.context;
    const canvas = this.canvas;
    if (!context || !canvas) return;
    const rectangle = canvas.getBoundingClientRect();
    const cssWidth = Math.max(1, Math.round(rectangle.width));
    const cssHeight = Math.max(1, Math.round(rectangle.height));
    const dpr = clamp(window.devicePixelRatio || 1, 1, DPR_LIMIT);
    const targetWidth = Math.max(1, Math.round(cssWidth * dpr));
    const targetHeight = Math.max(1, Math.round(cssHeight * dpr));
    if (canvas.width !== targetWidth || canvas.height !== targetHeight) {
      canvas.width = targetWidth;
      canvas.height = targetHeight;
    }
    context.setTransform(dpr, 0, 0, dpr, 0, 0);
    context.clearRect(0, 0, cssWidth, cssHeight);

    const fallbackPadding = clamp(cssWidth * 0.022, 9, 16);
    const mediaInset = clamp(cssWidth * 0.015, 8, 14);
    const bottomInset = clamp(cssHeight * 0.015, 6, 10);
    const nativeImages = this.nativeImageSession;
    const compact = cssWidth < 520;
    // Only the upper-left TBT stack follows TMAP's edge-attached layout.
    // Other media and synthesized footer text keep the same safe insets as
    // carrot-wip's live navigation panel.
    this.drawGuidanceStack(cssWidth, cssHeight, 0, compact, !nativeImages);

    const trafficDrawn = this.drawImage("traffic_signal", {
      x: cssWidth - mediaInset - Math.min(cssWidth * 0.32, 230),
      y: mediaInset,
      width: Math.min(cssWidth * 0.32, 230),
      height: Math.min(cssHeight * 0.17, 98),
    }, { alignX: 1, alignY: 0 });
    if (!trafficDrawn && !nativeImages) this.drawTrafficFallback(cssWidth, fallbackPadding);

    this.drawCrossroadOverlay(cssWidth, cssHeight, mediaInset, compact, !nativeImages);

    if (!compact) {
      const centerX = cssWidth * 0.5;
      this.drawImage("center_tbt_icon", {
        x: centerX - 112,
        y: cssHeight * 0.16,
        width: 92,
        height: 92,
      }, { alignX: 1 });
      this.drawImage("center_tbt_text", {
        x: centerX - 12,
        y: cssHeight * 0.17,
        width: Math.min(cssWidth * 0.3, 220),
        height: 78,
      }, { alignX: 0 });
      this.drawImage("center_tbt_fee", {
        x: centerX - 90,
        y: cssHeight * 0.33,
        width: 180,
        height: 58,
      });
    }
    const laneTopDrawn = this.drawImage("lane_top", {
      x: cssWidth * 0.23,
      y: cssHeight * 0.3,
      width: cssWidth * 0.54,
      height: Math.min(cssHeight * 0.15, 86),
    });

    const safetyName = ["safety_primary", "safety_secondary", "safety_section"].find((name) => this.image(name));
    const safetyDrawn = safetyName ? this.drawImage(safetyName, {
      x: mediaInset,
      y: cssHeight * 0.55,
      width: Math.min(cssWidth * 0.25, 130),
      height: Math.min(cssHeight * 0.23, 112),
    }, { alignX: 0 }) : false;
    if (!safetyDrawn && !nativeImages) this.drawSafetyFallback(cssWidth, cssHeight, fallbackPadding);

    const laneWidth = Math.min(cssWidth * (compact ? 0.54 : 0.46), 270);
    const laneHeight = Math.min(cssHeight * 0.18, 96);
    const laneSlot = {
      x: (cssWidth - laneWidth) * 0.5,
      y: cssHeight - laneHeight - bottomInset,
      width: laneWidth,
      height: laneHeight,
    };
    const laneBottomBounds = this.drawImage("lane_bottom", laneSlot);
    if (!laneTopDrawn && !laneBottomBounds && !nativeImages) this.drawLaneFallback(cssWidth, cssHeight);

    const route = this.footerState?.route;
    const laneLeft = laneBottomBounds?.x ?? laneSlot.x;
    const laneRight = laneBottomBounds ? laneBottomBounds.x + laneBottomBounds.width : laneSlot.x + laneSlot.width;
    if (recordPresent(route)) {
      const minutes = route.remainingTimeSec > 0 ? Math.max(1, Math.round(route.remainingTimeSec / 60)) : 0;
      const arrival = formatClock(route.remainingTimeSec);
      const leftX = mediaInset;
      const leftWidth = Math.max(0, laneLeft - leftX - mediaInset);
      if (leftWidth >= 70) {
        const detail = [arrival, minutes ? `${minutes}분` : ""].filter(Boolean).join(" · ");
        this.drawOutlinedText(formatDistance(route.remainingDistanceM), leftX, cssHeight - bottomInset - 25, clamp(cssWidth * 0.032, 16, 25), {
          maxWidth: leftWidth,
        });
        if (detail) {
          this.drawOutlinedText(detail, leftX, cssHeight - bottomInset, clamp(cssWidth * 0.027, 14, 21), {
            maxWidth: leftWidth,
          });
        }
      }
    }
    if (!nativeImages) this.drawRouteProgress(cssWidth, cssHeight, fallbackPadding);
    const footerVehicle = this.footerState?.vehicle;
    const roadName = footerVehicle?.roadName;
    const rightX = cssWidth - mediaInset;
    const rightWidth = Math.max(0, rightX - laneRight - mediaInset);
    if (recordPresent(footerVehicle) && roadName && rightWidth >= 70) {
      this.drawOutlinedText(roadName, rightX, cssHeight - bottomInset, clamp(cssWidth * 0.028, 13, 20), {
        align: "right",
        maxWidth: rightWidth,
      });
    }
    // Connection/recovery status remains web-owned. Navigation-state UI is
    // synthesized only when the native image channel is unavailable.
    this.drawNavigationStatus(cssWidth, cssHeight, fallbackPadding, !nativeImages);
  }
}

export function createOverlay(canvas) {
  return new OverlayRenderer(canvas);
}

export const CarrotNaviOverlay = Object.freeze({
  findAlphaRowBounds,
  create: createOverlay,
});

export function installCarrotNaviOverlayGlobal(target = globalThis) {
  target.CarrotNaviOverlay = CarrotNaviOverlay;
  return CarrotNaviOverlay;
}
