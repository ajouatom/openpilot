const DPR_LIMIT = 2;
const TBT_CURRENT_BOTTOM_TRANSPARENT_PX = 24;
const TBT_STACK_OVERLAP_PX = 6;
const TBT_ALPHA_VISIBLE_THRESHOLD = 12;

function clamp(value, minimum, maximum) {
  return Math.max(minimum, Math.min(maximum, value));
}

function finite(value, fallback = 0) {
  const number = Number(value);
  return Number.isFinite(number) ? number : fallback;
}

function recordPresent(value) {
  return Boolean(value?.meta?.present);
}

function findAlphaRowBounds(data, widthValue, heightValue, thresholdValue = TBT_ALPHA_VISIBLE_THRESHOLD) {
  const width = Math.max(0, Math.floor(finite(widthValue)));
  const height = Math.max(0, Math.floor(finite(heightValue)));
  const threshold = clamp(Math.floor(finite(thresholdValue, TBT_ALPHA_VISIBLE_THRESHOLD)), 0, 255);
  if (!data || width < 1 || height < 1 || data.length < width * height * 4) return null;

  let top = height;
  let bottom = -1;
  for (let y = 0; y < height; y += 1) {
    const rowStart = y * width * 4 + 3;
    const rowEnd = rowStart + width * 4;
    for (let offset = rowStart; offset < rowEnd; offset += 4) {
      if (data[offset] <= threshold) continue;
      top = Math.min(top, y);
      bottom = Math.max(bottom, y);
      break;
    }
  }
  return bottom >= top ? { top, bottom: bottom + 1 } : null;
}

function formatDistance(meters) {
  const value = Math.max(0, finite(meters));
  if (value >= 1000) {
    const kilometers = value / 1000;
    return `${kilometers >= 10 ? kilometers.toFixed(0) : kilometers.toFixed(1)}km`;
  }
  return `${Math.round(value)}m`;
}

function formatClock(secondsFromNow) {
  const seconds = Math.max(0, finite(secondsFromNow));
  if (!seconds) return "";
  const arrival = new Date(Date.now() + seconds * 1000);
  return arrival.toLocaleTimeString([], { hour: "2-digit", minute: "2-digit", hour12: false });
}

function roundedRect(context, x, y, width, height, radius) {
  const r = Math.min(Math.max(0, radius), width * 0.5, height * 0.5);
  context.beginPath();
  context.moveTo(x + r, y);
  context.arcTo(x + width, y, x + width, y + height, r);
  context.arcTo(x + width, y + height, x, y + height, r);
  context.arcTo(x, y + height, x, y, r);
  context.arcTo(x, y, x + width, y, r);
  context.closePath();
}

const overlayPrimitives = Object.freeze({
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
});

function installCarrotNaviOverlayPrimitivesGlobal(target = globalThis) {
  target.CarrotNaviOverlayPrimitives = overlayPrimitives;
  return overlayPrimitives;
}

export {
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
  overlayPrimitives,
  installCarrotNaviOverlayPrimitivesGlobal,
};
