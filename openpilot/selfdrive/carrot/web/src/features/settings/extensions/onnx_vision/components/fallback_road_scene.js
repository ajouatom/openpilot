export function fallbackRoadSceneMarkup() {
  return `
    <svg class="onnx-fallback-road" data-role="fallback-road" aria-hidden="true" preserveAspectRatio="none">
      <rect class="onnx-fallback-road__sky" data-fallback-part="sky"></rect>
      <rect class="onnx-fallback-road__ground" data-fallback-part="ground"></rect>
      <path class="onnx-fallback-road__horizon" data-fallback-part="horizon"></path>
      <path class="onnx-fallback-road__road" data-fallback-part="road"></path>
      <path class="onnx-fallback-road__shoulder" data-fallback-part="shoulders"></path>
      <path class="onnx-fallback-road__lane" data-fallback-part="lanes"></path>
    </svg>
  `;
}

export function layoutFallbackRoadScene(root, width, height) {
  if (!root) return;
  const w = Math.max(1, Number(width) || 1);
  const h = Math.max(1, Number(height) || 1);
  const horizon = 0;
  const center = w * 0.5;
  const roadTopHalf = w * 0.055;
  const roadBottomHalf = w * 0.5;
  const laneTop = w * 0.018;
  const laneBottom = w * 0.15;
  const part = (name) => root.querySelector(`[data-fallback-part="${name}"]`);

  root.setAttribute?.("viewBox", `0 0 ${w} ${h}`);
  Object.entries({ sky: [0, 0, w, horizon], ground: [0, horizon, w, h - horizon] }).forEach(([name, values]) => {
    const node = part(name);
    ["x", "y", "width", "height"].forEach((attribute, index) => node?.setAttribute(attribute, String(values[index])));
  });
  part("horizon")?.setAttribute("d", `M 0 ${horizon} H ${w}`);
  part("road")?.setAttribute("d", `M ${center - roadTopHalf} ${horizon} L ${center + roadTopHalf} ${horizon} L ${center + roadBottomHalf} ${h} L ${center - roadBottomHalf} ${h} Z`);
  part("shoulders")?.setAttribute("d", `M ${center - roadTopHalf} ${horizon} L ${center - roadBottomHalf} ${h} M ${center + roadTopHalf} ${horizon} L ${center + roadBottomHalf} ${h}`);
  part("lanes")?.setAttribute("d", [
    `M ${center} ${horizon} L ${center} ${h}`,
    `M ${center - laneTop} ${horizon} L ${center - laneBottom} ${h}`,
    `M ${center + laneTop} ${horizon} L ${center + laneBottom} ${h}`,
  ].join(" "));
}
