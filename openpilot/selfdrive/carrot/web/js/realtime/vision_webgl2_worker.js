"use strict";

let canvas = null;
let gl = null;
let program = null;
let positionBuffer = null;
let colorBuffer = null;
let resolutionLocation = null;
let positionLocation = -1;
let colorLocation = -1;

const VERTEX_SHADER = `#version 300 es
  in vec2 a_position;
  in vec4 a_color;
  uniform vec2 u_resolution;
  out vec4 v_color;
  void main() {
    vec2 zeroToOne = a_position / u_resolution;
    vec2 clip = zeroToOne * 2.0 - 1.0;
    gl_Position = vec4(clip.x, -clip.y, 0.0, 1.0);
    v_color = a_color;
  }
`;

const FRAGMENT_SHADER = `#version 300 es
  precision highp float;
  in vec4 v_color;
  out vec4 outColor;
  void main() {
    outColor = v_color;
  }
`;

function compileShader(type, source) {
  const shader = gl.createShader(type);
  gl.shaderSource(shader, source);
  gl.compileShader(shader);
  if (!gl.getShaderParameter(shader, gl.COMPILE_STATUS)) {
    const detail = gl.getShaderInfoLog(shader) || "shader compile failed";
    gl.deleteShader(shader);
    throw new Error(detail);
  }
  return shader;
}

function initialize(offscreenCanvas) {
  canvas = offscreenCanvas;
  gl = canvas.getContext("webgl2", {
    alpha: true,
    antialias: true,
    depth: false,
    stencil: false,
    premultipliedAlpha: true,
    preserveDrawingBuffer: false,
    powerPreference: "high-performance",
  });
  if (!gl) throw new Error("WebGL2 is unavailable in OffscreenCanvas");

  const vertex = compileShader(gl.VERTEX_SHADER, VERTEX_SHADER);
  const fragment = compileShader(gl.FRAGMENT_SHADER, FRAGMENT_SHADER);
  program = gl.createProgram();
  gl.attachShader(program, vertex);
  gl.attachShader(program, fragment);
  gl.linkProgram(program);
  gl.deleteShader(vertex);
  gl.deleteShader(fragment);
  if (!gl.getProgramParameter(program, gl.LINK_STATUS)) {
    throw new Error(gl.getProgramInfoLog(program) || "program link failed");
  }

  positionBuffer = gl.createBuffer();
  colorBuffer = gl.createBuffer();
  resolutionLocation = gl.getUniformLocation(program, "u_resolution");
  positionLocation = gl.getAttribLocation(program, "a_position");
  colorLocation = gl.getAttribLocation(program, "a_color");
  self.postMessage({ type: "ready" });
}

function resize(width, height) {
  if (!canvas) return;
  const nextWidth = Math.max(1, Number(width) || 1);
  const nextHeight = Math.max(1, Number(height) || 1);
  if (canvas.width !== nextWidth) canvas.width = nextWidth;
  if (canvas.height !== nextHeight) canvas.height = nextHeight;
}

function clear(width, height) {
  if (!gl) return;
  resize(width, height);
  gl.viewport(0, 0, canvas.width, canvas.height);
  gl.clearColor(0, 0, 0, 0);
  gl.clear(gl.COLOR_BUFFER_BIT);
}

function draw(data) {
  if (!gl || !program) return;
  clear(data.pixelWidth, data.pixelHeight);
  const positions = new Float32Array(data.positions);
  if (!positions.length) return;
  const colors = new Float32Array(data.colors);

  gl.useProgram(program);
  gl.uniform2f(resolutionLocation, Math.max(1, data.logicalWidth), Math.max(1, data.logicalHeight));
  gl.bindBuffer(gl.ARRAY_BUFFER, positionBuffer);
  gl.bufferData(gl.ARRAY_BUFFER, positions, gl.DYNAMIC_DRAW);
  gl.enableVertexAttribArray(positionLocation);
  gl.vertexAttribPointer(positionLocation, 2, gl.FLOAT, false, 0, 0);

  gl.bindBuffer(gl.ARRAY_BUFFER, colorBuffer);
  gl.bufferData(gl.ARRAY_BUFFER, colors, gl.DYNAMIC_DRAW);
  gl.enableVertexAttribArray(colorLocation);
  gl.vertexAttribPointer(colorLocation, 4, gl.FLOAT, false, 0, 0);

  gl.enable(gl.BLEND);
  gl.blendFunc(gl.SRC_ALPHA, gl.ONE_MINUS_SRC_ALPHA);
  gl.drawArrays(gl.TRIANGLES, 0, positions.length / 2);
}

self.onmessage = (event) => {
  try {
    const data = event.data || {};
    if (data.type === "init") initialize(data.canvas);
    else if (data.type === "resize") resize(data.width, data.height);
    else if (data.type === "clear") clear(data.width, data.height);
    else if (data.type === "draw") {
      draw(data);
      self.postMessage({ type: "drawn" });
    }
  } catch (error) {
    self.postMessage({ type: "error", error: String(error?.message || error) });
  }
};
