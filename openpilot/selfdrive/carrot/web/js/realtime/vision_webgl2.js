"use strict";

window.CarrotVisionWebGL2 = (() => {
  // Projection and triangulation already happen on the UI thread. Moving only
  // the final buffer upload/draw call to an OffscreenCanvas worker adds a
  // message/compositor frame and makes replay geometry trail the video. Keep
  // the low-latency WebGL submission path on the UI thread.
  const OFFSCREEN_WORKER_ENABLED = false;
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

  function compileShader(gl, type, source) {
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

  function createProgram(gl) {
    const vertex = compileShader(gl, gl.VERTEX_SHADER, VERTEX_SHADER);
    const fragment = compileShader(gl, gl.FRAGMENT_SHADER, FRAGMENT_SHADER);
    const program = gl.createProgram();
    gl.attachShader(program, vertex);
    gl.attachShader(program, fragment);
    gl.linkProgram(program);
    gl.deleteShader(vertex);
    gl.deleteShader(fragment);
    if (!gl.getProgramParameter(program, gl.LINK_STATUS)) {
      const detail = gl.getProgramInfoLog(program) || "program link failed";
      gl.deleteProgram(program);
      throw new Error(detail);
    }
    return program;
  }

  function supportsOffscreenWorker() {
    if (typeof Worker !== "function" || typeof OffscreenCanvas !== "function") return false;
    if (typeof HTMLCanvasElement === "undefined"
        || typeof HTMLCanvasElement.prototype?.transferControlToOffscreen !== "function") return false;
    try {
      const probe = new OffscreenCanvas(1, 1);
      const gl = probe.getContext("webgl2");
      if (!gl) return false;
      gl.getExtension("WEBGL_lose_context")?.loseContext?.();
      return true;
    } catch {
      return false;
    }
  }

  function parseColor(style) {
    if (style && typeof style === "object" && style.type === "vertical-gradient") return style;
    const match = String(style || "").match(/rgba?\(\s*([\d.]+)\s*,\s*([\d.]+)\s*,\s*([\d.]+)(?:\s*,\s*([\d.]+))?\s*\)/i);
    if (!match) return { r: 1, g: 1, b: 1, a: 0 };
    return {
      r: Math.max(0, Math.min(255, Number(match[1]))) / 255,
      g: Math.max(0, Math.min(255, Number(match[2]))) / 255,
      b: Math.max(0, Math.min(255, Number(match[3]))) / 255,
      a: Math.max(0, Math.min(1, match[4] == null ? 1 : Number(match[4]))),
    };
  }

  function signedArea(points) {
    let area = 0;
    for (let i = 0; i < points.length; i += 1) {
      const a = points[i];
      const b = points[(i + 1) % points.length];
      area += a.x * b.y - b.x * a.y;
    }
    return area * 0.5;
  }

  function cross(a, b, c) {
    return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
  }

  function pointInTriangle(point, a, b, c) {
    const ab = cross(a, b, point);
    const bc = cross(b, c, point);
    const ca = cross(c, a, point);
    const hasNegative = ab < -1e-5 || bc < -1e-5 || ca < -1e-5;
    const hasPositive = ab > 1e-5 || bc > 1e-5 || ca > 1e-5;
    return !(hasNegative && hasPositive);
  }

  function triangulate(input) {
    const points = input.filter((point) => Number.isFinite(point?.x) && Number.isFinite(point?.y));
    if (points.length < 3) return { points, indices: [] };
    if (points.length >= 4 && points.length % 2 === 0) {
      const half = points.length / 2;
      const ribbonTriangles = [];
      for (let index = 0; index < half - 1; index += 1) {
        const right = points.length - 1 - index;
        const rightNext = right - 1;
        ribbonTriangles.push(index, index + 1, rightNext, index, rightNext, right);
      }
      return { points, indices: ribbonTriangles };
    }
    const indices = points.map((_, index) => index);
    if (signedArea(points) < 0) indices.reverse();
    const triangles = [];
    let guard = points.length * points.length;

    while (indices.length > 3 && guard-- > 0) {
      let clipped = false;
      for (let i = 0; i < indices.length; i += 1) {
        const previous = indices[(i + indices.length - 1) % indices.length];
        const current = indices[i];
        const next = indices[(i + 1) % indices.length];
        const a = points[previous];
        const b = points[current];
        const c = points[next];
        if (cross(a, b, c) <= 1e-5) continue;

        let containsPoint = false;
        for (const candidate of indices) {
          if (candidate === previous || candidate === current || candidate === next) continue;
          if (pointInTriangle(points[candidate], a, b, c)) {
            containsPoint = true;
            break;
          }
        }
        if (containsPoint) continue;
        triangles.push(previous, current, next);
        indices.splice(i, 1);
        clipped = true;
        break;
      }
      if (!clipped) break;
    }
    if (indices.length === 3) triangles.push(indices[0], indices[1], indices[2]);
    if (!triangles.length) {
      for (let i = 1; i < points.length - 1; i += 1) triangles.push(0, i, i + 1);
    }
    return { points, indices: triangles };
  }

  class Renderer {
    constructor(canvas) {
      this.canvas = canvas;
      this.gl = null;
      this.program = null;
      this.positionBuffer = null;
      this.colorBuffer = null;
      this.positions = [];
      this.colors = [];
      this.logicalWidth = 1;
      this.logicalHeight = 1;
      this.pixelWidth = Math.max(1, canvas.width || 1);
      this.pixelHeight = Math.max(1, canvas.height || 1);
      this.contextLost = false;
      this.failed = false;
      this.worker = null;
      this.workerPending = false;
      this.workerReady = false;
      this.workerFrameInFlight = false;
      this.workerPendingFrame = null;
      this.offscreen = null;
      this.offscreenTransferred = false;
      canvas.addEventListener("webglcontextlost", (event) => {
        event.preventDefault();
        this.contextLost = true;
      });
      canvas.addEventListener("webglcontextrestored", () => {
        this.contextLost = false;
        this.failed = false;
        this.initialize(true);
      });
    }

    initialize(force = false) {
      if (this.failed || this.contextLost) return false;
      if (this.offscreen == null) this.offscreen = OFFSCREEN_WORKER_ENABLED && supportsOffscreenWorker();
      if (this.offscreen) {
        if (this.workerReady) return true;
        if (this.workerPending) return false;
        try {
          this.workerPending = true;
          this.worker = new Worker("/js/realtime/vision_webgl2_worker.js?v=2607-02");
          this.worker.onmessage = (event) => {
            if (event.data?.type === "ready") {
              this.workerPending = false;
              this.workerReady = true;
              window.dispatchEvent(new CustomEvent("carrot:render-request", {
                detail: { force: true, overlayDirty: true, hudDirty: false },
              }));
            } else if (event.data?.type === "drawn") {
              this.workerFrameInFlight = false;
              if (this.workerPendingFrame) {
                const nextFrame = this.workerPendingFrame;
                this.workerPendingFrame = null;
                this.postWorkerFrame(nextFrame);
              }
            } else if (event.data?.type === "error") {
              this.failWorker(event.data?.error || "worker initialization failed");
            }
          };
          this.worker.onerror = (event) => this.failWorker(event?.message || "worker failed");
          this.pixelWidth = Math.max(1, this.canvas.width);
          this.pixelHeight = Math.max(1, this.canvas.height);
          const offscreenCanvas = this.canvas.transferControlToOffscreen();
          this.offscreenTransferred = true;
          this.worker.postMessage({ type: "init", canvas: offscreenCanvas }, [offscreenCanvas]);
        } catch (error) {
          this.failWorker(error?.message || error);
        }
        return false;
      }
      if (this.program && !force) return true;
      try {
        const gl = this.canvas.getContext("webgl2", {
          alpha: true,
          antialias: true,
          depth: false,
          stencil: false,
          // Canvas2D is composited as premultiplied alpha. Matching that here
          // keeps translucent lane/path fills visually consistent between
          // compatibility and performance modes.
          premultipliedAlpha: true,
          preserveDrawingBuffer: false,
          powerPreference: "high-performance",
        });
        if (!gl) {
          this.failed = true;
          return false;
        }
        this.gl = gl;
        this.program = createProgram(gl);
        this.positionBuffer = gl.createBuffer();
        this.colorBuffer = gl.createBuffer();
        return true;
      } catch (error) {
        this.failed = true;
        console.warn("[vision webgl2] initialization failed", error);
        return false;
      }
    }

    failWorker(error) {
      this.workerPending = false;
      this.workerReady = false;
      this.workerFrameInFlight = false;
      this.workerPendingFrame = null;
      this.failed = true;
      try { this.worker?.terminate?.(); } catch {}
      this.worker = null;
      console.warn("[vision webgl2] offscreen worker disabled", error);
      window.dispatchEvent(new CustomEvent("carrot:render-request", {
        detail: { force: true, overlayDirty: true, hudDirty: false },
      }));
    }

    ownsOffscreenCanvas() {
      return this.offscreenTransferred;
    }

    resize(pixelWidth, pixelHeight) {
      if (!this.ownsOffscreenCanvas()) return;
      this.pixelWidth = Math.max(1, Number(pixelWidth) || 1);
      this.pixelHeight = Math.max(1, Number(pixelHeight) || 1);
      this.worker?.postMessage?.({
        type: "resize",
        width: this.pixelWidth,
        height: this.pixelHeight,
      });
    }

    beginFrame(width, height) {
      if (!this.initialize()) return false;
      const gl = this.gl;
      this.logicalWidth = Math.max(1, Number(width) || 1);
      this.logicalHeight = Math.max(1, Number(height) || 1);
      this.positions.length = 0;
      this.colors.length = 0;
      if (this.workerReady) return true;
      this.pixelWidth = Math.max(1, this.canvas.width);
      this.pixelHeight = Math.max(1, this.canvas.height);
      gl.viewport(0, 0, this.canvas.width, this.canvas.height);
      gl.clearColor(0, 0, 0, 0);
      gl.clear(gl.COLOR_BUFFER_BIT);
      return true;
    }

    colorAt(style, y) {
      const color = parseColor(style);
      if (color.type !== "vertical-gradient") return color;
      const base = color.color || { r: 255, g: 255, b: 255 };
      const stop = Math.max(0, Math.min(1, (this.logicalHeight - y) / (this.logicalHeight * 0.68)));
      let alpha;
      if (stop <= 0.55) {
        alpha = color.bottomAlpha + (color.midAlpha - color.bottomAlpha) * (stop / 0.55);
      } else {
        alpha = color.midAlpha * (1 - ((stop - 0.55) / 0.45));
      }
      return { r: base.r / 255, g: base.g / 255, b: base.b / 255, a: Math.max(0, Math.min(1, alpha)) };
    }

    drawPolygon(input, fillStyle) {
      if (!fillStyle || (!this.program && !this.workerReady)) return;
      const { points, indices } = triangulate(input);
      for (const index of indices) {
        const point = points[index];
        const color = this.colorAt(fillStyle, point.y);
        this.positions.push(point.x, point.y);
        this.colors.push(color.r, color.g, color.b, color.a);
      }
    }

    postWorkerFrame(frame) {
      if (!this.workerReady || !this.worker) return;
      this.workerFrameInFlight = true;
      this.worker.postMessage(frame, [frame.positions, frame.colors]);
    }

    endFrame() {
      if (this.workerReady && this.worker) {
        const positions = new Float32Array(this.positions);
        const colors = new Float32Array(this.colors);
        const frame = {
          type: "draw",
          logicalWidth: this.logicalWidth,
          logicalHeight: this.logicalHeight,
          pixelWidth: this.pixelWidth,
          pixelHeight: this.pixelHeight,
          positions: positions.buffer,
          colors: colors.buffer,
        };
        if (this.workerFrameInFlight) this.workerPendingFrame = frame;
        else this.postWorkerFrame(frame);
        return true;
      }
      const gl = this.gl;
      if (!gl || !this.program || !this.positions.length) return true;
      gl.useProgram(this.program);
      gl.uniform2f(gl.getUniformLocation(this.program, "u_resolution"), this.logicalWidth, this.logicalHeight);

      const positionLocation = gl.getAttribLocation(this.program, "a_position");
      gl.bindBuffer(gl.ARRAY_BUFFER, this.positionBuffer);
      gl.bufferData(gl.ARRAY_BUFFER, new Float32Array(this.positions), gl.DYNAMIC_DRAW);
      gl.enableVertexAttribArray(positionLocation);
      gl.vertexAttribPointer(positionLocation, 2, gl.FLOAT, false, 0, 0);

      const colorLocation = gl.getAttribLocation(this.program, "a_color");
      gl.bindBuffer(gl.ARRAY_BUFFER, this.colorBuffer);
      gl.bufferData(gl.ARRAY_BUFFER, new Float32Array(this.colors), gl.DYNAMIC_DRAW);
      gl.enableVertexAttribArray(colorLocation);
      gl.vertexAttribPointer(colorLocation, 4, gl.FLOAT, false, 0, 0);

      gl.enable(gl.BLEND);
      gl.blendFunc(gl.SRC_ALPHA, gl.ONE_MINUS_SRC_ALPHA);
      gl.drawArrays(gl.TRIANGLES, 0, this.positions.length / 2);
      return true;
    }

    clear() {
      if (this.workerReady && this.worker) {
        this.workerPendingFrame = null;
        this.worker.postMessage({
          type: "clear",
          width: this.pixelWidth,
          height: this.pixelHeight,
        });
        return;
      }
      if (!this.gl || this.contextLost) return;
      this.gl.viewport(0, 0, this.canvas.width, this.canvas.height);
      this.gl.clearColor(0, 0, 0, 0);
      this.gl.clear(this.gl.COLOR_BUFFER_BIT);
    }
  }

  let renderer = null;
  function getRenderer(canvas) {
    if (!canvas) return null;
    if (!renderer || renderer.canvas !== canvas) renderer = new Renderer(canvas);
    return renderer;
  }

  return { getRenderer };
})();
