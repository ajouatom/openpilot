/* ============================================================================
 * CARROT VISION GRAPHICS - DO NOT CHANGE CASUALLY
 *
 * This is the driving view the user actually looks at. Its smoothness was tuned
 * against real device behaviour and several "obvious" simplifications have
 * already been tried and reverted. If you are here while working on AR, replay
 * or any other feature, prefer adding your own layer over editing this one.
 *
 * Invariants. Breaking any of these brings back stutter, judder or heat:
 *
 *  1. One overlay update per presented video frame. requestVideoFrameCallback
 *     is taken first and unthrottled; the interval constant only paces the
 *     fallback. Never put a timer between a video frame and its overlay.
 *  2. Live and replay share one scheduler, one filter and one cadence. They are
 *     the same user experience; do not special-case one of them.
 *  3. Temporal smoothing is a time constant, never a per-call alpha. Render
 *     spacing moves constantly, and a fixed alpha makes the response wander.
 *  4. Geometry stays on the GPU: fills, strokes and dashes all land in one
 *     batched draw call. Moving any of them back to Canvas2D reintroduces a
 *     full-surface raster and its clear every frame.
 *  5. The 2D overlay is cleared only when something actually drew to it. If you
 *     add a direct ctx draw, mark that layer dirty or you will ship ghosting.
 *  6. Vertex pools are reused. Do not rebuild per-frame arrays.
 *
 * Already tried and rejected:
 *  - Overlay projection/triangulation in a worker: the extra hop makes geometry
 *    trail the video by a frame (see OFFSCREEN_WORKER_ENABLED).
 *  - Interpolating geometry between 20 Hz samples: the video is 20 fps, so the
 *    overlay slides against a still image.
 *  - Uniform Catmull-Rom resampling: overshoots on unevenly spaced projected
 *    points and draws streaks across the frame. Centripetal only.
 *
 * Measured state: worst curve kink 8.1deg -> 1.6deg, geometry fully GPU-batched,
 * per-frame 2D clear removed.
 * ==========================================================================*/

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
      this.positionData = null;
      this.colorData = null;
      this.vertexCount = 0;
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
      this.vertexCount = 0;
      if (this.workerReady) return true;
      this.pixelWidth = Math.max(1, this.canvas.width);
      this.pixelHeight = Math.max(1, this.canvas.height);
      gl.viewport(0, 0, this.canvas.width, this.canvas.height);
      gl.clearColor(0, 0, 0, 0);
      gl.clear(gl.COLOR_BUFFER_BIT);
      return true;
    }

    /* Vertex pools reused across frames.
     *
     * These used to be plain arrays rebuilt by push() and then copied into a
     * fresh Float32Array on every frame. With fills, strokes and dashes all
     * batched here that is hundreds of KB of garbage per frame, and the GC
     * pauses land as visible hitches. Grow-once pools keep the steady state
     * allocation-free. */
    ensureVertexCapacity(additional) {
      const needed = this.vertexCount + additional;
      if (!this.positionData || this.positionData.length < needed * 2) {
        let capacity = Math.max(1024, this.positionData ? this.positionData.length / 2 : 0);
        while (capacity < needed) capacity *= 2;
        const positions = new Float32Array(capacity * 2);
        const colors = new Float32Array(capacity * 4);
        if (this.positionData) {
          positions.set(this.positionData.subarray(0, this.vertexCount * 2));
          colors.set(this.colorData.subarray(0, this.vertexCount * 4));
        }
        this.positionData = positions;
        this.colorData = colors;
      }
    }

    pushVertex(x, y, color) {
      const p = this.vertexCount * 2;
      const c = this.vertexCount * 4;
      this.positionData[p] = x;
      this.positionData[p + 1] = y;
      this.colorData[c] = color.r;
      this.colorData[c + 1] = color.g;
      this.colorData[c + 2] = color.b;
      this.colorData[c + 3] = color.a;
      this.vertexCount += 1;
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
      this.ensureVertexCapacity(indices.length);
      for (const index of indices) {
        const point = points[index];
        this.pushVertex(point.x, point.y, this.colorAt(fillStyle, point.y));
      }
    }

    /* Stroke a polyline into the same triangle batch as the fills.
     *
     * Strokes used to stay on Canvas2D while only fills moved to the GPU. That
     * hybrid kept a full-surface 2D raster alive every frame, which is the CPU
     * and thermal cost this renderer exists to avoid. Emitting the stroke as
     * geometry costs no extra draw call: it lands in the batch endFrame()
     * already uploads once.
     *
     * Joins use a clamped miter so the quads share edges instead of
     * overlapping. Overlapping translucent quads would double-blend and show a
     * darker knot at every vertex.
     */
    drawStroke(input, strokeStyle, lineWidth, closed = true) {
      if (!strokeStyle || (!this.program && !this.workerReady)) return;
      const source = Array.isArray(input) ? input : input?.points;
      if (!Array.isArray(source) || source.length < 2) return;
      const half = Math.max(0.25, Number(lineWidth) || 1) / 2;

      const path = source.filter((point) => Number.isFinite(point?.x) && Number.isFinite(point?.y));
      if (path.length < 2) return;
      if (closed) {
        const first = path[0];
        const last = path[path.length - 1];
        if (Math.hypot(last.x - first.x, last.y - first.y) > 1e-6) path.push(first);
      }

      const at = (index) => path[Math.min(Math.max(index, 0), path.length - 1)];
      const normalOf = (a, b) => {
        const dx = b.x - a.x;
        const dy = b.y - a.y;
        const length = Math.hypot(dx, dy);
        if (!(length > 1e-9)) return null;
        return { x: -dy / length, y: dx / length };
      };

      const offsets = [];
      for (let i = 0; i < path.length; i += 1) {
        const before = normalOf(at(i - 1), path[i]);
        const after = normalOf(path[i], at(i + 1));
        const a = before || after;
        const b = after || before;
        if (!a || !b) { offsets.push({ x: 0, y: 0 }); continue; }
        let mx = a.x + b.x;
        let my = a.y + b.y;
        const length = Math.hypot(mx, my);
        if (!(length > 1e-9)) { offsets.push({ x: a.x * half, y: a.y * half }); continue; }
        mx /= length;
        my /= length;
        // 1 / cos(theta/2); clamped so a hairpin cannot shoot a spike across the frame.
        const scale = Math.min(1 / Math.max(mx * a.x + my * a.y, 0.25), 4);
        offsets.push({ x: mx * half * scale, y: my * half * scale });
      }

      for (let i = 0; i < path.length - 1; i += 1) {
        const p0 = path[i];
        const p1 = path[i + 1];
        const o0 = offsets[i];
        const o1 = offsets[i + 1];
        const quad = [
          { x: p0.x + o0.x, y: p0.y + o0.y },
          { x: p1.x + o1.x, y: p1.y + o1.y },
          { x: p1.x - o1.x, y: p1.y - o1.y },
          { x: p0.x + o0.x, y: p0.y + o0.y },
          { x: p1.x - o1.x, y: p1.y - o1.y },
          { x: p0.x - o0.x, y: p0.y - o0.y },
        ];
        this.ensureVertexCapacity(quad.length);
        for (const point of quad) {
          this.pushVertex(point.x, point.y, this.colorAt(strokeStyle, point.y));
        }
      }
    }

    postWorkerFrame(frame) {
      if (!this.workerReady || !this.worker) return;
      this.workerFrameInFlight = true;
      this.worker.postMessage(frame, [frame.positions, frame.colors]);
    }

    endFrame() {
      if (this.workerReady && this.worker) {
        // Transfer detaches the buffer, so the pool must not be handed over.
        const positions = this.positionData.slice(0, this.vertexCount * 2);
        const colors = this.colorData.slice(0, this.vertexCount * 4);
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
      if (!gl || !this.program || !this.vertexCount) return true;
      gl.useProgram(this.program);
      /* Attribute and uniform lookups are string queries into the driver. They
       * never change for a linked program, so resolve them once instead of on
       * every frame. */
      if (this.resolutionLocation === undefined) {
        this.resolutionLocation = gl.getUniformLocation(this.program, "u_resolution");
        this.positionLocation = gl.getAttribLocation(this.program, "a_position");
        this.colorLocation = gl.getAttribLocation(this.program, "a_color");
      }
      gl.uniform2f(this.resolutionLocation, this.logicalWidth, this.logicalHeight);

      const positionLocation = this.positionLocation;
      gl.bindBuffer(gl.ARRAY_BUFFER, this.positionBuffer);
      gl.bufferData(gl.ARRAY_BUFFER, this.positionData.subarray(0, this.vertexCount * 2), gl.DYNAMIC_DRAW);
      gl.enableVertexAttribArray(positionLocation);
      gl.vertexAttribPointer(positionLocation, 2, gl.FLOAT, false, 0, 0);

      const colorLocation = this.colorLocation;
      gl.bindBuffer(gl.ARRAY_BUFFER, this.colorBuffer);
      gl.bufferData(gl.ARRAY_BUFFER, this.colorData.subarray(0, this.vertexCount * 4), gl.DYNAMIC_DRAW);
      gl.enableVertexAttribArray(colorLocation);
      gl.vertexAttribPointer(colorLocation, 4, gl.FLOAT, false, 0, 0);

      gl.enable(gl.BLEND);
      gl.blendFunc(gl.SRC_ALPHA, gl.ONE_MINUS_SRC_ALPHA);
      gl.drawArrays(gl.TRIANGLES, 0, this.vertexCount);
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
