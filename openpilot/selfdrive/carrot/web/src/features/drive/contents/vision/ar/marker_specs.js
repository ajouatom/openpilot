/* AR 마커 지오메트리 생성기.
 *
 * 렌더러에 종속되지 않는다. 각 함수는 순수 함수이고
 *   { positions: Float32Array, indices: Uint16Array, normals?, uvs?, parts: [...] }
 * 형태의 mesh spec 을 돌려준다. Three.js 어댑터와 기존 WebGL2 오버레이가
 * 같은 spec 을 소비하므로, Three.js 도입 여부를 나중에 바꿔도 모델은 살아남는다.
 *
 * 좌표계: route-local FLU(오른손), 원점은 앵커 지점의 노면.
 *   +x = 진행 방향 전방, +y = 좌측, +z = 위
 * openpilot Device/Calibrated FRD와 같은 frame이 아니다. 최종 투영 경계가
 * FLU→FRD를 정확히 한 번 적용한 뒤 calibration을 사용한다.
 */

import { AR_GEOMETRY, AR_MARKER_KIND, arClamp, resolveLaneWidthM } from "./tokens.js";

function meshBuilder() {
  const positions = [];
  const indices = [];
  const parts = [];

  function vertex(x, y, z) {
    positions.push(x, y, z);
    return positions.length / 3 - 1;
  }

  function triangle(a, b, c) {
    indices.push(a, b, c);
  }

  function quad(a, b, c, d) {
    indices.push(a, b, c, a, c, d);
  }

  /** 축 정렬 직육면체. 중심(cx,cy,cz), 크기(sx,sy,sz). */
  function box(cx, cy, cz, sx, sy, sz) {
    const hx = sx / 2, hy = sy / 2, hz = sz / 2;
    const v = [
      vertex(cx - hx, cy - hy, cz - hz), vertex(cx + hx, cy - hy, cz - hz),
      vertex(cx + hx, cy + hy, cz - hz), vertex(cx - hx, cy + hy, cz - hz),
      vertex(cx - hx, cy - hy, cz + hz), vertex(cx + hx, cy - hy, cz + hz),
      vertex(cx + hx, cy + hy, cz + hz), vertex(cx - hx, cy + hy, cz + hz),
    ];
    quad(v[4], v[5], v[6], v[7]);   // top(+z)
    quad(v[1], v[0], v[3], v[2]);   // bottom(-z)
    quad(v[0], v[1], v[5], v[4]);   // -y
    quad(v[2], v[3], v[7], v[6]);   // +y
    quad(v[1], v[2], v[6], v[5]);   // +x
    quad(v[3], v[0], v[4], v[7]);   // -x
    return v;
  }

  /** 노면에 눕는 평면 폴리곤(z 고정). points = [[x,y], ...] 볼록 가정. */
  function flatFan(points, z) {
    if (points.length < 3) return [];
    const base = points.map(([x, y]) => vertex(x, y, z));
    for (let i = 1; i < base.length - 1; i += 1) {
      indices.push(base[0], base[i], base[i + 1]);
    }
    return base;
  }

  function mark(name, startIndexCount) {
    parts.push(Object.freeze({ name, start: startIndexCount, count: indices.length - startIndexCount }));
  }

  function build(meta) {
    return Object.freeze({
      ...meta,
      positions: new Float32Array(positions),
      indices: new Uint16Array(indices),
      vertexCount: positions.length / 3,
      triangleCount: indices.length / 3,
      parts: Object.freeze(parts.slice()),
    });
  }

  return { vertex, triangle, quad, box, flatFan, mark, build, indexCount: () => indices.length };
}

/* ── 회전 게이트 ─────────────────────────────────────────────
 * 차선을 가로지르는 문(門). 기둥 2개 + 상단 보 + 방향 셰브런.
 * lod:"flat" 이면 두께 0에 가까운 판으로 만들어 삼각형 수를 줄인다. */
export function buildTurnGate(options = {}) {
  const laneWidth = resolveLaneWidthM(options.laneWidthM);
  const g = AR_GEOMETRY.gate;
  const flat = options.lod === "flat";
  const depth = flat ? 0.02 : g.depthM;
  const width = laneWidth + g.sideMarginM * 2;
  const half = width / 2;
  const height = arClamp(Number(options.heightM) || g.heightM, 2.4, 6.0);
  const chevrons = arClamp(Math.round(options.chevronCount ?? g.chevronCount), 0, 5);
  // -1 = 좌회전, +1 = 우회전, 0 = 직진/불명
  const turnSign = Math.sign(Number(options.turnSign) || 0);

  const m = meshBuilder();
  let cursor = 0;

  // 좌/우 기둥
  cursor = m.indexCount();
  m.box(0, half - g.postWidthM / 2, height / 2, depth, g.postWidthM, height);
  m.box(0, -half + g.postWidthM / 2, height / 2, depth, g.postWidthM, height);
  m.mark("posts", cursor);

  // 상단 보
  cursor = m.indexCount();
  m.box(0, 0, height - g.beamHeightM / 2, depth, width, g.beamHeightM);
  m.mark("beam", cursor);

  // 상단 보 위의 방향 셰브런(회전 방향을 가리킴). 직진이면 생략.
  if (chevrons > 0 && turnSign !== 0) {
    cursor = m.indexCount();
    const chevW = Math.min(0.9, width / (chevrons + 2));
    const z = height - g.beamHeightM / 2;
    for (let i = 0; i < chevrons; i += 1) {
      // 회전 방향 쪽으로 진행하는 화살촉 배치
      const cy = turnSign * (half * 0.45 - i * chevW * 1.15) * -1;
      const tipY = cy + turnSign * chevW * 0.55;
      m.flatFan([
        [depth * 0.6, cy - chevW * 0.30],
        [depth * 0.6, cy + chevW * 0.30],
        [depth * 0.6, tipY],
      ], z);
    }
    m.mark("chevrons", cursor);
  }

  return m.build({
    kind: AR_MARKER_KIND.TURN_GATE,
    lod: flat ? "flat" : "solid",
    boundsM: Object.freeze({ width, height, depth }),
    anchorHeightM: height,
    laneWidthM: laneWidth,
    turnSign,
  });
}

/* ── 진입 화살표 ─────────────────────────────────────────────
 * 노면에 눕는 셰브런 스택. 진행 방향(+x)으로 나아가며 회전 방향으로 휜다. */
export function buildCommitArrow(options = {}) {
  const a = AR_GEOMETRY.arrow;
  const count = arClamp(Math.round(options.chevronCount ?? a.chevronCount), 1, 6);
  const width = arClamp(Number(options.widthM) || a.chevronWidthM, 1.0, 3.4);
  const spacing = arClamp(Number(options.spacingM) || a.spacingM, 1.2, 5.0);
  // 회전 곡률(라디안/미터). 0이면 직진 화살표.
  const curvature = arClamp(Number(options.curvature) || 0, -0.12, 0.12);
  const z = a.liftM;

  const m = meshBuilder();
  const cursor = m.indexCount();
  const halfW = width / 2;
  const depth = a.chevronDepthM;
  const thick = a.chevronThickM;

  for (let i = 0; i < count; i += 1) {
    const x = i * spacing;
    // 곡률에 따라 횡방향으로 밀고 화살촉도 같이 회전
    const y = 0.5 * curvature * x * x;
    const yaw = curvature * x;
    const cos = Math.cos(yaw), sin = Math.sin(yaw);
    const rot = (lx, ly) => [x + lx * cos - ly * sin, y + lx * sin + ly * cos];

    // 셰브런 = 두꺼운 "V". 바깥 윤곽과 안쪽 윤곽을 잇는 6각형으로 만든다.
    const outer = [rot(depth, 0), rot(0, halfW), rot(0, -halfW)];
    const inner = [rot(depth - thick, 0), rot(-thick, halfW - thick * 0.6), rot(-thick, -halfW + thick * 0.6)];
    // 좌 날개
    m.flatFan([outer[0], outer[1], inner[1], inner[0]], z);
    // 우 날개
    m.flatFan([outer[0], inner[0], inner[2], outer[2]], z);
  }
  m.mark("chevrons", cursor);

  return m.build({
    kind: AR_MARKER_KIND.COMMIT_ARROW,
    lod: "flat",
    boundsM: Object.freeze({ width, height: 0, depth: (count - 1) * spacing + depth }),
    anchorHeightM: 0,
    chevronCount: count,
    curvature,
  });
}

/* ── 목적지 핀 ───────────────────────────────────────────────
 * 기둥 + 구(球) 머리. 카메라를 향한 yaw 회전은 렌더 단계에서 적용한다. */
export function buildDestinationPin(options = {}) {
  const p = AR_GEOMETRY.pin;
  const height = arClamp(Number(options.heightM) || p.heightM, 2.0, 8.0);
  const headR = arClamp(Number(options.headRadiusM) || p.headRadiusM, 0.3, 1.6);
  const segments = arClamp(Math.round(options.radialSegments ?? p.radialSegments), 6, 24);
  const flat = options.lod === "flat";

  const m = meshBuilder();
  let cursor = m.indexCount();
  m.box(0, 0, (height - headR) / 2, p.stemRadiusM * 2, p.stemRadiusM * 2, height - headR);
  m.mark("stem", cursor);

  cursor = m.indexCount();
  const cz = height - headR;
  if (flat) {
    // 저사양: yz 평면의 원판 하나로 대체(billboard 로 카메라를 향하게 한다).
    const center = m.vertex(0, 0, cz);
    const ring = [];
    for (let i = 0; i < segments; i += 1) {
      const t = (i / segments) * Math.PI * 2;
      ring.push(m.vertex(0, Math.cos(t) * headR, cz + Math.sin(t) * headR));
    }
    for (let i = 0; i < segments; i += 1) {
      // 삼각형 팬. quad() 로 만들면 두 번째 삼각형이 축퇴한다.
      m.triangle(center, ring[i], ring[(i + 1) % segments]);
    }
  } else {
    // UV 구
    const rings = Math.max(4, Math.round(segments / 2));
    const grid = [];
    for (let r = 0; r <= rings; r += 1) {
      const phi = (r / rings) * Math.PI;
      const row = [];
      for (let s = 0; s <= segments; s += 1) {
        const theta = (s / segments) * Math.PI * 2;
        row.push(m.vertex(
          Math.sin(phi) * Math.cos(theta) * headR,
          Math.sin(phi) * Math.sin(theta) * headR,
          cz + Math.cos(phi) * headR,
        ));
      }
      grid.push(row);
    }
    for (let r = 0; r < rings; r += 1) {
      for (let s = 0; s < segments; s += 1) {
        m.quad(grid[r][s], grid[r][s + 1], grid[r + 1][s + 1], grid[r + 1][s]);
      }
    }
  }
  m.mark("head", cursor);

  return m.build({
    kind: AR_MARKER_KIND.DESTINATION_PIN,
    lod: flat ? "flat" : "solid",
    boundsM: Object.freeze({ width: headR * 2, height, depth: headR * 2 }),
    anchorHeightM: height,
    billboard: "yaw",   // 렌더 단계에서 카메라 방향으로 yaw 회전
  });
}

/* ── 주의/단속 표지 ───────────────────────────────────────────
 * 카메라를 향하는 원형 링. 안쪽은 비워 두고 아이콘/텍스트는 2D 레이어가 얹는다. */
export function buildCautionSign(options = {}) {
  const s = AR_GEOMETRY.sign;
  const radius = arClamp((Number(options.diameterM) || s.diameterM) / 2, 0.4, 1.6);
  const height = arClamp(Number(options.heightM) || s.heightM, 1.5, 6.0);
  const thick = arClamp(Number(options.ringThickM) || s.ringThickM, 0.08, 0.5);
  const segments = arClamp(Math.round(options.radialSegments ?? s.radialSegments), 8, 32);

  const m = meshBuilder();
  const cursor = m.indexCount();
  const inner = radius - thick;
  // yz 평면의 링(정면이 -x 를 향함 → 카메라 쪽). billboard 로 yaw 만 맞춘다.
  const outerIdx = [], innerIdx = [];
  for (let i = 0; i < segments; i += 1) {
    const t = (i / segments) * Math.PI * 2;
    const c = Math.cos(t), sn = Math.sin(t);
    outerIdx.push(m.vertex(0, c * radius, height + sn * radius));
    innerIdx.push(m.vertex(0, c * inner, height + sn * inner));
  }
  for (let i = 0; i < segments; i += 1) {
    const j = (i + 1) % segments;
    m.quad(outerIdx[i], outerIdx[j], innerIdx[j], innerIdx[i]);
  }
  m.mark("ring", cursor);

  return m.build({
    kind: AR_MARKER_KIND.CAUTION_SIGN,
    lod: "flat",
    boundsM: Object.freeze({ width: radius * 2, height: height + radius, depth: 0.02 }),
    anchorHeightM: height,
    billboard: "yaw",
    innerRadiusM: inner,
  });
}

/* ── 차선 안내 밴드 ───────────────────────────────────────────
 * 권장 차선 위에 눕는 띠. laneOffsetM 으로 좌우 차선군을 지정한다.
 * 주의: 지도 기준 N번 차선이 아니라 "권장 차선군" 표현이다. */
export function buildLaneBand(options = {}) {
  const b = AR_GEOMETRY.laneBand;
  const laneWidth = resolveLaneWidthM(options.laneWidthM);
  const length = arClamp(Number(options.lengthM) || b.lengthM, 6, 60);
  const offset = arClamp(Number(options.laneOffsetM) || 0, -12, 12);
  const width = laneWidth * 0.72;
  const segments = arClamp(Math.round(options.segments ?? 8), 2, 24);
  const curvature = arClamp(Number(options.curvature) || 0, -0.12, 0.12);

  const m = meshBuilder();
  const cursor = m.indexCount();
  const half = width / 2;
  let prevL = null, prevR = null;
  for (let i = 0; i <= segments; i += 1) {
    const x = (i / segments) * length;
    const y = offset + 0.5 * curvature * x * x;
    const l = m.vertex(x, y + half, b.liftM);
    const r = m.vertex(x, y - half, b.liftM);
    if (prevL !== null) m.quad(prevL, l, r, prevR);
    prevL = l; prevR = r;
  }
  m.mark("band", cursor);

  return m.build({
    kind: AR_MARKER_KIND.LANE_BAND,
    lod: "flat",
    boundsM: Object.freeze({ width, height: 0, depth: length }),
    anchorHeightM: 0,
    fadeStartRatio: b.fadeStartRatio,
    laneOffsetM: offset,
  });
}

/* ── 구간단속 게이트 ──────────────────────────────────────────
 * 회전 게이트보다 낮고 넓은 이중 보. 진행 중이면 상단에 진척 바를 얹는다. */
export function buildSectionGate(options = {}) {
  const laneWidth = resolveLaneWidthM(options.laneWidthM);
  const width = laneWidth * 2.1;
  const half = width / 2;
  const height = arClamp(Number(options.heightM) || 3.4, 2.0, 5.0);
  const post = 0.22;
  const beam = 0.28;
  const depth = options.lod === "flat" ? 0.02 : 0.18;
  const progress = arClamp(Number(options.progress) || 0, 0, 1);

  const m = meshBuilder();
  let cursor = m.indexCount();
  m.box(0, half - post / 2, height / 2, depth, post, height);
  m.box(0, -half + post / 2, height / 2, depth, post, height);
  m.mark("posts", cursor);

  cursor = m.indexCount();
  m.box(0, 0, height - beam / 2, depth, width, beam);
  m.box(0, 0, height - beam * 2.2, depth, width, beam * 0.55);
  m.mark("beams", cursor);

  if (progress > 0) {
    cursor = m.indexCount();
    const barW = width * progress;
    m.box(0, half - barW / 2, height - beam * 3.4, depth * 1.2, barW, beam * 0.45);
    m.mark("progress", cursor);
  }

  return m.build({
    kind: AR_MARKER_KIND.SECTION_GATE,
    lod: options.lod === "flat" ? "flat" : "solid",
    boundsM: Object.freeze({ width, height, depth }),
    anchorHeightM: height,
    progress,
  });
}

/* ── 신호등 헤드 ──────────────────────────────────────────────
 * 램프 개수만큼 원판을 세로로 쌓는다. 켜짐/꺼짐은 파트별로 색을 달리 준다. */
export function buildSignalHead(options = {}) {
  const lamps = Array.isArray(options.lamps) ? options.lamps.slice(0, 5) : [];
  const count = Math.max(1, lamps.length);
  const lampR = 0.28;
  const gap = lampR * 2.3;
  const height = arClamp(Number(options.heightM) || 4.4, 2.5, 7.0);
  const segments = 12;

  const m = meshBuilder();
  let cursor = m.indexCount();
  m.box(0, 0, height / 2, 0.10, 0.10, height);
  m.mark("pole", cursor);

  // 하우징
  const housingH = count * gap + lampR;
  cursor = m.indexCount();
  m.box(0, 0, height + housingH / 2, 0.10, lampR * 2.6, housingH);
  m.mark("housing", cursor);

  // 램프(yz 평면 원판). 파트 이름에 램프 id 를 실어 렌더러가 색을 고른다.
  for (let i = 0; i < count; i += 1) {
    cursor = m.indexCount();
    const cz = height + housingH - lampR - i * gap;
    const center = m.vertex(0.06, 0, cz);
    const ring = [];
    for (let s = 0; s < segments; s += 1) {
      const t = (s / segments) * Math.PI * 2;
      ring.push(m.vertex(0.06, Math.cos(t) * lampR, cz + Math.sin(t) * lampR));
    }
    for (let s = 0; s < segments; s += 1) {
      m.triangle(center, ring[s], ring[(s + 1) % segments]);
    }
    const lamp = lamps[i] || {};
    m.mark(`lamp:${lamp.id || i}:${lamp.on ? "on" : "off"}`, cursor);
  }

  return m.build({
    kind: AR_MARKER_KIND.SIGNAL_HEAD,
    lod: "flat",
    boundsM: Object.freeze({ width: lampR * 2.6, height: height + housingH, depth: 0.12 }),
    anchorHeightM: height + housingH / 2,
    billboard: "yaw",
    lampCount: count,
  });
}

/* ── 교차로 프리뷰 카드 ───────────────────────────────────────
 * 카메라를 향하는 얇은 판. 실제 이미지는 2D 레이어가 텍스처로 얹는다. */
export function buildCrossroadCard(options = {}) {
  const w = arClamp(Number(options.widthM) || 3.2, 1.2, 6.0);
  const h = arClamp(Number(options.heightM) || 2.2, 0.8, 4.0);
  const lift = arClamp(Number(options.liftM) || 3.6, 1.0, 8.0);
  const frame = 0.12;

  const m = meshBuilder();
  let cursor = m.indexCount();
  // yz 평면 판(정면 -x)
  const a = m.vertex(0, w / 2, lift - h / 2);
  const b = m.vertex(0, -w / 2, lift - h / 2);
  const c = m.vertex(0, -w / 2, lift + h / 2);
  const d = m.vertex(0, w / 2, lift + h / 2);
  m.quad(a, b, c, d);
  m.mark("panel", cursor);

  cursor = m.indexCount();
  m.box(0.01, 0, lift + h / 2 + frame / 2, 0.02, w + frame, frame);
  m.box(0.01, 0, lift - h / 2 - frame / 2, 0.02, w + frame, frame);
  m.mark("frame", cursor);

  return m.build({
    kind: AR_MARKER_KIND.CROSSROAD_CARD,
    lod: "flat",
    boundsM: Object.freeze({ width: w, height: lift + h / 2, depth: 0.02 }),
    anchorHeightM: lift,
    billboard: "yaw",
    uvPanel: Object.freeze([0, 1, 2, 3]),
  });
}

/* ── 제한속도 표지 ────────────────────────────────────────────
 * 한국/비엔나식 원형 표지. 숫자는 2D 레이어가 얹는다. */
export function buildSpeedSign(options = {}) {
  const radius = arClamp((Number(options.diameterM) || 1.4) / 2, 0.3, 1.4);
  const lift = arClamp(Number(options.heightM) || 3.0, 1.2, 6.0);
  const ring = arClamp(Number(options.ringThickM) || 0.20, 0.06, 0.5);
  const segments = 20;

  const m = meshBuilder();
  let cursor = m.indexCount();
  m.box(0, 0, lift / 2, 0.08, 0.08, lift);
  m.mark("pole", cursor);

  // 흰 판
  cursor = m.indexCount();
  const center = m.vertex(0.04, 0, lift + radius);
  const inner = [];
  for (let s = 0; s < segments; s += 1) {
    const t = (s / segments) * Math.PI * 2;
    inner.push(m.vertex(0.04, Math.cos(t) * (radius - ring), lift + radius + Math.sin(t) * (radius - ring)));
  }
  for (let s = 0; s < segments; s += 1) m.triangle(center, inner[s], inner[(s + 1) % segments]);
  m.mark("face", cursor);

  // 빨간 링
  cursor = m.indexCount();
  const outerIdx = [], innerIdx = [];
  for (let s = 0; s < segments; s += 1) {
    const t = (s / segments) * Math.PI * 2;
    const co = Math.cos(t), si = Math.sin(t);
    outerIdx.push(m.vertex(0.05, co * radius, lift + radius + si * radius));
    innerIdx.push(m.vertex(0.05, co * (radius - ring), lift + radius + si * (radius - ring)));
  }
  for (let s = 0; s < segments; s += 1) {
    const j = (s + 1) % segments;
    m.quad(outerIdx[s], outerIdx[j], innerIdx[j], innerIdx[s]);
  }
  m.mark("ring", cursor);

  return m.build({
    kind: AR_MARKER_KIND.SPEED_SIGN,
    lod: "flat",
    boundsM: Object.freeze({ width: radius * 2, height: lift + radius * 2, depth: 0.05 }),
    anchorHeightM: lift + radius,
    billboard: "yaw",
  });
}

export const AR_MARKER_BUILDERS = Object.freeze({
  [AR_MARKER_KIND.TURN_GATE]: buildTurnGate,
  [AR_MARKER_KIND.COMMIT_ARROW]: buildCommitArrow,
  [AR_MARKER_KIND.DESTINATION_PIN]: buildDestinationPin,
  [AR_MARKER_KIND.CAUTION_SIGN]: buildCautionSign,
  [AR_MARKER_KIND.LANE_BAND]: buildLaneBand,
  [AR_MARKER_KIND.SECTION_GATE]: buildSectionGate,
  [AR_MARKER_KIND.SIGNAL_HEAD]: buildSignalHead,
  [AR_MARKER_KIND.CROSSROAD_CARD]: buildCrossroadCard,
  [AR_MARKER_KIND.SPEED_SIGN]: buildSpeedSign,
});

/** kind 로 빌더를 골라 실행. 알 수 없으면 null. */
export function buildMarker(kind, options = {}) {
  const builder = AR_MARKER_BUILDERS[kind];
  return typeof builder === "function" ? builder(options) : null;
}
