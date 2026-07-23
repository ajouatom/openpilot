/* CarrotNaviState(TMap) 데이터 종류 → AR 마커 descriptor 매핑.
 *
 * 여기서 "코드 → 어떤 마커를 어떤 모양으로" 만 정한다. 화면 위치는 localizer/
 * world_anchor 가, 크기·표시 여부는 responsive 가, 그리기는 renderer 가 맡는다.
 *
 * 코드값 출처:
 *   turnType  : selfdrive/carrot/carrot_serv.py 의 nav_type_mapping
 *   sdiType   : web/js/realtime/mini_hud_model.js 의 CAMERA/SECTION/WAZE 분류
 * 의미가 확정되지 않은 코드는 UNKNOWN 으로 떨어뜨리고 과장하지 않는다.
 */

import { AR_MARKER_KIND, AR_PRODUCT_MARKERS } from "./tokens.js";
// tone 은 문자열 리터럴로 쓰지 않는다. "turn" 같은 존재하지 않는 톤이
// 조용히 폴백되어 색이 틀렸던 적이 있다. 상수를 쓰면 오타가 즉시 드러난다.
import { AR_TONE } from "./design_tokens.js";

/* ── turnType 분류 ────────────────────────────────────────── */

export const TURN_FAMILY = Object.freeze({
  TURN: "turn",
  FORK: "fork",
  RAMP: "ramp",
  ROTARY: "rotary",
  UTURN: "uturn",
  ARRIVE: "arrive",
  NOTIFICATION: "notification",
  UNKNOWN: "unknown",
});

export const TURN_DIRECTION = Object.freeze({
  LEFT: "left",
  RIGHT: "right",
  SLIGHT_LEFT: "slight_left",
  SLIGHT_RIGHT: "slight_right",
  SHARP_LEFT: "sharp_left",
  SHARP_RIGHT: "sharp_right",
  STRAIGHT: "straight",
  UTURN: "uturn",
});

const D = TURN_DIRECTION;
const F = TURN_FAMILY;

/* carrot_serv.py nav_type_mapping 을 그대로 옮긴 표. 값을 바꾸지 말 것. */
const TURN_TYPE_TABLE = new Map([
  [12, [F.TURN, D.LEFT]], [13, [F.TURN, D.RIGHT]],
  [16, [F.TURN, D.SHARP_LEFT]], [19, [F.TURN, D.SHARP_RIGHT]],
  [1000, [F.TURN, D.SLIGHT_LEFT]], [1001, [F.TURN, D.SLIGHT_RIGHT]],
  [14, [F.UTURN, D.UTURN]],
  [1002, [F.FORK, D.SLIGHT_LEFT]], [1003, [F.FORK, D.SLIGHT_RIGHT]],
  [7, [F.FORK, D.LEFT]], [17, [F.FORK, D.LEFT]], [44, [F.FORK, D.LEFT]],
  [75, [F.FORK, D.LEFT]], [76, [F.FORK, D.LEFT]], [118, [F.FORK, D.LEFT]],
  [6, [F.FORK, D.RIGHT]], [43, [F.FORK, D.RIGHT]], [73, [F.FORK, D.RIGHT]],
  [74, [F.FORK, D.RIGHT]], [117, [F.FORK, D.RIGHT]], [123, [F.FORK, D.RIGHT]],
  [124, [F.FORK, D.RIGHT]],
  [1006, [F.RAMP, D.LEFT]], [1007, [F.RAMP, D.RIGHT]],
  [102, [F.RAMP, D.SLIGHT_LEFT]], [105, [F.RAMP, D.SLIGHT_LEFT]],
  [112, [F.RAMP, D.SLIGHT_LEFT]], [115, [F.RAMP, D.SLIGHT_LEFT]],
  [101, [F.RAMP, D.SLIGHT_RIGHT]], [104, [F.RAMP, D.SLIGHT_RIGHT]],
  [111, [F.RAMP, D.SLIGHT_RIGHT]], [114, [F.RAMP, D.SLIGHT_RIGHT]],
  [131, [F.ROTARY, D.SLIGHT_RIGHT]], [132, [F.ROTARY, D.SLIGHT_RIGHT]],
  [133, [F.ROTARY, D.RIGHT]], [134, [F.ROTARY, D.SHARP_RIGHT]],
  [135, [F.ROTARY, D.SHARP_RIGHT]], [136, [F.ROTARY, D.SHARP_LEFT]],
  [137, [F.ROTARY, D.SHARP_LEFT]], [138, [F.ROTARY, D.SHARP_LEFT]],
  [139, [F.ROTARY, D.LEFT]], [140, [F.ROTARY, D.SLIGHT_LEFT]],
  [141, [F.ROTARY, D.SLIGHT_LEFT]], [142, [F.ROTARY, D.STRAIGHT]],
  [201, [F.ARRIVE, D.STRAIGHT]],
  [51, [F.NOTIFICATION, D.STRAIGHT]],
]);

/** 방향 → 부호(-1 좌, +1 우, 0 직진/불명)와 진입 곡률 계수. */
const DIRECTION_GEOMETRY = Object.freeze({
  [D.LEFT]: { sign: -1, curvature: -0.045, yawDeg: -80 },
  [D.RIGHT]: { sign: 1, curvature: 0.045, yawDeg: 80 },
  [D.SHARP_LEFT]: { sign: -1, curvature: -0.075, yawDeg: -120 },
  [D.SHARP_RIGHT]: { sign: 1, curvature: 0.075, yawDeg: 120 },
  [D.SLIGHT_LEFT]: { sign: -1, curvature: -0.020, yawDeg: -35 },
  [D.SLIGHT_RIGHT]: { sign: 1, curvature: 0.020, yawDeg: 35 },
  [D.STRAIGHT]: { sign: 0, curvature: 0, yawDeg: 0 },
  [D.UTURN]: { sign: -1, curvature: -0.110, yawDeg: -175 },
});

export function classifyTurnType(turnType) {
  const code = Number(turnType);
  const hit = Number.isFinite(code) ? TURN_TYPE_TABLE.get(code) : null;
  const [family, direction] = hit || [F.UNKNOWN, D.STRAIGHT];
  const geom = DIRECTION_GEOMETRY[direction] || DIRECTION_GEOMETRY[D.STRAIGHT];
  return Object.freeze({ code, family, direction, ...geom });
}

/* ── SDI(단속/주의) 분류 ──────────────────────────────────── */

export const SDI_FAMILY = Object.freeze({
  CAMERA: "camera",
  SECTION: "section",
  BUMP: "bump",
  POLICE: "police",
  OTHER: "other",
});

const SDI_CAMERA = new Set([0, 1, 7, 8, 75, 76]);
const SDI_SECTION = new Set([2, 3, 4]);
const SDI_POLICE = new Set([100, 101]);

export function classifySdiType(sdiType) {
  const code = Number(sdiType);
  if (!Number.isFinite(code) || code < 0) return Object.freeze({ code, family: null });
  if (code === 22) return Object.freeze({ code, family: SDI_FAMILY.BUMP });
  if (SDI_POLICE.has(code)) return Object.freeze({ code, family: SDI_FAMILY.POLICE });
  if (SDI_SECTION.has(code)) return Object.freeze({ code, family: SDI_FAMILY.SECTION });
  if (SDI_CAMERA.has(code)) return Object.freeze({ code, family: SDI_FAMILY.CAMERA });
  return Object.freeze({ code, family: SDI_FAMILY.OTHER });
}

/* ── 신호등 ──────────────────────────────────────────────── */

export function classifyTrafficSignal(signal) {
  if (!signal || signal.visible !== true) return null;
  const distanceM = Number(signal.distanceM);
  if (!Number.isFinite(distanceM) || distanceM <= 0) return null;
  const lamps = [];
  const push = (id, valid, on, remain) => {
    if (valid !== true) return;
    lamps.push(Object.freeze({ id, on: on === true, remainSec: Number(remain) || 0 }));
  };
  push("red", signal.redValid, signal.redOn, signal.redRemainSec);
  push("left", signal.leftValid, signal.leftOn, signal.leftRemainSec);
  push("green", signal.greenValid, signal.greenOn, signal.greenRemainSec);
  push("right", signal.rightValid, signal.rightOn, signal.rightRemainSec);
  push("uturn", signal.uturnValid, signal.uturnOn, signal.uturnRemainSec);
  return lamps.length ? Object.freeze({
    lamps: Object.freeze(lamps),
    distanceM,
    counterRemainSec: signal.uiCounterValid === true ? Number(signal.uiCounterRemainSec) || 0 : 0,
  }) : null;
}

/* ── 차선 안내 ───────────────────────────────────────────── */

/** available[] 에서 권장 차선군의 중심 오프셋(m)을 추정한다.
 *  주의: 지도 기준 N번 차선이 아니다. "권장 차선군"의 상대 위치일 뿐이다. */
export function laneGroupGeometry(lane, laneWidthM = 3.5) {
  const available = Array.isArray(lane?.available) ? lane.available : [];
  const count = Number(lane?.count) || available.length;
  if (!count || !available.length) return null;
  const picked = [];
  for (let i = 0; i < available.length; i += 1) if (available[i]) picked.push(i);
  if (!picked.length) return null;
  const mean = picked.reduce((a, b) => a + b, 0) / picked.length;
  // 차선 index 는 좌→우. 우리 좌표계는 +y 가 좌측이므로 부호를 뒤집는다.
  const centerIndex = (count - 1) / 2;
  const offsetM = -(mean - centerIndex) * laneWidthM;
  const span = Math.max(...picked) - Math.min(...picked) + 1;
  return Object.freeze({
    offsetM: offsetM === 0 ? 0 : offsetM,
    widthM: Math.max(1, span) * laneWidthM,
    selectedCount: picked.length,
  });
}

export function laneGroupOffsetM(lane, laneWidthM = 3.5) {
  return laneGroupGeometry(lane, laneWidthM)?.offsetM ?? null;
}

/* ── 데이터 종류 → 마커 descriptor ────────────────────────── */

export const AR_SOURCE = Object.freeze({
  GUIDANCE_CURRENT: "guidanceCurrent",
  GUIDANCE_NEXT: "guidanceNext",
  LANE: "lane",
  SDI: "sdi",
  SDI_SECONDARY: "sdiSecondary",
  SECTION: "section",
  TRAFFIC_SIGNAL: "trafficSignal",
  CROSSROAD: "crossroad",
  DESTINATION: "destination",
});

// Only CarrotNavi carries an event distance plus route/point context suitable
// for a persistent world marker. CarrotMan/stock values remain valid HUD
// fallbacks, but promoting them to AR would invent a coordinate and make the
// marker follow the camera.
export const AR_NAV_SOURCE_QUALITY = Object.freeze({
  CARROT_NAVI: "world-anchor",
  CARROT_MAN: "hud-only",
  STOCK_NAVI: "hud-only",
});

/**
 * CarrotNaviState 스냅샷 → 마커 descriptor 목록.
 * descriptor 는 지오메트리를 아직 만들지 않는다(모양·색·거리만 정함).
 */
export function describeMarkers(navi, options = {}) {
  const laneWidthM = Number(options.laneWidthM) || 3.5;
  const out = [];
  const add = (d) => { if (d) out.push(Object.freeze(d)); };

  const guidance = (item, source) => {
    /* 인코더(compact_state)는 유효성을 `present` 로 직접 보낸다. 예전엔 여기서
     * `meta.present` 를 봤는데 그런 필드는 오지 않으므로 검사가 항상 통과했고,
     * 안내가 없을 때도 0m 짜리 유령 마커가 생겼다. */
    if (!item || item.present === false) return null;
    const distanceM = Number(item.distanceM);
    // 0m 이하 안내는 이미 지났거나 값이 비어 있는 것이다.
    if (!Number.isFinite(distanceM) || distanceM <= 0) return null;
    const turn = classifyTurnType(item.turnType);
    /* 위경도를 그대로 실어 보낸다. runtime 이 이 좌표로 마커를 월드에 박고,
     * 좌표가 없거나 미덥지 않으면 modelV2 경로 위 배치로 떨어진다. */
    const point = item.pointValid === true
      ? { pointValid: true, latitude: item.latitude, longitude: item.longitude }
      : null;
    if (turn.family === F.ARRIVE) {
      return { source, kind: AR_MARKER_KIND.DESTINATION_PIN, distanceM, point,
               tone: AR_TONE.DESTINATION, label: item.mainText || item.roadName || "", turn };
    }
    // Straight-ahead guidance is persistent Navi state rather than a useful
    // world event. Keep it out of AR while preserving lane/SDI markers.
    if (turn.direction === D.STRAIGHT) return null;
    if (turn.family === F.NOTIFICATION || turn.family === F.UNKNOWN) return null;
    return {
      source, kind: AR_MARKER_KIND.TURN_GATE, distanceM, point, tone: AR_TONE.GUIDE,
      label: item.roadName || item.mainText || "", turn,
      variant: turn.family,          // turn / fork / ramp / rotary / uturn
      laneWidthM,
    };
  };

  add(guidance(navi?.guidanceCurrent, AR_SOURCE.GUIDANCE_CURRENT));
  // 다음 안내는 preview 하나만 예약한다(동시 다중 gate 금지).
  add(guidance(navi?.guidanceNext, AR_SOURCE.GUIDANCE_NEXT));

  // 진입 화살표: 현재 안내가 회전 계열일 때만
  const cur = out.find((d) => d.source === AR_SOURCE.GUIDANCE_CURRENT);
  if (cur && cur.kind === AR_MARKER_KIND.TURN_GATE && cur.turn.sign !== 0) {
    add({ source: AR_SOURCE.GUIDANCE_CURRENT, kind: AR_MARKER_KIND.COMMIT_ARROW,
          distanceM: Math.max(0, cur.distanceM - 6), tone: AR_TONE.GUIDE,
          eventDistanceM: cur.distanceM,
          turn: cur.turn, curvature: cur.turn.curvature, label: "" });
  }

  // 권장 차선 데이터와 BAND 구현은 보존하지만, 실차 화면에서 의미가 직관적이지
  // 않다는 사용자 판정에 따라 제품 노출은 정책 토큰으로 비활성화한다.
  if (AR_PRODUCT_MARKERS.laneBand) {
    // current가 비었을 때 laneAhead 중 가장 가까운 명시 안내 하나만 사용한다.
    const explicitLane = (item) => item && item.present !== false && item.visible === true
      && Number.isFinite(Number(item.distanceM)) && Number(item.distanceM) > 0;
    const lane = explicitLane(navi?.laneCurrent)
      ? navi.laneCurrent
      : (Array.isArray(navi?.laneAhead) ? navi.laneAhead : [])
        .filter(explicitLane)
        .sort((a, b) => Number(a.distanceM) - Number(b.distanceM))[0];
    if (lane) {
      const geometry = laneGroupGeometry(lane, laneWidthM);
      if (geometry) {
        add({ source: AR_SOURCE.LANE, kind: AR_MARKER_KIND.LANE_BAND,
              distanceM: Number(lane.distanceM), tone: AR_TONE.LANE,
              laneOffsetM: geometry.offsetM, laneWidthM: geometry.widthM,
              selectedLaneCount: geometry.selectedCount, label: "" });
      }
    }
  }

  // SDI (단속/과속방지턱/경찰). primary/secondary는 별도 identity를 쓴다.
  const speed = navi?.speed;
  const addSdi = (prefix, source) => {
    if (speed?.[`${prefix}Present`] !== true) return;
    const distanceM = Number(speed[`${prefix}DistanceM`]);
    const sdi = classifySdiType(speed[`${prefix}Type`]);
    // The section gate below owns section semantics and avoids a duplicate
    // caution sign for one event.
    if (!sdi.family || sdi.family === SDI_FAMILY.SECTION || !Number.isFinite(distanceM) || distanceM <= 0) return;
    const speedLimitKph = Number(speed[`${prefix}SpeedLimitKph`]) || 0;
    add({ source, kind: AR_MARKER_KIND.CAUTION_SIGN, distanceM, tone: AR_TONE.CAUTION,
          sdiFamily: sdi.family, speedLimitKph,
          blockType: Number(speed[`${prefix}BlockType`]) || 0,
          blockDistanceM: Number(speed[`${prefix}BlockDistanceM`]) || 0,
          label: String(speedLimitKph || "") });
  };
  addSdi("sdi", AR_SOURCE.SDI);
  addSdi("secondarySdi", AR_SOURCE.SDI_SECONDARY);
  // 구간단속은 별도 게이트(진입/진행 표시)
  const sectionDistanceM = Number(speed?.sectionRemainingDistanceM);
  if (speed?.sectionPresent === true && speed.sectionOffRoute !== true
      && Number.isFinite(sectionDistanceM) && sectionDistanceM > 0) {
    add({ source: AR_SOURCE.SECTION, kind: AR_MARKER_KIND.SECTION_GATE,
          distanceM: sectionDistanceM, tone: AR_TONE.RESTRICT,
          active: speed.sectionActive === true,
          averageKph: Number(speed.sectionAverageKph) || 0,
          overallAverageKph: Number(speed.sectionOverallAverageKph) || 0,
          remainingTimeSec: Number(speed.sectionRemainingTimeSec) || 0,
          suspended: speed.sectionSuspended === true,
          limitKph: Number(speed.sectionSpeedLimitKph) || 0,
          progress: Number(speed.sectionProgress) || 0, laneWidthM });
  }

  // 신호등
  const signal = classifyTrafficSignal(navi?.trafficSignal);
  if (signal) {
    add({ source: AR_SOURCE.TRAFFIC_SIGNAL, kind: AR_MARKER_KIND.SIGNAL_HEAD,
          distanceM: signal.distanceM, tone: AR_TONE.CAUTION, lamps: signal.lamps,
          counterRemainSec: signal.counterRemainSec, label: "" });
  }

  // 교차로 프리뷰
  const crossroad = navi?.crossroad;
  if (crossroad?.visible === true && Number(crossroad.distanceM) > 0) {
    add({ source: AR_SOURCE.CROSSROAD, kind: AR_MARKER_KIND.CROSSROAD_CARD,
          distanceM: Number(crossroad.distanceM) || 0, tone: AR_TONE.GUIDE,
          imageCode: Number(crossroad.imageCode) || 0, label: "" });
  }

  // roadLimitKph is persistent vehicle state with no world coordinate. It is
  // already visible in the normal HUD and must never be invented as a 30 m AR
  // event. Distance-bearing SDI/section events above remain valid AR markers.

  return Object.freeze(out);
}
