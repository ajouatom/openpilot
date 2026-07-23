/* 한 프레임에 무엇을 어디에 그릴지 정하는 순수 계산.
 *
 * 여기 있는 것은 전부 부수효과가 없는 계산이다. 그래서 워커에서 돌릴 수 있고,
 * 실제로 워커에서 돈다(OffscreenCanvas 를 못 쓰는 환경에서만 메인이 같은
 * 함수를 부른다). 워커와 메인이 **같은 코드**를 쓰므로 두 경로가 갈라지지 않는다.
 *
 * 입력은 provider 스냅샷에서 뽑은 순수 데이터뿐이다. DOM 도 캔버스도 모른다.
 */

import { describeMarkers } from "./tmap_catalog.js";
import { signboardFromMarker } from "./signboard.js";
import { AR_MARKER_KIND, AR_PHASE, phaseForDistance } from "./tokens.js";
import { discoveryRangeM, isWithinDiscoveryRange } from "./discovery.js";
import { pointOnPath } from "./projection.js";
import {
  AR_GEO_LIMITS,
  geoAnchor,
  routeDistanceAnchor,
  routeTangentHeading,
} from "./geo.js";
import { markerIdentity, markerLifecycleSlot } from "./marker_identity.js";
import { roadFrameFields, roadFrameFromHeading } from "./road_frame.js";
import { AR_COORDINATE_FRAME } from "./coordinate_frames.js";
import { matchRoutePosition } from "./route_matcher.js";

function finite(v, fallback = null) {
  const n = Number(v);
  return Number.isFinite(n) ? n : fallback;
}

/** navi 스냅샷 → 그릴 표지 목록(앵커는 아직 없음). */
export function collectSigns(input = {}, options = {}) {
  const navi = input.navi;
  const laneWidthM = finite(input.laneWidthM, 3.5);
  const egoSpeedMps = finite(input.egoSpeedMps, 0);
  const discoveryM = discoveryRangeM(input.discoveryRangeM);

  /* 실제 Navi 마커가 우선이다.
   * calibrationProbe 는 "Navi 가 없을 때 대신 세우는 검증용 표지"이지
   * "항상 검증용만 그린다"가 아니다. */
  const out = [];
  if (navi && input.naviUsable !== false) {
    for (const marker of describeMarkers(navi, { laneWidthM })) {
      const distanceM = finite(marker.distanceM, 0);
      const eventDistanceM = finite(marker.eventDistanceM, distanceM);
      if (!isWithinDiscoveryRange(eventDistanceM, discoveryM)) continue;
      const phase = phaseForDistance(distanceM, egoSpeedMps, { discoveryRangeM: discoveryM });
      // The far cue is the sign itself. A second commit arrow at the same
      // horizon adds clutter and can suppress guidanceNext in overlap choice.
      if (marker.kind === AR_MARKER_KIND.COMMIT_ARROW && phase === AR_PHASE.PREVIEW) continue;
      const descriptor = signboardFromMarker({
        ...marker,
        phase,
      });
      if (descriptor) {
        const identityInput = { ...marker, descriptor };
        out.push({
          identityInput,
          descriptor,
          distanceM,
          point: marker.point || null,
          source: marker.source || "unknown",
          confidence: phase === AR_PHASE.PREVIEW ? "low" : "high",
          discovery: Object.freeze({
            rangeM: discoveryM,
            phase,
            early: phase === AR_PHASE.PREVIEW,
          }),
        });
      }
    }
  }
  if (out.length) return attachMarkerIdentities(out, input.navi, options.identityTracker);

  /* 활성 안내가 off-route/미연결 상태이면 즉시 숨긴다. 이때 probe로 떨어지면
   * 실제 안내처럼 오해할 수 있다. 반대로 경로 안내 자체가 비활성인 상태는
   * 사용자가 명시적으로 AR을 켠 현장 점검에서만 probe를 허용한다. */
  const navigationStatus = navi?.navigationStatus;
  if (
    navi
    && input.naviUsable === false
    && (navigationStatus?.guidanceActive === true || navigationStatus?.offRoute === true)
  ) return [];

  if (!input.calibrationProbe) return [];
  const distanceM = finite(input.probeDistanceM, 40);
  const descriptor = signboardFromMarker({
    kind: "turn_gate", distanceM,
    // The field probe only confirms that the AR renderer is alive. Giving it
    // a real maneuver direction made replay users mistake the synthetic cue
    // for persistent TMap straight guidance.
    turn: { direction: "", sign: 0 },
    label: "AR 표시 확인", phase: phaseForDistance(distanceM, egoSpeedMps),
  });
  if (!descriptor) return [];
  const identityInput = { descriptor, source: "calibrationProbe" };
  return attachMarkerIdentities([{
    identityInput,
    descriptor,
    distanceM,
    source: "calibrationProbe",
  }], input.navi, options.identityTracker);
}

function attachMarkerIdentities(candidates, navi, identityTracker) {
  const identityInputs = candidates.map((item) => item.identityInput);
  const trackedIds = identityTracker?.assign?.(identityInputs, {
    sessionId: navi?.sessionId || (candidates.every((item) => item.source === "calibrationProbe")
      ? "calibration-probe"
      : "legacy"),
  });
  return candidates.map((item, index) => {
    const markerId = trackedIds?.[index] || markerIdentity(item.identityInput);
    return Object.freeze({
      descriptor: item.descriptor,
      distanceM: item.distanceM,
      point: item.point || null,
      source: item.source,
      markerId,
      // Compatibility alias for existing diagnostics and fixtures.
      eventKey: markerId,
      lifecycleSlot: markerLifecycleSlot(item.identityInput),
      confidence: item.confidence || "high",
      discovery: item.discovery || null,
    });
  });
}

/**
 * 표지에 앵커(자차 기준 좌표)를 붙인다.
 *
 * 배치는 두 갈래다.
 *   1순위 위경도(geo). 경로 길이와 무관하므로 190m 를 넘는 안내도 세울 수 있다.
 *         320m 앞 단속 카메라가 사라지던 원인이 여기 있었다.
 *   2순위 modelV2 경로. 카메라가 실제로 보는 도로라 근거리에서 더 정확하다.
 *
 * geo 는 좌표가 경로거리와 모순되면 스스로 null 을 준다(geo.js 의 검사).
 * 그때는 조용히 경로 배치로 떨어진다.
 */
export function anchorSigns(signs, {
  vehicle,
  routePolyline,
  routeMatch,
  modelPosition,
  geoAllowed = false,
  routeAllowed = geoAllowed,
  failures = null,
} = {}) {
  if (modelPosition?.coordinateFrame && modelPosition.coordinateFrame !== AR_COORDINATE_FRAME.ROUTE_FLU) {
    throw new TypeError("anchorSigns modelPosition must use route-local FLU");
  }
  const out = [];
  for (const item of Array.isArray(signs) ? signs : []) {
    const kind = item.descriptor?.kind;
    const maneuver = kind === "turn_gate" || kind === "commit_arrow" || kind === "destination_pin";
    const routeSemantic = maneuver
      || item.source === "guidanceCurrent"
      || item.source === "guidanceNext"
      || item.source === "crossroad";
    // TMap vehicle and guidance points share one map-matched source. A fresh
    // route is enough to use their relative geometry even when Comma GPS did
    // not approve an absolute geo fix. geoAnchor still rejects points behind
    // the car or incompatible with the advertised route distance.
    let viaGeo = (geoAllowed || routeAllowed) && item.point
      ? geoAnchor(vehicle, item.point, { routeDistanceM: item.distanceM })
      : null;
    if (viaGeo) {
      // At an intersection, "nearest segment to the event point" can be the
      // incoming or a parallel branch. For maneuvers, walk the already matched
      // ordered route to the guidance distance and use its outgoing tangent.
      const matchedBranch = maneuver && routeMatch
        ? routeDistanceAnchor(
          vehicle,
          routePolyline,
          item.distanceM + AR_GEO_LIMITS.maneuverLookAheadM,
          { routeMatch },
        )
        : null;
      const fallbackTangent = maneuver && Array.isArray(routePolyline)
        ? null
        : routeTangentHeading(vehicle, item.point, routePolyline);
      const headingRad = matchedBranch?.headingRad ?? fallbackTangent;
      const roadFrame = roadFrameFromHeading(headingRad ?? 0);
      viaGeo = Object.freeze({
        ...viaGeo,
        routeStartIndex: matchedBranch?.routeStartIndex ?? routeMatch?.index ?? null,
        routeAlongTrackM: matchedBranch?.routeAlongTrackM ?? routeMatch?.alongTrackM ?? null,
        ...roadFrameFields(roadFrame),
      });
    }
    let viaRoute = null;
    // 회전/분기 의미가 있는 안내는 modelV2가 가까이 있더라도 Navi route를 먼저
    // 쓴다. model path는 카메라가 보는 현재 차로라서 좌·우회전 진출 가지를 반드시
    // 포함하지 않는다. anchor_store가 이후 model 높이/노면을 연속 보정한다.
    // routeMatch is a precision hint, not a gate. routeDistanceAnchor already
    // falls back to its own nearest-segment search when no match is supplied,
    // so requiring routeMatch here made a matcher miss (a distant route start,
    // a >75° turn segment, missing progress data) delete the sign entirely.
    // Match present → precise ordered branch; match absent → visible fallback.
    if (!viaGeo && routeAllowed && routeSemantic) {
      viaRoute = routeDistanceAnchor(vehicle, routePolyline, item.distanceM, { routeMatch });
      if (viaRoute) {
        let headingRad = viaRoute.headingRad ?? 0;
        if (maneuver) {
          const ahead = routeDistanceAnchor(
            vehicle,
            routePolyline,
            item.distanceM + AR_GEO_LIMITS.maneuverLookAheadM,
            { routeMatch },
          );
          if (ahead && Math.hypot(ahead.x - viaRoute.x, ahead.y - viaRoute.y) > 0.5) {
            headingRad = Math.atan2(ahead.y - viaRoute.y, ahead.x - viaRoute.x);
          }
        }
        const roadFrame = roadFrameFromHeading(headingRad);
        viaRoute = Object.freeze({ ...viaRoute, headingRad, ...roadFrameFields(roadFrame) });
      }
    }
    const viaPath = viaGeo || viaRoute ? null : pointOnPath(modelPosition, item.distanceM);
    if (!viaGeo && !viaRoute && !viaPath && routeAllowed) {
      viaRoute = routeDistanceAnchor(vehicle, routePolyline, item.distanceM, { routeMatch });
      if (viaRoute) {
        const roadFrame = roadFrameFromHeading(viaRoute.headingRad ?? 0);
        viaRoute = Object.freeze({ ...viaRoute, ...roadFrameFields(roadFrame) });
      }
    }
    const anchor = viaGeo || viaPath || viaRoute;
    if (anchor) out.push({
      ...item,
      anchor,
      placedBy: viaGeo || viaRoute ? "geo" : "path",
    });
    else if (failures) failures.push({
      src: item.source,
      dist: Math.round(Number(item.distanceM) || 0),
      pt: item.point ? 1 : 0,
      sem: routeSemantic ? 1 : 0,
    });
  }
  return out;
}

/** collectSigns + anchorSigns 한 번에. 워커/메인이 공통으로 부른다. */
export function composeFrame(input = {}, options = {}) {
  const signs = collectSigns(input, options);
  const route = input.navi?.route;
  const routeContext = {
    sessionId: input.navi?.sessionId,
    positionSigmaM: input.routePositionSigmaM,
  };
  const routeMatch = input.canDrawPrecise
    && (input.routeAllowed === true || input.geoAllowed === true)
    ? (options.routeMatcher?.match
      ? options.routeMatcher.match(input.navi?.vehicle, route, routeContext)
      : matchRoutePosition(input.navi?.vehicle, route, routeContext))
    : null;
  const failures = [];
  const anchored = input.canDrawPrecise
    ? anchorSigns(signs, {
      vehicle: input.navi?.vehicle,
      routePolyline: route?.polyline,
      routeMatch,
      modelPosition: input.modelPosition,
      geoAllowed: input.geoAllowed === true,
      routeAllowed: input.routeAllowed === true || input.geoAllowed === true,
      failures,
    })
    : null;

  /* 앵커가 왜 안 만들어졌는지 워커 안에서 본 그대로 보고한다. 메인 스레드
   * 스냅샷으로는 정상인데 워커에서만 실패하는 경우(직렬화 유실 등)를 잡으려면
   * "워커가 실제로 받은 입력"을 봐야 한다. */
  const vehicle = input.navi?.vehicle;
  const diag = Object.freeze({
    precise: input.canDrawPrecise === true,
    geo: input.geoAllowed === true,
    route: input.routeAllowed === true || input.geoAllowed === true,
    routePts: Array.isArray(route?.polyline) ? route.polyline.length : -1,
    veh: vehicle?.present !== false && Number.isFinite(Number(vehicle?.latitude)) ? 1 : 0,
    head: Number.isFinite(Number(vehicle?.headingDeg)) ? 1 : 0,
    matchIdx: routeMatch ? routeMatch.index : null,
    pathM: Array.isArray(input.modelPosition?.x)
      ? Math.round(Number(input.modelPosition.x.at(-1)) || 0)
      : null,
    fails: Object.freeze(failures.slice(0, 4)),
  });
  return { signs, fresh: anchored && anchored.length ? anchored : null, diag };
}
