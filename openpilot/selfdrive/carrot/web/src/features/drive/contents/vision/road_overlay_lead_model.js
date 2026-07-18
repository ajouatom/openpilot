const LONG_PLAN_SOURCE_LEAD1 = 2;

function finiteNumber(value, fallback = 0) {
  const number = Number(value);
  return Number.isFinite(number) ? number : fallback;
}

export function createRoadOverlayLeadModel(options = {}) {
  const text = typeof options.text === "function" ? options.text : (_key, fallback) => fallback;

  function getPrimaryVisionDistance(model) {
    const lead = Array.isArray(model?.leadsV3) ? model.leadsV3[0] : null;
    const prob = finiteNumber(lead?.prob, 0);
    const distance = finiteNumber(lead?.x?.[0], 0);
    return prob > 0.5 && distance > 0 ? Math.max(0, distance - 1.52) : 0;
  }

  function resolveLeadState(overlayState, hudState) {
    const longPlan = hudState?.longitudinalPlan || {};
    const carrotMan = overlayState?.carrotMan || hudState?.carrotMan || {};
    const carState = hudState?.carState || {};
    const xState = finiteNumber(longPlan?.xState, finiteNumber(carrotMan?.xState, -1));
    const trafficState = finiteNumber(longPlan?.trafficState, finiteNumber(carrotMan?.trafficState, -1));
    const longActive = Boolean(hudState?.controlsState?.enabled);
    const vEgoMps = finiteNumber(carState?.vEgo, finiteNumber(carState?.vEgoCluster, 0));
    const brakeHoldActive = Boolean(carState?.brakeHoldActive);
    const softHoldActive = finiteNumber(carState?.softHoldActive, 0) > 0;
    const carrotCruise = finiteNumber(carState?.carrotCruise, 0) > 0;

    if (brakeHoldActive || softHoldActive || carrotCruise) {
      return {
        text: brakeHoldActive ? "AUTOHOLD" : (softHoldActive ? "SOFTHOLD" : "CARROT"),
        xState,
        showDistanceBadge: false,
      };
    }
    if (!longActive) {
      return { text: "", xState, showDistanceBadge: true };
    }
    if (xState === 3 || xState === 5) {
      return {
        text: vEgoMps < 1.0 ? (trafficState >= 1000 ? "Signal Error" : "Signal Ready") : "Signal slowing",
        xState,
        showDistanceBadge: false,
      };
    }
    if (xState === 4) {
      return {
        text: text("e2e_driving", "E2E driving"),
        xState,
        showDistanceBadge: false,
      };
    }
    if (xState === 0 || xState === 1 || xState === 2) {
      return { text: "", xState, showDistanceBadge: true };
    }
    return { text: "", xState, showDistanceBadge: false };
  }

  function getDistanceBadgeTextColor(xState) {
    if (xState === 0) return "#ffffff";
    if (xState === 1) return "#b0b0b0";
    return "#23d55d";
  }

  function getPathStatusText(hudState) {
    const carState = hudState?.carState || {};
    if (Boolean(carState?.brakeHoldActive)) return "AUTOHOLD";
    if (finiteNumber(carState?.softHoldActive, 0) > 0) return "SOFTHOLD";
    if (finiteNumber(carState?.carrotCruise, 0) > 0) return "CARROT";
    return "";
  }

  function getRadarTracks(radarState) {
    const source = radarState && typeof radarState === "object" ? radarState : {};
    const tracks = [
      ...(Array.isArray(source.leadsLeft) ? source.leadsLeft : []),
      ...(Array.isArray(source.leadsRight) ? source.leadsRight : []),
      ...(Array.isArray(source.leadsCenter) ? source.leadsCenter : []),
    ];
    if (tracks.length) return tracks;

    const fallback = [];
    if (source.leadOne?.status) fallback.push(source.leadOne);
    if (source.leadTwo?.status) fallback.push(source.leadTwo);
    return fallback;
  }

  function getRadarProjectionLine(model) {
    const laneLines = Array.isArray(model?.laneLines) ? model.laneLines : [];
    const centerLane = laneLines[2];
    if (Array.isArray(centerLane?.x) && Array.isArray(centerLane?.z) && centerLane.x.length && centerLane.z.length) {
      return centerLane;
    }
    return model?.position || null;
  }

  function resolveLeadTwo(radarState, longitudinalPlan) {
    const lead = radarState?.leadTwo || null;
    const valid = Boolean(lead?.status)
      && Boolean(lead?.radar)
      && finiteNumber(lead?.dRel, 0) > (finiteNumber(radarState?.leadOne?.dRel, 0) + 3);
    const status = finiteNumber(longitudinalPlan?.longitudinalPlanSource, 0) === LONG_PLAN_SOURCE_LEAD1 ? 2 : 1;
    return { lead, valid, status };
  }

  return Object.freeze({
    getPrimaryVisionDistance,
    resolveLeadState,
    getDistanceBadgeTextColor,
    getPathStatusText,
    getRadarTracks,
    getRadarProjectionLine,
    resolveLeadTwo,
  });
}

export const DriveVisionRoadOverlayLeadModel = Object.freeze({
  create: createRoadOverlayLeadModel,
});

export function installDriveVisionRoadOverlayLeadModelFacade(target = globalThis) {
  target.DriveVisionRoadOverlayLeadModel = DriveVisionRoadOverlayLeadModel;
  return DriveVisionRoadOverlayLeadModel;
}
