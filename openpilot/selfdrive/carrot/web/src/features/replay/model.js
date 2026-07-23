"use strict";

window.CarrotReplayInsightsModel = window.CarrotReplayInsightsModel || (() => {
  const POLICY = Object.freeze({
    eventCooldownMs: Object.freeze({
      control: 1200,
      warning: 15000,
      turn: 5000,
      nav: 10000,
    }),
  });
  function createIndexer() {
    const events = [];
    const services = new Set();
    const previous = {};
    const lastEventAt = new Map();

    function pushEvent({ timeMs, category, type, params = {}, sourceTitle = "", sourceDetail = "", sourceTag = "", dedupeKey = "" }) {
      const key = `${category}:${type}:${dedupeKey}`;
      const cooldownMs = POLICY.eventCooldownMs[category] || 1500;
      const lastTimeMs = Number(lastEventAt.get(key));
      if (Number.isFinite(lastTimeMs) && timeMs - lastTimeMs < cooldownMs) return;
      lastEventAt.set(key, timeMs);
      events.push({
        id: `${Math.max(0, Math.round(timeMs))}-${events.length}`,
        timeMs: Math.max(0, Number(timeMs) || 0),
        category,
        type,
        params: { ...params },
        sourceTitle: String(sourceTitle || "").trim(),
        sourceDetail: String(sourceDetail || "").trim(),
        sourceTag: String(sourceTag || "").trim(),
      });
    }

    function ingest(frame, timeMs) {
      const service = String(frame?.service || "");
      const decoded = frame?.decoded;
      if (!service || !decoded || typeof decoded !== "object") return;
      services.add(service);

      if (service === "carState" || service === "radarState") return;

      if (service === "selfdriveState") {
        const enabled = Boolean(decoded.enabled);
        if (previous.enabled != null && previous.enabled !== enabled) {
          pushEvent({
            timeMs,
            category: "control",
            type: enabled ? "control_engaged" : "control_disengaged",
          });
        }
        previous.enabled = enabled;

        const alertStatus = Number(decoded.alertStatus || 0);
        const alertType = String(decoded.alertType || "").trim();
        const alertTitle = String(decoded.alertText1 || "").trim();
        const alertDetail = String(decoded.alertText2 || "").trim();
        const alertKey = alertStatus > 0 ? `${alertType}|${alertTitle}|${alertDetail}` : "";
        if (alertKey && alertKey !== previous.alertKey) {
          pushEvent({
            timeMs,
            category: "warning",
            type: "system_alert",
            params: { alertStatus, alertType },
            sourceTitle: alertTitle,
            sourceDetail: alertDetail,
            dedupeKey: alertType || alertTitle,
          });
        }
        previous.alertKey = alertKey;
        return;
      }

      if (service === "lateralPlan") {
        const laneState = Number(decoded.laneChangeState || 0);
        const direction = Number(decoded.laneChangeDirection || 0);
        if (!Number(previous.laneChangeState || 0) && laneState > 0) {
          const type = direction === 1
            ? "lane_change_left"
            : direction === 2
              ? "lane_change_right"
              : "lane_change";
          pushEvent({ timeMs, category: "turn", type, dedupeKey: String(direction) });
        }
        previous.laneChangeState = laneState;
        return;
      }

      if (service === "carrotMan") {
        const turnInfo = Number(decoded.xTurnInfo || 0);
        const title = String(decoded.szTBTMainText || decoded.szPosRoadName || "").trim();
        const distanceM = Math.max(0, Number(decoded.xDistToTurn || 0));
        const navKey = turnInfo ? `${turnInfo}|${title}` : "";
        if (navKey && navKey !== previous.navKey) {
          pushEvent({
            timeMs,
            category: "nav",
            type: "navigation_maneuver",
            params: { turnInfo, distanceM },
            sourceTitle: title,
            sourceTag: "CarrotMan",
            dedupeKey: navKey,
          });
        }
        previous.navKey = navKey;
      }
    }

    return { events, services, ingest };
  }

  function downsample(series, limit) {
    const safeLimit = Math.max(2, Number(limit) || 2);
    if (series.length <= safeLimit) return series;
    const result = [];
    const step = (series.length - 1) / (safeLimit - 1);
    for (let index = 0; index < safeLimit; index += 1) {
      result.push(series[Math.min(series.length - 1, Math.round(index * step))]);
    }
    return result;
  }

  function nearestSample(series, timeMs) {
    if (!series.length) return null;
    let low = 0;
    let high = series.length - 1;
    while (low < high) {
      const middle = Math.floor((low + high) / 2);
      if (series[middle].timeMs < timeMs) low = middle + 1;
      else high = middle;
    }
    const current = series[low];
    const prior = series[Math.max(0, low - 1)];
    return Math.abs(prior.timeMs - timeMs) <= Math.abs(current.timeMs - timeMs) ? prior : current;
  }

  function layoutEvents(events, durationSeconds, railWidth) {
    const width = Math.max(1, Number(railWidth || 0) || 320);
    const durationMs = Math.max(1, Number(durationSeconds) * 1000);
    // Preserve the original single timeline and merge only marks that would
    // occupy the same two CSS pixels. Time-window clustering made short drives
    // look empty and hid unrelated events behind large counters.
    const collisionPx = 2;
    const groups = [];
    const sorted = [...(Array.isArray(events) ? events : [])]
      .filter((event) => Number.isFinite(Number(event?.timeMs)))
      .sort((left, right) => Number(left.timeMs) - Number(right.timeMs));

    for (const event of sorted) {
      const pixel = Math.max(0, Math.min(width, Number(event.timeMs) / durationMs * width));
      const previous = groups[groups.length - 1];
      // Keep the first mark as the collision anchor so a chain of close events
      // cannot collapse into one large group.
      if (previous && Math.abs(pixel - previous.anchorPx) <= collisionPx) {
        previous.events.push(event);
        previous.pixelTotal += pixel;
        previous.pixel = previous.pixelTotal / previous.events.length;
        previous.leftPercent = previous.pixel / width * 100;
      } else {
        groups.push({
          anchorPx: pixel,
          pixel,
          pixelTotal: pixel,
          leftPercent: pixel / width * 100,
          events: [event],
        });
      }
    }
    return groups;
  }

  return { POLICY, createIndexer, downsample, nearestSample, layoutEvents, clusterEvents: layoutEvents };
})();
