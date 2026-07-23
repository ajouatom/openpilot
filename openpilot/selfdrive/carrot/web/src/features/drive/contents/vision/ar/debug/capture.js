"use strict";

/* One-click replay diagnostic capture.
 *
 * The collector drives the real replay surface at 1x and records worker
 * completion packets. It does not synthesize AR frames, so clock mapping,
 * tracking transitions, worker backpressure and renderer visibility are the
 * same path the user actually sees. The resulting JSON is intentionally
 * compact enough to attach while retaining exact replay timestamps.
 */

export const AR_REPLAY_CAPTURE_SCHEMA = 3;
export const AR_REPLAY_CAPTURE_SECONDS = 60;
export const AR_REPLAY_CAPTURE_MAX_FRAMES = 3600;
export const AR_REPLAY_CAPTURE_DRAIN_MS = 2500;
const AR_REPLAY_CAPTURE_COVERAGE_TOLERANCE_MS = 250;

function finite(value, fallback = null) {
  const number = Number(value);
  return Number.isFinite(number) ? number : fallback;
}

function rounded(value, digits = 3) {
  const number = finite(value);
  if (number === null) return null;
  const scale = 10 ** digits;
  return Math.round(number * scale) / scale;
}

function strings(values) {
  return Array.isArray(values) ? values.map(String).filter(Boolean) : [];
}

function compactAnchor(anchor) {
  if (!anchor) return null;
  return {
    x: rounded(anchor.x), y: rounded(anchor.y), z: rounded(anchor.z),
    markerId: anchor.markerId ? String(anchor.markerId) : undefined,
    lifecycleSlot: anchor.lifecycleSlot ? String(anchor.lifecycleSlot) : undefined,
    headingRad: rounded(anchor.headingRad, 5),
    billboardYawRad: rounded(anchor.billboardYawRad, 5),
    source: anchor.source ? String(anchor.source) : undefined,
    roadGrade: rounded(anchor.roadGrade, 5),
    heightSource: anchor.heightSource ? String(anchor.heightSource) : undefined,
    heightConfidence: rounded(anchor.heightConfidence, 3),
    heightLimited: anchor.heightLimited === true,
    routeDerived: anchor.routeDerived === true,
  };
}

function compactMarker(marker) {
  return {
    markerId: String(marker?.markerId || ""),
    lifecycleSlot: String(marker?.lifecycleSlot || ""),
    source: String(marker?.source || "unknown"),
    kind: String(marker?.kind || ""),
    distanceM: rounded(marker?.distanceM, 2),
    visible: marker?.visible === true,
    visibilityState: String(marker?.visibilityState || ""),
    reason: String(marker?.reason || ""),
    phase: String(marker?.phase || ""),
    confidence: String(marker?.confidence || ""),
    alpha: rounded(marker?.alpha, 4),
    worldScale: rounded(marker?.worldScale, 4),
    anchor: compactAnchor(marker?.anchor),
    screen: marker?.screen ? {
      centerX: rounded(marker.screen.centerX, 2),
      centerY: rounded(marker.screen.centerY, 2),
      widthPx: rounded(marker.screen.widthPx, 2),
      heightPx: rounded(marker.screen.heightPx, 2),
      depth: rounded(marker.screen.depth, 4),
    } : null,
  };
}

function compactHandoff(handoff) {
  if (!handoff) return null;
  return {
    state: String(handoff.state || ""),
    reason: handoff.reason ? String(handoff.reason) : undefined,
    progress: rounded(handoff.progress, 4),
    durationMs: rounded(handoff.durationMs, 1),
    positionInnovationM: rounded(handoff.positionInnovationM),
    orientationInnovationRad: rounded(handoff.orientationInnovationRad, 5),
    lateralInnovationM: rounded(handoff.lateralInnovationM),
    heightInnovationM: rounded(handoff.heightInnovationM),
  };
}

export function compactArDiagnosticFrame(packet = {}, replayStatus = {}) {
  const debug = packet.debugFrame || {};
  const clock = debug.presentedClock || {};
  const sync = debug.sync || {};
  const tracking = packet.tracking || {};
  const hold = packet.hold || {};
  const composition = packet.composition || {};
  const renderer = packet.renderer || {};
  const worldPose = packet.worldPose || {};
  const mediaTimeMs = finite(debug.replayTimeMs, finite(replayStatus.currentTime, 0) * 1000);
  return {
    mediaTimeMs: rounded(mediaTimeMs, 1),
    traceFrameId: finite(packet.traceFrameId),
    ok: packet.ok === true,
    presented: {
      sequence: finite(debug.presented?.sequence),
      sourceFrameId: finite(clock.sourceFrameId ?? debug.presented?.frameId),
      confidence: String(clock.confidence || "unmapped"),
      mapped: clock.mapped === true,
      targetTimestampNs: clock.targetTimestampNs === null || clock.targetTimestampNs === undefined
        ? null : finite(clock.targetTimestampNs),
    },
    sources: {
      cameraFrameId: finite(debug.sources?.cameraFrameId),
      modelFrameId: finite(debug.sources?.modelFrameId),
      odometryFrameId: finite(debug.sources?.odometryFrameId),
      livePoseTimestamp: finite(debug.sources?.livePoseTimestamp),
    },
    sync: {
      state: String(sync.state || ""),
      precise: sync.canDrawPrecise === true,
      hold: sync.canHoldAnchor === true,
      frameIdGap: finite(sync.frameIdGap),
      modelAgeMs: rounded(sync.modelAgeMs, 1),
      odometryAgeMs: rounded(sync.odometryAgeMs, 1),
      poseAgeMs: rounded(sync.poseAgeMs, 1),
      naviAgeMs: rounded(sync.naviAgeMs, 1),
      naviUsable: sync.naviUsable === true,
      reasons: strings(sync.reasons),
    },
    navi: debug.navi || null,
    positionQuality: debug.positionQuality || null,
    tracking: {
      state: String(tracking.state || ""),
      canCreateAnchor: tracking.canCreateAnchor === true,
      canPropagateAnchor: tracking.canPropagateAnchor === true,
      retainAnchor: tracking.retainAnchor === true,
      alpha: rounded(tracking.alpha, 4),
      predictionAgeMs: rounded(tracking.predictionAgeMs, 1),
      lateralUncertaintyM: rounded(tracking.uncertainty?.lateralM),
      budgetM: rounded(tracking.uncertainty?.budgetM),
      reasons: strings(tracking.reasons),
    },
    composition: {
      signs: finite(composition.signCount, 0),
      anchors: finite(composition.anchoredCount, 0),
      sources: strings(composition.sources),
      gate: composition.diag || null,
    },
    hold: {
      state: String(hold.state || ""),
      sourceMode: hold.sourceMode ? String(hold.sourceMode) : null,
      anchorCount: finite(hold.anchorCount, 0),
      unresolvedCount: finite(hold.unresolvedCount, 0),
      creationDeferredCount: finite(hold.creationDeferredCount, 0),
      creationGateReason: hold.creationGateReason ? String(hold.creationGateReason) : null,
      handoffCount: finite(hold.handoffCount, 0),
      fixId: hold.fixId ? String(hold.fixId) : null,
      sourceId: hold.sourceId ? String(hold.sourceId) : null,
      worldEpoch: hold.worldEpoch ? String(hold.worldEpoch) : null,
      holdMs: rounded(hold.holdMs, 1),
      driftM: rounded(hold.driftM),
      reason: String(hold.reason || ""),
      sample: compactAnchor(hold.sample),
      handoff: compactHandoff(hold.handoff),
    },
    worldPose: {
      initialized: worldPose.initialized === true,
      epoch: String(worldPose.epoch || debug.worldEpoch || ""),
      integrations: finite(worldPose.integrations, 0),
      position: Array.isArray(worldPose.position)
        ? worldPose.position.slice(0, 3).map((value) => rounded(value, 4)) : null,
      orientation: Array.isArray(worldPose.orientation)
        ? worldPose.orientation.slice(0, 4).map((value) => rounded(value, 6)) : null,
      reason: String(worldPose.reason || ""),
      dtMs: rounded(worldPose.dtMs, 2),
      geoCorrection: worldPose.geoCorrection ? {
        referenceReady: worldPose.geoCorrection.referenceReady === true,
        accepted: finite(worldPose.geoCorrection.accepted, 0),
        rejected: finite(worldPose.geoCorrection.rejected, 0),
        yawAccepted: finite(worldPose.geoCorrection.yawAccepted, 0),
        yawRejected: finite(worldPose.geoCorrection.yawRejected, 0),
        appliedFrames: finite(worldPose.geoCorrection.appliedFrames, 0),
        positionInnovationM: rounded(worldPose.geoCorrection.lastPositionInnovationM),
        yawInnovationRad: rounded(worldPose.geoCorrection.lastYawInnovationRad, 5),
        positionSigmaM: rounded(worldPose.geoCorrection.lastPositionSigmaM),
        headingSigmaDeg: rounded(worldPose.geoCorrection.lastHeadingSigmaDeg),
        reason: String(worldPose.geoCorrection.lastReason || ""),
      } : null,
    },
    renderer: {
      drawn: finite(renderer.drawn, 0),
      skipped: finite(renderer.skipped, 0),
      selectionSuppressed: finite(renderer.selectionSuppressed, 0),
      trackingAlpha: rounded(renderer.trackingAlpha, 4),
      minimumVisibilityAlpha: rounded(renderer.minimumVisibilityAlpha, 4),
      reason: String(renderer.lastReason || ""),
      visibilityStates: renderer.visibilityStates && typeof renderer.visibilityStates === "object"
        ? { ...renderer.visibilityStates } : {},
      markers: Array.isArray(renderer.markers) ? renderer.markers.map(compactMarker) : [],
    },
    performance: packet.performance ? {
      fps: rounded(packet.performance.fps, 1),
      averageWorkMs: rounded(packet.performance.averageWorkMs, 3),
      lastWorkMs: rounded(packet.performance.lastWorkMs, 3),
      slowFrames: finite(packet.performance.slowFrames, 0),
      level: String(packet.performance.level || ""),
    } : null,
    transport: packet.transport ? {
      dpr: rounded(packet.transport.dpr, 2),
      queue: packet.transport.queue ? {
        posted: finite(packet.transport.queue.posted, 0),
        completed: finite(packet.transport.queue.completed, 0),
        deferred: finite(packet.transport.queue.deferred, 0),
        replaced: finite(packet.transport.queue.replaced, 0),
        discarded: finite(packet.transport.queue.discarded, 0),
        inFlight: packet.transport.queue.inFlight === true,
        pending: packet.transport.queue.pending === true,
        maxPending: finite(packet.transport.queue.maxPending, 1),
      } : null,
      sourceTransfers: packet.transport.sourceTransfers ? {
        navi: finite(packet.transport.sourceTransfers.navi, 0),
        modelPosition: finite(packet.transport.sourceTransfers.modelPosition, 0),
        reusedNavi: finite(packet.transport.sourceTransfers.reusedNavi, 0),
        reusedModelPosition: finite(packet.transport.sourceTransfers.reusedModelPosition, 0),
      } : null,
    } : null,
  };
}

function bump(object, key) {
  const value = String(key || "unknown");
  object[value] = (object[value] || 0) + 1;
}

function pushEvent(events, type, frame, detail = {}) {
  if (events.length >= 1200) return;
  events.push({ type, mediaTimeMs: frame.mediaTimeMs, traceFrameId: frame.traceFrameId, ...detail });
}

function eventDue(session, key, timeMs, intervalMs) {
  const previous = finite(session.lastEventAt?.get(key), Number.NEGATIVE_INFINITY);
  if (timeMs - previous < intervalMs) return false;
  session.lastEventAt?.set(key, timeMs);
  return true;
}

function clockMismatch(frame) {
  const publish = finite(frame.navi?.publishMonoTimeNanos);
  const target = finite(frame.presented?.targetTimestampNs);
  return frame.presented?.mapped === true
    && publish !== null && publish > 0
    && target !== null && target > 0
    && Math.abs(publish - target) > 5_000_000_000;
}

function observeAnomalies(session, frame) {
  const previous = session.frames.at(-1) || null;
  if (!frame.presented.mapped && eventDue(session, "clock-unmapped", frame.mediaTimeMs, 500)) {
    pushEvent(session.events, "clock-unmapped", frame);
  }
  if ((frame.sync.frameIdGap ?? 0) > 8) {
    pushEvent(session.events, "frame-gap", frame, { gap: frame.sync.frameIdGap });
  }
  if (
    frame.hold.sample && Math.abs(frame.hold.sample.y ?? 0) > 8
    && eventDue(session, "anchor-lateral-outlier", frame.mediaTimeMs, 500)
  ) {
    pushEvent(session.events, "anchor-lateral-outlier", frame, {
      x: frame.hold.sample.x, y: frame.hold.sample.y, fixId: frame.hold.fixId,
    });
  }
  if (
    frame.hold.anchorCount > 0 && frame.renderer.drawn === 0
    && eventDue(session, "anchor-not-rendered", frame.mediaTimeMs, 500)
  ) {
    pushEvent(session.events, "anchor-not-rendered", frame, {
      reason: frame.renderer.reason,
      visibilityStates: frame.renderer.visibilityStates,
      markers: frame.renderer.markers.map((marker) => ({
        markerId: marker.markerId,
        state: marker.visibilityState,
        reason: marker.reason,
      })),
    });
  }
  if (!previous) return;
  if (
    previous.hold.sample?.markerId
    && previous.hold.sample.markerId === frame.hold.sample?.markerId
  ) {
    const jumpM = Math.hypot(
      finite(frame.hold.sample.x, 0) - finite(previous.hold.sample.x, 0),
      finite(frame.hold.sample.y, 0) - finite(previous.hold.sample.y, 0),
    );
    if (jumpM >= 8) {
      pushEvent(session.events, "anchor-position-jump", frame, {
        meters: rounded(jumpM, 1),
        from: compactAnchor(previous.hold.sample),
        to: compactAnchor(frame.hold.sample),
        fromFixId: previous.hold.fixId,
        toFixId: frame.hold.fixId,
      });
    }
  }
  if (previous.tracking.state !== frame.tracking.state) {
    pushEvent(session.events, "tracking-transition", frame, {
      from: previous.tracking.state, to: frame.tracking.state,
    });
  }
  if ((previous.renderer.drawn > 0) !== (frame.renderer.drawn > 0)) {
    pushEvent(session.events, frame.renderer.drawn > 0 ? "marker-visible" : "marker-hidden", frame);
  }
  if (previous.hold.state !== frame.hold.state) {
    pushEvent(session.events, "hold-transition", frame, {
      from: previous.hold.state, to: frame.hold.state, reason: frame.hold.reason,
    });
  }
  const previousMatch = previous.composition.gate?.matchIdx ?? null;
  const nextMatch = frame.composition.gate?.matchIdx ?? null;
  if (previousMatch !== nextMatch) {
    pushEvent(session.events, "route-match-changed", frame, { from: previousMatch, to: nextMatch });
  }
  if (previous.worldPose.integrations > frame.worldPose.integrations) {
    pushEvent(session.events, "world-pose-reset", frame, {
      from: previous.worldPose.integrations, to: frame.worldPose.integrations,
    });
  }
  const previousNavi = previous.navi || {};
  const nextNavi = frame.navi || {};
  const naviSignature = (value) => [
    value.present, value.guidanceActive, value.routePresent,
    value.vehicle?.present, value.route?.pointCount,
  ].join(":");
  if (naviSignature(previousNavi) !== naviSignature(nextNavi)) {
    pushEvent(session.events, "navi-snapshot-changed", frame, {
      from: naviSignature(previousNavi), to: naviSignature(nextNavi),
    });
  }
  if (clockMismatch(previous) !== clockMismatch(frame)) {
    pushEvent(session.events, "navi-clock-domain-transition", frame, {
      from: clockMismatch(previous) ? "mismatch" : "aligned",
      to: clockMismatch(frame) ? "mismatch" : "aligned",
      naviPublishMonoTimeNanos: finite(frame.navi?.publishMonoTimeNanos),
      presentedTargetTimestampNs: finite(frame.presented?.targetTimestampNs),
    });
  }
}

function routeGeometry(target) {
  let navi = null;
  try { navi = target.CarrotDriveLiveStateProvider?.snapshot?.()?.overlayState?.carrotNavi; } catch {}
  const points = Array.isArray(navi?.route?.polyline) ? navi.route.polyline : [];
  if (points.length < 2) return null;
  const compact = points.map((point) => ({
    latitude: rounded(point?.latitude, 7), longitude: rounded(point?.longitude, 7),
  }));
  const middle = compact[Math.floor(compact.length / 2)];
  const signature = [
    String(navi?.sessionId || "legacy"), compact.length,
    compact[0]?.latitude, compact[0]?.longitude,
    middle?.latitude, middle?.longitude,
    compact.at(-1)?.latitude, compact.at(-1)?.longitude,
  ].join(":");
  return { signature, points: compact };
}

export function markerContinuitySummary(frames) {
  const histories = new Map();
  const maxAdjacentGapMs = 350;
  const blinkWindowMs = 2000;
  for (let frameIndex = 0; frameIndex < frames.length; frameIndex += 1) {
    const frame = frames[frameIndex];
    const mediaTimeMs = finite(frame?.mediaTimeMs);
    if (mediaTimeMs === null) continue;
    const seen = new Set();
    for (const marker of Array.isArray(frame?.renderer?.markers) ? frame.renderer.markers : []) {
      const markerId = String(marker?.markerId || marker?.lifecycleSlot || "");
      if (!markerId || seen.has(markerId)) continue;
      seen.add(markerId);
      const visible = marker?.visible === true;
      const history = histories.get(markerId) || {
        markerId,
        lastVisible: null,
        lastMediaTimeMs: null,
        lastFrameIndex: null,
        transitions: [],
      };
      const adjacent = history.lastFrameIndex === frameIndex - 1
        && mediaTimeMs - history.lastMediaTimeMs <= maxAdjacentGapMs;
      if (adjacent && history.lastVisible !== null && history.lastVisible !== visible) {
        history.transitions.push(mediaTimeMs);
      }
      history.lastVisible = visible;
      history.lastMediaTimeMs = mediaTimeMs;
      history.lastFrameIndex = frameIndex;
      histories.set(markerId, history);
    }
  }

  let transitions = 0;
  let maxRapidTransitions = 0;
  const blinkingIds = [];
  for (const history of histories.values()) {
    transitions += history.transitions.length;
    let blinking = false;
    for (let start = 0; start < history.transitions.length; start += 1) {
      let end = start;
      while (
        end + 1 < history.transitions.length
        && history.transitions[end + 1] - history.transitions[start] <= blinkWindowMs
      ) end += 1;
      const count = end - start + 1;
      maxRapidTransitions = Math.max(maxRapidTransitions, count);
      // One enter + one exit is a normal camera-FOV crossing. A third toggle
      // inside two seconds means the same semantic object visibly returned.
      if (count >= 3) blinking = true;
    }
    if (blinking) blinkingIds.push(history.markerId);
  }
  return Object.freeze({
    markerCount: histories.size,
    transitions,
    maxRapidTransitions,
    blinkingIds: Object.freeze(blinkingIds.sort()),
  });
}

function captureSummary(session) {
  const frames = session.frames;
  const firstMediaTimeMs = finite(frames[0]?.mediaTimeMs);
  const lastMediaTimeMs = finite(frames.at(-1)?.mediaTimeMs);
  const requestedEndMs = session.durationSeconds * 1000;
  const capturedSpanMs = firstMediaTimeMs !== null && lastMediaTimeMs !== null
    ? Math.max(0, lastMediaTimeMs - firstMediaTimeMs) : 0;
  const summary = {
    frames: frames.length,
    events: session.events.length,
    tracking: {}, hold: {}, clock: {}, visibility: {}, markerStates: {}, eventTypes: {},
    anchorRange: { x: [null, null], y: [null, null] },
    coverage: {
      requestedStartMs: 0,
      requestedEndMs: rounded(requestedEndMs, 1),
      firstMediaTimeMs: rounded(firstMediaTimeMs, 1),
      lastMediaTimeMs: rounded(lastMediaTimeMs, 1),
      capturedSpanMs: rounded(capturedSpanMs, 1),
      missingTailMs: rounded(Math.max(0, requestedEndMs - (lastMediaTimeMs ?? 0)), 1),
      ratio: rounded(requestedEndMs > 0 ? Math.min(1, (lastMediaTimeMs ?? 0) / requestedEndMs) : 0, 4),
      complete: firstMediaTimeMs !== null
        && firstMediaTimeMs <= AR_REPLAY_CAPTURE_COVERAGE_TOLERANCE_MS
        && lastMediaTimeMs >= requestedEndMs - AR_REPLAY_CAPTURE_COVERAGE_TOLERANCE_MS,
      effectiveFps: rounded(capturedSpanMs > 0 ? (frames.length - 1) * 1000 / capturedSpanMs : 0, 2),
    },
    transitions: { tracking: 0, hold: 0, visibility: 0, naviUsable: 0 },
    navi: { usableFrames: 0, unusableFrames: 0, clockMismatchFrames: 0 },
    worldPose: {
      integratedFrames: 0,
      maxIntegrations: 0,
      geoAccepted: 0,
      geoRejected: 0,
      geoAppliedFrames: 0,
      yawAccepted: 0,
      yawRejected: 0,
      anchorCreationDeferredFrames: 0,
      anchorCreationGateReasons: {},
    },
    markerContinuity: null,
    transport: {
      maxDpr: 0,
      queue: { deferred: 0, replaced: 0, discarded: 0, maxPending: 0 },
      sourceTransfers: { navi: 0, modelPosition: 0, reusedNavi: 0, reusedModelPosition: 0 },
    },
    reasons: { sync: {}, tracking: {}, hold: {} },
    flags: [],
  };
  const span = (range, value) => {
    const n = finite(value);
    if (n === null) return;
    range[0] = range[0] === null ? n : Math.min(range[0], n);
    range[1] = range[1] === null ? n : Math.max(range[1], n);
  };
  const previous = { tracking: null, hold: null, visibility: null, naviUsable: null };
  for (const frame of frames) {
    bump(summary.tracking, frame.tracking.state);
    bump(summary.hold, frame.hold.state);
    bump(summary.clock, frame.presented.confidence);
    const visibility = frame.renderer.drawn > 0 ? "visible" : "hidden";
    const naviUsable = frame.sync.naviUsable === true;
    bump(summary.visibility, visibility);
    for (const [state, count] of Object.entries(frame.renderer.visibilityStates || {})) {
      summary.markerStates[state] = (summary.markerStates[state] || 0) + Number(count || 0);
    }
    summary.navi[naviUsable ? "usableFrames" : "unusableFrames"] += 1;
    if (clockMismatch(frame)) summary.navi.clockMismatchFrames += 1;
    const integrations = finite(frame.worldPose?.integrations, 0);
    if (integrations > 0) summary.worldPose.integratedFrames += 1;
    summary.worldPose.maxIntegrations = Math.max(summary.worldPose.maxIntegrations, integrations);
    summary.worldPose.geoAccepted = Math.max(
      summary.worldPose.geoAccepted,
      finite(frame.worldPose?.geoCorrection?.accepted, 0),
    );
    summary.worldPose.geoRejected = Math.max(
      summary.worldPose.geoRejected,
      finite(frame.worldPose?.geoCorrection?.rejected, 0),
    );
    summary.worldPose.geoAppliedFrames = Math.max(
      summary.worldPose.geoAppliedFrames,
      finite(frame.worldPose?.geoCorrection?.appliedFrames, 0),
    );
    summary.worldPose.yawAccepted = Math.max(
      summary.worldPose.yawAccepted,
      finite(frame.worldPose?.geoCorrection?.yawAccepted, 0),
    );
    summary.worldPose.yawRejected = Math.max(
      summary.worldPose.yawRejected,
      finite(frame.worldPose?.geoCorrection?.yawRejected, 0),
    );
    if (finite(frame.hold?.creationDeferredCount, 0) > 0) {
      summary.worldPose.anchorCreationDeferredFrames += 1;
      bump(summary.worldPose.anchorCreationGateReasons, frame.hold.creationGateReason);
    }
    summary.transport.maxDpr = Math.max(summary.transport.maxDpr, finite(frame.transport?.dpr, 0));
    for (const key of ["deferred", "replaced", "discarded", "maxPending"]) {
      summary.transport.queue[key] = Math.max(
        summary.transport.queue[key],
        finite(frame.transport?.queue?.[key], 0),
      );
    }
    for (const key of ["navi", "modelPosition", "reusedNavi", "reusedModelPosition"]) {
      summary.transport.sourceTransfers[key] = Math.max(
        summary.transport.sourceTransfers[key],
        finite(frame.transport?.sourceTransfers?.[key], 0),
      );
    }
    for (const reason of strings(frame.sync?.reasons)) bump(summary.reasons.sync, reason);
    for (const reason of strings(frame.tracking?.reasons)) bump(summary.reasons.tracking, reason);
    if (frame.hold?.reason) bump(summary.reasons.hold, frame.hold.reason);
    for (const [key, value] of Object.entries({
      tracking: frame.tracking.state,
      hold: frame.hold.state,
      visibility,
      naviUsable,
    })) {
      if (previous[key] !== null && previous[key] !== value) summary.transitions[key] += 1;
      previous[key] = value;
    }
    span(summary.anchorRange.x, frame.hold.sample?.x);
    span(summary.anchorRange.y, frame.hold.sample?.y);
  }
  for (const event of session.events) bump(summary.eventTypes, event.type);
  summary.anchorRange.x = summary.anchorRange.x.map((value) => rounded(value));
  summary.anchorRange.y = summary.anchorRange.y.map((value) => rounded(value));
  const top = (counts, limit = 8) => Object.fromEntries(
    Object.entries(counts).sort((a, b) => b[1] - a[1]).slice(0, limit),
  );
  summary.reasons.sync = top(summary.reasons.sync);
  summary.reasons.tracking = top(summary.reasons.tracking);
  summary.reasons.hold = top(summary.reasons.hold);
  summary.worldPose.anchorCreationGateReasons = top(
    summary.worldPose.anchorCreationGateReasons,
  );
  summary.markerContinuity = markerContinuitySummary(frames);
  if (!summary.coverage.complete) summary.flags.push("capture-incomplete");
  if (summary.navi.clockMismatchFrames > 0) summary.flags.push("mixed-clock-domain");
  if (summary.transitions.naviUsable > 4) summary.flags.push("navi-flapping");
  if (summary.markerContinuity.blinkingIds.length > 0) summary.flags.push("marker-blinking");
  if (summary.worldPose.integratedFrames === 0) summary.flags.push("world-pose-not-integrated");
  if ((summary.tracking.lost || 0) > frames.length * 0.2) summary.flags.push("tracking-mostly-lost");
  return summary;
}

function seconds(value) {
  const milliseconds = finite(value);
  return milliseconds === null ? "—" : `${(milliseconds / 1000).toFixed(2)}초`;
}

function countLines(title, counts, limit = 5) {
  const entries = Object.entries(counts || {}).sort((a, b) => b[1] - a[1]).slice(0, limit);
  return entries.length ? [`${title}:`, ...entries.map(([key, count]) => `- ${key} ×${count}`)] : [];
}

/** Large frame logs stay in the JSON; this report is the small pasteable view. */
export function formatArReplayDiagnosticReport(payload = {}) {
  const summary = payload.summary || {};
  const coverage = summary.coverage || {};
  const replay = payload.replay || {};
  const lines = [
    "AR 리플레이 진단 요약",
    `구간: ${replay.segment || "—"}`,
    `판정: ${coverage.complete ? "수집 완료" : "수집 불완전"} · 종료 ${payload.stopReason || "—"}`,
    `범위: ${seconds(coverage.firstMediaTimeMs)} ~ ${seconds(coverage.lastMediaTimeMs)} / 요청 ${seconds(coverage.requestedEndMs)}`,
    `프레임: ${summary.frames || 0} · 실효 ${finite(coverage.effectiveFps, 0).toFixed(2)}fps · 이벤트 ${summary.events || 0}`,
    `상태전환: Navi ${summary.transitions?.naviUsable || 0} · 표시 ${summary.transitions?.visibility || 0} · tracking ${summary.transitions?.tracking || 0} · hold ${summary.transitions?.hold || 0}`,
    `Navi: 유효 ${summary.navi?.usableFrames || 0} · 무효 ${summary.navi?.unusableFrames || 0} · 시계불일치 ${summary.navi?.clockMismatchFrames || 0}`,
    `worldPose: 적분 프레임 ${summary.worldPose?.integratedFrames || 0} · 최대 ${summary.worldPose?.maxIntegrations || 0}`,
    `새 앵커 보류: ${summary.worldPose?.anchorCreationDeferredFrames || 0}프레임`,
    `Worker: DPR 최대 ${finite(summary.transport?.maxDpr, 0).toFixed(2)} · 대기 ${summary.transport?.queue?.deferred || 0} · 교체 ${summary.transport?.queue?.replaced || 0} · 폐기 ${summary.transport?.queue?.discarded || 0}`,
    `앵커: x ${summary.anchorRange?.x?.join(" ~ ") || "—"}m · y ${summary.anchorRange?.y?.join(" ~ ") || "—"}m`,
    `플래그: ${summary.flags?.length ? summary.flags.join(", ") : "없음"}`,
    ...countLines("마커 상태", summary.markerStates, 10),
    ...countLines("주요 tracking 차단", summary.reasons?.tracking),
    ...countLines("주요 sync 차단", summary.reasons?.sync),
    ...countLines("새 앵커 보류 사유", summary.worldPose?.anchorCreationGateReasons),
    ...countLines("이벤트", summary.eventTypes, 10),
  ];
  const representative = [];
  const perType = new Map();
  for (const event of Array.isArray(payload.events) ? payload.events : []) {
    const count = perType.get(event.type) || 0;
    if (count >= 2 || representative.length >= 18) continue;
    perType.set(event.type, count + 1);
    representative.push(`- ${seconds(event.mediaTimeMs)} ${event.type}`);
  }
  if (representative.length) lines.push("대표 이상 시각:", ...representative);
  return lines.join("\n");
}

function safeFilePart(value) {
  return String(value || "replay").replace(/[^a-zA-Z0-9._-]+/g, "_").slice(0, 96);
}

function downloadJson(target, filename, payload) {
  const BlobType = target.Blob || globalThis.Blob;
  const urlApi = target.URL || globalThis.URL;
  const documentRoot = target.document;
  if (!BlobType || !urlApi?.createObjectURL || !documentRoot?.createElement) return false;
  const blob = new BlobType([JSON.stringify(payload, null, 2)], { type: "application/json" });
  const href = urlApi.createObjectURL(blob);
  const link = documentRoot.createElement("a");
  link.href = href;
  link.download = filename;
  link.hidden = true;
  (documentRoot.body || documentRoot.documentElement)?.appendChild?.(link);
  link.click?.();
  link.remove?.();
  target.setTimeout?.(() => urlApi.revokeObjectURL?.(href), 1000);
  return true;
}

export function createArReplayDiagnosticCapture(options = {}) {
  const target = options.target || globalThis;
  const interval = options.setInterval || target.setInterval?.bind(target);
  const clearIntervalFn = options.clearInterval || target.clearInterval?.bind(target);
  const save = options.download || ((filename, payload) => downloadJson(target, filename, payload));
  const now = options.now || (() => target.performance?.now?.() ?? Date.now());
  const listeners = new Set();
  let phase = "idle";
  let progress = 0;
  let error = "";
  let session = null;
  let unsubscribe = null;
  let monitor = null;
  let lastResult = null;
  let generation = 0;

  function snapshot() {
    return Object.freeze({
      phase,
      running: phase === "preparing" || phase === "running" || phase === "saving",
      progress,
      error: error || null,
      frames: session?.frames.length || 0,
      durationSeconds: session?.durationSeconds || AR_REPLAY_CAPTURE_SECONDS,
      lastResult,
    });
  }

  function emit() {
    const state = snapshot();
    for (const listener of [...listeners]) listener(state);
  }

  function setPhase(next, nextError = "") {
    phase = next;
    error = String(nextError || "");
    emit();
  }

  function clearResources() {
    unsubscribe?.();
    unsubscribe = null;
    if (monitor !== null) clearIntervalFn?.(monitor);
    monitor = null;
  }

  async function finish(reason = "complete", shouldDownload = true) {
    if (!session || session.finishing) return lastResult;
    session.finishing = true;
    setPhase("saving");
    clearResources();
    const current = session;
    const control = current.control;
    control.pause?.();
    const endedAt = new Date().toISOString();
    const summary = captureSummary(current);
    const finalReason = reason === "complete" && !summary.coverage.complete
      ? "incomplete-render-tail" : reason;
    const payloadBase = {
      schemaVersion: AR_REPLAY_CAPTURE_SCHEMA,
      kind: "carrot-ar-replay-diagnostic",
      createdAt: current.createdAt,
      endedAt,
      stopReason: finalReason,
      replay: {
        route: current.initial.route,
        segment: current.initial.segment,
        duration: current.initial.duration,
        requestedStart: 0,
        requestedEnd: current.durationSeconds,
        captureStart: finite(summary.coverage.firstMediaTimeMs, 0) / 1000,
        captureEnd: finite(summary.coverage.lastMediaTimeMs, 0) / 1000,
        playbackRate: 1,
      },
      environment: {
        userAgent: String(target.navigator?.userAgent || ""),
        devicePixelRatio: finite(target.devicePixelRatio, 1),
        viewport: {
          width: finite(target.innerWidth), height: finite(target.innerHeight),
        },
      },
      summary,
      routeGeometries: [...current.routeGeometries.values()],
      events: current.events,
      frames: current.frames,
    };
    const report = formatArReplayDiagnosticReport(payloadBase);
    const payload = { ...payloadBase, report };
    const filename = `carrot-ar-${safeFilePart(current.initial.segment)}-${
      endedAt.replace(/[:.]/g, "-")}.json`;
    let downloaded = false;
    if (shouldDownload) downloaded = save(filename, payload) !== false;
    await control.restore?.(current.initial);
    lastResult = Object.freeze({
      filename, downloaded, frames: current.frames.length, reason: finalReason,
      complete: summary.coverage.complete, summary, report,
    });
    session = null;
    progress = summary.coverage.ratio;
    setPhase(summary.coverage.complete ? "complete" : "warning");
    return lastResult;
  }

  function record(packet) {
    if (!session || session.finishing) return;
    // A frame submitted before the listener was attached may still be in
    // flight. It deliberately carries no full diagnostics and must not be
    // mistaken for the first 0-second replay frame.
    if (!packet?.debugFrame) return;
    const replayStatus = target.CarrotVisionReplay?.status?.() || {};
    const frame = compactArDiagnosticFrame(packet, replayStatus);
    const timeMs = finite(frame.mediaTimeMs);
    if (timeMs === null) return;
    if (!session.armed) {
      if (timeMs > 1500) return;
      session.armed = true;
    }
    if (timeMs < -1 || timeMs > session.durationSeconds * 1000 + 250) return;
    if (session.frames.length >= AR_REPLAY_CAPTURE_MAX_FRAMES) {
      finish("frame-limit").catch(() => {});
      return;
    }
    observeAnomalies(session, frame);
    session.frames.push(frame);
    const routeProbeKey = [
      frame.navi?.sessionId || "",
      frame.navi?.route?.pointCount || 0,
      frame.navi?.publishMonoTimeNanos || 0,
    ].join(":");
    if (routeProbeKey !== session.lastRouteProbeKey) {
      session.lastRouteProbeKey = routeProbeKey;
      const geometry = routeGeometry(target);
      if (geometry && !session.routeGeometries.has(geometry.signature)) {
        session.routeGeometries.set(geometry.signature, geometry);
      }
    }
    progress = Math.max(progress, Math.min(1, timeMs / (session.durationSeconds * 1000)));
    emit();
    if (timeMs >= session.durationSeconds * 1000 - 20) {
      finish("complete").catch(() => {});
    }
  }

  async function start(settings = {}) {
    if (snapshot().running) return false;
    generation += 1;
    const token = generation;
    error = "";
    progress = 0;
    const replay = target.CarrotVisionReplay;
    const control = replay?.diagnosticPlayback;
    const runtime = target.CarrotVisionAr;
    const initial = control?.snapshot?.();
    if (!initial?.active || !initial?.ready || initial.loading) {
      setPhase("error", "준비된 리플레이가 없습니다");
      return false;
    }
    if (typeof runtime?.subscribeDiagnostics !== "function") {
      setPhase("error", "AR 프레임 진단 채널이 없습니다");
      return false;
    }
    const requested = Math.max(1, finite(settings.durationSeconds, AR_REPLAY_CAPTURE_SECONDS));
    const durationSeconds = Math.min(requested, Math.max(0.1, finite(initial.duration, requested)));
    session = {
      token,
      control,
      initial,
      durationSeconds,
      createdAt: new Date().toISOString(),
      frames: [], events: [], routeGeometries: new Map(),
      lastRouteProbeKey: "", armed: false, finishing: false, playbackEndedAtMs: null,
      lastEventAt: new Map(),
    };
    setPhase("preparing");
    unsubscribe = runtime.subscribeDiagnostics(record);
    control.pause?.();
    control.setRate?.(1);
    if (control.seek?.(0) !== true) {
      clearResources();
      session = null;
      setPhase("error", "리플레이 0초 이동에 실패했습니다");
      return false;
    }
    monitor = interval?.(() => {
      if (!session || session.token !== token || session.finishing) return;
      const state = control.snapshot?.() || {};
      progress = Math.max(progress, Math.min(1, finite(state.currentTime, 0) / durationSeconds));
      emit();
      if (!state.active) finish("replay-closed").catch(() => {});
      else if (state.ended || finite(state.currentTime, 0) >= durationSeconds) {
        if (session.playbackEndedAtMs === null) {
          session.playbackEndedAtMs = now();
          control.pause?.();
        }
        const summary = captureSummary(session);
        if (summary.coverage.complete) finish("complete").catch(() => {});
        else if (now() - session.playbackEndedAtMs >= AR_REPLAY_CAPTURE_DRAIN_MS) {
          finish("incomplete-render-tail").catch(() => {});
        }
      }
    }, 100) ?? null;
    try {
      await control.play?.();
      if (!session || session.token !== token) return false;
      setPhase("running");
      return true;
    } catch (cause) {
      clearResources();
      await control.restore?.(initial);
      session = null;
      setPhase("error", cause?.message || "리플레이 재생에 실패했습니다");
      return false;
    }
  }

  async function stop() {
    if (!snapshot().running) return false;
    await finish("user-stopped");
    return true;
  }

  async function destroy() {
    generation += 1;
    if (session) await finish("debug-panel-closed", false);
    clearResources();
    listeners.clear();
  }

  return Object.freeze({
    start,
    stop,
    snapshot,
    subscribe(listener) {
      if (typeof listener !== "function") throw new TypeError("capture listener must be a function");
      listeners.add(listener);
      listener(snapshot());
      return () => listeners.delete(listener);
    },
    destroy,
  });
}
