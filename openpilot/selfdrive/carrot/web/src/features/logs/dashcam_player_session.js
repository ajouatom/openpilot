"use strict";

export const DASHCAM_SEGMENT_READ_STATE = Object.freeze({
  READING: "reading",
  RECENT: "recent",
});

function normalizeSegment(value) {
  return String(value || "").trim();
}

function normalizeSegments(values) {
  const seen = new Set();
  const result = [];
  for (const value of Array.isArray(values) ? values : []) {
    const segment = normalizeSegment(value);
    if (!segment || seen.has(segment)) continue;
    seen.add(segment);
    result.push(segment);
  }
  return result;
}

function normalizeGroup(value, loadedSegmentCount) {
  const source = value && typeof value === "object" ? value : {};
  const rawCount = Number(source.segmentCount);
  const segmentCount = Number.isFinite(rawCount)
    ? Math.max(0, Math.trunc(rawCount))
    : loadedSegmentCount;
  return Object.freeze({
    route: String(source.route || ""),
    dateLabel: String(source.dateLabel || ""),
    timeRange: String(source.timeRange || ""),
    segmentCount,
  });
}

export function dashcamSegmentReadState(state, segment) {
  const target = normalizeSegment(segment);
  if (!target) return "";
  if (target === state?.activeSegment) return DASHCAM_SEGMENT_READ_STATE.READING;
  if (target === state?.previousSegment) return DASHCAM_SEGMENT_READ_STATE.RECENT;
  return "";
}

export function createDashcamReadStateStore(initialState = {}) {
  let activeSegment = normalizeSegment(initialState.activeSegment);
  let previousSegment = normalizeSegment(initialState.previousSegment);
  const listeners = new Set();
  if (previousSegment === activeSegment) previousSegment = "";

  const snapshot = () => Object.freeze({ activeSegment, previousSegment });
  const result = (changed) => Object.freeze({ changed, state: snapshot() });
  const notify = () => {
    const state = snapshot();
    listeners.forEach((listener) => listener(state));
  };
  const select = (segment) => {
    const target = normalizeSegment(segment);
    if (!target || target === activeSegment) return result(false);
    if (activeSegment) previousSegment = activeSegment;
    else if (previousSegment === target) previousSegment = "";
    activeSegment = target;
    notify();
    return result(true);
  };
  const restoreRecent = (segment) => {
    const target = normalizeSegment(segment);
    if (!target || target === activeSegment || target === previousSegment) return result(false);
    previousSegment = target;
    notify();
    return result(true);
  };
  const finish = () => {
    if (!activeSegment) return result(false);
    previousSegment = activeSegment;
    activeSegment = "";
    notify();
    return result(true);
  };
  const statusFor = (segment) => dashcamSegmentReadState(
    { activeSegment, previousSegment },
    segment,
  );
  const subscribe = (listener) => {
    if (typeof listener !== "function") return () => {};
    listeners.add(listener);
    return () => listeners.delete(listener);
  };

  return Object.freeze({
    finish,
    restoreRecent,
    select,
    snapshot,
    statusFor,
    subscribe,
  });
}

export const dashcamReadStateStore = createDashcamReadStateStore();

export function createDashcamPlayerSession(options = {}) {
  let segments = normalizeSegments(options.segments);
  const group = normalizeGroup(options.group, segments.length);
  let activeSegment = segments.includes(normalizeSegment(options.activeSegment))
    ? normalizeSegment(options.activeSegment)
    : (segments[0] || "");
  let previousSegment = segments.includes(normalizeSegment(options.previousSegment))
    ? normalizeSegment(options.previousSegment)
    : "";
  if (previousSegment === activeSegment) previousSegment = "";

  const snapshot = () => {
    const activeIndex = segments.indexOf(activeSegment);
    return Object.freeze({
      group,
      segments: Object.freeze([...segments]),
      activeSegment,
      previousSegment,
      activeIndex,
      previousIndex: segments.indexOf(previousSegment),
      canMovePrevious: activeIndex > 0,
      canMoveNext: activeIndex >= 0 && activeIndex < segments.length - 1,
    });
  };

  const result = (changed) => Object.freeze({ changed, state: snapshot() });

  const select = (segment) => {
    const target = normalizeSegment(segment);
    if (!segments.includes(target) || target === activeSegment) return result(false);
    previousSegment = activeSegment;
    activeSegment = target;
    return result(true);
  };

  const move = (offset) => {
    const direction = Math.sign(Number(offset) || 0);
    const currentIndex = segments.indexOf(activeSegment);
    if (!direction || currentIndex < 0) return result(false);
    const targetIndex = currentIndex + direction;
    if (targetIndex < 0 || targetIndex >= segments.length) return result(false);
    return select(segments[targetIndex]);
  };

  const replaceSegments = (values, replacements = {}) => {
    const nextSegments = normalizeSegments(values);
    const requestedActive = normalizeSegment(replacements.activeSegment ?? activeSegment);
    const requestedPrevious = normalizeSegment(replacements.previousSegment ?? previousSegment);
    const nextActive = nextSegments.includes(requestedActive) ? requestedActive : (nextSegments[0] || "");
    const nextPrevious = nextSegments.includes(requestedPrevious) && requestedPrevious !== nextActive
      ? requestedPrevious
      : "";
    const changed = nextSegments.length !== segments.length
      || nextSegments.some((segment, index) => segment !== segments[index])
      || nextActive !== activeSegment
      || nextPrevious !== previousSegment;
    segments = nextSegments;
    activeSegment = nextActive;
    previousSegment = nextPrevious;
    return result(changed);
  };

  const statusFor = (segment) => dashcamSegmentReadState({ activeSegment, previousSegment }, segment);

  return Object.freeze({
    move,
    replaceSegments,
    select,
    snapshot,
    statusFor,
  });
}
