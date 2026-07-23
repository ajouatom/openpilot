export const ROUTE_SUMMARY_POLICY = Object.freeze({
  schemaVersion: 1,
  policyVersion: 2,
  overAccel: 1.5,
  hardAccel: 2.5,
  overDecel: -2.0,
  hardDecel: -3.0,
  hardLatAccel: 3.0,
  cornerMinSpeed: 5.5,
  defaultSteerRatio: 14.0,
  defaultWheelbase: 2.8,
  eventMinGapSec: 1.5,
  dtCapSec: 0.5,
  movingSpeed: 0.3,
  maxEventsPerCategory: 60,
});

const DRIVING_GEARS = new Set(["drive", "low", "reverse", "sport", "eco", "manumatic"]);
const WARNING_EVENT_NAMES = Object.freeze({
  fcw: new Set([59, 65]),
  ldw: new Set([53]),
  driverDistracted: new Set([33, 34, 35, 36, 37, 38, 45]),
});

function finite(value, fallback = 0) {
  const number = Number(value);
  return Number.isFinite(number) ? number : fallback;
}

function eventTime(monoTime, wallTime) {
  return {
    monoSec: finite(monoTime) / 1e9,
    wallTimeSec: wallTime > 0 ? wallTime : 0,
  };
}

class Excursions {
  constructor(accel, overThreshold, hardThreshold, policy) {
    this.accel = accel;
    this.overThreshold = overThreshold;
    this.hardThreshold = hardThreshold;
    this.policy = policy;
    this.raw = [];
    this.current = null;
  }

  inBand(value) {
    return this.accel ? value >= this.overThreshold : value <= this.overThreshold;
  }

  peak(left, right) {
    return this.accel ? Math.max(left, right) : Math.min(left, right);
  }

  feed(monoTime, wallTime, value) {
    const time = finite(monoTime) / 1e9;
    if (this.inBand(value)) {
      if (!this.current) {
        this.current = { start: time, end: time, peak: value, wallTimeSec: wallTime || 0 };
      } else {
        this.current.end = time;
        this.current.peak = this.peak(this.current.peak, value);
      }
    } else if (this.current) {
      this.raw.push(this.current);
      this.current = null;
    }
  }

  finish() {
    if (this.current) {
      this.raw.push(this.current);
      this.current = null;
    }
    const merged = [];
    for (const excursion of this.raw) {
      const previous = merged[merged.length - 1];
      if (previous && excursion.start - previous.end < this.policy.eventMinGapSec) {
        previous.end = excursion.end;
        previous.peak = this.peak(previous.peak, excursion.peak);
      } else {
        merged.push({ ...excursion });
      }
    }
    const hard = [];
    const over = [];
    for (const excursion of merged) {
      const item = {
        monoSec: excursion.start,
        wallTimeSec: excursion.wallTimeSec,
        peak: Math.round(excursion.peak * 100) / 100,
      };
      const isHard = this.accel
        ? excursion.peak >= this.hardThreshold
        : excursion.peak <= this.hardThreshold;
      (isHard ? hard : over).push(item);
    }
    return { hard, over };
  }
}

function capped(items, policy) {
  return items.slice(0, policy.maxEventsPerCategory);
}

export function createRouteSummaryAccumulator(options = {}) {
  const policy = { ...ROUTE_SUMMARY_POLICY, ...(options.policy || {}) };
  const accelExcursions = new Excursions(true, policy.overAccel, policy.hardAccel, policy);
  const decelExcursions = new Excursions(false, policy.overDecel, policy.hardDecel, policy);
  const totals = {
    autoEnabledSec: 0,
    autoActiveSec: 0,
    manualSec: 0,
    manualGasSec: 0,
    manualBrakeSec: 0,
    stopSec: 0,
    steerOverrideSec: 0,
    totalDistanceM: 0,
    autoDistanceM: 0,
    maxSpeedMs: 0,
    maxLatAccel: 0,
  };
  const counts = {
    stop: 0,
    disengage: 0,
    steerOverride: 0,
    corner: 0,
  };
  const disengageCauses = { brake: 0, gas: 0, button: 0, steer: 0, other: 0 };
  const disengageItems = [];
  const warningCounts = { fcw: 0, ldw: 0, driverDistracted: 0 };
  const warningPrevious = { fcw: false, ldw: false, driverDistracted: false };
  let steerRatio = policy.defaultSteerRatio;
  let wheelbase = policy.defaultWheelbase;
  let previousEnabled = false;
  let previousStandstill = false;
  let previousSteerOverride = false;
  let previousCorner = false;
  let lastCancelSec = Number.NEGATIVE_INFINITY;
  let firstWallSec = 0;
  let lastWallSec = 0;
  let firstMonoSec = 0;
  let lastMonoSec = 0;
  let usedLog = false;
  let segmentState = null;

  function startSegment() {
    segmentState = {
      anchorMono: 0,
      anchorWall: 0,
      previousCarStateMono: null,
      lastCarState: null,
      enabled: previousEnabled,
      active: false,
    };
  }

  function wallTimeFor(monoTime) {
    if (!segmentState?.anchorWall) return 0;
    return segmentState.anchorWall + (monoTime - segmentState.anchorMono) / 1e9;
  }

  function disengageCause(carState, nowSec) {
    if (nowSec - lastCancelSec <= 1) return "button";
    if (carState?.brakePressed) return "brake";
    if (carState?.steeringPressed) return "steer";
    if (carState?.gasPressed) return "gas";
    return "other";
  }

  function ingestWarningList(events) {
    const names = new Set((Array.isArray(events) ? events : []).map((item) => finite(item?.name, -1)));
    for (const [category, configuredNames] of Object.entries(WARNING_EVENT_NAMES)) {
      let present = false;
      for (const name of configuredNames) {
        if (names.has(name)) {
          present = true;
          break;
        }
      }
      if (present && !warningPrevious[category]) warningCounts[category] += 1;
      warningPrevious[category] = present;
    }
  }

  function ingest(event) {
    if (!segmentState) startSegment();
    const service = String(event?.service || "");
    const decoded = event?.decoded;
    const monoTime = finite(event?.logMonoTime);
    if (!service || !decoded || monoTime <= 0) return;
    if (!firstMonoSec) firstMonoSec = monoTime / 1e9;
    lastMonoSec = Math.max(lastMonoSec, monoTime / 1e9);

    if (service === "initData" || service === "clocks") {
      const wallNanos = finite(decoded.wallTimeNanos);
      if (wallNanos > 0) {
        segmentState.anchorMono = monoTime;
        segmentState.anchorWall = wallNanos / 1e9;
      }
      return;
    }
    if (service === "carParams") {
      if (finite(decoded.steerRatio) > 0) steerRatio = finite(decoded.steerRatio);
      if (finite(decoded.wheelbase) > 0) wheelbase = finite(decoded.wheelbase);
      return;
    }
    if (service === "onroadEvents") {
      ingestWarningList(decoded);
      return;
    }
    const wallTime = wallTimeFor(monoTime);
    const monotonicSec = monoTime / 1e9;
    const logicalTime = wallTime || monotonicSec;

    if (service === "selfdriveState") {
      const enabled = Boolean(decoded.enabled);
      const active = Boolean(decoded.active);
      if (previousEnabled && !enabled) {
        counts.disengage += 1;
        const cause = disengageCause(segmentState.lastCarState, logicalTime);
        disengageCauses[cause] += 1;
        if (disengageItems.length < policy.maxEventsPerCategory) {
          disengageItems.push({ ...eventTime(monoTime, wallTime), cause });
        }
      }
      previousEnabled = enabled;
      segmentState.enabled = enabled;
      segmentState.active = active;
      return;
    }
    if (service !== "carState") return;

    const carState = decoded;
    segmentState.lastCarState = carState;
    for (const button of Array.isArray(carState.buttonEvents) ? carState.buttonEvents : []) {
      if (button?.type === "cancel" && button?.pressed) lastCancelSec = logicalTime;
    }
    if (wallTime > 0) {
      if (!firstWallSec) firstWallSec = wallTime;
      lastWallSec = Math.max(lastWallSec, wallTime);
    }

    let dt = 0;
    if (segmentState.previousCarStateMono != null) {
      dt = Math.min(policy.dtCapSec, Math.max(0, (monoTime - segmentState.previousCarStateMono) / 1e9));
    }
    segmentState.previousCarStateMono = monoTime;

    const speed = Math.max(0, finite(carState.vEgo));
    const acceleration = finite(carState.aEgo);
    const gear = String(carState.gearShifter || "").toLowerCase();
    const driving = DRIVING_GEARS.has(gear) || speed > policy.movingSpeed;
    totals.maxSpeedMs = Math.max(totals.maxSpeedMs, speed);
    accelExcursions.feed(monoTime, wallTime, acceleration);
    decelExcursions.feed(monoTime, wallTime, acceleration);

    let lateralAcceleration = 0;
    let cornering = false;
    if (speed >= policy.cornerMinSpeed) {
      const yawRate = finite(carState.yawRate);
      if (Math.abs(yawRate) > 1e-3) {
        lateralAcceleration = Math.abs(speed * yawRate);
      } else {
        const wheelRadians = (finite(carState.steeringAngleDeg) / steerRatio) * Math.PI / 180;
        lateralAcceleration = Math.abs(speed * speed * Math.tan(wheelRadians) / wheelbase);
      }
      totals.maxLatAccel = Math.max(totals.maxLatAccel, lateralAcceleration);
      cornering = lateralAcceleration >= policy.hardLatAccel;
    }
    if (cornering && !previousCorner) counts.corner += 1;
    previousCorner = cornering;

    if (carState.standstill && !previousStandstill) counts.stop += 1;
    previousStandstill = Boolean(carState.standstill);
    if (carState.standstill) totals.stopSec += dt;

    usedLog = true;
    if (segmentState.enabled) {
      totals.autoEnabledSec += dt;
      totals.autoDistanceM += speed * dt;
      if (segmentState.active) totals.autoActiveSec += dt;
      if (carState.steeringPressed && !previousSteerOverride) counts.steerOverride += 1;
      if (carState.steeringPressed) totals.steerOverrideSec += dt;
      previousSteerOverride = Boolean(carState.steeringPressed);
    } else {
      previousSteerOverride = false;
      if (driving) {
        totals.manualSec += dt;
        if (carState.gasPressed) totals.manualGasSec += dt;
        if (carState.brakePressed) totals.manualBrakeSec += dt;
      }
    }
    if (driving) totals.totalDistanceM += speed * dt;
  }

  function finish(diagnostics = {}) {
    const acceleration = accelExcursions.finish();
    const deceleration = decelExcursions.finish();
    const totalSec = totals.autoEnabledSec + totals.manualSec;
    return {
      ok: true,
      schemaVersion: policy.schemaVersion,
      policyVersion: policy.policyVersion,
      route: String(options.route || ""),
      source: String(options.source || ""),
      hasData: usedLog,
      time: {
        totalSec,
        autoEnabledSec: totals.autoEnabledSec,
        autoActiveSec: totals.autoActiveSec,
        manualSec: totals.manualSec,
        manualGasSec: totals.manualGasSec,
        manualBrakeSec: totals.manualBrakeSec,
        stopSec: totals.stopSec,
        steerOverrideSec: totals.steerOverrideSec,
        startWallSec: firstWallSec,
        endWallSec: lastWallSec,
        firstMonoSec,
        lastMonoSec,
      },
      distance: {
        totalM: totals.totalDistanceM,
        autoM: totals.autoDistanceM,
        manualM: Math.max(0, totals.totalDistanceM - totals.autoDistanceM),
        averageSpeedMs: totalSec > 0 ? totals.totalDistanceM / totalSec : 0,
        maxSpeedMs: totals.maxSpeedMs,
      },
      events: {
        hardAccel: { count: acceleration.hard.length, items: capped(acceleration.hard, policy) },
        overAccel: { count: acceleration.over.length, items: capped(acceleration.over, policy) },
        hardDecel: { count: deceleration.hard.length, items: capped(deceleration.hard, policy) },
        overDecel: { count: deceleration.over.length, items: capped(deceleration.over, policy) },
      },
      extras: {
        disengageCount: counts.disengage,
        disengageCauses,
        disengageItems,
        stopCount: counts.stop,
        steerOverrideCount: counts.steerOverride,
        cornerCount: counts.corner,
        maxLatAccel: totals.maxLatAccel,
        warningCounts,
      },
      diagnostics: { ...diagnostics },
    };
  }

  return Object.freeze({ startSegment, ingest, finish });
}
