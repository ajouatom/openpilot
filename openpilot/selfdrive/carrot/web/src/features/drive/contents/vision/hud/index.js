"use strict";

/* 신규 Carrot HUD 오버레이(클러스터 룩) 조립·마운트.
 * - 스테이지(#carrotStage) 위에 self-mount하는 SVG 오버레이.
 * - 공통 HUD payload를 직접 소비한다. 데이터 공급자가 라이브/리플레이를 구분하지 않는다.
 * - 위젯=순수 SVG 컴포넌트(widgets/*), 배치/degradation=layout.js, 룩=tokens/style.
 * 아직 구 HUD(#driveHudCard)는 제거하지 않음(비교/안정화 후 정리). */

import { injectStyle } from "./style.js";
import { applyHudDegradation, createHudLayoutObserver } from "./layout.js";
import { createSpeedPanel } from "./widgets/speed_panel.js";
import { createSpeedLimitSign } from "./widgets/speed_limit_sign.js";
import { createClock } from "./widgets/clock.js";
import { createWifiIcon } from "./widgets/wifi_icon.js";
import { createLfaIcon } from "./widgets/lfa_icon.js";
import { createAccelGauge } from "./widgets/accel_gauge.js";
import { createSteerGauge } from "./widgets/steer_gauge.js";
import { createTurnSignal } from "./widgets/turn_signal.js";
import { createLevelGauge } from "./widgets/level_gauge.js";
import { createTpmsBadge } from "./widgets/tpms_badge.js";
import { CarrotHudDataBridge } from "./data_bridge.js";
import { COLORS } from "./tokens.js";
import { el } from "./dom.js";

const installed = new WeakMap();
if (typeof globalThis === "object") globalThis.CarrotHudDataBridge = CarrotHudDataBridge;

function num(value) {
  if (value === null || value === undefined || value === "") return null;
  const n = Number(value);
  return Number.isFinite(n) ? n : null;
}

// 실 payload(js/realtime/vision_raw.js):
//  vEgoKph, vSetKph, tfGap, gear, gearStep, speedLimitKph, isMetric, driveMode...
// 속도류는 항상 kph로 옴 → isMetric=false면 mph 변환.
// Exported for tests: the single normalization every HUD payload passes through,
// shared identically by live (당근비전) and replay (both feed RAW_HUD_STATE →
// deriveCompactHudPayload → CarrotHudOverlay.update → mapPayload → widgets).
export function mapPayload(p = {}) {
  const metric = p.isMetric !== false;
  const toUnit = (kph) => (kph == null ? null : Math.round(metric ? kph : kph * 0.621371));
  const speed = num(p.vEgoKph ?? p.speed);
  const setSpeed = num(p.vSetKph ?? p.setSpeed);
  const gap = num(p.tfGap ?? p.gap);
  const limit = num(p.speedLimitKph ?? p.limit);
  const gearStep = num(p.gearStep);
  const gear = gearStep != null && gearStep > 0
    ? String(Math.round(gearStep))
    : (p.gear == null ? null : String(p.gear).trim().toUpperCase().slice(0, 2));
  const override = p.cruiseOverride && typeof p.cruiseOverride === "object" ? p.cruiseOverride : null;
  const overrideKph = override != null ? num(override.kph) : null;
  return {
    speed: toUnit(speed),
    // 크루즈 off면 0/음수로 오므로 숨김
    setSpeed: setSpeed != null && setSpeed > 0 ? toUnit(setSpeed) : null,
    gap: gap != null && gap > 0 ? Math.round(gap) : null,
    gear,
    speedLimit: limit != null && limit > 0 ? toUnit(limit) : null,
    // 클러스터 패리티: EV 텔테일 / LFA 레인 초록 날개 / 크루즈 오버라이드(감속=주황·eco=초록)
    evActive: p.evActive === true,
    activeLaneLine: p.activeLaneLine === true,
    cruiseOverride: overrideKph != null && overrideKph > 0
      ? { kph: toUnit(overrideKph), label: override.label == null ? "" : String(override.label), mode: num(override.mode) ?? 0 }
      : null,
    lfaActive: p.latActive ?? p.lfaActive,
    steerAngle: num(p.steeringAngleDeg),
    accel: num(p.aEgo ?? p.accel),
    steerOutput: num(p.steerOutput),
    leftBlinker: p.leftBlinker === true,
    rightBlinker: p.rightBlinker === true,
    fuelGauge: num(p.fuelGauge),
    ureaGauge: num(p.ureaGauge),
    tpms: p.tpms && typeof p.tpms === "object" ? p.tpms : null,
    trafficState: num(p.trafficState),
    networkConnected: p.networkConnected,
    clock: p.clock,
  };
}

export function createHudOverlay(doc) {
  injectStyle(doc);
  const root = el(doc, "div", { class: "chud", attrs: { "data-carrot-hud": "overlay" } });

  const lfa = createLfaIcon(doc);
  const wifi = createWifiIcon(doc);
  const clock = createClock(doc);
  const limit = createSpeedLimitSign(doc);
  const speed = createSpeedPanel(doc);
  const accel = createAccelGauge(doc);
  const steer = createSteerGauge(doc);
  const fuel = createLevelGauge(doc, {
    valueKey: "fuelGauge", label: "fuel", ariaLabel: "Fuel or battery level",
    hideWhenUnavailable: true,
  });
  const def = createLevelGauge(doc, {
    valueKey: "ureaGauge", label: "DEF", normalColor: COLORS.blue,
    lowColor: COLORS.amber, ariaLabel: "Diesel exhaust fluid level",
    hideWhenUnavailable: true,
  });
  const tpms = createTpmsBadge(doc);
  const turn = createTurnSignal(doc);

  const zoneTL = el(doc, "div", { class: "chud-zone chud-zone--tl" }, [
    el(doc, "div", { class: "chud-row" }, [lfa.el, wifi.el, clock.el]),
    limit.el,
  ]);
  const zoneTC = el(doc, "div", { class: "chud-zone chud-zone--tc" }, [turn.el]);
  const zoneTR = el(doc, "div", { class: "chud-zone chud-zone--tr" }, [
    el(doc, "div", { class: "chud-gauge-column" }, [accel.el, fuel.el]),
    el(doc, "div", { class: "chud-gauge-column" }, [steer.el, def.el]),
  ]);
  const zoneBL = el(doc, "div", { class: "chud-zone chud-zone--bl" }, [speed.el]);
  const zoneBR = el(doc, "div", { class: "chud-zone chud-zone--br" }, [tpms.el]);
  root.append(zoneTL, zoneTC, zoneTR, zoneBL, zoneBR);
  const layoutZones = {
    topLeft: zoneTL,
    topCenter: turn.el,
    topRight: zoneTR,
    bottomLeft: zoneBL,
    bottomRight: zoneBR,
  };

  const widgets = [lfa, wifi, clock, limit, speed, accel, steer, fuel, def, tpms, turn];
  let visibilitySignature = "";
  let layoutObserver = null;

  function applyLayout() {
    applyHudDegradation(root, layoutZones);
  }

  function scheduleLayout() {
    if (layoutObserver) layoutObserver.schedule();
    else applyLayout();
  }

  function update(payload) {
    const data = mapPayload(payload);
    for (const w of widgets) w.update(data);
    const nextVisibilitySignature = [
      data.speedLimit != null && data.speedLimit > 0,
      data.leftBlinker,
      data.rightBlinker,
      // 레인 날개는 LFA 폭을 바꾸므로 등장/소멸 시 재배치 판정이 필요하다.
      data.activeLaneLine,
      data.fuelGauge != null && data.fuelGauge > 0 && data.fuelGauge <= 1,
      data.ureaGauge != null && data.ureaGauge > 0 && data.ureaGauge <= 1,
    ].join(":");
    if (visibilitySignature !== nextVisibilitySignature) {
      visibilitySignature = nextVisibilitySignature;
      scheduleLayout();
    }
  }

  function relayout(viewport) {
    const width = num(viewport?.width);
    const height = num(viewport?.height);
    if (width > 0 && height > 0) scheduleLayout();
  }

  function startLayout(target) {
    if (!layoutObserver) layoutObserver = createHudLayoutObserver(root, applyLayout, target);
    layoutObserver?.schedule();
  }

  return {
    root,
    update,
    relayout,
    startClock: () => clock.start(),
    stopClock: () => clock.stop(),
    startLayout,
  };
}

export function installCarrotHudOverlay(target = globalThis, options = {}) {
  if (installed.has(target)) return installed.get(target);
  target.CarrotHudDataBridge = CarrotHudDataBridge;
  const doc = options.document || target.document;
  if (!doc) return null;
  const stage = options.stage || doc.getElementById?.("carrotStage");
  if (!stage) {
    // 스테이지 DOM 아직 없음 → 준비되면 1회 재시도
    if (!target.__carrotHudDeferred) {
      target.__carrotHudDeferred = true;
      const retry = () => { target.__carrotHudDeferred = false; installCarrotHudOverlay(target, options); };
      if (doc.readyState === "loading") doc.addEventListener("DOMContentLoaded", retry, { once: true });
      else target.setTimeout?.(retry, 0);
    }
    return null;
  }

  const overlay = createHudOverlay(doc);
  stage.appendChild(overlay.root);
  overlay.startLayout(target);
  overlay.startClock();
  overlay.update({});

  // 좌표는 DriveVisionViewport가 단독 소유한다. HUD는 공통 결과만 소비한다.
  target.addEventListener?.("carrot:viewportlayout", (event) => overlay.relayout(event.detail), { passive: true });

  target.CarrotHudOverlay = overlay;
  installed.set(target, overlay);
  return overlay;
}
