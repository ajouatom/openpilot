"use strict";

/* 위젯 내부는 클러스터 고정 좌표와 원본 자산을 사용한다.
 * 이 파일은 위젯 간 공통 색과 웹 반응형 배치 하한만 소유한다. */

export const COLORS = Object.freeze({
  white: "#ffffff",
  ink: "#0d1116",
  muted: "#96a0ac",    // DARK_CLUSTER_THEME.muted (150,160,172)
  carrot: "#14bc68",   // GREEN (20,188,104)
  blue: "#2684ff",     // BLUE (38,132,255)
  amber: "#f4ac36",    // AMBER (244,172,54)
  override: "#b87018", // CRUISE_OVERRIDE_APPLY_COLOR (184,112,24) — 감속 오버라이드 주황
  limit: "#de4840",    // RED (222,72,64)
  stroke: "#05090c",   // 텍스트 외곽선
});

// 주행모드 배지 색 — 클러스터 SPEED_DRIVING_MODE_STYLES 그대로(배경 α는 style.js).
// 1 연비(0,255,0) / 2 안전(255,165,0) / 3 일반(255,255,255) / 4 고속(255,0,0).
export const DRIVE_MODE_COLORS = Object.freeze({
  1: "#00ff00",
  2: "#ffa500",
  3: "#ffffff",
  4: "#ff0000",
});

// 가독성 하한(px): 이 값 아래로는 축소하지 않고 degradation에서 숨긴다.
export const MIN_PX = Object.freeze({
  speedPanel: 150,
  topIcon: 24,
  clock: 18,
  limitSign: 46,
});

// 우선순위(작을수록 오래 생존). layout.js의 degradation이 사용.
export const PRIORITY = Object.freeze({
  speedPanel: 0,
  limitSign: 1,
  clock: 2,
  lfa: 3,
  wifi: 3,
  gauges: 4,
  tpms: 5,
});
