"use strict";

import { svg, setText } from "../dom.js";
import { DRIVE_MODE_COLORS } from "../tokens.js";

/* 주행모드 배지(연비/안전/일반/고속). 클러스터 _draw_driving_mode_indicator 그대로 이식.
 * - 색/모양: 솔리드 모드색(배경 α 200/255) 둥근사각형 + 흰 테두리 + 흰 글자(어두운 외곽선). 색=토큰.
 * - 로케일: 라벨 길이에 맞춰 박스 폭을 동적 조정(한글 "안전" 2자 ↔ 영문 "Normal" 6자 모두 정합).
 * - 위치: 신호등 오른쪽에 넉넉한 간격으로 붙되, 좌표계는 패널(클러스터) 1:1. */

const SPEED_VALUE_CENTER_X = 85.2;                 // = speed_panel SPEED_X
const TRAFFIC_CENTER_X = SPEED_VALUE_CENTER_X - 38; // 신호등 중심(=speed_panel TRAFFIC_CENTER_X)
const TRAFFIC_RIGHT = TRAFFIC_CENTER_X + 34 * 0.5;  // 신호등 우측 끝(아이콘 34)
const GAP = 16;                                     // 좌측 HUD와의 간격(클러스터 5 → web에서 넉넉히)
const X_LEFT = TRAFFIC_RIGHT + GAP;                 // 배지 왼쪽 시작(고정), 폭은 우측으로 성장
const PAD_X = 10;                                   // 텍스트 좌우 여백
const MIN_W = 46;                                   // 최소 폭(2자 한글도 답답하지 않게)
const Y = 271;                                      // 클러스터 SPEED_DRIVING_MODE_Y
const H = 30;                                       // SPEED_DRIVING_MODE_H
const R = 8;                                        // 둥근 반경
const FS = 21;                                      // SPEED_DRIVING_MODE_FONT_SIZE
const CY = Y + H / 2;
const CAP = 0.35;                                   // 시각중심 보정(패널 다른 텍스트와 동일)

// 모드 번호 → kind(전역 driveModes 레지스트리 키). mini_hud/구 HUD와 동일 소스를 쓴다.
const MODE_KIND = Object.freeze({ 1: "eco", 2: "safe", 3: "normal", 4: "sport" });

// 레지스트리 미로드 시 폴백(값은 translations/*.js 의 driveModes 와 동일: en/ko/zh).
const FALLBACK_LABELS = Object.freeze({
  eco: { ko: "연비", en: "Eco", zh: "经济" },
  safe: { ko: "안전", en: "Safe", zh: "安全" },
  normal: { ko: "일반", en: "Normal", zh: "标准" },
  sport: { ko: "고속", en: "Sport", zh: "运动" },
});

const CJK_RE = /[ᄀ-ᇿ㄰-㆏가-힯　-ヿ㐀-鿿＀-￯]/;

// 신뢰 언어 소스 = <html lang>(i18n.js 가 활성 코드를 여기에 기록). window.LANG 은
// 모듈전용 전역이라 오버레이 컨텍스트에서 안 잡힘 → 절대 쓰지 말 것.
function currentLang(doc) {
  const raw = String(doc?.documentElement?.lang || "en").toLowerCase();
  return raw.split(/[-_]/)[0] || "en";
}

// 라벨 = 전역 driveModes 레지스트리(mini_hud/big HUD와 동일) → 레지스트리 없으면 폴백맵.
function resolveLabel(mode, doc, view) {
  const kind = MODE_KIND[mode];
  const lang = currentLang(doc);
  const table = view?.CarrotTranslations?.driveModes || {};
  const labels = table[lang] || table.en || {};
  const fallback = FALLBACK_LABELS[kind] || {};
  return labels[kind] || fallback[lang] || fallback.en || "";
}

// 정확 측정(getComputedTextLength)이 렌더 전이면 0 → CJK 1.0em / 라틴 0.56em 추정으로 폴백.
function estimateWidth(text) {
  let units = 0;
  for (const ch of text) units += CJK_RE.test(ch) ? 1.0 : 0.56;
  return units * FS;
}

export function createDriveModeBadge(doc) {
  const view = doc.defaultView || (typeof globalThis === "object" ? globalThis : null);
  const box = svg(doc, "rect", {
    class: "chud-drive-mode-box", x: X_LEFT, y: Y, width: MIN_W, height: H, rx: R,
  });
  const label = svg(doc, "text", {
    class: "chud-drive-mode-label", x: X_LEFT + MIN_W / 2, y: CY + CAP * FS,
    "font-size": FS, "text-anchor": "middle",
  });
  const root = svg(doc, "g", { class: "chud-drive-mode", role: "img" }, [box, label]);

  let lastMode = null;
  let lastText = "";
  let needsMeasure = false;

  function applyWidth(width) {
    const w = Math.max(MIN_W, Math.round(width + PAD_X * 2));
    box.setAttribute("width", w);
    label.setAttribute("x", X_LEFT + w / 2);
  }

  function update(data = {}) {
    const mode = Number(data.drivingMode);
    const known = mode === 1 || mode === 2 || mode === 3 || mode === 4;
    root.style.display = known ? "" : "none";
    if (!known) { lastMode = null; return; }

    if (mode !== lastMode) {
      lastMode = mode;
      box.style.fill = DRIVE_MODE_COLORS[mode];
    }

    const text = resolveLabel(mode, doc, view);
    if (text !== lastText) {
      lastText = text;
      setText(label, text);
      root.setAttribute("aria-label", text);
      applyWidth(estimateWidth(text)); // 즉시 추정폭(미측정이어도 안 뭉개짐)
      needsMeasure = true;
    }
    // 렌더된 뒤 정확폭으로 한 번 보정(로케일 폭 정밀 정합).
    if (needsMeasure && typeof label.getComputedTextLength === "function") {
      let exact = 0;
      try { exact = label.getComputedTextLength(); } catch {}
      if (exact > 0) { applyWidth(exact); needsMeasure = false; }
    }
  }

  update();
  return { el: root, update };
}
