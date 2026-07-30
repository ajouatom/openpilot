"use strict";

import { el } from "../dom.js";
import { CARROT_URI, LFA_LANE_URI } from "../assets.js";
import { LANE_MODE_PRESENTATION } from "../../lane_mode.js";

/* LFA 아이콘 = 클러스터와 동일한 원본 에셋. 클러스터 동작:
 *  - 활성 시 풀컬러 / 비활성 시 회색(muted)
 *  - 조향각만큼 회전(cluster: rotation = -steering_angle_deg)
 *  - 사용자 레인모드 선택/자동 폴백/최종 활성 상태를 고정 폭 날개로 구분.
 *    클러스터는 흰 실루엣 텍스처를 GREEN(활성)/muted(비활성)로 틴트 → 웹은 동일 자산을
 *    mask-image로 깔고 배경색만 토큰(--chud-carrot/--chud-muted)으로 칠한다(픽셀루프/새 에셋 없음). */
export function createLfaIcon(doc) {
  const root = el(doc, "span", { class: "chud-lfa", attrs: { role: "img", "aria-label": "LFA" } });
  const lane = el(doc, "span", { class: "chud-lfa-lane", attrs: { "aria-hidden": "true" } });
  const img = el(doc, "img", { class: "chud-lfa-img", attrs: { alt: "LFA", src: CARROT_URI } });
  // 마스크 URL은 자산 상수 단일출처에서 주입(CSS 하드코딩 금지). 색/크기/위치는 CSS 소유.
  lane.style.webkitMaskImage = `url("${LFA_LANE_URI}")`;
  lane.style.maskImage = `url("${LFA_LANE_URI}")`;
  root.append(lane, img);

  function update(data = {}) {
    root.classList.toggle("is-active", data.lfaActive !== false);
    const presentation = data.laneModePresentation || (
      data.activeLaneLine === true
        ? LANE_MODE_PRESENTATION.ACTIVE
        : LANE_MODE_PRESENTATION.LANELESS
    );
    const hasLane = (
      presentation === LANE_MODE_PRESENTATION.ARMED
      || presentation === LANE_MODE_PRESENTATION.ACTIVE
    );
    root.classList.toggle("has-lane", hasLane);
    root.classList.toggle("is-lane-armed", presentation === LANE_MODE_PRESENTATION.ARMED);
    root.classList.toggle("is-lane-active", presentation === LANE_MODE_PRESENTATION.ACTIVE);
    root.setAttribute("data-lane-state", presentation);
    const angle = Number(data.steerAngle);
    if (Number.isFinite(angle)) img.style.transform = `rotate(${(-angle).toFixed(1)}deg)`;
  }

  return { el: root, update };
}
