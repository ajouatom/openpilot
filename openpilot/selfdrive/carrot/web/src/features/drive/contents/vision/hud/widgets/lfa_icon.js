"use strict";

import { el } from "../dom.js";
import { CARROT_URI } from "../assets.js";

/* LFA 아이콘 = 클러스터와 동일한 원본 에셋. 클러스터 동작:
 *  - 활성 시 풀컬러 / 비활성 시 회색(muted)
 *  - 조향각만큼 회전(cluster: rotation = -steering_angle_deg) */
export function createLfaIcon(doc) {
  const root = el(doc, "span", { class: "chud-lfa", attrs: { role: "img", "aria-label": "LFA" } });
  const img = el(doc, "img", { class: "chud-lfa-img", attrs: { alt: "LFA", src: CARROT_URI } });
  root.appendChild(img);

  function update(data = {}) {
    root.classList.toggle("is-active", data.lfaActive !== false);
    const angle = Number(data.steerAngle);
    if (Number.isFinite(angle)) img.style.transform = `rotate(${(-angle).toFixed(1)}deg)`;
  }

  return { el: root, update };
}
