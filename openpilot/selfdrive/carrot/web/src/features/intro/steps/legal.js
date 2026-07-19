"use strict";

/* 5단계 — 법적 고지
 *
 * 마지막 인사(step6_outro) 바로 앞. 복원 경로를 포함해 모든 사용자가 거친다 —
 * 건너뛸 수 있으면 고지가 아니다.
 *
 * 다른 단계는 "선택 = 진행" 이지만 여기는 고를 게 없다.
 * outro 와 같은 방식으로 진행 액션(.intro-cta) 하나를 둔다.
 *
 * 본문만 좌측 정렬(.intro-lead--legal)이다. 나머지 단계는 가운데 정렬인데,
 * 이건 훑는 글이 아니라 읽는 글이라 가운데 정렬이면 줄 시작점이 흔들려
 * 읽기 나쁘다. 의도적 예외이며 크기·색·간격은 그대로 토큰을 따른다. */

CarrotIntro.register({
  id: "legal",
  file: "step5_legal.js",
  flow: true,

  render(el) {
    const { t } = CarrotIntro;
    const paras = t("legalBody");

    el.innerHTML = `
      ${CarrotIntroIcons.legal}
      <h2 class="intro-title">${t("legalTitle")}</h2>
      <div class="intro-legal">
        ${paras.map((p) => `<p class="intro-lead intro-lead--legal">${p}</p>`).join("")}
      </div>
      <button class="intro-cta" type="button" data-ack>${t("legalAck")}</button>`;

    el.querySelector("[data-ack]").addEventListener("click", () => {
      CarrotIntroShell.goTo("outro");
    });
  },
});
