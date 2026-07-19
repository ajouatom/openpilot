"use strict";

/* 3단계 — HDA1 / HDA2  (현대·기아·제네시스 전용)
 *
 * CanfdHDA2: 0 = HDA1, 1 = HDA2   (params_keys.h:294)
 * HKG 가 아니면 이 단계 자체를 건너뛰고 param 도 건드리지 않는다.
 *
 * 이 값은 CAN 버스 구성을 바꾼다:
 *   opendbc/car/hyundai/interface.py:42  hda2 = params.get_int("CanfdHDA2") > 0
 *   opendbc/car/hyundai/interface.py:43  CAN = CanBus(None, fingerprint, hda2)
 *
 * ⚠ HDA2 는 CAN-FD 차량 개념인데, 차량이 CAN-FD 인지 판별할 수단이 웹에 없다
 *   (/api/cars 는 이름만 준다). 지금은 사용자 자율 선택 + 설명 문구로만 안내.
 *   계획서 §7 남은 결정. */

CarrotIntro.register({
  id: "hda",
  file: "step3_hda.js",
  flow: true,
  hkgOnly: true,

  render(el) {
    const { t, ctx } = CarrotIntro;

    el.innerHTML = `
      ${CarrotIntroIcons.hda}
      <h2 class="intro-title">${t("hdaTitle")}</h2>
      <p class="intro-lead"><b>${ctx.car}</b></p>
      <div class="intro-opts">
        ${[[0, "hda1", "hda1d"], [1, "hda2", "hda2d"]].map(([v, tk, dk]) => `
          <button class="intro-opt intro-opt--desc ${ctx.hda === v ? "is-picked" : ""}"
                  type="button" data-hda="${v}" aria-pressed="${ctx.hda === v}">
            ${t(tk)}
            <span class="intro-opt__d">${t(dk)}</span>
          </button>`).join("")}
      </div>`;

    el.querySelectorAll("[data-hda]").forEach((opt) => {
      opt.addEventListener("click", () => {
        ctx.hda = Number(opt.dataset.hda);
        CarrotIntroApi.setHda(ctx.hda).catch(() => {});
        CarrotIntroShell.pick(opt, "control");
      });
    });
  },
});
