"use strict";

/* 0단계 — 로고 + 인사말 순환 + 언어선택 + 설정 불러오기
 *
 * 인사말 순환을 이 첫 화면에서 바로 보여준다. 언어를 고른 뒤가 아니라.
 * 순환하는 인사말 자체가 "여러 언어를 지원한다"는 안내이고,
 * 바로 아래 언어 목록이 그 선택지다.
 *
 * 기본 언어는 브라우저 언어를 감지해 하이라이트한다. 못 알아보면 영어.
 * (detectCarrotIntroLang — intro_i18n.js) */

CarrotIntro.register({
  id: "welcome",
  file: "step0_welcome.js",
  flow: true,

  render(el) {
    const { t, ctx, assetUrl } = CarrotIntro;
    const CYCLE_MS = 1800;

    el.innerHTML = `
      <img class="intro-logo" src="${assetUrl("img_spinner_comma.png")}" alt="CarrotPilot" />

      <div class="intro-cycler" aria-hidden="true">
        ${CarrotIntroLangs.map(([code, , greeting]) =>
          `<div class="intro-cyc"${code === "zh" ? ' lang="zh"' : ""}>${greeting}</div>`).join("")}
      </div>

      <div class="intro-opts intro-opts--inline">
        ${CarrotIntroLangs.map(([code, name]) => `
          <button class="intro-opt ${ctx.lang === code ? "is-picked" : ""}" type="button"
                  data-lang="${code}" lang="${code}" aria-pressed="${ctx.lang === code}">
            ${name}
          </button>`).join("")}
      </div>

      <button class="intro-ghost" type="button" data-restore>${t("restore")}</button>`;

    /* 차량 목록을 미리 받아둔다. 사용자가 언어를 고르는 동안 끝나므로
       2단계 진입 때 목록이 뒤늦게 채워지며 튀는 일이 없다. */
    globalThis.prefetchCarrotIntroCars?.();

    /* ── 인사말 순환 ──
       선택을 기다리는 화면이므로 끝나지 않고 계속 돈다.
       (자동으로 다음 단계로 넘어가지 않는다 — 언어를 골라야 진행) */
    const items = [...el.querySelectorAll(".intro-cyc")];
    let n = 0;
    const tick = () => {
      items.forEach((x) => x.classList.remove("is-on"));
      void items[n].offsetWidth;   // reflow 강제 — 같은 애니메이션 재시작
      items[n].classList.add("is-on");
      n = (n + 1) % items.length;
    };
    tick();
    const timer = setInterval(tick, CYCLE_MS);

    /* ── 언어 선택 ── */
    el.querySelectorAll("[data-lang]").forEach((opt) => {
      opt.addEventListener("click", () => {
        ctx.lang = opt.dataset.lang;
        CarrotIntro.applyLang();
        /* 언어는 즉시 저장한다. 실패해도 인트로는 계속 진행 —
           web_settings 저장 실패로 첫 화면이 막히면 안 된다. */
        CarrotIntroApi.setLanguage(ctx.lang).catch(() => {});
        CarrotIntroShell.pick(opt, "car");
      });
    });

    el.querySelector("[data-restore]").addEventListener("click", () => {
      CarrotIntroShell.goTo("restore");
    });

    /* 셸이 pane 을 버릴 때 호출한다. 타이머가 남아 죽은 pane 을
       가리키면 다음 단계가 엉킨다. */
    return () => clearInterval(timer);
  },
});
