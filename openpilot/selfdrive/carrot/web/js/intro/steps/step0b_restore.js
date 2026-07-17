"use strict";

/* 0-b단계 — 설정 불러오기 (본 흐름 밖. 0단계에서만 진입)
 *
 * 백업에는 CarSelected3 와 프리셋 param 이 전부 들어있으므로 차량·HDA·프리셋
 * 단계를 건너뛴다. 다만 앱으로 바로 튕기지는 않는다 —
 * 법적 고지와 마지막 인사는 복원 사용자도 똑같이 거친다.
 *
 *   복원 성공 → legal → outro → 시작하기 → 앱
 *
 * ctx.restored 를 세워두면 outro 가 프리셋을 적용하지 않고
 * complete(reason) 만 부른다 (백업 값을 덮어쓰면 안 되니까).
 *
 * QR 은 뺐다. 파일 하나로 충분하고, 카메라 권한·스캔 UI 가 인트로
 * 첫 화면에서 감당할 무게가 아니다.
 *
 * "이전" 은 셸이 방문 기록으로 알아서 0단계로 되돌린다. */

CarrotIntro.register({
  id: "restore",
  file: "step0b_restore.js",
  flow: false,

  render(el) {
    const { t, ctx } = CarrotIntro;

    el.innerHTML = `
      ${CarrotIntroIcons.restore}
      <h2 class="intro-title">${t("rsTitle")}</h2>
      <p class="intro-lead">${t("rsSub")}</p>
      <div class="intro-opts">
        <button class="intro-opt intro-opt--desc" type="button" data-pick>
          ${t("rsFile")}
          <span class="intro-opt__d">${t("rsFiled")}</span>
        </button>
      </div>
      <input type="file" id="introRestoreFile" accept="application/json,.json" hidden />`;

    const picker = el.querySelector("[data-pick]");
    const fileInput = el.querySelector("#introRestoreFile");

    picker.addEventListener("click", () => fileInput.click());

    fileInput.addEventListener("change", async () => {
      const file = fileInput.files && fileInput.files[0];
      if (!file) return;

      picker.classList.add("is-picked");
      try {
        await CarrotIntroApi.restoreFile(file);
      } catch (err) {
        picker.classList.remove("is-picked");
        CarrotIntroShell.error(`복원 실패: ${err.message}`);
        return;
      }

      /* 마법사 중간은 건너뛰되 고지와 마지막 인사는 거친다 */
      ctx.restored = true;
      CarrotIntroShell.goTo("legal");
    });
  },
});
