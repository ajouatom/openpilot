"use strict";

/* 4단계 — 주행제어 방식 (프리셋)
 *
 * 선택하면 서버가 param 7개를 한 번에 적용한다.
 * 값은 server/features/intro/presets.py 가 갖는다 — 여기서는 이름만 보낸다.
 *
 * ── 세 방식이 실제로 뭐가 다른가 (opendbc 코드 기준) ──
 *
 * HyundaiCameraSCC > 0 이면 interface.py:34 에서 HyundaiFlags.CAMERA_SCC 가 붙고,
 * interface.py:40 에서 카메라 CAN 버스가 1번으로 고정되며,
 * carstate.py:228 에서 SCC11~14 를 메인 버스가 아닌 **카메라 버스**로 읽는다.
 *
 * 그리고 여기가 당근파일럿 고유 부분 — interface.py:179 의 `# carrot` 주석:
 *
 *     ret.openpilotLongitudinalControl = alpha_long and ret.alphaLongitudinalAvailable
 *     # carrot, if camera_scc enabled, enable openpilotLongitudinalControl
 *     if ret.flags & HyundaiFlags.CAMERA_SCC.value or enable_radar_tracks > 0 ...:
 *       ret.radarUnavailable = False
 *       ret.openpilotLongitudinalControl = True if camera_scc < 3 else False
 *
 * 순정 openpilot 은 alpha_long(실험적 롱컨)이 켜져야만 롱컨을 한다.
 * 당근파일럿은 CAMERA_SCC 가 켜지면 그 게이트를 무시하고 롱컨을 강제로 켠다.
 * → "전방 카메라 배선만으로 롱컨" 이 당근파일럿 전용 기능인 이유.
 *
 * ③ 순정은 HyundaiCameraSCC=0 이라 위 분기를 안 타므로
 *    openpilotLongitudinalControl 은 꺼진 채고(interface.py:192 pcmCruise=True),
 *    SpeedFromPCM=2 로 커브/카메라 감속만 버튼으로 넣는다.
 *
 * ── 실질 차이 ──
 *                      HyundaiCameraSCC  SpeedFromPCM  EnableCornerRadar  AutoCruiseControl
 *   ① ADAS·레이더 롱컨        1              0                1                  1
 *   ② 카메라 롱컨             1              0                0                  1
 *   ③ 순정                    0              2                0                  0
 *
 * ⚠ EnableCornerRadar 가 ①과 ②를 가르는 유일한 축이다. 임의로 바꾸면
 *   두 선택지가 완전히 같은 일을 하게 된다 (실제로 한 번 그랬다 — 계획서 §5.1). */

CarrotIntro.register({
  id: "control",
  file: "step4_control.js",
  flow: true,

  render(el) {
    const { t, ctx, br } = CarrotIntro;

    const OPTIONS = [
      ["radar_long",  "p1", "p1d"],
      ["camera_long", "p2", "p2d"],
      ["stock",       "p3", "p3d"],
    ];

    el.innerHTML = `
      ${CarrotIntroIcons.ctrl}
      <h2 class="intro-title">${t("ctrlTitle")}</h2>
      <p class="intro-lead">${t("ctrlSub")}</p>
      <div class="intro-opts">
        ${OPTIONS.map(([preset, tk, dk]) => `
          <button class="intro-opt intro-opt--desc ${ctx.preset === preset ? "is-picked" : ""}"
                  type="button" data-preset="${preset}" aria-pressed="${ctx.preset === preset}">
            ${t(tk)}
            <span class="intro-opt__d">${br(t(dk))}</span>
          </button>`).join("")}
      </div>`;

    el.querySelectorAll("[data-preset]").forEach((opt) => {
      opt.addEventListener("click", () => {
        ctx.preset = opt.dataset.preset;
        /* 적용은 마지막 단계에서 한 번에 한다. 여기서 미리 쓰면
           사용자가 이전으로 돌아가 바꿨을 때 이전 값이 남는다. */
        CarrotIntroShell.pick(opt, "legal");
      });
    });
  },
});
