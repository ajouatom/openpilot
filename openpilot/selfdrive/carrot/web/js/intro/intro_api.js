"use strict";

/* 인트로 API
 *
 * 인트로가 백엔드를 부르는 유일한 자리. step 파일은 fetch 를 직접 하지 않는다.
 *
 *   POST /api/web_settings          features/web_settings.py   (언어)
 *   POST /api/param_set             features/params.py:56      (HDA)
 *   POST /api/params_restore        features/params.py:107     (백업 복원, multipart)
 *   POST /api/intro/apply_preset    features/intro/routes.py   (프리셋 일괄 적용)
 *   POST /api/intro/complete        features/intro/routes.py   (완료 기록)
 *
 * 차량 목록과 차량 선택은 여기 없다. step2_car.js 가 기존 웹의
 * ensureCarsLoaded() / setParam() / applyCurrentCarLabel() 을 그대로 쓴다 —
 * 같은 CARS 캐시와 같은 라벨 상태를 공유해야 하므로 인트로가 따로
 * /api/cars 를 부르면 안 된다.
 */

globalThis.CarrotIntroApi = (() => {
  const isPreview = () => CarrotIntro.ctx.preview === true;
  const previewResult = (action) => Promise.resolve({ ok: true, preview: true, action });

  async function postJson(url, body) {
    const r = await fetch(url, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(body || {}),
    });
    const j = await r.json().catch(() => ({}));
    if (!r.ok || j.ok === false) throw new Error(j.error || `HTTP ${r.status}`);
    return j;
  }

  /* 웹 UI 언어. web_settings 는 /data/carrot/state/web_settings.json 에 저장되어
     git reset 에도 살아남는다. */
  const setLanguage = (lang) => isPreview()
    ? previewResult("set_language")
    : postJson("/api/web_settings", { web_language: lang });

  /* 차량 선택 단계도 이 파사드를 통과시킨다. 프리뷰에서는 Params와
     localStorage/설정 화면 라벨을 모두 그대로 둔다. */
  async function setCar(fullName, label) {
    if (isPreview()) return previewResult("set_car");
    const write = setParam("CarSelected3", fullName);
    applyCurrentCarLabel(label);
    await write;
    return { ok: true };
  }

  /* HDA1/HDA2 → CanfdHDA2. HKG 가 아니면 아예 부르지 않는다(기존값 유지). */
  const setHda = (v) => isPreview()
    ? previewResult("set_hda")
    : postJson("/api/param_set", { name: "CanfdHDA2", value: v });

  /* 프리셋 적용. 값은 서버(features/intro/presets.py)가 갖고 있고 우리는
     이름만 보낸다. param_set 을 7번 부르지 않는 이유: 중간에 실패하면
     차량 제어 설정이 부분 적용된 채 남는다. */
  const applyPreset = (preset) => isPreview()
    ? previewResult("apply_preset")
    : postJson("/api/intro/apply_preset", { preset });

  const complete = (reason) => isPreview()
    ? previewResult("complete")
    : postJson("/api/intro/complete", { reason: reason || "user_finished" });

  /* 백업 파일 복원: multipart.
     QR 경로(/api/params_restore_preview + _json)는 인트로에서 쓰지 않는다 —
     파일 하나로 충분하고 카메라 권한·스캔 UI 는 인트로 첫 화면이 감당할
     무게가 아니다. 두 API 는 서버에 그대로 있으니 Tools 의 QR 복원은 영향 없다. */
  async function restoreFile(file) {
    if (isPreview()) return previewResult("restore_file");
    const fd = new FormData();
    fd.append("file", file, file.name || "params_backup.json");
    const r = await fetch("/api/params_restore", { method: "POST", body: fd });
    const j = await r.json().catch(() => ({}));
    if (!r.ok || j.ok === false) throw new Error(j.error || `HTTP ${r.status}`);
    return j;
  }

  return { isPreview, setLanguage, setCar, setHda, applyPreset, complete, restoreFile };
})();
