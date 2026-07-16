# Cluster 도로 카메라 선택 시 지도 영역 카메라 깜빡임 분석

## 분석 기준

- 브랜치/커밋: `origin/thftgr/navi-stream` / `4488bd591b26dad1b9d44a6f3890336a0ba25ec8`
- 확인일: 2026-07-16
- 설정: `ClusterHudCameraViewMode=2` (`전방 카메라 배경`)
- 범위: 표시 경로와 EGL/OpenGL 상태 분석 및 수정

## 결론

7714 지도 프레임이 H.264 하드웨어 디코딩의 `video/nv12-dmabuf` 경로를 사용하면서 전방 카메라도
TICI EGL zero-copy 경로를 사용할 때, 두 화면이 같은 OpenGL texture unit 0의
`GL_TEXTURE_EXTERNAL_OES` 바인딩을 공유한다. 그러나 현재 코드는 각각의 외부 EGL 이미지를 **새
소스 프레임이 들어온 때에만** 바인딩하고, 이미 캐시된 프레임을 다시 그릴 때는 바인딩하지 않는다.

따라서 다음 순서로 지도 위치에 전방 카메라가 그려질 수 있다.

1. 새 지도 프레임이 들어오면 map EGLImage가 external texture에 바인딩되고 지도는 정상 표시된다.
2. 이후 새 카메라 프레임이 들어오면 camera EGLImage가 같은 texture unit 0의 external target에
   바인딩된다.
3. 지도 sequence가 그대로인 렌더 프레임에서는 map texture 업로드/바인딩을 생략한다.
4. map의 `samplerExternalOES texture0`가 현재 남아 있는 camera external 바인딩을 읽어 지도 사각형에
   카메라 영상을 그린다.
5. 다음 새 지도 프레임이 오면 map을 다시 바인딩해 정상 지도로 돌아오므로 지도와 카메라가 번갈아
   보이는 깜빡임이 생긴다.

이것은 지도 패널의 visible/stale 조건이나 카메라 배경의 단순 위치 중첩만으로는 설명되지 않는다.
지도 패널은 카메라보다 나중에 그려지고, map 사각형에 불투명 배경도 먼저 그리므로 map 데이터가
잠깐 없거나 texture upload가 실패한 경우에는 검은/패널 배경이 보여야 한다. 지도 영역 전체에 실제
카메라 영상이 보이는 것은 잘못된 external texture가 샘플링된 결과와 일치한다.

## 코드 근거

### 렌더 순서와 패널 불투명 처리

- `cluster_renderer.py:1185-1197`: world를 먼저 그리고 HUD를 나중에 그린다.
- `cluster_renderer.py:1222-1228`: road-camera 모드에서는 전방 카메라를 world background에 그린다.
- `cluster_renderer.py:3267-3270`: 기본 화면의 navi live panel은 그보다 나중에 HUD에서 그린다.
- `cluster_renderer.py:4365-4379`: 지도 패널과 map rect에 불투명 배경을 그린 뒤 map texture를 그린다.

따라서 패널 자체가 정상 실행되는 한 하부 카메라가 지도 영역을 그대로 비추는 구조는 아니다.

### 카메라 external texture 바인딩

- `cluster_live_camera.py:205-228`: 현재 VisionIPC buffer index에 대응하는 EGLImage를 찾은 뒤
  `_texture_needs_update`일 때만 `bind_egl_image_to_texture()`를 호출한다.
- `cluster_live_camera.py:229-240`: 이후 external shader로 texture를 그린다.
- `cluster_live_camera.py:140-148`: `_texture_needs_update`는 새 카메라 프레임을 poll했을 때 켜진다.

즉, 카메라 화면을 매번 그리더라도 새 프레임이 아니면 external target을 다시 바인딩하지 않는다.

### 지도 external texture 바인딩

- `cluster_renderer.py:4027-4111`: 새 H.264 hardware buffer sequence를 처리할 때 EGLImage를 만들거나
  캐시에서 찾고 `bind_egl_image_to_texture()`를 호출한다.
- `cluster_renderer.py:4212-4227`: 캐시의 sequence와 현재 frame sequence가 같으면 바인딩 없이 기존
  texture를 즉시 반환한다.
- `cluster_renderer.py:3917-3930`: `video/nv12-dmabuf`는 `samplerExternalOES texture0` shader로
  그리지만 draw 직전에는 map EGLImage를 다시 바인딩하지 않는다.

### 공유되는 GL 상태

- `openpilot/system/ui/lib/egl.py:263-268`: 공용 helper는 항상 `GL_TEXTURE0`을 활성화하고
  `GL_TEXTURE_EXTERNAL_OES`에 texture id를 바인딩한다.
- `third_party/raylib/include/rlgl.h:3051-3060`: Raylib batch draw는 texture unit 0에서 전달받은 id를
  `GL_TEXTURE_2D`에만 바인딩한다. external target은 건드리지 않는다.
- `third_party/raylib/include/rlgl.h:4400-4408`: shader 전환 시 기존 batch를 flush하지만, 해당 texture를
  `GL_TEXTURE_EXTERNAL_OES`에 복구하는 처리는 없다.
- camera와 map이 서로 다른 Raylib texture object를 보유하더라도, external sampler가 실제로 참조할
  대상은 draw 시점의 texture unit 0 / external-target 바인딩이다.
- Raylib의 일반 `draw_texture_pro()`가 관리하는 `GL_TEXTURE_2D` 바인딩만으로는
  `GL_TEXTURE_EXTERNAL_OES` 바인딩을 각 object에 맞게 복구하지 못한다.

## 회귀가 들어온 지점

- `60bc80e4` (`Introduce TICI hardware-accelerated H264 decoding`)에서 지도에
  `video/nv12-dmabuf` + external EGL texture 경로가 추가됐다.
- 그 이전 software H.264 경로는 Y/U/V의 일반 `sampler2D` texture를 사용하므로 road-camera
  `samplerExternalOES` 바인딩과 충돌하지 않는다.
- 현재 테스트 `test_zero_copy_rebinds_only_for_new_frame`은 카메라를 새 프레임에서만 다시 바인딩하는
  동작을 명시적으로 기대한다. 지도 하드웨어 경로 테스트도 새 sequence upload 때의 bind만 검사한다.
  두 external texture 사용자를 한 렌더 프레임에 교차해서 그리는 테스트는 없다.

## 증상별 판별

| 관찰 | 판단 |
|---|---|
| 지도 사각형 전체가 카메라로 바뀌었다가 지도로 복귀 | external texture 바인딩 충돌과 정확히 일치 |
| 지도 패널 왼쪽 가장자리 약 76 px에만 카메라가 보임 | camera background 폭 1200과 panel 시작 x=1124의 의도된 76 px 중첩/클리핑 문제도 별도 확인 |
| `CLUSTER_HARDWARE_H264_DECODE=0`에서 증상 소멸 | 본 원인을 강하게 확정. map이 software YUV `sampler2D`로 바뀜 |
| 전방 카메라 배경을 끄면 증상 소멸 | 두 번째 external texture 사용자가 사라지므로 본 원인과 일치 |
| JPEG/PNG map에서는 증상 없음 | 본 원인과 일치. 일반 `sampler2D` 경로이기 때문 |
| 수신 원본 `render:map_main` 자체에 카메라가 포함됨 | 송신 측 key/stream 혼입이며 별도 원인. 현재 수신/렌더 구조에서는 가능성이 낮음 |

## 적용한 수정

1. `CAMERA_BACKGROUND_W`를 `NAVI_LIVE_PANEL_X`와 같은 공유 경계로 변경했다. 설계 좌표에서
   카메라는 `x=0..1124`, 내비 패널은 `x=1124..1916`만 사용하므로 기존 76 px 중첩이 제거된다.
2. camera zero-copy draw 직전에 현재 camera EGLImage를 매번 external texture에 다시 바인딩한다.
3. map `video/nv12-dmabuf` draw 직전에 `cached.hardware_token`의 EGLImage를 매번 map external texture에
   다시 바인딩한다.
4. 카메라의 매 draw 재바인딩, 지도의 매 draw 재바인딩, 카메라 우측 끝과 내비 패널 좌측 시작의
   일치를 검증하는 단위 테스트를 추가했다.

Windows 작업 환경에는 `pytest`와 `pyray`가 없어 단위 테스트 실행은 시작하지 못했다. 변경한 소스와
테스트 파일은 Python 3.12 `py_compile`을 통과했고 `git diff --check`에서도 오류가 없었다.

버퍼 token `(decoder_generation, capture_index)`와 EGLImage cache는 디코더 capture pool 재사용을
구분하고 있으며, EGLImage도 fd를 복제해 보유한다. 현재 증상과 가장 직접적으로 맞는 결함은 buffer
index 충돌보다는 draw 시점 external-target 재바인딩 누락이다.
