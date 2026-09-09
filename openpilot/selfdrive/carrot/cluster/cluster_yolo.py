"""Read-only, expiring YOLO summary for the yolo2 experimental cluster."""
from dataclasses import dataclass
import math


@dataclass(frozen=True, slots=True)
class YoloBox:
    label: str
    confidence: int
    points: tuple[float, ...]


@dataclass(frozen=True, slots=True)
class YoloDisplay:
    state: str
    runs: int = 0
    execution_ms: float | None = None
    objects: tuple[tuple[str, int, int], ...] = ()
    more: int = 0
    camera: str = ""
    timestamp_eof: int = 0
    boxes: tuple[YoloBox, ...] = ()


def _get(value, key, default=None):
    return value.get(key, default) if isinstance(value, dict) else getattr(value, key, default)


def build_yolo_display(message, *, enabled, valid, transport_age, image_age):
    if message is None:
        return YoloDisplay("waiting") if enabled else None
    runs = max(0, int(_get(message, "runs", 0)))
    if not enabled:
        return YoloDisplay("off", runs)
    if not math.isfinite(transport_age) or not 0 <= transport_age <= 2.0:
        return YoloDisplay("stale", runs)
    state = str(_get(message, "state", "waiting"))
    # Paused/error messages are deliberately invalid as detection data, but
    # their current status must remain visible and clear previous objects.
    if state in ("paused", "error", "overrun", "no_budget", "camera_pending", "warming"):
        return YoloDisplay(state, runs)
    if state != "run":
        return YoloDisplay("waiting", runs)
    if not valid:
        return YoloDisplay("invalid", runs)
    # The independent 3 Hz CPU observer has ~200 ms inference latency. Keep
    # its textual observation between frames; camera boxes retain their
    # separate, stricter image-alignment check below.
    max_age = .6 if _get(message, "modelId") == "signal-v33-observe-int8-s260911" else .35
    if not math.isfinite(image_age) or not 0 <= image_age <= max_age:
        return YoloDisplay("stale", runs)
    seconds = float(_get(message, "executionTime", 0))
    execution_ms = seconds*1000 if math.isfinite(seconds) and seconds > 0 else None
    grouped = {}
    boxes = []
    for detection in list(_get(message, "detections", ()))[:40]:
        score = float(_get(detection, "confidence", 0))
        if not math.isfinite(score) or not 0 <= score <= 1:
            continue
        label = str(_get(detection, "label", "object"))[:32]
        count, best = grouped.get(label, (0, 0))
        grouped[label] = (count+1, max(best, round(score*100)))
        points = tuple(float(v) for v in _get(detection, "cameraPoints", ()))
        if len(points) == 8 and all(math.isfinite(v) and abs(v) <= 10 for v in points):
            boxes.append(YoloBox(label, round(score*100), points))
    groups = sorted(((label, count, best) for label, (count, best) in grouped.items()), key=lambda x: (-x[2], x[0]))
    return YoloDisplay("run", runs, execution_ms, tuple(groups[:3]), sum(g[1] for g in groups[3:]),
                       str(_get(message, "camera", "")), int(_get(message, "timestampEof", 0)), tuple(boxes))


def camera_boxes(display, *, camera, timestamp_eof, video_rect):
    """Project original-camera corners through the very same video crop/zoom.

    Road boxes cannot be placed on wide images without depth/extrinsics. Also
    reject old camera frames even when the detection transport itself is fresh.
    """
    if (display is None or display.state != "run" or display.camera != camera
            or min(display.timestamp_eof, timestamp_eof) <= 0
            or abs(display.timestamp_eof-timestamp_eof) > 200_000_000):
        return ()
    x, y, width, height = video_rect
    return tuple((box, tuple((x+box.points[i]*width, y+box.points[i+1]*height) for i in range(0, 8, 2)))
                 for box in display.boxes)


def box_label(box, language):
    return f"{_KO_LABELS.get(box.label, box.label) if language == 'ko' else box.label} {box.confidence}%"


_KO_LABELS = {"person": "사람", "bicycle": "자전거", "car": "자동차", "motorcycle": "오토바이",
              "bus": "버스", "truck": "트럭", "traffic light": "신호등", "stop sign": "정지표지",
              "chair": "의자", "dog": "개", "cat": "고양이", "potted plant": "화분"}


_KO_LABELS.update({"red_visible": "빨강 감지", "green_visible": "초록 감지"})


def yolo_text(display, language):
    korean = language == "ko"
    states = {"run": ("실행", "RUN"), "paused": ("일시정지", "PAUSED"), "error": ("오류", "ERROR"),
              "overrun": ("시간초과", "OVERRUN"), "waiting": ("준비대기", "WAITING"),
              "stale": ("갱신대기", "STALE"), "invalid": ("수신오류", "INVALID"), "off": ("꺼짐", "OFF"),
              "no_budget": ("여유시간대기", "WAITING FOR BUDGET"), "camera_pending": ("카메라 우선", "CAMERA FIRST"),
              "warming": ("안정화대기", "STABILIZING")}
    title = f"YOLO {states.get(display.state, states['waiting'])[0 if korean else 1]}"
    if display.execution_ms is not None:
        title += f" {display.execution_ms:.1f}ms"
    title += f" · #{display.runs}"
    if display.state != "run":
        return title, "검출 표시 대기" if korean else "Detections unavailable"
    details = [f"{_KO_LABELS.get(label, label) if korean else label} {count} ({best}%)"
               for label, count, best in display.objects]
    if display.more:
        details.append(f"+{display.more}")
    return title, " · ".join(details) if details else ("검출 없음" if korean else "No detections")
