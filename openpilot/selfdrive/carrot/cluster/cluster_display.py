from __future__ import annotations

import math


CLUSTER_LANGUAGE_EN = "en"
CLUSTER_LANGUAGE_KO = "ko"
KPH_TO_MPH = 0.621371192237334
METERS_TO_FEET = 3.280839895013123
METERS_PER_MILE = 1609.344

_TEXT = {
    CLUSTER_LANGUAGE_EN: {
        "driving_report": "DRIVING REPORT",
        "trip_summary": "TRIP SUMMARY",
        "time": "TIME",
        "distance": "DISTANCE",
        "average_speed": "AVG SPEED",
        "max_speed": "MAX SPEED",
        "auto_drive": "AUTO DRIVE",
        "max_accel": "MAX ACCEL",
        "max_decel": "MAX DECEL",
        "hard_accel": "HARD ACCEL",
        "hard_brake": "HARD BRAKE",
        "hard_corner": "HARD CORNER",
        "system": "SYSTEM",
        "device_angle": "DEVICE ANGLE",
        "driving_mode_eco": "ECO",
        "driving_mode_safe": "SAFE",
        "driving_mode_normal": "NORMAL",
        "driving_mode_sport": "SPORT",
        "navigation": "NAVIGATION",
        "next": "NEXT",
        "section": "SECTION",
        "average": "AVG",
        "route": "ROUTE",
        "connected": "CONNECTED",
        "waiting": "WAITING",
        "off_route": "OFF ROUTE",
        "openpilot_unavailable": "openpilot Unavailable",
        "waiting_to_start": "Waiting to start",
        "take_control_immediately": "TAKE CONTROL IMMEDIATELY",
        "system_unresponsive": "System Unresponsive",
        "reboot_device": "Reboot Device",
    },
    CLUSTER_LANGUAGE_KO: {
        "driving_report": "주행리포트",
        "trip_summary": "주행 요약",
        "time": "시간",
        "distance": "거리",
        "average_speed": "평균속도",
        "max_speed": "최고속도",
        "auto_drive": "자동주행",
        "max_accel": "최대가속",
        "max_decel": "최대감속",
        "hard_accel": "급가속",
        "hard_brake": "급감속",
        "hard_corner": "급코너",
        "system": "시스템",
        "device_angle": "디바이스 설치각",
        "driving_mode_eco": "연비",
        "driving_mode_safe": "안전",
        "driving_mode_normal": "일반",
        "driving_mode_sport": "고속",
        "navigation": "내비게이션",
        "next": "다음",
        "section": "구간",
        "average": "평균",
        "route": "경로",
        "connected": "연결됨",
        "waiting": "연결 대기",
        "off_route": "경로 이탈",
        "openpilot_unavailable": "openpilot 사용 불가",
        "waiting_to_start": "시작 대기 중",
        "take_control_immediately": "즉시 운전대를 잡으세요",
        "system_unresponsive": "시스템 응답 없음",
        "reboot_device": "기기를 재부팅하세요",
    },
}


def normalize_cluster_language(value: object, default: str = CLUSTER_LANGUAGE_EN) -> str:
    if isinstance(value, bytes):
        value = value.decode("utf-8", "ignore")
    normalized = str(value or "").strip().lower().removeprefix("main_")
    if normalized in ("ko", "kr", "kor", "korean"):
        return CLUSTER_LANGUAGE_KO
    if normalized in ("en", "eng", "english"):
        return CLUSTER_LANGUAGE_EN
    return CLUSTER_LANGUAGE_KO if default == CLUSTER_LANGUAGE_KO else CLUSTER_LANGUAGE_EN


def normalize_metric_setting(value: object, default: bool = True) -> bool:
    if value is None:
        return bool(default)
    if isinstance(value, bytes):
        value = value.decode("utf-8", "ignore")
    if isinstance(value, str):
        normalized = value.strip().lower()
        if normalized in ("", "none"):
            return bool(default)
        return normalized not in ("0", "false", "off", "no")
    return bool(value)


def cluster_text(language: str, key: str) -> str:
    normalized = normalize_cluster_language(language)
    return _TEXT[normalized].get(key, _TEXT[CLUSTER_LANGUAGE_EN].get(key, key))


def display_speed(speed_kph: int | float, is_metric: bool) -> float:
    speed = float(speed_kph)
    if not math.isfinite(speed):
        return 0.0
    return speed if is_metric else speed * KPH_TO_MPH


def speed_unit(is_metric: bool) -> str:
    return "km/h" if is_metric else "mph"


def format_radar_distance(distance_m: int | float, is_metric: bool) -> str:
    distance = max(0.0, float(distance_m))
    if is_metric:
        return f"{distance:.0f} m"
    return f"{distance * METERS_TO_FEET:.0f} ft"


def format_navi_distance(distance_m: int | float, is_metric: bool) -> str:
    distance = max(0.0, float(distance_m))
    if is_metric:
        if distance < 1000.0:
            return f"{int(round(distance))} m"
        if distance < 10_000.0:
            return f"{distance / 1000.0:.1f} km"
        return f"{distance / 1000.0:.0f} km"

    miles = distance / METERS_PER_MILE
    if miles < 0.1:
        return f"{distance * METERS_TO_FEET:.0f} ft"
    if miles < 10.0:
        return f"{miles:.1f} mi"
    return f"{miles:.0f} mi"


def format_trip_distance(distance_m: int | float, is_metric: bool) -> str:
    distance = max(0.0, float(distance_m))
    if is_metric:
        if distance < 1000.0:
            return f"{distance:.0f} m"
        if distance < 10_000.0:
            return f"{distance / 1000.0:.2f} km"
        return f"{distance / 1000.0:.1f} km"

    miles = distance / METERS_PER_MILE
    if miles < 0.1:
        return f"{distance * METERS_TO_FEET:.0f} ft"
    if miles < 10.0:
        return f"{miles:.2f} mi"
    return f"{miles:.1f} mi"
