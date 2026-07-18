from __future__ import annotations

import math
import time
from dataclasses import replace
from typing import Any

from cluster_models import (
    NaviCrossroadInfo,
    NaviGuidanceInfo,
    NaviItemMeta,
    NaviLaneInfo,
    NaviLiveState,
    NaviRouteInfo,
    NaviSpeedInfo,
    NaviStatusInfo,
    NaviTrafficLightInfo,
    NaviVehicleInfo,
)


LIVE_TTL_S = 6.0
STATUS_TTL_S = 10.0
ROUTE_TTL_S = 30.0
TRAFFIC_TTL_S = 60.0
MAX_ROUTE_POINTS = 256


def resolve_navi_speed_limit(
    base_limit_kph: int | None,
    base_source: str | None,
    navi: NaviLiveState | None,
) -> tuple[int | None, str | None]:
    if base_source == "v":
        return base_limit_kph, base_source
    if navi is None or navi.speed is None:
        return base_limit_kph, base_source

    navi_limit = navi.speed.road_limit_kph
    if navi_limit is not None and navi_limit > 0:
        return navi_limit, "n"
    return None, None


def _get(obj: Any, name: str, default: Any = None) -> Any:
    if isinstance(obj, dict):
        return obj.get(name, default)
    try:
        return getattr(obj, name)
    except Exception:
        return default


def _int(obj: Any, name: str, default: int = 0) -> int:
    try:
        return int(_get(obj, name, default))
    except (TypeError, ValueError):
        return default


def _float(obj: Any, name: str, default: float = 0.0) -> float:
    try:
        value = float(_get(obj, name, default))
    except (TypeError, ValueError):
        return default
    return value if math.isfinite(value) else default


def _text(obj: Any, name: str) -> str:
    return str(_get(obj, name, "") or "")


def _ints(value: Any, limit: int = 16) -> tuple[int, ...]:
    try:
        return tuple(int(item) for item in value[:limit])
    except Exception:
        try:
            return tuple(int(item) for _, item in zip(range(limit), value, strict=False))
        except Exception:
            return ()


def _meta(item: Any, now: float) -> NaviItemMeta | None:
    meta = _get(item, "meta")
    if meta is None or not bool(_get(meta, "present", False)):
        return None
    received_mono_s = _int(meta, "receivedMonoTimeNanos") / 1_000_000_000.0
    if received_mono_s <= 0.0 or received_mono_s > now + 60.0:
        received_mono_s = now
    return NaviItemMeta(
        sequence=max(0, _int(meta, "sequence")),
        source_timestamp_ms=max(0, _int(meta, "sourceTimestampMillis")),
        received_mono_s=received_mono_s,
    )


def _same_meta(item: Any | None, meta: NaviItemMeta | None) -> bool:
    return item is not None and meta is not None and _get(item, "meta") == meta


def _guidance(
    item: Any,
    now: float,
    previous: NaviGuidanceInfo | None = None,
) -> NaviGuidanceInfo | None:
    meta = _meta(item, now)
    if meta is None:
        return None
    if _same_meta(previous, meta):
        return previous
    return NaviGuidanceInfo(
        meta=meta,
        distance_m=max(0, _int(item, "distanceM")),
        time_s=max(0, _int(item, "timeSec")),
        turn_type=_int(item, "turnType", -1),
        road_name=_text(item, "roadName"),
        main_text=_text(item, "mainText"),
        near_direction=_text(item, "nearDirection"),
        mid_direction=_text(item, "midDirection"),
        far_direction=_text(item, "farDirection"),
    )


def _lane(item: Any, now: float, previous: NaviLaneInfo | None = None) -> NaviLaneInfo | None:
    meta = _meta(item, now)
    if meta is None:
        return None
    if _same_meta(previous, meta):
        return previous
    return NaviLaneInfo(
        meta=meta,
        count=max(0, min(16, _int(item, "count"))),
        distance_m=max(0, _int(item, "distanceM")),
        visible=bool(_get(item, "visible", False)),
        lane_play=bool(_get(item, "lanePlay", False)),
        current_lane=_int(item, "currentLane", -1),
        turn_code=_int(item, "turnCode", -1),
        turn_info=_ints(_get(item, "turnInfo", ())),
        available=_ints(_get(item, "available", ())),
    )


def parse_carrot_navi(
    data: Any,
    now: float | None = None,
    previous: NaviLiveState | None = None,
) -> NaviLiveState | None:
    now = time.monotonic() if now is None else now
    if _int(data, "schemaVersion") != 1 or not bool(_get(data, "connected", False)):
        return None

    session_id = _text(data, "sessionId")
    if previous is not None and previous.session_id != session_id:
        previous = None

    vehicle_item = _get(data, "vehicle")
    vehicle_meta = _meta(vehicle_item, now)
    vehicle = None
    if vehicle_meta is not None:
        previous_vehicle = previous.vehicle if previous is not None else None
        if _same_meta(previous_vehicle, vehicle_meta):
            vehicle = previous_vehicle
        else:
            vehicle = NaviVehicleInfo(
                meta=vehicle_meta,
                latitude=_float(vehicle_item, "latitude"),
                longitude=_float(vehicle_item, "longitude"),
                heading_deg=_float(vehicle_item, "headingDeg"),
                speed_kph=max(0.0, _float(vehicle_item, "speedKph")),
                road_name=_text(vehicle_item, "roadName"),
                virtual_gps=bool(_get(vehicle_item, "virtualGps", False)),
            )

    previous_lane_ahead = previous.lane_ahead if previous is not None else ()
    lane_ahead_items: list[NaviLaneInfo] = []
    for index, item in enumerate(_get(data, "laneAhead", ()) or ()):
        previous_lane = previous_lane_ahead[index] if index < len(previous_lane_ahead) else None
        lane = _lane(item, now, previous_lane)
        if lane is not None:
            lane_ahead_items.append(lane)
    lane_ahead = tuple(lane_ahead_items)

    speed_item = _get(data, "speed")
    speed_meta = _meta(speed_item, now)
    speed = None
    if speed_meta is not None:
        previous_speed = previous.speed if previous is not None else None
        if _same_meta(previous_speed, speed_meta):
            speed = previous_speed
        else:
            speed = NaviSpeedInfo(
                meta=speed_meta,
                current_kph=max(0.0, _float(speed_item, "currentKph")),
                road_limit_kph=max(0, _int(speed_item, "roadLimitKph")) if bool(_get(speed_item, "roadLimitValid")) else None,
                sdi_type=_int(speed_item, "sdiType", -1) if bool(_get(speed_item, "sdiPresent")) else None,
                sdi_distance_m=max(0, _int(speed_item, "sdiDistanceM")) if bool(_get(speed_item, "sdiPresent")) else None,
                sdi_speed_limit_kph=max(0, _int(speed_item, "sdiSpeedLimitKph")) if bool(_get(speed_item, "sdiPresent")) else None,
                secondary_sdi_type=(
                    _int(speed_item, "secondarySdiType", -1) if bool(_get(speed_item, "secondarySdiPresent")) else None
                ),
                secondary_sdi_distance_m=(
                    max(0, _int(speed_item, "secondarySdiDistanceM"))
                    if bool(_get(speed_item, "secondarySdiPresent")) else None
                ),
                secondary_sdi_speed_limit_kph=(
                    max(0, _int(speed_item, "secondarySdiSpeedLimitKph"))
                    if bool(_get(speed_item, "secondarySdiPresent")) else None
                ),
                section_active=bool(_get(speed_item, "sectionActive", False)),
                section_speed_limit_kph=(
                    max(0, _int(speed_item, "sectionSpeedLimitKph")) if bool(_get(speed_item, "sectionPresent")) else None
                ),
                section_average_kph=(
                    max(0.0, _float(speed_item, "sectionAverageKph"))
                    if bool(_get(speed_item, "sectionPresent")) else None
                ),
                section_remaining_distance_m=(
                    max(0.0, _float(speed_item, "sectionRemainingDistanceM"))
                    if bool(_get(speed_item, "sectionPresent")) else None
                ),
                section_remaining_time_s=(
                    max(0, _int(speed_item, "sectionRemainingTimeSec"))
                    if bool(_get(speed_item, "sectionPresent")) else None
                ),
                section_progress=(
                    max(0.0, min(1.0, _float(speed_item, "sectionProgress")))
                    if bool(_get(speed_item, "sectionPresent")) else None
                ),
            )

    traffic_item = _get(data, "trafficSignal")
    traffic_meta = _meta(traffic_item, now)
    traffic = None
    if traffic_meta is not None and bool(_get(traffic_item, "visible", False)):
        previous_traffic = previous.traffic_light if previous is not None else None
        if _same_meta(previous_traffic, traffic_meta):
            traffic = previous_traffic
        else:
            traffic = NaviTrafficLightInfo(
                distance_m=max(0, _int(traffic_item, "distanceM")),
                red_s=max(0, _int(traffic_item, "redRemainSec")) if bool(_get(traffic_item, "redValid")) else None,
                straight_s=(
                    max(0, _int(traffic_item, "greenRemainSec")) if bool(_get(traffic_item, "greenValid")) else None
                ),
                left_s=max(0, _int(traffic_item, "leftRemainSec")) if bool(_get(traffic_item, "leftValid")) else None,
                right_s=max(0, _int(traffic_item, "rightRemainSec")) if bool(_get(traffic_item, "rightValid")) else None,
                uturn_s=max(0, _int(traffic_item, "uturnRemainSec")) if bool(_get(traffic_item, "uturnValid")) else None,
                red_on=bool(_get(traffic_item, "redOn")) if bool(_get(traffic_item, "redValid")) else None,
                straight_on=bool(_get(traffic_item, "greenOn")) if bool(_get(traffic_item, "greenValid")) else None,
                left_on=bool(_get(traffic_item, "leftOn")) if bool(_get(traffic_item, "leftValid")) else None,
                right_on=bool(_get(traffic_item, "rightOn")) if bool(_get(traffic_item, "rightValid")) else None,
                uturn_on=bool(_get(traffic_item, "uturnOn")) if bool(_get(traffic_item, "uturnValid")) else None,
                meta=traffic_meta,
            )

    crossroad_item = _get(data, "crossroad")
    crossroad_meta = _meta(crossroad_item, now)
    crossroad = None
    if crossroad_meta is not None:
        previous_crossroad = previous.crossroad if previous is not None else None
        if _same_meta(previous_crossroad, crossroad_meta):
            crossroad = previous_crossroad
        else:
            crossroad = NaviCrossroadInfo(
                meta=crossroad_meta,
                visible=bool(_get(crossroad_item, "visible", False)),
                distance_m=max(0, _int(crossroad_item, "distanceM")),
                image_code=max(0, _int(crossroad_item, "imageCode")),
            )

    route_item = _get(data, "route")
    route_meta = _meta(route_item, now)
    route = None
    if route_meta is not None:
        previous_route = previous.route if previous is not None else None
        if _same_meta(previous_route, route_meta):
            route = previous_route
        else:
            polyline = []
            for point in tuple(_get(route_item, "polyline", ()) or ())[:MAX_ROUTE_POINTS]:
                latitude = _float(point, "latitude", math.nan)
                longitude = _float(point, "longitude", math.nan)
                if math.isfinite(latitude) and math.isfinite(longitude):
                    polyline.append((latitude, longitude))
            route = NaviRouteInfo(
                meta=route_meta,
                remaining_distance_m=max(0, _int(route_item, "remainingDistanceM")),
                remaining_time_s=max(0, _int(route_item, "remainingTimeSec")),
                moved_distance_m=max(0, _int(route_item, "movedDistanceM")),
                moved_time_s=max(0, _int(route_item, "movedTimeSec")),
                total_distance_m=max(0, _int(route_item, "totalDistanceM")),
                polyline=tuple(polyline),
            )

    status_item = _get(data, "navigationStatus")
    status_meta = _meta(status_item, now)
    status = None
    if status_meta is not None:
        previous_status = previous.status if previous is not None else None
        if _same_meta(previous_status, status_meta):
            status = previous_status
        else:
            status = NaviStatusInfo(
                meta=status_meta,
                mode=_text(status_item, "mode"),
                guidance_active=bool(_get(status_item, "guidanceActive", False)),
                off_route=bool(_get(status_item, "offRoute", False)),
                route_present=bool(_get(status_item, "routePresent", False)),
            )

    state = NaviLiveState(
        generation=max(0, _int(data, "generation")),
        session_id=session_id,
        vehicle=vehicle,
        current=_guidance(
            _get(data, "guidanceCurrent"),
            now,
            previous.current if previous is not None else None,
        ),
        next=_guidance(
            _get(data, "guidanceNext"),
            now,
            previous.next if previous is not None else None,
        ),
        lane_current=_lane(
            _get(data, "laneCurrent"),
            now,
            previous.lane_current if previous is not None else None,
        ),
        lane_ahead=lane_ahead,
        speed=speed,
        traffic_light=traffic,
        crossroad=crossroad,
        route=route,
        status=status,
    )
    return fresh_carrot_navi(state, now)[0]


def fresh_carrot_navi(state: NaviLiveState | None, now: float | None = None) -> tuple[NaviLiveState | None, float]:
    if state is None:
        return None, math.inf
    now = time.monotonic() if now is None else now
    deadlines: list[float] = []

    def fresh(item: Any, ttl: float) -> Any | None:
        if item is None:
            return None
        meta = _get(item, "meta")
        if meta is None:
            return None
        deadline = float(meta.received_mono_s) + ttl
        if deadline <= now:
            return None
        deadlines.append(deadline)
        return item

    vehicle = fresh(state.vehicle, LIVE_TTL_S)
    current = fresh(state.current, LIVE_TTL_S)
    next_guidance = fresh(state.next, LIVE_TTL_S)
    lane_current = fresh(state.lane_current, LIVE_TTL_S)
    lane_ahead = tuple(item for item in state.lane_ahead if fresh(item, LIVE_TTL_S) is not None)
    speed = fresh(state.speed, LIVE_TTL_S)
    traffic = fresh(state.traffic_light, TRAFFIC_TTL_S)
    crossroad = fresh(state.crossroad, LIVE_TTL_S)
    route = fresh(state.route, ROUTE_TTL_S)
    status = fresh(state.status, STATUS_TTL_S)

    if not any((vehicle, current, next_guidance, lane_current, lane_ahead, speed, traffic, crossroad, route, status)):
        return None, math.inf
    effective = replace(
        state,
        vehicle=vehicle,
        current=current,
        next=next_guidance,
        lane_current=lane_current,
        lane_ahead=lane_ahead,
        speed=speed,
        traffic_light=traffic,
        crossroad=crossroad,
        route=route,
        status=status,
    )
    return effective, min(deadlines, default=math.inf)
