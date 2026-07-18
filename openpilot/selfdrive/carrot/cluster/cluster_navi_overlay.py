from __future__ import annotations

from dataclasses import replace

from cluster_models import ClusterUiState, NaviLiveState
from cluster_navi import resolve_navi_speed_limit


def navi_guidance_active(navi: NaviLiveState | None) -> bool:
    return bool(
        navi is not None
        and (
            navi.current is not None
            or (navi.status is not None and navi.status.guidance_active)
        )
    )


def merge_navi_overlay_state(base: ClusterUiState, overlay: ClusterUiState) -> ClusterUiState:
    """Attach live navigation surfaces without replacing replay/live vehicle state."""
    navi = overlay.navi_live
    speed_limit_kph, speed_limit_source = resolve_navi_speed_limit(
        base.speed_limit_kph,
        base.speed_limit_source,
        navi,
    )

    return replace(
        base,
        speed_limit_kph=speed_limit_kph,
        speed_limit_source=speed_limit_source,
        external_nav_active=base.external_nav_active or navi_guidance_active(navi),
        navi_live=navi,
        navi_dashboard=overlay.navi_dashboard,
        center_clock_text=base.center_clock_text or overlay.center_clock_text,
    )
