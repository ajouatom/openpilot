from __future__ import annotations

from cluster_config import (
    TURN_SIGNAL_BLINK_ON_SECONDS,
    TURN_SIGNAL_BLINK_PERIOD_SECONDS,
)


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def smoothstep(value: float) -> float:
    value = clamp(value, 0.0, 1.0)
    return value * value * (3.0 - 2.0 * value)


def blend_color(
    color: tuple[int, int, int],
    target: tuple[int, int, int],
    amount: float,
) -> tuple[int, int, int]:
    amount = clamp(amount, 0.0, 1.0)
    return tuple(
        int(round(channel + (target_channel - channel) * amount))
        for channel, target_channel in zip(color, target)
    )


def lighten(color: tuple[int, int, int], amount: float) -> tuple[int, int, int]:
    return blend_color(color, (255, 255, 255), amount)


def darken(color: tuple[int, int, int], amount: float) -> tuple[int, int, int]:
    return blend_color(color, (0, 0, 0), amount)


def blink_visible(now: float, started_at: float, until: float) -> bool:
    if now >= until:
        return False
    elapsed = now - started_at
    return elapsed % TURN_SIGNAL_BLINK_PERIOD_SECONDS < TURN_SIGNAL_BLINK_ON_SECONDS
