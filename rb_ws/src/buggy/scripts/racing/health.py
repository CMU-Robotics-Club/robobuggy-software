"""Source-time readiness checks. Thresholds are policy assumptions, not accuracy claims."""

import math
from dataclasses import dataclass

import numpy as np


def stamp_seconds(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def header_stamp_seconds(msg):
    """
    Source time of a message, or None when it carries none.

    std_msgs/Header has .stamp directly. The Microstrain 4.x driver wraps it: its messages
    carry a MipHeader whose .header is the std_msgs/Header (seen on the GQ7 with driver
    4.5.0, 2026-09-09). Reading .header.stamp blindly raised AttributeError and took the
    localization monitor down on the buggy.
    """
    hdr = getattr(msg, "header", None)
    if hdr is not None and not hasattr(hdr, "stamp"):
        hdr = getattr(hdr, "header", None)
    stamp = getattr(hdr, "stamp", None)
    if stamp is None or not hasattr(stamp, "sec"):
        return None
    return stamp_seconds(stamp)


def fresh(stamp, now, maximum_age, future_tolerance=0.02):
    """Reject missing/zero stamps, clock rollback, future and expired measurements."""
    return (stamp is not None and math.isfinite(stamp) and math.isfinite(now)
            and stamp > 0 and -future_tolerance <= now - stamp <= maximum_age)


def valid_covariance(values, size):
    """A covariance must be finite, symmetric and positive semidefinite."""
    matrix = np.asarray(values, dtype=float)
    if matrix.size != size * size or not np.isfinite(matrix).all():
        return False
    matrix = matrix.reshape(size, size)
    return bool(np.allclose(matrix, matrix.T, atol=1e-10, rtol=1e-7)
                and np.linalg.eigvalsh(matrix).min() >= -1e-10)


@dataclass(frozen=True)
class HealthPolicy:
    state_age: float = 0.3
    quality_age: float = 1.0
    std_ok: float = 0.5
    std_bad: float = 2.0
    require_rtk: bool = True
    require_filter: bool = True


def evaluate_health(now, pose, covariance, state_stamp, fix, fix_stamp,
                    filter_state, filter_stamp, policy):
    """Return (0 OK / 1 degraded / 2 bad, reasons, horizontal std).

    Only the verified Microstrain MIP enumeration is used: float=5, fixed=6,
    full-navigation filter=4. This table does not apply to NAND radio packets.
    """
    reasons = []
    level = 0
    std = None
    if pose is None or not np.isfinite(pose).all():
        reasons.append("state_missing_or_nonfinite")
        level = 2
    if not fresh(state_stamp, now, policy.state_age):
        reasons.append("state_stamp_missing_stale_or_future")
        level = 2
    if covariance is None or not valid_covariance(covariance, 6):
        reasons.append("invalid_pose_covariance")
        level = 2
    else:
        cov = np.asarray(covariance).reshape(6, 6)
        std = float(np.sqrt(max(0.0, cov[0, 0] + cov[1, 1])))
        if std > policy.std_bad:
            reasons.append("position_uncertainty_bad")
            level = 2
        elif std > policy.std_ok:
            reasons.append("position_uncertainty_degraded")
            level = max(level, 1)
    if policy.require_rtk or fix is not None:
        if not fresh(fix_stamp, now, policy.quality_age):
            reasons.append("fix_unknown_or_stale")
            level = max(level, 1)
        elif fix in (2, 3, 4) or fix not in (0, 1, 5, 6):
            reasons.append("no_valid_position_fix")
            level = 2
        elif policy.require_rtk and fix not in (5, 6):
            reasons.append("fix_not_rtk")
            level = max(level, 1)
    if policy.require_filter or filter_state is not None:
        if not fresh(filter_stamp, now, policy.quality_age):
            reasons.append("filter_unknown_or_stale")
            level = 2
        elif filter_state != 4:
            reasons.append("filter_not_full_navigation")
            level = 2
    return level, reasons, std
