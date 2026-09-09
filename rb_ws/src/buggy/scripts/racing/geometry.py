"""The curve the controller actually follows, sampled densely enough to validate it (D1).

`Trajectory(positions=xy)` rebuilds a published path through an Akima interpolant,
re-spaces it by arc length and fits a cubic spline; the Stanley controller reads
headings and curvature from that spline, not from the waypoints. Validation must
therefore run on the same object. This module wraps that class; it does not
re-implement it.
"""

import math
from dataclasses import dataclass

import numpy as np

from util.trajectory import Trajectory


@dataclass(frozen=True)
class CurveSamples:
    distance: np.ndarray    # arc length along the reconstructed curve (m)
    xy: np.ndarray          # (n, 2) positions on the spline
    heading: np.ndarray     # rad
    curvature: np.ndarray   # 1/m, signed (left positive)

    @property
    def max_abs_curvature(self):
        return float(np.max(np.abs(self.curvature))) if len(self.curvature) else 0.0


class ReconstructedCurve:
    """Exactly `Trajectory(positions=xy)` plus dense sampling helpers."""

    MIN_POINTS = 4

    def __init__(self, xy):
        points = np.asarray(xy, dtype=float)
        if points.ndim != 2 or points.shape[1] != 2:
            raise ValueError("path must be an (n, 2) array")
        if len(points) < self.MIN_POINTS:
            raise ValueError(f"path needs at least {self.MIN_POINTS} points")
        if not np.isfinite(points).all():
            raise ValueError("path contains non-finite coordinates")
        self.points = points
        self.trajectory = Trajectory(positions=points)
        self.length = float(self.trajectory.distances[-1])
        if not math.isfinite(self.length) or self.length <= 0.0:
            raise ValueError("path has no length")

    def sample(self, step_m=0.25):
        """Positions, headings and curvature every `step_m` along the spline (ends included)."""
        n = max(2, int(math.ceil(self.length / float(step_m))) + 1)
        distance = np.linspace(0.0, self.length, n)
        index = self.trajectory.get_index_from_distance(distance)
        xy = self.trajectory.interpolation(index)
        d1 = self.trajectory.interpolation(index, nu=1)
        d2 = self.trajectory.interpolation(index, nu=2)
        speed_sq = d1[:, 0] ** 2 + d1[:, 1] ** 2
        speed_sq = np.where(speed_sq > 1e-12, speed_sq, np.nan)
        curvature = (d1[:, 0] * d2[:, 1] - d1[:, 1] * d2[:, 0]) / np.power(speed_sq, 1.5)
        heading = np.arctan2(d1[:, 1], d1[:, 0])
        return CurveSamples(distance=distance, xy=np.asarray(xy, dtype=float),
                            heading=heading, curvature=curvature)

    def curvature_at_index(self, index):
        """Same formula the controller uses (Trajectory.get_curvature_by_index)."""
        return float(np.asarray(self.trajectory.get_curvature_by_index(index)).reshape(-1)[0])
