"""
track.py
--------
Arc-length parameterised track model used by the raceline optimizer and the
Frenet local planner.

A Track is a reference line (any polyline in UTM metres) resampled at a fixed
spacing, plus a drivable width to the left and to the right of that line at
every sample. Left is positive, following the unit normal obtained by rotating
the tangent 90 degrees counter-clockwise (same convention as
util/trajectory.py::get_unit_normal_by_index).

Frenet coordinates: s = distance along the reference line, d = signed lateral
offset (left positive). cartesian(s, d) and frenet(x, y) convert both ways.

Boundaries are ordinary waypoint files in the repo's lat/lon JSON format. If a
right boundary file is not available (the course files only map the left
curb), a synthetic right boundary is created a fixed distance to the right of
the reference line.
"""

import json
import uuid

import numpy as np
from scipy.interpolate import CubicSpline
from scipy.spatial import cKDTree

import utm

from util.constants import Constants


class Track:
    def __init__(self, center_xy, left_width, right_width, ds=1.0, smooth=True):
        """
        Args:
            center_xy: (M, 2) polyline in UTM metres, will be resampled at ds
            left_width: float or (N,) array of drivable metres to the left of the
                resampled line (after resampling; pass a float or use from_files)
            right_width: float or (N,) array, same to the right
            ds: sample spacing in metres
            smooth: fit a cubic spline through the waypoints before sampling, so
                curvature is continuous. The hand-clicked course files have 5 m
                median spacing and gaps over 50 m; linear resampling would put a
                curvature spike at every vertex and the planner's hard curvature
                limit would reject everything near a corner.
        """
        self.ds = float(ds)
        pts = np.asarray(center_xy, dtype=float)
        if smooth and len(pts) >= 4:
            self.xy = Track.spline_resample(pts, self.ds)
        else:
            self.xy = Track.resample_polyline(pts, self.ds)
        n = len(self.xy)
        seg = np.linalg.norm(np.diff(self.xy, axis=0), axis=1)
        self.s = np.concatenate([[0.0], np.cumsum(seg)])
        self.length = float(self.s[-1])

        dx = np.gradient(self.xy[:, 0], self.s)
        dy = np.gradient(self.xy[:, 1], self.s)
        norm = np.hypot(dx, dy)
        norm[norm == 0] = 1.0
        self.tangent = np.c_[dx / norm, dy / norm]
        self.normal = np.c_[-self.tangent[:, 1], self.tangent[:, 0]]
        self.heading = np.arctan2(self.tangent[:, 1], self.tangent[:, 0])
        self.curvature = Track._curvature(self.xy, self.s)

        self.w_left = np.broadcast_to(np.asarray(left_width, dtype=float), (n,)).copy()
        self.w_right = np.broadcast_to(np.asarray(right_width, dtype=float), (n,)).copy()

        self._tree = cKDTree(self.xy)

    # ------------------------------------------------------------------ builders
    @classmethod
    def from_files(cls, center_json, left_boundary_json=None, right_boundary_json=None,
                   default_left=3.0, default_right=0.5, margin=0.0, ds=1.0):
        """
        Build a track from waypoint JSON files.

        Widths are measured from the resampled center line to the nearest point
        of each boundary polyline, minus `margin`. Where a boundary file is
        missing the default width is used everywhere.
        """
        center = cls.load_waypoints_utm(center_json)
        track = cls(center, default_left, default_right, ds=ds)

        if left_boundary_json is not None:
            left = cls.load_waypoints_utm(left_boundary_json)
            off = cls.signed_lateral_offset(track.xy, cls.resample_polyline(left, 0.5))
            track.w_left = np.maximum(off - margin, 0.0)
        else:
            track.w_left = np.full(len(track.xy), max(default_left - margin, 0.0))

        if right_boundary_json is not None:
            right = cls.load_waypoints_utm(right_boundary_json)
            off = cls.signed_lateral_offset(track.xy, cls.resample_polyline(right, 0.5))
            track.w_right = np.maximum(-off - margin, 0.0)
        else:
            track.w_right = np.full(len(track.xy), max(default_right - margin, 0.0))

        return track

    # ------------------------------------------------------------------ geometry
    @staticmethod
    def resample_polyline(points, ds):
        """Resample a polyline at (approximately) uniform spacing ds using linear interpolation."""
        points = np.asarray(points, dtype=float)
        if len(points) < 2:
            return points.copy()
        seg = np.linalg.norm(np.diff(points, axis=0), axis=1)
        keep = np.concatenate([[True], seg > 1e-9])
        points = points[keep]
        seg = np.linalg.norm(np.diff(points, axis=0), axis=1)
        s = np.concatenate([[0.0], np.cumsum(seg)])
        n = max(int(np.floor(s[-1] / ds)) + 1, 2)
        ss = np.linspace(0.0, s[-1], n)
        return np.c_[np.interp(ss, s, points[:, 0]), np.interp(ss, s, points[:, 1])]

    @staticmethod
    def spline_resample(points, ds):
        """
        Cubic spline through the waypoints (chord-length parameterised), then
        sampled at uniform arc length. Mirrors what util/trajectory.py does for
        the controller, so the planner and the controller see the same geometry.
        """
        points = np.asarray(points, dtype=float)
        seg = np.linalg.norm(np.diff(points, axis=0), axis=1)
        keep = np.concatenate([[True], seg > 1e-9])
        points = points[keep]
        seg = np.linalg.norm(np.diff(points, axis=0), axis=1)
        u = np.concatenate([[0.0], np.cumsum(seg)])
        spline = CubicSpline(u, points, axis=0)
        # dense evaluation to measure true arc length along the spline
        uu = np.linspace(0.0, u[-1], max(int(u[-1] / 0.1), 10))
        dense = spline(uu)
        arc = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(dense, axis=0), axis=1))])
        n = max(int(np.floor(arc[-1] / ds)) + 1, 2)
        target = np.linspace(0.0, arc[-1], n)
        u_at = np.interp(target, arc, uu)
        return spline(u_at)

    @staticmethod
    def _curvature(xy, s, smooth=5):
        dx = np.gradient(xy[:, 0], s)
        dy = np.gradient(xy[:, 1], s)
        ddx = np.gradient(dx, s)
        ddy = np.gradient(dy, s)
        k = (dx * ddy - dy * ddx) / np.power(dx * dx + dy * dy, 1.5)
        if smooth and len(k) > smooth:
            kernel = np.ones(smooth) / smooth
            k = np.convolve(np.pad(k, (smooth // 2, smooth // 2), mode="edge"), kernel, mode="valid")
        return k

    @staticmethod
    def signed_lateral_offset(ref_xy, other_xy):
        """
        For each point of ref_xy, the signed distance (left positive) from that
        point to the nearest point of other_xy, measured along ref's normal.
        """
        ref_xy = np.asarray(ref_xy, dtype=float)
        tree = cKDTree(np.asarray(other_xy, dtype=float))
        _, idx = tree.query(ref_xy)
        dx = np.gradient(ref_xy[:, 0])
        dy = np.gradient(ref_xy[:, 1])
        normal = np.c_[-dy, dx]
        normal /= np.linalg.norm(normal, axis=1)[:, None]
        return np.sum((np.asarray(other_xy)[idx] - ref_xy) * normal, axis=1)

    # ------------------------------------------------------------------ frenet
    def frenet(self, x, y):
        """
        Project a point onto the track.

        Returns:
            s (float): arc length of the projection
            d (float): signed lateral offset, left positive
        """
        _, i = self._tree.query([x, y])
        p = np.array([x, y], dtype=float)
        best = None
        for a in (i - 1, i):
            if a < 0 or a + 1 >= len(self.xy):
                continue
            p0, p1 = self.xy[a], self.xy[a + 1]
            seg = p1 - p0
            seg_len2 = float(seg @ seg)
            if seg_len2 == 0:
                continue
            t = float(np.clip((p - p0) @ seg / seg_len2, 0.0, 1.0))
            proj = p0 + t * seg
            dist2 = float((p - proj) @ (p - proj))
            if best is None or dist2 < best[0]:
                s = self.s[a] + t * np.sqrt(seg_len2)
                tangent = seg / np.sqrt(seg_len2)
                d = float(-(p - proj)[0] * tangent[1] + (p - proj)[1] * tangent[0])
                best = (dist2, s, d)
        if best is None:
            return float(self.s[i]), 0.0
        return float(best[1]), float(best[2])

    def cartesian(self, s, d=0.0):
        """Map arc length(s) and lateral offset(s) to UTM points. Vectorised."""
        s = np.atleast_1d(np.asarray(s, dtype=float))
        d = np.broadcast_to(np.asarray(d, dtype=float), s.shape)
        s_c = np.clip(s, 0.0, self.length)
        x = np.interp(s_c, self.s, self.xy[:, 0])
        y = np.interp(s_c, self.s, self.xy[:, 1])
        nx = np.interp(s_c, self.s, self.normal[:, 0])
        ny = np.interp(s_c, self.s, self.normal[:, 1])
        nn = np.hypot(nx, ny)
        nn[nn == 0] = 1.0
        return np.c_[x + d * nx / nn, y + d * ny / nn]

    def heading_at(self, s):
        s_c = np.clip(np.asarray(s, dtype=float), 0.0, self.length)
        tx = np.interp(s_c, self.s, self.tangent[:, 0])
        ty = np.interp(s_c, self.s, self.tangent[:, 1])
        return np.arctan2(ty, tx)

    def curvature_at(self, s):
        return np.interp(np.clip(s, 0.0, self.length), self.s, self.curvature)

    def width_at(self, s):
        s_c = np.clip(np.asarray(s, dtype=float), 0.0, self.length)
        return np.interp(s_c, self.s, self.w_left), np.interp(s_c, self.s, self.w_right)

    # ------------------------------------------------------------------ files
    @staticmethod
    def load_waypoints_utm(json_path):
        with open(json_path, "r") as f:
            data = json.load(f)
        pts = []
        for w in data:
            x, y, _, _ = utm.from_latlon(w["lat"], w["lon"])
            pts.append((x, y))
        return np.array(pts, dtype=float)

    @staticmethod
    def save_waypoints_latlon(xy, json_path):
        """Write UTM points as the repo's waypoint JSON (key, lat, lon, active)."""
        out = []
        for x, y in np.asarray(xy, dtype=float):
            lat, lon = utm.to_latlon(x, y, Constants.UTM_ZONE_NUM, Constants.UTM_ZONE_LETTER)
            out.append({"key": str(uuid.uuid4()), "lat": float(lat), "lon": float(lon), "active": False})
        with open(json_path, "w") as f:
            json.dump(out, f, indent=2)
