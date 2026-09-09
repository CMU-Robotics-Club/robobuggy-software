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
        of each boundary polyline, minus `margin` (normally half the vehicle
        width plus the hard boundary margin). Where a boundary file is missing
        the default width is an ASSUMED corridor used everywhere; pass None to
        record the width as UNKNOWN (NaN), which planners must never drive
        through. Widths are not clamped: a negative width means the corridor is
        narrower than the vehicle there, and the planner must see that.
        """
        center = cls.load_waypoints_utm(center_json)
        track = cls(center, np.nan, np.nan, ds=ds)

        def measured(boundary_json, sign):
            pts = cls.load_waypoints_utm(boundary_json)
            off = cls.signed_lateral_offset(track.xy, cls.resample_polyline(pts, 0.5))
            return sign * off - margin

        def assumed(default):
            if default is None:
                return np.full(len(track.xy), np.nan)
            return np.full(len(track.xy), float(default) - margin)

        track.w_left = measured(left_boundary_json, 1.0) if left_boundary_json is not None else assumed(default_left)
        track.w_right = measured(right_boundary_json, -1.0) if right_boundary_json is not None else assumed(default_right)
        track.left_source = "boundary_file" if left_boundary_json is not None else ("assumed" if default_left is not None else "unknown")
        track.right_source = "boundary_file" if right_boundary_json is not None else ("assumed" if default_right is not None else "unknown")
        return track

    def width_known(self):
        """True where both drivable widths are known (finite)."""
        return np.isfinite(self.w_left) & np.isfinite(self.w_right)

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
        Project a point (or arrays of points) onto the track.

        Returns:
            s: arc length of the projection (float, or array for array input)
            d: signed lateral offset, left positive
        """
        scalar = np.isscalar(x) and np.isscalar(y)
        px = np.atleast_1d(np.asarray(x, dtype=float))
        py = np.atleast_1d(np.asarray(y, dtype=float))
        pts = np.c_[px, py]
        _, nearest = self._tree.query(pts)
        n_seg = len(self.xy) - 1
        best_dist2 = np.full(len(pts), np.inf)
        best_s = self.s[np.clip(nearest, 0, len(self.s) - 1)].astype(float)
        best_d = np.zeros(len(pts))
        for a in (nearest - 1, nearest):
            valid = (a >= 0) & (a < n_seg)
            idx = np.clip(a, 0, max(n_seg - 1, 0))
            p0 = self.xy[idx]
            seg = self.xy[np.minimum(idx + 1, len(self.xy) - 1)] - p0
            seg_len2 = np.einsum("ij,ij->i", seg, seg)
            ok = valid & (seg_len2 > 0)
            safe_len2 = np.where(ok, seg_len2, 1.0)
            t = np.clip(np.einsum("ij,ij->i", pts - p0, seg) / safe_len2, 0.0, 1.0)
            proj = p0 + t[:, None] * seg
            rel = pts - proj
            dist2 = np.einsum("ij,ij->i", rel, rel)
            better = ok & (dist2 < best_dist2)
            seg_len = np.sqrt(safe_len2)
            tangent = seg / seg_len[:, None]
            s_here = self.s[idx] + t * seg_len
            d_here = -rel[:, 0] * tangent[:, 1] + rel[:, 1] * tangent[:, 0]
            best_dist2 = np.where(better, dist2, best_dist2)
            best_s = np.where(better, s_here, best_s)
            best_d = np.where(better, d_here, best_d)
        if scalar:
            return float(best_s[0]), float(best_d[0])
        return best_s, best_d

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
