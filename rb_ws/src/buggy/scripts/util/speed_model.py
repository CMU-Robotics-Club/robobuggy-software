"""
speed_model.py
--------------
Point-mass speed profile for an unpowered buggy along a track.

On pushed hills the speed is whatever the pushers manage (a per-zone constant).
On the freeroll the speed obeys

    v dv/ds = -g dz/ds - g crr - cda_over_m v^2 - c_scrub v^2 kappa^2

i.e. gravity along the slope, rolling resistance, aerodynamic drag, and a tyre
scrub term that grows with curvature. Integrated forward with small steps from
the release speed at the end of the last pushed zone. The output v(s) is used

  * by the raceline optimizer to weight fast sections (scrub ~ v^2 kappa), and
  * by the simulator (speed_model_node.py) to publish sim/velocity so lap time
    in the 2D sim depends on the line, which is the whole point of racing.

Zones and physics constants come from config/course_zones.yaml; see the notes
in that file about how approximate they are.
"""

import os

import numpy as np
import yaml

G = 9.81


class SpeedModel:
    def __init__(self, zones_yaml, track):
        """
        Args:
            zones_yaml: path to course_zones.yaml
            track: util.track.Track (gives s, curvature)
        """
        with open(zones_yaml, "r") as f:
            cfg = yaml.safe_load(f)
        self.zones = sorted(cfg["zones"], key=lambda z: z["s_start"])
        ph = cfg.get("physics", {})
        self.crr = float(ph.get("crr", 0.012))
        self.cda_over_m = float(ph.get("cda_over_m", 0.0035))
        self.c_scrub = float(ph.get("c_scrub", 0.6))
        self.v_min = float(ph.get("v_min_mps", 1.0))
        self.v_max = float(ph.get("v_max_mps", 20.0))
        self.track = track
        # optional measured elevation profile (CSV s_m,z_m from bag_to_course.py). When present it
        # replaces the per-zone dz_m guesses for the slope term.
        self.elev_s = None
        self.elev_z = None
        prof = cfg.get("elevation_profile")
        if prof:
            path = prof if os.path.isabs(prof) else os.path.join(os.path.dirname(os.path.abspath(zones_yaml)), "..", prof.replace("config/", ""))
            path = os.path.normpath(path)
            if os.path.exists(path):
                data = np.genfromtxt(path, delimiter=",", names=True)
                self.elev_s = np.asarray(data["s_m"], dtype=float)
                self.elev_z = np.asarray(data["z_m"], dtype=float)
            else:
                print(f"speed_model: elevation profile {path} not found, using zone dz_m values")
        self.s, self.v = self.integrate(track.s, track.curvature)

    # ------------------------------------------------------------------ zones
    def zone_at(self, s):
        for z in self.zones:
            if z["s_start"] <= s < z["s_end"]:
                return z
        return self.zones[-1]

    def grade_at(self, s):
        """dz/ds at s: from the measured profile if loaded, else constant per zone."""
        if self.elev_s is not None and len(self.elev_s) > 2:
            h = 5.0
            z1 = np.interp(s + h, self.elev_s, self.elev_z)
            z0 = np.interp(s - h, self.elev_s, self.elev_z)
            return float((z1 - z0) / (2 * h))
        z = self.zone_at(s)
        length = max(z["s_end"] - z["s_start"], 1e-6)
        return float(z.get("dz_m", 0.0)) / length

    # ------------------------------------------------------------------ physics
    def integrate(self, s_grid, kappa_grid, ds=0.5):
        """Forward-integrate v(s) on a fine grid; returns (s, v) arrays."""
        s_end = float(s_grid[-1])
        s = np.arange(0.0, s_end + ds, ds)
        kappa = np.interp(s, s_grid, kappa_grid)
        v = np.zeros_like(s)
        v_prev = None
        for i, si in enumerate(s):
            z = self.zone_at(si)
            if z["type"] == "pushed":
                v[i] = float(z.get("push_speed_mps", 5.0))
            else:
                if v_prev is None:
                    v_prev = float(z.get("push_speed_mps", 7.0))
                vv = max(v_prev, self.v_min)
                accel = (-G * self.grade_at(si) - G * self.crr
                         - self.cda_over_m * vv * vv
                         - self.c_scrub * vv * vv * kappa[i] * kappa[i])
                # v dv/ds = accel  ->  dv = accel/v ds
                vv = vv + accel / vv * ds
                v[i] = float(np.clip(vv, self.v_min, self.v_max))
            v_prev = v[i]
        return s, v

    # ------------------------------------------------------------------ queries
    def speed_at(self, s):
        return float(np.interp(s, self.s, self.v))

    def lap_time(self):
        """Seconds to cover the whole track at v(s)."""
        ds = np.diff(self.s)
        v_mid = 0.5 * (self.v[1:] + self.v[:-1])
        return float(np.sum(ds / np.maximum(v_mid, self.v_min)))

    def segment_times(self):
        out = {}
        for z in self.zones:
            m = (self.s >= z["s_start"]) & (self.s < z["s_end"])
            if m.sum() < 2:
                continue
            ds = np.diff(self.s[m])
            v_mid = 0.5 * (self.v[m][1:] + self.v[m][:-1])
            out[z["name"]] = float(np.sum(ds / np.maximum(v_mid, self.v_min)))
        return out


def evaluate_line(xy, zones_yaml, ds=1.0):
    """Convenience: lap time and speed stats for an arbitrary UTM polyline."""
    from util.track import Track
    tr = Track(xy, 1.0, 1.0, ds=ds)
    sm = SpeedModel(zones_yaml, tr)
    return {
        "length_m": tr.length,
        "lap_time_s": sm.lap_time(),
        "segment_times_s": sm.segment_times(),
        "v_max_mps": float(sm.v.max()),
        "v_min_freeroll_mps": float(min(sm.speed_at(s) for s in np.linspace(175, 1000, 200))),
    }
