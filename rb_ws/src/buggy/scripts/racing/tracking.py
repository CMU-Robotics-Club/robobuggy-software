"""Constant-velocity tracking with one-to-one gated assignment and no merging.

Measurements must arrive here in capture-time order. The ROS adapter buffers a
bounded amount of transport latency and rejects older observations explicitly.
Publishing a prediction never changes the measurement count or source age.
"""

from dataclasses import dataclass, field

import numpy as np
from scipy.optimize import linear_sum_assignment

from racing.health import fresh, valid_covariance


@dataclass
class Observation:
    position: np.ndarray
    covariance: np.ndarray
    extent: np.ndarray = field(default_factory=lambda: np.zeros(3))
    extent_known: bool = False
    observed: bool = True
    object_id: str = ""

    def valid(self):
        return (self.observed and np.shape(self.position) == (2,)
                and np.isfinite(self.position).all()
                and valid_covariance(self.covariance, 2)
                and np.shape(self.extent) == (3,) and np.isfinite(self.extent).all()
                and (not self.extent_known or np.all(self.extent > 0)))


def prediction(x, covariance, dt, acceleration_std):
    dt = max(0.0, dt)
    transition = np.eye(4)
    transition[0, 2] = transition[1, 3] = dt
    drive = np.array([[dt * dt / 2, 0], [0, dt * dt / 2], [dt, 0], [0, dt]])
    return transition @ x, transition @ covariance @ transition.T + (
        acceleration_std ** 2 * drive @ drive.T)


@dataclass
class ObjectTrack:
    id: int
    x: np.ndarray
    covariance: np.ndarray
    stamp: float
    last_observed: float
    hits: int = 1
    sources: set = field(default_factory=set)
    identities: dict = field(default_factory=dict)
    extent: np.ndarray = field(default_factory=lambda: np.zeros(3))
    extent_known: bool = False


class MultiObjectTracker:
    def __init__(self, max_age=1.5, confirm_hits=3, gate_chi2=9.21,
                 gate_distance=6.0, acceleration_std=3.0, source_std=None):
        # acceleration_std: buggies on this course turn with v^2/R up to about 5 m/s^2 (13 m/s on a
        # 30 m radius, 9 m/s on 16 m); a constant-velocity model needs process noise of that order
        # or its gate rejects the real object after any gap and spawns a duplicate.
        self.max_age = max_age
        self.confirm_hits = confirm_hits
        self.gate_chi2 = gate_chi2
        self.gate_distance = gate_distance
        self.acceleration_std = acceleration_std
        self.source_std = dict(source_std or {})
        self.tracks = []
        self.next_id = 1
        self.last_stamp = 0.0
        self.source_stamps = {}
        self.counts = dict(repeated=0, invalid=0, late=0, associated=0, created=0)

    def ingest(self, observations, stamp, source, now, max_input_age=0.5):
        if not source or not fresh(stamp, now, max_input_age):
            self.counts["invalid"] += 1
            return False
        if stamp <= self.source_stamps.get(source, 0.0):
            self.counts["repeated"] += 1
            return False
        if stamp < self.last_stamp:
            self.counts["late"] += 1
            return False
        if any(not obs.valid() for obs in observations):
            self.counts["invalid"] += 1
            return False
        self.source_stamps[source] = stamp
        self.last_stamp = stamp
        self.tracks = [track for track in self.tracks
                       if stamp - track.last_observed <= self.max_age]
        for track in self.tracks:
            track.x, track.covariance = prediction(track.x, track.covariance,
                                                  stamp - track.stamp, self.acceleration_std)
            track.stamp = stamp

        noise = float(self.source_std.get(source, 0.0)) ** 2
        covariances = [np.asarray(obs.covariance).reshape(2, 2) + np.eye(2) * (noise + 1e-6)
                       for obs in observations]
        forbidden = 1e12
        costs = np.full((len(self.tracks), len(observations)), forbidden)
        for row, track in enumerate(self.tracks):
            for col, (obs, measurement_cov) in enumerate(zip(observations, covariances)):
                # Identity is only meaningful within a source/generation, never across sensors.
                old_identity = track.identities.get(source)
                if old_identity and obs.object_id and old_identity != obs.object_id:
                    continue
                residual = obs.position - track.x[:2]
                innovation = track.covariance[:2, :2] + measurement_cov
                distance = float(residual @ np.linalg.solve(innovation, residual))
                if distance <= self.gate_chi2 and np.linalg.norm(residual) <= self.gate_distance:
                    costs[row, col] = distance
        matched = set()
        if costs.size:
            rows, cols = linear_sum_assignment(costs)
            for row, col in zip(rows, cols):
                if costs[row, col] == forbidden:
                    continue
                track, obs = self.tracks[row], observations[col]
                measurement_cov = covariances[col]
                gain = np.linalg.solve(track.covariance[:2, :2] + measurement_cov,
                                       track.covariance[:2, :]).T
                track.x += gain @ (obs.position - track.x[:2])
                correction = np.eye(4)
                correction[:, :2] -= gain
                # Joseph form preserves positive semidefiniteness under finite precision.
                track.covariance = (correction @ track.covariance @ correction.T
                                    + gain @ measurement_cov @ gain.T)
                if stamp > track.last_observed:
                    track.hits += 1
                track.last_observed = stamp
                self._metadata(track, obs, source)
                matched.add(col)
                self.counts["associated"] += 1
        for col, obs in enumerate(observations):
            if col in matched:
                continue
            covariance = np.diag([1.0, 1.0, 25.0, 25.0])
            covariance[:2, :2] = covariances[col]
            track = ObjectTrack(self.next_id, np.r_[obs.position, [0.0, 0.0]], covariance,
                                stamp, stamp)
            self._metadata(track, obs, source)
            self.next_id += 1
            self.tracks.append(track)
            self.counts["created"] += 1
        return True

    @staticmethod
    def _metadata(track, observation, source):
        track.sources.add(source)
        if observation.object_id:
            track.identities[source] = observation.object_id
        if observation.extent_known:
            track.extent = np.maximum(track.extent, observation.extent)
            track.extent_known = True

    def snapshot(self, now):
        """Return live tracks at their LAST FILTERED state (no forward extrapolation).

        Straight-line extrapolation in map coordinates is wrong for objects following a
        curved road; consumers that need the object's position at a later time advance it
        along the road using the velocity and the track's age (frenet_planner does this).
        """
        return [(track, track.x, track.covariance) for track in self.tracks
                if 0 <= now - track.last_observed <= self.max_age]

    def suspected_duplicates(self, now, distance=1.5):
        positions = [state[:2] for _, state, _ in self.snapshot(now)]
        return sum(np.linalg.norm(a - b) < distance
                   for index, a in enumerate(positions) for b in positions[index + 1:])
