"""Validation samples the very spline the controller evaluates."""

import numpy as np
import pytest

from racing.geometry import ReconstructedCurve
from util.trajectory import Trajectory


def circle(radius, n=80, span=np.pi):
    a = np.linspace(0.0, span, n)
    return np.c_[radius * np.cos(a), radius * np.sin(a)]


def test_reconstruction_is_the_controllers_trajectory():
    xy = circle(20.0)
    curve = ReconstructedCurve(xy)
    reference = Trajectory(positions=xy)
    index = np.linspace(0.0, len(xy) - 1.001, 37)
    assert np.allclose(curve.trajectory.interpolation(index), reference.interpolation(index))
    for i in index[::6]:
        assert curve.curvature_at_index(i) == pytest.approx(reference.get_curvature_by_index(i))


def test_dense_samples_recover_circle_curvature():
    curve = ReconstructedCurve(circle(20.0))
    samples = curve.sample(0.25)
    assert np.all(np.diff(samples.distance) > 0)
    assert samples.distance[-1] == pytest.approx(curve.length)
    interior = samples.curvature[5:-5]
    assert np.allclose(np.abs(interior), 1.0 / 20.0, rtol=0.05)
    assert samples.max_abs_curvature == pytest.approx(0.05, rel=0.1)


def test_step_controls_resolution_and_ends_are_included():
    curve = ReconstructedCurve(circle(20.0))
    coarse = curve.sample(2.0)
    fine = curve.sample(0.1)
    assert len(fine.distance) > 10 * len(coarse.distance)
    assert np.allclose(coarse.xy[0], fine.xy[0])
    assert np.allclose(coarse.xy[-1], fine.xy[-1])


@pytest.mark.parametrize("bad", [
    np.zeros((3, 2)),
    np.array([[0.0, 0.0], [1.0, np.nan], [2.0, 0.0], [3.0, 0.0]]),
    np.array([[0.0, 0.0, 0.0]] * 5),
])
def test_degenerate_paths_are_rejected(bad):
    with pytest.raises(ValueError):
        ReconstructedCurve(bad)
