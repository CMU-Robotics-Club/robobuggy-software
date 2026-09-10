"""Regressions for missing, stale and malformed localization inputs."""

from types import SimpleNamespace

import numpy as np
import pytest

from racing.health import HealthPolicy, evaluate_health, header_stamp_seconds


def health(**overrides):
    args = dict(now=10.0, pose=[1, 2, 3, 0, 0, 0, 1],
                covariance=np.eye(6).ravel() * 0.01, state_stamp=9.9,
                fix=6, fix_stamp=9.9, filter_state=4, filter_stamp=9.9,
                policy=HealthPolicy())
    args.update(overrides)
    return evaluate_health(**args)


def test_good_fresh_state():
    assert health()[0] == 0


@pytest.mark.parametrize("index,value", [(0, float("nan")), (7, float("inf")), (0, -1)])
def test_invalid_covariance_never_ok(index, value):
    cov = np.eye(6).ravel() * 0.01
    cov[index] = value
    assert health(covariance=cov)[0] == 2


@pytest.mark.parametrize("stamp", [None, 0, 1, 11, float("nan")])
def test_source_stamp_is_required(stamp):
    assert health(state_stamp=stamp)[0] == 2


def test_missing_and_stale_quality_never_ok():
    assert health(fix=None, fix_stamp=None)[0] != 0
    assert health(fix_stamp=1)[0] != 0
    assert health(filter_state=None, filter_stamp=None)[0] == 2
    assert health(filter_stamp=1)[0] == 2


def test_simulation_policy_is_explicit():
    assert health(fix=None, fix_stamp=None, filter_state=None, filter_stamp=None,
                  policy=HealthPolicy(require_rtk=False, require_filter=False))[0] == 0


def _stamp(sec, nanosec):
    return SimpleNamespace(sec=sec, nanosec=nanosec)


def test_header_stamp_reads_plain_and_microstrain_nested_headers():
    plain = SimpleNamespace(header=SimpleNamespace(stamp=_stamp(12, 500_000_000)))
    assert header_stamp_seconds(plain) == pytest.approx(12.5)
    # microstrain_inertial_msgs 4.x: MipHeader{header: std_msgs/Header, event_source, reference_timestamp}
    nested = SimpleNamespace(header=SimpleNamespace(header=SimpleNamespace(stamp=_stamp(7, 0)),
                                                    event_source=0, reference_timestamp=0))
    assert header_stamp_seconds(nested) == pytest.approx(7.0)


def test_header_stamp_is_none_without_a_usable_header():
    assert header_stamp_seconds(SimpleNamespace(fix_type=6)) is None
    assert header_stamp_seconds(SimpleNamespace(header=SimpleNamespace(event_source=0))) is None
