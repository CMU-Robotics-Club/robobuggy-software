"""Identity preservation, capture-time freshness and genuine observation counts."""

import numpy as np

from racing.tracking import MultiObjectTracker, Observation


def observation(x, y=0, std=0.1, identity=""):
    return Observation(np.array([x, y], dtype=float), np.eye(2) * std ** 2,
                       object_id=identity)


def test_same_frame_close_opponents_are_never_suppressed_or_merged():
    tracker = MultiObjectTracker()
    for stamp in [10, 10.1, 10.2]:
        assert tracker.ingest([observation(0), observation(1.4)], stamp, "lidar", stamp)
    assert len(tracker.snapshot(10.2)) == 2
    assert all(track.hits == 3 for track in tracker.tracks)


def test_close_established_cross_source_tracks_are_not_merged():
    tracker = MultiObjectTracker()
    tracker.ingest([observation(0), observation(1.4)], 10, "lidar", 10)
    tracker.ingest([observation(0.25, std=0.05)], 10.1, "camera", 10.1)
    tracker.ingest([observation(1.15, std=0.05)], 10.2, "lidar", 10.2)
    assert {track.id for track in tracker.tracks} == {1, 2}
    assert len(tracker.snapshot(10.2)) == 2


def test_calibrated_cross_sensor_bias_can_associate_one_object():
    tracker = MultiObjectTracker(source_std={"camera": 0.3, "lidar": 0.15})
    tracker.ingest([observation(0)], 10, "lidar", 10)
    tracker.ingest([observation(0.5)], 10.1, "camera", 10.1)
    assert len(tracker.tracks) == 1


def test_repeated_or_predicted_values_never_confirm_track_or_refresh_age():
    tracker = MultiObjectTracker()
    for _ in range(3):
        tracker.ingest([observation(0)], 10, "lidar", 10.1)
    assert tracker.tracks[0].hits == 1
    tracker.snapshot(10.4)
    assert tracker.tracks[0].last_observed == 10
    assert not tracker.snapshot(12)
    predicted = observation(1)
    predicted.observed = False
    assert not tracker.ingest([predicted], 10.2, "lidar", 10.2)


def test_late_nonfinite_and_invalid_covariance_are_rejected():
    tracker = MultiObjectTracker()
    tracker.ingest([observation(0)], 10, "lidar", 10)
    assert not tracker.ingest([observation(1)], 9.9, "camera", 10)
    assert not tracker.ingest([observation(float("nan"))], 10.1, "lidar", 10.1)
    obs = observation(0)
    obs.covariance[0, 0] = -1
    assert not tracker.ingest([obs], 10.1, "lidar", 10.1)


def test_assignment_is_global_and_one_to_one():
    tracker = MultiObjectTracker(gate_distance=1.0, gate_chi2=100)
    tracker.ingest([observation(0), observation(1)], 10, "lidar", 10)
    # A greedy 0->0.4 assignment strands -0.5; global matching finds both.
    tracker.ingest([observation(0.4), observation(-0.5)], 10.1, "lidar", 10.1)
    assert len(tracker.tracks) == 2
    assert all(track.hits == 2 for track in tracker.tracks)


def test_empty_observed_frames_are_valid_coverage_not_fake_tracks():
    tracker = MultiObjectTracker()
    assert tracker.ingest([], 10, "lidar", 10)
    assert tracker.source_stamps["lidar"] == 10
    assert not tracker.snapshot(10)
