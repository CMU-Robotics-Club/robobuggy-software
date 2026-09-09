"""
bag_to_course.py: fix freshness, gap segmentation, offsets, manifest and atomic
output. The pure tests need only numpy; the end-to-end tests build a synthetic
MCAP bag and skip when rosbag2 is not importable (run them with ROS sourced).
"""

import hashlib
import json
from pathlib import Path

import numpy as np
import pytest

from util import bag_to_course as btc


# --------------------------------------------------------------------------- fix freshness
def test_stale_fix_never_labels_later_samples():
    # the review's probe: one RTK-fixed status at 1 s, twelve states at 1000..1011 s
    fix = btc.assign_fix_types(np.arange(1000.0, 1012.0), [1.0], [btc.RTK_FIXED], max_age=2.0)
    assert (fix == btc.UNKNOWN_FIX).all()
    assert btc.fix_histogram(fix) == {"unknown": 12, "RTK float": 0, "RTK fixed": 0}


def test_fresh_fix_labels_sample():
    assert btc.assign_fix_types([10.0], [9.5], [btc.RTK_FIXED], 2.0).tolist() == [btc.RTK_FIXED]
    # exactly at the age limit still counts; a fix that arrives after the sample does not
    assert btc.assign_fix_types([10.0], [8.0], [btc.RTK_FIXED], 2.0).tolist() == [btc.RTK_FIXED]
    assert btc.assign_fix_types([10.0], [10.5], [btc.RTK_FIXED], 2.0).tolist() == [btc.UNKNOWN_FIX]
    assert btc.assign_fix_types([10.0], [], [], 2.0).tolist() == [btc.UNKNOWN_FIX]


def test_most_recent_fix_wins_whatever_the_bag_order():
    fix_t, values = [5.0, 3.0, 4.0], [btc.RTK_FIXED, 0, btc.RTK_FLOAT]
    assert btc.assign_fix_types([4.5, 5.5, 3.5], fix_t, values, 2.0).tolist() == [btc.RTK_FLOAT, btc.RTK_FIXED, 0]


def test_require_rtk_drops_unknown_samples():
    t = np.arange(12.0)
    xy = np.c_[t, np.zeros(12)]          # 1 m/s eastbound
    std = np.zeros(12)
    fix = np.full(12, btc.UNKNOWN_FIX)
    fix[3:6] = btc.RTK_FIXED
    fix[6] = btc.RTK_FLOAT
    assert btc.select_samples(t, xy, std, fix, require_rtk=True).tolist() == [False] * 3 + [True] * 3 + [False] * 6
    assert btc.select_samples(t, xy, std, fix, require_rtk=True, allow_float=True).sum() == 4
    # without --require-rtk only the first sample goes (it has no speed yet)
    assert btc.select_samples(t, xy, std, fix).tolist() == [False] + [True] * 11


# --------------------------------------------------------------------------- gaps
def line_with_gap(gap_m, n=20):
    first = np.arange(n, dtype=float)
    second = first[-1] + gap_m + np.arange(n, dtype=float)
    return np.c_[np.concatenate([first, second]), np.zeros(2 * n)]


def test_gap_over_limit_splits_in_two():
    segments, gaps = btc.split_segments(line_with_gap(15.0), 10.0)
    assert segments == [(0, 20), (20, 40)]
    assert [g["after_sample"] for g in gaps] == [19]
    assert gaps[0]["gap_m"] == pytest.approx(15.0)


def test_gap_under_limit_keeps_one_segment():
    segments, gaps = btc.split_segments(line_with_gap(8.0), 10.0)
    assert segments == [(0, 40)]
    assert gaps == []


# --------------------------------------------------------------------------- offsets and smoothing
def test_edge_offset_direction_and_magnitude():
    # eastbound, then curving to the north (counter-clockwise)
    theta = np.linspace(0.0, np.pi / 2, 50)
    xy = np.c_[100.0 * np.sin(theta), 100.0 * (1.0 - np.cos(theta))]
    tangent, normal = btc.path_frames(xy)
    left = btc.apply_offsets(xy, "left", edge_offset=0.3)
    right = btc.apply_offsets(xy, "right", edge_offset=0.3)
    np.testing.assert_allclose(left - xy, 0.3 * normal, atol=1e-9)
    np.testing.assert_allclose(right - xy, -0.3 * normal, atol=1e-9)
    np.testing.assert_allclose(np.linalg.norm(left - xy, axis=1), 0.3)
    np.testing.assert_allclose(np.linalg.norm(right - xy, axis=1), 0.3)
    # heading east, left is +y
    assert left[0, 1] > xy[0, 1] > right[0, 1]
    # the centre line gets no lateral shift; the lever arm moves along the direction of travel
    np.testing.assert_allclose(btc.apply_offsets(xy, "centerline", edge_offset=0.3), xy)
    np.testing.assert_allclose(btc.apply_offsets(xy, "left", lever_arm=0.5) - xy, 0.5 * tangent, atol=1e-9)


def test_smoothing_window_is_optional():
    rng = np.random.default_rng(0)
    xy = np.c_[np.arange(0.0, 40.0), rng.normal(0.0, 0.1, 40)]
    raw, keep = btc.thin_and_smooth(xy, 1.0, smooth=1)
    np.testing.assert_allclose(raw, xy[keep])
    smoothed, _ = btc.thin_and_smooth(xy, 1.0, smooth=5)
    assert np.std(smoothed[1:-1, 1]) < np.std(raw[1:-1, 1])
    with pytest.raises(ValueError):
        btc.thin_and_smooth(xy, 1.0, smooth=4)


def test_elevation_profile_bins_median_altitude():
    s = np.arange(0.0, 100.0)
    centers, medians = btc.elevation_profile(s, 0.1 * s, 100.0)
    np.testing.assert_allclose(centers, np.arange(2.5, 100.0, 5.0))
    np.testing.assert_allclose(medians, 0.1 * (centers - 0.5))


# --------------------------------------------------------------------------- manifest and atomic output
MANIFEST_KEYS = ("bag", "topics", "samples_read", "samples_kept", "fix_histogram", "largest_gap_m", "gaps",
                 "segments", "offsets", "reference", "arguments", "calibration_unknown", "timestamp")


def make_manifest(tmp_path, edge_offset=0.0, lever_arm=0.0):
    ref = tmp_path / "ref.json"
    ref.write_text("[]")
    return btc.build_manifest("bags/left_x", "left", {"position": "/SC/self/state", "fix": "/mip/gnss_1/fix_info"},
                              100, 80, btc.fix_histogram([6, 6, -1]), [{"after_sample": 3, "gap_m": 12.0}],
                              [{"index": 1, "file": "a.json"}], edge_offset, lever_arm,
                              btc.reference_record(str(ref)), {"bag": "bags/left_x"}, ["a.json"])


def test_manifest_content(tmp_path):
    manifest = make_manifest(tmp_path)
    for key in MANIFEST_KEYS:
        assert key in manifest, key
    assert manifest["fix_histogram"] == {"unknown": 1, "RTK float": 0, "RTK fixed": 2}
    assert manifest["largest_gap_m"] == 12.0
    assert manifest["calibration_unknown"] is True
    assert "NOT the curb" in manifest["line_is"]
    assert manifest["reference"]["name"] == "ref.json"
    assert manifest["reference"]["sha256"] == hashlib.sha256(b"[]").hexdigest()
    assert json.loads(json.dumps(manifest)) == manifest
    calibrated = make_manifest(tmp_path, edge_offset=0.3)
    assert calibrated["calibration_unknown"] is False
    assert calibrated["offsets"] == {"edge_offset_m": 0.3, "lever_arm_m": 0.0, "lateral_sign": 1.0}


def test_failed_run_leaves_no_files(tmp_path):
    good = tmp_path / "out.json"
    bad = tmp_path / "out.manifest.json"

    def write_ok(path):
        Path(path).write_text("{}")

    def write_then_fail(path):
        Path(path).write_text("partial")
        raise RuntimeError("check failed after the temporary file was written")

    with pytest.raises(RuntimeError):
        btc.write_atomically([(str(good), write_ok), (str(bad), write_then_fail)])
    assert not good.exists()
    assert not bad.exists()
    assert list(tmp_path.iterdir()) == []      # no temporaries left behind either


def test_successful_run_replaces_files(tmp_path):
    good = tmp_path / "out.json"
    good.write_text("old")
    written = btc.write_atomically([(str(good), lambda path: Path(path).write_text("new"))])
    assert written == [str(good)]
    assert good.read_text() == "new"
    assert list(tmp_path.iterdir()) == [good]


# --------------------------------------------------------------------------- end to end on a synthetic bag
STATE_TOPIC = "/SC/self/state"
FIX_TOPIC = "/mip/gnss_1/fix_info"
X0, Y0 = 589000.0, 4477000.0          # UTM 17T, Pittsburgh


def write_synthetic_bag(path, states, fixes):
    """states: (t, x, y, z) tuples; fixes: (t, fix_type) tuples; written in time order as MCAP."""
    rosbag2_py = pytest.importorskip("rosbag2_py")
    fix_msgs = pytest.importorskip("microstrain_inertial_msgs.msg")
    from nav_msgs.msg import Odometry
    from rclpy.serialization import serialize_message

    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(path), storage_id="mcap"), rosbag2_py.ConverterOptions("", ""))
    writer.create_topic(rosbag2_py.TopicMetadata(name=STATE_TOPIC, type="nav_msgs/msg/Odometry",
                                                 serialization_format="cdr"))
    writer.create_topic(rosbag2_py.TopicMetadata(name=FIX_TOPIC, type="microstrain_inertial_msgs/msg/MipGnssFixInfo",
                                                 serialization_format="cdr"))
    events = [(t, STATE_TOPIC, (x, y, z)) for t, x, y, z in states] + [(t, FIX_TOPIC, f) for t, f in fixes]
    for t, topic, payload in sorted(events, key=lambda e: e[0]):
        if topic == STATE_TOPIC:
            msg = Odometry()
            msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z = payload
            cov = [0.0] * 36
            cov[0] = cov[7] = 0.01
            msg.pose.covariance = cov
        else:
            msg = fix_msgs.MipGnssFixInfo()
            msg.fix_type = payload
        writer.write(topic, serialize_message(msg), int(round(t * 1e9)))
    del writer


def eastbound_states(count, t0=100.0, x0=X0, gap_after=None, gap_m=0.0):
    """One sample per second, 1 m apart heading east; optionally a gap_m jump after sample gap_after."""
    states = []
    x = x0
    for i in range(count):
        if gap_after is not None and i == gap_after + 1:
            x += gap_m - 1.0
        states.append((t0 + i, x, Y0, 250.0))
        x += 1.0
    return states


def run_tool(bag, out, *extra):
    ref = Path(out).parent / "reference.json"
    ref.write_text("[]")
    btc.main(["--bag", str(bag), "--kind", "left", "--out", str(out), "--require-rtk",
              "--reference", str(ref), *extra])
    return hashlib.sha256(b"[]").hexdigest()


def test_tool_ignores_a_stale_fix(tmp_path, capsys):
    bag = tmp_path / "stale_bag"
    write_synthetic_bag(bag, eastbound_states(12, t0=1000.0), [(1.0, btc.RTK_FIXED)])
    out = tmp_path / "left.json"
    with pytest.raises(SystemExit) as excinfo:
        run_tool(bag, out)
    assert "too few samples" in str(excinfo.value)
    printed = capsys.readouterr().out
    assert "unknown=12" in printed
    assert "RTK fixed=0" in printed
    assert not out.exists()
    assert not (tmp_path / "left.manifest.json").exists()


def test_tool_writes_offset_line_and_manifest(tmp_path):
    from util.track import Track

    bag = tmp_path / "good_bag"
    states = eastbound_states(40)
    write_synthetic_bag(bag, states, [(t - 0.5, btc.RTK_FIXED) for t, *_ in states])
    out = tmp_path / "left.json"
    ref_hash = run_tool(bag, out, "--edge-offset-m", "0.3")
    manifest = json.loads((tmp_path / "left.manifest.json").read_text())
    assert manifest["fix_histogram"] == {"unknown": 0, "RTK float": 0, "RTK fixed": 40}
    assert manifest["samples_read"] == 40
    assert manifest["samples_kept"] == 39
    assert manifest["calibration_unknown"] is False
    assert manifest["topics"] == {"position": STATE_TOPIC, "fix": FIX_TOPIC}
    assert manifest["reference"]["sha256"] == ref_hash
    assert manifest["gaps"] == []
    assert [seg["file"] for seg in manifest["segments"]] == [str(out)]
    assert manifest["arguments"]["edge_offset_m"] == 0.3
    line = Track.load_waypoints_utm(str(out))
    assert len(line) >= 30
    # eastbound travel: the left edge is 0.3 m north of the driven line
    np.testing.assert_allclose(line[:, 1], Y0 + 0.3, atol=0.01)


def test_tool_splits_on_gap_and_fail_on_gap_writes_nothing(tmp_path):
    bag = tmp_path / "gap_bag"
    states = eastbound_states(40, gap_after=19, gap_m=15.0)
    write_synthetic_bag(bag, states, [(t - 0.5, btc.RTK_FIXED) for t, *_ in states])
    split_dir = tmp_path / "split"
    split_dir.mkdir()
    run_tool(bag, split_dir / "left.json")
    manifest = json.loads((split_dir / "left.manifest.json").read_text())
    assert len(manifest["gaps"]) == 1
    assert manifest["gaps"][0]["gap_m"] == pytest.approx(15.0)
    assert manifest["largest_gap_m"] == pytest.approx(15.0)
    assert [seg["file"] for seg in manifest["segments"]] == [str(split_dir / "left_seg1.json"),
                                                            str(split_dir / "left_seg2.json")]
    assert (split_dir / "left_seg1.json").exists()
    assert (split_dir / "left_seg2.json").exists()
    assert not (split_dir / "left.json").exists()
    assert manifest["calibration_unknown"] is True

    strict_dir = tmp_path / "strict"
    strict_dir.mkdir()
    with pytest.raises(SystemExit) as excinfo:
        run_tool(bag, strict_dir / "left.json", "--fail-on-gap")
    assert excinfo.value.code
    assert "nothing written" in str(excinfo.value)
    assert sorted(p.name for p in strict_dir.iterdir()) == ["reference.json"]
