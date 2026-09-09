"""
record_course.sh with a fake `ros2` on PATH: a failing recorder makes the wrapper
fail, SIGINT is a clean stop that also ends the heartbeat, the topic regex covers
the root-level topics, and the post-record manifest is written.
"""

import os
import re
import shutil
import signal
import subprocess
import time
from pathlib import Path

import pytest

SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "util" / "record_course.sh"
BASH = shutil.which("bash")

pytestmark = pytest.mark.skipif(BASH is None or os.name != "posix", reason="needs bash on a POSIX system")

# Behaviour is chosen by environment variables: FAKE_RECORD_MODE=fail exits 23 at once,
# =run creates the bag directory and sleeps until SIGINT/SIGTERM; FAKE_COUNTS sets the
# message counts that `bag info` reports for the three required topics.
FAKE_ROS2 = '''#!/usr/bin/env python3
import os
import signal
import sys
import time

args = sys.argv[1:]
with open(os.environ["FAKE_ROS2_LOG"], "a") as log:
    print(" ".join(args), file=log)
if args[:2] == ["bag", "record"]:
    if os.environ.get("FAKE_RECORD_MODE", "fail") == "fail":
        print("fake recorder failure: storage error", file=sys.stderr)
        sys.exit(23)

    def stop(signum, _frame):
        print("fake recorder got signal %d, recording stopped" % signum, flush=True)
        sys.exit(0)

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    os.makedirs(args[args.index("-o") + 1], exist_ok=True)
    while True:
        time.sleep(0.1)
elif args[:2] == ["bag", "info"]:
    n_state, n_ekf, n_fix = os.environ.get("FAKE_COUNTS", "42,0,7").split(",")
    print("Files:             fake_0.mcap")
    print("Topic information: Topic: /SC/self/state | Type: nav_msgs/msg/Odometry | Count: %s | Serialization Format: cdr" % n_state)
    print("                   Topic: /ekf/odometry_earth | Type: nav_msgs/msg/Odometry | Count: %s | Serialization Format: cdr" % n_ekf)
    print("                   Topic: /mip/gnss_1/fix_info | Type: microstrain_inertial_msgs/msg/MipGnssFixInfo | Count: %s | Serialization Format: cdr" % n_fix)
'''


@pytest.fixture(name="fake_ros2")
def fake_ros2_fixture(tmp_path):
    """(env with the fake ros2 first on PATH, path of the log of every ros2 call)."""
    bin_dir = tmp_path / "bin"
    bin_dir.mkdir()
    exe = bin_dir / "ros2"
    exe.write_text(FAKE_ROS2)
    exe.chmod(0o755)
    log = tmp_path / "ros2_calls.log"
    env = dict(os.environ, PATH=f"{bin_dir}{os.pathsep}{os.environ['PATH']}", FAKE_ROS2_LOG=str(log))
    return env, log


def run_wrapper(env, out_dir, lidar="true"):
    return subprocess.Popen([BASH, str(SCRIPT), str(out_dir), "test", "300", lidar], env=env,
                            stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)


def wait_for(predicate, timeout=15.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        value = predicate()
        if value:
            return value
        time.sleep(0.1)
    return predicate()


def processes_mentioning(needle):
    """Command lines under /proc (other than ours) containing needle."""
    found = []
    for pid in os.listdir("/proc"):
        if not pid.isdigit() or int(pid) == os.getpid():
            continue
        try:
            cmdline = Path(f"/proc/{pid}/cmdline").read_bytes()
        except OSError:
            continue
        if needle.encode() in cmdline:
            found.append((pid, cmdline))
    return found


def recorded_regex(log):
    record_line = next(line for line in log.read_text().splitlines() if line.startswith("bag record"))
    argv = record_line.split()
    return argv[argv.index("--regex") + 1]


def test_recorder_failure_propagates(fake_ros2, tmp_path):
    env, _ = fake_ros2
    env["FAKE_RECORD_MODE"] = "fail"
    proc = run_wrapper(env, tmp_path / "bags")
    out, err = proc.communicate(timeout=30)
    assert proc.returncode == 23, (out, err)
    assert "fake recorder failure: storage error" in err       # the recorder's own message is visible
    assert "ERROR: ros2 bag record exited with status 23" in err
    assert "WARNING: no bag directory" in out


def test_sigint_is_a_clean_stop_and_the_heartbeat_dies(fake_ros2, tmp_path):
    env, _ = fake_ros2
    env["FAKE_RECORD_MODE"] = "run"
    bags = tmp_path / "bags"
    proc = run_wrapper(env, bags)
    bag_dir = wait_for(lambda: next(bags.glob("test_*"), None))
    assert bag_dir is not None, "fake recorder never created the bag directory"
    proc.send_signal(signal.SIGINT)                              # what `ros2 launch` sends, to the wrapper only
    out, err = proc.communicate(timeout=30)
    assert proc.returncode == 0, (out, err)
    assert "recording stopped" in out                            # the recorder was told and stopped cleanly
    assert "clean stop" in out
    assert "WARNING" not in out
    manifest = (bag_dir / "manifest.txt").read_text()
    assert "## ros2 bag info" in manifest
    assert "topic_regex: " in manifest
    assert "## df -h" in manifest
    assert "## REQUIRED TOPICS" in manifest
    assert "position OK" in manifest
    assert "gnss OK" in manifest
    assert wait_for(lambda: not processes_mentioning(str(bags)), timeout=10), processes_mentioning(str(bags))


def test_missing_required_topics_warn_but_do_not_fail(fake_ros2, tmp_path):
    env, _ = fake_ros2
    env["FAKE_RECORD_MODE"] = "run"
    env["FAKE_COUNTS"] = "0,0,0"
    bags = tmp_path / "bags"
    proc = run_wrapper(env, bags)
    bag_dir = wait_for(lambda: next(bags.glob("test_*"), None))
    assert bag_dir is not None
    proc.send_signal(signal.SIGTERM)
    out, err = proc.communicate(timeout=30)
    assert proc.returncode == 0, (out, err)
    assert "WARNING: /SC/self/state has no messages" in out
    assert "WARNING: neither /ekf/odometry_earth nor /mip/gnss_1/fix_info" in out
    manifest = (bag_dir / "manifest.txt").read_text()
    assert "position MISSING" in manifest
    assert "gnss MISSING" in manifest


@pytest.mark.parametrize("lidar,clouds_recorded", [("true", True), ("false", False)])
def test_topic_regex_coverage(fake_ros2, tmp_path, lidar, clouds_recorded):
    env, log = fake_ros2
    env["FAKE_RECORD_MODE"] = "fail"
    proc = run_wrapper(env, tmp_path / "bags", lidar=lidar)
    proc.communicate(timeout=30)
    regex = recorded_regex(log)
    for topic in ("/tf", "/tf_static", "/clock", "/lidar/obstacle_centroid", "/SC/self/state",
                  "/ekf/odometry_earth", "/mip/gnss_1/fix_info", "/imu/data", "/gnss_1/llh_position"):
        assert re.search(regex, topic), (regex, topic)
    for topic in ("/velodyne_points", "/velodyne_packets", "/lidar/filtered", "/lidar/obstacle_cloud"):
        assert bool(re.search(regex, topic)) is clouds_recorded, (regex, topic)
    for topic in ("/rosout", "/parameter_events", "/tf_other"):
        assert not re.search(regex, topic), (regex, topic)
