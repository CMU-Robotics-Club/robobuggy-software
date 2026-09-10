#!/bin/bash
# record_course.sh <output_dir> <pass_label> <max_bag_duration_s> <record_lidar true|false>
#
# Called by launch/record_course.xml. Records the GQ7, buggy, lidar, tf/clock and camera
# topics into one MCAP bag at <output_dir>/<pass_label>_<date>_<time>, split every
# <max_bag_duration_s> seconds. Kept in a script so the date is evaluated here and the
# regex needs no quoting inside the launch file.
#
# Exit status: the recorder's own status is propagated (a recorder that dies with an
# error makes this script fail too), except when the recording was ended on purpose
# with Ctrl-C or a launch shutdown (SIGINT / SIGTERM), which exits 0.
# After the recorder stops, `ros2 bag info` on the bag, `df -h` of the output disk, the
# regex and a REQUIRED TOPICS check are written to <bag_dir>/manifest.txt; a missing
# required topic prints a WARNING but does not fail the run.
out="${1:-${RBROOT:-/rb_ws}/bags}"
label="${2:-center}"
dur="${3:-300}"
lidar="${4:-true}"

mkdir -p "$out"
stamp=$(date +%Y%m%d_%H%M%S)
bag_dir="$out/${label}_${stamp}"
# /tf, /tf_static, /clock and /lidar/* live at the root: buggy_lidar.py publishes
# /lidar/obstacle_centroid, /lidar/obstacle_cloud and /lidar/filtered there.
if [ "$lidar" = "true" ] || [ "$lidar" = "True" ]; then
    re='^/(ekf|gnss_1|gnss_2|imu|mip|nmea|rtcm|SC|velodyne|zed).*|^/lidar/.*|^/(tf|tf_static|clock)$'
else
    # no point clouds (/velodyne*, /lidar/filtered, /lidar/obstacle_cloud) but keep the small
    # centroid topic so the bag still shows what the lidar pipeline detected
    re='^/(ekf|gnss_1|gnss_2|imu|mip|nmea|rtcm|SC|zed).*|^/lidar/obstacle_centroid$|^/(tf|tf_static|clock)$'
fi
echo "recording to $bag_dir  (lidar=$lidar, split every ${dur}s)"
echo "topic regex: $re"

# fix-quality heartbeat every 3 s so the operator sees RTK status while walking
(
    while true; do
        line=$(timeout 3 ros2 topic echo /SC/localization/status --once 2>/dev/null | grep -m1 "data:" || true)
        if [ -n "$line" ]; then echo "[health] ${line#data: }"; else echo "[health] no localization/status yet"; fi
        sleep 3
    done
) &
hb=$!

# The recorder runs in the background so a SIGINT/SIGTERM sent to this script alone (which
# is what `ros2 launch` does on shutdown: it signals only its direct child) is forwarded to
# it. With a foreground child bash would run the trap only after the recorder ended, which
# it never would. ros2 bag record installs its own handler, so the forwarded signal stops it
# cleanly and the MCAP is finalised (checked in the container: exit 0 within a second).
ros2 bag record -s mcap -o "$bag_dir" -d "$dur" --regex "$re" &
rec=$!
stopped_by=""
forward() {
    stopped_by="$1"
    kill -"$1" "$rec" 2>/dev/null
}
trap 'forward INT' INT
trap 'forward TERM' TERM
trap 'kill $hb 2>/dev/null' EXIT

# `wait` returns >128 as soon as a trapped signal arrives; wait again until the recorder has
# really exited so the status we report is its own.
wait "$rec"
status=$?
while [ "$status" -gt 128 ] && kill -0 "$rec" 2>/dev/null; do
    wait "$rec"
    status=$?
done

if [ -n "$stopped_by" ]; then
    echo "recorder stopped on SIG$stopped_by (recorder status $status): clean stop"
    status=0
elif [ "$status" -eq 130 ] || [ "$status" -eq 143 ]; then
    echo "recorder ended on SIGINT/SIGTERM (status $status): clean stop"
    status=0
elif [ "$status" -ne 0 ]; then
    echo "ERROR: ros2 bag record exited with status $status; the bag at $bag_dir may be incomplete" >&2
fi

# ---------------------------------------------------------------- post-record manifest
topic_count() {
    # message count of topic $1 in the `ros2 bag info` output, 0 if absent
    local n
    n=$(printf '%s\n' "$info" | grep -E -m1 "Topic: $1 \|" | grep -oE 'Count: [0-9]+' | grep -oE '[0-9]+')
    echo "${n:-0}"
}

if [ -d "$bag_dir" ]; then
    info=$(ros2 bag info "$bag_dir" 2>&1)
    n_state=$(topic_count /SC/self/state)
    n_ekf=$(topic_count /ekf/odometry_earth)
    n_fix=$(topic_count /mip/gnss_1/fix_info)
    if [ "$n_state" -gt 0 ]; then pos_ok="OK"; else pos_ok="MISSING"; fi
    if [ "$n_ekf" -gt 0 ] || [ "$n_fix" -gt 0 ]; then gnss_ok="OK"; else gnss_ok="MISSING"; fi
    manifest="$bag_dir/manifest.txt"
    {
        echo "# course survey bag manifest, written by record_course.sh"
        echo "bag: $bag_dir"
        echo "label: $label"
        echo "record_lidar: $lidar"
        echo "topic_regex: $re"
        echo "recorder_exit_status: $status"
        echo "stopped_by_signal: ${stopped_by:-none}"
        echo
        echo "## ros2 bag info"
        printf '%s\n' "$info"
        echo
        echo "## df -h $out"
        df -h "$out" 2>&1
        echo
        echo "## REQUIRED TOPICS"
        echo "/SC/self/state: $n_state messages -> position $pos_ok"
        echo "/ekf/odometry_earth: $n_ekf messages, /mip/gnss_1/fix_info: $n_fix messages -> gnss $gnss_ok"
    } > "$manifest"
    echo "manifest written to $manifest"
    if [ "$pos_ok" != "OK" ]; then
        echo "WARNING: /SC/self/state has no messages in $bag_dir; bag_to_course.py will fall back to /ekf/odometry_earth or fail"
    fi
    if [ "$gnss_ok" != "OK" ]; then
        echo "WARNING: neither /ekf/odometry_earth nor /mip/gnss_1/fix_info has messages in $bag_dir; no GNSS data was recorded"
    fi
else
    echo "WARNING: no bag directory at $bag_dir; nothing was recorded"
fi

exit "$status"
