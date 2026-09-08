#!/bin/bash
# record_course.sh <output_dir> <pass_label> <max_bag_duration_s> <record_lidar true|false>
#
# Called by launch/record_course.xml. Records the GQ7, buggy, lidar and camera topics
# into one MCAP bag at <output_dir>/<pass_label>_<date>_<time>, split every
# <max_bag_duration_s> seconds. Kept in a script so the date is evaluated here and
# the regex needs no quoting inside the launch file.
out="${1:-${RBROOT:-/rb_ws}/bags}"
label="${2:-center}"
dur="${3:-300}"
lidar="${4:-true}"

mkdir -p "$out"
stamp=$(date +%Y%m%d_%H%M%S)
if [ "$lidar" = "true" ] || [ "$lidar" = "True" ]; then
    re='^/(ekf|gnss_1|gnss_2|imu|mip|nmea|rtcm|SC|velodyne|zed).*'
else
    re='^/(ekf|gnss_1|gnss_2|imu|mip|nmea|rtcm|SC|zed).*'
fi
echo "recording to $out/${label}_${stamp}  (lidar=$lidar, split every ${dur}s)"

# fix-quality heartbeat every 3 s so the operator sees RTK status while walking
(
    while true; do
        line=$(timeout 3 ros2 topic echo /SC/localization/status --once 2>/dev/null | grep -m1 "data:" || true)
        if [ -n "$line" ]; then echo "[health] ${line#data: }"; else echo "[health] no localization/status yet"; fi
        sleep 3
    done
) &
hb=$!
# Ctrl-C (or launch shutdown) stops the recorder; we then stop the heartbeat and exit cleanly
trap 'kill $hb 2>/dev/null; exit 0' INT TERM
trap 'kill $hb 2>/dev/null' EXIT

ros2 bag record -s mcap -o "$out/${label}_${stamp}" -d "$dur" --regex "$re" || true
exit 0
