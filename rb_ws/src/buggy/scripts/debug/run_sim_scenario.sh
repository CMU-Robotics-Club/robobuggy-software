#!/bin/bash
# run_sim_scenario.sh : start a simulator launch, run a metrics script against it, stop everything.
#
#   run_sim_scenario.sh --launch sim_2d_double.xml --config src/buggy/config/scenarios/double_pass.yaml \
#                       --metrics pass_metrics.py --duration 100 --out /tmp/double_pass.json \
#                       [--warmup 8] [--domain 170] [-- extra launch args, e.g. use_perception_sim:=true]
#
# Exit status is the metrics script's exit status (non-zero on collision, missing input, or
# whatever the script defines), so this can gate a CI-style run. The launch is started in its
# own process group on its own ROS domain and killed at the end, so two scenarios never share
# topics (the classic silent metric corruption).
set -u

LAUNCH=""; CONFIG=""; METRICS=""; DURATION="60"; OUT=""; WARMUP="8"; DOMAIN="170"; BAG=""
while [ $# -gt 0 ]; do
    case "$1" in
        --launch) LAUNCH="$2"; shift 2 ;;
        --config) CONFIG="$2"; shift 2 ;;
        --metrics) METRICS="$2"; shift 2 ;;
        --bag) BAG="$2"; shift 2 ;;      # record the diagnostic topics to this directory (scenario_timeline.py reads it)
        --duration) DURATION="$2"; case "$DURATION" in *.*) ;; *) DURATION="$DURATION.0" ;; esac; shift 2 ;;
        --out) OUT="$2"; shift 2 ;;
        --warmup) WARMUP="$2"; shift 2 ;;
        --domain) DOMAIN="$2"; shift 2 ;;
        --) shift; break ;;
        *) echo "unknown argument $1" >&2; exit 2 ;;
    esac
done
if [ -z "$LAUNCH" ] || [ -z "$CONFIG" ] || [ -z "$METRICS" ] || [ -z "$OUT" ]; then
    echo "usage: $0 --launch <file.xml> --config <params.yaml> --metrics <script.py> --duration <s> --out <json> [-- launch args]" >&2
    exit 2
fi

export ROS_DOMAIN_ID="$DOMAIN"
root="${RBROOT:-/rb_ws}"
cd "$root" || exit 2
log="${OUT%.json}.launch.log"
echo "scenario: $LAUNCH $CONFIG $* (domain $DOMAIN, ${DURATION}s) -> $OUT"

setsid ros2 launch buggy "$LAUNCH" "config_file:=$CONFIG" "$@" > "$log" 2>&1 &
lpid=$!
cleanup() {
    kill -INT -- "-$lpid" 2>/dev/null
    for _ in 1 2 3 4 5 6; do
        kill -0 "$lpid" 2>/dev/null || break
        sleep 1
    done
    kill -KILL -- "-$lpid" 2>/dev/null
}
trap cleanup EXIT INT TERM

sleep "$WARMUP"
if ! kill -0 "$lpid" 2>/dev/null; then
    echo "launch died during warm-up; see $log" >&2
    tail -n 30 "$log" >&2
    exit 3
fi
bpid=""
if [ -n "$BAG" ]; then
    rm -rf "$BAG"
    setsid ros2 bag record -s mcap -o "$BAG" \
        /SC/self/state /NAND/self/state /SC/input/steering /SC/controller/controller/debug/cross_track_error \
        /SC/planning/status /SC/controller/plan_source /SC/debug/planner/target_offset /SC/debug/planner/state \
        /SC/debug/planner/num_opponents \
        /SC/perception/tracking /SC/debug/tracker/status /SC/perception_sim/ghost_truth /SC/localization/status \
        > "${OUT%.json}.bag.log" 2>&1 &
    bpid=$!
    trap 'kill -INT -- "-$bpid" 2>/dev/null; sleep 2; cleanup' EXIT INT TERM
fi
# extra metrics parameters, e.g. METRICS_ARGS="-p require_pass:=true -p expect_pass_side:=right"
# shellcheck disable=SC2086
ros2 run buggy "$METRICS" --ros-args -r __ns:=/SC -p "duration:=$DURATION" -p "out:=$OUT" ${METRICS_ARGS:-}
rc=$?
echo "metrics exit $rc"
exit $rc
