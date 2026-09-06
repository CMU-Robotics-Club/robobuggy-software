# Racing stack: perception, localization and path planning upgrades

Branch `feature/racing-perception-planning`, based on `rolls` (Sept 2026).
This document explains what was researched, what was built, how it was tested
in the 2D simulator, and what still needs hardware. Raw research output with
every source URL is in `research_raw.json` next to the repo checkout.

## 1. The problem, stated honestly

A buggy is a gravity vehicle. Software commands steering only: no throttle, no
brakes. So "keep speed up" has exactly one lever in code, curvature. Every
metre of the line that turns more than it has to scrubs speed, and the chute
is where that speed is lost or kept. Overtaking helps only when NAND is slower
and only when a pass can be completed without contact (the passing buggy bears
full responsibility; contact is a disqualification). Knowing our own position to
centimetres is what allows a line close to the curb; knowing NAND's position
early is what allows a smooth, early pass instead of a late swerve.

The research (five parallel literature and code sweeps, synthesized and then
attacked by a skeptic) converged on a short list:

| Area | Choice | Why this and not the alternatives |
| --- | --- | --- |
| Global line | Offline minimum-curvature raceline (Heilmeier et al. 2019, TUM), optionally weighted by a gravity speed model | Standard in F1TENTH, Roborace and the Indy Autonomous Challenge; convex, fast, needs only boundaries. Min-time optimal control is wrong here because it assumes brakes. |
| Local planning | Arc-length Frenet lateral sampler d(s) (Werling et al. 2010, low-speed form) with hard curvature and clearance limits and a small behaviour state machine | Spatial planning keeps path shape independent of the speed estimate; MPCC and progress-maximising MPC degenerate without throttle or brakes. |
| Opponent state | One UKF fed by radio, camera and lidar with per-source covariance and chi-square gating; prediction along the track at measured speed | ForzaETH, TUM and PoliMOVE all use a single tracker plus constant-velocity-along-track prediction; learned lidar detectors need labelled data the team does not have. |
| Own position | RTK over NTRIP on the existing Microstrain GQ7 first (configuration only), a localization health signal that scales planner margins, lidar odometry (KISS-ICP) as the later fallback | SC runs single-point GNSS today (1 to 2.5 m). RTK fixed is 1 to 6 cm with no new hardware. GNSS under the Schenley trees is the team's own historical failure mode. |
| Perception | Geometric lidar pipeline (ground removal, clustering, tracking) for range; camera for class confirmation | One known opponent on a fixed course; geometric detection plus a track prior already works at this scale. |

## 2. What was built

New and changed files, all under `rb_ws/src/buggy/`:

| File | Role |
| --- | --- |
| `scripts/util/track.py` | Arc-length track model: Frenet conversions, left/right widths from boundary files, curvature. |
| `scripts/path_planner/raceline_optimizer.py` | Offline minimum-curvature line. Bounded least squares on second differences, re-linearised 3 times. `--zones` weights fast sections using the speed model. Writes the repo's lat/lon waypoint JSON. |
| `scripts/path_planner/frenet_planner.py` | Online local planner replacing the sigmoid rule. Samples lateral targets on a 0.25 m grid with 15/25/40 m transitions, rejects candidates that leave the road, exceed `kappa_max` (0.25 1/m, the Stanley steering clip), or cut inside the opponent; passes on the left only; states RACELINE, PASS, REJOIN with hysteresis; refuses new passes in no-pass zones or when localization health is degraded; holds the raceline when health is bad. Publishes `self/cur_traj` exactly like the old planner. |
| `scripts/path_planner/path_planner.py` | Unchanged legacy sigmoid planner, selectable with `use_frenet:=false`. |
| `scripts/estimation/nand_estimator.py` | Now fuses `other/stateNoUKF` (radio), `vision/other/state` (camera) and `lidar/other/state` (lidar) through one gated UKF update; publishes per-source health on `debug/opponent_sources`; stamps `other/state`. |
| `scripts/estimation/localization_monitor.py` | Publishes `localization/health` (0 ok, 1 degraded, 2 bad) from state freshness, position covariance, GQ7 fix type and filter state. |
| `scripts/perception/lidar_opponent_node.py` | Converts the lidar branch's `/lidar/obstacle_centroid` into a UTM detection with range-dependent covariance. |
| `scripts/util/speed_model.py`, `scripts/simulator/speed_model_node.py`, `config/course_zones.yaml` | Point-mass gravity model v(s): pushed hills at pusher speed, freeroll integrating slope, rolling resistance, drag and tyre scrub. Drives `sim/velocity` so lap time in the sim depends on the line. Zone table also carries the no-pass bands. |
| `scripts/simulator/perception_sim.py` | Fake lidar and camera detections of NAND from ground truth with noise, dropouts, outliers and latency, so the fused estimator runs in the sim. |
| `scripts/debug/sim_metrics.py`, `scripts/debug/pass_metrics.py` | Regression harnesses: tracking quality for a line; pass success, minimum separation, lateral gap and planner state times for the double sim. |
| `scripts/controller/controller_node.py` | Fixed the start-up covariance check, which squared variances (passed a 0.99 m² variance, failed a healthy non-RTK GQ7). Now `sqrt(var_x + var_y) < maxInitPositionStd`. |
| `scripts/simulator/engine.py` | Added start pose `Hill1_SC_BEHIND` (25 m behind Hill 1) for pass scenarios. |
| `launch/sim_2d_double.xml`, `launch/sim_2d_single.xml`, `launch/sc-main.xml`, `launch/sc-system.xml` | Toggles `use_frenet`, `use_speed_model`, `use_perception_sim`, `use_lidar`; localization monitor added everywhere. |
| `config/sim_double.yaml`, `config/sc-roll.yaml` | Planner parameters under `SC_path_planner`. |

## 3. How to run

Everything below runs inside the container, in `/rb_ws` after the normal shell start.

Generate a raceline (about 90 s):

```bash
python3 src/buggy/scripts/path_planner/raceline_optimizer.py \
  --center buggycourse_sc.json --left-boundary buggycourse_curb.json \
  --right-width 0.5 --margin 0.6 --vehicle-width 1.2 \
  --zones src/buggy/config/course_zones.yaml \
  --out buggycourse_sc_raceline.json
```

Single buggy with the gravity speed model, then measure it:

```bash
ros2 launch buggy sim_2d_single.xml use_speed_model:=true
ros2 run buggy sim_metrics.py --ros-args -r __ns:=/SC -p duration:=90.0
```

Two buggies with the Frenet planner and fake perception, then score the pass:

```bash
ros2 launch buggy sim_2d_double.xml use_frenet:=true use_perception_sim:=true
ros2 run buggy pass_metrics.py --ros-args -r __ns:=/SC -p duration:=120.0
```

To compare against the old planner run the same with `use_frenet:=false`.

## 4. What the simulator showed

Constant 12 m/s from Hill 1 for 90 s, Stanley following each line (`sim_metrics.py`):

| Line | Cross-track RMS | Steering RMS | Steering max | Steering rate RMS | Heading change per m |
| --- | --- | --- | --- | --- | --- |
| `buggycourse_sc.json` (team line) | 0.026 m | 0.76 deg | 3.64 deg | 26.7 deg/s | 0.0081 |
| `buggycourse_sc_raceline.json` | 0.023 m | 0.71 deg | 3.43 deg | 23.8 deg/s | 0.0076 |

The optimized line steers 6 to 11 percent less. The gain is small for a
data reason, not an algorithm reason: the repo only has a left curb file, so the
optimizer was allowed 0.5 m to the right of the current line. With a surveyed
right boundary the same tool can use the full road width. The gravity speed
model rates the current line at about 169 s and the optimized line 0.2 s
faster, with the approximate zone table in `course_zones.yaml`.

Pass scenario (`pass_metrics.py`): SC starts 25 m behind NAND on Hill 1
(`Hill1_SC_BEHIND`), SC 13 m/s, NAND 10 m/s, constant speeds, radio relay
dropping about 95 percent of NAND's messages, 110 s:

| Planner | Pass completed at | Closest approach | Lateral gap when alongside | SC tracking error RMS / max | Contact |
| --- | --- | --- | --- | --- | --- |
| Legacy sigmoid (`use_frenet:=false`) | 12.7 s | 2.07 m | 1.74 m | 0.316 m / 2.645 m | no |
| Frenet planner (`use_frenet:=true`) | 9.7 s | 2.89 m | 2.78 m | 0.045 m / 0.901 m | no |

The Frenet planner completes the pass 3 s earlier with 0.8 m more room, and the
controller tracks its path seven times more accurately, because the sigmoid
planner republishes a line that jumps every cycle while the Frenet planner
starts each plan from the offset it already committed to. Planner states over
the run: RACELINE 95 s, PASS 3.8 s, REJOIN 11.1 s. In both runs the SC
controller exits with "Ran out of path to follow" at about 106 s: at 13 m/s SC
reaches the end of the 1374 m course inside the window. That is the existing
end-of-course behaviour, not a planner fault.

## 5. Parameters worth knowing

| Parameter | Where | Value | Meaning |
| --- | --- | --- | --- |
| `kappa_max` | frenet planner | 0.25 1/m | Hard cap; Stanley saturates at tan(20 deg)/wheelbase = 0.33. Recompute when the wheelbase is measured. |
| `a_lat_max`, `w_a_lat` | frenet planner | 4.0 m/s², 30 | Soft penalty on lateral acceleration at current speed. |
| `lateral_clearance`, `longitudinal_window` | frenet planner | 1.6 m, 6 m | Centre-to-centre gap required while alongside. |
| `w_curvature` | frenet planner | 400 | Dominant cost: curvature is the only thing that costs speed. |
| `right_width` | frenet planner, optimizer | 0.5 m | Placeholder until a right boundary is surveyed. |
| `gate_chi2`, `gate_warmup_updates` | nand estimator | 9.21, 5 | 99 percent innovation gate after 5 accepted updates. |
| `radio_accuracy_mm` | nand estimator | 50 | Only true if NAND's receiver is RTK fixed; use about 1500 otherwise. |
| `pos_std_ok_m`, `pos_std_bad_m` | localization monitor | 0.5 m, 2.0 m | Health thresholds; set `require_rtk_for_ok` once RTK is wired. |
| `no_pass_zones` | course_zones.yaml | chute 780 to 960 m, two crossings | Approximate; re-survey. |

## 6. What needs hardware, in order

1. Survey the right edge of the legal course with the GQ7 in RTK-fixed mode
   and store it as `paths/buggycourse_right_bound.json`. Re-record the left
   curb the same way. Then rerun the raceline optimizer with `--right-boundary`.
2. Enable RTK on the GQ7 in `INS_params.yml`: `ntrip_interface_enable: true`,
   `aux_port` on the second USB CDC port pinned by a udev rule, caster host,
   mountpoint and credentials, GLONASS and BeiDou enabled, GGA output for VRS
   casters, antenna lever arms measured to within 5 cm. Verify "RTK Fixed" in
   `localization/status`, then set `require_rtk_for_ok: true`.
3. Fit the speed model from one logged roll: coast-down on a straight gives
   rolling resistance and drag; speed lost through the chute gives the scrub
   coefficient. Replace the zone elevations with the surveyed profile.
4. Measure the wheelbase and steering rate limit; `constants.py` values are
   still marked as guesses. Recompute `kappa_max`.
5. Lidar: run the Velodyne at 600 rpm with per-point time, merge one frame,
   deskew with the 100 Hz INS, replace single-plane RANSAC with a zone-wise
   ground segmenter, and publish through `lidar_opponent_node.py` with
   `use_lidar:=true`. The current lidar branch settings (300 rpm, 4 merged
   frames) smear a target by 8 to 13 m at race speed.
6. Camera: export the YOLO weights to TensorRT on the Orin, fill the header
   stamp and covariance in `detector_node.py`; the estimator already consumes it.
7. Time sync: discipline the Jetson clock to the GQ7 (chrony plus PPS). Every
   10 ms of unaccounted latency is 15 cm at race speed.
8. Characterise the steering actuator on day 0: stepper slew rate, step
   response and stamp-to-steer latency (the `debug/roundtrip_time` and
   `debug/control_latency` topics already exist). Put the measured rate into
   `steering_rate_limit_dps` (sim), `maxSteerRateDps` (controller) and the
   raceline curvature-rate bound.
9. Survey GNSS availability along the course: log fix type, filter status and
   covariance against arc length on a pushed lap and on a roll, before and after
   RTK, then choose the planner margins per zone from that map.
10. Log the real radio link (RSSI, sequence gaps, timeouts) on a roll and
    replace `radio_sim.py`'s guessed 95 percent loss with the measured
    distribution. Put GPS time-of-week in NAND's radio packet so radio
    measurements can be fused with their true timestamps.
11. Replace the zone elevations in `course_zones.yaml` with a USGS 3DEP 1 m DEM
    sample along the SC line until the RTK altitude survey exists, and let the
    Hill 3 zone decay from freeroll speed so rollout counts in lap time.

Things the skeptic flagged that the code now handles: hard curvature rejection
needed a smooth reference (Track now fits a cubic spline through the waypoints
instead of linear segments, otherwise every hand-clicked vertex is a curvature
spike); pusher transition zones are no-lateral-change zones; the steering
slew-rate limit exists in the sim and the controller (off until measured); the
radio covariance follows NAND's fix type; the estimator gate cannot starve the
filter when covariances are mis-tuned (a measurement is rejected only when it
fails the statistical test and is more than 3 m from the prediction).

Known limits of the simulator: kinematic bicycle, no lateral dynamics, so a spin
cannot be observed; lateral-acceleration violations are only flagged, never
felt. Lap-time differences under a second are inside the speed model's own
uncertainty until its coefficients are fitted from a logged roll.

## 7. Validation log

Append results here as they are produced.

- 2026-09-06 single-buggy A/B, table in section 4.
- 2026-09-06 pass scenario, legacy vs Frenet, table in section 4.
- 2026-09-06 pass scenario with fake lidar and camera feeding the fused
  estimator (`use_perception_sim:=true`), 100 s: passed at 8.3 s, closest
  approach 2.77 m, lateral gap 2.52 m, no contact, SC tracking error 0.022 m RMS
  and 0.156 m max, planner rejected fraction 0 at the end of the run.
  Estimator source statistics at the end: radio 196 accepted / 84 rejected,
  camera 51 / 42 (camera age 98 s because NAND was behind SC after the pass,
  outside the forward field of view, which is correct). The rejection rates are
  too high for a source that is truth plus 2 cm of noise: the UKF prediction
  drifts between updates because its steering input comes from the shadow
  Stanley controller that runs on the very estimate being corrected. The gate
  distance default was raised to 5 m; the shadow-steering feedback loop is the
  next thing to tune, and the fusion should be re-checked with logged radio,
  lidar and camera data before it is trusted on the buggy. The planner fell back
  to "ignore the opponent constraint" 12 times in the run, always when the
  opponent estimate sat on top of SC's own position.
- 2026-09-06 single buggy with the gravity speed model
  (`use_speed_model:=true`), 120 s from Hill 1: mean speed 9.3 m/s, 1128 m
  covered, cross-track RMS 0.021 m, no errors. Nominal model lap 168.8 s.
