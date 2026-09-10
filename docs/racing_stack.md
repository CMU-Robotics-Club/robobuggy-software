# Racing stack: perception, localization and path planning upgrades

Branch `feature/racing-perception-planning`, based on `rolls` (Sept 2026).
This document explains what was researched, what was built, how it was tested
in the 2D simulator, and what still needs hardware. Raw research output with
every source URL is in `research_raw.json` next to the repo checkout. The
review that reshaped the design (three rounds, thirteen recorded decisions) is
in the untracked `.ai-collab/` directory: `DECISIONS.md` is the authority for
what the code is supposed to do.

## 1. The problem, stated honestly

A buggy is a gravity vehicle. Software commands steering only: no throttle, no
brakes. So "keep speed up" has exactly one lever in code, curvature. Every
metre of the line that turns more than it has to scrubs speed, and the chute
is where that speed is lost or kept. Overtaking helps only when the other buggy
is slower and only when a pass can be completed without contact (the passing
buggy bears full responsibility; contact is a disqualification). Knowing our
own position to centimetres is what allows a line close to the curb; knowing
an opponent's position early is what allows a smooth, early pass instead of a
late swerve.

The research (five parallel literature and code sweeps, synthesized and then
attacked by a skeptic) converged on a short list, and the review then added the
part the research had skipped: the contract between sensing, planning and
steering, so that nothing unsafe or unknown is ever acted on.

| Area | Choice | Why this and not the alternatives |
| --- | --- | --- |
| Global line | Offline bounded second-difference curvature smoother (an approximation of the minimum-curvature line, Heilmeier et al. 2019), optionally weighted by squared lateral acceleration from the gravity speed model | Standard in F1TENTH, Roborace and the Indy Autonomous Challenge; convex, fast, needs only boundaries. Min-time optimal control is wrong here because it assumes brakes. |
| Local planning | Arc-length Frenet lateral sampler d(s) with hard curvature, road and footprint checks on the exact curve the controller follows, a symmetric pass state machine, and one result envelope that says whether the plan may be steered on | Spatial planning keeps path shape independent of the speed estimate; MPCC and progress-maximising MPC degenerate without throttle or brakes. |
| Opponent state | One tracker (global gated assignment, constant velocity, no merging) fed by structured detections from lidar, camera and, where verified, radio; objects advanced along the road by their age | ForzaETH, TUM and PoliMOVE all use a single tracker plus constant-velocity-along-track prediction; learned lidar detectors need labelled data the team does not have. |
| Own position | RTK over NTRIP on the existing Microstrain GQ7 first (configuration only), a stamped localization health signal with expiry that the planner must see, lidar odometry (GLIM, FAST-LIO2) benchmarked later on real bags | SC runs single-point GNSS today (1 to 2.5 m). RTK fixed is 1 to 6 cm with no new hardware. GNSS under the Schenley trees is the team's own historical failure mode. |
| Perception | Geometric lidar pipeline (ground removal, clustering, every cluster published) for range; camera object list for class confirmation | One or a few opponents on a fixed course; geometric detection plus a track prior already works at this scale. |

## 2. What was built

New and changed files, all under `rb_ws/src/buggy/` unless noted. A longer,
decision-by-decision list is in `.ai-collab/IMPLEMENTATION_LOG.md`.

| File | Role |
| --- | --- |
| `config/vehicle_sc.yaml`, `scripts/racing/profile.py` | The single source of geometric and actuator limits. Every field carries a status (`verified_source`, `measured`, `assumed`, `policy`, `unverified`). The planning curvature cap can only be tightened, never raised above tan(20 deg)/wheelbase. A hardware profile refuses to start on unverified fields. |
| `scripts/racing/planning.py`, `scripts/racing/geometry.py` | Pure validation: a plan is control-eligible only if the spline the controller will actually follow (built with the controller's own `Trajectory` class) has finite geometry, curvature at or below the cap with no tolerance, stays inside the known road with the hard margin, and stays outside every observed opponent's rectangular footprint (inflated by a hard margin and two sigma) within a 6 s prediction horizon. Preferred limits (boundary margin, lateral clearance) only downgrade a plan to DEGRADED. Unknown width is unknown, never driven through. |
| `scripts/racing/health.py`, `scripts/racing/tracking.py` | Source-time readiness checks (NaN, missing, stale, unknown are never OK) and the tracker core (one-to-one gated assignment, observations counted once per capture stamp, prediction never counts as evidence, no post-hoc merging). |
| `msg/PlanningResultMsg.msg`, `LocalizationHealthMsg.msg`, `OffsetEstimateMsg.msg`, `DetectionMsg.msg`, `DetectionArrayMsg.msg`, `TrackMsg.msg`, `TrackingResultMsg.msg` | The contracts: planning result envelope (plan id, source stamps, validity, geometry, status, reasons, hard-check results, `control_eligible`), stamped health with expiry, stamped steering-offset estimate with generation and validity, structured capture-time detections, tracks with covariance and age. |
| `scripts/path_planner/frenet_planner.py` | The online planner. Samples lateral targets (0.25 m grid, 15/25/40 m transitions, pass-and-return when the road narrows ahead), screens them analytically, validates the best few on the reconstructed spline, and publishes ONE envelope on `planning/result` every cycle. Symmetric pass logic (either side), PASS commitment that keeps side and amplitude while an opponent is in reach and is released only when the committed side has no candidate at all, no new maneuver in no-pass zones or with degraded health, no lateral change in pusher zones, no emergency override. When nothing is eligible it re-validates and reuses its previous plan, and otherwise publishes the best diagnostic candidate marked INELIGIBLE, which nothing steers on. |
| `scripts/controller/controller_node.py` | Legacy behaviour unchanged when `planningResultTopic` is empty. In envelope mode it steers only on control-eligible, unexpired plans (checked every cycle, also when the planner goes silent), otherwise follows a smooth curvature-checked splice onto the static reference and says so on `controller/plan_source`. Offset correction only from a valid, fresh, same-generation `OffsetEstimateMsg` within the profile bound; composed command clamped to the profile's command limit; rate limit by elapsed time. |
| `scripts/estimation/steer_offset_estimator.py` | Publishes `self/steering_offset/estimate` with a generation counter (bumped on every reset), variance, observability from real motion and a validity verdict. The convergence flag alone was shown (review round 2) to admit a 36 degree estimate for a 4 degree offset. |
| `scripts/estimation/localization_monitor.py` | `localization/health_stamped`: source-time freshness, finite covariance, GQ7 fix and filter status with age; unknown is never OK when required. Sim launches set the simulation policy explicitly; `config/sc-roll.yaml` holds the hardware policy. |
| `scripts/perception/opponent_tracker.py` | The single fusion authority. Consumes `lidar/detection_array`, `vision/detection_array` and, only where configured, raw radio (`other/stateNoUKF` in the simulator, `radio/detection_array` on hardware). Publishes `perception/tracking` (with `perception_ready`) and the legacy `perception/tracks`. The NAND UKF (`nand_estimator.py`) is no longer a planner input; it still serves the NAND shadow controller. |
| `scripts/lidar/buggy_lidar.py`, `scripts/perception/lidar_opponent_node.py` | Every ellipse-filtered cluster is published with its extent; the adapter transforms them with the ego pose interpolated at the scan time and explicit, assumed extrinsics (`sensor_forward_axis` defaults to the lidar branch's negative-X convention until measured). |
| `scripts/vision/detector_node.py`, `scripts/perception/radio_detection_adapter.py` | Every ZED object published with the image timestamp and an `observed` flag (SDK-predicted objects never count); a radio adapter that keeps sequence and fix quality, marks capture time unknown and does not assume the MIP fix table. Both untested on hardware. |
| `scripts/path_planner/raceline_optimizer.py` | Speed profile reprojected every iteration (the weighted mode used to crash when the sample count changed), solver checked, final curvature and clearance validated, atomic write, honest naming. With the shipped arguments the usable right room is zero: the earlier claim of 0.5 m was wrong. |
| `scripts/util/track.py` | Vectorised Frenet projection; widths no longer clamped to zero (negative means narrower than the vehicle), unknown widths stay unknown. |
| `scripts/util/bag_to_course.py`, `scripts/util/record_course.sh`, `launch/record_course.xml`, `launch/replay_bag.xml` | Survey tool with fix freshness, gap segmentation, explicit edge offsets, manifest and atomic outputs; recorder that propagates failures, records `/tf`, `/clock` and `/lidar/*`, and writes a manifest; input-only replay. |
| `scripts/simulator/engine.py`, `scripts/simulator/perception_sim.py` | Warm-up no longer reports 12 m/s while standing still; `seed`; structured detections with capture stamps and extents; ghost-only scenes. |
| `config/scenarios/*.yaml`, `scripts/debug/run_sim_scenario.sh`, `sim_metrics.py`, `pass_metrics.py`, `scenario_timeline.py` | Committed seeded scenarios, a runner that isolates each on its own ROS domain and returns the metrics gate as its exit status, metrics that exit non-zero on collision, missing input or an ineligible plan being steered, and a bag timeline tool for diagnosing near misses. |
| `test/` | 84 pytest tests, registered in CMake (`racing_regressions`): health, tracking, profile, geometry, planning, survey (including real MCAP round trips), recorder, optimizer. |
| `launch/sim_2d_single.xml`, `sim_2d_double.xml`, `sc-main.xml`, `sc-system.xml`, `bench-system.xml`, `perception_bench.xml` | See section 3. The hardware launch has no argument that lets the experimental planner steer. |
| `setup_dev.sh`, `docker-dev.yml`, `DEV_NETWORK.md` (repo root) | Development script scoped to this project's containers; Foxglove and tile ports bound to localhost unless `BIND_ADDR` says otherwise. |

## 3. How to run

Everything below runs inside the container, in `/rb_ws` after the normal shell start.

Single buggy on the reference line with the experimental planner driving in envelope mode:

```bash
ros2 launch buggy sim_2d_single.xml use_frenet:=true
```

Two buggies (default: experimental planner in envelope mode; `use_frenet:=false` for the legacy sigmoid planner), with fake lidar and camera and ghost buggies:

```bash
ros2 launch buggy sim_2d_double.xml use_perception_sim:=true
```

Watch in Foxglove (`ws://localhost:8765`): `/SC/planning/status` (status, reasons, committed offset, cycle time), `/SC/controller/plan_source` (envelope or reference fallback and why), `/SC/perception/tracking`, `/SC/localization/health_stamped`.

Committed scenarios with pass/fail gates (see `config/scenarios/README.md`):

```bash
S=src/buggy/config/scenarios; R=src/buggy/scripts/debug/run_sim_scenario.sh
METRICS_ARGS="-p require_pass:=true" bash $R --launch sim_2d_double.xml --config $S/traffic.yaml --metrics pass_metrics.py --duration 100 --bag /tmp/traffic_bag --out /tmp/traffic.json -- use_perception_sim:=true
python3 src/buggy/scripts/debug/scenario_timeline.py --bag /tmp/traffic_bag --around-min-ghost 5
```

Tests and lint:

```bash
cd src/buggy && python3 -m pytest test -q
pylint --rcfile=../../../.github/workflows/.pylintrc <files>
```

Raceline (the honest name is a smoother; right room is zero until a right boundary is surveyed):

```bash
python3 src/buggy/scripts/path_planner/raceline_optimizer.py \
  --center buggycourse_sc.json --left-boundary buggycourse_curb.json \
  --right-width 1.3 --margin 0.6 --vehicle-width 1.2 --kappa-max 0.25 \
  --zones src/buggy/config/course_zones.yaml --out buggycourse_sc_raceline.json
```

On the buggy: `sc-main.xml` runs the LEGACY planner and controller as before, plus (by default, `frenet_shadow:=true`) the experimental planner and a shadow controller whose commands go to `debug/shadow/input_steering` and never to the serial node. `bench-system.xml` is `sc-system.xml` without the serial bridge, for stationary sensor tests with `perception_bench.xml` (see `docs/workshop_test.md`).

## 4. What the simulator showed

Numbers from 2026-09-06 (before the review) are kept for the record but were
produced by a planner that relaxed its own constraints silently; do not compare
them one to one with the numbers below.

Constant 12 m/s from Hill 1 for 90 s, Stanley following each line (2026-09-06):

| Line | Cross-track RMS | Steering RMS | Steering max | Steering rate RMS |
| --- | --- | --- | --- | --- |
| `buggycourse_sc.json` (team line) | 0.026 m | 0.76 deg | 3.64 deg | 26.7 deg/s |
| `buggycourse_sc_raceline.json` | 0.023 m | 0.71 deg | 3.43 deg | 23.8 deg/s |

The optimized line steers 6 to 11 percent less, entirely from left-side room:
with `right_width 0.5` and a 1.2 m vehicle plus margins the usable right room
was zero at every station (the earlier text claimed 0.5 m; that was wrong).

Committed scenarios on 2026-09-08 with the reviewed stack (all gates pass, no
plan the planner had marked ineligible was ever steered on):

| Scenario | Outcome | Envelope in use | Tracking error RMS / max | Closest opponent |
| --- | --- | --- | --- | --- |
| `single_reference` (60 s, 12 m/s) | reference line, envelope mode | 96.9 % | 0.028 / 0.124 m | none |
| `double_pass` (SC 13 m/s from 25 m behind NAND at 10 m/s, NAND known only through the raw radio) | passed at 5.9 s, no contact | 100 % | 0.020 / 0.103 m | 2.11 m |
| `traffic` (NAND plus two ghosts seen by fake lidar and camera) | passed at 6.2 s, no contact with anyone | 96.7 % | 0.026 / 0.285 m | 2.56 m NAND, 2.78 m ghost |
| `right_corridor` (wide assumed right side, ghost 1 m left of the line) | passed on the RIGHT, no contact | 99.7 % | 0.019 / 0.086 m | 2.15 m |
| `blocked` (two stationary ghosts filling the corridor) | no eligible plan once confirmed; controller reported the fallback; the reference line runs into the ghost | 92.4 % | 0.271 m max | 0.29 m (expected) |

The blocked scenario is the honest one: a steering-only vehicle with no
drivable corridor cannot avoid the obstacle, the planner says so, and what the
real buggy should then do (operator, brake, firmware) is the open hardware
contract in `DECISIONS.md`, not something this code pretends to solve.

Planner cycle time was 22 to 38 ms on the development laptop; the Orin is
unmeasured.

## 5. Parameters worth knowing

| Parameter | Where | Value | Meaning |
| --- | --- | --- | --- |
| `planning_curvature_cap_1_per_m` | `config/vehicle_sc.yaml` | 0.25 (policy) | Never above the software ceiling tan(20 deg)/1.104 m = 0.330. |
| `width_m`, `length_m` | vehicle profile | 1.2, 2.5 (assumed) | Footprint; measure the buggy. |
| `physical_wheel_angle_deg`, `actuator_slew_dps`, `actuator_delay_s` | vehicle profile | unverified | A hardware profile refuses to start until measured. |
| `hard_boundary_margin`, `boundary_margin` | planner | 0.2 m hard, 0.5 m preferred | Hard decides eligibility; preferred only degrades. |
| `hard_opponent_margin`, `opponent_sigma_gain` | planner | 0.2 m, 2 | Footprint inflation. |
| `lateral_clearance`, `longitudinal_window` | planner | 1.6 m, 6 m | Preferred centre-to-centre gap while alongside. |
| `right_width` | planner, configs | 1.3 m (assumed) | Distance from the line to the assumed right edge; every plan is DEGRADED with `right_width_assumed` until surveyed. |
| `prediction_horizon_s` | planner | 6 s | Opponents are checked where ego arrives within this time. |
| `lateral_velocity_deadband`, `lateral_velocity_max_sigma`, `max_lateral_drift_m` | planner | 0.5 m/s, 0.5 m/s, 1 m | Lateral drift model, a documented assumption. |
| `w_change` | planner | 10 | Damps target hopping between cycles. |
| `acceleration_std` | tracker | 3 m/s² | Course curves demand up to about 5 m/s²; lower values spawn duplicate tracks after any gap. |
| `require_rtk_for_ok`, `require_filter_for_ok` | monitor | true on hardware, false in sim | Unknown is never OK. |
| `fallbackSpliceLengthM` | controller | 30 m | Smooth return onto the reference when no plan is eligible. |
| `maxPlausibleOffsetDeg` | offset estimator | 10 deg (assumed) | Plausibility bound on the steering-offset estimate. |

## 6. What needs hardware, in order

Before any of it: `docs/workshop_test.md` explains how to put this branch on the
buggy, run the perception chain standing still (`bench-system.xml` plus
`perception_bench.xml`), watch it in Foxglove and bring a recording home.

1. Answer the open questions in `DECISIONS.md`: what the firmware does when
   autonomous steering is off, whether it clamps angle or rate, what the alarm
   packet does, who controls the brake, whether the NAND radio is fitted, which
   Microstrain and Velodyne driver versions are installed.
2. Survey the course (`docs/course_survey_checklist.md`): centre line and
   elevation on a rolled pass, left and right edges walked with RTK fixed,
   converted with `bag_to_course.py` (explicit edge offsets, manifest). Then
   rerun the smoother with `--right-boundary` and retire `right_width`.
3. Enable RTK on the GQ7 in `INS_params.yml` (NTRIP, antenna lever arms) and
   confirm "RTK Fixed" in `localization/health_stamped`; the hardware policy
   already requires it.
4. Measure the buggy: width, length, wheelbase, mechanical steering stop,
   actuator slew and delay from the firmware's `true_steering_angle` against
   commanded steps. Put them in `config/vehicle_sc.yaml` with status `measured`.
5. Measure the lidar and camera extrinsics on the bench (a target at a taped
   offset) and set `extrinsics_status` accordingly; verify the forward-axis
   convention.
6. Fit the speed model from one logged roll; replace the zone elevations with
   the surveyed profile.
7. Run the shadow planner for whole rolls and compare `debug/shadow/input_steering`
   with `input/steering` in the bags before anyone considers letting it steer.
8. Benchmark GLIM and FAST-LIO2 on the recorded bags against the RTK track in
   shadow mode; consider a gravity-adapted MPCC only against measured criteria.

## 7. Validation log

- 2026-09-06 single-buggy A/B and pass scenarios, tables in section 4 (pre-review planner).
- 2026-09-08 review rounds 1 to 3 (`.ai-collab/`): 13 reproduced defects, 13 decisions.
  Notable findings: the composed steering command reached 36 degrees in the stock sim
  because the inherited offset estimator published a 36 degree estimate for a 4 degree
  offset five seconds after declaring convergence; the simulator warm-up reported 12 m/s
  while stationary; the reference line's curvature peaks at 0.061 1/m, so the planner's
  old "raise the limit to the reference" logic was dead code.
- 2026-09-08 reviewed stack: five committed scenarios, table in section 4; 84 tests;
  lint 10.00/10 under CI conditions. Details and the list of problems found and fixed
  during verification: `.ai-collab/TEST_RESULTS.md`.
- 2026-09-09 first run on the SC NUC, stationary, serial node stopped, lidar unplugged
  (`docs/workshop_test.md`, bench log). Branch built clean on the buggy; tracker, lidar
  adapter, both legacy controllers and the shadow controller ran. Two crashes found and
  fixed: the localization monitor on the Microstrain 4.x nested header, and the planner
  on NaN indoor positions (now INELIGIBLE `state_not_finite`). ZED not detected (USB 2
  port). Nothing moved; hardware questions in section 6 are unchanged.
