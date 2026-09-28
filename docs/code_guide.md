# Working on the racing stack

How this code is put together, so you can change it without asking anyone. It assumes you
can read Python and have seen ROS 2 before. `docs/racing_stack.md` says *what* was built and
why; this file says *where things are* and *how to work on them*.

Paths are relative to `rb_ws/src/buggy/` unless they start with `rb_ws/`.

---

## 1. The one thing to understand first

There are **two control stacks in this repo**, running side by side.

**The legacy stack** is what drives the buggy today:

```
self/state ──> path_planner.py ──> self/cur_traj ──> controller_node.py ──> input/steering ──> ros_to_bnyahaj.py ──> Teensy
```

`path_planner.py` is the sigmoid planner: it swings the path a few metres left of wherever
NAND is, with no state machine and no obstacle checking. The controller follows whatever
arrives and publishes a steering angle. Nothing validates anything.

**The envelope stack** is the new work:

```
self/state ─┐
tracking ───┼─> frenet_planner.py ──> planning/result ──> controller_node.py (envelope mode) ──> steering
health ─────┘        (plan + verdict)         (steers only if the verdict allows)
```

The difference in one sentence: **the planner grades its own plan, and the controller
refuses plans that are not graded eligible.** Everything else follows from that.

On the real buggy the envelope stack runs as a **shadow**: it publishes to
`debug/shadow/*`, which the serial node does not subscribe to. There is deliberately no
launch argument that lets it steer. If you are about to add one, that is the decision you
are reversing, and it is written down in `.ai-collab/DECISIONS.md` (D1, D13).

---

## 2. Where everything lives

### The new code (`scripts/racing/` — pure Python, no ROS)

This package is the core. It has **no ROS imports**, so you can run and test it directly.
That is deliberate: the policy logic is testable without a robot.

| file | what it is |
| --- | --- |
| `planning.py` | The validation envelope. `validate_plan()` decides ELIGIBLE / DEGRADED / INELIGIBLE. Read this one first. |
| `geometry.py` | `ReconstructedCurve` — wraps a path in the *same* spline class the controller uses, then samples it densely. |
| `profile.py` | `VehicleProfile` — loads `config/vehicle_sc.yaml`. Every physical number with its provenance. |
| `health.py` | Freshness and covariance checks; `evaluate_health()` returns OK / DEGRADED / BAD. |
| `tracking.py` | `MultiObjectTracker` — constant-velocity Kalman filter with gated global assignment. |

### The ROS nodes

| file | role |
| --- | --- |
| `scripts/path_planner/frenet_planner.py` | The online planner. One big `plan()` at 10 Hz. |
| `scripts/path_planner/path_planner.py` | The legacy sigmoid planner. Don't edit; it's what we compare against. |
| `scripts/path_planner/raceline_optimizer.py` | Offline. Minimum-curvature line, writes a waypoint JSON. |
| `scripts/controller/controller_node.py` | Both modes. `experimental` is true when `planningResultTopic` is set. |
| `scripts/controller/stanley_controller.py` | The control law. **Unchanged from `rolls`** — don't touch it casually. |
| `scripts/perception/opponent_tracker.py` | ROS wrapper around `racing/tracking.py`. The single fusion authority. |
| `scripts/perception/lidar_opponent_node.py` | Lidar clusters (sensor frame) → UTM detections, using ego pose at scan time. |
| `scripts/perception/radio_detection_adapter.py` | NAND radio → detections. Off in every launch file. |
| `scripts/lidar/buggy_lidar.py` | Point cloud → ground removal → DBSCAN clusters. From the lidar branch. |
| `scripts/vision/detector_node.py` | ZED + YOLO → detections. Needs the ZED SDK. |
| `scripts/estimation/localization_monitor.py` | Publishes `localization/health_stamped`. |
| `scripts/estimation/steer_offset_estimator.py` | UKF for the constant steering bias. |
| `scripts/estimation/nand_estimator.py` | Legacy UKF for NAND's position. Not a planner input any more. |

### Support

| file | role |
| --- | --- |
| `scripts/util/track.py` | `Track` — the Frenet model. Station `s`, lateral `d`, road widths. |
| `scripts/util/trajectory.py` | `Trajectory` — the spline the controller follows. Old code, widely used. |
| `scripts/simulator/engine.py` | Kinematic bicycle. Publishes `self/state`. |
| `scripts/simulator/lidar_sim.py` | Ray-cast VLP-16 against a 3D world built from the course files. |
| `scripts/simulator/perception_sim.py` | Shortcut detections (skips the point cloud). Ghost buggies. |
| `scripts/debug/course_viz.py` | Foxglove markers: course, plan, tracks, HUD. |
| `scripts/debug/sim_demo.sh` | Start the visual sim in the container. |
| `scripts/debug/run_sim_scenario.sh` | Run one scenario headless; exit code is the pass/fail gate. |
| `scripts/debug/sim_metrics.py`, `pass_metrics.py` | The gates themselves. |

### Everything else

- `msg/` — 16 message definitions. The new ones are `PlanningResultMsg`, `TrackingResultMsg`,
  `TrackMsg`, `DetectionArrayMsg`, `DetectionMsg`, `LocalizationHealthMsg`, `OffsetEstimateMsg`.
- `launch/` — `sc-*.xml` is hardware, `sim_2d_*.xml` is simulation, `*bench*.xml` is stationary testing.
- `config/` — `sc-roll.yaml` is hardware parameters, `sim_*.yaml` is simulation,
  `vehicle_sc.yaml` is the vehicle profile, `scenarios/` holds the five committed test cases.
- `paths/` — waypoint JSON files (lat/lon). `buggycourse_sc.json` is the reference line.
- `test/` — pytest. Runs without ROS.

---

## 3. The development loop

Everything runs in the Docker container. The buggy itself runs ROS natively, no Docker.

```bash
./setup_dev.sh                                    # start the container (once)
docker exec -it robobuggy-software-main-1 bash    # get a shell inside it
```

Inside the container you are in `/rb_ws`, which is your `rb_ws/` bind-mounted. Editing files
on Windows changes them instantly inside the container.

```bash
cd /rb_ws
colcon build --symlink-install
source install/local_setup.bash
```

**When you must rebuild.** `--symlink-install` means the installed scripts are symlinks to
your source, so **editing an existing Python file needs no rebuild** — just restart the node.
You must rebuild when you:

- add a new script (and add it to `CMakeLists.txt`),
- change anything in `msg/`,
- change `CMakeLists.txt` or `package.xml`.

A message change also needs the build directory cleaned if you are switching between branches
that declare different messages:

```bash
rm -rf /rb_ws/build/buggy /rb_ws/install/buggy && colcon build --symlink-install
```

Skipping that gives you `fatal error: geometry_msgs/msg/detail/point__struct.h: No such file`,
which is rosidl compiling a message that no longer exists.

### Tests

```bash
cd /rb_ws/src/buggy && python3 -m pytest test -q
```

Around 86 tests, a handful skipped when rosbag2 is unavailable. They import
`scripts/racing/*` directly and need no ROS graph, so they are fast. Add a test whenever you
change policy logic.

### Lint (this is what CI runs)

```bash
cd /rb_ws/src/buggy
pylint --rcfile=.github/workflows/.pylintrc <changed files>
```

CI runs it **without ROS on the path**, over files changed against `origin/main`. Keep it at
10.00. The usual complaint is import order: standard library, then third party, then
`racing`/`util`, then `buggy.msg` last.

### The visual simulation

```bash
docker exec robobuggy-software-main-1 /rb_ws/src/buggy/scripts/debug/sim_demo.sh
```

Then Foxglove → Open connection → `ws://localhost:8765`, and import
`foxglove/racing_sim_layout.json`. The log is `/tmp/sim.log` inside the container.

### Scenario gates (what to run before claiming a planner change works)

```bash
bash src/buggy/scripts/debug/run_sim_scenario.sh \
  --launch sim_2d_double.xml \
  --config src/buggy/config/scenarios/double_pass.yaml \
  --metrics pass_metrics.py --duration 100 --domain 171 --out /tmp/out -- use_perception_sim:=true
echo $?    # 0 = gates passed
```

The five scenarios are described in `config/scenarios/README.md`.

---

## 4. The core abstractions

### `Track` (`util/track.py`) — the coordinate system

Everything geometric happens in Frenet coordinates: `s` is distance along the reference
line, `d` is lateral offset, **left positive**.

```python
track = Track.from_files(path_to_json, left_boundary_json=..., right_boundary_json=None,
                         default_left=3.0, default_right=1.3, margin=0.8)
s, d = track.frenet(x, y)          # vectorised: arrays or scalars
xy   = track.cartesian(s, d)       # ALWAYS returns (n, 2), even for a scalar s
wl, wr = track.width_at(s)
```

Three things will bite you:

1. **`cartesian()` always returns a 2-D array.** `px, py = track.cartesian(30.0, 0.0)` raises
   `not enough values to unpack`. Write `track.cartesian(30.0, 0.0)[0]`.
2. **The widths already have the margin removed.** The planner passes
   `margin = hard_boundary_margin + half_width`, so `w_left` is how far the *vehicle centre*
   may go, not where the curb is. Do not subtract half width again.
3. **NaN width means unknown, and unknown is never drivable.** There is no right-boundary
   file, so the right width is an assumption (1.3 m) and every plan carries the reason
   `right_width_assumed`. If you set the default to `None`, the width becomes NaN and every
   plan becomes INELIGIBLE. That is the intended behaviour, not a bug.

### `Trajectory` and `ReconstructedCurve` — the curve the controller follows

`Trajectory` is old code and slightly surprising: it fits an Akima interpolant, resamples
uniformly by arc length, then fits a **cubic spline parameterised by index, not distance**.
So `get_position_by_index(i)` takes an index; convert with `get_index_from_distance(d)`.

The important consequence: the controller does not follow your waypoints, it follows that
spline. A path that looks smooth as points can have curvature spikes once splined. That is
why `racing/geometry.py` wraps the *same class* and samples it every 0.25 m, and why
validation runs on those samples. **If you write new validation, use `ReconstructedCurve`,
never the raw waypoints.**

### `VehicleProfile` (`racing/profile.py`) — where numbers come from

`config/vehicle_sc.yaml` is the only source of physical limits. Every field has a status:
`verified_source`, `measured`, `assumed`, `policy`, or `unverified`. Unverified fields must
have a null value, and asking for one raises.

```python
profile = VehicleProfile.load()
cap = profile.planning_curvature_cap()    # min(policy 0.25, tan(20°)/1.104 = 0.330)
profile.require_measured(["width_m"])     # raises: width is still 'assumed'
```

If you measure something on the real buggy, change the value **and** the status **and** put
the date and method in the note. Do not upgrade a status because a simulation looked good.

### `validate_plan()` (`racing/planning.py`) — the envelope

This is the heart of the design. It returns a `ValidationResult` with a status and reasons.

- **Hard checks** make a plan INELIGIBLE: non-finite geometry, curvature above the cap
  (no tolerance), leaving the known road, unknown road width, or overlapping an opponent's
  inflated footprint.
- **Preferred checks** only downgrade to DEGRADED: less than 0.5 m of road margin, or less
  than 1.6 m of lateral clearance while alongside.
- DEGRADED is still `control_eligible`. INELIGIBLE never is.

Opponent footprints are **rectangles in track coordinates**, inflated by
`0.2 m + 2 × sigma`. Circles were tried and demanded about 2.4 m of lateral room, which made
every pass ineligible.

One escape hatch: a plan that *starts* outside the road is allowed if it is back inside
within 25 m (`recovers_into_road`). It is DEGRADED, never silently fine. That exists because
the buggy can already be outside when planning starts.

### `MultiObjectTracker` (`racing/tracking.py`)

State is `[x, y, vx, vy]`, constant velocity, `acceleration_std = 3.0 m/s²`. Detections are
gated by Mahalanobis distance (chi-square 9.21) and 6 m, then assigned globally with the
Hungarian algorithm. A track is `confirmed` after 3 hits and dropped after 1.5 s.

`snapshot()` returns the **last filtered state with no extrapolation**. That is deliberate:
straight-line extrapolation in UTM is wrong on a curve and added up to 1.3 m of lateral
error. The planner advances opponents *along the road* by track age instead.

---

## 5. Reading `frenet_planner.plan()`

One function, roughly 300 lines, runs at 10 Hz. It goes in this order:

1. **Gates.** Ego state older than 0.3 s, non-finite position, bad or expired health, stale
   or missing tracker output → record a hard failure. Non-finite position short-circuits to
   `publish_no_geometry()`.
2. **Opponents.** Confirmed tracks only. Convert to Frenet, advance along the road by track
   age, build `OpponentPrediction`s. Lateral velocity is dead-banded at 0.5 m/s, gated on its
   own sigma, and capped at 1 m of predicted drift.
3. **Candidates.** A grid of target offsets every 0.25 m up to ±3 m, times three transition
   lengths (15, 25, 40 m). Each is a quintic Hermite lateral profile, then held.
4. **Stage one screening.** Cheap rejects: unknown width, outside the road, curvature over
   the cap (using the analytic estimate `κ_ref/(1 − κ_ref·d) + d''`), opponent footprint
   overlap. Survivors get a cost.
5. **Cost.** Six weighted terms — curvature 400, lateral acceleration 30, deviation 1,
   boundary margin 3, change from last target 10, proximity 6.
6. **State machine.** RACELINE / PASS / REJOIN. Entering PASS commits a side and amplitude;
   candidates that flip side or shrink the offset are rejected until the commitment is
   released because nothing is left on that side.
7. **Stage two.** Full `validate_plan()` on the four cheapest. First eligible one wins.
8. **Fallbacks.** No eligible candidate → re-validate and reuse the previous plan if 20 m of
   it remains → otherwise publish a diagnostic plan forced to INELIGIBLE.
9. **Publish** `PlanningResultMsg` plus a JSON summary on `planning/status`.

When you change planner behaviour, the thing to watch in `planning/status` is `reasons`. It
tells you exactly which check rejected what.

---

## 6. The controller in envelope mode

`controller_node.py` runs in two modes. `self.experimental` is simply
`bool(planningResultTopic)`. In envelope mode:

- `select_trajectory()` accepts a plan only if `control_eligible`, status is ELIGIBLE or
  DEGRADED, `valid_until` has not passed, and it arrived under 0.5 s ago. Otherwise it sets
  a reason and falls back.
- `build_fallback()` builds a smooth splice from wherever the buggy is back onto the static
  reference, trying 30 / 45 / 60 m and taking the first that stays under the curvature cap.
  **It is not checked against obstacles** — a known, deliberate gap.
- `offset_correction_rad()` uses the steering-offset estimate only if it is fresh, under the
  plausibility bound, and at least 1 s past a filter reset.
- The final command is clamped to the profile's limit *after* the offset is applied. The
  legacy path clamps before, which is a real difference.

`controller/plan_source` publishes which trajectory is in use and why. That topic is the
first place to look when the buggy is not doing what the planner said.

---

## 7. How to do common things

**Change a planner parameter.** Add `declare_parameter` in `frenet_planner.__init__`, read it
into an attribute, then set it in `config/sim_double.yaml` and `config/sc-roll.yaml`. No
rebuild needed. Remember both configs, or hardware and simulation silently diverge.

**Add a message.** Create `msg/Foo.msg`, add it to the `rosidl_generate_interfaces` block in
`CMakeLists.txt`, then clean-rebuild (see above). Import it *last* in Python files to keep
pylint happy.

**Add a node.** Write it under `scripts/`, make it executable, add the path to the
`install(PROGRAMS ...)` list in `CMakeLists.txt`, rebuild, then add it to a launch file.
Commit with the executable bit set or `colcon` will set it on the buggy and dirty the tree:

```bash
git update-index --chmod=+x rb_ws/src/buggy/scripts/your/new_node.py
```

**Add a scenario.** Copy a YAML in `config/scenarios/`, set a seed, and run it through
`run_sim_scenario.sh` with the right metrics script. Document the gate in the README there.

**Tune the tracker.** `acceleration_std` is the parameter that matters. Too low and the gate
rejects real detections on curves and spawns duplicate tracks. 3.0 was chosen because course
curves pull about 5 m/s².

---

## 8. Invariants — break these and the design stops meaning anything

1. **Unknown is never OK.** Missing, stale, or unmeasured input makes a plan INELIGIBLE. Do
   not add a default that papers over absent data.
2. **Hard limits are never relaxed at runtime.** Preferred limits can only downgrade.
3. **Validate on the reconstructed spline**, not on the waypoints.
4. **One fusion authority.** The tracker is the only thing that decides where other objects
   are. Do not let a node read a detection topic directly and act on it.
5. **Nothing experimental reaches the serial node on hardware.** The shadow controller
   publishes to `debug/shadow/*` by configuration, and `ros_to_bnyahaj.py` subscribes only to
   `input/steering`.
6. **Provenance is not a formality.** A number's status in `vehicle_sc.yaml` is a claim about
   the physical world.

---

## 9. Gotchas that will cost you an afternoon

- **`cartesian()` returns `(n, 2)`** even for a scalar station.
- **Clean the build when switching branches** that declare different messages.
- **The boot service ignores build failures.** `start_buggy.sh` runs `colcon build` and
  launches regardless, so a failed build silently keeps the previous `install/`. Always read
  the build summary.
- **`sudo` needs a password on the NUC.** You do not need it — stopping the stack is just
  tmux (`tmux respawn-pane -k -t buggy.0` and `buggy.1`).
- **pytest 8+ breaks ROS's `launch_testing` plugin** on the buggy. Pin `pytest<8` there.
- **Set `OPENBLAS_NUM_THREADS=1`** for these nodes. Without it the 100 Hz numpy loops leave
  BLAS workers spin-waiting and every controller shows about 200 % CPU. The sim launch files
  set it; the hardware ones do not yet.
- **Lidar clustering takes 250–450 ms per scan.** With a queue depth above 1 its detections
  go stale and the UTM adapter drops them all with "no ego pose covering the scan time".
- **Indoors there is no fix**, so `self/state` is infinite, health is BAD, and every plan is
  INELIGIBLE. That is correct behaviour and makes indoor planner testing meaningless.

---

## 10. When something is wrong, look here first

| symptom | look at |
| --- | --- |
| Buggy ignores the plan | `controller/plan_source` — it names the reason |
| Every plan INELIGIBLE | `planning/status` → `reasons` |
| No plans at all | Is `self/state` finite? Is health published and fresh? |
| Tracker sees nothing | `debug/tracker/status` counters, and `perception_ready` |
| Tracks duplicating | `acceleration_std`, and whether detections are stale |
| Detections dropped | The lidar adapter's warning about ego pose coverage |
| Node died on launch | `/tmp/sim.log` in the container, or the tmux pane on the buggy |
