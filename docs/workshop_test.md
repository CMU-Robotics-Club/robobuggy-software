# Workshop test: running this branch on the real buggy

How to get `feature/racing-perception-planning` onto Short Circuit, run the
perception chain with the buggy standing still, look at the result, and bring
the data home. Nothing in this test commands steering.

Checked on the NUC on 2026-09-09: the repo is at `~/robobuggy-software`, ROS 2
Humble is native (no Docker on the buggy), the Python packages live in the
virtualenv `~/robobuggy-software/.sc`, and the systemd service `buggy` starts the
stack in a tmux session at boot. The machine is an Intel NUC11PHi7 with an RTX
2060, hostname `nuc-NUC11PHi7`. On CMU wifi it is `roboclub-robobuggy-nuc.wifi.local.cmu.edu`
(`ssh sc_cmu`); `192.168.1.217` is its wired interface, so `ssh sc` only works from
the ShortCircuit network. If the team has moved to the Jetson Orin, ask them for
the new address and user; everything else is the same.

## 0. Safety, before touching the keyboard

- Buggy on stands or wheels chocked, brake engaged, drop-brake test done.
- The RC transmitter's autonomous-steering switch stays OFF the whole time.
  Team lore says the firmware only obeys software steering while that switch is
  on; that is UNVERIFIED and the team has to confirm it against the firmware
  source before anyone relies on it. Until then the stands, the chocks and the
  e-stop are what keep the wheel from moving, not the switch. Nothing in this
  test publishes a steering command in any case.
- One person at the e-stop while anything is running.
- Lidar spinning and a camera on: no one needs to be in front of the buggy
  except deliberately, as a test target.

## 1. Connect

The two buggy computers, as SSH aliases (put these in `~/.ssh/config`):

| alias | address | user | network |
| --- | --- | --- | --- |
| `sc` | 192.168.1.217 | nuc | ShortCircuit hotspot |
| `sc_cmu` | roboclub-robobuggy-nuc.wifi.local.cmu.edu | nuc | CMU wifi |
| `nand` | 192.168.10.191 | nand | NAND hotspot |
| `nand_cmu` | nandjetson.wifi.local.cmu.edu | nand | CMU wifi |

```
Host sc
  HostName 192.168.1.217
  User nuc
  SetEnv TERM=xterm-256color
```

(repeat for the other three), then:

```bash
ssh sc
```

Look around before changing anything:

```bash
systemctl status buggy          # is the stack already running?
tmux ls                         # the "buggy" session, if so
cd ~/robobuggy-software && git status && git branch --show-current
```

The service starts `sc-system.xml` and `sc-main.xml` at boot. It is only a
wrapper: `buggy.service` runs `/home/nuc/start_buggy.sh` once, which builds the
workspace and sends the two launch commands into panes 0 and 1 of the `buggy`
tmux session, owned by `nuc`. Two consequences worth knowing:

- `sudo` asks for a password on the NUC. You do not need it: stopping the stack
  is what the team's own `bootstrap/stop_buggy.sh` does, and that is just tmux.
- A failed build does not stop the service. The `colcon` errors scroll past and
  the stack starts from whatever is left in `install/`. Read the build summary.

For a perception bench test you want the sensors but not the controller and
planner, so stop the stack (the same commands as `stop_buggy.sh`) and start the
sensors yourself:

```bash
tmux respawn-pane -k -t buggy.0     # sc-system.xml: INS, foxglove, serial node
tmux respawn-pane -k -t buggy.1     # sc-main.xml: controllers, planner, estimators
pgrep -af "ros_to_bnyahaj|controller_node" || echo stopped
```

## 2. Get the branch onto the buggy

The buggy has no access to your laptop, so the branch has to travel through
GitHub or over the SSH link. Pushing a branch is not a pull request; nobody has
to review it and it does not touch `rolls` or `main`.

Option A, through GitHub (simplest). On your laptop:

```bash
git push -u origin feature/racing-perception-planning
```

On the buggy:

```bash
cd ~/robobuggy-software
git fetch origin
git checkout feature/racing-perception-planning
```

Option B, straight over SSH, no GitHub. On your laptop, once:

```bash
git remote add buggy sc:robobuggy-software
git push buggy feature/racing-perception-planning
```

then check the branch out on the buggy as above. Either way, note which branch
the buggy was on before (`rolls`, most likely) so you can put it back.

The two branches declare different message sets: `rolls` has none of
`DetectionArrayMsg`, `PlanningResultMsg`, `TrackingResultMsg`. rosidl does not
clean its generated code between configures, so after switching in either
direction remove the package's build output before building. Otherwise the next
build fails on a stale header, as it did on 2026-09-09
(`geometry_msgs/msg/detail/point__struct.h: No such file` while compiling a
message that no longer existed):

```bash
rm -rf ~/robobuggy-software/rb_ws/build/buggy ~/robobuggy-software/rb_ws/install/buggy
```

## 3. Install the extra packages and build

```bash
cd ~/robobuggy-software
source .sc/bin/activate
pip install -r perception-requirements.txt      # open3d, scikit-learn, ultralytics
pip install "pytest<8" mcap                     # for test/; pytest 8+ breaks ROS's launch_testing plugin
cd rb_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/local_setup.bash
source environments/sc_env.bash
cd src/buggy && python3 -m pytest test -q -p no:cacheprovider   # the branch's regressions on the buggy's Python
```

As of 2026-09-09 the NUC already has everything: numpy 1.26, scipy 1.15, open3d
0.19, scikit-learn 1.7, ultralytics 8.3, torch 2.6 with CUDA 12.4, `pyzed`,
`ros-humble-velodyne` 2.5.1 and the Microstrain driver 4.5.0. The branch built
clean there in 10 s. Only `pytest` and `mcap` were missing.

The ZED camera needs the ZED SDK and its Python package `pyzed`, which the
vision people installed for `detector_node.py`. If `python3 -c "import pyzed"`
fails, run the bench test with `use_camera:=false` and sort the camera out
separately. On 2026-09-09 `pyzed` imported fine but the SDK reported
`CAMERA NOT DETECTED` although `lsusb` listed the ZED 2i: it was enumerated behind
a USB 2.0 hub, and the ZED SDK needs USB 3. Move it to a blue port directly on
the NUC before blaming the software.

## 4. Run the bench test

Open three tmux panes (`tmux new -s bench`, then `Ctrl-b %` to split; the
`.bashrc` on the buggy already sources everything):

Pane 1, sensors and infrastructure, with the serial-free system launch:

```bash
ros2 launch buggy bench-system.xml use_lidar:=false
```

`bench-system.xml` is `sc-system.xml` without the serial node `ros_to_bnyahaj.py`.
Standing still on the bench the firmware link is not needed, the serial node is
the one process that can hand a steering or alarm packet to the Teensy, and with
the Teensy unplugged or being flashed it would only respawn and fill the log with
port errors. `use_lidar:=false` because `perception_bench.xml` (pane 2) starts
`lidar_opponent_node.py` itself.

Pane 2, the perception chain from this branch:

```bash
ros2 launch buggy perception_bench.xml
```

Pane 3, a recording so you can replay it at home:

```bash
ros2 launch buggy record_course.xml pass:=bench
```

Then from your laptop on the same Wi-Fi, open Foxglove and connect to
`ws://192.168.1.217:8765`. Import `sc_telematics_layout.json` and add panels for:

| Topic | What good looks like |
| --- | --- |
| `/velodyne_points` | a 3D cloud updating at 10 Hz (`ros2 topic hz /velodyne_points`) |
| `/lidar/obstacle_centroid` | a point that jumps to whatever is closest in front |
| `/SC/lidar/detections` | the same point in UTM coordinates |
| `/SC/vision/other/state` | appears when a buggy-shaped thing is in the camera's view |
| `/SC/perception/tracks` | a track appears after about 0.3 s when someone walks in front, keeps its id while they move, disappears 1.5 s after they leave |
| `/SC/debug/tracker/status` | counters climbing: detections, associated, new_tracks, dropped |
| `/SC/localization/status` | fix type and reasons; indoors it will say degraded, that is expected |

Things to try, in order: a person walking a slow circle 5 to 15 m in front;
two people at once (two ids, once the all-cluster output of `buggy_lidar` is
live; today it publishes only the nearest cluster as `/lidar/obstacle_centroid`,
so until then expect one id that jumps between the two); a person walking behind
a pillar and back (does the id survive the 1.5 s blackout?); NAND pushed slowly
past if it is around.
Write down what the tracker saw and missed, and at what range. That list is
the tuning input for the lidar parameters at the top of `perception_bench.xml`.

Useful checks while it runs:

```bash
ros2 topic list | grep -E "lidar|vision|tracks|localization"
ros2 topic hz /SC/perception/tracks
ros2 topic echo /SC/debug/tracker/status --once
```

## 5. Bring the data home

Stop the recorder with Ctrl-C in pane 3, then from your laptop:

```bash
scp -r sc:robobuggy-software/rb_ws/bags/bench_* ./rb_ws/bags/
```

At home, inside the container, replay it with `launch/replay_bag.xml`, which
plays the input topics only (the recorded `/SC/perception/tracks` and tracker
status are deliberately not replayed, so you never look at a mix of recorded and
recomputed outputs), publishes `/clock`, and with `run_tracker:=true` starts
`opponent_tracker.py` in namespace `SC` with `use_sim_time:=true`:

```bash
export ROS_DOMAIN_ID=<a number nobody else on this machine uses>
ros2 launch buggy replay_bag.xml bag:=bags/bench_<stamp> run_tracker:=true
```

Any other node you start next to it (Foxglove bridge, lidar nodes) also needs
`use_sim_time:=true`, or its timers run on the wall clock while the data is on
bag time. Then open Foxglove on `ws://localhost:8765` as usual. Replaying is how
you tune the tracker without going back to the workshop.

## 6. Put the buggy back

```bash
cd ~/robobuggy-software && git checkout rolls      # or whatever it was on
rm -rf rb_ws/build/buggy rb_ws/install/buggy       # stale generated messages, see section 2
BUGGY=sc BAG_DIR=/home/nuc/bags/ PROJECT_ROOT=/home/nuc/robobuggy-software /home/nuc/start_buggy.sh
```

That last line is exactly what the service runs at boot: it rebuilds and respawns
the two launch panes, no `sudo` needed. Confirm with `git branch --show-current`
(`rolls`) and `ros2 node list` (`/SC/bnyahaj`, the serial node, is back). If
`git checkout` refuses because scripts show as modified, that is `colcon` setting
executable bits on files committed without them; `git stash` and carry on.

## Bench log

**2026-09-09, workshop, NUC on CMU wifi, lidar unplugged, buggy stationary.**
The serial node was stopped first, so nothing in this session could reach the
Teensy; the firmware topic reported `auton_steer: false` and `tx12_state: false`
throughout.

| check | result |
| --- | --- |
| `colcon build --symlink-install` of this branch on the NUC | clean, 10 s; the five new messages registered |
| `bench-system.xml use_lidar:=false` | INS at 100 Hz, Foxglove and state converter up; `localization_monitor` crashed (fix 1) |
| `perception_bench.xml use_lidar_driver:=false` | `buggy_lidar`, `lidar_opponent` and the tracker up, tracker status at 20 Hz; `detector_node` exited, ZED not detected (USB 2, section 3) |
| `sc-main.xml` beside it (legacy + shadow, no serial) | both legacy controllers, path planner, offset and NAND estimators, shadow controller up and reporting `source: reference`; shadow planner crashed (fix 2) |
| unit tests on the NUC | not run: pytest 9 clashed with `launch_testing`; pin `pytest<8` |

Fixed in the branch afterwards:

1. `localization_monitor.py`: driver 4.5.0 messages carry a `MipHeader` whose
   `header` holds the stamp; reading `msg.header.stamp` raised. Now
   `racing.health.header_stamp_seconds` accepts both layouts.
2. `frenet_planner.py`: indoors the state converter publishes NaN UTM and the
   KD-tree query raised. The planner now publishes an INELIGIBLE result with the
   reason `state_not_finite` and skips the cycle.
3. New scripts were committed without the executable bit; `colcon` sets it on
   the buggy, which dirties the tree and blocks `git checkout`. Fixed in git.

Not covered: lidar (unplugged), camera (USB), anything moving.

## What this test does and does not tell you

It tells you whether the lidar and camera produce detections at all, at what
range, and whether the tracker turns them into stable tracks. It does not test
passing, the raceline, or localization quality; those need the buggy moving
outdoors with RTK, which is the survey day in `course_survey_checklist.md`.
