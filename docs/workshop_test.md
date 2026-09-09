# Workshop test: running this branch on the real buggy

How to get `feature/racing-perception-planning` onto Short Circuit, run the
perception chain with the buggy standing still, look at the result, and bring
the data home. Nothing in this test commands steering.

Assumptions from the repo's `bootstrap/` files and README: the buggy computer
is reachable on the **ShortCircuit** Wi-Fi as `nuc@192.168.1.217`, the repo is
checked out at `~/robobuggy-software`, ROS 2 Humble is installed natively (no
Docker on the buggy), the Python packages live in a virtualenv at
`~/robobuggy-software/.sc`, and a systemd service called `buggy` starts the
stack in a tmux session at boot. If the team has moved to the Jetson Orin, ask
them for the new address and user; everything else is the same.

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

```bash
ssh nuc@192.168.1.217
```

Look around before changing anything:

```bash
systemctl status buggy          # is the stack already running?
tmux ls                         # the "buggy" session, if so
cd ~/robobuggy-software && git status && git branch --show-current
```

The service starts `sc-system.xml` and `sc-main.xml` at boot. For a perception
bench test you want the sensors but not the controller and planner, so stop the
service and start the sensors yourself:

```bash
sudo systemctl stop buggy
tmux kill-session -t buggy 2>/dev/null
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
git remote add buggy nuc@192.168.1.217:robobuggy-software
git push buggy feature/racing-perception-planning
```

then check the branch out on the buggy as above. Either way, note which branch
the buggy was on before (`rolls`, most likely) so you can put it back.

## 3. Install the extra packages and build

```bash
cd ~/robobuggy-software
source .sc/bin/activate
pip install -r perception-requirements.txt      # open3d, scikit-learn, ultralytics
sudo apt install ros-humble-velodyne             # lidar driver, if not already there
cd rb_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/local_setup.bash
source environments/sc_env.bash
```

The ZED camera needs the ZED SDK and its Python package `pyzed`, which the
vision people installed for `detector_node.py`. If `python3 -c "import pyzed"`
fails, run the bench test with `use_camera:=false` and sort the camera out
separately.

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
scp -r nuc@192.168.1.217:robobuggy-software/rb_ws/bags/bench_* ./rb_ws/bags/
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
cd rb_ws && colcon build --symlink-install
sudo systemctl start buggy
```

## What this test does and does not tell you

It tells you whether the lidar and camera produce detections at all, at what
range, and whether the tracker turns them into stable tracks. It does not test
passing, the raceline, or localization quality; those need the buggy moving
outdoors with RTK, which is the survey day in `course_survey_checklist.md`.
