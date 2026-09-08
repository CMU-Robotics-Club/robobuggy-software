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
- The RC transmitter's autonomous-steering switch stays OFF the whole time. The
  firmware only obeys software steering when that switch is on, so with it off
  nothing here can move the wheel.
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

Pane 1, sensors and infrastructure:

```bash
ros2 launch buggy sc-system.xml
```

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
two people at once (two ids); a person walking behind a pillar and back (does
the id survive the 1.5 s blackout?); NAND pushed slowly past if it is around.
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

At home, inside the container, replay it through the same nodes:

```bash
ros2 bag play bags/bench_<stamp> --clock &
ros2 run buggy opponent_tracker.py --ros-args -r __ns:=/SC
```

and open Foxglove on `ws://localhost:8765` as usual. Replaying is how you tune
the tracker without going back to the workshop.

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
