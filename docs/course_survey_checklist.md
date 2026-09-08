# Course survey checklist

Goal: one afternoon on the course that produces the exact centre line, both road
edges, the elevation profile, and real lidar / camera / GNSS logs. Those files
feed the raceline optimizer, the planner's road limits, the speed model, and the
first real test of the perception pipeline. Everything after the survey is only
as good as this data, so do it with RTK fixed or do not do it.

## A. The week before

- [ ] **RTK working.** `INS_params.yml`: `aux_port` points at the pinned AUX device
      (udev rule), `ntrip_interface_enable: True`, GLONASS and BeiDou enabled.
      `ntrip_client` configured with the campus caster host, mount point, and login
      (Radio Club / Robobuggy base station). Jetson has a hotspot for the caster.
- [ ] **Proof:** in the garage with sky view, `ros2 topic echo /SC/localization/status`
      shows `"fix_type": 6` (RTK fixed) within a few minutes of power-up. 5 = float
      is not good enough for boundaries; 0 = plain GPS means the survey is wasted.
- [ ] **Antenna lever arms** in `INS_params.yml` re-measured with a tape (within 5 cm).
- [ ] **Clock:** Jetson time within a second of phone time (`date`). Better: chrony
      fed by the GQ7 NMEA. Bag timestamps and SVO timestamps must line up later.
- [ ] **Disk:** lidar is about 400 MB per minute. A 15 minute rolled lap plus two
      walked edges without lidar is roughly 8 GB. Check `df -h` on the Jetson.
- [ ] **Lidar:** Velodyne web page set to 600 rpm (10 Hz) and saved; the launch's
      `rpm` matches; `ros2 topic hz /velodyne_points` reads ~10 Hz.
- [ ] **Camera:** `record_frames.py` starts and writes an SVO in `rb_ws/svo_files/`.
      Make sure the ZED lens is clean and the mount is tight.
- [ ] **Dry run of the recorder** in the garage:
      `ros2 launch buggy record_course.xml pass:=test` for 60 s, then
      `ros2 bag info rb_ws/bags/test_*` shows all topic groups with sane counts.
- [ ] Charge everything. Bring the AUX cable, a spare, tape measure, laptop.

## B. On the course: three passes

Start the normal stack first (`sc-system.xml`) so `/SC/self/state`, the GQ7 topics
and the lidar are live. Then, for each pass, start a fresh recorder with the right
label and stop it with Ctrl-C at the end. Watch the fix type printed in the
recorder terminal; if it drops below 6 for more than a few seconds, note where.

1. **`pass:=center`, the driving line.** Roll the course at rolling speed exactly
   as in a race, pushers and all. Also start `record_frames.py` for the camera
   before pushing off. This pass gives the centre line, the elevation profile,
   the speed profile for calibrating the speed model, and lidar/camera data at
   real speed.
2. **`pass:=left_curb record_lidar:=false`, the left limit.** Walk the buggy slowly
   with the left wheels 30 cm from the curb (or the hay bale line in the chute).
   Walk, do not roll: slower is straighter. Keep a constant offset; the tool
   subtracts nothing.
3. **`pass:=right_edge record_lidar:=false`, the right limit.** Same, along the
   legal right edge (edge of asphalt, cones, or the line the team agrees is legal).

Optional fourth pass: `pass:=roll` for a second rolled lap on a different line.
Two rolled laps are much better than one for fitting the speed model.

During every pass write down in a notebook: time of day, weather, where the fix
dropped, anything unusual on the surface (patches, bollards, mud, bales).

## C. Back at the lab

Copy `rb_ws/bags/` and `rb_ws/svo_files/` off the Jetson. Then, in the container:

```bash
# centre line (also writes the elevation profile from this pass)
python3 src/buggy/scripts/util/bag_to_course.py --bag bags/center_<stamp> \
    --kind centerline --out buggycourse_survey_center.json --require-rtk

# boundaries
python3 src/buggy/scripts/util/bag_to_course.py --bag bags/left_curb_<stamp> \
    --kind left --out buggycourse_survey_left.json --require-rtk
python3 src/buggy/scripts/util/bag_to_course.py --bag bags/right_edge_<stamp> \
    --kind right --out buggycourse_survey_right.json --require-rtk
```

Each run prints the length, the RTK fix histogram, the largest gap between
samples, and where it wrote the files. Then:

- [ ] Open the three JSON files in the eracer portal or Foxglove and eyeball them
      against the satellite map. Fix any gaps by hand if the fix dropped.
- [ ] Re-run the raceline optimizer with `--center buggycourse_survey_center.json
      --left-boundary buggycourse_survey_left.json --right-boundary
      buggycourse_survey_right.json --zones config/course_zones.yaml`.
- [ ] Point `course_zones.yaml` at the new `config/course_elevation.csv`
      (`elevation_profile:` key) and re-run the lap-time estimate.
- [ ] Update `traj_name`, `curb_name`, and `right_boundary_name` in the configs.
- [ ] Replay the centre pass through the perception stack:
      `ros2 bag play bags/center_<stamp>` with `opponent_tracker.py` and the lidar
      nodes running, and watch `perception/tracks` in Foxglove. Note what it sees
      and what it misses. That is the first real perception result.

## D. What the survey does not do

It does not measure the steering motor's slew rate or latency (do that in the
garage with `debug/roundtrip_time` and `debug/control_latency`), and it does not
calibrate the tyre scrub loss unless a rolled pass includes a full-speed chute.
Both are on the hardware list in `docs/racing_stack.md`.
