#!/usr/bin/env python3
"""
lidar_sim.py
------------
A simulated Velodyne VLP-16 for the 2D simulator. Ray-casts 16 beams against a 3D world
built from the course files and publishes a real sensor_msgs/PointCloud2, so the 3D view
shows what the buggy's lidar would see and, with use_lidar_sim:=true in sim_2d_double.xml,
the REAL clustering node (buggy_lidar.py) and adapter (lidar_opponent_node.py) run on it
instead of perception_sim's shortcut detections.

World (all from the reference line, the left curb file and the assumed right edge):
    road surface          z = 0, between the edges
    curbs                 0.15 m step at both edges; sidewalk 0.15 m high outside them
    hedge / hillside      1.3 m wall 3.5 m outside the left curb, 1.0 m wall 2.5 m outside
                          the right edge
    trees                 trunks + canopies every ~11 m left and ~14 m right
    other buggies         2.5 x 1.2 x 1.0 m boxes at the simulator's truth poses
                          (NAND on /NAND/self/state, ghosts on perception_sim/ghost_truth)

Sensor: 16 beams -15..+15 deg, azimuth_step_deg apart, at (lidar_x, lidar_y, lidar_z) on the
buggy, published in frame ``velodyne`` with the lidar branch's convention (sensor -X is the
buggy's forward, ``sensor_forward_axis: neg_x``) and a static TF ego -> velodyne so Foxglove
places the cloud on the course. Range noise 2 cm, 2 % dropouts. The right edge is an
ASSUMPTION in the planner and it is an assumption here too: the sim curb stands where the
config says the road ends, nowhere the survey has been.
"""

import math
import os
import time

# one thread: the ray casts are many small array ops, and a multi-threaded BLAS spends more time
# handing them out than doing them (436 % CPU for one 10 Hz node on a 24-core laptop)
for _v in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS"):
    os.environ.setdefault(_v, "1")

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from tf2_ros import StaticTransformBroadcaster

from util.track import Track

I_GROUND, I_SIDEWALK, I_CURB, I_WALL, I_TREE, I_BOX = 10.0, 25.0, 45.0, 70.0, 90.0, 220.0


def ray_segments(o, dirs2, dz, z0, P, Q, z_lo, z_hi):
    """First hit distance of each ray with vertical walls on segments P->Q (2D), or inf."""
    if len(P) == 0:
        return np.full(len(dirs2), np.inf)
    E = Q - P                                     # (m,2)
    OP = P - o                                    # (m,2)
    dx, dy = dirs2[:, 0:1], dirs2[:, 1:2]          # (n,1)
    denom = dx * E[:, 1] - dy * E[:, 0]            # (n,m)
    with np.errstate(divide="ignore", invalid="ignore"):
        t = (OP[:, 0] * E[:, 1] - OP[:, 1] * E[:, 0]) / denom
        u = (OP[:, 0] * dy - OP[:, 1] * dx) / denom
    z = z0 + t * dz[:, None]
    ok = (np.abs(denom) > 1e-9) & (t > 0.05) & (u >= 0) & (u <= 1) & (z >= z_lo) & (z <= z_hi)
    t = np.where(ok, t, np.inf)
    return t.min(axis=1)


def ray_cylinders(o, dirs2, dz, z0, C, r, h):
    """First hit with vertical cylinders (centres C, radii r, heights h from z=0)."""
    if len(C) == 0:
        return np.full(len(dirs2), np.inf)
    OC = o - C                                    # (m,2)
    a = (dirs2 ** 2).sum(axis=1)[:, None]          # (n,1)
    b = 2 * (dirs2[:, 0:1] * OC[:, 0] + dirs2[:, 1:2] * OC[:, 1])
    c = (OC ** 2).sum(axis=1) - r ** 2
    disc = b * b - 4 * a * c
    with np.errstate(invalid="ignore"):
        t = (-b - np.sqrt(disc)) / (2 * a)
    z = z0 + t * dz[:, None]
    ok = (disc > 0) & (t > 0.05) & (z >= 0) & (z <= h)
    return np.where(ok, t, np.inf).min(axis=1)


def ray_spheres(o3, dirs3, C3, r):
    if len(C3) == 0:
        return np.full(len(dirs3), np.inf)
    OC = o3 - C3                                  # (m,3)
    b = 2 * dirs3 @ OC.T                          # (n,m)
    c = (OC ** 2).sum(axis=1) - r ** 2
    disc = b * b - 4 * c
    with np.errstate(invalid="ignore"):
        t = (-b - np.sqrt(disc)) / 2
    ok = (disc > 0) & (t > 0.05)
    return np.where(ok, t, np.inf).min(axis=1)


class LidarSim(Node):
    def __init__(self):
        super().__init__("lidar_sim")
        self.declare_parameter("traj_name", "buggycourse_sc.json")
        self.declare_parameter("curb_name", "buggycourse_curb.json")
        self.declare_parameter("right_boundary_name", "")
        self.declare_parameter("left_width", 3.0)
        self.declare_parameter("right_width", 1.3)
        self.declare_parameter("lidar_x", 0.9)
        self.declare_parameter("lidar_y", 0.0)
        self.declare_parameter("lidar_z", 1.0)
        self.declare_parameter("sensor_forward_axis", "neg_x")
        self.declare_parameter("frame_id", "velodyne")
        self.declare_parameter("cloud_topic", "/velodyne_points")
        self.declare_parameter("rate_hz", 10.0)
        self.declare_parameter("azimuth_step_deg", 1.0)
        self.declare_parameter("max_range_m", 60.0)
        self.declare_parameter("range_noise_m", 0.02)
        self.declare_parameter("dropout_prob", 0.02)
        self.declare_parameter("truth_topics", ["/NAND/self/state"])
        self.declare_parameter("seed", 7)
        p = lambda n: self.get_parameter(n).value  # noqa: E731

        tp = os.environ.get("TRAJPATH", "")
        lw, rw = float(p("left_width")), float(p("right_width"))
        curb, right_name = str(p("curb_name")), str(p("right_boundary_name"))
        self.track = Track.from_files(tp + str(p("traj_name")),
                                      left_boundary_json=tp + curb if curb else None,
                                      right_boundary_json=tp + right_name if right_name else None,
                                      default_left=lw if lw > 0 else None, default_right=rw if rw > 0 else None,
                                      margin=0.0)
        self.rng = np.random.default_rng(int(p("seed")))
        self.lidar_off = np.array([float(p("lidar_x")), float(p("lidar_y"))])
        self.z0 = float(p("lidar_z"))
        self.flip = str(p("sensor_forward_axis")).lower() == "neg_x"
        self.frame = str(p("frame_id"))
        self.max_range = float(p("max_range_m"))
        self.noise = float(p("range_noise_m"))
        self.dropout = float(p("dropout_prob"))
        self.build_world()
        self.build_rays(float(p("azimuth_step_deg")))

        self.ego = None
        self.truth = {}
        self.ghosts = None
        self.create_subscription(Odometry, "self/state", self.on_state, 1)
        self.create_subscription(PoseArray, "perception_sim/ghost_truth", self.on_ghosts, 1)
        for topic in [str(t) for t in p("truth_topics") if str(t).strip()]:
            self.create_subscription(Odometry, topic, lambda m, t=topic: self.truth.__setitem__(t, m), 1)
        self.pub = self.create_publisher(PointCloud2, str(p("cloud_topic")), 1)
        self.static_tf = StaticTransformBroadcaster(self)
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = "ego"
        tf.child_frame_id = self.frame
        tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = float(self.lidar_off[0]), float(self.lidar_off[1]), self.z0
        yaw = math.pi if self.flip else 0.0
        tf.transform.rotation.z, tf.transform.rotation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        self.static_tf.sendTransform(tf)
        self.create_timer(1.0 / float(p("rate_hz")), self.scan)
        self.get_logger().info(f"lidar sim: {len(self.dirs3)} rays/scan, world {len(self.walls)} wall sets, "
                               f"{len(self.trees)} trees, forward axis {'-X' if self.flip else '+X'}")

    # ------------------------------------------------------------------ world
    def build_world(self):
        t = self.track
        wl = np.nan_to_num(t.w_left, nan=3.0)
        wr = np.nan_to_num(t.w_right, nan=1.3)
        s = t.s
        def poly(off):
            return t.cartesian(s, off)
        left, right = poly(wl), poly(-wr)
        hedge, hill = poly(wl + 3.5), poly(-wr - 2.5)
        # 3 m chords (5 cm sagitta on the course's tightest 21 m radius): a third of the segments,
        # which is most of the per-scan cost
        left, right, hedge, hill = (a[::3].astype(np.float32) for a in (left, right, hedge, hill))
        # (P, Q, z_lo, z_hi, intensity)
        self.walls = [
            (left[:-1], left[1:], 0.0, 0.15, I_CURB),
            (right[:-1], right[1:], 0.0, 0.15, I_CURB),
            (hedge[:-1], hedge[1:], 0.15, 1.45, I_WALL),
            (hill[:-1], hill[1:], 0.15, 1.15, I_WALL),
        ]
        self.wall_mid = [0.5 * (P.astype(np.float64) + Q.astype(np.float64)) for P, Q, *_ in self.walls]
        trees = []
        for st in np.arange(6.0, t.length, 11.0):
            j = np.clip(int(st), 0, len(s) - 1)
            xy = t.cartesian(st, wl[j] + 2.0 + 0.6 * math.sin(st))[0]
            trees.append((xy[0], xy[1], 0.22, 4.5 + 1.5 * (0.5 + 0.5 * math.cos(st))))
        for st in np.arange(9.0, t.length, 14.0):
            j = np.clip(int(st), 0, len(s) - 1)
            xy = t.cartesian(st, -wr[j] - 1.6 - 0.4 * math.cos(st))[0]
            trees.append((xy[0], xy[1], 0.2, 3.5 + 1.2 * (0.5 + 0.5 * math.sin(st))))
        self.trees = np.array(trees) if trees else np.zeros((0, 4))
        self.canopies = np.c_[self.trees[:, 0], self.trees[:, 1], self.trees[:, 3] + 1.2] if len(trees) else np.zeros((0, 3))
        self.canopy_r = 1.6 + 0.4 * (self.trees[:, 3] - 3.5) if len(trees) else np.zeros(0)

    def build_rays(self, az_step):
        elev = np.deg2rad(np.arange(-15.0, 15.01, 2.0))           # 16 beams
        az = np.deg2rad(np.arange(0.0, 360.0, az_step))
        E, A = np.meshgrid(elev, az, indexing="ij")
        ce = np.cos(E)
        self.dirs_body = np.c_[(ce * np.cos(A)).ravel(), (ce * np.sin(A)).ravel(), np.sin(E).ravel()]
        self.ring = np.repeat(np.arange(16, dtype=np.float32), len(az))
        self.dirs3 = self.dirs_body   # rotated per scan

    # ------------------------------------------------------------------ callbacks
    def on_state(self, m):
        self.ego = m

    def on_ghosts(self, m):
        self.ghosts = m

    def opponents(self):
        """(x, y, yaw) of every other buggy from the simulator's truth."""
        out = []
        for m in self.truth.values():
            x, y = m.pose.pose.position.x, m.pose.pose.position.y
            if math.isfinite(x) and math.isfinite(y) and x > 1e5:
                out.append((x, y, float(m.pose.pose.orientation.z)))
        if self.ghosts is not None:
            for pose in self.ghosts.poses:
                s, _ = self.track.frenet(pose.position.x, pose.position.y)
                out.append((pose.position.x, pose.position.y, float(self.track.heading_at(s))))
        return out

    # ------------------------------------------------------------------ scan
    def scan(self):
        if self.ego is None:
            return
        t_start = time.perf_counter()
        ex, ey = self.ego.pose.pose.position.x, self.ego.pose.pose.position.y
        yaw = float(self.ego.pose.pose.orientation.z)
        if not (math.isfinite(ex) and math.isfinite(ey) and ex > 1e5):
            return
        c, s = math.cos(yaw), math.sin(yaw)
        o = np.array([ex + c * self.lidar_off[0] - s * self.lidar_off[1], ey + s * self.lidar_off[0] + c * self.lidar_off[1]])
        o32 = o.astype(np.float32)
        R = np.array([[c, -s], [s, c]])
        d2 = (self.dirs_body[:, :2] @ R.T).astype(np.float32)   # world-frame horizontal directions
        dz = self.dirs_body[:, 2].astype(np.float32)
        n = len(d2)
        best = np.full(n, np.inf)
        inten = np.zeros(n, dtype=np.float32)

        def take(t, value):
            nonlocal best, inten
            hit = t < best
            best = np.where(hit, t, best)
            inten = np.where(hit, value, inten)

        # ground and sidewalk planes
        with np.errstate(divide="ignore", invalid="ignore"):
            t_ground = np.where(dz < -1e-6, -self.z0 / dz, np.inf)
            t_side = np.where(dz < -1e-6, (0.15 - self.z0) / dz, np.inf)
        gx = o[0] + t_ground * d2[:, 0]
        gy = o[1] + t_ground * d2[:, 1]
        fin = np.isfinite(t_ground) & (t_ground < self.max_range)
        on_road = np.zeros(n, dtype=bool)
        if fin.any():
            sg, dg = self.track.frenet(gx[fin], gy[fin])
            wl, wr = self.track.width_at(sg)
            wl = np.nan_to_num(wl, nan=3.0)
            wr = np.nan_to_num(wr, nan=1.3)
            on_road[fin] = (dg <= wl) & (dg >= -wr)
        take(np.where(on_road, t_ground, np.inf), I_GROUND)
        take(np.where(fin & ~on_road, t_side, np.inf), I_SIDEWALK)
        # walls near the sensor
        for (P, Q, zlo, zhi, val), mid in zip(self.walls, self.wall_mid):
            near = np.hypot(mid[:, 0] - o[0], mid[:, 1] - o[1]) < self.max_range + 2.0
            take(ray_segments(np.zeros(2, np.float32), d2, dz, self.z0, (P[near] - o).astype(np.float32), (Q[near] - o).astype(np.float32), zlo, zhi), val)
        # trees
        if len(self.trees):
            near = np.hypot(self.trees[:, 0] - o[0], self.trees[:, 1] - o[1]) < self.max_range
            T = self.trees[near]
            take(ray_cylinders(o, d2, dz, self.z0, T[:, :2], T[:, 2], T[:, 3]), I_TREE)
            o3 = np.array([o[0], o[1], self.z0])
            dirs3 = np.c_[d2, dz]
            take(ray_spheres(o3, dirs3, self.canopies[near], self.canopy_r[near]), I_TREE)
        # other buggies: four walls and a roof
        for (bx, by, byaw) in self.opponents():
            if math.hypot(bx - o[0], by - o[1]) > self.max_range:
                continue
            cb, sb = math.cos(byaw), math.sin(byaw)
            L, W, H = 1.25, 0.6, 1.0
            corners = np.array([[L, W], [-L, W], [-L, -W], [L, -W]]) @ np.array([[cb, sb], [-sb, cb]]) + np.array([bx, by])
            P = corners
            Q = np.roll(corners, -1, axis=0)
            take(ray_segments(o, d2, dz, self.z0, P, Q, 0.0, H), I_BOX)
            with np.errstate(divide="ignore", invalid="ignore"):
                t_roof = np.where(dz < -1e-6, (H - self.z0) / dz, np.inf)
            rx = o[0] + t_roof * d2[:, 0] - bx
            ry = o[1] + t_roof * d2[:, 1] - by
            lx = cb * rx + sb * ry
            ly = -sb * rx + cb * ry
            inside = (np.abs(lx) <= L) & (np.abs(ly) <= W)
            take(np.where(inside, t_roof, np.inf), I_BOX)

        hit = np.isfinite(best) & (best < self.max_range)
        if self.dropout > 0:
            hit &= self.rng.random(n) > self.dropout
        rng = best[hit] + self.rng.normal(0.0, self.noise, hit.sum())
        # sensor frame: body-frame direction * range, then the lidar branch's -X-forward convention
        pts = self.dirs_body[hit] * rng[:, None]
        if self.flip:
            pts[:, 0] *= -1.0
            pts[:, 1] *= -1.0
        self.publish(pts.astype(np.float32), inten[hit], self.ring[hit])
        self.get_logger().info(f"scan: {int(hit.sum())} points in {1000 * (time.perf_counter() - t_start):.0f} ms",
                               throttle_duration_sec=5.0)

    def publish(self, pts, inten, ring):
        n = len(pts)
        data = np.empty(n, dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("intensity", "<f4"), ("ring", "<f4")])
        data["x"], data["y"], data["z"] = pts[:, 0], pts[:, 1], pts[:, 2]
        data["intensity"], data["ring"] = inten, ring
        msg = PointCloud2()
        msg.header = Header(stamp=self.get_clock().now().to_msg(), frame_id=self.frame)
        msg.height, msg.width = 1, n
        msg.fields = [PointField(name=nm, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                      for i, nm in enumerate(("x", "y", "z", "intensity", "ring"))]
        msg.is_bigendian = False
        msg.point_step = 20
        msg.row_step = 20 * n
        msg.is_dense = True
        msg.data = data.tobytes()
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LidarSim()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
