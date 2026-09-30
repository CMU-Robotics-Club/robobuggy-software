#!/usr/bin/env python3
"""
course_viz.py
-------------
Turns what the racing stack knows into things Foxglove's 3D panel can draw, in a local
frame called ``course`` whose origin is the first point of the reference line (UTM
coordinates are ~4.5e6 m, which WebGL's float32 renders with visible jitter).

Static (published once, latched):
    viz/course            MarkerArray   reference line, left curb file, right edge (file or
                                        ASSUMED, drawn dashed), the hard corridor the planner
                                        allows the vehicle centre to use, 100 m station labels
    viz/reference_path    nav_msgs/Path the reference line

Dynamic (10 Hz):
    viz/live              MarkerArray   ego box + heading arrow, the plan coloured by status
                                        (green ELIGIBLE, amber DEGRADED, red INELIGIBLE), the
                                        legacy self/cur_traj line, every track (box sized by
                                        extent, 2 sigma circle, id label), simulator truth for
                                        NAND and ghosts (grey), and a text HUD with speed,
                                        steering, planner state and health
    viz/plan_path         nav_msgs/Path the current plan
    TF course -> ego                    so the 3D panel can follow the buggy

Nothing here feeds back into planning or control.
"""

import json
import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import Point, PoseArray, PoseStamped, TransformStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import ColorRGBA, Int8, String
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import TransformBroadcaster

from util.track import Track
from buggy.msg import LocalizationHealthMsg, PlanningResultMsg, StampedFloat64Msg, TrackingResultMsg, TrajectoryMsg

FRAME = "course"
STATUS_COLOR = {0: (0.6, 0.6, 0.6), 1: (0.1, 0.85, 0.2), 2: (1.0, 0.7, 0.0), 3: (0.95, 0.15, 0.15)}
STATUS_NAME = {0: "NONE", 1: "ELIGIBLE", 2: "DEGRADED", 3: "INELIGIBLE"}
HEALTH_NAME = {0: "OK", 1: "DEGRADED", 2: "BAD"}
STATE_NAME = {0: "RACELINE", 1: "PASS", 2: "REJOIN"}


def rgba(r, g, b, a=1.0):
    return ColorRGBA(r=float(r), g=float(g), b=float(b), a=float(a))


def yaw_quat(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class CourseViz(Node):
    def __init__(self):
        super().__init__("course_viz")
        self.declare_parameter("traj_name", "buggycourse_sc.json")
        self.declare_parameter("curb_name", "buggycourse_curb.json")
        self.declare_parameter("right_boundary_name", "")
        self.declare_parameter("left_width", 3.0)
        self.declare_parameter("right_width", 1.3)
        self.declare_parameter("half_width", 0.6)
        self.declare_parameter("hard_boundary_margin", 0.2)
        self.declare_parameter("vehicle_length", 2.5)
        self.declare_parameter("truth_topics", ["/NAND/self/state"])
        self.declare_parameter("secondary_traj_name", "buggycourse_sc.json")   # drawn dashed magenta when it differs
        self.declare_parameter("rate_hz", 10.0)
        p = lambda n: self.get_parameter(n).value  # noqa: E731

        import os
        trajpath = os.environ.get("TRAJPATH", "")
        lw, rw = float(p("left_width")), float(p("right_width"))
        right_name = str(p("right_boundary_name"))
        curb = str(p("curb_name"))
        self.half_width = float(p("half_width"))
        self.length = float(p("vehicle_length"))
        margin = float(p("hard_boundary_margin"))
        # raw edges (what the files / assumptions say) and the hard corridor for the vehicle centre
        self.edges = Track.from_files(trajpath + str(p("traj_name")),
                                      left_boundary_json=trajpath + curb if curb else None,
                                      right_boundary_json=trajpath + right_name if right_name else None,
                                      default_left=lw if lw > 0 else None, default_right=rw if rw > 0 else None,
                                      margin=0.0)
        self.corridor = Track.from_files(trajpath + str(p("traj_name")),
                                         left_boundary_json=trajpath + curb if curb else None,
                                         right_boundary_json=trajpath + right_name if right_name else None,
                                         default_left=lw if lw > 0 else None, default_right=rw if rw > 0 else None,
                                         margin=margin + self.half_width)
        self.origin = self.edges.xy[0].copy()
        sec = str(p("secondary_traj_name"))
        self.secondary = None
        if sec and sec != str(p("traj_name")):
            self.secondary = (sec, Track.load_waypoints_utm(trajpath + sec))
        self.get_logger().info(f"course frame origin UTM {self.origin[0]:.1f}, {self.origin[1]:.1f}; "
                               f"right edge {self.edges.right_source}, left edge {self.edges.left_source}")

        self.ego = None
        self.plan = None
        self.plan_status_json = {}
        self.tracks = None
        self.health = None
        self.legacy = None
        self.steering = None
        self.planner_state = None
        self.plan_source = None
        self.truth = {}
        self.ghosts = None

        self.create_subscription(Odometry, "self/state", self.on_state, 1)
        self.create_subscription(PlanningResultMsg, "planning/result", self.on_plan, 1)
        self.create_subscription(String, "planning/status", self.on_plan_status, 1)
        self.create_subscription(TrackingResultMsg, "perception/tracking", self.on_tracks, 1)
        self.create_subscription(LocalizationHealthMsg, "localization/health_stamped", self.on_health, 1)
        self.create_subscription(TrajectoryMsg, "self/cur_traj", self.on_legacy, 1)
        self.create_subscription(StampedFloat64Msg, "input/steering", self.on_steering, 1)
        self.create_subscription(Int8, "debug/planner/state", self.on_planner_state, 1)
        self.create_subscription(String, "controller/plan_source", self.on_plan_source, 1)
        self.create_subscription(PoseArray, "perception_sim/ghost_truth", self.on_ghosts, 1)
        for topic in [str(t) for t in p("truth_topics") if str(t).strip()]:
            self.create_subscription(Odometry, topic, lambda m, t=topic: self.truth.__setitem__(t, m), 1)

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.static_pub = self.create_publisher(MarkerArray, "viz/course", latched)
        self.ref_path_pub = self.create_publisher(Path, "viz/reference_path", latched)
        self.live_pub = self.create_publisher(MarkerArray, "viz/live", 1)
        self.plan_path_pub = self.create_publisher(Path, "viz/plan_path", 1)
        self.tf = TransformBroadcaster(self)

        self.publish_static()
        self.create_timer(1.0 / float(p("rate_hz")), self.publish_live)
        self.create_timer(5.0, self.publish_static)   # re-latch for late subscribers that missed it

    # ------------------------------------------------------------------ callbacks
    def on_state(self, m):
        self.ego = m
        x, y = m.pose.pose.position.x, m.pose.pose.position.y
        if not (math.isfinite(x) and math.isfinite(y)):
            return
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = FRAME
        t.child_frame_id = "ego"
        t.transform.translation.x = float(x - self.origin[0])
        t.transform.translation.y = float(y - self.origin[1])
        qx, qy, qz, qw = yaw_quat(float(m.pose.pose.orientation.z))
        t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = qx, qy, qz, qw
        self.tf.sendTransform(t)

    def on_plan(self, m):
        self.plan = m

    def on_plan_status(self, m):
        try:
            self.plan_status_json = json.loads(m.data)
        except ValueError:
            self.plan_status_json = {}

    def on_tracks(self, m):
        self.tracks = m

    def on_health(self, m):
        self.health = m

    def on_legacy(self, m):
        self.legacy = m

    def on_steering(self, m):
        self.steering = float(m.data)

    def on_planner_state(self, m):
        self.planner_state = int(m.data)

    def on_plan_source(self, m):
        try:
            self.plan_source = json.loads(m.data)
        except ValueError:
            self.plan_source = None

    def on_ghosts(self, m):
        self.ghosts = m

    # ------------------------------------------------------------------ helpers
    def local(self, x, y, z=0.0):
        return Point(x=float(x - self.origin[0]), y=float(y - self.origin[1]), z=float(z))

    def marker(self, ns, mid, mtype, color, scale=(0.3, 0.3, 0.3), lifetime=0.0):
        m = Marker()
        m.header.frame_id = FRAME
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns, m.id, m.type, m.action = ns, int(mid), mtype, Marker.ADD
        m.scale.x, m.scale.y, m.scale.z = [float(s) for s in scale]
        m.color = rgba(*color) if len(color) == 4 else rgba(*color, 1.0)
        m.pose.orientation.w = 1.0
        if lifetime > 0:
            m.lifetime.sec = int(lifetime)
            m.lifetime.nanosec = int((lifetime - int(lifetime)) * 1e9)
        return m

    def line(self, ns, mid, xy, color, width=0.25, z=0.0, lifetime=0.0):
        m = self.marker(ns, mid, Marker.LINE_STRIP, color, (width, 0, 0), lifetime)
        m.points = [self.local(px, py, z) for px, py in xy if math.isfinite(px) and math.isfinite(py)]
        return m

    def dashed(self, ns, mid, xy, color, width=0.25, dash=4, z=0.0):
        m = self.marker(ns, mid, Marker.LINE_LIST, color, (width, 0, 0))
        pts = []
        for i in range(0, len(xy) - 1, 2 * dash):
            seg = xy[i:i + dash + 1]
            for a, b in zip(seg[:-1], seg[1:]):
                pts += [self.local(*a, z), self.local(*b, z)]
        m.points = pts
        return m

    def text(self, ns, mid, x, y, s, color=(1, 1, 1), size=1.2, z=1.0, lifetime=0.0):
        m = self.marker(ns, mid, Marker.TEXT_VIEW_FACING, color, (0, 0, size), lifetime)
        m.pose.position = self.local(x, y, z)
        m.text = s
        return m

    def box(self, ns, mid, x, y, yaw, length, width, color, z=0.5, height=1.0, lifetime=0.0):
        m = self.marker(ns, mid, Marker.CUBE, color, (length, width, height), lifetime)
        m.pose.position = self.local(x, y, z)
        qx, qy, qz, qw = yaw_quat(yaw)
        m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w = qx, qy, qz, qw
        return m

    def circle(self, ns, mid, x, y, r, color, width=0.08, lifetime=0.0):
        th = np.linspace(0, 2 * np.pi, 40)
        return self.line(ns, mid, np.c_[x + r * np.cos(th), y + r * np.sin(th)], color, width, 0.05, lifetime)

    def path_msg(self, xy):
        path = Path()
        path.header.frame_id = FRAME
        path.header.stamp = self.get_clock().now().to_msg()
        for px, py in xy:
            if not (math.isfinite(px) and math.isfinite(py)):
                continue
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position = self.local(px, py, 0.02)
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        return path

    # ------------------------------------------------------------------ publishers
    def publish_static(self):
        e, c = self.edges, self.corridor
        arr = MarkerArray()
        arr.markers.append(self.line("reference", 0, e.xy, (1, 1, 1), 0.35))
        if self.secondary is not None:
            name, xy = self.secondary
            arr.markers.append(self.dashed("secondary_line", 0, xy, (1.0, 0.2, 1.0), 0.3, dash=3, z=0.03))
            px, py = xy[0]
            arr.markers.append(self.text("station", 998, px, py, f"dashed magenta = {name}", (1.0, 0.4, 1.0), 2.0, 2.5))
        left = e.cartesian(e.s, np.nan_to_num(e.w_left, nan=0.0))
        right = e.cartesian(e.s, -np.nan_to_num(e.w_right, nan=0.0))
        arr.markers.append(self.line("edge_left", 0, left, (1.0, 0.55, 0.1), 0.3))
        if e.right_source == "boundary_file":
            arr.markers.append(self.line("edge_right", 0, right, (1.0, 0.55, 0.1), 0.3))
        else:
            arr.markers.append(self.dashed("edge_right_assumed", 0, right, (1.0, 0.55, 0.1), 0.3))
        cl = c.cartesian(c.s, np.nan_to_num(c.w_left, nan=0.0))
        cr = c.cartesian(c.s, -np.nan_to_num(c.w_right, nan=0.0))
        arr.markers.append(self.line("hard_corridor", 0, cl, (0.3, 0.6, 1.0, 0.6), 0.12, 0.01))
        arr.markers.append(self.line("hard_corridor", 1, cr, (0.3, 0.6, 1.0, 0.6), 0.12, 0.01))
        for i, s in enumerate(np.arange(0, e.s[-1], 100.0)):
            px, py = e.cartesian(s, 0.0)[0]
            arr.markers.append(self.text("station", i, px, py, f"{s:.0f} m", (0.8, 0.8, 0.8), 2.0, 0.5))
        label = f"right edge: {'file' if e.right_source == 'boundary_file' else 'ASSUMED ' + str(self.get_parameter('right_width').value) + ' m'}"
        px, py = e.cartesian(30.0, -6.0)[0]
        arr.markers.append(self.text("station", 999, px, py, label, (1.0, 0.55, 0.1), 2.0, 0.5))
        self.static_pub.publish(arr)
        self.ref_path_pub.publish(self.path_msg(e.xy))

    def publish_live(self):
        arr = MarkerArray()
        clear = Marker()
        clear.header.frame_id = FRAME
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)
        life = 0.6
        hud = []
        if self.ego is not None:
            x, y = self.ego.pose.pose.position.x, self.ego.pose.pose.position.y
            yaw = float(self.ego.pose.pose.orientation.z)
            v = math.hypot(self.ego.twist.twist.linear.x, self.ego.twist.twist.linear.y)
            if math.isfinite(x) and math.isfinite(y):
                arr.markers.append(self.box("ego", 0, x, y, yaw, self.length, 2 * self.half_width, (0.2, 0.5, 1.0, 0.9), lifetime=life))
                arrow = self.marker("ego", 1, Marker.ARROW, (0.2, 0.5, 1.0), (3.0, 0.3, 0.3), life)
                arrow.pose.position = self.local(x, y, 1.2)
                qx, qy, qz, qw = yaw_quat(yaw)
                arrow.pose.orientation.x, arrow.pose.orientation.y, arrow.pose.orientation.z, arrow.pose.orientation.w = qx, qy, qz, qw
                arr.markers.append(arrow)
                hud.append(f"v {v:4.1f} m/s")
                if self.steering is not None:
                    hud.append(f"steer {self.steering:+5.1f} deg")
        if self.plan is not None:
            tr = self.plan.trajectory
            xy = np.c_[np.asarray(tr.easting), np.asarray(tr.northing)]
            col = STATUS_COLOR.get(int(self.plan.status), (1, 1, 1))
            if len(xy) >= 2:
                arr.markers.append(self.line("plan", 0, xy, (*col, 1.0), 0.5, 0.1, life))
                self.plan_path_pub.publish(self.path_msg(xy))
            reasons = ", ".join(self.plan.reasons[:3])
            hud.append(f"plan {int(self.plan.plan_id)} {STATUS_NAME.get(int(self.plan.status), '?')}"
                       + ("" if self.plan.control_eligible else " (controller falls back)"))
            if reasons:
                hud.append(f"  {reasons}")
            st = self.plan_status_json
            if st:
                hud.append(f"target {st.get('target_offset', 0) or 0:+.2f} m  kappa {st.get('max_curvature', 0):.3f}  "
                           f"{st.get('candidates', 0)} cand  {st.get('cycle_ms', 0):.0f} ms")
        if self.legacy is not None:
            xy = np.c_[np.asarray(self.legacy.easting), np.asarray(self.legacy.northing)]
            if len(xy) >= 2:
                arr.markers.append(self.line("legacy_plan", 0, xy, (0.9, 0.4, 0.9, 0.8), 0.2, 0.08, life))
        if self.planner_state is not None:
            hud.append(f"planner {STATE_NAME.get(self.planner_state, '?')}")
        if self.plan_source:
            hud.append(f"controller uses {self.plan_source.get('source')}"
                       + (f" ({self.plan_source.get('reason')})" if self.plan_source.get('reason') else ""))
        if self.health is not None:
            hud.append(f"localization {HEALTH_NAME.get(int(self.health.level), '?')}"
                       + (f" {', '.join(self.health.reasons[:2])}" if self.health.reasons else ""))
        for topic, m in self.truth.items():
            x, y = m.pose.pose.position.x, m.pose.pose.position.y
            if math.isfinite(x) and math.isfinite(y) and x > 1e5:
                arr.markers.append(self.box("truth", abs(hash(topic)) % 10000, x, y, float(m.pose.pose.orientation.z), 2.5, 1.2, (0.5, 0.5, 0.5, 0.5), lifetime=life))
                arr.markers.append(self.text("truth_label", abs(hash(topic)) % 10000, x, y, topic.split('/')[1], (0.7, 0.7, 0.7), 1.0, 1.8, life))
        if self.ghosts is not None:
            for i, pose in enumerate(self.ghosts.poses):
                arr.markers.append(self.box("truth", 20000 + i, pose.position.x, pose.position.y, 0.0, 2.5, 1.2, (0.5, 0.5, 0.5, 0.5), lifetime=life))
                arr.markers.append(self.text("truth_label", 20000 + i, pose.position.x, pose.position.y, f"ghost {i}", (0.7, 0.7, 0.7), 1.0, 1.8, life))
        if self.tracks is not None:
            now = self.get_clock().now().nanoseconds * 1e-9
            for t in self.tracks.tracks:
                x, y = t.position.x, t.position.y
                if not (math.isfinite(x) and math.isfinite(y)):
                    continue
                vx, vy = t.velocity.x, t.velocity.y
                yaw = math.atan2(vy, vx) if math.hypot(vx, vy) > 0.5 else 0.0
                col = (1.0, 0.2, 0.2, 0.85) if t.confirmed else (1.0, 0.8, 0.2, 0.5)
                L = max(t.extent.x, 2.5) if t.extent_known else 2.5
                W = max(t.extent.y, 1.2) if t.extent_known else 1.2
                arr.markers.append(self.box("track", int(t.id), x, y, yaw, L, W, col, z=0.6, height=1.2, lifetime=life))
                sig = math.sqrt(max(t.covariance[0], t.covariance[5], 0.0))
                arr.markers.append(self.circle("track_sigma", int(t.id), x, y, max(2 * sig, 0.2), (1.0, 0.3, 0.3, 0.7), lifetime=life))
                age = now - (t.last_observed_stamp.sec + t.last_observed_stamp.nanosec * 1e-9)
                arr.markers.append(self.text("track_label", int(t.id), x, y,
                                             f"#{t.id} {'ok' if t.confirmed else 'tentative'} {math.hypot(vx, vy):.1f} m/s "
                                             f"2s {2*sig:.1f} m age {age:.1f}s {'+'.join(t.sources)}",
                                             (1, 0.85, 0.85), 0.9, 2.4, life))
            hud.append(f"tracker {len(self.tracks.tracks)} tracks, perception_ready {self.tracks.perception_ready}")
        if self.ego is not None and hud:
            x, y = self.ego.pose.pose.position.x, self.ego.pose.pose.position.y
            if math.isfinite(x) and math.isfinite(y):
                arr.markers.append(self.text("hud", 0, x, y, "\n".join(hud), (1, 1, 1), 1.1, 6.0, life))
        self.live_pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = CourseViz()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
