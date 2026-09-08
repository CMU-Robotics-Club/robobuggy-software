#!/usr/bin/env python3
"""
localization_monitor.py
-----------------------
Watches how good our own position estimate is and publishes one number the
rest of the stack can act on:

    localization/health   std_msgs/Int8
        0 = OK        RTK fixed or float, filter running, fresh, covariance small
        1 = DEGRADED  single-point GNSS or covariance above the "no passing" threshold
        2 = BAD       stale state, filter not running, or covariance above the "hold line" threshold

    localization/status   std_msgs/String  JSON with the reasons, for Foxglove.

The Frenet planner refuses new passes at 1 and holds the raceline at 2. The
controller's init check still guards the start; this node covers the whole run.

Inputs (all optional; the node degrades gracefully when a topic is absent):
    self/state                     nav_msgs/Odometry, position covariance and freshness
    /ekf/status                    microstrain_inertial_msgs/FilterStatus (filter_state, dynamics_mode)
    /gnss_1/fix_info               microstrain_inertial_msgs/GNSSFixInfo (fix_type)
    /mip/gnss_1/fix_info           newer driver name for the same message
The microstrain message types are imported lazily so the node also runs in the
simulator where the driver package is installed but nothing publishes.
"""

import json

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Int8, String


class LocalizationMonitor(Node):
    def __init__(self):
        super().__init__("localization_monitor")
        self.declare_parameter("state_topic", "self/state")
        self.declare_parameter("max_state_age_s", 0.3)
        self.declare_parameter("pos_std_ok_m", 0.5)        # below: OK (if fix is good)
        self.declare_parameter("pos_std_bad_m", 2.0)       # above: BAD
        self.declare_parameter("require_rtk_for_ok", False)  # set true once RTK corrections are wired up
        self.declare_parameter("rate_hz", 5.0)

        p = lambda n: self.get_parameter(n).value  # noqa: E731
        self.max_age = float(p("max_state_age_s"))
        self.std_ok = float(p("pos_std_ok_m"))
        self.std_bad = float(p("pos_std_bad_m"))
        self.require_rtk = bool(p("require_rtk_for_ok"))

        self.state = None
        self.state_time = None
        self.fix_type = None        # from GNSSFixInfo: 0 3D,1 2D,2 time only,3 none,4 invalid,5 RTK float,6 RTK fixed
        self.filter_state = None    # from FilterStatus: 1 init, 2 vertical gyro, 3 AHRS, 4 full nav
        self.have_sim_state = False

        self.create_subscription(Odometry, p("state_topic"), self.on_state, 1)
        self._try_subscribe_microstrain()

        self.health_pub = self.create_publisher(Int8, "localization/health", 1)
        self.status_pub = self.create_publisher(String, "localization/status", 1)
        self.create_timer(1.0 / float(p("rate_hz")), self.evaluate)

    def _try_subscribe_microstrain(self):
        """
        Subscribe to the GQ7 driver's fix and filter status. The message names changed
        between driver generations (GNSSFixInfo/FilterStatus in 3.x, MipGnssFixInfo/
        MipFilterStatus in 4.x), so try both and skip quietly when neither exists.
        """
        try:
            import importlib
            mod = importlib.import_module("microstrain_inertial_msgs.msg")
        except ImportError as e:
            self.get_logger().warn(f"microstrain messages unavailable ({e}); using covariance only")
            return
        fix_cls = next((getattr(mod, n) for n in ("MipGnssFixInfo", "GNSSFixInfo") if hasattr(mod, n)), None)
        filt_cls = next((getattr(mod, n) for n in ("MipFilterStatus", "FilterStatus") if hasattr(mod, n)), None)
        if fix_cls is not None:
            for topic in ("/gnss_1/fix_info", "/mip/gnss_1/fix_info", "/mip/gnss1/fix_info"):
                self.create_subscription(fix_cls, topic, self.on_fix, 1)
        if filt_cls is not None:
            for topic in ("/ekf/status", "/mip/ekf/status", "/mip/filter/status"):
                self.create_subscription(filt_cls, topic, self.on_filter, 1)
        self.get_logger().info(
            f"microstrain status: fix msg {'yes' if fix_cls else 'no'}, filter msg {'yes' if filt_cls else 'no'}"
        )

    def on_state(self, msg):
        self.state = msg
        self.state_time = self.get_clock().now()

    def on_fix(self, msg):
        v = getattr(msg, "fix_type", None)
        if v is not None:
            self.fix_type = int(v)

    def on_filter(self, msg):
        v = getattr(msg, "filter_state", None)
        if v is not None:
            self.filter_state = int(v)

    def evaluate(self):
        reasons = []
        level = 0

        if self.state is None or self.state_time is None:
            level = 2
            reasons.append("no state yet")
        else:
            age = (self.get_clock().now() - self.state_time).nanoseconds * 1e-9
            if age > self.max_age:
                level = 2
                reasons.append(f"state stale {age:.2f}s")
            c = self.state.pose.covariance
            pos_std = (max(c[0], 0.0) + max(c[7], 0.0)) ** 0.5
            if pos_std > self.std_bad:
                level = max(level, 2)
                reasons.append(f"pos std {pos_std:.2f}m > {self.std_bad}")
            elif pos_std > self.std_ok:
                level = max(level, 1)
                reasons.append(f"pos std {pos_std:.2f}m > {self.std_ok}")

        if self.filter_state is not None and self.filter_state != 4:
            level = max(level, 2)
            reasons.append(f"INS filter state {self.filter_state} (need 4 = full nav)")

        if self.fix_type is not None:
            if self.fix_type in (2, 3, 4):
                level = max(level, 2)
                reasons.append(f"GNSS fix type {self.fix_type} (no position fix)")
            elif self.fix_type not in (5, 6) and self.require_rtk:
                level = max(level, 1)
                reasons.append(f"GNSS fix type {self.fix_type} is not RTK")

        self.health_pub.publish(Int8(data=level))
        self.status_pub.publish(String(data=json.dumps({
            "health": level,
            "fix_type": self.fix_type,
            "filter_state": self.filter_state,
            "reasons": reasons,
        })))


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
