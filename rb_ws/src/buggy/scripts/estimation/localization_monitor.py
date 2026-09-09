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

The experimental planner consumes localization/health_stamped with source time
and expiry. The Int8 topic remains a legacy diagnostic, not an authorization.

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
from buggy.msg import LocalizationHealthMsg
from rclpy.duration import Duration

from racing.health import HealthPolicy, evaluate_health, stamp_seconds


class LocalizationMonitor(Node):
    def __init__(self):
        super().__init__("localization_monitor")
        self.declare_parameter("state_topic", "self/state")
        self.declare_parameter("max_state_age_s", 0.3)
        self.declare_parameter("pos_std_ok_m", 0.5)        # below: OK (if fix is good)
        self.declare_parameter("pos_std_bad_m", 2.0)       # above: BAD
        self.declare_parameter("require_rtk_for_ok", True)
        self.declare_parameter("require_filter_for_ok", True)
        self.declare_parameter("max_quality_age_s", 1.0)
        self.declare_parameter("health_validity_s", 0.3)
        self.declare_parameter("rate_hz", 5.0)

        p = lambda n: self.get_parameter(n).value  # noqa: E731
        self.max_age = float(p("max_state_age_s"))
        self.std_ok = float(p("pos_std_ok_m"))
        self.std_bad = float(p("pos_std_bad_m"))
        self.require_rtk = bool(p("require_rtk_for_ok"))
        self.policy = HealthPolicy(self.max_age, float(p("max_quality_age_s")),
                                   self.std_ok, self.std_bad, self.require_rtk,
                                   bool(p("require_filter_for_ok")))
        self.validity = float(p("health_validity_s"))

        self.state = None
        self.state_time = None
        self.fix_type = None        # from GNSSFixInfo: 0 3D,1 2D,2 time only,3 none,4 invalid,5 RTK float,6 RTK fixed
        self.filter_state = None    # from FilterStatus: 1 init, 2 vertical gyro, 3 AHRS, 4 full nav
        self.fix_stamp = None
        self.filter_stamp = None

        self.create_subscription(Odometry, p("state_topic"), self.on_state, 1)
        self._try_subscribe_microstrain()

        self.health_pub = self.create_publisher(Int8, "localization/health", 1)
        self.status_pub = self.create_publisher(String, "localization/status", 1)
        self.stamped_pub = self.create_publisher(LocalizationHealthMsg, "localization/health_stamped", 1)
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
            self.get_logger().warn(f"microstrain messages unavailable ({e}); required quality remains unknown")
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
        self.state_time = stamp_seconds(msg.header.stamp)

    def on_fix(self, msg):
        v = getattr(msg, "fix_type", None)
        if v is not None:
            self.fix_type = int(v)
            self.fix_stamp = stamp_seconds(msg.header.stamp) if hasattr(msg, "header") else None

    def on_filter(self, msg):
        v = getattr(msg, "filter_state", None)
        if v is not None:
            self.filter_state = int(v)
            self.filter_stamp = stamp_seconds(msg.header.stamp) if hasattr(msg, "header") else None

    def evaluate(self):
        now = self.get_clock().now()
        pose, cov = None, None
        if self.state is not None:
            p = self.state.pose.pose.position
            q = self.state.pose.pose.orientation
            pose = [p.x, p.y, p.z, q.x, q.y, q.z, q.w]
            cov = self.state.pose.covariance
        level, reasons, std = evaluate_health(
            now.nanoseconds * 1e-9, pose, cov, self.state_time,
            self.fix_type, self.fix_stamp, self.filter_state, self.filter_stamp, self.policy)

        stamped = LocalizationHealthMsg()
        stamped.header.stamp = now.to_msg()
        if self.state is not None:
            stamped.header.frame_id = self.state.header.frame_id
            stamped.state_stamp = self.state.header.stamp
        stamped.valid_until = (now + Duration(seconds=self.validity)).to_msg()
        stamped.level = level
        stamped.reasons = reasons
        self.stamped_pub.publish(stamped)

        self.health_pub.publish(Int8(data=level))
        self.status_pub.publish(String(data=json.dumps({
            "health": level,
            "fix_type": self.fix_type,
            "filter_state": self.filter_state,
            "position_std_m": std,
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
