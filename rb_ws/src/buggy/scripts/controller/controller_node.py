#!/usr/bin/env python3
"""
controller_node.py
------------------
Runs the Stanley controller on the current trajectory and publishes the
steering command.

Two ways to receive a trajectory:

  * Legacy (planningResultTopic empty, the default): TrajectoryMsg on
    trajectoryTopic, followed as-is. This path is unchanged from the team's
    stack.

  * Experimental (planningResultTopic set, DECISIONS.md D1/D9): the planner's
    PlanningResultMsg envelope is the only authorization. A plan is used only
    while control_eligible is true, its status is ELIGIBLE or DEGRADED, its
    valid_until has not passed and it is younger than maxPathAgeS. The check
    runs every control cycle, including when the planner goes silent. When no
    plan qualifies the controller falls back to the static reference
    trajectory it was started with (the legacy behaviour) and says so on
    controller/plan_source. Diagnostic geometry never steers.

    In this mode the steering-offset correction is taken from the stamped
    OffsetEstimateMsg and applied only while it is valid, fresh, from the same
    estimator generation and within the profile's plausibility bound; otherwise
    the configured fallback correction is used. The composed command is clamped
    to the vehicle profile's command limit and rate limited by elapsed time.
"""

import json
import math
import os

import numpy as np
import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32, Bool, Float64, String
from nav_msgs.msg import Odometry
from racing.health import stamp_seconds
from racing.profile import VehicleProfile, default_profile_path
from util.trajectory import Trajectory
from controller.stanley_controller import StanleyController
from buggy.msg import TrajectoryMsg, StampedFloat64Msg, PlanningResultMsg, OffsetEstimateMsg

STATUS_USABLE = (PlanningResultMsg.ELIGIBLE, PlanningResultMsg.DEGRADED)


class Controller(Node):

    def __init__(self):
        """
        Constructor for Controller class.

        Creates a ROS node with a publisher that periodically sends a message
        indicating whether the node is still alive.

        """
        super().__init__('controller')
        self.get_logger().info('INITIALIZED.')

        # Parameters
        self.declare_parameter("dist", 0.0) # Starting Distance along path
        start_dist = self.get_parameter("dist").value
        self.declare_parameter("stateTopic", "self/state")
        self.declare_parameter("steeringTopic", "input/steering")
        self.declare_parameter("rawSteeringTopic", "input/steering_raw")
        self.declare_parameter("trajectoryTopic", "self/cur_traj")
        self.declare_parameter("steerOffsetTopic", "self/steering_offset/filtered")
        self.declare_parameter("useSteerOffset", False)
        self.use_steer_offset = self.get_parameter("useSteerOffset").value
        # position standard deviation (metres) above which we refuse to start autonomous steering
        self.declare_parameter("maxInitPositionStd", 1.0)
        self.max_init_pos_std = float(self.get_parameter("maxInitPositionStd").value)
        # limit how fast the commanded steering may change (deg/s); 0 = off. Measured by elapsed time.
        self.declare_parameter("maxSteerRateDps", 0.0)
        self.max_steer_rate = float(self.get_parameter("maxSteerRateDps").value)
        self.last_cmd_deg = None
        self.last_cmd_time = None

        # ---- experimental envelope mode (D1, D9); empty topic keeps the legacy behaviour exactly
        self.declare_parameter("planningResultTopic", "")
        self.declare_parameter("maxPathAgeS", 0.5)
        # fallback: instead of snapping onto the raw reference line (a 3 m step is a 20 deg yank), follow
        # a smooth splice from where the buggy is onto the reference over this many metres, checked
        # against the profile's curvature cap; 0 = raw reference. Not validated against obstacles,
        # exactly like the raw reference it converges to.
        self.declare_parameter("fallbackSpliceLengthM", 30.0)
        self.declare_parameter("offsetEstimateTopic", "self/steering_offset/estimate")
        self.declare_parameter("offsetMaxAgeS", 0.5)
        self.declare_parameter("offsetFallbackDeg", 0.0)
        self.declare_parameter("vehicleProfile", "")
        self.declare_parameter("hardwareProfile", False)
        self.plan_topic = str(self.get_parameter("planningResultTopic").value)
        self.experimental = bool(self.plan_topic)
        self.max_path_age = float(self.get_parameter("maxPathAgeS").value)
        self.fallback_splice = float(self.get_parameter("fallbackSpliceLengthM").value)
        self.fallback_built_t = None
        self.curvature_cap = None
        self.offset_max_age = float(self.get_parameter("offsetMaxAgeS").value)
        self.offset_fallback_deg = float(self.get_parameter("offsetFallbackDeg").value)

        self.declare_parameter("traj_name", "buggycourse_safe.json")
        traj_name = self.get_parameter("traj_name").value
        self.reference_traj = Trajectory(json_filepath=os.environ["TRAJPATH"] + traj_name)
        self.cur_traj = self.reference_traj
        start_index = self.cur_traj.get_index_from_distance(start_dist)
        self.declare_parameter("useHeadingRate", True)
        self.declare_parameter("debugHeadingTopic", "debug/heading")

        self.declare_parameter("controllerName", "controller")
        self.declare_parameter("controller", "stanley")
        controller_name = self.get_parameter("controller").value
        print(controller_name.lower())
        if (controller_name.lower() == "stanley"):
            self.controller = StanleyController(start_index = start_index, namespace = self.get_namespace(),
                                                node=self, usingHeadingRateError=self.get_parameter("useHeadingRate").value,
                                                controllerName=self.get_parameter("controllerName").value) #IMPORT STANLEY
        else:
            self.get_logger().error("Invalid Controller Name: " + controller_name.lower())
            raise Exception("Invalid Controller Argument")

        # Publishers
        self.init_check_publisher = self.create_publisher(Bool,
            "debug/init_safety_check", 1
        )
        self.steer_publisher = self.create_publisher(
            StampedFloat64Msg, self.get_parameter("steeringTopic").value, 1
        )
        self.steer_raw_publisher = self.create_publisher(
            StampedFloat64Msg, self.get_parameter("rawSteeringTopic").value, 1
        )
        self.heading_publisher = self.create_publisher(
            Float32, self.get_parameter("debugHeadingTopic").value, 1
        )

        # Subscribers
        self.odom_subscriber = self.create_subscription(Odometry, self.get_parameter("stateTopic").value, self.odom_listener, 1)
        self.traj_subscriber = self.create_subscription(TrajectoryMsg, self.get_parameter("trajectoryTopic").value, self.traj_listener, 1)
        self.steer_offset_subscriber = self.create_subscription(Float64, self.get_parameter("steerOffsetTopic").value, self.offset_listener, 1)

        self.odom = None
        self.passed_init = False
        self.steer_offset : float = 0.0

        # ---- experimental mode state
        self.profile = None
        self.command_limit_deg = None
        self.max_offset_deg = None
        self.plan_msg = None
        self.plan_rx = None
        self.active_plan_id = None
        self.plan_source = "reference"
        self.plan_reason = "startup"
        self.plan_reason_logged = None
        self.offset_msg = None
        self.offset_rx = None
        self.offset_generation = None
        self.offset_in_use = False
        if self.experimental:
            self.profile = VehicleProfile.load(self.get_parameter("vehicleProfile").value or default_profile_path())
            if bool(self.get_parameter("hardwareProfile").value):
                self.profile.require_measured(["wheelbase_m", "software_steering_clip_deg"])
            self.command_limit_deg, limit_source = self.profile.command_limit_deg()
            self.max_offset_deg = self.profile.max_offset_correction_deg()
            self.curvature_cap = self.profile.planning_curvature_cap()
            self.get_logger().info(
                f"experimental envelope mode on {self.plan_topic}; command limit {self.command_limit_deg:.1f} deg "
                f"({limit_source}); offset bound {self.max_offset_deg} deg; profile {self.profile.provenance()}"
            )
            self.create_subscription(PlanningResultMsg, self.plan_topic, self.plan_listener, 1)
            self.create_subscription(OffsetEstimateMsg, self.get_parameter("offsetEstimateTopic").value,
                                     self.offset_estimate_listener, 1)
            self.plan_source_publisher = self.create_publisher(String, "controller/plan_source", 1)
            self.clamped_publisher = self.create_publisher(Bool, "controller/debug/command_clamped", 1)
            self.offset_used_publisher = self.create_publisher(Float64, "controller/debug/offset_used_deg", 1)
            self.create_timer(1.0, self.publish_plan_source)

        timer_period = 0.01  # seconds (100 Hz)
        self.timer = self.create_timer(timer_period, self.loop)

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def odom_listener(self, msg : Odometry):
        '''
        This is the subscriber that updates the buggies state for navigation
        msg, should be a CLEAN state as defined in the wiki
        '''
        self.odom = msg

    def traj_listener(self, msg):
        '''
        This is the subscriber that updates the buggies trajectory for navigation
        (legacy TrajectoryMsg path; ignored in envelope mode)
        '''
        if self.experimental:
            return
        self.cur_traj, self.controller.current_traj_index = Trajectory.unpack(msg)

    def offset_listener(self, msg):
        '''
        This is the subscriber that updates the steer offset, from offset_estimator.py
        '''
        self.steer_offset = np.deg2rad(msg.data)

    def plan_listener(self, msg: PlanningResultMsg):
        self.plan_msg = msg
        self.plan_rx = self.now_s()

    def offset_estimate_listener(self, msg: OffsetEstimateMsg):
        self.offset_msg = msg
        self.offset_rx = self.now_s()

    # ------------------------------------------------------------------ experimental helpers
    def set_plan_source(self, source, reason):
        changed = source != self.plan_source or reason != self.plan_reason
        self.plan_source, self.plan_reason = source, reason
        if changed:
            if source != "envelope" or self.plan_reason_logged != source:
                self.get_logger().info(f"trajectory source: {source} ({reason})")
            self.plan_reason_logged = source
            self.publish_plan_source()

    def publish_plan_source(self):
        if not self.experimental:
            return
        self.plan_source_publisher.publish(String(data=json.dumps({
            "source": self.plan_source, "reason": self.plan_reason,
            "plan_id": self.active_plan_id, "offset_in_use": self.offset_in_use,
        })))

    def select_trajectory(self, odom):
        """Envelope mode: use the plan only while it qualifies; otherwise the static reference."""
        now = self.now_s()
        msg = self.plan_msg
        reason = None
        if msg is None:
            reason = "no_plan_received"
        elif not msg.control_eligible or msg.status not in STATUS_USABLE:
            reason = f"plan_not_eligible:{','.join(msg.reasons)[:120]}"
        elif now > stamp_seconds(msg.valid_until):
            reason = "plan_expired"
        elif now - self.plan_rx > self.max_path_age:
            reason = "plan_too_old"
        elif len(msg.trajectory.easting) < 4:
            reason = "plan_geometry_degenerate"

        if reason is None:
            if msg.plan_id != self.active_plan_id:
                self.cur_traj, self.controller.current_traj_index = Trajectory.unpack(msg.trajectory)
                self.active_plan_id = msg.plan_id
            self.fallback_built_t = None
            self.set_plan_source("envelope", "degraded" if msg.status == PlanningResultMsg.DEGRADED else "eligible")
            return
        # fallback: (re)build the splice on entry and every 2 s so it never runs out under the buggy
        if self.fallback_built_t is None or now - self.fallback_built_t > 2.0:
            self.cur_traj = self.build_fallback(odom)
            self.controller.current_traj_index = self.cur_traj.get_closest_index_on_path(
                odom.pose.pose.position.x, odom.pose.pose.position.y)
            self.fallback_built_t = now
            self.active_plan_id = None
        self.set_plan_source("reference_fallback", reason)

    def build_fallback(self, odom):
        """A smooth return from the current offset onto the static reference, or the reference itself."""
        ref = self.reference_traj
        x, y = odom.pose.pose.position.x, odom.pose.pose.position.y
        idx0 = float(ref.get_closest_index_on_path(x, y))
        s0 = float(ref.get_distance_from_index(idx0))
        p0 = np.asarray(ref.get_position_by_index(idx0), dtype=float).reshape(-1)[:2]
        n0 = ref.get_unit_normal_by_index(np.array([idx0]))[0]
        d0 = float(np.dot(np.array([x, y]) - p0, n0))
        span = min(60.0, float(ref.distances[-1]) - s0 - 1.0)
        if self.fallback_splice <= 0.0 or abs(d0) < 0.2 or span < 15.0 or self.curvature_cap is None:
            return ref
        dists = s0 + np.linspace(0.0, span, 150)
        idx = ref.get_index_from_distance(dists)
        pts = np.asarray(ref.interpolation(idx), dtype=float)
        normals = ref.get_unit_normal_by_index(idx)
        kappa_ref = np.asarray(ref.get_curvature_by_index(idx), dtype=float).reshape(-1)
        for length in (self.fallback_splice, 1.5 * self.fallback_splice, 2.0 * self.fallback_splice):
            if length > span:
                break
            u = np.clip((dists - s0) / length, 0.0, 1.0)
            d = d0 * (1.0 - u * u * u * (u * (u * 6.0 - 15.0) + 10.0))
            d2 = np.gradient(np.gradient(d, dists), dists)
            kappa = kappa_ref / np.clip(1.0 - kappa_ref * d, 0.2, None) + d2
            if np.all(np.isfinite(kappa)) and float(np.max(np.abs(kappa))) <= self.curvature_cap:
                return Trajectory(positions=pts + d[:, None] * normals)
        return ref

    def offset_correction_rad(self):
        """Envelope mode: a validated, fresh, in-generation, plausible offset estimate or the fallback."""
        now = self.now_s()
        msg = self.offset_msg
        usable = (msg is not None and msg.valid and math.isfinite(msg.offset_rad)
                  and now - self.offset_rx <= self.offset_max_age
                  and now - stamp_seconds(msg.header.stamp) <= self.offset_max_age)
        if usable and self.max_offset_deg is not None:
            usable = abs(math.degrees(msg.offset_rad)) <= self.max_offset_deg
        if usable and self.offset_generation is not None and msg.generation != self.offset_generation:
            # a reset happened: the new generation must prove itself again before it is used
            usable = msg.generation_age_s >= 1.0
        if usable:
            self.offset_generation = msg.generation
        if usable != self.offset_in_use:
            self.get_logger().info("steering offset correction " + ("in use" if usable else "dropped to fallback"))
            self.offset_in_use = usable
        value = msg.offset_rad if usable else math.radians(self.offset_fallback_deg)
        self.offset_used_publisher.publish(Float64(data=float(math.degrees(value))))
        return float(value)

    def init_check(self):
        """
        Checks if it's safe to switch the buggy into autonomous driving mode.
        Specifically, it checks:
            if we can recieve odometry messages from the buggy
            if the covariance is acceptable (less than 1 meter)
            if the buggy thinks it is facing in the correct direction wrt the local trajectory (not 180 degrees flipped)

        Returns:
           A boolean describing the status of the buggy (safe for auton or unsafe for auton)
        """
        odom = self.odom

        if odom is None:
            self.get_logger().warn("WARNING: no available position estimate")
            return False

        # covariance[0] and [7] are the x and y VARIANCES (m^2); the combined position std is
        # sqrt(var_x + var_y). The previous check squared the variances, which passed a 0.99 m^2
        # variance and failed anything a healthy non-RTK GQ7 reports.
        pos_std = np.sqrt(max(odom.pose.covariance[0], 0.0) + max(odom.pose.covariance[7], 0.0))
        if not np.isfinite(pos_std) or pos_std > self.max_init_pos_std:
            self.get_logger().warn(
                f"checking position estimate certainty | position std {pos_std:.2f} m > {self.max_init_pos_std:.2f} m"
            )
            return False

        current_heading = odom.pose.pose.orientation.z % (2 * np.pi)
        closest_heading = (self.cur_traj.get_heading_by_index(self.cur_traj.get_closest_index_on_path(odom.pose.pose.position.x, odom.pose.pose.position.y))) % (2 * np.pi)

        self.get_logger().info("current heading: " + str(np.rad2deg(current_heading)))
        msg = Float32()
        msg.data = np.rad2deg(current_heading)
        self.heading_publisher.publish(msg)

        # https://math.stackexchange.com/questions/1649841/signed-angle-difference-without-conditions
        delta = (current_heading - closest_heading + 3 * np.pi) % (2 * np.pi) - np.pi

        if abs(delta) >= np.pi/2:
            self.get_logger().error("WARNING: INCORRECT HEADING! restart stack. Current heading [-180, 180]: " + str(np.rad2deg(current_heading)))
            return False

        return True

    def loop(self):
        if not self.passed_init:
            self.passed_init = self.init_check()
            msg = Bool()
            msg.data = self.passed_init
            self.init_check_publisher.publish(msg)
            if self.passed_init:
                self.get_logger().info("Passed Initialization Check")
            else:
                return

        odom = self.odom
        self.heading_publisher.publish(Float32(data=np.rad2deg(odom.pose.pose.orientation.z)))

        if self.experimental:
            self.select_trajectory(odom)

        steering_angle = self.controller.compute_control(odom, self.cur_traj)

        steering_angle_raw_deg = np.rad2deg(steering_angle)
        self.steer_raw_publisher.publish(StampedFloat64Msg(header=odom.header, data=float(steering_angle_raw_deg.item())))

        if self.experimental:
            if self.use_steer_offset:
                steering_angle -= self.offset_correction_rad()
        elif self.use_steer_offset:
            steering_angle -= self.steer_offset

        steering_angle_deg = float(np.asarray(np.rad2deg(steering_angle)).reshape(-1)[0])
        if self.experimental and self.command_limit_deg is not None:
            clamped = abs(steering_angle_deg) > self.command_limit_deg
            steering_angle_deg = float(np.clip(steering_angle_deg, -self.command_limit_deg, self.command_limit_deg))
            self.clamped_publisher.publish(Bool(data=bool(clamped)))
        now = self.now_s()
        if self.max_steer_rate > 0.0 and self.last_cmd_deg is not None and self.last_cmd_time is not None:
            max_step = self.max_steer_rate * max(now - self.last_cmd_time, 0.0)
            steering_angle_deg = float(np.clip(steering_angle_deg, self.last_cmd_deg - max_step, self.last_cmd_deg + max_step))
        self.last_cmd_deg = steering_angle_deg
        self.last_cmd_time = now
        self.steer_publisher.publish(StampedFloat64Msg(header=odom.header, data=steering_angle_deg))


def main(args=None):
    rclpy.init(args=args)

    controller = Controller()

    rclpy.spin(controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
