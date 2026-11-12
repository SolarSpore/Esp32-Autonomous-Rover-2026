#!/usr/bin/env python3
"""
rover_odometry.py

Wheel odometry from the two REAR wheel encoders of the mecanum rover.
Publishes odom (nav_msgs/Odometry) and the transform odom -> base_link.

Input: the rover's encoder line, read from rover/telemetry_raw:
  "Enc: <rear left>,<rear right>,<rover millis>"
  Counts are signed and cumulative. The encoders have one channel each, so the
  rover gives each count the direction its wheel was last DRIVEN.

What two rear wheels can and cannot tell:
  rear left  = vx + vy - k*wz        rear right = vx - vy + k*wz
  with k = (wheelbase + track width) / 2. So the sum of the two gives the
  forward speed vx exactly. The difference gives only (vy - k*wz): turning and
  sideways strafing look the same to these two wheels. This node splits it by
  looking at /cmd_vel: while a sideways command (linear.y) is fresh, the
  difference is treated as strafing, otherwise as turning. The front wheels
  would be needed to tell them apart for certain, or a gyro.
  Wheel slip is not seen. Expect drift, mostly in the turning angle.

Settings (config/odometry.yaml). The node does nothing until the first three
are set:
  wheel_radius_m       half the wheel diameter
  ticks_per_wheel_rev  count the encoder gives for one full turn of a wheel:
                       lift the rover, turn a rear wheel by hand exactly one
                       turn, and read the change in the Enc number
  wheelbase_m          distance between the front and rear axles
  track_width_m        distance between the left and right wheels
Checking them on the floor: drive 1 m and compare odom x; the distance scales
with wheel_radius_m, so multiply it by (real distance / reported distance).
Turn on the spot and compare the angle; the turning scales with
1 / (wheelbase_m + track_width_m). If a wheel counts down while driving
forward, flip its sign; if a left turn reports a negative angle, flip yaw_sign.
"""

import math

import rclpy
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String
from tf2_ros import TransformBroadcaster


def parse_encoder_line(line):
    """'Enc: 120,-118,45210' -> (120, -118, 45210). None if not an Enc line."""
    for part in line.split('|'):
        if ':' not in part:
            continue
        key, value = part.split(':', 1)
        if key.strip() != 'Enc':
            continue
        pieces = value.strip().split(',')
        if len(pieces) != 3:
            return None
        try:
            return int(pieces[0]), int(pieces[1]), int(pieces[2])
        except ValueError:
            return None
    return None


def rear_wheel_motion(v_rl, v_rr, k, strafe_commanded, yaw_sign, strafe_sign):
    """Rear wheel speeds (m/s) -> (vx, vy, wz). See the module notes."""
    vx = (v_rl + v_rr) / 2.0
    diff = (v_rl - v_rr) / 2.0          # = vy - k * wz
    if strafe_commanded:
        return vx, strafe_sign * diff, 0.0
    wz = yaw_sign * (-diff / k) if k > 0.0 else 0.0
    return vx, 0.0, wz


def integrate(x, y, yaw, vx, vy, wz, dt):
    """Moves the pose by one step, using the middle heading of the step."""
    yaw_mid = yaw + 0.5 * wz * dt
    x += (vx * math.cos(yaw_mid) - vy * math.sin(yaw_mid)) * dt
    y += (vx * math.sin(yaw_mid) + vy * math.cos(yaw_mid)) * dt
    return x, y, yaw + wz * dt


class RoverOdometry(Node):

    def __init__(self):
        super().__init__('rover_odometry')

        # ---- Parameters -----------------------------------------------
        self.declare_parameter('wheel_radius_m', 0.0)         # REQUIRED
        self.declare_parameter('ticks_per_wheel_rev', 0.0)    # REQUIRED
        self.declare_parameter('wheelbase_m', 0.0)            # REQUIRED
        self.declare_parameter('track_width_m', 0.30)
        self.declare_parameter('rear_left_sign', 1.0)
        self.declare_parameter('rear_right_sign', 1.0)
        self.declare_parameter('yaw_sign', 1.0)
        self.declare_parameter('strafe_sign', 1.0)
        self.declare_parameter('strafe_threshold', 0.05)      # m/s on cmd_vel linear.y
        self.declare_parameter('cmd_timeout_s', 0.5)
        self.declare_parameter('min_dt_s', 0.005)
        self.declare_parameter('max_dt_s', 1.0)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_tf', True)

        g = lambda n: self.get_parameter(n).value
        self.wheel_radius = float(g('wheel_radius_m'))
        self.ticks_per_rev = float(g('ticks_per_wheel_rev'))
        self.wheelbase = float(g('wheelbase_m'))
        self.track = float(g('track_width_m'))
        self.sign_rl = float(g('rear_left_sign'))
        self.sign_rr = float(g('rear_right_sign'))
        self.yaw_sign = float(g('yaw_sign'))
        self.strafe_sign = float(g('strafe_sign'))
        self.strafe_threshold = float(g('strafe_threshold'))
        self.cmd_timeout = float(g('cmd_timeout_s'))
        self.min_dt = float(g('min_dt_s'))
        self.max_dt = float(g('max_dt_s'))
        self.odom_frame = str(g('odom_frame'))
        self.base_frame = str(g('base_frame'))
        self.publish_tf = bool(g('publish_tf'))

        self.configured = (self.wheel_radius > 0.0 and self.ticks_per_rev > 0.0
                           and self.wheelbase > 0.0 and self.track > 0.0)
        self.metres_per_tick = (
            2.0 * math.pi * self.wheel_radius / self.ticks_per_rev
            if self.configured else 0.0)
        self.k = (self.wheelbase + self.track) / 2.0

        # ---- State ------------------------------------------------------
        self.last_counts = None        # (rear left, rear right)
        self.last_time_ms = None
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.cmd = Twist()
        self.cmd_time = None

        # ---- ROS interfaces ---------------------------------------------
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.speeds_pub = self.create_publisher(
            Float32MultiArray, 'odometry/rear_wheel_speeds', 10)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None
        self.create_subscription(String, 'rover/telemetry_raw', self.telemetry_callback, 50)
        self.create_subscription(Twist, 'cmd_vel', self.cmd_callback, 10)

        if self.configured:
            self.get_logger().info(
                f'rover_odometry started: {self.metres_per_tick * 1000:.3f} mm per '
                f'count, k = {self.k:.3f} m')
        else:
            self.get_logger().error(
                'rover_odometry is not set up: wheel_radius_m, ticks_per_wheel_rev '
                'and wheelbase_m must be above zero (config/odometry.yaml). '
                'Nothing is published until they are set.')

    def cmd_callback(self, msg: Twist):
        self.cmd = msg
        self.cmd_time = self.get_clock().now()

    def strafe_commanded(self):
        if self.cmd_time is None:
            return False
        age = (self.get_clock().now() - self.cmd_time).nanoseconds / 1e9
        return age <= self.cmd_timeout and abs(self.cmd.linear.y) > self.strafe_threshold

    def telemetry_callback(self, msg: String):
        if not self.configured:
            return
        parsed = parse_encoder_line(msg.data)
        if parsed is None:
            return
        self.handle_encoders(*parsed)

    def handle_encoders(self, rear_left: int, rear_right: int, time_ms: int):
        counts = (rear_left, rear_right)

        if self.last_counts is None or time_ms <= self.last_time_ms:
            # First reading, or the rover rebooted (its clock restarted and
            # the counts went back to zero): start again from here.
            self.last_counts, self.last_time_ms = counts, time_ms
            return

        dt = (time_ms - self.last_time_ms) / 1000.0
        if dt < self.min_dt:
            return
        previous = self.last_counts
        self.last_counts, self.last_time_ms = counts, time_ms
        if dt > self.max_dt:
            return   # a long gap (Wi-Fi dropped): do not guess the motion

        v_rl = self.sign_rl * (rear_left - previous[0]) * self.metres_per_tick / dt
        v_rr = self.sign_rr * (rear_right - previous[1]) * self.metres_per_tick / dt

        vx, vy, wz = rear_wheel_motion(
            v_rl, v_rr, self.k, self.strafe_commanded(),
            self.yaw_sign, self.strafe_sign)
        self.x, self.y, self.yaw = integrate(
            self.x, self.y, self.yaw, vx, vy, wz, dt)

        self.publish(vx, vy, wz, v_rl, v_rr)

    def publish(self, vx, vy, wz, v_rl, v_rr):
        stamp = self.get_clock().now().to_msg()
        qz = math.sin(self.yaw / 2.0)
        qw = math.cos(self.yaw / 2.0)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = wz
        # Rough confidence: position is better than heading (see the notes).
        pose_cov = [0.0] * 36
        pose_cov[0] = pose_cov[7] = 0.05
        pose_cov[35] = 0.2
        twist_cov = [0.0] * 36
        twist_cov[0] = twist_cov[7] = 0.02
        twist_cov[35] = 0.1
        odom.pose.covariance = pose_cov
        odom.twist.covariance = twist_cov
        self.odom_pub.publish(odom)

        if self.tf_broadcaster is not None:
            transform = TransformStamped()
            transform.header.stamp = stamp
            transform.header.frame_id = self.odom_frame
            transform.child_frame_id = self.base_frame
            transform.transform.translation.x = self.x
            transform.transform.translation.y = self.y
            transform.transform.rotation.z = qz
            transform.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(transform)

        self.speeds_pub.publish(Float32MultiArray(data=[float(v_rl), float(v_rr)]))


def main(args=None):
    rclpy.init(args=args)
    node = RoverOdometry()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
