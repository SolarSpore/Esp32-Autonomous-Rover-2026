#!/usr/bin/env python3
"""
rover_motor_bridge.py

Subscribes to geometry_msgs/Twist on 'cmd_vel' and converts it into the
ESP32 rover's UDP motor protocol: "FL,FR,RL,RR" with values in -255..255.

Kinematics: mecanum (X-pattern rollers), which also covers plain
differential drive as the special case linear.y == 0.

  fl = linear.x - linear.y - angular.z * wheel_separation / 2
  fr = linear.x + linear.y + angular.z * wheel_separation / 2
  rl = linear.x + linear.y - angular.z * wheel_separation / 2
  rr = linear.x - linear.y + angular.z * wheel_separation / 2

When linear.y is 0 this reduces to fl == rl and fr == rr, i.e. the same
left/right split as plain differential drive. linear.y != 0 (published by
rover_control_node's strafe mode) drives all four wheels independently so
the rover can move sideways.

If strafing comes out backwards, or the diagonal pairs fight each other
instead of driving together, flip 'strafe_sign' (a parameter) rather than
re-deriving the formula.
"""

import socket

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class RoverMotorBridge(Node):

    def __init__(self):
        super().__init__('rover_motor_bridge')

        # ---- Parameters ---------------------------------------------------
        self.declare_parameter('robot_ip', '192.168.1.26')
        self.declare_parameter('robot_port', 4210)
        self.declare_parameter('wheel_separation', 0.30)   # meters, track width
        self.declare_parameter('max_linear_speed', 0.5)    # m/s that maps to PWM 255
        self.declare_parameter('send_rate_hz', 20.0)       # must beat the ESP32's
                                                            # 500ms failsafe timeout
        self.declare_parameter('cmd_vel_timeout', 0.5)     # seconds
        self.declare_parameter('min_pwm', 0)               # lowest PWM that reliably spins the motors
        self.declare_parameter('strafe_sign', 1.0)         # flip if strafe direction is backwards

        self.robot_ip = self.get_parameter('robot_ip').value
        self.robot_port = int(self.get_parameter('robot_port').value)
        self.wheel_separation = float(self.get_parameter('wheel_separation').value)
        self.max_linear_speed = float(self.get_parameter('max_linear_speed').value)
        self.send_rate_hz = float(self.get_parameter('send_rate_hz').value)
        self.cmd_vel_timeout = float(self.get_parameter('cmd_vel_timeout').value)
        self.min_pwm = int(self.get_parameter('min_pwm').value)
        self.strafe_sign = float(self.get_parameter('strafe_sign').value)

        # ---- UDP socket -----------------------------------------------
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # ---- State ----------------------------------------------------
        self.last_twist = Twist()
        self.last_cmd_time = self.get_clock().now()

        # ---- ROS interfaces ---------------------------------------------
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10,
        )

        timer_period = 1.0 / self.send_rate_hz
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info(
            f'rover_motor_bridge started -> {self.robot_ip}:{self.robot_port} '
            f'@ {self.send_rate_hz} Hz (mecanum, strafe_sign={self.strafe_sign})'
        )

    def cmd_vel_callback(self, msg: Twist):
        self.last_twist = msg
        self.last_cmd_time = self.get_clock().now()

    def timer_callback(self):
        now = self.get_clock().now()
        age = (now - self.last_cmd_time).nanoseconds / 1e9

        if age > self.cmd_vel_timeout:
            fl = fr = rl = rr = 0
        else:
            fl, fr, rl, rr = self.twist_to_wheel_pwm(self.last_twist)

        message = f'{fl},{fr},{rl},{rr}'
        try:
            self.sock.sendto(message.encode('utf-8'), (self.robot_ip, self.robot_port))
        except OSError as exc:
            self.get_logger().warn(f'UDP send failed: {exc}')

    def twist_to_wheel_pwm(self, twist: Twist):
        linear_x = twist.linear.x
        linear_y = twist.linear.y * self.strafe_sign
        angular = twist.angular.z
        half_sep = self.wheel_separation / 2.0

        fl_mps = linear_x - linear_y - angular * half_sep
        fr_mps = linear_x + linear_y + angular * half_sep
        rl_mps = linear_x + linear_y - angular * half_sep
        rr_mps = linear_x - linear_y + angular * half_sep

        scale = 255.0 / self.max_linear_speed if self.max_linear_speed > 0 else 0.0

        fl_pwm = self.apply_min_pwm(int(max(-255, min(255, fl_mps * scale))))
        fr_pwm = self.apply_min_pwm(int(max(-255, min(255, fr_mps * scale))))
        rl_pwm = self.apply_min_pwm(int(max(-255, min(255, rl_mps * scale))))
        rr_pwm = self.apply_min_pwm(int(max(-255, min(255, rr_mps * scale))))

        return fl_pwm, fr_pwm, rl_pwm, rr_pwm

    def apply_min_pwm(self, pwm):
        if self.min_pwm <= 0:
            return pwm
        if abs(pwm) < 3:
            return 0
        mag = int(self.min_pwm + abs(pwm) * (255 - self.min_pwm) / 255.0)
        return mag if pwm > 0 else -mag


def main(args=None):
    rclpy.init(args=args)
    node = RoverMotorBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.sock.sendto(b'0,0,0,0', (node.robot_ip, node.robot_port))
        except OSError:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
