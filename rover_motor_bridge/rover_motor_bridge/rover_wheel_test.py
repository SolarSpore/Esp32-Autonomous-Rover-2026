#!/usr/bin/env python3
"""
rover_wheel_test.py

The simplest possible bridge: listens for a 4-element integer array on the
'wheel_cmd' topic and forwards it directly to the ESP32 over UDP as
"FL,FR,RL,RR". No kinematics, no math -- what you publish is exactly what
each wheel does. Useful for testing individual wheels and understanding
the raw UDP protocol before the differential-drive node adds a layer of
abstraction on top.

Message type: std_msgs/Int32MultiArray
  data = [FL, FR, RL, RR], each in the range -255..255

Publish from Foxglove's "Publish" panel, e.g.:
  { "data": [150, 0, 0, 0] }       -> FL forward, everything else stopped
  { "data": [150, 150, 150, 150] } -> all wheels forward
  { "data": [0, 0, 0, 0] }         -> stop
"""

import socket

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray


class RoverWheelTest(Node):

    def __init__(self):
        super().__init__('rover_wheel_test')

        # ---- Parameters ---------------------------------------------------
        self.declare_parameter('robot_ip', '192.168.1.26')
        self.declare_parameter('robot_port', 4210)
        self.declare_parameter('cmd_timeout', 0.5)  # seconds before auto-stop

        self.robot_ip = self.get_parameter('robot_ip').value
        self.robot_port = int(self.get_parameter('robot_port').value)
        self.cmd_timeout = float(self.get_parameter('cmd_timeout').value)

        # ---- UDP socket -----------------------------------------------
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # ---- State ----------------------------------------------------
        self.last_wheels = [0, 0, 0, 0]
        self.last_cmd_time = self.get_clock().now()

        # ---- ROS interfaces ---------------------------------------------
        self.subscription = self.create_subscription(
            Int32MultiArray,
            'wheel_cmd',
            self.wheel_cmd_callback,
            10,
        )

        # Re-send at 10Hz so the ESP32's 500ms failsafe never trips on a
        # command that's still supposed to be active.
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.get_logger().info(
            f'rover_wheel_test started -> {self.robot_ip}:{self.robot_port}'
        )

    def wheel_cmd_callback(self, msg: Int32MultiArray):
        if len(msg.data) != 4:
            self.get_logger().warn(
                f'Expected 4 values [FL,FR,RL,RR], got {len(msg.data)}. Ignoring.'
            )
            return

        self.last_wheels = [max(-255, min(255, int(v))) for v in msg.data]
        self.last_cmd_time = self.get_clock().now()

    def timer_callback(self):
        now = self.get_clock().now()
        age = (now - self.last_cmd_time).nanoseconds / 1e9

        if age > self.cmd_timeout:
            wheels = [0, 0, 0, 0]
        else:
            wheels = self.last_wheels

        message = ','.join(str(v) for v in wheels)
        try:
            self.sock.sendto(message.encode('utf-8'), (self.robot_ip, self.robot_port))
        except OSError as exc:
            self.get_logger().warn(f'UDP send failed: {exc}')


def main(args=None):
    rclpy.init(args=args)
    node = RoverWheelTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Send one final stop so the rover doesn't keep driving if the
        # node is killed uncleanly.
        try:
            node.sock.sendto(b'0,0,0,0', (node.robot_ip, node.robot_port))
        except OSError:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
