#!/usr/bin/env python3
"""
rover_peripherals_bridge.py

Everything on the ESP32 that is not driving: head swivel, buzzer and sensor
telemetry (ultrasonic today; LEDs and other sensors can be added here later).
Never touches /cmd_vel and never sends to the drive port (4210), so it cannot
affect driving.

Rover-bound (UDP port 4212, this node is the ONLY sender):
  head/angle   (std_msgs/Float32, degrees)  -> "HEAD,<angle>" at head_send_hz
  buzzer/beep  (std_msgs/Empty)             -> one "BUZZ,<hz>,<ms>"

  The firmware releases the servo (goes limp) if no HEAD packet arrives for
  500 ms, so HEAD is streamed only while head/angle is arriving. If the
  publisher stops, the head goes limp on its own. On shutdown this node also
  sends "RELEASE".

Rover telemetry (UDP port 4211, broadcast by the rover, this node listens):
  "Dist:   32.5 cm | Line: 01000 | IR: 00"
  ultrasonic/range     (sensor_msgs/Range, meters; inf = no echo)
  rover/telemetry_raw  (std_msgs/String, the full line: includes line + IR)

  The line is parsed by label ("Name: value" separated by "|"), so a new
  sensor can be added to the rover's line without breaking the others.
"""

import socket

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import Empty, Float32, String


def parse_telemetry(line):
    """'Dist: 32.5 cm | Line: 01000' -> {'Dist': '32.5 cm', 'Line': '01000'}"""
    fields = {}
    for part in line.split('|'):
        if ':' in part:
            key, value = part.split(':', 1)
            fields[key.strip()] = value.strip()
    return fields


def parse_distance_cm(text):
    """'32.5 cm' -> 32.5. Returns None if it is not a number."""
    try:
        return float(text.split()[0])
    except (ValueError, IndexError):
        return None


class RoverPeripheralsBridge(Node):

    def __init__(self):
        super().__init__('rover_peripherals_bridge')

        # ---- Parameters -----------------------------------------------
        self.declare_parameter('robot_ip', '192.168.1.26')
        self.declare_parameter('aux_port', 4212)
        self.declare_parameter('telemetry_port', 4211)
        self.declare_parameter('head_send_hz', 20.0)
        self.declare_parameter('head_cmd_timeout', 0.3)   # seconds
        self.declare_parameter('beep_hz', 1000)
        self.declare_parameter('beep_ms', 200)
        self.declare_parameter('telemetry_poll_hz', 50.0)
        self.declare_parameter('range_min_m', 0.02)
        self.declare_parameter('range_max_m', 2.0)
        self.declare_parameter('range_fov_rad', 0.26)
        self.declare_parameter('range_frame_id', 'ultrasonic_link')

        g = lambda n: self.get_parameter(n).value
        self.robot_ip = str(g('robot_ip'))
        self.aux_port = int(g('aux_port'))
        self.telemetry_port = int(g('telemetry_port'))
        self.head_send_hz = float(g('head_send_hz'))
        self.head_cmd_timeout = float(g('head_cmd_timeout'))
        self.beep_hz = int(g('beep_hz'))
        self.beep_ms = int(g('beep_ms'))
        self.telemetry_poll_hz = float(g('telemetry_poll_hz'))
        self.range_min_m = float(g('range_min_m'))
        self.range_max_m = float(g('range_max_m'))
        self.range_fov_rad = float(g('range_fov_rad'))
        self.range_frame_id = str(g('range_frame_id'))

        # ---- Sockets ----------------------------------------------------
        self.tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.rx = None
        try:
            rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            rx.bind(('', self.telemetry_port))
            rx.setblocking(False)
            self.rx = rx
        except (OSError, OverflowError) as exc:
            # Head and buzzer still work without telemetry.
            self.get_logger().error(
                f'Cannot listen for telemetry on UDP {self.telemetry_port}: '
                f'{exc}. Telemetry disabled.'
            )

        # ---- State ------------------------------------------------------
        self.head_angle = None
        self.head_time = None

        # ---- ROS interfaces ---------------------------------------------
        self.create_subscription(Float32, 'head/angle', self.head_callback, 10)
        self.create_subscription(Empty, 'buzzer/beep', self.beep_callback, 10)
        self.range_pub = self.create_publisher(Range, 'ultrasonic/range', 10)
        self.raw_pub = self.create_publisher(String, 'rover/telemetry_raw', 10)

        if self.head_send_hz > 0.0:
            self.create_timer(1.0 / self.head_send_hz, self.head_timer)
        if self.rx is not None and self.telemetry_poll_hz > 0.0:
            self.create_timer(1.0 / self.telemetry_poll_hz, self.telemetry_timer)

        self.get_logger().info(
            f'rover_peripherals_bridge started -> {self.robot_ip}:{self.aux_port} '
            f'(head, buzzer); telemetry on UDP {self.telemetry_port}'
        )

    # ---- Sending ---------------------------------------------------------
    def send(self, text: str):
        try:
            self.tx.sendto(text.encode('utf-8'), (self.robot_ip, self.aux_port))
        except OSError as exc:
            self.get_logger().warn(f'UDP send failed: {exc}',
                                   throttle_duration_sec=5.0)

    def head_callback(self, msg: Float32):
        self.head_angle = float(msg.data)
        self.head_time = self.get_clock().now()

    def head_timer(self):
        if self.head_angle is None or self.head_time is None:
            return
        age = (self.get_clock().now() - self.head_time).nanoseconds / 1e9
        if age > self.head_cmd_timeout:
            return   # stop sending: the firmware releases the servo itself
        angle = int(round(max(0.0, min(180.0, self.head_angle))))
        self.send(f'HEAD,{angle}')

    def beep_callback(self, _msg: Empty):
        self.send(f'BUZZ,{self.beep_hz},{self.beep_ms}')

    # ---- Telemetry ---------------------------------------------------------
    def telemetry_timer(self):
        for _ in range(20):   # drain what is waiting, but never spin forever
            try:
                data, _addr = self.rx.recvfrom(256)
            except BlockingIOError:
                return
            except OSError as exc:
                self.get_logger().warn(f'Telemetry receive failed: {exc}',
                                       throttle_duration_sec=5.0)
                return
            self.handle_telemetry(data.decode('utf-8', errors='replace').strip())

    def handle_telemetry(self, line: str):
        if not line:
            return
        self.raw_pub.publish(String(data=line))

        fields = parse_telemetry(line)
        if not fields:
            self.get_logger().warn(f'Unrecognized telemetry: {line!r}',
                                   throttle_duration_sec=10.0)
            return

        if 'Dist' in fields:
            dist_cm = parse_distance_cm(fields['Dist'])
            if dist_cm is None:
                self.get_logger().warn(f'Bad Dist value: {fields["Dist"]!r}',
                                       throttle_duration_sec=10.0)
            else:
                self.publish_range(dist_cm)

    def publish_range(self, dist_cm: float):
        msg = Range()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.range_frame_id
        msg.radiation_type = Range.ULTRASOUND
        msg.field_of_view = self.range_fov_rad
        msg.min_range = self.range_min_m
        msg.max_range = self.range_max_m
        # The firmware reports -1.0 when no echo came back (nothing in range).
        msg.range = float('inf') if dist_cm < 0.0 else dist_cm / 100.0
        self.range_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RoverPeripheralsBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.tx.sendto(b'RELEASE', (node.robot_ip, node.aux_port))
        except OSError:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
