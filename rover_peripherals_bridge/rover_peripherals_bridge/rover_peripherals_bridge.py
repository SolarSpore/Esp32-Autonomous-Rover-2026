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
  line_sensor/raw      (std_msgs/UInt8MultiArray, the Line bits as reported: 1 =
                       the sensor sees a reflective surface, left to right)
  line_sensor/detected (std_msgs/Bool, true while any sensor sees the line)
  line_sensor/position (std_msgs/Float32, -1.0 far left to +1.0 far right,
                       only while the line is detected)
  ir_sensors/raw       (std_msgs/UInt8MultiArray, the IR bits as reported)

  The line is parsed by label ("Name: value" separated by "|"), so a new
  sensor can be added to the rover's line without breaking the others.
"""

import socket

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import Bool, Empty, Float32, String, UInt8MultiArray


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


def parse_bits(text):
    """'01000' -> [0, 1, 0, 0, 0]. Returns None if it is not only 0s and 1s."""
    text = text.strip()
    if not text or any(c not in '01' for c in text):
        return None
    return [int(c) for c in text]


def line_position(bits, dark_line, weights):
    """Where the line is under a row of sensors. The firmware reports 1 for a
    sensor that sees a reflective surface, so a dark line on a light floor is
    the sensors that read 0 (dark_line=True). Returns (detected, position):
    position runs from -1.0 (far left) to +1.0 (far right), the average of the
    weights of the sensors that see the line, and is 0.0 when none do."""
    on = [(bit == 0) if dark_line else (bit == 1) for bit in bits]
    count = sum(1 for seen in on if seen)
    if count == 0:
        return False, 0.0
    usable = weights[:len(bits)]
    total = sum(w for w, seen in zip(usable, on) if seen)
    scale = max((abs(w) for w in usable), default=1.0) or 1.0
    return True, total / count / scale


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
        # 'dark' = a dark line on a light floor (the sensors that read 0 see
        # the line); 'light' = a light line on a dark floor.
        self.declare_parameter('line_color', 'dark')
        self.declare_parameter('line_weights', [-2.0, -1.0, 0.0, 1.0, 2.0])

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
        self.line_dark = str(g('line_color')).strip().lower() != 'light'
        self.line_weights = [float(w) for w in g('line_weights')]

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
        self.line_raw_pub = self.create_publisher(UInt8MultiArray, 'line_sensor/raw', 10)
        self.line_detected_pub = self.create_publisher(Bool, 'line_sensor/detected', 10)
        self.line_position_pub = self.create_publisher(Float32, 'line_sensor/position', 10)
        self.ir_raw_pub = self.create_publisher(UInt8MultiArray, 'ir_sensors/raw', 10)

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

        if 'Line' in fields:
            self.publish_line(fields['Line'])
        if 'IR' in fields:
            self.publish_ir(fields['IR'])

    def publish_line(self, text: str):
        bits = parse_bits(text)
        if bits is None:
            self.get_logger().warn(f'Bad Line value: {text!r}',
                                   throttle_duration_sec=10.0)
            return
        self.line_raw_pub.publish(UInt8MultiArray(data=bits))
        detected, position = line_position(bits, self.line_dark, self.line_weights)
        self.line_detected_pub.publish(Bool(data=detected))
        if detected:
            self.line_position_pub.publish(Float32(data=position))

    def publish_ir(self, text: str):
        bits = parse_bits(text)
        if bits is None:
            self.get_logger().warn(f'Bad IR value: {text!r}',
                                   throttle_duration_sec=10.0)
            return
        self.ir_raw_pub.publish(UInt8MultiArray(data=bits))

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
