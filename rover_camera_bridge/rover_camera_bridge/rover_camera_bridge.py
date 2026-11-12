#!/usr/bin/env python3
"""
rover_camera_bridge.py

Bridge for the camera board (Seeed XIAO ESP32-S3 Sense, the rover's second
ESP32): video, microphone, LED strip and sensors. Never touches /cmd_vel or any
rover port (4210 to 4212), so the drive, head, buzzer and ultrasonic keep
working if the camera board is off or restarting.

Video (HTTP, camera board port 81, path /stream, MJPEG):
  -> camera/image/compressed (sensor_msgs/CompressedImage, format jpeg)
  Each JPEG frame is passed on as it is, without decoding. The board serves one
  stream viewer at a time: close any other viewer (a browser tab) while this
  node runs. The video is read in its own thread, so a slow stream never delays
  the other topics.

Telemetry (UDP port 4213, broadcast by the board, this node listens), one line:
  "IR: 10 | CAM: 1 | MIC: 0 | LVL: 0.000 | LED: solid ff0000 50"
  camera/ir          (std_msgs/UInt8MultiArray, one 0 or 1 per IR sensor; only
                      published once the sensors are fitted)
  camera/enabled     (std_msgs/Bool, latched: the camera state the board reported)
  microphone/level   (std_msgs/Float32, 0.0 to 1.0, about 5 times a second)
  microphone/mode    (std_msgs/String, latched: off, level or stream)
  leds/state         (std_msgs/String, latched: "solid ff0000 50" = mode, RRGGBB
                      colour and brightness percent)
  camera/ir_raw      (std_msgs/String, the whole line)

Camera on/off (the chip, Wi-Fi, OTA, LEDs and the microphone stay on; only the
camera sensor is switched):
  Service camera/enable (std_srvs/SetBool): true = on, false = off. The node
  sends "CAM,ON" or "CAM,OFF" (UDP port 4214) and repeats it until the board's
  telemetry confirms. The last request wins. D-pad up held for toggle_hold_s
  seconds on the gamepad (/joy) toggles it. A shorter hold of D-pad up (at least
  light_toggle_min_s, but released before toggle_hold_s) toggles the LEDs instead.

Microphone (off until asked; the board streams audio to this computer only
while it is in stream mode):
  Topic microphone/set_mode (std_msgs/String): off, level or stream.
  Service microphone/enable (std_srvs/SetBool): true = stream, false = off.
  D-pad down held for mic_toggle_hold_s seconds toggles off <-> mic_toggle_mode.
  microphone/audio (std_msgs/Int16MultiArray): 16 kHz mono samples, 20 ms per
  message, while the board streams (UDP port 4215).
  If the board reports the microphone on when this node starts, the node
  switches it off (mic_off_on_start). On shutdown the node sends "MIC,OFF".

LED strip (commands are sent twice, because UDP can lose a packet):
  leds/color      (std_msgs/ColorRGBA, 0.0 to 1.0)  -> all LEDs one colour
  leds/mode       (std_msgs/String: off, solid, breathe, chase, rainbow)
  leds/brightness (std_msgs/UInt8, 0 to 100)
  leds/pixel      (std_msgs/UInt8MultiArray: index, r, g, b)  -> one LED
  Lights on/off from the gamepad: hold D-pad up for 1 second or more, then let
  go before 3 seconds. On restores the last mode the strip was in (lights_on_mode
  if that is unknown); off keeps the colour. Holding for 3 seconds toggles the
  camera instead, and does not touch the lights.
"""

import array
import math
import socket
import sys
import threading
import time
import urllib.request

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, DurabilityPolicy
)
from sensor_msgs.msg import CompressedImage, Joy
from std_msgs.msg import (
    Bool, ColorRGBA, Float32, Int16MultiArray, String, UInt8, UInt8MultiArray
)
from std_srvs.srv import SetBool

JPEG_START = b'\xff\xd8'

MIC_NAMES = {0: 'off', 1: 'level', 2: 'stream'}
MIC_NUMBERS = {name: number for number, name in MIC_NAMES.items()}
MIC_COMMANDS = {0: 'MIC,OFF', 1: 'MIC,LEVEL', 2: 'MIC,ON'}
LED_MODES = ('off', 'solid', 'breathe', 'chase', 'rainbow')


def clamp(value, low, high):
    return max(low, min(high, value))


def to_byte(value):
    """0.0 to 1.0 -> 0 to 255."""
    return int(round(clamp(float(value), 0.0, 1.0) * 255))


def parse_telemetry(line):
    """'IR: 10 | CAM: 1' -> {'IR': '10', 'CAM': '1'}"""
    fields = {}
    for part in line.split('|'):
        if ':' in part:
            key, value = part.split(':', 1)
            fields[key.strip()] = value.strip()
    return fields


def parse_ir_bits(line):
    """'IR: 10' -> [1, 0]. Returns None if there is no valid IR field."""
    bits = parse_telemetry(line).get('IR')
    if not bits or any(c not in '01' for c in bits):
        return None
    return [int(c) for c in bits]


def parse_cam_state(line):
    """'CAM: 1' -> True, 'CAM: 0' -> False, anything else -> None."""
    value = parse_telemetry(line).get('CAM')
    if value == '1':
        return True
    if value == '0':
        return False
    return None


def parse_mic_mode(line):
    """'MIC: 2' -> 2 (0 off, 1 level, 2 stream). None if missing or invalid."""
    value = parse_telemetry(line).get('MIC')
    if value in ('0', '1', '2'):
        return int(value)
    return None


def parse_level(line):
    """'LVL: 0.034' -> 0.034. None if missing or not a number."""
    value = parse_telemetry(line).get('LVL')
    try:
        level = float(value)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(level):
        return None
    return clamp(level, 0.0, 1.0)


def parse_led_state(line):
    """'LED: solid ff0000 50' -> 'solid ff0000 50'. None if missing."""
    value = parse_telemetry(line).get('LED')
    return value if value else None


def parse_audio_packet(data):
    """A 4 byte little-endian sequence number, then 16 bit samples.
    Returns (sequence, sample bytes) or None."""
    if len(data) < 6:
        return None
    sequence = int.from_bytes(data[:4], 'little')
    payload = data[4:]
    if len(payload) % 2:
        payload = payload[:-1]
    return sequence, payload


def audio_gap(last, sequence):
    """How many packets went missing between two sequence numbers. A restart
    of the counter (a new stream) counts as none."""
    gap = (sequence - last - 1) & 0xFFFFFFFF
    return gap if gap < 1000 else 0


class HoldDetector:
    """update() returns True once when pressed has stayed True for hold_s."""

    def __init__(self, hold_s):
        self.hold_s = hold_s
        self.start = None
        self.fired = False

    def update(self, pressed, now):
        if not pressed:
            self.start = None
            self.fired = False
            return False
        if self.start is None:
            self.start = now
        if not self.fired and now - self.start >= self.hold_s:
            self.fired = True
            return True
        return False


def read_exactly(resp, count):
    data = bytearray()
    while len(data) < count:
        chunk = resp.read(count - len(data))
        if not chunk:
            return None
        data += chunk
    return bytes(data)


def read_frames(url, timeout, should_stop):
    """Yields one JPEG (as bytes) for every frame in the MJPEG stream."""
    with urllib.request.urlopen(url, timeout=timeout) as resp:
        while not should_stop():
            length = None
            while True:
                line = resp.readline()
                if not line:
                    return
                text = line.strip().lower()
                if text.startswith(b'content-length:'):
                    length = int(text.split(b':', 1)[1])
                elif text == b'' and length is not None:
                    break
            frame = read_exactly(resp, length)
            if frame is None:
                return
            if frame.startswith(JPEG_START):
                yield frame


class RoverCameraBridge(Node):

    def __init__(self):
        super().__init__('rover_camera_bridge')

        # ---- Parameters -----------------------------------------------
        self.declare_parameter('camera_host', '192.168.1.27')
        self.declare_parameter('stream_port', 81)
        self.declare_parameter('stream_path', '/stream')
        self.declare_parameter('telemetry_port', 4213)
        self.declare_parameter('cmd_port', 4214)
        self.declare_parameter('audio_port', 4215)
        self.declare_parameter('telemetry_poll_hz', 50.0)
        self.declare_parameter('audio_poll_hz', 100.0)
        self.declare_parameter('read_timeout', 5.0)          # seconds
        self.declare_parameter('reconnect_delay', 2.0)       # seconds
        self.declare_parameter('stats_interval', 30.0)       # seconds
        self.declare_parameter('image_frame_id', 'camera_link')
        self.declare_parameter('command_retries', 5)
        self.declare_parameter('command_retry_period', 1.0)  # seconds
        self.declare_parameter('mic_off_on_start', True)
        # D-pad on the Steam Deck's virtual Xbox pad: /joy axis 7 (up = +1).
        self.declare_parameter('toggle_axis', 7)             # camera: D-pad up
        self.declare_parameter('toggle_direction', 1)
        self.declare_parameter('toggle_threshold', 0.5)
        self.declare_parameter('toggle_hold_s', 3.0)
        self.declare_parameter('mic_toggle_axis', 7)         # microphone: D-pad down
        self.declare_parameter('mic_toggle_direction', -1)
        self.declare_parameter('mic_toggle_hold_s', 3.0)
        self.declare_parameter('mic_toggle_mode', 'level')   # level or stream
        self.declare_parameter('joy_timeout', 1.0)           # seconds
        self.declare_parameter('light_toggle_min_s', 1.0)    # D-pad up: 0 = no light toggle
        self.declare_parameter('lights_on_mode', 'solid')    # used when no earlier mode is known

        g = lambda n: self.get_parameter(n).value
        self.camera_host = str(g('camera_host'))
        self.stream_port = int(g('stream_port'))
        self.stream_path = str(g('stream_path'))
        self.telemetry_port = int(g('telemetry_port'))
        self.cmd_port = int(g('cmd_port'))
        self.audio_port = int(g('audio_port'))
        self.telemetry_poll_hz = float(g('telemetry_poll_hz'))
        self.audio_poll_hz = float(g('audio_poll_hz'))
        self.read_timeout = float(g('read_timeout'))
        self.reconnect_delay = float(g('reconnect_delay'))
        self.stats_interval = float(g('stats_interval'))
        self.image_frame_id = str(g('image_frame_id'))
        self.command_retries = int(g('command_retries'))
        self.command_retry_period = float(g('command_retry_period'))
        self.mic_off_on_start = bool(g('mic_off_on_start'))
        self.toggle_axis = int(g('toggle_axis'))
        self.toggle_direction = 1 if int(g('toggle_direction')) >= 0 else -1
        self.toggle_threshold = float(g('toggle_threshold'))
        self.toggle_hold_s = float(g('toggle_hold_s'))
        self.mic_toggle_axis = int(g('mic_toggle_axis'))
        self.mic_toggle_direction = 1 if int(g('mic_toggle_direction')) >= 0 else -1
        self.mic_toggle_hold_s = float(g('mic_toggle_hold_s'))
        mic_toggle_mode = str(g('mic_toggle_mode')).strip().lower()
        self.mic_toggle_mode = MIC_NUMBERS.get(mic_toggle_mode, 1)
        if self.mic_toggle_mode == 0:
            self.mic_toggle_mode = 1
        self.joy_timeout = float(g('joy_timeout'))
        self.light_toggle_min_s = float(g('light_toggle_min_s'))
        lights_on_mode = str(g('lights_on_mode')).strip().lower()
        self.lights_on_mode = (
            lights_on_mode if lights_on_mode in LED_MODES and lights_on_mode != 'off'
            else 'solid')
        self.stream_url = (
            f'http://{self.camera_host}:{self.stream_port}{self.stream_path}'
        )

        # ---- ROS interfaces ---------------------------------------------
        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )   # a node that starts later still gets the current state

        self.image_pub = self.create_publisher(
            CompressedImage, 'camera/image/compressed', qos_profile_sensor_data
        )
        self.ir_pub = self.create_publisher(UInt8MultiArray, 'camera/ir', 10)
        self.ir_raw_pub = self.create_publisher(String, 'camera/ir_raw', 10)
        self.enabled_pub = self.create_publisher(Bool, 'camera/enabled', latched)
        self.level_pub = self.create_publisher(Float32, 'microphone/level', 10)
        self.mic_mode_pub = self.create_publisher(String, 'microphone/mode', latched)
        self.audio_pub = self.create_publisher(
            Int16MultiArray, 'microphone/audio', qos_profile_sensor_data
        )
        self.led_state_pub = self.create_publisher(String, 'leds/state', latched)

        self.create_service(SetBool, 'camera/enable', self.enable_service)
        self.create_service(SetBool, 'microphone/enable', self.mic_enable_service)
        self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        self.create_subscription(String, 'microphone/set_mode', self.mic_mode_callback, 10)
        self.create_subscription(ColorRGBA, 'leds/color', self.led_color_callback, 10)
        self.create_subscription(String, 'leds/mode', self.led_mode_callback, 10)
        self.create_subscription(UInt8, 'leds/brightness', self.led_brightness_callback, 10)
        self.create_subscription(UInt8MultiArray, 'leds/pixel', self.led_pixel_callback, 10)

        # ---- Sockets ------------------------------------------------------
        self.tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.rx = self.open_rx(self.telemetry_port, 'telemetry')
        self.audio_rx = self.open_rx(self.audio_port, 'audio')

        # ---- Camera on/off state ---------------------------------------------
        self.camera_running = None     # what the board last reported
        self.desired = None            # a request still waiting for the board
        self.retries_left = 0
        self.cam_hold = HoldDetector(self.toggle_hold_s)

        # ---- Microphone state ------------------------------------------------
        self.mic_reported = None       # 0, 1 or 2, as the board last reported
        self.mic_desired = None
        self.mic_retries_left = 0
        self.mic_hold = HoldDetector(self.mic_toggle_hold_s)
        self.last_audio_seq = None
        self.audio_packets = 0
        self.audio_lost = 0
        self.audio_stats_time = time.monotonic()

        # ---- LED state and repeated sends --------------------------------------
        self.led_reported = None
        self.led_last_mode = None      # the last mode that was not off
        self.up_since = None           # when D-pad up was pressed
        self.up_camera_fired = False   # the camera toggle used this press
        self.pending_sends = []        # (time due, text)

        # ---- Gamepad -------------------------------------------------------------
        self.cam_pressed = False
        self.mic_pressed = False
        self.last_joy_time = None

        if self.rx is not None and self.telemetry_poll_hz > 0.0:
            self.create_timer(1.0 / self.telemetry_poll_hz, self.telemetry_timer)
        if self.audio_rx is not None and self.audio_poll_hz > 0.0:
            self.create_timer(1.0 / self.audio_poll_hz, self.audio_timer)
        self.create_timer(self.command_retry_period, self.retry_timer)
        self.create_timer(0.05, self.pending_timer)
        self.create_timer(0.1, self.toggle_timer)

        # ---- Video thread -------------------------------------------------
        self.stop_event = threading.Event()
        self.video_allowed = threading.Event()   # cleared while the camera is off
        self.video_allowed.set()
        self.frame_count = 0
        self.stats_time = self.get_clock().now()
        self.video_thread = threading.Thread(
            target=self.video_loop, name='video', daemon=True
        )
        self.video_thread.start()

        self.get_logger().info(
            f'rover_camera_bridge started: video from {self.stream_url}; '
            f'telemetry on UDP {self.telemetry_port}, audio on UDP '
            f'{self.audio_port}; commands to {self.camera_host}:{self.cmd_port}'
        )

    def open_rx(self, port, name):
        try:
            rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            rx.bind(('', port))
            rx.setblocking(False)
            return rx
        except (OSError, OverflowError) as exc:
            # Video and the other features still work without this socket.
            self.get_logger().error(
                f'Cannot listen for {name} on UDP {port}: {exc}. '
                f'{name.capitalize()} disabled.'
            )
            return None

    # ---- Sending ---------------------------------------------------------
    def send_text(self, text: str):
        try:
            self.tx.sendto(text.encode('utf-8'), (self.camera_host, self.cmd_port))
        except OSError as exc:
            self.get_logger().warn(f'Command failed: {exc}',
                                   throttle_duration_sec=5.0)

    def send_repeated(self, text: str, repeats: int = 2, gap: float = 0.1):
        """UDP can lose a packet: send state-setting commands more than once."""
        self.send_text(text)
        now = time.monotonic()
        for i in range(1, repeats):
            self.pending_sends.append((now + i * gap, text))

    def pending_timer(self):
        if not self.pending_sends:
            return
        now = time.monotonic()
        due = [item for item in self.pending_sends if item[0] <= now]
        self.pending_sends = [item for item in self.pending_sends if item[0] > now]
        for _when, text in due:
            self.send_text(text)

    # ---- Video -----------------------------------------------------------
    def video_should_stop(self):
        return self.stop_event.is_set() or not self.video_allowed.is_set()

    def video_loop(self):
        while not self.stop_event.is_set():
            if not self.video_allowed.wait(timeout=0.5):
                continue   # the camera is off: nothing to read
            try:
                self.get_logger().info(
                    f'Connecting to {self.stream_url}',
                    throttle_duration_sec=30.0,
                )
                first_frame = True
                for frame in read_frames(
                        self.stream_url, self.read_timeout,
                        self.video_should_stop):
                    if first_frame:
                        self.get_logger().info('Camera stream connected.')
                        first_frame = False
                    self.publish_frame(frame)
                if self.video_allowed.is_set():
                    self.get_logger().warn('Camera stream ended.',
                                           throttle_duration_sec=10.0)
            except Exception as exc:
                if self.video_allowed.is_set():
                    self.get_logger().warn(f'Camera stream problem: {exc}',
                                           throttle_duration_sec=10.0)
            self.stop_event.wait(self.reconnect_delay)

    def publish_frame(self, frame: bytes):
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.image_frame_id
        msg.format = 'jpeg'
        msg.data = frame
        self.image_pub.publish(msg)

        self.frame_count += 1
        now = self.get_clock().now()
        elapsed = (now - self.stats_time).nanoseconds / 1e9
        if self.stats_interval > 0.0 and elapsed >= self.stats_interval:
            self.get_logger().info(
                f'Video: {self.frame_count / elapsed:.1f} frames/s'
            )
            self.frame_count = 0
            self.stats_time = now

    # ---- Telemetry ----------------------------------------------------------
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
        self.ir_raw_pub.publish(String(data=line))

        bits = parse_ir_bits(line)
        cam = parse_cam_state(line)
        mic = parse_mic_mode(line)
        level = parse_level(line)
        led = parse_led_state(line)

        if bits is not None:
            self.ir_pub.publish(UInt8MultiArray(data=bits))
        if cam is not None:
            self.update_camera_state(cam)
        if mic is not None:
            self.update_mic_state(mic)
        if level is not None:
            self.level_pub.publish(Float32(data=level))
        if led is not None:
            self.update_led_state(led)
        if (bits is None and cam is None and mic is None
                and level is None and led is None):
            self.get_logger().warn(f'Unrecognized telemetry: {line!r}',
                                   throttle_duration_sec=10.0)

    # ---- Camera on/off --------------------------------------------------------
    def update_camera_state(self, running: bool):
        if running != self.camera_running:
            self.camera_running = running
            self.enabled_pub.publish(Bool(data=running))
            self.get_logger().info(
                f'Camera is {"on" if running else "off"} (reported by the board)'
            )
        # Follow the board, unless a request is still waiting for it.
        if self.desired is None or running == self.desired:
            if running:
                self.video_allowed.set()
            else:
                self.video_allowed.clear()

    def request_camera(self, on: bool, source: str):
        """The last request wins. The command is repeated until confirmed."""
        self.desired = on
        self.retries_left = self.command_retries
        if on:
            self.video_allowed.set()
        else:
            self.video_allowed.clear()
        self.send_text('CAM,ON' if on else 'CAM,OFF')
        self.get_logger().info(
            f'Camera {"on" if on else "off"} requested ({source})'
        )

    def retry_camera(self):
        if self.desired is None:
            return
        if self.camera_running == self.desired:
            self.desired = None
            return
        if self.retries_left <= 0:
            self.get_logger().warn(
                'The camera board did not confirm the camera switch. '
                'Is it powered, and does it run the firmware with CAM support?'
            )
            self.desired = None
            if self.camera_running is None or self.camera_running:
                self.video_allowed.set()   # nothing changed: keep the video going
            return
        self.retries_left -= 1
        self.send_text('CAM,ON' if self.desired else 'CAM,OFF')

    def enable_service(self, request, response):
        if self.desired is None and self.camera_running == request.data:
            response.success = True
            response.message = (
                f'The camera is already {"on" if request.data else "off"}.'
            )
            return response
        self.request_camera(bool(request.data), 'service camera/enable')
        response.success = True
        response.message = (
            f'Camera {"on" if request.data else "off"} requested: '
            f'watch camera/enabled for the board\'s answer.'
        )
        return response

    # ---- Microphone -------------------------------------------------------------
    def update_mic_state(self, mode: int):
        first_report = self.mic_reported is None
        if mode != self.mic_reported:
            self.mic_reported = mode
            self.mic_mode_pub.publish(String(data=MIC_NAMES[mode]))
            self.get_logger().info(
                f'Microphone is {MIC_NAMES[mode]} (reported by the board)'
            )
        if (first_report and mode != 0 and self.mic_desired is None
                and self.mic_off_on_start):
            self.get_logger().warn(
                'The board reports the microphone on at start-up: switching it off.'
            )
            self.request_mic(0, 'start-up safety')

    def request_mic(self, mode: int, source: str):
        """The last request wins. The command is repeated until confirmed."""
        self.mic_desired = mode
        self.mic_retries_left = self.command_retries
        self.send_text(MIC_COMMANDS[mode])
        self.get_logger().info(
            f'Microphone {MIC_NAMES[mode]} requested ({source})'
        )

    def retry_mic(self):
        if self.mic_desired is None:
            return
        if self.mic_reported == self.mic_desired:
            self.mic_desired = None
            return
        if self.mic_retries_left <= 0:
            self.get_logger().warn(
                'The camera board did not confirm the microphone mode. '
                'Is it powered, and does it run the firmware with MIC support?'
            )
            self.mic_desired = None
            return
        self.mic_retries_left -= 1
        self.send_text(MIC_COMMANDS[self.mic_desired])

    def retry_timer(self):
        self.retry_camera()
        self.retry_mic()

    def mic_mode_callback(self, msg: String):
        name = msg.data.strip().lower()
        if name not in MIC_NUMBERS:
            self.get_logger().warn(
                f'microphone/set_mode: "{msg.data}" is not off, level or stream'
            )
            return
        self.request_mic(MIC_NUMBERS[name], 'topic microphone/set_mode')

    def mic_enable_service(self, request, response):
        mode = 2 if request.data else 0
        if self.mic_desired is None and self.mic_reported == mode:
            response.success = True
            response.message = f'The microphone is already {MIC_NAMES[mode]}.'
            return response
        self.request_mic(mode, 'service microphone/enable')
        response.success = True
        response.message = (
            f'Microphone {MIC_NAMES[mode]} requested: '
            f'watch microphone/mode for the board\'s answer.'
        )
        return response

    # ---- Audio ---------------------------------------------------------------------
    def audio_timer(self):
        for _ in range(10):
            try:
                data, _addr = self.audio_rx.recvfrom(2048)
            except BlockingIOError:
                return
            except OSError as exc:
                self.get_logger().warn(f'Audio receive failed: {exc}',
                                       throttle_duration_sec=5.0)
                return
            self.handle_audio(data)

    def handle_audio(self, data: bytes):
        packet = parse_audio_packet(data)
        if packet is None:
            return
        sequence, payload = packet

        if self.last_audio_seq is not None:
            self.audio_lost += audio_gap(self.last_audio_seq, sequence)
        self.last_audio_seq = sequence

        samples = array.array('h')
        samples.frombytes(payload)
        if sys.byteorder == 'big':
            samples.byteswap()
        self.audio_pub.publish(Int16MultiArray(data=samples))

        self.audio_packets += 1
        now = time.monotonic()
        if (self.stats_interval > 0.0
                and now - self.audio_stats_time >= self.stats_interval):
            self.get_logger().info(
                f'Audio: {self.audio_packets} packets, {self.audio_lost} lost'
            )
            self.audio_packets = 0
            self.audio_lost = 0
            self.audio_stats_time = now

    # ---- LEDs ------------------------------------------------------------------------
    def update_led_state(self, state: str):
        words = state.split()
        if words and words[0] in LED_MODES and words[0] != 'off':
            self.led_last_mode = words[0]
        if state != self.led_reported:
            self.led_reported = state
            self.led_state_pub.publish(String(data=state))

    def toggle_lights(self, source: str):
        """Off if the strip is lit; otherwise back to the last mode it was in."""
        words = (self.led_reported or 'off').split()
        lit = bool(words) and words[0] != 'off'
        if lit:
            target = 'off'
        elif self.led_last_mode is not None:
            target = self.led_last_mode
        else:
            target = self.lights_on_mode
        self.send_repeated(f'LED,MODE,{target}')
        self.get_logger().info(f'Lights {target} requested ({source})')

    def led_color_callback(self, msg: ColorRGBA):
        self.send_repeated(
            f'LED,COLOR,{to_byte(msg.r)},{to_byte(msg.g)},{to_byte(msg.b)}'
        )

    def led_mode_callback(self, msg: String):
        name = msg.data.strip().lower()
        if name not in LED_MODES:
            self.get_logger().warn(
                f'leds/mode: "{msg.data}" is not one of {", ".join(LED_MODES)}'
            )
            return
        self.send_repeated(f'LED,MODE,{name}')

    def led_brightness_callback(self, msg: UInt8):
        self.send_repeated(f'LED,BRIGHT,{int(clamp(int(msg.data), 0, 100))}')

    def led_pixel_callback(self, msg: UInt8MultiArray):
        values = [int(v) for v in msg.data]
        if len(values) < 4:
            self.get_logger().warn('leds/pixel needs four numbers: index, r, g, b')
            return
        index, r, g, b = (int(clamp(v, 0, 255)) for v in values[:4])
        self.send_repeated(f'LED,PIXEL,{index},{r},{g},{b}')

    # ---- Gamepad: D-pad up toggles the camera, D-pad down the microphone -----------
    @staticmethod
    def axis_pressed(msg, axis, direction, threshold):
        if 0 <= axis < len(msg.axes):
            return msg.axes[axis] * direction > threshold
        return False

    def joy_callback(self, msg: Joy):
        self.last_joy_time = self.get_clock().now()
        self.cam_pressed = self.axis_pressed(
            msg, self.toggle_axis, self.toggle_direction, self.toggle_threshold)
        self.mic_pressed = self.axis_pressed(
            msg, self.mic_toggle_axis, self.mic_toggle_direction,
            self.toggle_threshold)

    def toggle_timer(self):
        fresh = False
        if self.last_joy_time is not None:
            age = (self.get_clock().now() - self.last_joy_time).nanoseconds / 1e9
            fresh = age <= self.joy_timeout   # a silent /joy counts as released
        now = time.monotonic()

        pressed_up = self.cam_pressed and fresh
        if pressed_up:
            if self.up_since is None:
                self.up_since = now
        else:
            # A short hold of D-pad up, let go in time, toggles the lights.
            if (self.up_since is not None and fresh and self.light_toggle_min_s > 0.0
                    and not self.up_camera_fired
                    and self.light_toggle_min_s <= now - self.up_since < self.toggle_hold_s):
                self.toggle_lights('D-pad up, short hold')
            self.up_since = None
            self.up_camera_fired = False

        if self.cam_hold.update(pressed_up, now):
            self.up_camera_fired = True
            if self.desired is not None:
                current = self.desired
            elif self.camera_running is not None:
                current = self.camera_running
            else:
                current = True
            self.request_camera(not current, 'D-pad up held')

        if self.mic_hold.update(self.mic_pressed and fresh, now):
            if self.mic_desired is not None:
                current = self.mic_desired
            elif self.mic_reported is not None:
                current = self.mic_reported
            else:
                current = 0
            self.request_mic(self.mic_toggle_mode if current == 0 else 0,
                             'D-pad down held')

    # ---- Shutdown ---------------------------------------------------------------------
    def cleanup(self):
        """Switch the microphone off, so it is never left streaming to nobody."""
        for _ in range(2):
            try:
                self.tx.sendto(b'MIC,OFF', (self.camera_host, self.cmd_port))
            except OSError:
                pass


def main(args=None):
    rclpy.init(args=args)
    node = RoverCameraBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.stop_event.set()
        node.cleanup()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
