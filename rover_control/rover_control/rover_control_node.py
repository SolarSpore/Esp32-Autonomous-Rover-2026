#!/usr/bin/env python3
"""
rover_control_node.py

Reads the Steam Deck gamepad on /joy and publishes /cmd_vel for
rover_motor_bridge to drive the wheels. Replaces teleop_twist_joy so all
driving modes live in one place. Also turns the right stick and B button
into head/buzzer intents (topics only; a separate bridge sends them to the
rover, the same way rover_motor_bridge sends the drive commands).

Modes
-----
Arcade (default):
  Left stick  -> forward/back (Y) and steering (X)
  L1          -> deadman (required to move, in every mode)
  R1          -> turbo

Tank (toggled):
  Hold A for `tank_hold_seconds`, then release, to turn tank mode on.
  Do the same again to turn it back off. A quick tap does nothing.
  Left stick  -> left wheels
  Right stick -> right wheels
  L1 / R1     -> same as arcade

Strafe (mecanum, X-pattern rollers):
  Hold X, D-pad left/right drives the rover sideways via linear.y.
  Works from either arcade or tank mode -- whichever was active before X
  was pressed resumes the instant X is released. Overrides stick input
  while held: this is a pure sideways command, not blended with steering.
  L1 is still required, same as every other mode.

Head swivel (SG90) -- does not touch driving:
  Right stick X sets the head's turning SPEED, not its position.
  Speed = max_rate * |stick| ** head_curve_exponent, ramped at
  head_accel_dps2. Release the stick and the head holds its angle.
  Requires L1. Releasing L1 freezes the head. Disabled in tank mode
  (the right stick drives the wheels there). Nothing is published until
  the stick is first moved, so the head does not jump at startup.
  Publishes head/angle (std_msgs/Float32, degrees, center = 90).

Buzzer:
  B plays one beep (requires L1). Publishes buzzer/beep (std_msgs/Empty).

Releasing L1 always stops the rover, in every mode.
"""

import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty, Float32


class RoverControlNode(Node):

    def __init__(self):
        super().__init__('rover_control_node')

        # ---- Parameters -----------------------------------------------
        self.declare_parameter('enable_button', 9)
        self.declare_parameter('turbo_button', 10)
        self.declare_parameter('tank_toggle_button', 0)
        self.declare_parameter('strafe_button', 2)
        self.declare_parameter('tank_hold_seconds', 3.0)

        self.declare_parameter('axis_forward', 1)
        self.declare_parameter('axis_steer', 0)
        self.declare_parameter('axis_left_wheel', 1)
        self.declare_parameter('axis_right_wheel', 3)
        self.declare_parameter('axis_dpad_x', 6)

        self.declare_parameter('scale_linear', 0.7)
        self.declare_parameter('scale_linear_turbo', 1.5)
        self.declare_parameter('scale_angular', 4.0)
        self.declare_parameter('scale_angular_turbo', 4.0)
        self.declare_parameter('scale_strafe', 0.7)
        self.declare_parameter('scale_strafe_turbo', 1.5)
        self.declare_parameter('wheel_separation', 0.30)

        # Head and buzzer. A wrong index here only disables the head or
        # buzzer; it can never stop driving.
        self.declare_parameter('buzzer_button', 1)
        self.declare_parameter('axis_head', 2)
        self.declare_parameter('head_in_tank_mode', False)
        self.declare_parameter('head_direction', 1.0)
        self.declare_parameter('head_max_rate_dps', 120.0)
        self.declare_parameter('head_curve_exponent', 2.0)
        self.declare_parameter('head_accel_dps2', 600.0)
        self.declare_parameter('head_min_deg', 0.0)
        self.declare_parameter('head_max_deg', 180.0)
        self.declare_parameter('head_center_deg', 90.0)
        self.declare_parameter('head_update_hz', 50.0)
        self.declare_parameter('joy_timeout', 0.5)

        g = lambda n: self.get_parameter(n).value
        self.enable_button = int(g('enable_button'))
        self.turbo_button = int(g('turbo_button'))
        self.tank_toggle_button = int(g('tank_toggle_button'))
        self.strafe_button = int(g('strafe_button'))
        self.tank_hold_seconds = float(g('tank_hold_seconds'))

        self.axis_forward = int(g('axis_forward'))
        self.axis_steer = int(g('axis_steer'))
        self.axis_left_wheel = int(g('axis_left_wheel'))
        self.axis_right_wheel = int(g('axis_right_wheel'))
        self.axis_dpad_x = int(g('axis_dpad_x'))

        self.scale_linear = float(g('scale_linear'))
        self.scale_linear_turbo = float(g('scale_linear_turbo'))
        self.scale_angular = float(g('scale_angular'))
        self.scale_angular_turbo = float(g('scale_angular_turbo'))
        self.scale_strafe = float(g('scale_strafe'))
        self.scale_strafe_turbo = float(g('scale_strafe_turbo'))
        self.wheel_separation = float(g('wheel_separation'))

        self.buzzer_button = int(g('buzzer_button'))
        self.axis_head = int(g('axis_head'))
        self.head_in_tank_mode = bool(g('head_in_tank_mode'))
        self.head_direction = float(g('head_direction'))
        self.head_max_rate_dps = float(g('head_max_rate_dps'))
        self.head_curve_exponent = float(g('head_curve_exponent'))
        self.head_accel_dps2 = float(g('head_accel_dps2'))
        self.head_min_deg = float(g('head_min_deg'))
        self.head_max_deg = float(g('head_max_deg'))
        self.head_center_deg = float(g('head_center_deg'))
        self.head_update_hz = float(g('head_update_hz'))
        self.joy_timeout = float(g('joy_timeout'))

        # Driving-only indices. The head and buzzer indices are checked
        # separately so they can never block the driving path.
        self._max_button = max(self.enable_button, self.turbo_button,
                                self.tank_toggle_button, self.strafe_button)
        self._max_axis = max(self.axis_forward, self.axis_steer,
                              self.axis_left_wheel, self.axis_right_wheel,
                              self.axis_dpad_x)

        # ---- State ------------------------------------------------------
        self.tank_mode = False
        self.tank_button_down_since = None   # wall-clock time A was pressed
        self.tank_toggled_this_hold = False   # avoid double-toggling on jitter
        self.strafe_active = False
        self.last_enabled = False             # for the "release L1 -> stop" edge

        # Head
        self.head_angle = self.head_center_deg
        self.head_rate = 0.0                  # degrees/second, signed
        self.head_axis_value = 0.0            # latest allowed stick value
        self.head_engaged = False             # no head output until first use
        self.last_joy_time = None
        self.head_last_update = time.monotonic()

        # Buzzer
        self.buzzer_was_pressed = False

        # ---- ROS interfaces ----------------------------------------------
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.head_pub = self.create_publisher(Float32, 'head/angle', 10)
        self.buzzer_pub = self.create_publisher(Empty, 'buzzer/beep', 10)
        self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        if self.head_update_hz > 0.0:
            self.create_timer(1.0 / self.head_update_hz,
                              self.head_timer_callback)

        self.get_logger().info(
            f'rover_control_node started. Arcade by default; hold button '
            f'{self.tank_toggle_button} for {self.tank_hold_seconds:.1f}s to '
            f'toggle tank mode; hold button {self.strafe_button} + D-pad '
            f'left/right to strafe (works in either mode). Head: axis '
            f'{self.axis_head}, buzzer: button {self.buzzer_button} '
            f'(L1 held).'
        )

    # ------------------------------------------------------------------
    def joy_callback(self, msg: Joy):
        if len(msg.buttons) <= self._max_button or len(msg.axes) <= self._max_axis:
            self.get_logger().warn(
                'Joy message shorter than the configured button/axis indices',
                throttle_duration_sec=5.0,
            )
            self.head_axis_value = 0.0
            self.head_rate = 0.0
            self.stop()
            return

        enabled = msg.buttons[self.enable_button] == 1
        turbo = msg.buttons[self.turbo_button] == 1

        self.update_tank_toggle(msg)
        self.update_strafe(msg, enabled)

        try:
            self.update_aux(msg, enabled)
        except Exception as exc:  # head/buzzer must never block driving
            self.get_logger().error(
                f'head/buzzer input error: {exc}',
                throttle_duration_sec=5.0,
            )

        if not enabled:
            if self.last_enabled:
                self.stop()
            self.last_enabled = False
            return
        self.last_enabled = True

        if self.strafe_active:
            # Pure sideways command while X is held. Overrides stick input
            # in either mode; whichever mode was active resumes on release.
            self.pub.publish(self.strafe_twist(msg, turbo))
            return

        if self.tank_mode:
            self.pub.publish(self.tank_twist(msg, turbo))
        else:
            self.pub.publish(self.arcade_twist(msg, turbo))

    # ------------------------------------------------------------------
    def update_tank_toggle(self, msg: Joy):
        """Hold tank_toggle_button for tank_hold_seconds, then release, to
        flip tank_mode. A tap shorter than the hold time does nothing."""
        pressed = msg.buttons[self.tank_toggle_button] == 1
        now = time.monotonic()

        if pressed:
            if self.tank_button_down_since is None:
                self.tank_button_down_since = now
                self.tank_toggled_this_hold = False
            held_for = now - self.tank_button_down_since
            if held_for >= self.tank_hold_seconds and not self.tank_toggled_this_hold:
                # Mark it now so we don't re-fire every callback while still
                # held; the actual mode flips only once, on release below.
                self.tank_toggled_this_hold = True
        else:
            if self.tank_button_down_since is not None:
                held_for = now - self.tank_button_down_since
                if held_for >= self.tank_hold_seconds:
                    self.tank_mode = not self.tank_mode
                    self.get_logger().info(
                        f'Tank mode {"ON" if self.tank_mode else "OFF"} '
                        f'(held {held_for:.1f}s)'
                    )
                    self.pub.publish(Twist())  # stop on mode change
            self.tank_button_down_since = None
            self.tank_toggled_this_hold = False

    # ------------------------------------------------------------------
    def update_strafe(self, msg: Joy, enabled: bool):
        held = enabled and msg.buttons[self.strafe_button] == 1

        if held and not self.strafe_active:
            self.strafe_active = True
            self.get_logger().info('Strafe active (X held)')
        elif not held and self.strafe_active:
            self.strafe_active = False
            self.pub.publish(Twist())  # stop on the way out; caller mode resumes next callback
            self.get_logger().info('Strafe released')

    # ------------------------------------------------------------------
    # Head and buzzer
    # ------------------------------------------------------------------
    def update_aux(self, msg: Joy, enabled: bool):
        """Record the buzzer button and the head stick. Both need L1. The
        head also needs to be out of tank mode (unless head_in_tank_mode)."""
        self.last_joy_time = time.monotonic()

        # Buzzer: one beep per press (edge), never a repeating tone.
        pressed = False
        if 0 <= self.buzzer_button < len(msg.buttons):
            pressed = enabled and msg.buttons[self.buzzer_button] == 1
        if pressed and not self.buzzer_was_pressed:
            self.buzzer_pub.publish(Empty())
        self.buzzer_was_pressed = pressed

        # Head: remember the stick only while it is allowed to move.
        allowed = (enabled
                   and (self.head_in_tank_mode or not self.tank_mode)
                   and 0 <= self.axis_head < len(msg.axes))
        if allowed:
            self.head_axis_value = msg.axes[self.axis_head]
        else:
            self.head_axis_value = 0.0
            self.head_rate = 0.0   # freeze immediately, no coasting

    def head_curve(self, stick: float) -> float:
        """Shape the stick: small deflection = slow, full = fast."""
        magnitude = min(1.0, abs(stick))
        return math.copysign(magnitude ** self.head_curve_exponent, stick)

    def head_timer_callback(self):
        now = time.monotonic()
        dt = min(now - self.head_last_update, 0.1)
        self.head_last_update = now

        joy_fresh = (self.last_joy_time is not None
                     and now - self.last_joy_time <= self.joy_timeout)
        stick = self.head_axis_value if joy_fresh else 0.0
        if not joy_fresh:
            self.head_rate = 0.0

        # Stay silent until the stick is first used, so the head does not
        # snap to center at startup.
        if not self.head_engaged:
            if stick == 0.0:
                return
            self.head_engaged = True

        # Target speed from the stick, then ramp toward it.
        target_rate = (self.head_direction * self.head_max_rate_dps
                       * self.head_curve(stick))
        max_step = self.head_accel_dps2 * dt
        delta = max(-max_step, min(max_step, target_rate - self.head_rate))
        self.head_rate += delta

        # Integrate into an angle and stop at the limits.
        new_angle = self.head_angle + self.head_rate * dt
        clamped = max(self.head_min_deg, min(self.head_max_deg, new_angle))
        if clamped != new_angle:
            self.head_rate = 0.0
        self.head_angle = clamped

        out = Float32()
        out.data = float(self.head_angle)
        self.head_pub.publish(out)

    # ------------------------------------------------------------------
    def arcade_twist(self, msg: Joy, turbo: bool) -> Twist:
        lin_scale = self.scale_linear_turbo if turbo else self.scale_linear
        ang_scale = self.scale_angular_turbo if turbo else self.scale_angular

        t = Twist()
        t.linear.x = msg.axes[self.axis_forward] * lin_scale
        t.angular.z = msg.axes[self.axis_steer] * ang_scale
        return t

    def tank_twist(self, msg: Joy, turbo: bool) -> Twist:
        scale = self.scale_linear_turbo if turbo else self.scale_linear
        left = msg.axes[self.axis_left_wheel] * scale
        right = msg.axes[self.axis_right_wheel] * scale

        t = Twist()
        t.linear.x = (left + right) / 2.0
        if self.wheel_separation > 0.0:
            t.angular.z = (right - left) / self.wheel_separation
        return t

    def strafe_twist(self, msg: Joy, turbo: bool) -> Twist:
        scale = self.scale_strafe_turbo if turbo else self.scale_strafe
        dpad = msg.axes[self.axis_dpad_x]

        t = Twist()
        if dpad > 0.5:
            t.linear.y = scale
        elif dpad < -0.5:
            t.linear.y = -scale
        # else: X held but no D-pad direction pressed -> hold still (0,0,0)
        return t

    # ------------------------------------------------------------------
    def stop(self):
        self.pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = RoverControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
