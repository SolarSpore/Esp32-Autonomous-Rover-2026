#!/usr/bin/env python3
"""
hold_all_wheels.py  (was hold_pwm.py before the human-friendly rename)

Holds all four wheels at one PWM value for 8 seconds, over raw UDP.
Used during bring-up to find the L298N stall zone (this rover's motors
stall below roughly PWM 80-100) and to check for jitter independent of
the ROS/joystick path.

Usage: python3 hold_all_wheels.py <pwm>
"""
import socket, sys, time
pwm = int(sys.argv[1])
cmd = f'{pwm},{pwm},{pwm},{pwm}'.encode()
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
addr = ('192.168.1.26', 4210)
print('Holding', cmd, 'for 8s')
try:
    end = time.time() + 8
    while time.time() < end:
        s.sendto(cmd, addr)
        time.sleep(0.05)
finally:
    s.sendto(b'0,0,0,0', addr)
