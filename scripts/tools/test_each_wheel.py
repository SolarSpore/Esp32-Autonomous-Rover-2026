#!/usr/bin/env python3
"""
test_each_wheel.py  (was wheel_test.py before the human-friendly rename)

Drives each wheel channel alone over raw UDP, bypassing ROS entirely.
Use this with the rover on blocks to verify wiring: the wheels should
move in order FL, FR, RL, RR, all forward.

Stop rover_motor_bridge (and rover_wheel_test) before running this --
two things sending to the ESP32 at once shows up as motor stutter.
"""
import socket, time
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
addr = ('192.168.1.26', 4210)
tests = [('FL', '200,0,0,0'), ('FR', '0,200,0,0'), ('RL', '0,0,200,0'), ('RR', '0,0,0,200')]
for name, cmd in tests:
    print('Driving channel', name, 'forward at 200')
    end = time.time() + 2.0
    while time.time() < end:
        s.sendto(cmd.encode(), addr)
        time.sleep(0.05)
    s.sendto(b'0,0,0,0', addr)
    time.sleep(1.5)
print('done')
