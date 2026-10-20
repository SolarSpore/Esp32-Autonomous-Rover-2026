# Tools

Test scripts and a controller helper for the rover. On the Steam Deck they live in `~/Robotics/tools` (see `Setup-Guide.md`, step 5).

| Tool | What it does |
|---|---|
| `test_each_wheel.py` | Drives the wheels one at a time, front left, front right, rear left, rear right, forward at PWM 200 for 2 seconds each. Use it to check wiring and wheel order |
| `hold_all_wheels.py <pwm>` | Holds all four wheels at one PWM value for 8 seconds. Use it to find the stall zone and to check for jitter |
| `joy_mapper.py` | A browser-based wizard that finds which axis or button each control uses on your gamepad, and writes the result to a YAML file |

## The wheel tests

`test_each_wheel.py` and `hold_all_wheels.py` send raw UDP packets to the ESP32 at `192.168.1.26`, port 4210, and do not use ROS.

* Put the rover on blocks first.
* Stop the rover controls first (the Stop Rover Controls icon). Two programs sending to the same ESP32 make the motors stutter.
* If your rover uses a different address, edit it in the script before you run it.

```bash
python3 ~/Robotics/tools/test_each_wheel.py
python3 ~/Robotics/tools/hold_all_wheels.py 120
```

The first command moves the wheels in the order front left, front right, rear left, rear right, all forward. The second holds all four at PWM 120 (change the number to the value you want to test). Start the rover controls again afterwards.

## The controller wizard

`joy_mapper.py` listens to `/joy` and serves a web page. It walks through each control the rover needs (throttle, brake, steering, the two tank sticks, the tank mode button and an optional enable button), detects which axis or button moved, and writes the result to `~/Robotics/Rover/rover_control/config/joy_mapping.yaml`. The rover's teleoperation node does not read that file automatically.

It needs `joy_node` running, but not the rest of the rover stack. Click Stop Rover Controls, then in one Konsole:

```bash
cd ~/Robotics/ros2_ws && pixi run ros2 run joy joy_node
```

and in a second Konsole:

```bash
cd ~/Robotics/ros2_ws && pixi run python3 ~/Robotics/tools/joy_mapper.py
```

Then open `http://localhost:8899` in a browser on the Deck and follow the prompts. Press Ctrl+C in both Konsoles when you are done.
