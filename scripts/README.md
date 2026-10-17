# Steam Deck scripts

These are plain scripts and desktop launchers for the Steam Deck. They are not a ROS 2
package and are meant to live directly on the Deck's filesystem, not inside the ROS workspace.

## Install on the Deck

Run these from this `scripts/` folder:

    mkdir -p ~/Robotics ~/Robotics/logs
    cp start_rover_controls.sh stop_rover_controls.sh ~/Robotics/
    chmod +x ~/Robotics/start_rover_controls.sh ~/Robotics/stop_rover_controls.sh
    cp desktop/*.desktop ~/Desktop/
    chmod +x ~/Desktop/RoverControls.desktop ~/Desktop/StopRoverControls.desktop
    mkdir -p ~/Robotics/tools
    cp -n tools/*.py ~/Robotics/tools/

KDE may ask you to right-click a new `.desktop` icon and explicitly allow it to run, the first time.
`cp -n` never overwrites a file that is already in `~/Robotics/tools`.

## What is here

- `start_rover_controls.sh` / `stop_rover_controls.sh`: launch and stop the rover's ROS 2 nodes
  (teleop, motor bridge, peripherals bridge) and `foxglove_bridge` together, detached, with a lock
  so a double-click cannot stack a second launch on top of a running one.
- `desktop/`: the two desktop icons that call the scripts above.
- `tools/`: standalone UDP test scripts for bring-up and debugging, independent of ROS
  (`hold_all_wheels.py`, `test_each_wheel.py`), plus `joy_mapper.py`, a browser-based controller
  calibration wizard moved here from `rover_control/`. Stop `rover_motor_bridge` before running
  the UDP tests.

## Note

These scripts have the rover's IP address and ports built in. Generic versions with editable
settings live in the platform repository under `template_tools/`.
