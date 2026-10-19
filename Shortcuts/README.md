# Rover shortcuts (Steam Deck)

The start and stop scripts and the desktop icons for the rover. They are plain scripts, not a ROS 2
package, and are meant to live directly on the Deck's filesystem, not inside the ROS workspace.

## Install on the Deck

Run these from the `Shortcuts/` folder of this repository:

    mkdir -p ~/Robotics ~/Robotics/logs ~/Robotics/Shortcuts
    cp start_rover_controls.sh stop_rover_controls.sh ~/Robotics/Shortcuts/
    chmod +x ~/Robotics/Shortcuts/start_rover_controls.sh ~/Robotics/Shortcuts/stop_rover_controls.sh
    cp desktop/*.desktop ~/Desktop/
    chmod +x ~/Desktop/RoverControls.desktop ~/Desktop/StopRoverControls.desktop

KDE may ask you to right-click a new `.desktop` icon and explicitly allow it to run, the first time.
The test tools in `../tools/` are installed separately: see `../tools/README.md`.

## What is here

- `start_rover_controls.sh`: stops any leftover rover processes, then starts `foxglove_bridge`,
  teleop (`rover_control`, with `joy_node`), `rover_motor_bridge` and `rover_peripherals_bridge`,
  all detached. Logs go to `~/Robotics/logs`. A lock stops a double-click from starting a second
  copy on top of a running one.
- `stop_rover_controls.sh`: stops all of the above.
- `desktop/RoverControls.desktop`: the "Rover Controls" icon. It runs the start script and has a
  right-click action, "Stop Rover Controls".
- `desktop/StopRoverControls.desktop`: a separate "Stop Rover Controls" icon.

## How the icons work

Each icon's `Exec=` line holds the full path of a script, because launchers do not use your shell
PATH. The scripts use full paths too: `/home/deck/.pixi/bin/pixi` for Pixi,
`/home/deck/Robotics/ros2_ws` for the workspace and `/home/deck/Robotics/logs` for the logs. They are
written for the default `deck` user. A notification appears when the rover controls start or stop.
