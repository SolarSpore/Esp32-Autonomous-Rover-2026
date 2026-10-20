# Rover shortcuts (Steam Deck)

The start and stop scripts and the desktop icons for the rover. They are plain scripts, not a ROS 2
package, and are meant to live directly on the Deck's filesystem, not inside the ROS workspace.

## Before you use them

The scripts expect Pixi installed and the ROS workspace built at `~/Robotics/ros2_ws`, with this
repository at `~/Robotics/Rover`. Steps 1 to 4 of `../Setup-Guide.md` set that up.

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

## Using them

1. Click "Rover Controls" and wait for the notification.
2. Hold L1 (the deadman) and drive.
3. Right-click the icon and choose "Stop Rover Controls" when done.

Each part writes its own log in `~/Robotics/logs`: `foxglove.log`, `teleop.log`, `bridge.log` and
`peripherals.log`. Check them first if something does not start.

## How the icons work

Each icon's `Exec=` line holds the full path of a script, because launchers do not use your shell
PATH. The scripts use full paths too: `/home/deck/.pixi/bin/pixi` for Pixi,
`/home/deck/Robotics/ros2_ws` for the workspace and `/home/deck/Robotics/logs` for the logs. They are
written for the default `deck` user. A notification appears when the rover controls start or stop.

## Adapting them

The top of `start_rover_controls.sh` sets three paths:

    PIXI=/home/deck/.pixi/bin/pixi
    WS=/home/deck/Robotics/ros2_ws
    LOGDIR=/home/deck/Robotics/logs

Change `PIXI` if Pixi is installed somewhere else, `WS` if your workspace is in a different folder,
and `LOGDIR` if you want the logs elsewhere. If your username is not `deck`, change all three. Then
change the `Exec=` line in each `.desktop` file to the new location of the scripts, and run the
`chmod +x` commands again.

The `start bridge` line passes `max_linear_speed:=1.5` and `min_pwm:=110` to the motor bridge. Edit
them to change the top speed or the minimum motor power. The rover's IP address is not set in this
script: both bridges default to `192.168.1.26`, and you can change it by adding `-p robot_ip:=YOUR_IP`
to the `start bridge` and `start peripherals` lines.

## Troubleshooting

- An icon does nothing: check the `Exec=` path, make sure the script is executable, and read the logs.
- Controls behave strangely: two `joy_node`s may be running. Run `pkill -f joy_node`, then click
  Rover Controls again.
