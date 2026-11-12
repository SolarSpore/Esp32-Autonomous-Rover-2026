# Rover Setup Guide

How to install this rover's software and get it driving. It has three parts, and a fourth that is planned:

- **Part 1, on the Steam Deck:** ROS 2 (through Pixi), the rover's ROS packages, the desktop shortcuts and Foxglove.
- **Part 2, on a PC:** the main rover board's firmware, with PlatformIO in Visual Studio Code, flashed over USB and then over Wi-Fi (OTA).
- **Part 3, on a PC (optional):** the camera board, a second ESP32 with a camera, a microphone, an LED strip and IR sensors.
- **Part 4 (planned, not fitted yet):** wheel odometry from encoders on the rear wheels.

The same Pixi, ROS 2 and Foxglove steps, explained in more detail and for building a different robot, are in the Distributed-Robotics-Platform-2026 repository (`docs/install_guide.md`): https://github.com/SolarSpore/Distributed-Robotics-Platform-2026

If you already followed sections 1 to 5 of that guide on this Deck, skip steps 1, 3 and 6 of Part 1 below: they repeat it. Step 3 would overwrite your workspace's `pixi.toml` and `pixi.lock`.

Wiring, print settings and assembly are not covered here. The pin numbers are constants at the top of `firmware/src/main.cpp` (and `firmware-cam/src/main.cpp` for the camera board), and the 3D-printed parts are in the `STL/` folder (see "3D-printed parts" below).

## What you need

- A Steam Deck in Desktop Mode, using the default `deck` user. The scripts and icons have `/home/deck/...` paths built in. All Deck commands run in Konsole.
- Internet access on the Deck. The first install downloads the whole ROS 2 environment and takes several minutes.
- The rover: an ESP32 (ESP32-WROOM-32 dev board) with its motor drivers, head servo, buzzer and sensors wired as the firmware expects.
- The rover's 3D-printed parts. The STL files are in the `STL/` folder at the root of this repository.
- A PC for Parts 2 and 3 (any Linux, Windows or macOS machine that can run Visual Studio Code) and a USB cable for the first flash.
- A Wi-Fi network that the Deck and the ESP32 boards can all join.
- Optional, for Part 3: a Seeed XIAO ESP32-S3 Sense board (camera and microphone), a USB-C data cable, a strip of 7 addressable LEDs (WS2812 type) and two digital IR sensors.
- Planned, for Part 4: two wheel encoders for the rear wheels.

## What goes where

```text
~/Robotics/
├── Rover/          this repository (cloned here)
├── ros2_ws/        the Pixi workspace, built from Rover/ros2_ws
├── Shortcuts/      start_rover_controls.sh and stop_rover_controls.sh
├── tools/          wheel test scripts and the controller mapper
└── logs/           one log file per started process (foxglove, teleop, bridge, peripherals, camera)
~/Desktop/          RoverControls.desktop, StopRoverControls.desktop, the Foxglove icon
~/.local/bin/       foxglove-steamdeck.sh
```

---

## 3D-printed parts

The STL files for the rover's printed parts are in the `STL/` folder at the root of this repository. After step 2 of Part 1 they are in `~/Robotics/Rover/STL`, and on the PC you flash from they are in the `STL` folder of the clone from section 8. Open them in your slicer to print. Print settings and assembly steps are not covered in this guide.

---

# Part 1. The Steam Deck

## 1. Install Pixi

```bash
curl -fsSL https://pixi.sh/install.sh | bash
```

Open a new Konsole (the installer adds `~/.pixi/bin` to PATH in `~/.bashrc`) and check `pixi --version`. The rover was tested with Pixi 0.79.

## 2. Get this repository

The repository must sit at `~/Robotics/Rover`: the workspace links and the shortcuts expect it there.

```bash
mkdir -p ~/Robotics
git clone https://github.com/SolarSpore/Esp32-Autonomous-Rover-2026.git ~/Robotics/Rover
```

## 3. Create the workspace

The repository's `ros2_ws/` folder holds the workspace definition: `pixi.toml` lists the software (ROS 2 Humble, the Foxglove bridge, the build tool) and `pixi.lock` pins the exact versions the rover was tested with. The working workspace is a separate folder, `~/Robotics/ros2_ws`, next to `Rover`.

```bash
mkdir -p ~/Robotics/ros2_ws/src
cp ~/Robotics/Rover/ros2_ws/pixi.toml ~/Robotics/Rover/ros2_ws/pixi.lock ~/Robotics/ros2_ws/
cd ~/Robotics/ros2_ws
pixi install
```

Check it:

```bash
pixi run bash -c 'echo $ROS_DISTRO; ros2 pkg list | wc -l'
```

It should print `humble` and a few hundred packages.

## 4. Link the rover packages and build

```bash
cd ~/Robotics/ros2_ws/src
ln -s ../../Rover/rover_control rover_control
ln -s ../../Rover/rover_motor_bridge rover_motor_bridge
ln -s ../../Rover/rover_peripherals_bridge rover_peripherals_bridge
ln -s ../../Rover/rover_camera_bridge rover_camera_bridge
cd ~/Robotics/ros2_ws
pixi run colcon build --symlink-install
ls install
```

Expected: `rover_control`, `rover_motor_bridge`, `rover_peripherals_bridge` and `rover_camera_bridge` in the listing. (`rover_description` is a placeholder and is not built. `rover_odometry` is added in Part 4. Without a camera board, skip the `rover_camera_bridge` link: the other nodes do not depend on it.)

## 5. Install the shortcuts and tools

```bash
cd ~/Robotics/Rover/Shortcuts
mkdir -p ~/Robotics/logs ~/Robotics/Shortcuts ~/Robotics/tools
cp start_rover_controls.sh stop_rover_controls.sh ~/Robotics/Shortcuts/
chmod +x ~/Robotics/Shortcuts/start_rover_controls.sh ~/Robotics/Shortcuts/stop_rover_controls.sh
cp desktop/*.desktop ~/Desktop/
chmod +x ~/Desktop/RoverControls.desktop ~/Desktop/StopRoverControls.desktop
cp -n ../tools/*.py ~/Robotics/tools/
```

KDE may ask you to right-click a new icon and allow it to run, the first time. `cp -n` never overwrites a file that is already in `~/Robotics/tools`.

### How the shortcuts work

The shortcuts start and stop everything the rover needs on the Deck: the Foxglove bridge, teleop, `joy_node`, the two bridges to the main ESP32 and the camera board's bridge. Each script, icon and log file is documented in [Shortcuts/README.md](Shortcuts/README.md), including how to adapt them to another setup. The short version:

- **`RoverControls.desktop`** is the "Rover Controls" icon. Its `Exec=` line is the full path of `start_rover_controls.sh`, and it has a right-click action, "Stop Rover Controls".
- **`StopRoverControls.desktop`** is a separate "Stop Rover Controls" icon.
- **`start_rover_controls.sh`** stops any leftover rover processes, then starts `foxglove_bridge`, teleop (`rover_control`, which also starts `joy_node`), `rover_motor_bridge`, `rover_peripherals_bridge` and `rover_camera_bridge`, all detached, with one log per part in `~/Robotics/logs` (`foxglove.log`, `teleop.log`, `bridge.log`, `peripherals.log`, `camera.log`). A lock stops a double-click from starting a second copy on top of a running one. A notification says "Started. Hold L1 to drive."
- **`stop_rover_controls.sh`** stops all of the above. The motor bridge sends a stop command to the rover when it shuts down, and the camera bridge switches the microphone off.

Launchers do not inherit your shell's PATH, so the scripts call Pixi by its full path (`/home/deck/.pixi/bin/pixi`) and use full paths for the workspace and logs. If you want the scripts elsewhere, change the `Exec=` lines in the two icons to match.

### Using the tools

Each tool is documented in [tools/README.md](tools/README.md): what it does, how to run it and what to expect. The short version, with the rover on blocks:

- **Wheel tests** (`test_each_wheel.py`, `hold_all_wheels.py`): standalone UDP scripts for checking wiring and motor behavior. Click Stop Rover Controls first, because two senders to the ESP32 make the motors stutter, then run them with `python3`:

```bash
python3 ~/Robotics/tools/test_each_wheel.py
python3 ~/Robotics/tools/hold_all_wheels.py 120
```

- **Controller wizard** (`joy_mapper.py`): a browser-based tool that finds which axis or button each control uses. It needs `joy_node` running, but not the rest of the rover stack. Click Stop Rover Controls, then in one Konsole:

```bash
cd ~/Robotics/ros2_ws && pixi run ros2 run joy joy_node
```

and in a second Konsole:

```bash
cd ~/Robotics/ros2_ws && pixi run python3 ~/Robotics/tools/joy_mapper.py
```

Open `http://localhost:8899` in a browser on the Deck and follow the prompts. The result is written to `~/Robotics/Rover/rover_control/config/joy_mapping.yaml`. The rover's teleop does not read that file automatically.

## 6. Foxglove

`foxglove_bridge` is already installed by Pixi and is started by the Rover Controls icon. It serves ROS topics on port 8765. Foxglove Studio, the viewer, runs in a container, because SteamOS does not allow system-wide installs. SteamOS 3.5 and later ships with Distrobox and Podman.

Create the container and install Studio. Download the Linux amd64 `.deb` package from Foxglove's website into `~/Downloads` first. The container shares your home folder, so it can see the file:

```bash
distrobox create --name foxglove --image ubuntu:24.04
ls ~/Downloads/*.deb
distrobox enter foxglove -- sudo apt update
distrobox enter foxglove -- sudo apt install -y libasound2t64 ~/Downloads/foxglove-studio*.deb
distrobox enter foxglove -- bash -c 'dpkg -l | grep -i foxglove; ls /opt/Foxglove | head'
```

Expected: a line for `foxglove-studio` and the application's files. The pattern `foxglove-studio*.deb` matches the downloaded file with or without a version number in its name. These commands match the install records of the container the rover was tested with (Ubuntu 24.04, Foxglove Studio 3.1.1). Report any step that fails on a clean Deck.

Add a launcher script and an icon:

```bash
mkdir -p ~/.local/bin
cat > ~/.local/bin/foxglove-steamdeck.sh <<'EOF'
#!/bin/bash

export XDG_RUNTIME_DIR=/run/user/1000
export WAYLAND_DISPLAY=wayland-0
export QT_QPA_PLATFORM=wayland

exec /usr/bin/distrobox-enter -n foxglove -- \
  /opt/Foxglove/foxglove-studio \
  --ozone-platform=wayland \
  --start-fullscreen \
  --foreground
EOF
chmod +x ~/.local/bin/foxglove-steamdeck.sh

cat > ~/Desktop/foxglove-steamdeck.desktop <<'EOF'
[Desktop Entry]
Name=Foxglove Studio
Comment=Robot visualization and control
Exec=/home/deck/.local/bin/foxglove-steamdeck.sh
Terminal=false
Type=Application
Icon=foxglove-studio
Categories=Development;Utility;
StartupNotify=true
EOF
chmod +x ~/Desktop/foxglove-steamdeck.desktop
```

To connect: start the rover controls, open Foxglove Studio, choose Open connection, pick Foxglove WebSocket, and enter `ws://localhost:8765`.

### Dashboard layouts

Foxglove layouts (dashboards) can be exported as JSON files and imported again. The rover's layouts go in the `foxglove/` folder of this repository, which is described in [foxglove/README.md](foxglove/README.md). That file also lists the topics and panels for the camera, microphone, LEDs and sensors.

## 7. Drive

1. Power on the rover and wait for it to join Wi-Fi. Both firmware bench tests are enabled by default (a buzzer jingle and a servo sweep), and the rover accepts no commands for about 19 seconds after it connects. Part 2 shows how to turn the tests off.
2. With Steam running, the Deck's controls appear as a virtual Xbox 360 gamepad at `/dev/input/js0`.
3. Click the Rover Controls icon. Wait for the notification.
4. Hold L1 (the deadman) and drive. Right-click the icon and choose Stop Rover Controls to stop.

The controls are defined in `rover_control/config/steam_deck_controls.yaml` and in the Deck-side camera bridge:

| Control | Action |
|---|---|
| L1 (button 9) | Deadman: nothing moves unless it is held (proportional trigger drive is the one exception) |
| R1 (button 10) | Turbo |
| Left stick up/down (axis 1) | Forward and back (arcade mode, the default) |
| Left stick left/right (axis 0) | Steering (arcade mode and trigger drive) |
| X (button 2) | Strafe, with D-pad left/right (axis 6) giving the direction |
| A (button 0), held 3 seconds | Toggle tank mode: the sticks' up/down axes (axes 1 and 3) then drive the left and right sides |
| L1 + left trigger + right trigger, held 3 seconds | Toggle proportional trigger drive (see below) |
| B (button 1) | Beep |
| Right stick left/right (axis 2) | Move the head |
| D-pad up, held 1 to 3 seconds, then released | Lights on or off |
| D-pad up, held 3 seconds | Camera on or off |
| D-pad down, held 3 seconds | Microphone off or on (level only) |

**Proportional trigger drive** is a third driving mode. The right trigger (axis 5) is the gas, in proportion to the pull. The left trigger (axis 4) brakes, then reverses: the speed is the right trigger minus the left. The left stick only steers. Boost works as in the other modes, and the speeds are the same. In this mode, **no L1 is needed**: letting go of the triggers stops the rover. Set `trigger_mode_requires_enable: true` in the yaml to require L1 here too. The tank toggle (A held 3 seconds) switches to tank mode and turns trigger drive off. Arcade stays the default at start-up.

The trigger drive, the light toggle and the microphone toggle are new: check each one with the rover on blocks before you drive it on the floor.

The rover is expected at `192.168.1.26`: the motor bridge's `robot_ip` parameter defaults to it, and so does the peripherals bridge. The simplest fix for a different network is to give the ESP32 that address in your router's settings (a DHCP reservation), and to use the same address as `upload_port` in Part 2.

---

# Part 2. The main board's firmware, on a PC

The firmware is a PlatformIO project in `firmware/`. Do this on a PC, not the Deck. The first flash needs a USB cable. After that you can update the rover over Wi-Fi.

## 8. Set up PlatformIO in Visual Studio Code

1. Install Visual Studio Code.
2. Open the Extensions panel, search for "PlatformIO IDE", install it, and restart Visual Studio Code when asked. The first start downloads the toolchain and takes a few minutes.
3. Get this repository on the PC:

```bash
git clone https://github.com/SolarSpore/Esp32-Autonomous-Rover-2026.git
```

4. In Visual Studio Code, choose File, Open Folder, and open the **`firmware`** folder (not the repository root). PlatformIO recognises it by the `platformio.ini` file. It contains `src/main.cpp` and `include/config.example.h`.

## 9. Create `config.h`

`config.h` holds your Wi-Fi details and the OTA password. It is not in the repository, and must never be committed. Copy the example and edit it:

```bash
cd firmware/include
cp config.example.h config.h
```

Open `config.h` and replace every `CHANGE_ME`:

| Setting | Meaning |
|---|---|
| `sta_ssid`, `sta_password` | Your Wi-Fi network. The rover tries it for 8 seconds at boot |
| `ap_ssid`, `ap_password` | The rover's own fallback access point, started if it cannot join your Wi-Fi |
| `mdnsHostname` | The rover's name on the network, and the OTA host name (`esp32rover` is the default used in this project) |
| `ota_password` | The password for over-the-air updates. Choose your own |

Optional: the bench tests at the top of `src/main.cpp` run a buzzer jingle and a servo sweep at every boot. Set `RUN_SERVO_BENCH_TEST` and `RUN_BUZZER_BENCH_TEST` to `false` to skip them.

The firmware also has stand-in settings for **wheel encoders on the rear wheels** (`ENCODERS_FITTED`, `WHEEL_DIAMETER_MM`, `ENCODER_COUNTS_PER_WHEEL_REV`). They are off until the encoders are fitted: see Part 4. The two IR sensors that used to be on this board are now on the camera board (Part 3).

## 10. Build

`platformio.ini` defines two environments: `esp32dev` (USB) and `esp32dev-ota` (Wi-Fi). Build the first one from the PlatformIO toolbar (the check mark) or in the PlatformIO terminal (the terminal icon in the toolbar):

```bash
pio run -e esp32dev
```

## 11. Flash over USB (the first time, and for recovery)

1. Plug the ESP32 into the PC. On Linux it appears as `/dev/ttyUSB0`, which is what `upload_port` and `monitor_port` use in `platformio.ini`. On Windows or macOS, change both lines to the port you see, or delete them and let PlatformIO detect it.
2. Upload:

```bash
pio run -e esp32dev -t upload
```

3. Watch the output (baud rate 115200):

```bash
pio device monitor -e esp32dev
```

- If the upload hangs at "Connecting...", hold the BOOT button on the board until it starts (some boards need this).
- On Linux, a "permission denied" on `/dev/ttyUSB0` means your user is not in the group that owns the port. `ls -l /dev/ttyUSB0` shows the group (`dialout` on Ubuntu, `uucp` on Arch). Add yourself with `sudo usermod -aG <group> $USER` and log in again.

After the first flash the rover joins your Wi-Fi, and it announces itself as `mdnsHostname` for over-the-air updates. USB stays the recovery method if an OTA update ever fails.

## 12. Flash over Wi-Fi (OTA, by IP address)

The `esp32dev-ota` environment uploads over the network to the rover's IP address with `espota`. Edit these two things first:

1. In `platformio.ini`, set `upload_port` in `[env:esp32dev-ota]` to your rover's IP address (it is `192.168.1.26` in the repository).
2. The password is not in the file: it comes from the `OTA_PASSWORD` environment variable, and it must match `ota_password` in `config.h`. Set it in the terminal you upload from:

```bash
export OTA_PASSWORD='the same password as ota_password in config.h'
pio run -e esp32dev-ota -t upload
```

On Windows PowerShell the variable is set with `$env:OTA_PASSWORD='...'`. If you upload with the PlatformIO toolbar buttons instead of a terminal, start Visual Studio Code from a terminal where the variable is already set (`code .`), because the buttons use Visual Studio Code's own environment. A button upload without the variable sends an empty password and fails with "Authentication Failed".

Put the rover on blocks first. When an update starts, the firmware stops the motors, servo and buzzer; when it finishes the rover reboots. The PC and the rover must be on the same network, and the rover must have joined your Wi-Fi (not its fallback access point).

If an OTA upload fails with an authentication error, the environment variable does not match `ota_password`. If it cannot find the rover, check the IP address and flash by USB.

## 13. Check that the rover answers

With the rover powered and the Deck on the same network, click Rover Controls on the Deck and drive (section 7). The Deck talks to the rover over three UDP ports:

| Port | Direction | Content |
|---|---|---|
| 4210 | Deck to rover | Wheel speeds `FL,FR,RL,RR`, each -255 to 255 |
| 4211 | Rover to network | Sensor telemetry: distance and the line sensor (and the wheel encoder counts once Part 4 is done) |
| 4212 | Deck to rover | `HEAD,<angle>`, `BUZZ,<hz>,<ms>` and `RELEASE` |

If the rover loses the Deck's commands, it stops the motors after half a second.

---

# Part 3. The camera board (optional)

The camera board is a second ESP32, a Seeed XIAO ESP32-S3 Sense, with its own firmware in `firmware-cam/`. It is a separate project from the main board, so a problem with the camera cannot affect driving. It has a camera, a microphone and a strip of 7 addressable LEDs, and it carries the two IR sensors. Its full command list is in [firmware-cam/README.md](firmware-cam/README.md).

## 14. What it does

| Feature | How you use it |
|---|---|
| Live video | A web page at `http://192.168.1.27/` and, through the Deck, the topic `camera/image/compressed` for Foxglove |
| Camera on/off | Service `camera/enable`, or D-pad up held 3 seconds. The chip, Wi-Fi and the other features stay on |
| Microphone | Off at start. Level only, or level plus audio to the Deck. Topic `microphone/set_mode`, service `microphone/enable`, or D-pad down held 3 seconds |
| LED strip | Off at start, white when switched on. Colour, brightness and mode from the web page, from ROS topics, or D-pad up held 1 to 3 seconds for on and off |
| IR sensors | Two digital sensors on D0 and D1, published as `camera/ir` once fitted |

## 15. Set up the project

1. On the PC, open the **`firmware-cam`** folder from this repository in Visual Studio Code (not the repository root).
2. Create `include/config.h` from the example and fill it in. It has the same settings as the main board's, plus `use_static_ip`:

```bash
cd firmware-cam/include
cp config.example.h config.h
```

Use a different `ap_ssid` from the main board's, and `mdnsHostname` `esp32cam`. Give the board a fixed address in your router (the project assumes `192.168.1.27`, one above the main board).

3. The first build downloads the NeoPixel library, so it needs internet access.

## 16. Build and flash over USB

Use the board's own USB-C port, which appears on Linux as `/dev/ttyACM0` (on Windows or macOS, change `upload_port` and `monitor_port` in `platformio.ini`).

```bash
pio run -e xiao_s3 -t upload
pio device monitor -e xiao_s3
```

If no port appears, hold the board's BOOT button while you plug in the cable, then release it. On Arch Linux, a "permission denied" means your user needs the `uucp` group, as in section 11.

Expected in the monitor: `Camera ready.`, `Wi-Fi connected.` with an address, and then a telemetry line such as `CAM: 1 | MIC: 0 | LVL: 0.000 | LED: off ffffff 100`. If it says "Home network unavailable", check `sta_ssid` and `sta_password`: the board has started its own access point instead.

## 17. Flash over Wi-Fi (OTA)

Set the password in the terminal you upload from, as in section 12, then:

```bash
pio run -e xiao_s3-ota -t upload
```

`upload_port` in `[env:xiao_s3-ota]` is the board's address (`192.168.1.27`). An update switches the microphone and LEDs off first, and ends the video stream.

## 18. Check it

- Open `http://192.168.1.27/` in a browser: the live picture, with a Lights card underneath (on/off, colour, brightness and mode).
- `curl -s http://192.168.1.27/led` answers with the LED state, for example `{"mode":"off","color":"ffffff","brightness":100}`.
- Camera orientation is set by `CAMERA_VFLIP` and `CAMERA_HMIRROR` at the top of `src/main.cpp`.
- The board serves one video viewer at a time: close the browser tab before the Deck's camera bridge connects.

## 19. On the Deck

Click Rover Controls: the start script launches `rover_camera_bridge`, which connects to the board at `192.168.1.27` (the parameter `camera_host`). It publishes these, and Foxglove can show them (see [foxglove/README.md](foxglove/README.md)):

| Topic or service | Type | What it is |
|---|---|---|
| `camera/image/compressed` | `sensor_msgs/CompressedImage` | The video |
| `camera/enabled` | `std_msgs/Bool` | Whether the camera is on |
| `camera/enable` | service, `std_srvs/SetBool` | Camera on or off |
| `camera/ir` | `std_msgs/UInt8MultiArray` | The IR sensors, one 0 or 1 each (once fitted) |
| `microphone/level` | `std_msgs/Float32` | Loudness, 0.0 to 1.0 |
| `microphone/mode` | `std_msgs/String` | `off`, `level` or `stream` |
| `microphone/set_mode` | `std_msgs/String` | Send `off`, `level` or `stream` |
| `microphone/enable` | service, `std_srvs/SetBool` | `true` streams, `false` stops |
| `microphone/audio` | `std_msgs/Int16MultiArray` | 16 kHz mono audio, while streaming |
| `leds/state` | `std_msgs/String` | Mode, colour and brightness, for example `solid ff0000 50` |
| `leds/color` | `std_msgs/ColorRGBA` | Set all LEDs to one colour |
| `leds/mode` | `std_msgs/String` | `off`, `solid`, `breathe`, `chase` or `rainbow` |
| `leds/brightness` | `std_msgs/UInt8` | 0 to 100, a share of the power cap |
| `leds/pixel` | `std_msgs/UInt8MultiArray` | One LED: index, red, green, blue |

The microphone is a privacy-sensitive feature. It starts off, the Deck switches it off when it starts and stops, and anyone on your network who can send the board a "mic on" command receives the audio.

---

# Part 4. Wheel odometry (planned, not fitted yet)

The rover is prepared for wheel encoders on the two rear wheels, but none are fitted. Until they are, everything below is off.

- **Firmware:** `firmware/src/main.cpp` has stand-in settings for the encoders, off by default (`ENCODERS_FITTED = false`). When the encoders are wired to GPIO 17 (rear left) and GPIO 4 (rear right), set it to `true`, and set the real `WHEEL_DIAMETER_MM` and `ENCODER_COUNTS_PER_WHEEL_REV`. The rover then sends its encoder counts 20 times a second.
- **Deck:** the `rover_odometry` package turns the counts into `odom` and the `odom` to `base_link` transform. It reads its settings from `rover_odometry/config/odometry.yaml` (wheel radius, counts per turn and the distance between the axles) and does nothing until they are set.

To add it on the Deck, link it like the other packages and build it. Add a `start odometry` line to the start script, as described in [Shortcuts/README.md](Shortcuts/README.md), and fill in the yaml:

```bash
cd ~/Robotics/ros2_ws/src && ln -s ../../Rover/rover_odometry rover_odometry
cd ~/Robotics/ros2_ws && pixi run colcon build --symlink-install --packages-select rover_odometry
```

Two rear wheels give the forward speed exactly, but turning on the spot and strafing sideways look alike to them. The node uses the drive commands to tell which one is happening, so the heading will drift. Wheel slip is not seen. This is experimental and untested on the rover.

---

# Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| An icon does nothing | The script path or permissions are wrong | Check the `Exec=` path in the icon, `chmod +x` the script, and check `~/Robotics/logs` |
| Nothing moves | The deadman is not held, or the rover is still in its boot tests | Hold L1, and wait about 19 seconds after the rover connects |
| Motors stutter | Two programs are sending to the rover | Stop the rover controls before using the wheel tools in `~/Robotics/tools` |
| Controls behave strangely | Two `joy_node`s are running | `pkill -f joy_node`, then click Rover Controls again |
| No data in Foxglove | The bridge is not running or the URL is wrong | Click Rover Controls, then connect to `ws://localhost:8765` |
| Upload cannot connect (USB) | Wrong port or permissions | See the notes in section 11 |
| OTA upload fails | Wrong IP, wrong password, or the rover is on its fallback access point | See section 12 |
| `Package 'rover_camera_bridge' not found` in `camera.log` | The package is not linked or built, or its `package.xml` is empty | Repeat step 4, and check `colcon list` shows it as `ros.ament_python` |
| `Camera stream problem: No route to host` | The camera board is off, still starting, or on a different address | Ping `192.168.1.27`, and check the board's address in your router |
| The camera picture is stuck | The camera was switched off (D-pad up, or `camera/enable`) | Switch it on again. The last picture stays in the panel |
| The video does not appear while a browser tab shows it | The board serves one viewer at a time | Close the tab |
| The Deck falls asleep during a test and everything drops out | SteamOS suspends an idle Deck | Turn off suspend in System Settings, Power Management |
| The lights do nothing | No strip is connected, or the data line level is wrong | Check that `leds/state` changes. A 5 V strip may need a level shifter |
