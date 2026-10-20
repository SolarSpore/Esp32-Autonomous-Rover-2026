# Rover Setup Guide

How to install this rover's software and get it driving. It has two parts:

- **Part 1, on the Steam Deck:** ROS 2 (through Pixi), the rover's ROS packages, the desktop shortcuts and Foxglove.
- **Part 2, on a PC:** the ESP32 firmware, with PlatformIO in Visual Studio Code, flashed over USB and then over Wi-Fi (OTA).

The same Pixi, ROS 2 and Foxglove steps, explained in more detail and for building a different robot, are in the Distributed-Robotics-Platform-2026 repository (`docs/install_guide.md`): https://github.com/SolarSpore/Distributed-Robotics-Platform-2026

If you already followed sections 1 to 5 of that guide on this Deck, skip steps 1, 3 and 6 of Part 1 below: they repeat it. Step 3 would overwrite your workspace's `pixi.toml` and `pixi.lock`.

Wiring, print settings and assembly are not covered here. The pin numbers are constants at the top of `firmware/src/main.cpp`, and the 3D-printed parts are in the `STL/` folder (see "3D-printed parts" below).

## What you need

- A Steam Deck in Desktop Mode, using the default `deck` user. The scripts and icons have `/home/deck/...` paths built in. All Deck commands run in Konsole.
- Internet access on the Deck. The first install downloads the whole ROS 2 environment and takes several minutes.
- The rover: an ESP32 (ESP32-WROOM-32 dev board) with its motor drivers, head servo, buzzer and sensors wired as the firmware expects.
- The rover's 3D-printed parts. The STL files are in the `STL/` folder at the root of this repository.
- A PC for Part 2 (any Linux, Windows or macOS machine that can run Visual Studio Code) and a USB cable for the first flash.
- A Wi-Fi network that both the Deck and the ESP32 can join.

## What goes where

```text
~/Robotics/
├── Rover/          this repository (cloned here)
├── ros2_ws/        the Pixi workspace, built from Rover/ros2_ws
├── Shortcuts/      start_rover_controls.sh and stop_rover_controls.sh
├── tools/          wheel test scripts and the controller mapper
└── logs/           one log file per started process
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
cd ~/Robotics/ros2_ws
pixi run colcon build --symlink-install
ls install
```

Expected: `rover_control`, `rover_motor_bridge` and `rover_peripherals_bridge` in the listing. (`rover_description` is a placeholder and is not built.)

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

The shortcuts start and stop everything the rover needs on the Deck: the Foxglove bridge, teleop, `joy_node` and the two bridges to the ESP32. Each script, icon and log file is documented in [Shortcuts/README.md](Shortcuts/README.md), including how to adapt them to another setup. The short version:

- **`RoverControls.desktop`** is the "Rover Controls" icon. Its `Exec=` line is the full path of `start_rover_controls.sh`, and it has a right-click action, "Stop Rover Controls".
- **`StopRoverControls.desktop`** is a separate "Stop Rover Controls" icon.
- **`start_rover_controls.sh`** stops any leftover rover processes, then starts `foxglove_bridge`, teleop (`rover_control`, which also starts `joy_node`), `rover_motor_bridge` and `rover_peripherals_bridge`, all detached, with one log per part in `~/Robotics/logs` (`foxglove.log`, `teleop.log`, `bridge.log`, `peripherals.log`). A lock stops a double-click from starting a second copy on top of a running one. A notification says "Started. Hold L1 to drive."
- **`stop_rover_controls.sh`** stops all of the above. The motor bridge sends a stop command to the rover when it shuts down.

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

Foxglove layouts (dashboards) can be exported as JSON files and imported again. The rover's layouts go in the `foxglove/` folder of this repository, which is described in [foxglove/README.md](foxglove/README.md).

## 7. Drive

1. Power on the rover and wait for it to join Wi-Fi. Both firmware bench tests are enabled by default (a buzzer jingle and a servo sweep), and the rover accepts no commands for about 19 seconds after it connects. Part 2 shows how to turn the tests off.
2. With Steam running, the Deck's controls appear as a virtual Xbox 360 gamepad at `/dev/input/js0`.
3. Click the Rover Controls icon. Wait for the notification.
4. Hold L1 (the deadman) and drive. Right-click the icon and choose Stop Rover Controls to stop.

The controls are defined in `rover_control/config/steam_deck_controls.yaml`:

| Control | Action |
|---|---|
| L1 (button 9) | Deadman: nothing moves unless it is held |
| R1 (button 10) | Turbo |
| Left stick up/down (axis 1) | Forward and back |
| Left stick left/right (axis 0) | Steering |
| X (button 2) | Strafe, with D-pad left/right (axis 6) giving the direction |
| A (button 0), held 3 seconds | Toggle tank mode: the sticks' up/down axes (axes 1 and 3) then drive the left and right sides |
| B (button 1) | Beep |
| Right stick left/right (axis 2) | Move the head |

The rover is expected at `192.168.1.26`: the motor bridge's `robot_ip` parameter defaults to it, and so does the peripherals bridge. The simplest fix for a different network is to give the ESP32 that address in your router's settings (a DHCP reservation), and to use the same address as `upload_port` in Part 2.

---

# Part 2. The firmware, on a PC

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

On Windows PowerShell the variable is set with `$env:OTA_PASSWORD='...'`. If you upload with the PlatformIO toolbar buttons instead of a terminal, start Visual Studio Code from a terminal where the variable is already set (`code .`), because the buttons use Visual Studio Code's own environment.

Put the rover on blocks first. When an update starts, the firmware stops the motors, servo and buzzer; when it finishes the rover reboots. The PC and the rover must be on the same network, and the rover must have joined your Wi-Fi (not its fallback access point).

If an OTA upload fails with an authentication error, the environment variable does not match `ota_password`. If it cannot find the rover, check the IP address and flash by USB.

## 13. Check that the rover answers

With the rover powered and the Deck on the same network, click Rover Controls on the Deck and drive (section 7). The Deck talks to the rover over three UDP ports:

| Port | Direction | Content |
|---|---|---|
| 4210 | Deck to rover | Wheel speeds `FL,FR,RL,RR`, each -255 to 255 |
| 4211 | Rover to network | Sensor telemetry (distance, line sensors, IR) |
| 4212 | Deck to rover | `HEAD,<angle>`, `BUZZ,<hz>,<ms>` and `RELEASE` |

If the rover loses the Deck's commands, it stops the motors after half a second.

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
