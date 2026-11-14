# Rover Changelog

Changes to the ESP32 mecanum rover: firmware, ROS 2 packages, launch scripts and tools.
Platform-level changes are recorded in the Distributed-Robotics-Platform-2026 repo.

## 2026-11-14

### Documentation

- Rewrote the README around the Steam Deck as the operator station: driving with its gamepad and watching the camera and sensors in Foxglove.
- Added an overview diagram and a ROS 2 node diagram, both drawn top to bottom so they stay readable on GitHub.
- Added the Driving with the Steam Deck controls table and the Seeing the Rover with Foxglove section.
- Added sections for the camera board and wheel odometry, and updated the repository layout, components and UDP interface lists.

## 2026-11-11

### Added

- `rover_camera_bridge/`: the Deck-side bridge for the camera board. Video as `camera/image/compressed`, camera on/off, microphone level and audio, LED strip control, IR sensors, and gamepad toggles (D-pad up and down).
- `rover_odometry/`: wheel odometry from encoders on the two rear wheels (`odom` and the odom to base_link transform). Experimental: no encoders are fitted yet.
- `firmware-cam/`: PlatformIO project for the camera board (XIAO ESP32-S3 Sense): video and a web page, microphone, 7 addressable LEDs with a web control page, IR sensors, Wi-Fi with a fallback access point, and OTA updates.

### Changed

- `rover_control`: a third driving mode, proportional trigger drive (right trigger gas, left trigger brake and reverse, no deadman). Toggled by holding L1 and both triggers for 3 seconds. Arcade stays the default.
- `firmware/src/main.cpp`: the two IR sensors moved to the camera board; stand-in support for wheel encoders on the rear wheels, off until `ENCODERS_FITTED` is true.
- `Shortcuts/`: the start and stop scripts also start and stop the camera bridge.

### Documentation

- `Setup-Guide.md`: a new controls table, Part 3 (the camera board) and Part 4 (wheel odometry), updated ports and troubleshooting.
- `Shortcuts/README.md` and `foxglove/README.md` updated. Added `firmware-cam/README.md`.

## 2026-10-19

### Documentation

- Added `tools/README.md`: how to run the wheel tests and the controller wizard.
- Added a "Using the tools" section to `Setup-Guide.md`.

## 2026-10-18

### Setup and layout

- Added `Setup-Guide.md`, a quick start for the Steam Deck and for flashing the firmware with PlatformIO.
- Rewrote `README.md`: overview, repository layout, components, and the rover's ROS 2 nodes, hardware and UDP protocol.
- Added `ros2_ws/` (Pixi workspace files) and `foxglove/` (dashboard layouts, none yet).
- Renamed `scripts/` to `Shortcuts/` and moved the tools to `tools/`, mirroring the folders on the Deck.

## 2026-10-17

### Repository

- Added `scripts/README.md`, moved here from the platform repository. The platform's older
  variants of the start/stop scripts stay in that repository's git history.
- Moved `rover_control/joy_mapper.py` to `scripts/tools/`. It is a helper, not part of the ROS package.
- Added `docs/` for rover-specific documentation.

## 2026-10-12

### Repository

- Created this repository from the working rover files: PlatformIO firmware for the ESP32,
  the ROS 2 packages (`rover_control`, `rover_motor_bridge`, `rover_peripherals_bridge`,
  `rover_description`), and the Steam Deck scripts, desktop launchers and bring-up tools.
- Firmware `config.h` (Wi-Fi and OTA credentials) is git-ignored. `config.example.h` has placeholders.
- The OTA password in `platformio.ini` is replaced with `${sysenv.OTA_PASSWORD}`. Set that
  environment variable before using the OTA upload environment.

### Notes

- `rover_description` is an unfinished placeholder and is not built. Its `wheel_separation`
  (0.20) differs from the 0.30 used by the control node and motor bridge.
