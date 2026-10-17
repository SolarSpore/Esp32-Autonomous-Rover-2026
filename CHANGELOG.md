# Rover Changelog

Changes to the ESP32 mecanum rover: firmware, ROS 2 packages, launch scripts and tools.
Platform-level changes are recorded in the Distributed-Robotics-Platform-2026 repo.

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
