# Foxglove

Foxglove dashboard layouts for the rover go here, as exported JSON files. None have been added yet.

## Setup

Installing Foxglove Studio and connecting it to the rover are covered in section 6 of
`../Setup-Guide.md`. The Rover Controls icon starts `foxglove_bridge`, which serves the rover's ROS
topics at `ws://localhost:8765`.

## Panels that work with the rover's topics

Add these panels in Foxglove Studio, then choose the topic or path in each panel's settings. If a topic
is missing from the list, close the connection and open it again.

| Panel | Topic or path | What it shows |
|---|---|---|
| Image | `/camera/image/compressed` | Live video from the camera board |
| Indicator | `/camera/enabled.data` | Whether the camera is on |
| Gauge or Plot | `/ultrasonic/range.range` | Distance in metres (maximum 2) |
| Plot | `/microphone/level.data` | Microphone loudness, 0 to 1 |
| Indicator | `/microphone/mode.data` | `off`, `level` or `stream` |
| Raw Messages | `/leds/state` | LED mode, colour and brightness |
| Indicator | `/drive_mode.data` | `arcade`, `tank` or `trigger` |
| Plot | `/cmd_vel.linear.x`, `/cmd_vel.angular.z` | What the controller is asking the wheels to do |
| Raw Messages | `/rover/telemetry_raw` | The rover's whole telemetry line |
| Publish | `/leds/color`, `/leds/brightness`, `/leds/mode`, `/microphone/set_mode` | Control the lights and the microphone (see below) |

The LED strip also has its own control page: open `http://192.168.1.27/` in a browser for a colour
picker, a brightness slider and a mode menu next to the live video.

Publish panels send a message you type as JSON. For example, on `/leds/color` send
`{"r": 1.0, "g": 0.5, "b": 0.0, "a": 1.0}`, on `/leds/brightness` send `{"data": 60}`, and on
`/leds/mode` send `{"data": "rainbow"}`. Which Publish panel features work through `foxglove_bridge`
has not been tested on this setup.

The microphone's audio stream (`/microphone/audio`) shows as a list of numbers: Foxglove is used here
for the level, not for listening.

## Using a layout

Foxglove can export a layout as a JSON file and import it again. To use a layout from this folder,
import the file in Foxglove Studio. To add one, export it from Foxglove Studio and put the file in
this folder.
