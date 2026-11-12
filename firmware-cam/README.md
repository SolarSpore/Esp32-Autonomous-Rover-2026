# Camera board firmware

PlatformIO project for the rover's second ESP32, a Seeed XIAO ESP32-S3 Sense. It serves video, listens
with the microphone, drives a strip of 7 addressable LEDs and reads two IR sensors, and it takes simple
text commands over UDP. It is separate from the main rover firmware in `../firmware/`. Installing and
flashing it is described in Part 3 of `../Setup-Guide.md`.

## Files

| File | What it is |
|---|---|
| `platformio.ini` | Two environments: `xiao_s3` (USB, `/dev/ttyACM0`) and `xiao_s3-ota` (Wi-Fi, `192.168.1.27`). The OTA password comes from the `OTA_PASSWORD` environment variable |
| `include/config.example.h` | Copy it to `include/config.h` and fill it in. `config.h` holds your Wi-Fi details and the OTA password and is never committed |
| `src/main.cpp` | The firmware. The settings are constants at the top of each section |

## Network

| Port | Direction | What it is |
|---|---|---|
| 80 (HTTP) | Board to you | `/` the page with the video and the light controls, `/capture` one JPEG frame, `/led` the LED state as JSON |
| 81 (HTTP) | Board to you | `/stream`, the MJPEG video. One viewer at a time |
| 4213 (UDP) | Board to network | Telemetry, one text line (below) |
| 4214 (UDP) | To the board | Commands (below) |
| 4215 (UDP) | Board to the computer that sent `MIC,ON` | Audio packets |
| 3232 | To the board | OTA updates |

The board tries your Wi-Fi for 8 seconds at start, then starts its own access point (`ap_ssid`). Its name
on the network is `esp32cam.local`.

### Telemetry line

Broadcast five times a second and at once after any change:

```text
IR: 10 | CAM: 1 | MIC: 0 | LVL: 0.000 | LED: solid ff0000 50
```

| Field | Meaning |
|---|---|
| `IR` | One digit per IR sensor, `1` when it sees something. Left out until the sensors are fitted |
| `CAM` | `1` while the camera is running |
| `MIC` | `0` off, `1` level only, `2` level and audio stream |
| `LVL` | Microphone loudness, 0.0 to 1.0 |
| `LED` | Mode, colour (hex RRGGBB) and brightness percent |

### Commands

One text line per UDP packet, to port 4214:

| Command | Effect |
|---|---|
| `CAM,ON` or `CAM,OFF` | Start the camera, or release the camera driver and stop its clock. The chip, Wi-Fi, OTA, the lights and the microphone keep running. `/capture` and `/stream` answer 503 while it is off |
| `MIC,ON` | Stream audio to the sender of the command, and report the level |
| `MIC,LEVEL` | Report the level only |
| `MIC,OFF` | Stop the microphone (the default at start) |
| `LED,OFF` | Lights off (the default at start) |
| `LED,COLOR,r,g,b` | All LEDs one colour, each value 0 to 255 |
| `LED,MODE,name` or `LED,MODE,name,r,g,b` | `solid`, `breathe`, `chase` or `rainbow`, optionally with a colour |
| `LED,PIXEL,i,r,g,b` | One LED. The others keep their colour |
| `LED,BRIGHT,0..100` | Share of the maximum brightness |

### Audio packets

Each UDP packet is a 4 byte sequence number (little endian) followed by 16 bit mono samples at 16 kHz,
20 ms per packet.

## Settings to know about

| Constant | Default | What it does |
|---|---|---|
| `CAMERA_FRAME_SIZE`, `CAMERA_JPEG_QUALITY` | VGA, 12 | Picture size and compression |
| `STREAM_MIN_FRAME_MS` | `66` | Frame gap: about 15 frames a second. `33` is about 30 |
| `CAMERA_XCLK_HZ` | `20000000` | Camera clock. A lower value runs cooler |
| `CAMERA_VFLIP`, `CAMERA_HMIRROR` | `true`, `false` | Picture orientation for how the board is mounted |
| `CAMERA_START_ENABLED` | `true` | Start with the camera on |
| `LED_COUNT` | `7` | Number of LEDs. Everything follows it |
| `LED_MAX_BRIGHTNESS_PERCENT` | `25` | Power cap: the brightness slider is a share of this |
| `MIC_GAIN` | `4` | Raise it if the audio is too quiet |
| `IR_SENSORS_FITTED` | `false` | Set to `true` once the two IR sensors are wired to D0 and D1 |

## Things to know

- **Power:** the board with its camera can draw 500 to 700 mA at peak, plus the LEDs. A weak supply
  gives a black picture or a failed camera start.
- **LED pin:** the strip's data line is on D2 (GPIO 3), which is a strapping pin and the SD card chip
  select. It works after boot. If the board stops starting with the strip connected, unplug the data
  wire. The pin is 3.3 V logic: a strip run from 5 V may need a level shifter.
- **Privacy:** the microphone starts off. Anyone on your network who can send the board `MIC,ON`
  receives the audio.
- **Heat:** the board gets warm streaming video. A smaller picture, a lower frame rate or a lower camera
  clock helps.
