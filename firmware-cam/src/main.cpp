/**********************************************************************
  XIAO ESP32-S3 Sense camera board (second ESP32 on the rover)

  Hardware:
    - Seeed XIAO ESP32-S3 Sense: camera, PDM microphone, 8 MB PSRAM
    - 7 addressable LEDs (WS2812 type) on D2 (GPIO 3)
    - IR digital sensors on D0 and D1 (GPIO 1 and 2), once fitted

  Web server:
    http://<ip>/            page with the live video and the light controls
    http://<ip>/capture     one JPEG frame
    http://<ip>:81/stream   MJPEG stream (one viewer at a time)
    http://<ip>/led         LED state as JSON. ?color=RRGGBB&brightness=0..100
                            &mode=off|solid|breathe|chase|rainbow changes it.
                            The lights start off, white, at full brightness.

  Telemetry (port 4213, camera board -> network, one text line):
    "IR: 10 | CAM: 1 | MIC: 0 | LVL: 0.000 | LED: solid ff0000 50"
    IR   one bit per sensor, 1 = active (left out until IR_SENSORS_FITTED)
    CAM  1 while the camera is running
    MIC  0 off, 1 level only, 2 level + audio stream
    LVL  microphone loudness, 0.0 to 1.0
    LED  mode, colour (hex RRGGBB) and brightness percent
    Broadcast every IR_REPORT_INTERVAL_MS and at once after a change.

  Commands (port 4214, Deck -> camera board, one text line each):
    CAM,ON | CAM,OFF
        Starts the camera, or releases the camera driver and stops its
        clock. /capture and /stream answer 503 while it is off.
    MIC,ON | MIC,LEVEL | MIC,OFF
        ON streams audio to the sender of the command and reports the
        level, LEVEL reports the level only, OFF stops the microphone.
        The microphone starts off.
    LED,OFF
    LED,COLOR,r,g,b            all LEDs one colour (0 to 255 each)
    LED,MODE,name[,r,g,b]      solid, breathe, chase or rainbow
    LED,PIXEL,i,r,g,b          one LED (0 to 6); the rest keep their colour
    LED,BRIGHT,0..100          share of the maximum brightness

  Audio (port 4215, camera board -> the computer that sent MIC,ON):
    UDP packets: a 4 byte sequence number (little endian) followed by 16 bit
    mono samples at 16 kHz, 20 ms per packet.

  The chip, Wi-Fi, OTA, the web server, the LEDs and the sensors keep
  running whatever the camera or microphone is doing.

  Network:
    1. Try the configured Wi-Fi network.
    2. If unavailable, start a fallback access point.
    3. mDNS hostname: esp32cam.local
    4. Wi-Fi modem sleep is disabled in station mode for steady
       streaming.

  OTA:
    - ArduinoOTA, hostname and password from config.h
    - The stream, the microphone and the LEDs stop when an update starts
    - USB-C stays the recovery method

  If the camera fails to start, Wi-Fi and OTA still run, so the
  board can be fixed and reflashed without a cable.
**********************************************************************/

#include <Arduino.h>
#include "config.h"

#include <WiFi.h>
#include <ESPmDNS.h>
#include <ArduinoOTA.h>
#include <WiFiUdp.h>
#include <Adafruit_NeoPixel.h>

#include "esp_camera.h"
#include "esp_http_server.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "driver/i2s.h"

// ============================================================
// General constants
// ============================================================

const unsigned long SERIAL_BAUD_RATE     = 115200;
const unsigned long SERIAL_WAIT_MS       = 2000;   // wait for USB serial
const unsigned long BOOT_SETTLE_DELAY_MS = 500;

// ============================================================
// Network constants
// ============================================================

const unsigned long STA_TIMEOUT_MS        = 8000;
const unsigned long WIFI_POLL_INTERVAL_MS = 300;
const unsigned long AP_START_DELAY_MS     = 100;

const uint16_t PAGE_PORT   = 80;
const uint16_t STREAM_PORT = 81;
const uint16_t OTA_PORT    = 3232;   // ArduinoOTA default

// ============================================================
// Camera pins (XIAO ESP32-S3 Sense, fixed on the board)
// ============================================================

const int CAM_PIN_PWDN  = -1;
const int CAM_PIN_RESET = -1;
const int CAM_PIN_XCLK  = 10;
const int CAM_PIN_SDA   = 40;
const int CAM_PIN_SCL   = 39;
const int CAM_PIN_VSYNC = 38;
const int CAM_PIN_HREF  = 47;
const int CAM_PIN_PCLK  = 13;

const int CAM_PIN_D0 = 15;
const int CAM_PIN_D1 = 17;
const int CAM_PIN_D2 = 18;
const int CAM_PIN_D3 = 16;
const int CAM_PIN_D4 = 14;
const int CAM_PIN_D5 = 12;
const int CAM_PIN_D6 = 11;
const int CAM_PIN_D7 = 48;

// ============================================================
// Camera settings
// ============================================================

// Start small. A bigger picture or a higher frame rate uses more
// Wi-Fi airtime, which the rover's drive commands also share.
const framesize_t CAMERA_FRAME_SIZE   = FRAMESIZE_VGA;   // 640x480
const int         CAMERA_JPEG_QUALITY = 12;   // 10 best .. 63 smallest
const int         CAMERA_FRAME_BUFFERS = 2;
const int         CAMERA_XCLK_HZ      = 20000000;

// Flip these if the picture is upside down or mirrored once the
// board is mounted.
const bool CAMERA_VFLIP   = true;
const bool CAMERA_HMIRROR = false;

// Frame rate cap for the stream: 66 ms is about 15 frames/second.
const unsigned long STREAM_MIN_FRAME_MS = 66;

// Set to false to boot with the camera off (the Deck can switch it on).
const bool CAMERA_START_ENABLED = true;

// How long a frame request, or a camera switch, waits for the camera.
const unsigned long CAMERA_LOCK_WAIT_MS   = 500;
const unsigned long CAMERA_SWITCH_WAIT_MS = 6000;

// ============================================================
// Shared state
// ============================================================

volatile bool otaActive    = false;   // set while an OTA update runs
volatile bool telemetryDue = false;   // send a telemetry line at once

// True while a network is available to send on.
bool networkIsUp() {
  return (WiFi.getMode() == WIFI_AP) || (WiFi.status() == WL_CONNECTED);
}

// Parses a whole number. Returns false if the text is not a clean integer.
bool parseInt(const char* text, long& value) {
  if (text == nullptr) {
    return false;
  }

  char* end = nullptr;
  value = strtol(text, &end, 10);

  return end != text && *end == '\0';
}

// ============================================================
// LED strip (7 addressable LEDs on D2 = GPIO 3)
// ============================================================

// GPIO 3 is a strapping pin (it chooses the JTAG source at reset) and the
// SD card chip select. The LED data line only drives it after boot and the
// SD card is not used, so it works here. If the board ever fails to start
// with the strip connected, unplug the data wire and report it.
//
// Power: each LED draws up to 60 mA at full white, and the board's supply is
// already shared with the camera. LED_MAX_BRIGHTNESS_PERCENT caps the total
// (7 LEDs at 25% is about 105 mA at worst). Raise it only if the supply allows.
// The LED data line should be 5 V logic for a strip run from 5 V. If the
// strip flickers or ignores commands, add a level shifter.
const int LED_PIN   = 3;
const int LED_COUNT = 7;

const int LED_MAX_BRIGHTNESS_PERCENT     = 25;
const int LED_DEFAULT_BRIGHTNESS_PERCENT = 100;   // share of the maximum

const unsigned long LED_FRAME_MS          = 30;
const unsigned long LED_BREATHE_PERIOD_MS = 3000;
const unsigned long LED_CHASE_STEP_MS     = 100;

enum LedMode {
  LED_OFF = 0,
  LED_SOLID,
  LED_BREATHE,
  LED_CHASE,
  LED_RAINBOW,
  LED_CUSTOM
};

const char* const LED_MODE_NAMES[] = {
  "off", "solid", "breathe", "chase", "rainbow", "custom"
};

Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

LedMode       ledMode              = LED_OFF;
uint32_t      ledColor             = 0xFFFFFF;   // 0xRRGGBB
int           ledBrightnessPercent = LED_DEFAULT_BRIGHTNESS_PERCENT;
uint32_t      ledCustom[LED_COUNT] = {};
bool          ledDirty             = true;
unsigned long ledLastFrameMs       = 0;

void setupLeds() {
  strip.begin();
  strip.clear();
  strip.show();
  ledDirty = true;
}

void ledSetMode(LedMode mode) {
  ledMode      = mode;
  ledDirty     = true;
  telemetryDue = true;
}

// A mode from its name. Returns false if the name is not one of them.
bool parseLedMode(const char* name, LedMode& mode) {
  if (strcasecmp(name, "off") == 0)          mode = LED_OFF;
  else if (strcasecmp(name, "solid") == 0)   mode = LED_SOLID;
  else if (strcasecmp(name, "breathe") == 0) mode = LED_BREATHE;
  else if (strcasecmp(name, "chase") == 0)   mode = LED_CHASE;
  else if (strcasecmp(name, "rainbow") == 0) mode = LED_RAINBOW;
  else return false;

  return true;
}

// Switches the strip off at once (used when an OTA update starts).
void ledsOff() {
  ledMode      = LED_OFF;
  ledDirty     = false;
  telemetryDue = true;
  strip.clear();
  strip.show();
}

uint32_t ledPackColor(long r, long g, long b) {
  r = constrain(r, 0, 255);
  g = constrain(g, 0, 255);
  b = constrain(b, 0, 255);
  return ((uint32_t)r << 16) | ((uint32_t)g << 8) | (uint32_t)b;
}

// One colour scaled by 0.0 to 1.0.
uint32_t ledScaleColor(uint32_t color, float factor) {
  float r = ((color >> 16) & 0xFF) * factor;
  float g = ((color >> 8) & 0xFF) * factor;
  float b = (color & 0xFF) * factor;
  return ledPackColor((long)r, (long)g, (long)b);
}

// Draws the strip. Called every pass of the main loop; it only does work
// when something changed or an animated mode is due for its next frame.
void ledUpdate() {
  unsigned long now = millis();

  bool animated =
    (ledMode == LED_BREATHE) || (ledMode == LED_CHASE) || (ledMode == LED_RAINBOW);

  if (!ledDirty && !(animated && now - ledLastFrameMs >= LED_FRAME_MS)) {
    return;
  }

  ledLastFrameMs = now;
  ledDirty       = false;

  int brightness =
    (255 * LED_MAX_BRIGHTNESS_PERCENT / 100) * ledBrightnessPercent / 100;
  strip.setBrightness((uint8_t)brightness);

  switch (ledMode) {

    case LED_OFF:
      strip.clear();
      break;

    case LED_SOLID:
      strip.fill(ledColor);
      break;

    case LED_BREATHE: {
      float phase  = (float)(now % LED_BREATHE_PERIOD_MS) / LED_BREATHE_PERIOD_MS;
      float factor = 0.1f + 0.9f * (0.5f - 0.5f * cosf(2.0f * PI * phase));
      strip.fill(ledScaleColor(ledColor, factor));
      break;
    }

    case LED_CHASE: {
      int position = (now / LED_CHASE_STEP_MS) % LED_COUNT;
      int tail     = (position + LED_COUNT - 1) % LED_COUNT;
      strip.clear();
      strip.setPixelColor(tail, ledScaleColor(ledColor, 0.2f));
      strip.setPixelColor(position, ledColor);
      break;
    }

    case LED_RAINBOW: {
      uint32_t base = (now * 12UL) % 65536UL;
      for (int i = 0; i < LED_COUNT; i++) {
        uint16_t hue = (uint16_t)((base + (uint32_t)i * 65536UL / LED_COUNT) % 65536UL);
        strip.setPixelColor(i, strip.gamma32(strip.ColorHSV(hue)));
      }
      break;
    }

    case LED_CUSTOM:
      for (int i = 0; i < LED_COUNT; i++) {
        strip.setPixelColor(i, ledCustom[i]);
      }
      break;
  }

  strip.show();
}

// Handles the "LED,..." commands. tokens[0] is "LED".
void handleLedCommand(char** tokens, int count) {
  const char* sub = tokens[1];

  if (strcasecmp(sub, "OFF") == 0) {
    ledSetMode(LED_OFF);
    return;
  }

  long a = 0, b = 0, c = 0, d = 0;

  if (strcasecmp(sub, "COLOR") == 0 && count >= 5 &&
      parseInt(tokens[2], a) && parseInt(tokens[3], b) && parseInt(tokens[4], c)) {
    ledColor = ledPackColor(a, b, c);
    ledSetMode(LED_SOLID);
    return;
  }

  if (strcasecmp(sub, "MODE") == 0 && count >= 3) {
    LedMode mode = LED_OFF;
    bool known = true;

    if (strcasecmp(tokens[2], "solid") == 0)        mode = LED_SOLID;
    else if (strcasecmp(tokens[2], "breathe") == 0) mode = LED_BREATHE;
    else if (strcasecmp(tokens[2], "chase") == 0)   mode = LED_CHASE;
    else if (strcasecmp(tokens[2], "rainbow") == 0) mode = LED_RAINBOW;
    else if (strcasecmp(tokens[2], "off") == 0)     mode = LED_OFF;
    else known = false;

    if (known) {
      if (count >= 6 &&
          parseInt(tokens[3], a) && parseInt(tokens[4], b) && parseInt(tokens[5], c)) {
        ledColor = ledPackColor(a, b, c);
      }
      ledSetMode(mode);
      return;
    }
  }

  if (strcasecmp(sub, "PIXEL") == 0 && count >= 6 &&
      parseInt(tokens[2], a) && parseInt(tokens[3], b) &&
      parseInt(tokens[4], c) && parseInt(tokens[5], d) &&
      a >= 0 && a < LED_COUNT) {
    if (ledMode != LED_CUSTOM) {
      for (int i = 0; i < LED_COUNT; i++) {
        ledCustom[i] = 0;
      }
    }
    ledCustom[a] = ledPackColor(b, c, d);
    ledSetMode(LED_CUSTOM);
    return;
  }

  if (strcasecmp(sub, "BRIGHT") == 0 && count >= 3 && parseInt(tokens[2], a)) {
    ledBrightnessPercent = constrain(a, 0, 100);
    ledDirty     = true;
    telemetryDue = true;
    return;
  }

  Serial.println("Invalid LED command.");
}

// ============================================================
// Microphone (PDM, on the Sense expansion board)
// ============================================================

// The microphone clock is on GPIO 42 and its data on GPIO 41. It starts
// off: the Deck switches it on. Anyone on the network who can send a
// MIC,ON command receives the audio, so it is off until asked.
const i2s_port_t MIC_I2S_PORT = I2S_NUM_0;
const int        MIC_CLK_PIN  = 42;
const int        MIC_DATA_PIN = 41;

const int MIC_SAMPLE_RATE   = 16000;
const int MIC_CHUNK_SAMPLES = 320;    // 20 ms
const int MIC_GAIN          = 4;      // the raw PDM signal is quiet

const unsigned int  AUDIO_PORT           = 4215;
const unsigned long MIC_SWITCH_WAIT_MS   = 1000;

enum MicMode {
  MIC_OFF    = 0,
  MIC_LEVEL  = 1,
  MIC_STREAM = 2
};

volatile int   micMode  = MIC_OFF;
volatile float micLevel = 0.0f;   // 0.0 to 1.0

SemaphoreHandle_t micMutex = nullptr;   // one user of the I2S driver at a time
bool              micDriverInstalled = false;
IPAddress         audioTarget;
uint32_t          audioSequence = 0;
WiFiUDP           audioUdp;

bool installMicDriver() {
  i2s_config_t config = {};
  config.mode                 = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_PDM);
  config.sample_rate          = MIC_SAMPLE_RATE;
  config.bits_per_sample      = I2S_BITS_PER_SAMPLE_16BIT;
  config.channel_format       = I2S_CHANNEL_FMT_ONLY_LEFT;
  config.communication_format = I2S_COMM_FORMAT_STAND_I2S;
  config.intr_alloc_flags     = ESP_INTR_FLAG_LEVEL1;
  config.dma_buf_count        = 8;
  config.dma_buf_len          = MIC_CHUNK_SAMPLES;
  config.use_apll             = false;

  i2s_pin_config_t pins = {};
  pins.mck_io_num   = I2S_PIN_NO_CHANGE;
  pins.bck_io_num   = I2S_PIN_NO_CHANGE;
  pins.ws_io_num    = MIC_CLK_PIN;        // the PDM clock
  pins.data_out_num = I2S_PIN_NO_CHANGE;
  pins.data_in_num  = MIC_DATA_PIN;

  if (i2s_driver_install(MIC_I2S_PORT, &config, 0, nullptr) != ESP_OK) {
    Serial.println("Microphone driver failed to install.");
    return false;
  }

  if (i2s_set_pin(MIC_I2S_PORT, &pins) != ESP_OK) {
    Serial.println("Microphone pins failed to set.");
    i2s_driver_uninstall(MIC_I2S_PORT);
    return false;
  }

  return true;
}

// Switches the microphone. For MIC_STREAM, target is the address that
// receives the audio. Returns true if the microphone is in the mode asked for.
bool setMicMode(int mode, IPAddress target) {
  if (xSemaphoreTake(micMutex, pdMS_TO_TICKS(MIC_SWITCH_WAIT_MS)) != pdTRUE) {
    Serial.println("Microphone busy: switch skipped.");
    return false;
  }

  if (mode != MIC_OFF && !micDriverInstalled) {
    micDriverInstalled = installMicDriver();
    if (!micDriverInstalled) {
      mode = MIC_OFF;
    }
  }

  if (mode == MIC_OFF && micDriverInstalled) {
    i2s_driver_uninstall(MIC_I2S_PORT);
    micDriverInstalled = false;
  }

  if (mode == MIC_OFF) {
    micLevel = 0.0f;
  }

  if (mode == MIC_STREAM) {
    audioTarget   = target;
    audioSequence = 0;
  }

  micMode = mode;

  xSemaphoreGive(micMutex);

  telemetryDue = true;

  return true;
}

// Reads the microphone in 20 ms chunks, keeps the level up to date, and
// sends the audio while streaming.
void audioTask(void* parameter) {
  int16_t samples[MIC_CHUNK_SAMPLES];
  uint8_t packet[4 + MIC_CHUNK_SAMPLES * 2];
  float   dcOffset = 0.0f;

  for (;;) {
    if (micMode == MIC_OFF) {
      vTaskDelay(pdMS_TO_TICKS(50));
      continue;
    }

    if (xSemaphoreTake(micMutex, pdMS_TO_TICKS(50)) != pdTRUE) {
      continue;
    }

    size_t bytesRead = 0;
    if (micMode != MIC_OFF && micDriverInstalled) {
      i2s_read(MIC_I2S_PORT, samples, sizeof(samples), &bytesRead, pdMS_TO_TICKS(100));
    }
    int       mode   = micMode;
    IPAddress target = audioTarget;

    xSemaphoreGive(micMutex);

    int count = bytesRead / sizeof(int16_t);
    if (count <= 0) {
      continue;
    }

    // Remove the slow DC offset, apply the gain, and measure the level.
    double sumSquares = 0.0;
    for (int i = 0; i < count; i++) {
      dcOffset += ((float)samples[i] - dcOffset) * 0.001f;
      float value = ((float)samples[i] - dcOffset) * MIC_GAIN;
      if (value > 32767.0f)  value = 32767.0f;
      if (value < -32768.0f) value = -32768.0f;
      samples[i] = (int16_t)value;
      sumSquares += (double)value * value;
    }

    float level = sqrtf((float)(sumSquares / count)) / 32768.0f;
    micLevel = (level > 1.0f) ? 1.0f : level;

    if (mode == MIC_STREAM && !otaActive && networkIsUp()) {
      packet[0] = audioSequence & 0xFF;
      packet[1] = (audioSequence >> 8) & 0xFF;
      packet[2] = (audioSequence >> 16) & 0xFF;
      packet[3] = (audioSequence >> 24) & 0xFF;
      memcpy(packet + 4, samples, count * sizeof(int16_t));
      audioSequence++;

      audioUdp.beginPacket(target, AUDIO_PORT);
      audioUdp.write(packet, 4 + count * sizeof(int16_t));
      audioUdp.endPacket();
    }
  }
}

void setupMic() {
  micMutex = xSemaphoreCreateMutex();

  xTaskCreatePinnedToCore(audioTask, "audio", 6144, nullptr, 1, nullptr, 1);
}

// Handles "MIC,ON", "MIC,LEVEL" and "MIC,OFF". sender is where the command
// came from; ON streams the audio there.
void handleMicCommand(const char* arg, IPAddress sender) {
  if (strcasecmp(arg, "ON") == 0) {
    setMicMode(MIC_STREAM, sender);
  }
  else if (strcasecmp(arg, "LEVEL") == 0) {
    setMicMode(MIC_LEVEL, IPAddress());
  }
  else if (strcasecmp(arg, "OFF") == 0) {
    setMicMode(MIC_OFF, IPAddress());
  }
  else {
    Serial.println("Invalid microphone command.");
  }
}

// ============================================================
// IR sensors and telemetry
// ============================================================

// Digital IR modules on header pins D0 and D1 (GPIO 1 and 2).
// D2 (GPIO 3) drives the LED strip. Spare header pins for more
// sensors: D3 = GPIO 4, D4 = GPIO 5, D5 = GPIO 6. Avoid D8 to D10
// (SD card SPI).
// The pins are 3.3 V only: power the modules from the 3V3 pin, or
// check that their output never goes above 3.3 V.
const int IR_SENSOR_COUNT = 2;
const int IR_PINS[IR_SENSOR_COUNT] = {1, 2};

// No sensors are wired yet. Floating inputs would report noise, so the
// pins are left alone and no IR field is sent. Set this to true once
// the sensors are connected.
const bool IR_SENSORS_FITTED = false;

// Most of these modules output LOW when they see a reflective
// surface. Flip this if the readings are inverted.
const int IR_ACTIVE_LEVEL = LOW;

const unsigned int  TELEMETRY_PORT        = 4213;
const unsigned long IR_REPORT_INTERVAL_MS = 200;
const size_t        IR_REPORT_BUFFER_SIZE = 128;

const unsigned int  CMD_PORT        = 4214;   // commands: camera, microphone, LEDs
const size_t        CMD_BUFFER_SIZE = 64;
const int           CMD_MAX_TOKENS  = 8;

const IPAddress TELEMETRY_BROADCAST_IP(255, 255, 255, 255);

WiFiUDP telemetryUdp;
WiFiUDP cmdUdp;

unsigned long lastIrReportMs = 0;
char lastIrReport[IR_REPORT_BUFFER_SIZE] = "";
char cmdBuffer[CMD_BUFFER_SIZE];

// True while the camera driver is running (read by the web handlers).
volatile bool cameraReady = false;

void setupIrSensors() {
  if (!IR_SENSORS_FITTED) {
    return;
  }

  // The modules have their own pull-ups, so plain INPUT is used.
  for (int i = 0; i < IR_SENSOR_COUNT; i++) {
    pinMode(IR_PINS[i], INPUT);
  }
}

// Returns true if the IR sensor at this index reads "active".
bool irActive(int index) {
  return digitalRead(IR_PINS[index]) == IR_ACTIVE_LEVEL;
}

// Broadcasts one line of text. Skipped if there is no network.
void sendTelemetry(const char* line) {
  if (!networkIsUp()) {
    return;
  }

  telemetryUdp.beginPacket(TELEMETRY_BROADCAST_IP, TELEMETRY_PORT);
  telemetryUdp.print(line);
  telemetryUdp.endPacket();
}

// Sends one telemetry line, for example
// "IR: 10 | CAM: 1 | MIC: 0 | LVL: 0.000 | LED: solid ff0000 50".
// The IR field is left out until IR_SENSORS_FITTED. The line is printed to
// Serial only when something other than the microphone level changes.
void reportTelemetry() {
  char ir[IR_SENSOR_COUNT + 16];
  ir[0] = '\0';

  if (IR_SENSORS_FITTED) {
    char bits[IR_SENSOR_COUNT + 1];

    for (int i = 0; i < IR_SENSOR_COUNT; i++) {
      bits[i] = irActive(i) ? '1' : '0';
    }
    bits[IR_SENSOR_COUNT] = '\0';

    snprintf(ir, sizeof(ir), "IR: %s | ", bits);
  }

  char led[48];
  snprintf(
    led, sizeof(led), "LED: %s %06lx %d",
    LED_MODE_NAMES[ledMode], (unsigned long)ledColor, ledBrightnessPercent
  );

  char state[IR_REPORT_BUFFER_SIZE];
  snprintf(state, sizeof(state), "%sCAM: %d | MIC: %d", ir, cameraReady ? 1 : 0, (int)micMode);

  char report[IR_REPORT_BUFFER_SIZE];
  snprintf(report, sizeof(report), "%s | LVL: %.3f | %s", state, (double)micLevel, led);

  char key[IR_REPORT_BUFFER_SIZE];
  snprintf(key, sizeof(key), "%s | %s", state, led);

  if (strcmp(key, lastIrReport) != 0) {
    Serial.println(report);
    snprintf(lastIrReport, sizeof(lastIrReport), "%s", key);
  }

  sendTelemetry(report);
}

// ============================================================
// State
// ============================================================

SemaphoreHandle_t cameraMutex = nullptr;   // one user of the camera at a time

httpd_handle_t pageServer   = nullptr;
httpd_handle_t streamServer = nullptr;

// ============================================================
// Web page
// ============================================================

const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Rover camera</title>
<style>
body { margin: 0; background: #111; color: #eee; font-family: sans-serif; }
#wrap { max-width: 720px; margin: 0 auto; padding: 8px; }
img { width: 100%; height: auto; background: #000; display: block; }
.card { margin-top: 12px; padding: 12px; background: #1c1c1c; border-radius: 8px; }
.row { display: flex; align-items: center; gap: 12px; margin: 10px 0; }
.row label { width: 90px; }
input[type=range] { flex: 1; }
input[type=color] { width: 64px; height: 36px; border: 0; padding: 0; background: none; }
select, button { font-size: 16px; padding: 6px 12px; }
#status { font-size: 13px; color: #9a9; min-height: 1.2em; }
</style>
</head>
<body>
<div id="wrap">
<img id="v" alt="Rover camera">
<div class="card">
<div class="row"><label>Lights</label><button id="power">Turn on</button></div>
<div class="row"><label>Colour</label><input type="color" id="color" value="#ffffff"></div>
<div class="row"><label>Brightness</label><input type="range" id="bright" min="0" max="100" value="100"><span id="brightValue">100</span></div>
<div class="row"><label>Mode</label>
<select id="mode">
<option value="off">off</option>
<option value="solid">solid</option>
<option value="breathe">breathe</option>
<option value="chase">chase</option>
<option value="rainbow">rainbow</option>
</select></div>
<div id="status"></div>
</div>
</div>
<script>
var video = document.getElementById('v');
var power = document.getElementById('power');
var colorInput = document.getElementById('color');
var bright = document.getElementById('bright');
var brightValue = document.getElementById('brightValue');
var modeSelect = document.getElementById('mode');
var statusLine = document.getElementById('status');
var lastOnMode = 'solid';
var timer = null;

video.src = 'http://' + location.hostname + ':81/stream';

function apply(state) {
  var mode = state.mode === 'custom' ? 'solid' : state.mode;
  modeSelect.value = mode;
  colorInput.value = '#' + state.color;
  bright.value = state.brightness;
  brightValue.textContent = state.brightness;
  power.textContent = mode === 'off' ? 'Turn on' : 'Turn off';
  if (mode !== 'off') { lastOnMode = mode; }
  statusLine.textContent = '';
}

function send(query) {
  fetch('/led' + query)
    .then(function (r) { return r.json(); })
    .then(apply)
    .catch(function () { statusLine.textContent = 'No answer from the board'; });
}

function sendSoon(query) {
  clearTimeout(timer);
  timer = setTimeout(function () { send(query); }, 80);
}

colorInput.oninput = function () { sendSoon('?color=' + colorInput.value.substring(1)); };
bright.oninput = function () {
  brightValue.textContent = bright.value;
  sendSoon('?brightness=' + bright.value);
};
modeSelect.onchange = function () { send('?mode=' + modeSelect.value); };
power.onclick = function () {
  send('?mode=' + (power.textContent === 'Turn off' ? 'off' : lastOnMode));
};

send('');
</script>
</body>
</html>
)rawliteral";

const char* const STREAM_CONTENT_TYPE =
  "multipart/x-mixed-replace;boundary=frame";
const char* const STREAM_BOUNDARY = "\r\n--frame\r\n";
const char* const STREAM_PART_FORMAT =
  "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

// ============================================================
// Camera
// ============================================================

bool setupCamera() {
  camera_config_t config = {};

  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;

  config.pin_d0 = CAM_PIN_D0;
  config.pin_d1 = CAM_PIN_D1;
  config.pin_d2 = CAM_PIN_D2;
  config.pin_d3 = CAM_PIN_D3;
  config.pin_d4 = CAM_PIN_D4;
  config.pin_d5 = CAM_PIN_D5;
  config.pin_d6 = CAM_PIN_D6;
  config.pin_d7 = CAM_PIN_D7;

  config.pin_xclk     = CAM_PIN_XCLK;
  config.pin_pclk     = CAM_PIN_PCLK;
  config.pin_vsync    = CAM_PIN_VSYNC;
  config.pin_href     = CAM_PIN_HREF;
  config.pin_sccb_sda = CAM_PIN_SDA;
  config.pin_sccb_scl = CAM_PIN_SCL;
  config.pin_pwdn     = CAM_PIN_PWDN;
  config.pin_reset    = CAM_PIN_RESET;

  config.xclk_freq_hz = CAMERA_XCLK_HZ;
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size   = CAMERA_FRAME_SIZE;
  config.jpeg_quality = CAMERA_JPEG_QUALITY;
  config.fb_count     = CAMERA_FRAME_BUFFERS;
  config.fb_location  = CAMERA_FB_IN_PSRAM;
  config.grab_mode    = CAMERA_GRAB_LATEST;

  esp_err_t err = esp_camera_init(&config);

  if (err != ESP_OK) {
    Serial.printf("Camera init failed: 0x%x\n", err);
    return false;
  }

  sensor_t* sensor = esp_camera_sensor_get();

  if (sensor != nullptr) {
    sensor->set_vflip(sensor, CAMERA_VFLIP ? 1 : 0);
    sensor->set_hmirror(sensor, CAMERA_HMIRROR ? 1 : 0);
  }

  Serial.println("Camera ready.");
  return true;
}

// ============================================================
// Camera switch
// ============================================================

// Takes the camera lock and one frame. The lock is held until
// releaseFrame(), so the camera cannot be switched off while a frame
// is in use. Returns nullptr if the camera is off or busy.
camera_fb_t* grabFrame() {
  if (xSemaphoreTake(cameraMutex, pdMS_TO_TICKS(CAMERA_LOCK_WAIT_MS)) != pdTRUE) {
    return nullptr;
  }

  if (!cameraReady) {
    xSemaphoreGive(cameraMutex);
    return nullptr;
  }

  camera_fb_t* frame = esp_camera_fb_get();

  if (frame == nullptr) {
    xSemaphoreGive(cameraMutex);
  }

  return frame;
}

void releaseFrame(camera_fb_t* frame) {
  esp_camera_fb_return(frame);
  xSemaphoreGive(cameraMutex);
}

// Turns the camera on or off. Off releases the camera driver and
// stops its clock. The chip, Wi-Fi, OTA and the IR sensors keep
// running. Returns true if the camera is in the state asked for.
bool setCameraRunning(bool on) {
  if (xSemaphoreTake(cameraMutex, pdMS_TO_TICKS(CAMERA_SWITCH_WAIT_MS)) != pdTRUE) {
    Serial.println("Camera busy: switch skipped.");
    return false;
  }

  if (on && !cameraReady) {
    cameraReady = setupCamera();
  }
  else if (!on && cameraReady) {
    cameraReady = false;
    esp_camera_deinit();
    Serial.println("Camera off.");
  }

  bool inRequestedState = (cameraReady == on);

  xSemaphoreGive(cameraMutex);

  lastIrReportMs = 0;   // report the new state right away

  return inRequestedState;
}

// ============================================================
// HTTP handlers
// ============================================================

esp_err_t indexHandler(httpd_req_t* req) {
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, INDEX_HTML, HTTPD_RESP_USE_STRLEN);
}

// GET /led  answers with the LED state as JSON.
// GET /led?color=RRGGBB&brightness=0..100&mode=name  changes it first.
// Picking a colour while the lights are off turns them on (solid).
esp_err_t ledHandler(httpd_req_t* req) {
  char query[96];
  char value[16];

  if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK) {

    if (httpd_query_key_value(query, "color", value, sizeof(value)) == ESP_OK &&
        strlen(value) == 6) {
      char* end = nullptr;
      unsigned long rgb = strtoul(value, &end, 16);

      if (*end == '\0') {
        ledColor = (uint32_t)rgb;
        ledSetMode(ledMode == LED_OFF ? LED_SOLID : ledMode);
      }
    }

    if (httpd_query_key_value(query, "brightness", value, sizeof(value)) == ESP_OK) {
      long percent = 0;

      if (parseInt(value, percent)) {
        ledBrightnessPercent = constrain(percent, 0, 100);
        ledDirty     = true;
        telemetryDue = true;
      }
    }

    if (httpd_query_key_value(query, "mode", value, sizeof(value)) == ESP_OK) {
      LedMode mode = LED_OFF;

      if (parseLedMode(value, mode)) {
        ledSetMode(mode);
      }
    }
  }

  char json[96];
  snprintf(
    json, sizeof(json),
    "{\"mode\":\"%s\",\"color\":\"%06lx\",\"brightness\":%d}",
    LED_MODE_NAMES[ledMode], (unsigned long)ledColor, ledBrightnessPercent
  );

  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  return httpd_resp_send(req, json, HTTPD_RESP_USE_STRLEN);
}

esp_err_t captureHandler(httpd_req_t* req) {
  if (otaActive) {
    httpd_resp_send_500(req);
    return ESP_FAIL;
  }

  camera_fb_t* frame = grabFrame();

  if (frame == nullptr) {
    httpd_resp_set_status(req, "503 Service Unavailable");
    httpd_resp_send(req, "camera off or busy", HTTPD_RESP_USE_STRLEN);
    return ESP_FAIL;
  }

  httpd_resp_set_type(req, "image/jpeg");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

  esp_err_t result = httpd_resp_send(
    req,
    (const char*)frame->buf,
    frame->len
  );

  releaseFrame(frame);
  return result;
}

// Sends frames until the viewer disconnects or an OTA update
// starts. This blocks the stream server's task, which is why the
// stream has its own server and only one viewer at a time.
esp_err_t streamHandler(httpd_req_t* req) {
  if (!cameraReady) {
    httpd_resp_set_status(req, "503 Service Unavailable");
    return httpd_resp_send(req, "camera off", HTTPD_RESP_USE_STRLEN);
  }

  esp_err_t result = httpd_resp_set_type(req, STREAM_CONTENT_TYPE);

  if (result != ESP_OK) {
    return result;
  }

  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

  char partHeader[64];
  unsigned long lastFrameMs = 0;

  while (!otaActive) {
    unsigned long now = millis();

    if (now - lastFrameMs < STREAM_MIN_FRAME_MS) {
      vTaskDelay(pdMS_TO_TICKS(STREAM_MIN_FRAME_MS - (now - lastFrameMs)));
    }

    lastFrameMs = millis();

    camera_fb_t* frame = grabFrame();

    if (frame == nullptr) {
      if (!cameraReady) {
        // The camera was switched off: end the stream cleanly.
        httpd_resp_send_chunk(req, nullptr, 0);
        return ESP_OK;
      }

      Serial.println("Camera frame capture failed.");
      return ESP_FAIL;
    }

    size_t headerLength = snprintf(
      partHeader,
      sizeof(partHeader),
      STREAM_PART_FORMAT,
      (unsigned int)frame->len
    );

    result = httpd_resp_send_chunk(req, STREAM_BOUNDARY, strlen(STREAM_BOUNDARY));

    if (result == ESP_OK) {
      result = httpd_resp_send_chunk(req, partHeader, headerLength);
    }

    if (result == ESP_OK) {
      result = httpd_resp_send_chunk(
        req,
        (const char*)frame->buf,
        frame->len
      );
    }

    releaseFrame(frame);

    if (result != ESP_OK) {
      break;   // the viewer went away
    }
  }

  return result;
}

void startCameraServers() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();

  httpd_uri_t indexUri = {};
  indexUri.uri     = "/";
  indexUri.method  = HTTP_GET;
  indexUri.handler = indexHandler;

  httpd_uri_t captureUri = {};
  captureUri.uri     = "/capture";
  captureUri.method  = HTTP_GET;
  captureUri.handler = captureHandler;

  httpd_uri_t ledUri = {};
  ledUri.uri     = "/led";
  ledUri.method  = HTTP_GET;
  ledUri.handler = ledHandler;

  httpd_uri_t streamUri = {};
  streamUri.uri     = "/stream";
  streamUri.method  = HTTP_GET;
  streamUri.handler = streamHandler;

  config.server_port = PAGE_PORT;

  if (httpd_start(&pageServer, &config) == ESP_OK) {
    httpd_register_uri_handler(pageServer, &indexUri);
    httpd_register_uri_handler(pageServer, &captureUri);
    httpd_register_uri_handler(pageServer, &ledUri);
  }
  else {
    Serial.println("ERROR: page server failed to start.");
  }

  // Each server needs its own control port.
  config.server_port = STREAM_PORT;
  config.ctrl_port  += 1;

  if (httpd_start(&streamServer, &config) == ESP_OK) {
    httpd_register_uri_handler(streamServer, &streamUri);
  }
  else {
    Serial.println("ERROR: stream server failed to start.");
  }
}

// ============================================================
// Commands (Deck -> camera board)
// ============================================================

// Handles "CAM,ON" and "CAM,OFF".
void handleCameraCommand(const char* arg) {
  if (strcasecmp(arg, "ON") == 0) {
    setCameraRunning(true);
  }
  else if (strcasecmp(arg, "OFF") == 0) {
    setCameraRunning(false);
  }
  else {
    Serial.println("Invalid camera command.");
  }
}

// Reads one pending command on CMD_PORT and passes it on: "CAM,...",
// "MIC,..." or "LED,...".
void handleCommands() {
  int packetSize = cmdUdp.parsePacket();

  if (packetSize <= 0) {
    return;
  }

  int length = cmdUdp.read(cmdBuffer, CMD_BUFFER_SIZE - 1);

  if (length <= 0) {
    return;
  }

  cmdBuffer[length] = '\0';

  IPAddress sender = cmdUdp.remoteIP();

  // Drop a trailing newline or space.
  while (length > 0 &&
         (cmdBuffer[length - 1] == '\n' || cmdBuffer[length - 1] == '\r' ||
          cmdBuffer[length - 1] == ' ')) {
    cmdBuffer[--length] = '\0';
  }

  char* tokens[CMD_MAX_TOKENS];
  int   count = 0;

  char* token = strtok(cmdBuffer, ",");
  while (token != nullptr && count < CMD_MAX_TOKENS) {
    tokens[count++] = token;
    token = strtok(nullptr, ",");
  }

  if (count < 2) {
    Serial.println("Invalid command received.");
    return;
  }

  if (strcmp(tokens[0], "CAM") == 0) {
    handleCameraCommand(tokens[1]);
  }
  else if (strcmp(tokens[0], "MIC") == 0) {
    handleMicCommand(tokens[1], sender);
  }
  else if (strcmp(tokens[0], "LED") == 0) {
    handleLedCommand(tokens, count);
  }
  else {
    Serial.println("Invalid command received.");
  }
}

// ============================================================
// Network setup
// ============================================================

bool setupNetwork() {

  WiFi.setHostname(mdnsHostname);
  WiFi.mode(WIFI_STA);

  if (use_static_ip) {
    WiFi.config(static_ip, static_gateway, static_subnet);
  }

  Serial.print("Trying to join Wi-Fi network: ");
  Serial.println(sta_ssid);

  WiFi.begin(sta_ssid, sta_password);

  unsigned long startAttempt = millis();

  while (
    WiFi.status() != WL_CONNECTED &&
    millis() - startAttempt < STA_TIMEOUT_MS
  ) {
    delay(WIFI_POLL_INTERVAL_MS);
    Serial.print(".");
  }

  Serial.println();

  // ----------------------------------------------------------
  // Normal Wi-Fi mode
  // ----------------------------------------------------------

  if (WiFi.status() == WL_CONNECTED) {

    Serial.println("Wi-Fi connected.");

    // Steady streaming needs modem sleep off. Do not remove.
    WiFi.setSleep(false);
    WiFi.setAutoReconnect(true);

    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());

  }

  // ----------------------------------------------------------
  // Fallback AP mode
  // ----------------------------------------------------------

  else {

    Serial.println("Home network unavailable.");
    Serial.println("Starting fallback access point.");

    WiFi.disconnect(true);

    delay(AP_START_DELAY_MS);

    WiFi.mode(WIFI_AP);

    bool apStarted = WiFi.softAP(ap_ssid, ap_password);

    if (apStarted) {

      Serial.print("Access point started: ");
      Serial.println(ap_ssid);

      Serial.print("Camera IP address: ");
      Serial.println(WiFi.softAPIP());

    }
    else {

      Serial.println("ERROR: Failed to start fallback access point.");

      return false;
    }
  }

  // ----------------------------------------------------------
  // mDNS
  // ----------------------------------------------------------

  if (MDNS.begin(mdnsHostname)) {

    Serial.print("mDNS responder started: ");
    Serial.print(mdnsHostname);
    Serial.println(".local");

  }
  else {

    Serial.println("WARNING: mDNS startup failed.");

  }

  return true;
}

// ============================================================
// OTA setup
// ============================================================

void setupOTA() {

  ArduinoOTA.setHostname(mdnsHostname);
  ArduinoOTA.setPassword(ota_password);

  ArduinoOTA.onStart([]() {

    Serial.println();
    Serial.println("OTA update starting...");

    // Ends any running stream and refuses new frames.
    otaActive = true;

    // Quiet the microphone and the LEDs too.
    setMicMode(MIC_OFF, IPAddress());
    ledsOff();

  });

  ArduinoOTA.onEnd([]() {

    Serial.println();
    Serial.println("OTA update complete.");

  });

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {

    unsigned int percent = (progress * 100U) / total;

    Serial.printf("OTA progress: %u%%\r", percent);

  });

  ArduinoOTA.onError([](ota_error_t error) {

    Serial.println();
    Serial.print("OTA error: ");

    switch (error) {

      case OTA_AUTH_ERROR:
        Serial.println("Authentication failed");
        break;

      case OTA_BEGIN_ERROR:
        Serial.println("Begin failed");
        break;

      case OTA_CONNECT_ERROR:
        Serial.println("Connection failed");
        break;

      case OTA_RECEIVE_ERROR:
        Serial.println("Receive failed");
        break;

      case OTA_END_ERROR:
        Serial.println("End failed");
        break;

      default:
        Serial.println("Unknown error");
        break;
    }

    // The update failed, so streaming can resume.
    otaActive = false;

  });

  ArduinoOTA.begin();

  Serial.println("OTA ready.");
  Serial.print("OTA hostname: ");
  Serial.println(mdnsHostname);
  Serial.print("OTA port: ");
  Serial.println(OTA_PORT);
}

// ============================================================
// Setup
// ============================================================

void setup() {

  Serial.begin(SERIAL_BAUD_RATE);

  // Native USB: give a connected computer a moment to attach so the
  // first lines are not lost. Without a computer this just times out.
  unsigned long serialStart = millis();
  while (!Serial && millis() - serialStart < SERIAL_WAIT_MS) {
    delay(10);
  }

  delay(BOOT_SETTLE_DELAY_MS);

  Serial.println();
  Serial.println("==============================");
  Serial.println("ESP32 Camera Booting");
  Serial.println("==============================");

  cameraMutex = xSemaphoreCreateMutex();

  if (CAMERA_START_ENABLED) {
    cameraReady = setupCamera();
  }
  else {
    Serial.println("Camera starts off: send CAM,ON to start it.");
  }

  setupLeds();
  setupMic();
  setupIrSensors();

  if (!setupNetwork()) {

    Serial.println("Network setup failed.");

    return;
  }

  setupOTA();

  cmdUdp.begin(CMD_PORT);

  Serial.print("Listening for camera, microphone and LED commands on port ");
  Serial.println(CMD_PORT);

  // The web servers always run: they answer 503 while the camera is off.
  startCameraServers();

  IPAddress ip =
    (WiFi.getMode() == WIFI_AP) ? WiFi.softAPIP() : WiFi.localIP();

  Serial.print("Video page:   http://");
  Serial.println(ip);

  Serial.print("Video stream: http://");
  Serial.print(ip);
  Serial.print(":");
  Serial.print(STREAM_PORT);
  Serial.println("/stream");

  if (!cameraReady) {
    Serial.println("Camera is off: Wi-Fi, OTA, the LEDs and the microphone control are running.");
  }

  Serial.println("Camera board ready.");
}

// ============================================================
// Main loop
// ============================================================

void loop() {

  // OTA must be serviced continuously. The web servers run in
  // their own tasks.
  ArduinoOTA.handle();

  handleCommands();

  ledUpdate();

  if (!otaActive &&
      (telemetryDue || millis() - lastIrReportMs >= IR_REPORT_INTERVAL_MS)) {
    telemetryDue   = false;
    lastIrReportMs = millis();
    reportTelemetry();
  }

  delay(1);
}
