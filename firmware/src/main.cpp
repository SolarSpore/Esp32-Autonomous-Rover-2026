/**********************************************************************
  ESP32 4-Motor UDP Rover Controller + OTA + Head Servo + Sensors

  Hardware:
    - ESP32-WROOM-32
    - 2x L298N
    - 4x DC gear motors (mecanum wheels, X-pattern rollers)
    - Motors: FL, FR, RL, RR
    - SG90 head servo (GPIO 13)
    - HC-SR04 ultrasonic sensor (TRIG 14, ECHO 34)
    - Buzzer module on GPIO 32 (idles HIGH, sounds when driven LOW;
      behaves as a passive buzzer, so it needs a tone to be audible)
    - 5-way TCRT5000L digital line module (33, 35, 36, 39, 16)
    - 2x IR digital sensors (17, 4)

  UDP drive commands (port 4210, Deck -> rover):
    "FL,FR,RL,RR"    each value -255 .. 255

  UDP aux commands (port 4212, Deck -> rover, one sender only):
    "HEAD,<angle>"       hold the head at <angle> degrees (0..180)
    "RELEASE"            head goes limp
    "BUZZ,<hz>,<ms>"     play a tone (non-blocking, volume fixed)

  Failsafes (independent):
    Drive: motors stop if no valid drive packet for FAILSAFE_MS.
    Head:  servo is released if no HEAD packet for AUX_FAILSAFE_MS.
    Buzzer: every commanded tone ends by itself (max
            BUZZER_CMD_MAX_MS). stopAll() (OTA start/error) stops
            everything: motors, head and buzzer.

  Network:
    1. Try configured Wi-Fi network.
    2. If unavailable, start fallback access point.
    3. mDNS hostname: esp32rover.local
    4. Wi-Fi modem sleep is disabled in station mode for steady
       UDP latency. Do not remove.

  OTA:
    - ArduinoOTA
    - OTA hostname: esp32rover
    - OTA password configured in config.h
    - Motors, servo and buzzer are stopped when an OTA update starts
    - USB remains the recovery method

  Sensors / telemetry (port 4211, rover -> network):
    Sensor readings are printed to Serial and also broadcast as a
    text line over UDP every SENSOR_REPORT_INTERVAL_MS. One-way and
    separate from the command ports. rover_sensor_bridge consumes it.

  Buzzer note:
    The idle (silent) level is HIGH. Use the tone functions for
    audible sound. Loudness is set by BUZZER_VOLUME_PERCENT.
    Never write the pin directly.

  Servo note:
    No position feedback. Align the horn mechanically, then fine
    tune with SERVO_CENTER_TRIM_DEG.

  Boot sequence:
    Wi-Fi connect -> ready jingle -> servo sweep and center hold.
**********************************************************************/

#include "config.h"

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ESPmDNS.h>
#include <ArduinoOTA.h>

// ============================================================
// Bench test switches
// ============================================================

// Set to true to run a one-time test at the end of setup().
const bool RUN_SERVO_BENCH_TEST  = true;
const bool RUN_BUZZER_BENCH_TEST = true;

// ============================================================
// General constants
// ============================================================

const unsigned long SERIAL_BAUD_RATE       = 115200;
const unsigned long BOOT_SETTLE_DELAY_MS   = 500;

// ============================================================
// Network constants
// ============================================================

const unsigned long STA_TIMEOUT_MS         = 8000;
const unsigned long WIFI_POLL_INTERVAL_MS  = 300;
const unsigned long AP_START_DELAY_MS      = 100;

const unsigned int  UDP_PORT               = 4210;   // drive commands
const unsigned int  TELEMETRY_PORT         = 4211;   // sensor readings
const unsigned int  AUX_PORT               = 4212;   // head + buzzer
const unsigned int  OTA_PORT               = 3232;   // ArduinoOTA default

const size_t        PACKET_BUFFER_SIZE     = 128;
const size_t        AUX_BUFFER_SIZE        = 64;

WiFiUDP udp;
WiFiUDP telemetryUdp;
WiFiUDP auxUdp;

char packetBuffer[PACKET_BUFFER_SIZE];
char auxBuffer[AUX_BUFFER_SIZE];

const IPAddress TELEMETRY_BROADCAST_IP(255, 255, 255, 255);

// ============================================================
// Aux protocol constants
// ============================================================

const char* const AUX_CMD_HEAD    = "HEAD";
const char* const AUX_CMD_RELEASE = "RELEASE";
const char* const AUX_CMD_BUZZ    = "BUZZ";

const int           NUMBER_BASE     = 10;
const unsigned long AUX_FAILSAFE_MS = 500;   // head releases after this

// ============================================================
// Motor constants
// ============================================================

const int MOTOR_COUNT            = 4;
const int MOTOR_SPEED_MIN        = -255;
const int MOTOR_SPEED_MAX        = 255;
const int MOTOR_SPEED_STOP       = 0;

const int PWM_CHANNELS_PER_MOTOR = 2;   // one channel per H-bridge input
const int MOTOR_PWM_FREQ_HZ      = 5000;
const int MOTOR_PWM_RES_BITS     = 8;

struct MotorPins {
  int pinA;
  int pinB;
};

// FL, FR, RL, RR
const MotorPins MOTOR_PINS[MOTOR_COUNT] = {
  {23, 25},  // FL
  {26, 27},  // FR
  {21, 22},  // RL
  {18, 19}   // RR
};

// LEDC channel layout:
//   Motor 0: channels 0, 1
//   Motor 1: channels 2, 3
//   Motor 2: channels 4, 5
//   Motor 3: channels 6, 7
//   Servo:   channel 8 (see SERVO_PWM_CHANNEL); channel 9 unused
//   Buzzer:  channel 10 (see BUZZER_PWM_CHANNEL)
//
// LEDC channels share timers in pairs, so the servo (8) and buzzer
// (10) must not sit in the same pair or changing the buzzer tone
// would change the servo frequency.

// ============================================================
// Drive failsafe constants
// ============================================================

const unsigned long FAILSAFE_MS = 500;

unsigned long lastPacketMs = 0;

// ============================================================
// Buzzer constants
// ============================================================

const int BUZZER_PIN = 32;

// Idle level is HIGH (silent). LOW is the "driven" level.
const int BUZZER_ON_LEVEL  = LOW;
const int BUZZER_OFF_LEVEL = HIGH;

const int           BUZZER_PWM_CHANNEL  = 10;
const int           BUZZER_PWM_RES_BITS = 8;
const uint32_t      BUZZER_DUTY_MAX     = (1UL << BUZZER_PWM_RES_BITS) - 1;
const unsigned long BUZZER_TEST_BEEP_MS = 200;

// Limits applied to tones commanded over the network.
const long BUZZER_CMD_MIN_FREQ_HZ = 200;
const long BUZZER_CMD_MAX_FREQ_HZ = 5000;
const long BUZZER_CMD_MIN_MS      = 1;
const long BUZZER_CMD_MAX_MS      = 1000;

// Loudness, 1 to 50. 50 is the loudest (square wave); lower is
// quieter. Jingle and commanded tones share this setting.
const int BUZZER_VOLUME_PERCENT = 10;

// The module sounds while the pin is LOW, so the HIGH share of each
// cycle is the silent part.
const uint32_t BUZZER_TONE_DUTY =
  (BUZZER_DUTY_MAX * (100 - BUZZER_VOLUME_PERCENT)) / 100;

// Non-blocking tone state.
bool          buzzerToneActive = false;
unsigned long buzzerToneEndMs  = 0;

// Note frequencies in Hz.
const unsigned int NOTE_REST = 0;
const unsigned int NOTE_C5   = 523;
const unsigned int NOTE_D5   = 587;
const unsigned int NOTE_E5   = 659;
const unsigned int NOTE_F5   = 698;
const unsigned int NOTE_G5   = 784;
const unsigned int NOTE_A5   = 880;
const unsigned int NOTE_C6   = 1047;

struct Note {
  unsigned int  freqHz;
  unsigned long durationMs;
};

// Boot "I'm ready" jingle. Edit this table to change the tune.
const Note BOOT_JINGLE[] = {
  {NOTE_C5,   120},
  {NOTE_E5,   120},
  {NOTE_G5,   120},
  {NOTE_REST,  60},
  {NOTE_C6,   300}
};

const int BOOT_JINGLE_LENGTH =
  sizeof(BOOT_JINGLE) / sizeof(BOOT_JINGLE[0]);

const unsigned long JINGLE_NOTE_GAP_MS = 30;

// ============================================================
// Servo constants (SG90 head)
// ============================================================

const int SERVO_PIN             = 13;
const int SERVO_PWM_CHANNEL     = MOTOR_COUNT * PWM_CHANNELS_PER_MOTOR;  // 8
const int SERVO_PWM_FREQ_HZ     = 50;   // standard hobby servo
const int SERVO_PWM_RES_BITS    = 16;

const uint32_t SERVO_PERIOD_US  = 1000000UL / SERVO_PWM_FREQ_HZ;         // 20000
const uint32_t SERVO_DUTY_MAX   = (1UL << SERVO_PWM_RES_BITS) - 1;       // 65535
const uint32_t SERVO_DUTY_OFF   = 0;    // no pulses: servo goes limp

// Pulse range. 1000-2000 us centers exactly at 1500 us. For the
// wider SG90 range use 500 / 2400 (center becomes 1450 us).
const int SERVO_MIN_US          = 1000;
const int SERVO_MAX_US          = 2000;

const int SERVO_MIN_DEG         = 0;
const int SERVO_MAX_DEG         = 180;
const int SERVO_CENTER_DEG      = 90;

// Fine adjustment so "center" faces forward. Keep within about
// +/-5 degrees; fix larger offsets by refitting the horn.
const int SERVO_CENTER_TRIM_DEG = 0;

const unsigned long SERVO_TEST_STEP_MS  = 1000;
const unsigned long SERVO_ALIGN_HOLD_MS = 15000;

// Head command state (see AUX_FAILSAFE_MS).
bool          headActive        = false;
unsigned long lastHeadPacketMs  = 0;

// ============================================================
// Ultrasonic constants (HC-SR04)
// ============================================================

// TRIG goes to GPIO 14. Avoid GPIO 12: it is a strapping pin and
// can cause boot problems. The wire and this constant must match.
const int ULTRASONIC_TRIG_PIN = 14;
const int ULTRASONIC_ECHO_PIN = 34;   // input-only pin, ECHO must be 3.3V-safe

const float ULTRASONIC_MAX_RANGE_CM  = 200.0f;
const float SOUND_CM_PER_US          = 0.0343f;   // ~343 m/s at room temp
const float ULTRASONIC_NO_ECHO_CM    = -1.0f;     // returned when nothing is heard

// Round trip: the pulse travels out and back, so 2x the range.
const unsigned long ULTRASONIC_TIMEOUT_US =
  (unsigned long)((2.0f * ULTRASONIC_MAX_RANGE_CM) / SOUND_CM_PER_US);

const unsigned int ULTRASONIC_TRIG_SETTLE_US = 2;
const unsigned int ULTRASONIC_TRIG_PULSE_US  = 10;

// ============================================================
// Line sensor and IR sensor constants
// ============================================================

const int LINE_SENSOR_COUNT = 5;
const int IR_SENSOR_COUNT   = 2;

// Pins, left to right as wired.
const int LINE_PINS[LINE_SENSOR_COUNT] = {33, 35, 36, 39, 16};
const int IR_PINS[IR_SENSOR_COUNT]     = {17, 4};

// Most of these modules output LOW when they see a reflective
// surface (LED lit). Verify with a white and a dark surface, and
// flip these if the readings are inverted.
const int LINE_ACTIVE_LEVEL = LOW;
const int IR_ACTIVE_LEVEL   = LOW;

// ============================================================
// Sensor report constants
// ============================================================

const unsigned long SENSOR_REPORT_INTERVAL_MS = 200;
const size_t        SENSOR_REPORT_BUFFER_SIZE = 96;

unsigned long lastSensorReportMs = 0;

// ============================================================
// Buzzer
// ============================================================

// The single place that knows which pin level is "driven".
void buzzerSet(bool sounding) {
  digitalWrite(BUZZER_PIN, sounding ? BUZZER_ON_LEVEL : BUZZER_OFF_LEVEL);
}

void buzzerOn() {
  buzzerSet(true);
}

void buzzerOff() {
  buzzerSet(false);
}

// Level beep. On a passive buzzer this is only a click; use the
// tone functions for audible sound. Blocks for durationMs.
void buzzerBeep(unsigned long durationMs) {
  buzzerOn();
  delay(durationMs);
  buzzerOff();
}

// Repeated level beeps with a gap between them. Blocks while it runs.
void buzzerBeepCount(int count, unsigned long onMs, unsigned long offMs) {
  for (int i = 0; i < count; i++) {
    buzzerBeep(onMs);
    if (i < count - 1) {
      delay(offMs);
    }
  }
}

// Starts a tone at the configured volume. Does not block.
void buzzerStartTone(unsigned int freqHz) {
  ledcSetup(BUZZER_PWM_CHANNEL, freqHz, BUZZER_PWM_RES_BITS);
  ledcAttachPin(BUZZER_PIN, BUZZER_PWM_CHANNEL);
  ledcWrite(BUZZER_PWM_CHANNEL, BUZZER_TONE_DUTY);
}

// Ends a tone and returns the pin to its silent idle level.
void buzzerStopTone() {
  ledcWrite(BUZZER_PWM_CHANNEL, BUZZER_DUTY_MAX);  // held HIGH = silent
  ledcDetachPin(BUZZER_PIN);
  pinMode(BUZZER_PIN, OUTPUT);
  buzzerOff();
}

// Plays a tone for durationMs. Blocks while it plays.
void buzzerTone(unsigned int freqHz, unsigned long durationMs) {
  buzzerStartTone(freqHz);
  delay(durationMs);
  buzzerStopTone();
}

// Plays a sequence of notes. A NOTE_REST entry is a silent pause.
// Blocks until the whole melody has played.
void buzzerPlayJingle(const Note* notes, int count) {
  for (int i = 0; i < count; i++) {
    if (notes[i].freqHz == NOTE_REST) {
      delay(notes[i].durationMs);
    }
    else {
      buzzerTone(notes[i].freqHz, notes[i].durationMs);
    }
    delay(JINGLE_NOTE_GAP_MS);
  }
}

// Starts a tone that ends by itself after durationMs. Does not
// block; buzzerUpdate() must be called every loop pass.
void buzzerStartTimedTone(unsigned int freqHz, unsigned long durationMs) {
  buzzerStartTone(freqHz);
  buzzerToneActive = true;
  buzzerToneEndMs  = millis() + durationMs;
}

// Ends a timed tone once its time is up.
void buzzerUpdate() {
  if (buzzerToneActive && (long)(millis() - buzzerToneEndMs) >= 0) {
    buzzerStopTone();
    buzzerToneActive = false;
  }
}

// Immediately silences the buzzer, whatever state it is in.
void buzzerSilence() {
  if (buzzerToneActive) {
    buzzerStopTone();
    buzzerToneActive = false;
  }
  else {
    buzzerOff();
  }
}

// Called first thing in setup() so the buzzer never chirps at boot.
// The level is written before pinMode so the pin never glitches LOW.
void setupBuzzer() {
  digitalWrite(BUZZER_PIN, BUZZER_OFF_LEVEL);
  pinMode(BUZZER_PIN, OUTPUT);
  buzzerOff();
}

// ============================================================
// Head servo (SG90)
// ============================================================

// Convert a pulse width in microseconds to a LEDC duty value.
uint32_t servoUsToDuty(int pulseUs) {
  return (uint32_t)(((uint64_t)pulseUs * SERVO_DUTY_MAX) / SERVO_PERIOD_US);
}

// Angle in degrees, with the center trim applied. Sends pulses, so
// the servo holds this position.
void setServoAngle(int angle) {
  angle = constrain(
    angle + SERVO_CENTER_TRIM_DEG,
    SERVO_MIN_DEG,
    SERVO_MAX_DEG
  );
  int pulseUs = map(
    angle,
    SERVO_MIN_DEG, SERVO_MAX_DEG,
    SERVO_MIN_US, SERVO_MAX_US
  );
  ledcWrite(SERVO_PWM_CHANNEL, servoUsToDuty(pulseUs));
}

// Stop sending pulses. The servo goes limp and can't strain
// against anything. This is the servo's "stopped" state.
void servoRelease() {
  ledcWrite(SERVO_PWM_CHANNEL, SERVO_DUTY_OFF);
}

void setupServo() {
  ledcSetup(SERVO_PWM_CHANNEL, SERVO_PWM_FREQ_HZ, SERVO_PWM_RES_BITS);
  ledcAttachPin(SERVO_PIN, SERVO_PWM_CHANNEL);
  servoRelease();  // start limp, so there is no jerk at boot
}

// ============================================================
// Motor control
// ============================================================

void setMotor(int index, int speed) {
  if (index < 0 || index >= MOTOR_COUNT) {
    return;
  }

  speed = constrain(speed, MOTOR_SPEED_MIN, MOTOR_SPEED_MAX);

  int channelA = index * PWM_CHANNELS_PER_MOTOR;
  int channelB = channelA + 1;

  if (speed > MOTOR_SPEED_STOP) {
    // Forward
    ledcWrite(channelA, speed);
    ledcWrite(channelB, 0);
  }
  else if (speed < MOTOR_SPEED_STOP) {
    // Reverse
    ledcWrite(channelA, 0);
    ledcWrite(channelB, -speed);
  }
  else {
    // Stop
    ledcWrite(channelA, 0);
    ledcWrite(channelB, 0);
  }
}

void setupMotors() {
  for (int i = 0; i < MOTOR_COUNT; i++) {
    int channelA = i * PWM_CHANNELS_PER_MOTOR;
    int channelB = channelA + 1;

    ledcSetup(channelA, MOTOR_PWM_FREQ_HZ, MOTOR_PWM_RES_BITS);
    ledcSetup(channelB, MOTOR_PWM_FREQ_HZ, MOTOR_PWM_RES_BITS);

    ledcAttachPin(MOTOR_PINS[i].pinA, channelA);
    ledcAttachPin(MOTOR_PINS[i].pinB, channelB);
  }
}

// Stops the four drive motors only. Used by the drive failsafe.
void stopMotors() {
  for (int i = 0; i < MOTOR_COUNT; i++) {
    setMotor(i, MOTOR_SPEED_STOP);
  }
}

// ============================================================
// Safe state
// ============================================================

// Puts every actuator in its safe state: motors stopped, head
// released (limp), buzzer silent. Used by OTA start and OTA error.
void stopAll() {
  stopMotors();
  servoRelease();
  headActive = false;
  buzzerSilence();
}

// ============================================================
// Ultrasonic sensor (HC-SR04)
// ============================================================

void setupUltrasonic() {
  pinMode(ULTRASONIC_TRIG_PIN, OUTPUT);
  pinMode(ULTRASONIC_ECHO_PIN, INPUT);
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
}

// Returns distance in cm, or ULTRASONIC_NO_ECHO_CM if no echo was
// received. Blocks for up to ULTRASONIC_TIMEOUT_US when nothing is
// in range.
float readUltrasonicCm() {
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
  delayMicroseconds(ULTRASONIC_TRIG_SETTLE_US);
  digitalWrite(ULTRASONIC_TRIG_PIN, HIGH);
  delayMicroseconds(ULTRASONIC_TRIG_PULSE_US);
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);

  unsigned long echoUs = pulseIn(
    ULTRASONIC_ECHO_PIN,
    HIGH,
    ULTRASONIC_TIMEOUT_US
  );

  if (echoUs == 0) {
    return ULTRASONIC_NO_ECHO_CM;
  }

  // Halve the round-trip time to get the one-way distance.
  return (echoUs * SOUND_CM_PER_US) / 2.0f;
}

// ============================================================
// Line sensor (5-way TCRT5000L digital module) and IR sensors
// ============================================================

void setupLineAndIrSensors() {
  // The modules have their own pull-ups. GPIO 35/36/39 have no
  // internal pull-ups, so plain INPUT is used everywhere.
  for (int i = 0; i < LINE_SENSOR_COUNT; i++) {
    pinMode(LINE_PINS[i], INPUT);
  }
  for (int i = 0; i < IR_SENSOR_COUNT; i++) {
    pinMode(IR_PINS[i], INPUT);
  }
}

// Returns true if the line sensor at this index reads "active".
bool lineActive(int index) {
  return digitalRead(LINE_PINS[index]) == LINE_ACTIVE_LEVEL;
}

// Returns true if the IR sensor at this index reads "active".
bool irActive(int index) {
  return digitalRead(IR_PINS[index]) == IR_ACTIVE_LEVEL;
}

// ============================================================
// Telemetry (rover -> network, one-way)
// ============================================================

// Broadcasts one line of text. Skipped if there is no network.
void sendTelemetry(const char* line) {
  bool networkUp =
    (WiFi.getMode() == WIFI_AP) ||
    (WiFi.status() == WL_CONNECTED);

  if (!networkUp) {
    return;
  }

  telemetryUdp.beginPacket(TELEMETRY_BROADCAST_IP, TELEMETRY_PORT);
  telemetryUdp.print(line);
  telemetryUdp.endPacket();
}

// Reads every sensor and sends one report over Serial and UDP.
void reportSensors() {
  float distanceCm = readUltrasonicCm();

  char lineBits[LINE_SENSOR_COUNT + 1];
  for (int i = 0; i < LINE_SENSOR_COUNT; i++) {
    lineBits[i] = lineActive(i) ? '1' : '0';
  }
  lineBits[LINE_SENSOR_COUNT] = '\0';

  char irBits[IR_SENSOR_COUNT + 1];
  for (int i = 0; i < IR_SENSOR_COUNT; i++) {
    irBits[i] = irActive(i) ? '1' : '0';
  }
  irBits[IR_SENSOR_COUNT] = '\0';

  char report[SENSOR_REPORT_BUFFER_SIZE];
  snprintf(
    report,
    sizeof(report),
    "Dist: %6.1f cm | Line: %s | IR: %s",
    distanceCm,
    lineBits,
    irBits
  );

  Serial.println(report);
  sendTelemetry(report);
}

// ============================================================
// UDP drive command parser
// ============================================================

// Parses "FL,FR,RL,RR". Applies the speeds and returns true only
// if exactly MOTOR_COUNT values were present.
bool parseCommand(char* command) {
  int values[MOTOR_COUNT] = {0, 0, 0, 0};

  int index = 0;

  char* token = strtok(command, ",");

  while (token != nullptr && index < MOTOR_COUNT) {
    values[index] = atoi(token);
    index++;

    token = strtok(nullptr, ",");
  }

  // We require exactly MOTOR_COUNT motor values.
  if (index != MOTOR_COUNT) {
    return false;
  }

  for (int i = 0; i < MOTOR_COUNT; i++) {
    setMotor(i, values[i]);
  }

  return true;
}

// Reads one pending UDP drive packet, if any. Refreshes the
// failsafe timer on a valid command, stops the motors on an
// invalid one.
void handleDriveCommands() {
  int packetSize = udp.parsePacket();

  if (packetSize <= 0) {
    return;
  }

  int length = udp.read(packetBuffer, PACKET_BUFFER_SIZE - 1);

  if (length <= 0) {
    return;
  }

  packetBuffer[length] = '\0';

  if (parseCommand(packetBuffer)) {
    lastPacketMs = millis();
  }
  else {
    Serial.println("Invalid UDP command received.");
    stopMotors();
  }
}

// ============================================================
// UDP aux command parser (head + buzzer)
// ============================================================

// Parses a whole number. Returns false if text is not a clean integer.
bool parseLong(const char* text, long& value) {
  if (text == nullptr) {
    return false;
  }

  char* end = nullptr;
  value = strtol(text, &end, NUMBER_BASE);

  return end != text && *end == '\0';
}

// Handles "HEAD,<angle>", "RELEASE" and "BUZZ,<hz>,<ms>".
// Returns true if the command was valid and applied.
bool parseAuxCommand(char* command) {
  char* word = strtok(command, ",");

  if (word == nullptr) {
    return false;
  }

  if (strcmp(word, AUX_CMD_HEAD) == 0) {
    long angle = 0;
    if (!parseLong(strtok(nullptr, ","), angle)) {
      return false;
    }
    setServoAngle((int)angle);
    headActive       = true;
    lastHeadPacketMs = millis();
    return true;
  }

  if (strcmp(word, AUX_CMD_RELEASE) == 0) {
    servoRelease();
    headActive = false;
    return true;
  }

  if (strcmp(word, AUX_CMD_BUZZ) == 0) {
    long freqHz = 0;
    long durationMs = 0;
    if (!parseLong(strtok(nullptr, ","), freqHz) ||
        !parseLong(strtok(nullptr, ","), durationMs)) {
      return false;
    }
    freqHz     = constrain(freqHz, BUZZER_CMD_MIN_FREQ_HZ, BUZZER_CMD_MAX_FREQ_HZ);
    durationMs = constrain(durationMs, BUZZER_CMD_MIN_MS, BUZZER_CMD_MAX_MS);
    buzzerStartTimedTone((unsigned int)freqHz, (unsigned long)durationMs);
    return true;
  }

  return false;
}

// Reads one pending UDP aux packet, if any.
void handleAuxCommands() {
  int packetSize = auxUdp.parsePacket();

  if (packetSize <= 0) {
    return;
  }

  int length = auxUdp.read(auxBuffer, AUX_BUFFER_SIZE - 1);

  if (length <= 0) {
    return;
  }

  auxBuffer[length] = '\0';

  if (!parseAuxCommand(auxBuffer)) {
    Serial.println("Invalid aux command received.");
  }
}

// ============================================================
// Failsafes
// ============================================================

// Drive failsafe: stops the motors if no valid drive packet
// arrived recently. Head failsafe: releases the servo if no HEAD
// packet arrived recently.
void checkFailsafes() {
  unsigned long now = millis();

  if (now - lastPacketMs > FAILSAFE_MS) {
    stopMotors();
  }

  if (headActive && now - lastHeadPacketMs > AUX_FAILSAFE_MS) {
    servoRelease();
    headActive = false;
    Serial.println("Head command lost: servo released.");
  }
}

// ============================================================
// Network setup
// ============================================================

bool setupNetwork() {

  WiFi.mode(WIFI_STA);

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

    // Disable modem sleep for steady UDP latency (fixes bursty
    // packet timing). Do not remove.
    WiFi.setSleep(false);

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

      Serial.print("Robot IP address: ");
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

  // Name shown to OTA tools.
  ArduinoOTA.setHostname(mdnsHostname);

  // Require authentication for OTA updates.
  ArduinoOTA.setPassword(ota_password);

  ArduinoOTA.onStart([]() {

    Serial.println();
    Serial.println("OTA update starting...");

    // Never leave the robot driving (or buzzing) during a
    // firmware update.
    stopAll();

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

    // Make absolutely sure everything is stopped.
    stopAll();

  });

  ArduinoOTA.begin();

  Serial.println("OTA ready.");
  Serial.print("OTA hostname: ");
  Serial.println(mdnsHostname);
  Serial.print("OTA port: ");
  Serial.println(OTA_PORT);
}

// ============================================================
// Bench tests
// ============================================================

// Sweeps the servo, then holds center so the horn can be pulled
// off and refitted facing forward. Rover on blocks, nothing in the
// servo's path.
void runServoBenchTest() {
  Serial.println("Servo test: sweep, then hold center");

  setServoAngle(SERVO_CENTER_DEG); delay(SERVO_TEST_STEP_MS);
  setServoAngle(SERVO_MIN_DEG);    delay(SERVO_TEST_STEP_MS);
  setServoAngle(SERVO_MAX_DEG);    delay(SERVO_TEST_STEP_MS);
  setServoAngle(SERVO_CENTER_DEG);

  delay(SERVO_ALIGN_HOLD_MS);

  servoRelease();
}

void runBuzzerBenchTest() {
  Serial.println("Buzzer test: boot jingle");
  buzzerPlayJingle(BOOT_JINGLE, BOOT_JINGLE_LENGTH);
}

// ============================================================
// Setup
// ============================================================

void setup() {

  // Buzzer first, so it never chirps during boot.
  setupBuzzer();

  Serial.begin(SERIAL_BAUD_RATE);

  delay(BOOT_SETTLE_DELAY_MS);

  Serial.println();
  Serial.println("==============================");
  Serial.println("ESP32 Rover Booting");
  Serial.println("==============================");

  // Actuators and sensors. Servo and buzzer are set up before
  // stopAll() is first called.
  setupServo();
  setupMotors();
  stopAll();

  setupUltrasonic();
  setupLineAndIrSensors();

  if (!setupNetwork()) {

    Serial.println("Network setup failed.");

    stopAll();

    return;
  }

  setupOTA();

  udp.begin(UDP_PORT);
  auxUdp.begin(AUX_PORT);

  Serial.print("Listening for drive commands on port ");
  Serial.println(UDP_PORT);

  Serial.print("Listening for head/buzzer commands on port ");
  Serial.println(AUX_PORT);

  Serial.print("Broadcasting telemetry on port ");
  Serial.println(TELEMETRY_PORT);

  stopAll();

  // Jingle first so the "ready" chime is not stuck behind the
  // servo hold. Swap these two blocks to sweep first.
  if (RUN_BUZZER_BENCH_TEST) {
    runBuzzerBenchTest();
  }

  if (RUN_SERVO_BENCH_TEST) {
    runServoBenchTest();
  }

  Serial.println("Rover ready.");
}

// ============================================================
// Main loop
// ============================================================

void loop() {

  // OTA must be serviced continuously.
  ArduinoOTA.handle();

  handleDriveCommands();

  handleAuxCommands();

  checkFailsafes();

  buzzerUpdate();

  if (millis() - lastSensorReportMs >= SENSOR_REPORT_INTERVAL_MS) {
    lastSensorReportMs = millis();
    reportSensors();
  }
}