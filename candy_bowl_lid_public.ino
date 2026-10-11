/* ============================================================================
   Halloween Candy Bowl -- motion-activated lid  ->  Home Assistant (MQTT)
   Board: Seeed XIAO ESP32C6      Motor: 28BYJ-48 + ULN2003
   Sensor: HC-SR04 (on this board, or a separate MQTT sensor node)
   Audio: MAX98357A I2S amp + 28 mm speaker

   QUICK START
     1. Fill in section 1 below: your WiFi, your MQTT broker, your MQTT login.
     2. Pick the sensor mode (USE_LOCAL_SENSOR). Default: HC-SR04 wired here.
     3. Board settings + libraries as listed further down, then upload.
     4. Power up with the lid CLOSED. In Home Assistant, open the new
        "Candy Bowl" device and press Cycle. If the lid tries to go down,
        turn on "Reverse Motor" and press Cycle again.

   WHAT IT DOES
     Someone walks up (seen by the HC-SR04) -> the lid creaks open (sector gear, ~4 s), a clip plays
     ("take one!"), it holds for 5 s, then closes. Then a short cooldown so a
     kid standing there can't make it flap. Everything is tunable from HA
     without a reflash, and HA can trigger it by hand.

   ENTITIES CREATED IN HA (MQTT auto-discovery, no YAML)
     switch.candy_bowl_armed          motion triggering on/off (for setup)
     button.candy_bowl_cycle          run one open/hold/close cycle now
     switch.candy_bowl_lid            ON = open and stay open, OFF = close
     switch.candy_bowl_sound          play the clip on each cycle
     switch.candy_bowl_reverse        flip motor direction (set once at the bench)
     binary_sensor.candy_bowl_presence
     binary_sensor.candy_bowl_playing  ON while the clip is coming out of the speaker
     sensor.candy_bowl_distance       live cm reading
     sensor.candy_bowl_state          idle / opening / holding / closing / cooldown
     (board LED, GPIO 15 on the XIAO, lights while presence is ON)
     number.candy_bowl_trigger_distance   cm, default 90
     number.candy_bowl_hold_time          s,  default 5
     number.candy_bowl_cooldown           s,  default 4
     number.candy_bowl_open_angle         deg, default 65 (hardware max 70)
     number.candy_bowl_speed              motor half-steps/s, default 700

   HOMING
     There is no limit switch. Power up with the lid CLOSED. Every close
     overdrives a few degrees into the rim; the motor skips harmlessly and
     the position is re-zeroed, so it never drifts open.

   WIRING -- XIAO ESP32C6 (pin label / GPIO). No components, just modules.

        28BYJ-48 via ULN2003             XIAO ESP32C6
        --------------------             ------------
        IN1 ---------------------------- D0  (GPIO 0)
        IN2 ---------------------------- D1  (GPIO 1)
        IN3 ---------------------------- D2  (GPIO 2)
        IN4 ---------------------------- D3  (GPIO 21)
        +   ---------------------------- 5V
        -   ---------------------------- GND

        HC-SR04  (when USE_LOCAL_SENSOR = true, the default)
        VCC  --------------------------- 5V
        GND  --------------------------- GND
        TRIG --------------------------- D8  (GPIO 19)
        ECHO --------------------------- D10 (GPIO 18)   (5 V echo wired straight
                                           in: technically above the C6's 3.6 V
                                           limit, but the pin's clamp diode
                                           absorbs the weak HC-SR04 drive and it
                                           works in practice. To be in spec, use
                                           a 1k/2k divider on ECHO.)

        MAX98357A                        XIAO ESP32C6
        VIN  --------------------------- 5V
        GND  --------------------------- GND
        BCLK --------------------------- D4  (GPIO 22)
        LRC  --------------------------- D5  (GPIO 23)
        DIN  --------------------------- D9  (GPIO 20)
        SD   --------------------------- (leave open = enabled)
        GAIN --------------------------- (leave open = 9 dB; to GND = 12 dB)
        +/-  --------------------------- 28 mm speaker (4 or 8 ohm)

     Pins avoid the C6 strapping pins (4, 5, 8, 9, 15) and UART0 (16, 17).
     Power: one 5 V / 2 A USB brick into the XIAO's USB-C. Motor + amp + sensor
     all hang off the 5V pin (~400 mA peak).

   BOARD SETTINGS  (Arduino IDE)
     Board:            XIAO_ESP32C6   (NOT "ESP32C6 Dev Module" - WiFi is deaf
                                       without the XIAO's RF-switch init)
     USB CDC On Boot:  Enabled
     Partition Scheme: Huge APP (3MB No OTA)   <- room for the audio clip

   LIBRARIES (Tools -> Manage Libraries)
     AccelStepper (Mike McCauley), PubSubClient (Nick O'Leary),
     ArduinoJson 7.x. ESP_I2S ships with the ESP32 core 3.x.

   AUDIO CLIP
     candy_clip.h holds the voice line as raw audio (16 kHz, 16-bit mono).
     The included clip says "Trick or treat! Please take one only."
     To use your own: convert any WAV to a header with the same layout
     (CLIP_RATE, CLIP_SAMPLES, CLIP_PCM[]) and replace candy_clip.h.
     Keep it short; ~20 s fits in the Huge APP partition.
   ============================================================================ */

#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include <AccelStepper.h>
#include <ESP_I2S.h>
#include "candy_clip.h"

// ---------------------------------------------------------------------------
// 1. EDIT THIS BLOCK
// ---------------------------------------------------------------------------
// WiFi: your network name and password. The XIAO ESP32C6 supports 2.4 GHz
// networks (and Wi-Fi 6 on 2.4 GHz), not 5 GHz-only networks.
const char* WIFI_SSID     = "YOUR_WIFI_NAME";
const char* WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";

// MQTT broker: usually the Mosquitto add-on in Home Assistant.
//   MQTT_HOST: the IP address of your Home Assistant machine, for example
//              "192.168.1.50". Find it in HA under Settings > System > Network,
//              or in your router's device list.
//   MQTT_USER / MQTT_PASS: a Home Assistant user (or a Mosquitto login) that
//              is allowed to use MQTT. A dedicated HA user such as "mqtt"
//              works well. Settings > People > Users to create one.
const char* MQTT_HOST     = "YOUR_HA_IP_ADDRESS";
const uint16_t MQTT_PORT  = 1883;               // Mosquitto default
const char* MQTT_USER     = "YOUR_MQTT_USERNAME";
const char* MQTT_PASS     = "YOUR_MQTT_PASSWORD";

// Device name: used for the MQTT topics and HA entity ids. Lowercase and
// underscores only. Change it if you build more than one bowl.
const char* NODE_ID       = "candy_bowl";      // topics + entity ids
const char* FRIENDLY_NAME = "Candy Bowl";

// Defaults for a fresh device; all live-tunable from HA afterwards.
const int   DEFAULT_TRIGGER_CM = 90;
const int   DEFAULT_HOLD_S     = 5;
const int   DEFAULT_COOLDOWN_S = 4;
const int   DEFAULT_OPEN_DEG   = 65;    // the printed sector allows 70
const int   DEFAULT_SPEED      = 700;   // motor half-steps/s (28BYJ-48 is happy to ~900)

// Where presence comes from.
//   true  (default): HC-SR04 wired to this board (TRIG D8 / ECHO D10). Tune
//          Trigger Distance from the Candy Bowl device in HA.
//   false: use a SEPARATE presence sensor that already talks MQTT. This board
//          subscribes to that sensor's state topic and runs one cycle each
//          time its JSON field "presence" changes to "ON" (it also shows the
//          sensor's "distance" field if present). Set REMOTE_STATE_TOPIC to
//          that sensor's state topic, e.g. "porch_sensor/state". The sensor's
//          own settings decide what counts as presence.
const bool  USE_LOCAL_SENSOR   = true;
const char* REMOTE_STATE_TOPIC = "your_sensor_node/state";   // only used when USE_LOCAL_SENSOR = false
const unsigned long REMOTE_STALE_MS = 15000;                 // no message this long -> treat as clear

// ---------------------------------------------------------------------------
// 2. PINS  (XIAO ESP32C6 GPIO numbers)
// ---------------------------------------------------------------------------
const int PIN_IN1 = 0, PIN_IN2 = 1, PIN_IN3 = 2, PIN_IN4 = 21;   // D0 D1 D2 D3
const int PIN_TRIG = 19, PIN_ECHO = 18;                           // D8 D10
const int PIN_BCLK = 22, PIN_LRC = 23, PIN_DIN = 20;              // D4 D5 D9


// ---------------------------------------------------------------------------
// 3. MECHANICS + TUNING
// ---------------------------------------------------------------------------
const float MOTOR_STEPS_PER_REV = 4096.0f;   // 28BYJ-48, half-stepping
const float GEAR_RATIO          = 4.0f;      // 48-tooth sector / 12-tooth pinion
const float STEPS_PER_LID_DEG   = MOTOR_STEPS_PER_REV * GEAR_RATIO / 360.0f;  // 45.5
const int   CLOSE_OVERDRIVE_DEG = 3;         // push past closed to re-home
const int   ACCEL               = 1500;      // steps/s^2 - gentle start, no tooth skip
const int   OPEN_MAX_DEG        = 70;        // sector gear runs out of teeth past this

const unsigned long PING_INTERVAL_MS = 70;
const unsigned long ECHO_TIMEOUT_US  = 25000;
const int           SAMPLES          = 3;
const int           MIN_VALID_CM     = 3;
const int           MAX_VALID_CM     = 400;
const int           CONSEC_HITS      = 2;
const unsigned long PUBLISH_INTERVAL_MS = 2000;
const int           DISTANCE_DELTA_CM   = 5;
const unsigned long MIN_PUBLISH_GAP_MS  = 250;
const unsigned long MQTT_RETRY_MS       = 5000;

// ---------------------------------------------------------------------------
// Globals
// ---------------------------------------------------------------------------
WiFiClient   net;
PubSubClient mqtt(net);
Preferences  prefs;
I2SClass     i2s;
// HALF4WIRE with the middle pins swapped gives the 28BYJ-48 its proper
// 8-step sequence (4096 half-steps per output revolution).
AccelStepper stepper(AccelStepper::HALF4WIRE, PIN_IN1, PIN_IN3, PIN_IN2, PIN_IN4);

// settings (NVS)
int  triggerCm = DEFAULT_TRIGGER_CM;
int  holdS     = DEFAULT_HOLD_S;
int  cooldownS = DEFAULT_COOLDOWN_S;
int  openDeg   = DEFAULT_OPEN_DEG;
int  speedSps  = DEFAULT_SPEED;
bool armed     = true;
bool soundOn   = true;
bool reverse   = false;

// state machine
enum LidState { IDLE, OPENING, HOLDING, CLOSING, COOLDOWN };
LidState      state        = IDLE;
bool          manualOpen   = false;     // HA "Lid" switch ON = stay open
unsigned long stateSinceMs = 0;
float         lastDistance = -1;
bool          presence     = false;
int           consecHits   = 0;
unsigned long lastRemoteMs = 0;         // last state message from the sensor node
unsigned long lastPingMs   = 0;
unsigned long lastPublishMs = 0;
int           lastPublishedCm = -999;
unsigned long lastMqttTryMs = 0;
volatile bool audioBusy    = false;
bool          playingPublished = false;   // last 'playing' value sent to HA
TaskHandle_t  audioTask    = nullptr;

String topAvail, topState;
String topCmdArmed, topCmdCycle, topCmdLid, topCmdSound, topCmdReverse;
String topCmdTrigger, topCmdHold, topCmdCooldown, topCmdAngle, topCmdSpeed;

const char* stateName() {
  switch (state) {
    case OPENING:  return "opening";
    case HOLDING:  return "holding";
    case CLOSING:  return "closing";
    case COOLDOWN: return "cooldown";
    default:       return "idle";
  }
}

// Status LED: lit while the sensor node reports presence ON, off when it
// drops. Hard-wired to the XIAO ESP32C6's yellow user LED (GPIO 15, active
// LOW) on purpose: if the IDE is set to "ESP32C6 Dev Module" the board
// macros point at a WS2812 on GPIO 8 that the XIAO does not have, and the
// LED silently never lights. Set PIN_LED = -1 for a board without one.
const int  PIN_LED        = 15;
const bool LED_ACTIVE_LOW = true;
void setLed(bool on) {
  if (PIN_LED < 0) return;
  digitalWrite(PIN_LED, (on != LED_ACTIVE_LOW) ? HIGH : LOW);
}

// ===========================================================================
// Audio: the clip lives in flash as 16-bit PCM. A FreeRTOS task streams it
// into the I2S DMA so loop() keeps stepping the motor while it plays.
// ===========================================================================
void audioTaskFn(void*) {
  const size_t CHUNK = 512;                  // samples per write
  int16_t buf[CHUNK * 2];                    // stereo frames (L=R)
  for (uint32_t pos = 0; pos < CLIP_SAMPLES && soundOn; pos += CHUNK) {
    size_t n = min((size_t)(CLIP_SAMPLES - pos), CHUNK);
    for (size_t i = 0; i < n; i++) {
      int16_t s = (int16_t)pgm_read_word(&CLIP_PCM[pos + i]);
      buf[2 * i] = s;
      buf[2 * i + 1] = s;
    }
    i2s.write((uint8_t*)buf, n * 4);         // blocks only when DMA is full
  }
  // a little silence so the amp doesn't pop on the last sample
  memset(buf, 0, sizeof(buf));
  for (int k = 0; k < 4; k++) i2s.write((uint8_t*)buf, sizeof(buf));
  audioBusy = false;
  audioTask = nullptr;
  vTaskDelete(nullptr);
}

void playClip() {
  if (!soundOn || audioBusy) return;
  audioBusy = true;
  xTaskCreate(audioTaskFn, "audio", 4096, nullptr, 1, &audioTask);
}

// ===========================================================================
// Distance
// ===========================================================================
float pingOnce() {
  digitalWrite(PIN_TRIG, LOW);  delayMicroseconds(3);
  digitalWrite(PIN_TRIG, HIGH); delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);
  unsigned long us = pulseIn(PIN_ECHO, HIGH, ECHO_TIMEOUT_US);
  if (us == 0) return -1;
  float cm = us / 58.0f;
  if (cm < MIN_VALID_CM || cm > MAX_VALID_CM) return -1;
  return cm;
}

float measure() {
  float v[SAMPLES]; int n = 0;
  for (int i = 0; i < SAMPLES; i++) {
    float d = pingOnce();
    if (d > 0) v[n++] = d;
    delay(10);
  }
  if (n == 0) return -1;
  for (int i = 1; i < n; i++) { float k = v[i]; int j = i - 1;
    while (j >= 0 && v[j] > k) { v[j + 1] = v[j]; j--; } v[j + 1] = k; }
  return v[n / 2];
}

// ===========================================================================
// Lid motion
// ===========================================================================
long lidSteps(int deg) {
  long s = (long)(deg * STEPS_PER_LID_DEG + 0.5f);
  return reverse ? -s : s;
}

void setState(LidState s) {
  state = s;
  stateSinceMs = millis();
  publishState();
}

void startOpen() {
  stepper.enableOutputs();
  stepper.setMaxSpeed(speedSps);
  stepper.setAcceleration(ACCEL);
  stepper.moveTo(lidSteps(constrain(openDeg, 10, OPEN_MAX_DEG)));
  playClip();
  setState(OPENING);
}

void startClose() {
  stepper.setMaxSpeed(speedSps);
  stepper.moveTo(lidSteps(-CLOSE_OVERDRIVE_DEG));   // into the rim: self-homes
  setState(CLOSING);
}

void runLid() {
  unsigned long now = millis();
  switch (state) {
    case OPENING:
      if (stepper.distanceToGo() == 0) setState(HOLDING);
      break;
    case HOLDING:
      // coils stay energised so the lid holds against gravity
      if (!manualOpen && now - stateSinceMs >= (unsigned long)holdS * 1000UL) startClose();
      break;
    case CLOSING:
      if (stepper.distanceToGo() == 0) {
        stepper.setCurrentPosition(0);        // re-home on the rim
        stepper.disableOutputs();             // no heat while idle
        setState(COOLDOWN);
      }
      break;
    case COOLDOWN:
      if (now - stateSinceMs >= (unsigned long)cooldownS * 1000UL) setState(IDLE);
      break;
    default:
      break;
  }
  stepper.run();
}

// ===========================================================================
// Presence
// ===========================================================================
void updatePresence() {
  bool hit = (lastDistance > 0 && lastDistance < triggerCm);
  if (hit) { if (consecHits < CONSEC_HITS) consecHits++; } else consecHits = 0;
  bool was = presence;
  presence = (consecHits >= CONSEC_HITS);
  if (presence != was) { setLed(presence); publishState(); }

  if (presence && armed && state == IDLE) startOpen();
}

// ===========================================================================
// MQTT
// ===========================================================================
void publishState() {
  if (!mqtt.connected()) return;
  JsonDocument doc;
  doc["armed"]    = armed ? "ON" : "OFF";
  doc["lid"]      = (manualOpen || state == OPENING || state == HOLDING) ? "ON" : "OFF";
  doc["sound"]    = soundOn ? "ON" : "OFF";
  doc["reverse"]  = reverse ? "ON" : "OFF";
  doc["presence"] = presence ? "ON" : "OFF";
  doc["playing"]  = audioBusy ? "ON" : "OFF";
  playingPublished = audioBusy;
  doc["state"]    = stateName();
  int reportCm = (lastDistance < 0) ? MAX_VALID_CM : (int)(lastDistance + 0.5f);
  doc["distance"] = reportCm;
  lastPublishedCm = reportCm;
  doc["trigger"]  = triggerCm;
  doc["hold"]     = holdS;
  doc["cooldown"] = cooldownS;
  doc["angle"]    = openDeg;
  doc["speed"]    = speedSps;
  doc["rssi"]     = WiFi.RSSI();
  char buf[384];
  size_t n = serializeJson(doc, buf);
  mqtt.publish(topState.c_str(), (const uint8_t*)buf, n, true);
  lastPublishMs = millis();
}

void addDeviceBlock(JsonDocument& doc) {
  JsonObject dev = doc["device"].to<JsonObject>();
  JsonArray ids = dev["identifiers"].to<JsonArray>();
  ids.add(NODE_ID);
  dev["name"]         = FRIENDLY_NAME;
  dev["manufacturer"] = "DIY";
  dev["model"]        = "XIAO ESP32C6 + 28BYJ-48 + HC-SR04";
  dev["sw_version"]   = "1.0";
}

void publishDiscovery(const char* component, const char* objectId, const char* name, JsonDocument& doc) {
  doc["name"]      = name;
  doc["unique_id"] = String(NODE_ID) + "_" + objectId;
  if (strcmp(component, "button") != 0) doc["state_topic"] = topState;
  doc["availability_topic"] = topAvail;
  addDeviceBlock(doc);
  String topic = String("homeassistant/") + component + "/" + NODE_ID + "/" + objectId + "/config";
  char buf[768];
  size_t n = serializeJson(doc, buf);
  mqtt.publish(topic.c_str(), (const uint8_t*)buf, n, true);
}

void announceSwitch(const char* id, const char* name, const char* key, const String& cmd, const char* icon, bool config = false) {
  JsonDocument d;
  d["value_template"] = String("{{ value_json.") + key + " }}";
  d["command_topic"]  = cmd;
  d["icon"]           = icon;
  if (config) d["entity_category"] = "config";
  publishDiscovery("switch", id, name, d);
}

void announceNumber(const char* id, const char* name, const char* key, const String& cmd, int mn, int mx, int step, const char* unit) {
  JsonDocument d;
  d["value_template"] = String("{{ value_json.") + key + " }}";
  d["command_topic"]  = cmd;
  d["min"] = mn; d["max"] = mx; d["step"] = step;
  d["unit_of_measurement"] = unit;
  d["mode"] = "box";
  d["entity_category"] = "config";
  publishDiscovery("number", id, name, d);
}

void announce() {
  announceSwitch("armed",   "Armed",   "armed",   topCmdArmed,   "mdi:motion-sensor");
  announceSwitch("lid",     "Lid",     "lid",     topCmdLid,     "mdi:treasure-chest");
  announceSwitch("sound",   "Sound",   "sound",   topCmdSound,   "mdi:volume-high", true);
  announceSwitch("reverse", "Reverse Motor", "reverse", topCmdReverse, "mdi:swap-horizontal", true);
  {
    JsonDocument d;
    d["command_topic"] = topCmdCycle;
    d["payload_press"] = "GO";
    d["icon"]          = "mdi:play-circle";
    publishDiscovery("button", "cycle", "Cycle", d);
  }
  {
    JsonDocument d;
    d["value_template"] = "{{ value_json.presence }}";
    d["device_class"]   = "occupancy";
    publishDiscovery("binary_sensor", "presence", "Presence", d);
  }
  {
    JsonDocument d;
    d["value_template"] = "{{ value_json.playing }}";
    d["device_class"]   = "running";
    d["icon"]           = "mdi:speaker-wireless";
    publishDiscovery("binary_sensor", "playing", "Playing", d);
  }
  {
    JsonDocument d;
    d["value_template"]      = "{{ value_json.distance }}";
    d["unit_of_measurement"] = "cm";
    d["state_class"]         = "measurement";
    d["icon"]                = "mdi:arrow-expand-horizontal";
    publishDiscovery("sensor", "distance", "Distance", d);
  }
  {
    JsonDocument d;
    d["value_template"] = "{{ value_json.state }}";
    d["icon"]           = "mdi:state-machine";
    publishDiscovery("sensor", "state", "Lid State", d);
  }
  if (USE_LOCAL_SENSOR)
    announceNumber("trigger_distance", "Trigger Distance", "trigger",  topCmdTrigger,  10, 400, 5,  "cm");
  announceNumber("hold_time",        "Hold Time",        "hold",     topCmdHold,     1,  60,  1,  "s");
  announceNumber("cooldown",         "Cooldown",         "cooldown", topCmdCooldown, 0,  120, 1,  "s");
  announceNumber("open_angle",       "Open Angle",       "angle",    topCmdAngle,    10, OPEN_MAX_DEG, 5, "deg");
  announceNumber("speed",            "Motor Speed",      "speed",    topCmdSpeed,    200, 1000, 50, "steps/s");
}

void onMessage(char* topic, byte* payload, unsigned int len) {
  String t(topic), p;
  for (unsigned int i = 0; i < len; i++) p += (char)payload[i];
  p.trim();
  bool on = p.equalsIgnoreCase("ON");

  if (t == topCmdArmed)        { armed = on; prefs.putBool("armed", armed); }
  else if (t == topCmdSound)   { soundOn = on; prefs.putBool("sound", soundOn); }
  else if (t == topCmdReverse) { if (state == IDLE) { reverse = on; prefs.putBool("reverse", reverse); } }
  else if (t == topCmdCycle)   { if (state == IDLE || state == COOLDOWN) startOpen(); }
  else if (t == topCmdLid) {
    manualOpen = on;
    if (on && (state == IDLE || state == COOLDOWN)) startOpen();
    if (!on && state == HOLDING) startClose();
  }
  else if (t == topCmdTrigger)  { int v = p.toInt(); if (v >= 10 && v <= 400) { triggerCm = v; prefs.putInt("trigger", v); } }
  else if (t == topCmdHold)     { int v = p.toInt(); if (v >= 1 && v <= 60)   { holdS = v;     prefs.putInt("hold", v); } }
  else if (t == topCmdCooldown) { int v = p.toInt(); if (v >= 0 && v <= 120)  { cooldownS = v; prefs.putInt("cooldown", v); } }
  else if (t == topCmdAngle)    { int v = p.toInt(); if (v >= 10 && v <= OPEN_MAX_DEG) { openDeg = v; prefs.putInt("angle", v); } }
  else if (t == topCmdSpeed)    { int v = p.toInt(); if (v >= 200 && v <= 1000) { speedSps = v; prefs.putInt("speed", v); } }
  else if (!USE_LOCAL_SENSOR && t == REMOTE_STATE_TOPIC) {
    // Sensor node's state JSON: {"presence":"ON","distance":87,...}
    JsonDocument d;
    if (deserializeJson(d, p) != DeserializationError::Ok) return;
    lastRemoteMs = millis();
    if (d["distance"].is<int>()) {
      int cm = d["distance"].as<int>();
      lastDistance = (cm >= MAX_VALID_CM) ? -1 : cm;      // mirrored for the HA gauge
    }
    bool now = String(d["presence"] | "OFF").equalsIgnoreCase("ON");
    bool rising = now && !presence;
    if (now != presence) setLed(now);                      // LED follows the sensor node
    presence = now;
    if (rising && armed && state == IDLE) startOpen();     // one cycle per visit
    else publishState();
    return;
  }

  publishState();
}

const char* mqttStateText(int st) {
  switch (st) {
    case -4: return "TIMEOUT - broker did not answer. Wrong IP, or firewall.";
    case -3: return "CONNECTION_LOST";
    case -2: return "CONNECT_FAILED - no TCP route to the broker IP/port.";
    case -1: return "DISCONNECTED";
    case  1: return "BAD_PROTOCOL_VERSION";
    case  2: return "BAD_CLIENT_ID";
    case  3: return "UNAVAILABLE - broker up but refusing connections.";
    case  4: return "BAD_CREDENTIALS - MQTT_USER / MQTT_PASS rejected.";
    case  5: return "NOT_AUTHORIZED - user exists but lacks permission.";
    default: return "unknown";
  }
}

bool mqttConnect() {
  String clientId = String(NODE_ID) + "-" + String((uint32_t)ESP.getEfuseMac(), HEX);
  Serial.printf("[MQTT] connecting to %s:%u as '%s' ... ", MQTT_HOST, MQTT_PORT, MQTT_USER);
  bool ok = mqtt.connect(clientId.c_str(), MQTT_USER, MQTT_PASS, topAvail.c_str(), 0, true, "offline");
  if (!ok) {
    int st = mqtt.state();
    Serial.printf("FAILED (state %d)\n        %s\n", st, mqttStateText(st));
    return false;
  }
  Serial.println("OK");
  mqtt.publish(topAvail.c_str(), "online", true);
  const String* subs[] = { &topCmdArmed, &topCmdCycle, &topCmdLid, &topCmdSound, &topCmdReverse,
                           &topCmdTrigger, &topCmdHold, &topCmdCooldown, &topCmdAngle, &topCmdSpeed };
  for (auto s : subs) mqtt.subscribe(s->c_str());
  if (!USE_LOCAL_SENSOR) mqtt.subscribe(REMOTE_STATE_TOPIC);
  announce();
  publishState();
  Serial.println("[MQTT] connected + announced");
  return true;
}

// ===========================================================================
void setup() {
  Serial.begin(115200);
  delay(200);

  if (USE_LOCAL_SENSOR) { pinMode(PIN_TRIG, OUTPUT); pinMode(PIN_ECHO, INPUT); digitalWrite(PIN_TRIG, LOW); }
  if (PIN_LED >= 0) pinMode(PIN_LED, OUTPUT);
  setLed(false);

  stepper.setMaxSpeed(DEFAULT_SPEED);
  stepper.setAcceleration(ACCEL);
  stepper.setCurrentPosition(0);           // lid is CLOSED at power-up
  stepper.disableOutputs();

  i2s.setPins(PIN_BCLK, PIN_LRC, PIN_DIN);
  if (!i2s.begin(I2S_MODE_STD, CLIP_RATE, I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO)) {
    Serial.println("[I2S] init failed - no audio");
    soundOn = false;
  }

  prefs.begin("candybowl", false);
  triggerCm = prefs.getInt("trigger",  DEFAULT_TRIGGER_CM);
  holdS     = prefs.getInt("hold",     DEFAULT_HOLD_S);
  cooldownS = prefs.getInt("cooldown", DEFAULT_COOLDOWN_S);
  openDeg   = prefs.getInt("angle",    DEFAULT_OPEN_DEG);
  speedSps  = prefs.getInt("speed",    DEFAULT_SPEED);
  armed     = prefs.getBool("armed",   true);
  soundOn   = prefs.getBool("sound",   soundOn);
  reverse   = prefs.getBool("reverse", false);

  String base   = String(NODE_ID);
  topAvail      = base + "/availability";
  topState      = base + "/state";
  topCmdArmed   = base + "/cmd/armed";
  topCmdCycle   = base + "/cmd/cycle";
  topCmdLid     = base + "/cmd/lid";
  topCmdSound   = base + "/cmd/sound";
  topCmdReverse = base + "/cmd/reverse";
  topCmdTrigger = base + "/cmd/trigger";
  topCmdHold    = base + "/cmd/hold";
  topCmdCooldown= base + "/cmd/cooldown";
  topCmdAngle   = base + "/cmd/angle";
  topCmdSpeed   = base + "/cmd/speed";

  WiFi.mode(WIFI_STA);
  WiFi.setHostname(NODE_ID);
  WiFi.setSleep(false);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("[WiFi] connecting");
  for (int i = 0; i < 60 && WiFi.status() != WL_CONNECTED; i++) { delay(500); Serial.print("."); }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("[WiFi] IP "); Serial.println(WiFi.localIP());
    IPAddress broker;
    if (broker.fromString(MQTT_HOST)) {
      IPAddress mask = WiFi.subnetMask(), me = WiFi.localIP();
      bool same = true;
      for (int i = 0; i < 4; i++) if ((broker[i] & mask[i]) != (me[i] & mask[i])) same = false;
      if (!same) Serial.printf("  *** MQTT_HOST %s is not on this subnet - nothing will reach HA ***\n", MQTT_HOST);
    }
  } else {
    Serial.println("[WiFi] failed, will keep retrying in loop(). Lid still works standalone.");
  }

  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setCallback(onMessage);
  mqtt.setBufferSize(1024);
  mqtt.setKeepAlive(30);
  mqtt.setSocketTimeout(5);
}

void loop() {
  unsigned long now = millis();
  bool moving = (state == OPENING || state == CLOSING);

  // --- motor first, every pass ---
  runLid();

  // --- network: never block while the lid is moving ---
  if (!moving) {
    if (WiFi.status() != WL_CONNECTED) {
      static unsigned long lastWifiTry = 0;
      if (now - lastWifiTry > 10000) { lastWifiTry = now; WiFi.reconnect(); }
    } else if (!mqtt.connected()) {
      if (now - lastMqttTryMs > MQTT_RETRY_MS) { lastMqttTryMs = now; mqttConnect(); }
    }
  }
  if (mqtt.connected()) mqtt.loop();

  // --- sense: only while the motor is parked (measure() takes ~40 ms) ---
  if (USE_LOCAL_SENSOR && !moving && now - lastPingMs >= PING_INTERVAL_MS) {
    lastPingMs   = now;
    lastDistance = measure();
    updatePresence();
  }

  // --- remote watchdog: sensor node gone quiet -> clear, so the next ON re-arms ---
  if (!USE_LOCAL_SENSOR && presence && lastRemoteMs != 0 && now - lastRemoteMs > REMOTE_STALE_MS) {
    presence = false; lastDistance = -1; setLed(false); publishState();
  }

  // --- speaker: the audio task flips audioBusy; publish the edge from here
  //     (PubSubClient is not safe to call from another task) ---
  if (audioBusy != playingPublished) publishState();

  // --- heartbeat / change publish ---
  if (!moving) {
    int curCm = (lastDistance < 0) ? MAX_VALID_CM : (int)(lastDistance + 0.5f);
    bool changed = abs(curCm - lastPublishedCm) >= DISTANCE_DELTA_CM;
    if (changed && (now - lastPublishMs >= MIN_PUBLISH_GAP_MS)) publishState();
    else if (now - lastPublishMs >= PUBLISH_INTERVAL_MS) publishState();
  }
}
