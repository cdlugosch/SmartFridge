#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>
#include <Preferences.h>
#include <PubSubClient.h>
#include <DallasTemperature.h>
#include <OneWire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <stdarg.h>
#include "config.h"

// Never call Serial directly - on the C3 (USB CDC) writes block when running
// standalone without a host. Uncomment DEBUG for serial output.
//#define DEBUG
#ifdef DEBUG
  #define LOG(x) if (Serial) Serial.println(x)
  #define LOGF(...) if (Serial) Serial.printf(__VA_ARGS__)
#else
  #define LOG(x)
  #define LOGF(...)
#endif

/* =========================================================
   Fridge thermostat
   ---------------------------------------------------------
   Seeed XIAO ESP32-C3 on the XIAO Expansion Board v1.1.
   A DS18B20 on the back wall inside the fridge replaces the
   original thermostat; a 3.3 V relay module switches the
   compressor circuit where the original thermostat contact was.

   Control (cooling):  temp >= high  -> relay ON  (compressor runs)
                       temp <= low   -> relay OFF
   ========================================================= */

/* =========================================================
   User configuration
   ========================================================= */

// Pins (XIAO ESP32-C3 names, see variants/XIAO_ESP32C3/pins_arduino.h)
#define ONE_WIRE_BUS  D0    // GPIO2  - 1st Grove port (A0/D0), internal pull-up enabled - external 4.7k to 3V3 recommended
#define RELAY_PIN     D7    // GPIO20 - 2nd Grove port (UART D7/D6), free as Serial runs over USB CDC
#define BUTTON_PIN    D1    // GPIO3  - expansion board user button, active LOW
#define OLED_SDA      SDA   // GPIO6  - expansion board OLED
#define OLED_SCL      SCL   // GPIO7
#define OLED_ADDRESS  0x3C

// Relay module trigger level: HIGH for "high level trigger" modules, LOW for "low level trigger"
#define RELAY_ON_LEVEL  HIGH
#define RELAY_OFF_LEVEL (RELAY_ON_LEVEL == HIGH ? LOW : HIGH)

// Hysteresis defaults in °C - used until values are set via MQTT (then stored in flash)
const float HYST_LOW_DEFAULT  = 3.0f;
const float HYST_HIGH_DEFAULT = 6.0f;
const float HYST_MIN_ALLOWED  = -10.0f;
const float HYST_MAX_ALLOWED  = 20.0f;
const float HYST_MIN_GAP      = 0.5f;   // high must be at least this much above low

// Alarm when the temperature is this many percent outside the hysteresis limits
const float ALARM_PERCENT = 15.0f;

// Topics (built from topic_root in config.h, e.g. "esp/fridge/")
char topicTemperature[64];   // published - plain value, every 5 min
char topicRelay[64];         // published - "1"/"0", on change and every 5 min
char topicMode[64];          // published - "auto"/"manual", together with relay
char topicAlarm[64];         // published - JSON, on change and every 5 min
char topicHysteresis[64];    // published - JSON {"low":..,"high":..}, retained
char topicDeviceStatus[64];  // published - JSON, sensor errors
char topicLog[64];           // published - when debug enabled
char topicHystLowSet[64];    // subscribed - new low value
char topicHystHighSet[64];   // subscribed - new high value
char topicDebugCtrl[64];     // subscribed - "1"/"0"

/* =========================================================
   Globals
   ========================================================= */

WiFiClient espClient;
PubSubClient mqtt_client(espClient);
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature owSensors(&oneWire);
Adafruit_SSD1306 display(128, 64, &Wire, -1);
Preferences prefs;

bool debugEnabled  = false;
bool displayOk     = false;

float hystLow      = HYST_LOW_DEFAULT;
float hystHigh     = HYST_HIGH_DEFAULT;
bool  hystChanged  = false;   // set in MQTT callback, saved + published in loop

bool relayOn       = false;
bool relayChanged  = false;   // also set when the mode changes

// Manual override via the user button
bool manualMode    = false;
bool manualRelay   = false;   // requested relay state in manual mode
unsigned long manualSince = 0;

float ds18Temp         = NAN;
bool  ds18Valid        = false;
int   ds18FailCount    = 0;
const int DS18_MAX_FAILS = 3;   // consecutive failed reads before the sensor counts as broken

enum AlarmState { ALARM_NONE, ALARM_HIGH, ALARM_LOW, ALARM_SENSOR };
AlarmState alarmState    = ALARM_NONE;
AlarmState alarmPending  = ALARM_NONE;
bool       alarmChanged  = false;

unsigned long lastWifiAttempt  = 0;
unsigned long lastMqttAttempt  = 0;
unsigned long lastDs18Request  = 0;
unsigned long lastDs18Collect  = 0;
unsigned long lastRelayChange  = 0;
unsigned long alarmPendingSince = 0;
unsigned long lastMqttPublish  = 0;
unsigned long lastDisplayUpdate = 0;
unsigned long lastStatusPrint  = 0;

const unsigned long WIFI_RETRY_INTERVAL_MS   = 10000;
const unsigned long WIFI_CONNECT_TIMEOUT_MS  = 20000;
const unsigned long MQTT_RETRY_INTERVAL_MS   = 5000;
const unsigned long DS18_REQUEST_INTERVAL_MS = 10000;
const unsigned long DS18_CONVERSION_MS       = 1000;   // 12-bit worst-case ~750 ms
const unsigned long MQTT_PUSH_INTERVAL_MS    = 5UL * 60 * 1000;
const unsigned long DISPLAY_INTERVAL_MS      = 1000;
const unsigned long STATUS_PRINT_INTERVAL_MS = 30000;

// Compressor protection - avoid short cycling
const unsigned long RELAY_MIN_OFF_MS = 5UL * 60 * 1000;
const unsigned long RELAY_MIN_ON_MS  = 2UL * 60 * 1000;
const unsigned long RELAY_BOOT_DELAY_MS = 60000;          // min off time after boot

// User button: short press toggles the relay (manual mode), long press returns to auto.
// Manual mode ends automatically after MANUAL_OVERRIDE_MS.
const unsigned long BUTTON_DEBOUNCE_MS   = 50;
const unsigned long BUTTON_LONG_PRESS_MS = 2000;
const unsigned long MANUAL_OVERRIDE_MS   = 30UL * 60 * 1000;

// Temperature must stay outside the alarm limit this long before alarming
// (door openings, overshoot after the compressor stops)
const unsigned long ALARM_DELAY_MS = 10UL * 60 * 1000;

bool ds18ConversionRunning = false;

/* =========================================================
   Logging — Serial when DEBUG defined, MQTT log topic when debug enabled
   =========================================================
   Enable:  mosquitto_pub -t esp/fridge/debug -m 1
   Disable: mosquitto_pub -t esp/fridge/debug -m 0
   ========================================================= */

void log(const char* msg) {
  LOG(msg);
  if (debugEnabled && mqtt_client.connected()) {
    mqtt_client.publish(topicLog, msg);
  }
}

void logf(const char* fmt, ...) {
  char buf[128];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  log(buf);
}

/* =========================================================
   Utility helpers
   ========================================================= */

bool isValidTemperatureRange(float t) {
  return !isnan(t) && t > -30.0f && t < 50.0f;
}

void buildTopic(char* buf, size_t len, const char* suffix) {
  snprintf(buf, len, "%s%s", topic_root, suffix);
}

float alarmLimitHigh() { return hystHigh + fabsf(hystHigh) * ALARM_PERCENT / 100.0f; }
float alarmLimitLow()  { return hystLow  - fabsf(hystLow)  * ALARM_PERCENT / 100.0f; }

const char* alarmName(AlarmState s) {
  switch (s) {
    case ALARM_HIGH:   return "high";
    case ALARM_LOW:    return "low";
    case ALARM_SENSOR: return "sensor";
    default:           return "none";
  }
}

void publishDeviceStatus(const char* device, const char* statusText) {
  if (!mqtt_client.connected()) return;

  char payload[128];
  snprintf(payload, sizeof(payload),
           "{\"device\":\"%s\",\"status\":\"%s\"}",
           device, statusText);

  mqtt_client.publish(topicDeviceStatus, payload);
}

void publishHysteresis() {
  if (!mqtt_client.connected()) return;

  char payload[64];
  snprintf(payload, sizeof(payload), "{\"low\":%.1f,\"high\":%.1f}", hystLow, hystHigh);
  mqtt_client.publish(topicHysteresis, payload, true);
}

void publishRelay() {
  if (!mqtt_client.connected()) return;
  mqtt_client.publish(topicRelay, relayOn ? "1" : "0");
  mqtt_client.publish(topicMode, manualMode ? "manual" : "auto");
}

void publishAlarm() {
  if (!mqtt_client.connected()) return;

  char payload[128];
  float limit = (alarmState == ALARM_LOW) ? alarmLimitLow() : alarmLimitHigh();
  snprintf(payload, sizeof(payload),
           "{\"alarm\":%s,\"reason\":\"%s\",\"temperature\":%.2f,\"limit\":%.2f}",
           alarmState != ALARM_NONE ? "true" : "false",
           alarmName(alarmState),
           ds18Valid ? ds18Temp : 0.0f,
           alarmState == ALARM_SENSOR ? 0.0f : limit);
  mqtt_client.publish(topicAlarm, payload);
}

/* =========================================================
   Hysteresis persistence
   ========================================================= */

void loadHysteresis() {
  prefs.begin("fridge", true);
  hystLow  = prefs.getFloat("low",  HYST_LOW_DEFAULT);
  hystHigh = prefs.getFloat("high", HYST_HIGH_DEFAULT);
  prefs.end();

  if (hystHigh - hystLow < HYST_MIN_GAP) {
    hystLow  = HYST_LOW_DEFAULT;
    hystHigh = HYST_HIGH_DEFAULT;
  }
}

void saveHysteresis() {
  prefs.begin("fridge", false);
  prefs.putFloat("low",  hystLow);
  prefs.putFloat("high", hystHigh);
  prefs.end();
}

/* =========================================================
   MQTT callback
   ========================================================= */

bool parseTemperature(const char* msg, float& out) {
  char* end;
  float v = strtof(msg, &end);
  if (end == msg) return false;
  if (v < HYST_MIN_ALLOWED || v > HYST_MAX_ALLOWED) return false;
  out = v;
  return true;
}

void mqtt_callback(char* topic, byte* payload, unsigned int length) {
  // Copy topic and payload into local buffers BEFORE doing anything else —
  // PubSubClient reuses its internal buffer.
  char topicBuf[64];
  strncpy(topicBuf, topic, sizeof(topicBuf) - 1);
  topicBuf[sizeof(topicBuf) - 1] = '\0';

  char msg[32];
  if (length >= sizeof(msg)) length = sizeof(msg) - 1;
  memcpy(msg, payload, length);
  msg[length] = '\0';

  // Never call mqtt_client.publish() from inside this callback —
  // PubSubClient uses a single shared buffer for send and receive.
  // Only set flags here; publishing happens in loop(). Serial (LOG) only.
  LOGF("MQTT rx [%s]: %s\r\n", topicBuf, msg);

  float v;
  if (strcmp(topicBuf, topicHystLowSet) == 0) {
    if (parseTemperature(msg, v) && v <= hystHigh - HYST_MIN_GAP) {
      if (v != hystLow) {
        hystLow     = v;
        hystChanged = true;
      }
      LOGF("Hysteresis low set to %.1f C\r\n", hystLow);
    } else {
      LOGF("Rejected invalid hysteresis low: %s (high is %.1f)\r\n", msg, hystHigh);
    }

  } else if (strcmp(topicBuf, topicHystHighSet) == 0) {
    if (parseTemperature(msg, v) && v >= hystLow + HYST_MIN_GAP) {
      if (v != hystHigh) {
        hystHigh    = v;
        hystChanged = true;
      }
      LOGF("Hysteresis high set to %.1f C\r\n", hystHigh);
    } else {
      LOGF("Rejected invalid hysteresis high: %s (low is %.1f)\r\n", msg, hystLow);
    }

  } else if (strcmp(topicBuf, topicDebugCtrl) == 0) {
    debugEnabled = (msg[0] == '1');
    LOG(debugEnabled ? "Debug: ENABLED" : "Debug: DISABLED");

  } else {
    LOGF("Unhandled topic: %s\r\n", topicBuf);
  }
}

/* =========================================================
   Network handling
   ========================================================= */

void ensureWiFiConnected() {
  wl_status_t status = WiFi.status();
  if (status == WL_CONNECTED) return;

  // Do not restart while a connection attempt is still in progress,
  // but enforce a timeout so a stalled WL_IDLE_STATUS does not hang forever.
  unsigned long now = millis();
  if (status == WL_IDLE_STATUS && (now - lastWifiAttempt < WIFI_CONNECT_TIMEOUT_MS)) return;

  if (now - lastWifiAttempt < WIFI_RETRY_INTERVAL_MS) return;
  lastWifiAttempt = now;

  log("WiFi: connecting...");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, WiFi_password);
}

void ensureMqttConnected() {
  if (WiFi.status() != WL_CONNECTED) return;
  if (mqtt_client.connected()) return;

  unsigned long now = millis();
  if (now - lastMqttAttempt < MQTT_RETRY_INTERVAL_MS) return;
  lastMqttAttempt = now;

  log("MQTT: connecting...");

  bool ok = (strlen(mqtt_user) > 0)
    ? mqtt_client.connect(mqtt_ClientName, mqtt_user, mqtt_password)
    : mqtt_client.connect(mqtt_ClientName);

  if (ok) {
    log("MQTT: connected");

    bool subOk = mqtt_client.subscribe(topicHystLowSet);
    subOk &= mqtt_client.subscribe(topicHystHighSet);
    subOk &= mqtt_client.subscribe(topicDebugCtrl);
    if (!subOk) {
      LOG("MQTT: WARNING - one or more subscriptions failed");
    }

    publishHysteresis();
    publishRelay();
    publishAlarm();
  } else {
    logf("MQTT: connect failed, state=%d", mqtt_client.state());
  }
}

/* =========================================================
   Sensor handling
   ========================================================= */

void ds18ReadFailed(const char* reason) {
  if (ds18FailCount < DS18_MAX_FAILS) ds18FailCount++;
  if (ds18FailCount >= DS18_MAX_FAILS) ds18Valid = false;
  logf("DS18B20: %s (%d/%d)", reason, ds18FailCount, DS18_MAX_FAILS);
}

void updateDs18b20Reading() {
  unsigned long now = millis();

  if (!ds18ConversionRunning && (now - lastDs18Request >= DS18_REQUEST_INTERVAL_MS)) {
    lastDs18Request = now;

    // re-scan the bus each time, so a re-plugged sensor is found again
    owSensors.begin();
    if (owSensors.getDeviceCount() <= 0) {
      ds18ReadFailed("no sensor found");
      return;
    }

    owSensors.requestTemperatures();
    ds18ConversionRunning = true;
    lastDs18Collect = now;
    return;
  }

  if (ds18ConversionRunning && (now - lastDs18Collect >= DS18_CONVERSION_MS)) {
    ds18ConversionRunning = false;
    float t = owSensors.getTempCByIndex(0);

    if (isValidTemperatureRange(t)) {
      ds18Temp      = t;
      ds18Valid     = true;
      ds18FailCount = 0;
      logf("DS18B20: %.2f C", ds18Temp);
    } else {
      char reason[40];
      snprintf(reason, sizeof(reason), "invalid reading (%.2f)", t);
      ds18ReadFailed(reason);
    }
  }
}

/* =========================================================
   Relay (compressor) logic
   ========================================================= */

void setRelay(bool on) {
  relayOn         = on;
  relayChanged    = true;
  lastRelayChange = millis();
  digitalWrite(RELAY_PIN, on ? RELAY_ON_LEVEL : RELAY_OFF_LEVEL);
  logf("Relay: %s (%.2f C, low %.1f, high %.1f)", on ? "ON" : "OFF", ds18Temp, hystLow, hystHigh);
}

void setManualMode(bool on) {
  if (manualMode == on) return;
  manualMode   = on;
  relayChanged = true;   // publishes relay + mode
  log(on ? "Mode: MANUAL" : "Mode: AUTO");
}

void updateRelay() {
  if (manualMode && millis() - manualSince >= MANUAL_OVERRIDE_MS) {
    setManualMode(false);
  }

  bool wanted = relayOn;

  if (manualMode) {
    // manual works without a sensor too - emergency operation
    wanted = manualRelay;
    if (!wanted && relayOn) {
      setRelay(false);   // switching off by hand is always immediate
      return;
    }
  } else if (!ds18Valid) {
    // no reliable temperature -> stop the compressor (alarm is raised separately)
    if (relayOn) setRelay(false);
    return;
  } else {
    if (ds18Temp >= hystHigh) wanted = true;
    else if (ds18Temp <= hystLow) wanted = false;
  }

  if (wanted == relayOn) return;

  unsigned long minTime = relayOn ? RELAY_MIN_ON_MS : RELAY_MIN_OFF_MS;
  if (millis() - lastRelayChange < minTime) return;

  setRelay(wanted);
}

/* =========================================================
   User button (expansion board)
   ========================================================= */

void updateButton() {
  static bool          lastRaw     = HIGH;
  static bool          stable      = HIGH;
  static unsigned long lastChange  = 0;
  static unsigned long pressStart  = 0;
  static bool          longHandled = false;

  unsigned long now = millis();
  bool raw = digitalRead(BUTTON_PIN);

  if (raw != lastRaw) {
    lastRaw    = raw;
    lastChange = now;
  }

  if (now - lastChange >= BUTTON_DEBOUNCE_MS && raw != stable) {
    stable = raw;
    if (stable == LOW) {
      pressStart  = now;
      longHandled = false;
    } else if (!longHandled) {
      // short press: toggle relay, starting from the current state
      manualRelay = manualMode ? !manualRelay : !relayOn;
      manualSince = now;
      setManualMode(true);
      logf("Button: relay %s requested", manualRelay ? "ON" : "OFF");
    }
  }

  if (stable == LOW && !longHandled && now - pressStart >= BUTTON_LONG_PRESS_MS) {
    longHandled = true;
    setManualMode(false);
  }
}

/* =========================================================
   Alarm logic
   ========================================================= */

void updateAlarm() {
  unsigned long now = millis();
  AlarmState s = ALARM_NONE;

  if (ds18FailCount >= DS18_MAX_FAILS)  s = ALARM_SENSOR;
  else if (!ds18Valid)                  s = ALARM_NONE;   // no reading yet after boot
  else if (ds18Temp > alarmLimitHigh()) s = ALARM_HIGH;
  else if (ds18Temp < alarmLimitLow())  s = ALARM_LOW;

  if (s != alarmPending) {
    alarmPending      = s;
    alarmPendingSince = now;
  }

  // temperature alarms need to persist; clearing and sensor failures are immediate
  bool delayOver = (s == ALARM_NONE || s == ALARM_SENSOR)
                || (now - alarmPendingSince >= ALARM_DELAY_MS);

  if (delayOver && s != alarmState) {
    alarmState   = s;
    alarmChanged = true;
    logf("Alarm: %s", alarmName(alarmState));
  }
}

/* =========================================================
   MQTT publishing
   ========================================================= */

void publishMqttData() {
  if (!mqtt_client.connected()) return;

  // values changed in the callback or by the control logic -> publish now
  if (hystChanged) {
    hystChanged = false;
    saveHysteresis();
    publishHysteresis();
    logf("Hysteresis: low %.1f, high %.1f (saved)", hystLow, hystHigh);
  }
  if (relayChanged) {
    relayChanged = false;
    publishRelay();
  }
  if (alarmChanged) {
    alarmChanged = false;
    publishAlarm();
  }

  unsigned long now = millis();
  if (now - lastMqttPublish < MQTT_PUSH_INTERVAL_MS) return;

  // wait for the first reading after boot instead of reporting an error
  if (!ds18Valid && ds18FailCount < DS18_MAX_FAILS) return;
  lastMqttPublish = now;

  log("Publishing data...");

  if (ds18Valid) {
    char buf[16];
    dtostrf(ds18Temp, 1, 2, buf);
    mqtt_client.publish(topicTemperature, buf);
  } else {
    publishDeviceStatus("ds18b20", "error - no valid reading");
  }

  publishRelay();
  publishAlarm();
}

/* =========================================================
   Display (expansion board OLED 128x64)
   ========================================================= */

void updateDisplay() {
  if (!displayOk) return;

  unsigned long now = millis();
  if (now - lastDisplayUpdate < DISPLAY_INTERVAL_MS) return;
  lastDisplayUpdate = now;

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  // header: title + connection state
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print("Fridge");
  display.setCursor(78, 0);
  display.print(WiFi.status() == WL_CONNECTED ? "W+" : "W-");
  display.setCursor(102, 0);
  display.print(mqtt_client.connected() ? "M+" : "M-");

  // temperature, large
  display.setTextSize(3);
  display.setCursor(0, 14);
  if (ds18Valid) {
    display.print(ds18Temp, 1);
    display.setTextSize(2);
    display.write(248);   // degree sign (cp437)
    display.print("C");
  } else {
    display.print("--.-");
  }

  // relay status
  display.setTextSize(1);
  display.setCursor(0, 42);
  display.print("Relay: ");
  display.print(relayOn ? "ON  " : "OFF ");
  if (!manualMode) {
    display.print("auto");
  } else if (manualRelay != relayOn) {
    // manual ON waiting for the compressor protection time
    unsigned long left = (RELAY_MIN_OFF_MS - (millis() - lastRelayChange)) / 1000;
    display.printf(">ON %lu:%02lu", left / 60, left % 60);
  } else {
    unsigned long left = (MANUAL_OVERRIDE_MS - (millis() - manualSince)) / 60000 + 1;
    display.printf("man %lum", left);
  }

  // hysteresis or alarm
  if (alarmState != ALARM_NONE) {
    display.fillRect(0, 53, 128, 11, SSD1306_WHITE);
    display.setTextColor(SSD1306_BLACK);
    display.setCursor(2, 55);
    display.print("ALARM: ");
    display.print(alarmState == ALARM_HIGH ? "too warm"
                : alarmState == ALARM_LOW  ? "too cold" : "sensor");
  } else {
    display.setCursor(0, 55);
    display.printf("Low %.1f  High %.1f", hystLow, hystHigh);
  }

  display.display();
}

/* =========================================================
   Periodic status
   ========================================================= */

void printPeriodicStatus() {
  unsigned long now = millis();
  if (now - lastStatusPrint < STATUS_PRINT_INTERVAL_MS) return;
  lastStatusPrint = now;

  LOG("===== STATUS =====");
  LOGF("WiFi: %s\r\n",  WiFi.status() == WL_CONNECTED ? "connected" : "disconnected");
  LOGF("MQTT: %s\r\n",  mqtt_client.connected() ? "connected" : "disconnected");
  LOGF("Debug: %s\r\n", debugEnabled ? "enabled" : "disabled");
  LOGF("Mode: %s\r\n", manualMode ? "manual" : "auto");
  LOGF("Relay: %s, hysteresis low %.1f C / high %.1f C\r\n",
                relayOn ? "ON" : "OFF", hystLow, hystHigh);
  LOGF("Alarm: %s (limits %.2f / %.2f C)\r\n",
                alarmName(alarmState), alarmLimitLow(), alarmLimitHigh());
  if (ds18Valid)
    LOGF("DS18B20: %.2f C\r\n", ds18Temp);
  else
    LOG("DS18B20: no valid reading");
  LOG("==================");

  if (debugEnabled && mqtt_client.connected()) {
    char buf[200];
    snprintf(buf, sizeof(buf),
             "{\"relay\":%s,\"low\":%.1f,\"high\":%.1f,\"alarm\":\"%s\","
             "\"ds18_ok\":%s,\"ds18_temp\":%.2f}",
             relayOn ? "true" : "false",
             hystLow, hystHigh,
             alarmName(alarmState),
             ds18Valid ? "true" : "false",
             ds18Valid ? ds18Temp : 0.0f);
    mqtt_client.publish(topicLog, buf);
  }
}

/* =========================================================
   Setup / Loop
   ========================================================= */

void setup() {
#ifdef DEBUG
  Serial.begin(115200);
  delay(200);
#endif

  // relay off first - compressor stays off until a valid reading arrives
  // and RELAY_BOOT_DELAY_MS has passed. Backdate the last change so the
  // regular RELAY_MIN_OFF_MS check expires after RELAY_BOOT_DELAY_MS
  // (unsigned wrap-around is fine for the millis() differences).
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, RELAY_OFF_LEVEL);
  lastRelayChange = millis() - (RELAY_MIN_OFF_MS - RELAY_BOOT_DELAY_MS);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  log("BOOTING UP - FRIDGE THERMOSTAT");

  buildTopic(topicTemperature,  sizeof(topicTemperature),  "temperature");
  buildTopic(topicRelay,        sizeof(topicRelay),        "relay");
  buildTopic(topicMode,         sizeof(topicMode),         "mode");
  buildTopic(topicAlarm,        sizeof(topicAlarm),        "alarm");
  buildTopic(topicHysteresis,   sizeof(topicHysteresis),   "hysteresis");
  buildTopic(topicDeviceStatus, sizeof(topicDeviceStatus), "device_status");
  buildTopic(topicLog,          sizeof(topicLog),          "log");
  buildTopic(topicHystLowSet,   sizeof(topicHystLowSet),   "hysteresis/low/set");
  buildTopic(topicHystHighSet,  sizeof(topicHystHighSet),  "hysteresis/high/set");
  buildTopic(topicDebugCtrl,    sizeof(topicDebugCtrl),    "debug");

  loadHysteresis();
  logf("Hysteresis: low %.1f, high %.1f", hystLow, hystHigh);

  Wire.begin(OLED_SDA, OLED_SCL);
  displayOk = display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS);
  if (displayOk) {
    display.cp437(true);
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.print("Fridge - booting...");
    display.display();
  } else {
    log("Display: not found");
  }

  mqtt_client.setServer(mqttServer, mqttPort);
  mqtt_client.setCallback(mqtt_callback);
  mqtt_client.setBufferSize(512);  // accommodate log payloads

  // no external pull-up fitted: use the internal one (~45k). OneWire only switches
  // the pin direction afterwards, so the pull-up stays active.
  pinMode(ONE_WIRE_BUS, INPUT_PULLUP);
  owSensors.begin();
  owSensors.setWaitForConversion(false);

  // Pre-expire timers so first cycle runs immediately
  lastMqttPublish = millis() - MQTT_PUSH_INTERVAL_MS;
  lastDs18Request = millis() - DS18_REQUEST_INTERVAL_MS;
  lastStatusPrint = millis() - STATUS_PRINT_INTERVAL_MS;
}

void loop() {
  ensureWiFiConnected();
  ensureMqttConnected();

  if (mqtt_client.connected()) {
    mqtt_client.loop();
  }

  updateDs18b20Reading();
  updateButton();
  updateRelay();
  updateAlarm();
  publishMqttData();
  updateDisplay();
  printPeriodicStatus();
}
