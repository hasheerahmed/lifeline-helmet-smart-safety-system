

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <TinyGPSPlus.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <Preferences.h>

// ---------- COMPONENTS & PINS (Beetle ESP32-C6) ----------
Adafruit_MPU6050 mpu;
TinyGPSPlus gps;

#define FSR_PIN         4    // FSR ADC pin
#define ALCOHOL_ADC_PIN 5    // ADC pin for MQ-3 (ADC1_6 / D12 mapping)
#define BUZZER_PIN      7    // buzzer output (digital)
#define CANCEL_BTN      23   // cancel button (INPUT_PULLUP)
#define GPS_RX          16   // GPS TX -> ESP32 RX
#define GPS_TX          17   // GPS RX <- ESP32 TX

HardwareSerial SerialGPS(1);

// ---------- WIFI ----------
const char* ssid = "your wifi name ";
const char* password = "your wifi password";

// ---------- TextBee API (SMS) ----------
const char* apiKey = "enter your api key";
const char* deviceID = "enter your device id";
const char* recipient = "enter recipient mobile number";

WiFiClientSecure secureClient; // for HTTPS SMS

// ---------- TCP (to bike) ----------
WiFiClient bikeClient;
IPAddress bikeIP;                // resolved by mDNS
const uint16_t BIKE_PORT = 4210;

unsigned long lastBikeConnectAttempt = 0;
unsigned long bikeConnectInterval = 2000; // start 2s
const unsigned long BIKE_CONNECT_MAX = 60000; // 60s

// ---------- ACCIDENT DETECTION (tunable) ----------
float accThreshold   = 1.0f;       // g
float gyroThreshold  = 50.0f;      // deg/s
unsigned long sampleInterval = 100; // ms between IMU samples

bool accidentDetected = false;
bool alertCancelled = false;
unsigned long accidentTime = 0;
const unsigned long alertConfirmDelay = 5000; // ms before confirming accident
const unsigned long alertCooldown = 30000;    // ms lockout after alert

// cancel button debounce
unsigned long lastBtnChange = 0;
const unsigned long btnDebounce = 50;


const double FALLBACK_LAT = 12.941777777777778;  // 12°56'30.4"N
const double FALLBACK_LON = 77.56547222222223;  // 77°33'55.7"E


// ---------- FSR (helmet detection) ----------
bool helmetWorn = false;
const unsigned long fsrSampleInterval = 150; // ms (tune: 100..250)
const int fsrSamplesForAverage = 4; // smoothing avg window (tune: 3..6)
int fsrReadings[fsrSamplesForAverage];
int fsrReadIndex = 0;
int fsrCountFilled = 0;
int FSR_THRESHOLD = 250;
const int fsrConsecNeeded = 3; // consecutive readings required

// ---------- ALCOHOL (MQ-3 analog) ----------
Preferences prefs;
const char* PREF_NAMESPACE = "mq3";
const char* PREF_KEY_R0 = "R0";

const int ADC_RES_BITS = 12;
const int ADC_MAX = (1 << ADC_RES_BITS) - 1;
const float V_REF = 3.3f;
const float VCC = 3.3f;
float RL_VALUE = 10000.0f;

const float R_DIV_TOP = 10000.0;
const float R_DIV_BOTTOM = 10000.0;
const float VOLTAGE_DIVIDER_RATIO = (R_DIV_BOTTOM) / (R_DIV_TOP + R_DIV_BOTTOM);

const float EMA_ALPHA = 0.08f;
float emaRs = -1.0f;

float R0 = -1.0f;
float ALCOHOL_RATIO_THRESHOLD = 0.97f; // tune after calibration

bool alcoholDetected = false;
const unsigned long alcoholSampleInterval = 300; // ms
const int alcoholSamplesNeeded = 3; // require consecutive same state
int alcoholConsecCount = 0;
bool lastAlcoholDetect = false;

// ---------- PENDING QUEUE FLAGS ----------
bool pendingStatusToSend = false;
String pendingStatusPayload = "";
bool pendingIgnitionToSend = false;
bool currentIgnitionLocked = false; // track last-applied ignition locally

// ---------- UTIL TYPES ----------
struct SensorData { float ax, ay, az; float gx, gy, gz; };

// ---------- UTILS ----------
float magnitude(float x, float y, float z) { return sqrt(x*x + y*y + z*z); }

void sendTextBeeSMS(const String &message) {
  if (WiFi.status() != WL_CONNECTED) { Serial.println("WiFi not connected - cannot send SMS"); return; }
  secureClient.setInsecure(); // convenient for dev; replace with certs in production

  HTTPClient http;
  String url = "https://api.textbee.dev/api/v1/gateway/devices/" + String(deviceID) + "/send-sms";
  if (!http.begin(secureClient, url)) { Serial.println("HTTP begin failed"); return; }

  http.addHeader("Content-Type", "application/json");
  http.addHeader("x-api-key", apiKey);

  StaticJsonDocument<512> doc;
  JsonArray recipients = doc.createNestedArray("recipients");
  recipients.add(recipient);
  doc["message"] = message;

  String body; serializeJson(doc, body);
  int code = http.POST(body);
  if (code > 0) Serial.printf("SMS Sent! Code: %d\n", code);
  else Serial.printf("Error sending SMS: %d\n", code);
  http.end();
}

// ----------------------- TCP send helpers -----------------------
void sendToBikeRaw(const String &s) {
  if (bikeClient && bikeClient.connected()) {
    bikeClient.println(s);
    Serial.printf("[TX->BIKE] %s\n", s.c_str());
  } else {
    Serial.printf("[TX->BIKE] not connected, queueing: %s\n", s.c_str());
    // if this is STATUS or IGNITION, queue as appropriate
    if (s.startsWith("STATUS:")) {
      pendingStatusToSend = true;
      pendingStatusPayload = s.substring(7); // store JSON portion
    } else if (s.startsWith("CMD:")) {
      pendingIgnitionToSend = true;
    } else if (s == "ACCIDENT") {
      // keep a note but do not endlessly queue accidents; we'll try once when reconnected
      pendingStatusToSend = true;
      pendingStatusPayload = "{\"accident\":1}";
    }
  }
}

void flushPendingToBike() {
  if (!(bikeClient && bikeClient.connected())) return;
  if (pendingStatusToSend && pendingStatusPayload.length()) {
    String s = "STATUS:" + pendingStatusPayload;
    bikeClient.println(s);
    Serial.printf("[FLUSH] Sent pending status -> %s\n", pendingStatusPayload.c_str());
    pendingStatusToSend = false; pendingStatusPayload = "";
  }
  if (pendingIgnitionToSend) {
    String cmd = currentIgnitionLocked ? "CMD:LOCK" : "CMD:UNLOCK";
    bikeClient.println(cmd);
    Serial.printf("[FLUSH] Sent pending ignition -> %s\n", cmd.c_str());
    pendingIgnitionToSend = false;
  }
}

// Build and send STATUS JSON (same fields as before)
void sendStatus() {
  StaticJsonDocument<256> doc;
  doc["alcohol"] = alcoholDetected ? 1 : 0;
  doc["helmet_worn"] = helmetWorn ? 1 : 0;
  if (gps.location.isValid()) { doc["lat"] = gps.location.lat(); doc["lon"] = gps.location.lng(); }
  String s; serializeJson(doc, s);
  String frame = "STATUS:" + s;
  sendToBikeRaw(frame);
}

// send ignition request (applies local immediately)
void sendIgnitionCommandIfChanged(bool locked) {
  if (locked == currentIgnitionLocked) return;
  currentIgnitionLocked = locked;
  Serial.printf("[%lu] Local ignition state -> %s (queued if offline)\n", millis(), locked ? "LOCK" : "UNLOCK");
  String cmd = locked ? "CMD:LOCK" : "CMD:UNLOCK";
  sendToBikeRaw(cmd);
}

// send accident notice (and SMS)
void sendAccidentNotice() {
  sendToBikeRaw("ACCIDENT");
  // also send SMS (blocking) — you may want to do this in a state machine if blocking is a problem
  String message;
  if (gps.location.isValid()) {
    message = "🚨 Accident detected! Location: https://maps.google.com/?q=" +
              String(gps.location.lat(), 6) + "," + String(gps.location.lng(), 6);
  } else {
    // Use provided fallback coordinates when GPS is not available
    message = "🚨 Accident detected! Location: https://maps.google.com/?q=12.941778,77.565472";
  }
  sendTextBeeSMS(message);
}

// ----------------------- TCP connection & mDNS -----------------------
bool resolveBikeByMdns() {
  Serial.println("[MDNS] resolving bike-module.local ...");
  IPAddress ip = MDNS.queryHost("bike-module");
  if (ip) {
    bikeIP = ip;
    Serial.printf("[MDNS] Found bike at %s\n", bikeIP.toString().c_str());
    return true;
  }
  Serial.println("[MDNS] bike-module not found");
  return false;
}

void tryConnectToBike() {
  if (bikeClient && bikeClient.connected()) return;

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[TCP] WiFi not connected - skipping bike connect");
    return;
  }

  unsigned long now = millis();
  if (now - lastBikeConnectAttempt < bikeConnectInterval) return;
  lastBikeConnectAttempt = now;

  // resolve if unknown
  if (bikeIP[0] == 0) {
    if (!resolveBikeByMdns()) {
      bikeConnectInterval = min(BIKE_CONNECT_MAX, bikeConnectInterval * 2);
      bikeConnectInterval += random(200, 800);
      return;
    }
  }

  Serial.printf("[TCP] Connecting to bike %s:%u (heap=%u)...\n", bikeIP.toString().c_str(), BIKE_PORT, ESP.getFreeHeap());
  if (bikeClient.connect(bikeIP, BIKE_PORT, 2000)) {
    Serial.println("[TCP] Connected to bike!");
    bikeConnectInterval = 2000; // reset backoff
    flushPendingToBike();
  } else {
    Serial.println("[TCP] Connect failed");
    bikeConnectInterval = min(BIKE_CONNECT_MAX, bikeConnectInterval * 2);
    bikeConnectInterval += random(200, 800);
  }
}

// ----------------------- Handle incoming messages from Bike -----------------------
void handleBikeIncoming() {
  while (bikeClient && bikeClient.connected() && bikeClient.available()) {
    String line = bikeClient.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) continue;
    Serial.printf("[RX<-BIKE] %s\n", line.c_str());

    // ACK handling
    if (line == "ACK:LOCK") {
      currentIgnitionLocked = true;
      Serial.println("[INFO] Bike ACKed LOCK");
    } else if (line == "ACK:UNLOCK") {
      currentIgnitionLocked = false;
      Serial.println("[INFO] Bike ACKed UNLOCK");
    } else if (line == "HEARTBEAT_ACK") {
      // optional
    } else if (line.startsWith("CMD:")) {
      // Bike can instruct helmet to do actions if you want (rare). We'll accept LOCK/UNLOCK
      String cmd = line.substring(4);
      cmd.trim();
      if (cmd == "LOCK") {
        currentIgnitionLocked = true;
        Serial.println("[CMD] Bike requested LOCK (applied locally)");
        // reply ack
        bikeClient.println("ACK:LOCK");
      } else if (cmd == "UNLOCK") {
        currentIgnitionLocked = false;
        Serial.println("[CMD] Bike requested UNLOCK (applied locally)");
        bikeClient.println("ACK:UNLOCK");
      }
    }
  }
}

// ----------------------- Heartbeat -----------------------
void sendHeartbeatToBike() {
  static unsigned long lastHB = 0;
  if (!(bikeClient && bikeClient.connected())) return;
  if (millis() - lastHB > 3000) {
    bikeClient.println("HEARTBEAT");
    lastHB = millis();
  }
}

// ----------------------- MQ-3 ADC helpers -----------------------
int readRawADC() {
  return analogRead(ALCOHOL_ADC_PIN);
}
float adcRawToMeasuredV(int raw) {
  return ((float)raw / (float)ADC_MAX) * V_REF;
}
float measuredToSensorV(float measuredV) {
  if (VOLTAGE_DIVIDER_RATIO <= 0.0) return measuredV;
  return measuredV / VOLTAGE_DIVIDER_RATIO;
}
float voltageToRs(float vSensor) {
  if (vSensor <= 0.00001) return 1e9;
  return RL_VALUE * (VCC - vSensor) / vSensor;
}
float readSmoothedRs() {
  int raw = readRawADC();
  float measV = adcRawToMeasuredV(raw);
  float vSensor = measuredToSensorV(measV);
  float Rs = voltageToRs(vSensor);
  if (emaRs < 0) emaRs = Rs;
  emaRs = EMA_ALPHA * Rs + (1.0 - EMA_ALPHA) * emaRs;
  return emaRs;
}

bool loadR0(float &outR0) {
  prefs.begin(PREF_NAMESPACE, true);
  if (prefs.isKey(PREF_KEY_R0)) {
    outR0 = prefs.getFloat(PREF_KEY_R0, -1.0f);
    prefs.end();
    return (outR0 > 0.0f);
  }
  prefs.end();
  return false;
}
void saveR0(float val) {
  prefs.begin(PREF_NAMESPACE, false);
  prefs.putFloat(PREF_KEY_R0, val);
  prefs.end();
}
float calibrateR0Blocking() {
  Serial.printf("Calibrating R0 for  %d seconds. Keep sensor in clean air.\n", 30);
  unsigned long endt = millis() + 30UL * 1000UL;
  float sum = 0.0;
  int count = 0;
  while (millis() < endt) {
    float burst = 0.0;
    for (int i = 0; i < 8; ++i) {
      burst += readSmoothedRs();
      delay(60);
    }
    burst /= 8.0f;
    sum += burst;
    count++;
    Serial.print(".");
  }
  Serial.println();
  if (count == 0) return -1.0f;
  float r0 = sum / (float)count;
  saveR0(r0);
  return r0;
}

int computeADCThresholdFromRatio(float r0, float ratioThreshold) {
  if (r0 <= 0.0f) return -1;
  float Rs_thresh = r0 * ratioThreshold;
  float vSensor = VCC * RL_VALUE / (RL_VALUE + Rs_thresh);
  float measV = vSensor * VOLTAGE_DIVIDER_RATIO;
  int adc = (int)round((measV / V_REF) * ADC_MAX);
  if (adc < 0) adc = 0;
  if (adc > ADC_MAX) adc = ADC_MAX;
  return adc;
}

// ---------- Adafruit MPU wrapper ----------
SensorData readMPU() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  SensorData d;
  d.ax = a.acceleration.x / 9.80665f;
  d.ay = a.acceleration.y / 9.80665f;
  d.az = a.acceleration.z / 9.80665f;
  const float RAD2DEG = 57.29577951308232f;
  d.gx = g.gyro.x * RAD2DEG;
  d.gy = g.gyro.y * RAD2DEG;
  d.gz = g.gyro.z * RAD2DEG;
  return d;
}

// ----------------------- SETUP -----------------------
void setup() {
  Serial.begin(115200);
  delay(50);

  Wire.begin(19, 20);   // SDA=19, SCL=20
  SerialGPS.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);

  pinMode(BUZZER_PIN, OUTPUT); digitalWrite(BUZZER_PIN, LOW);
  pinMode(CANCEL_BTN, INPUT_PULLUP);

  // FSR: prime buffer
  for (int i = 0; i < fsrSamplesForAverage; i++) fsrReadings[i] = analogRead(FSR_PIN);
  fsrCountFilled = fsrSamplesForAverage; fsrReadIndex = 0;

  // Setup MQ-3 ADC
  analogReadResolution(ADC_RES_BITS);
  #if defined(ARDUINO_ARCH_ESP32)
    analogSetPinAttenuation(ALCOHOL_ADC_PIN, ADC_11db);
  #endif

  if (loadR0(R0)) {
    Serial.printf("Loaded stored R0 = %.2f ohm\n", R0);
  } else {
    Serial.println("No stored R0 found. Type 'c' in Serial Monitor to calibrate R0.");
  }

  Serial.println("Initializing MPU6050 (Adafruit)...");
  if (!mpu.begin()) { Serial.println("MPU6050 NOT FOUND - check wiring!"); while (1) delay(10); }
  Serial.println("MPU6050 connected!");

  // WiFi connect (short blocking window)
  Serial.print("Connecting to WiFi");
  WiFi.begin(ssid, password);
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 10000) { delay(200); Serial.print("."); }
  if (WiFi.status() == WL_CONNECTED) Serial.println("\nWiFi Connected"); else Serial.println("\nWiFi fail (will retry)");

  // mDNS responder (so bike can use bike-module.local)
  if (!MDNS.begin("helmet-module")) {
    Serial.println("MDNS responder start failed (helmet-module)");
  } else {
    Serial.println("mDNS responder started as helmet-module.local");
  }

  // prime IMU baseline
  readMPU();
}

// ----------------------- LOOP -----------------------
void loop() {
  // GPS feed
  while (SerialGPS.available() > 0) {
    char c = SerialGPS.read();
    gps.encode(c);
  }

  // TCP to Bike
  if (WiFi.status() == WL_CONNECTED) {
    tryConnectToBike();
    if (bikeClient && bikeClient.connected()) {
      handleBikeIncoming();
      sendHeartbeatToBike();
    }
  }

  // Serial commands for MQ-3 calibration
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'c' || c == 'C') {
      float r0new = calibrateR0Blocking();
      if (r0new > 0) {
        R0 = r0new;
        Serial.printf("Calibration saved. R0 = %.2f ohm\n", R0);
      } else {
        Serial.println("Calibration failed.");
      }
    } else if (c == 'r' || c == 'R') {
      prefs.begin(PREF_NAMESPACE, false);
      prefs.remove(PREF_KEY_R0);
      prefs.end();
      R0 = -1.0f;
      Serial.println("Stored R0 removed.");
    } else if (c == 't' || c == 'T') {
      Serial.printf("Current ALCOHOL_RATIO_THRESHOLD = %.3f\n", ALCOHOL_RATIO_THRESHOLD);
    }
  }

  // Cancel button debounce
  bool rawBtn = digitalRead(CANCEL_BTN);
  static bool lastRawBtn = HIGH;
  if (rawBtn != lastRawBtn) { lastBtnChange = millis(); lastRawBtn = rawBtn; }

  // IMU sampling non-blocking
  static unsigned long lastSample = 0;
  static SensorData prev = {0};
  unsigned long now = millis();
  if (now - lastSample >= sampleInterval) {
    lastSample = now;
    SensorData curr = readMPU();
    float magPrevAcc = magnitude(prev.ax, prev.ay, prev.az);
    float magCurrAcc = magnitude(curr.ax, curr.ay, curr.az);
    float deltaAcc = fabs(magCurrAcc - magPrevAcc);
    float magPrevGyro = magnitude(prev.gx, prev.gy, prev.gz);
    float magCurrGyro = magnitude(curr.gx, curr.gy, curr.gz);
    float deltaGyro = fabs(magCurrGyro - magPrevGyro);

    // --- ALCOHOL INFO (minimal) ---
    int rawADC = readRawADC();
    String alcInfo;
    if (R0 <= 0.0f) {
      alcInfo = String("RawADC=") + rawADC + String(" | R0 not calibrated");
    } else {
      int adcThreshold = computeADCThresholdFromRatio(R0, ALCOHOL_RATIO_THRESHOLD);
      bool isAlcohol = (rawADC >= adcThreshold);
      alcInfo = String("RawADC=") + rawADC +
                String(", ADCThreshold=") + adcThreshold +
                String(", ") + (isAlcohol ? "ALCOHOL" : "CLEAN");
    }

    Serial.printf("[%lu] deltaAcc: %.3f g, deltaGyro: %.2f deg/s  | %s\n", millis(), deltaAcc, deltaGyro, alcInfo.c_str());

    static unsigned long lastAccidentResolved = 0;
    if (!accidentDetected && (millis() - lastAccidentResolved > alertCooldown)) {
      if (deltaAcc > accThreshold && deltaGyro > gyroThreshold) {
        accidentDetected = true; alertCancelled = false; accidentTime = millis();
        digitalWrite(BUZZER_PIN, HIGH); Serial.println("⚠️ Accident detected!");
        // notify bike immediately if possible
        sendAccidentNotice();
      }
    }

    if (accidentDetected) {
      // check cancel (debounced)
      if (rawBtn == LOW && (millis() - lastBtnChange) > btnDebounce && !alertCancelled) {
        alertCancelled = true; accidentDetected = false; digitalWrite(BUZZER_PIN, LOW);
        // inform bike of cancel as a status update
        sendStatus();
        Serial.println("❌ Alert Cancelled by user");
        lastAccidentResolved = millis();
      }

      if ((millis() - accidentTime > alertConfirmDelay) && !alertCancelled) {
        digitalWrite(BUZZER_PIN, LOW); Serial.println("🚨 Accident confirmed!");
        sendAccidentNotice();
        delay(200);
        accidentDetected = false;
        lastAccidentResolved = millis();
      }
    }

    prev = curr;
  }

  // Alcohol sampling (non-blocking)
  static unsigned long lastAlcoholSample = 0;
  if (now - lastAlcoholSample >= alcoholSampleInterval) {
    lastAlcoholSample = now;
    if (R0 > 0.0f) {
      int raw = readRawADC();
      int adcThreshold = computeADCThresholdFromRatio(R0, ALCOHOL_RATIO_THRESHOLD);
      bool detected = (raw >= adcThreshold);

      if (detected == lastAlcoholDetect) alcoholConsecCount++; else { alcoholConsecCount = 1; lastAlcoholDetect = detected; }
      if (alcoholConsecCount >= alcoholSamplesNeeded) {
        if (detected != alcoholDetected) {
          alcoholDetected = detected;
          Serial.println(alcoholDetected ? ">>> Alcohol DETECTED" : ">>> Alcohol CLEARED");
          sendStatus();
          bool shouldLock = alcoholDetected || (!helmetWorn);
          sendIgnitionCommandIfChanged(shouldLock);
        }
        if (alcoholConsecCount > 1000) alcoholConsecCount = alcoholSamplesNeeded;
      }
      readSmoothedRs();
    }
  }

  // FSR sampling (helmet detection)
  static unsigned long lastFSRSample = 0;
  static int fsrRiseCount = 0;
  static int fsrFallCount = 0;
  const int FSR_CONSEC_MAX = 6;

  if (now - lastFSRSample >= fsrSampleInterval) {
    lastFSRSample = now;
    int v = analogRead(FSR_PIN); // 0..4095 on ESP32
    fsrReadings[fsrReadIndex++] = v;
    if (fsrReadIndex >= fsrSamplesForAverage) fsrReadIndex = 0;
    if (fsrCountFilled < fsrSamplesForAverage) fsrCountFilled++;

    long sum = 0;
    for (int i = 0; i < fsrCountFilled; i++) sum += fsrReadings[i];
    int avg = (int)(sum / fsrCountFilled);
    Serial.printf("[%lu] FSR avg=%d\n", millis(), avg);

    bool detect = (avg > FSR_THRESHOLD);

    if (detect) {
      if (fsrRiseCount < FSR_CONSEC_MAX) fsrRiseCount++;
      fsrFallCount = 0;
    } else {
      if (fsrFallCount < FSR_CONSEC_MAX) fsrFallCount++;
      fsrRiseCount = 0;
    }

    if (fsrRiseCount >= fsrConsecNeeded && !helmetWorn) {
      helmetWorn = true;
      Serial.printf("[%lu] Helmet WORN detected\n", millis());
      sendStatus();
      bool shouldLock = alcoholDetected || (!helmetWorn);
      sendIgnitionCommandIfChanged(shouldLock);
    } else if (fsrFallCount >= fsrConsecNeeded && helmetWorn) {
      helmetWorn = false;
      Serial.printf("[%lu] Helmet NOT worn\n", millis());
      sendStatus();
      bool shouldLock = alcoholDetected || (!helmetWorn);
      sendIgnitionCommandIfChanged(shouldLock);
    }
  }

  delay(5);
}
