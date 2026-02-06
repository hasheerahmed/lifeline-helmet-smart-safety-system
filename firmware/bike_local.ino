/* ESP32 Bike Module — TCP Server + I2C LCD + Relay-controlled motor (ON/OFF)
   - Listens on port 4210 for helmet TCP client (helmet-module)
   - I2C SDA = GPIO21, SCL = GPIO22
   - Relay IN1 = GPIO19
*/

#include <WiFi.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <ArduinoJson.h>
#include <ESPmDNS.h>

// ---------- LCD ----------
#ifndef LCD_I2C_ADDR
#define LCD_I2C_ADDR 0x27
#endif
#define LCD_COLS 16
#define LCD_ROWS 2
LiquidCrystal_I2C lcd(LCD_I2C_ADDR, LCD_COLS, LCD_ROWS);

// ---------- WiFi ----------
const char* ssid = "wifi-name";
const char* password = "wifi-password";

// ---------- TCP Server ----------
WiFiServer server(4210);
WiFiClient clientPeer;
unsigned long lastClientActivity = 0;
const unsigned long CLIENT_TIMEOUT = 15000; // 15s idle timeout

// ---------- state ----------
String displayLine1 = "";
String displayLine2 = "";
bool ignitionLocked = false; // current ignition state on bike
String lastStatusJson = "";

// ---------- I2C pins ----------
const int I2C_SDA_PIN = 21;
const int I2C_SCL_PIN = 22;

// ---------- Relay (module) ----------
const int RELAY_IN1 = 19;     // changed from 25 to 19
const bool RELAY_ACTIVE_LOW = true; // true for most modules (LOW energizes relay)

// ---------- printing state tracking ----------
String lastPrintedIgnitionState = "";
String lastPrintedRelayState = "";
String lastPrintedStatusJson = "";
String lastEventMsg = "";

// helper to update LCD only when content changes
void updateLCDIfChanged(const String &line1, const String &line2) {
  if (line1 == displayLine1 && line2 == displayLine2) return;
  displayLine1 = line1;
  displayLine2 = line2;

  String l1 = line1;
  String l2 = line2;
  if (l1.length() > LCD_COLS) l1 = l1.substring(0, LCD_COLS);
  if (l2.length() > LCD_COLS) l2 = l2.substring(0, LCD_COLS);

  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(l1);
  lcd.setCursor(0,1);
  lcd.print(l2);
}

// helper: read relay pin and return "ON"/"OFF" based on RELAY_ACTIVE_LOW
String relayStateStr() {
  int pinVal = digitalRead(RELAY_IN1);
  bool motorOn = RELAY_ACTIVE_LOW ? (pinVal == LOW) : (pinVal == HIGH);
  return motorOn ? "ON" : "OFF";
}

// motor control via relay (ON/OFF)
void setMotor(bool on) {
  if (RELAY_ACTIVE_LOW) {
    digitalWrite(RELAY_IN1, on ? LOW : HIGH);
  } else {
    digitalWrite(RELAY_IN1, on ? HIGH : LOW);
  }
}

// summarize a status JSON-ish string into compact flags
String summarizeStatusJson(const String &json) {
  if (json.length() == 0) return "";
  String s = "";
  if (json.indexOf("alcohol") >= 0) {
    if (json.indexOf("\"alcohol\":1") >= 0 || json.indexOf("\"alcohol\": 1") >= 0) s += "ALC ";
  }
  if (json.indexOf("helmet_worn") >= 0) {
    if (json.indexOf("\"helmet_worn\":1") >= 0 || json.indexOf("\"helmet_worn\": 1") >= 0) s += "HLM ";
    else s += "HLM_OFF ";
  }
  if (json.indexOf("accident") >= 0) {
    if (json.indexOf("\"accident\":1") >= 0 || json.indexOf("\"accident\": 1") >= 0) s += "ACC ";
  }
  s.trim();
  return s;
}

// parse STATUS JSON and update ignition based ONLY on alcohol & helmet_worn
bool handleStatusJsonAndMaybeSetIgnition(const String &json) {
  if (json.length() == 0) return false;
  StaticJsonDocument<256> doc;
  DeserializationError err = deserializeJson(doc, json);
  if (err) {
    return false;
  }
  int alcohol = doc.containsKey("alcohol") ? doc["alcohol"].as<int>() : 0;
  int helmet_worn = doc.containsKey("helmet_worn") ? doc["helmet_worn"].as<int>() : 1; // default worn

  bool shouldLock = (alcohol == 1) || (helmet_worn == 0);
  if (shouldLock != ignitionLocked) {
    ignitionLocked = shouldLock;
    // apply motor state according to new ignitionLocked
    setMotor(!ignitionLocked);
    lastEventMsg = String("[IGNITION] updated from status JSON -> ") + (ignitionLocked ? "LOCK" : "UNLOCK");
    return true;
  }
  return false;
}

// centralized serial update printer — prints only when something relevant changed
void printStateIfChanged() {
  String curIgn = ignitionLocked ? "LOCK" : "UNLOCK";
  String curRelay = relayStateStr();
  String curStatus = lastStatusJson;

  bool changed = false;
  if (curIgn != lastPrintedIgnitionState) changed = true;
  if (curRelay != lastPrintedRelayState) changed = true;
  if (curStatus != lastPrintedStatusJson) changed = true;
  if (lastEventMsg.length() > 0) changed = true;

  if (!changed) return;

  if (lastEventMsg.length() > 0) {
    Serial.println(lastEventMsg);
  }

  if (curStatus.length()) {
    Serial.printf("[STATUS RX] %s\n", curStatus.c_str());
  }
  Serial.printf("[IGNITION STATE] %s\n", curIgn.c_str());
  Serial.printf("[RELAY HARDSTATE] %s (pin=%d)\n", curRelay.c_str(), digitalRead(RELAY_IN1));
  Serial.println("-------------");

  lastPrintedIgnitionState = curIgn;
  lastPrintedRelayState = curRelay;
  lastPrintedStatusJson = curStatus;
  lastEventMsg = "";
}

// process incoming line (single framed message) from helmet
void processIncomingLine(const String &line) {
  if (line.length() == 0) return;
  Serial.printf("[RX<-HELMET] %s\n", line.c_str());

  if (line.startsWith("STATUS:")) {
    String json = line.substring(7);
    lastStatusJson = json;
    bool ignChanged = handleStatusJsonAndMaybeSetIgnition(json);

    // Build AD|HND style flags and update LCD (same as before)
    StaticJsonDocument<128> d;
    DeserializationError derr = deserializeJson(d, json);
    int alc  = 0;
    int helm = 1;
    if (!derr) {
      alc  = d.containsKey("alcohol")     ? d["alcohol"].as<int>()     : 0;
      helm = d.containsKey("helmet_worn") ? d["helmet_worn"].as<int>() : 1;
    }
    String alcFlag  = (alc  == 1) ? "AD"  : "AND";
    String helmFlag = (helm == 1) ? "HD"  : "HND";
    String line1 = "STATUS: " + alcFlag + "|" + helmFlag;
    if (line1.length() > LCD_COLS) line1 = line1.substring(0, LCD_COLS);
    String ignLine = ignitionLocked ? "IGN: LOCKED" : "IGN: UNLOCK";
    updateLCDIfChanged(line1, ignLine);

    if (ignChanged && lastEventMsg.length() == 0) {
      // message already set in handleStatus...
    }
    printStateIfChanged();

  } else if (line.startsWith("CMD:")) {
    String cmd = line.substring(4);
    cmd.trim();
    if (cmd == "LOCK") {
      ignitionLocked = true;
      setMotor(false);
      lastEventMsg = "[IGNITION] Received LOCK -> motor OFF";
      // reply ack to helmet
      if (clientPeer && clientPeer.connected()) clientPeer.println("ACK:LOCK");
    } else if (cmd == "UNLOCK") {
      ignitionLocked = false;
      setMotor(true);
      lastEventMsg = "[IGNITION] Received UNLOCK -> motor ON";
      if (clientPeer && clientPeer.connected()) clientPeer.println("ACK:UNLOCK");
    }
    updateLCDIfChanged(displayLine1.length() ? displayLine1 : "Waiting...", ignitionLocked ? "IGN: LOCKED" : "IGN: UNLOCK");
    printStateIfChanged();

  } else if (line == "ACCIDENT") {
    // show accident prominently on LCD but do NOT change ignition/motor
    updateLCDIfChanged("! ACCIDENT DETECTED !", (ignitionLocked ? "IGN: LOCKED" : "IGN: UNLOCK"));
    lastEventMsg = "[ACCIDENT] displayed (from helmet)";
    printStateIfChanged();

  } else if (line == "HEARTBEAT") {
    // reply heartbeat ack
    if (clientPeer && clientPeer.connected()) clientPeer.println("HEARTBEAT_ACK");
  } else {
    // Other messages — display on LCD second line
    updateLCDIfChanged(line, (ignitionLocked ? "IGN: LOCKED" : "IGN: UNLOCK"));
    printStateIfChanged();
  }
}

// Accept new client or read incoming data from connected client
void handleTcpServer() {
  // Accept new client if none connected
  if (!clientPeer || !clientPeer.connected()) {
    WiFiClient newClient = server.available();
    if (newClient) {
      Serial.println("[TCP] New helmet connected");
      clientPeer = newClient;
      clientPeer.setNoDelay(true);
      lastClientActivity = millis();
    }
  } else {
    // Read available data
    while (clientPeer.available()) {
      String line = clientPeer.readStringUntil('\n');
      line.trim();
      if (line.length() == 0) continue;
      processIncomingLine(line);
      lastClientActivity = millis();
    }

    // Disconnect idle client
    if (millis() - lastClientActivity > CLIENT_TIMEOUT) {
      Serial.println("[TCP] Client idle - disconnecting");
      clientPeer.stop();
    }
    if (!clientPeer.connected()) clientPeer.stop();
  }
}

void setup() {
  Serial.begin(115200);
  delay(50);

  // I2C explicit pins
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(100000); // force 100 kHz I2C for stability

  // LCD init
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Booting...");
  lcd.setCursor(0,1);
  lcd.print("Connecting WiFi");

  // WiFi attempt (short blocking window)
  WiFi.begin(ssid, password);
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 8000) {
    delay(200);
    Serial.print(".");
  }

  lcd.clear();
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi connected");
    lcd.print("WiFi Connected");
    lcd.setCursor(0,1);
    lcd.print(WiFi.localIP().toString());
    delay(700);
  } else {
    Serial.println("\nWiFi not connected (will retry)");
    lcd.print("WiFi not ready");
    delay(600);
  }

  // start mDNS responder (bike-module.local)
  if (!MDNS.begin("bike-module")) {
    Serial.println("mDNS responder start failed (bike-module)");
  } else {
    Serial.println("mDNS: bike-module.local");
    MDNS.addService("helmettcp", "tcp", 4210);
  }

  // start TCP server
  server.begin();
  server.setNoDelay(true);
  Serial.println("[TCP] Server started on port 4210");

  // Relay pin setup
  pinMode(RELAY_IN1, OUTPUT);
  // ensure motor is OFF at boot
  if (RELAY_ACTIVE_LOW) digitalWrite(RELAY_IN1, HIGH);
  else digitalWrite(RELAY_IN1, LOW);

  // initial LCD
  lcd.clear();
  lcd.print("Waiting for Data");
  lcd.setCursor(0,1);
  lcd.print("IGN: UNLOCK");

  // print initial module state once
  lastStatusJson = "";
  lastEventMsg = "[INIT] Initial module state";
  printStateIfChanged();
}

void loop() {
  handleTcpServer();

  // keep loop responsive
  delay(10);
}
