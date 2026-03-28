/*
 * train_controller.ino
 * ====================
 * Harry Locomotive Project — ESP32 Arduino BLE + IR Sensor
 *
 * WHAT IT DOES:
 *   1. Connects to LionChief outer loop train via BLE
 *   2. Monitors IR beam sensors on GPIO16 (entry) and GPIO17 (exit)
 *   3. Sends STOP when inner train breaks entry beam
 *   4. Sends RESUME when inner train clears exit beam
 *
 * CONFIRMED UUIDs (Peter's train):
 *   Service UUID   : e20a39f4-73f5-4bc4-a12f-17d1ad07a961
 *   Characteristic : 08590f7e-db05-467e-8757-72f6faeb13d4
 *
 * LIBRARIES NEEDED (install via Arduino Library Manager):
 *   - "ESP32 BLE Arduino" by Neil Kolban
 *     (Source: https://github.com/nkolban/ESP32_BLE_Arduino)
 *
 * BOARD SETTINGS:
 *   Tools → Board: ESP32 Dev Module
 *   Tools → Upload Speed: 115200
 *   Tools → CPU Frequency: 240MHz
 *
 * PIN CONNECTIONS:  (see wiring diagram in docs/wiring_diagram.md)
 *   GPIO16 → IR Sensor 1 OUT (entry beam)
 *   GPIO17 → IR Sensor 2 OUT (exit beam)
 *   GPIO2  → Built-in LED (status indicator)
 *   3.3V   → IR Sensor VCC
 *   GND    → IR Sensor GND
 */

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEClient.h>
#include <BLERemoteCharacteristic.h>

// ── CONFIGURATION ─────────────────────────────────────────────────────────────
// Peter's confirmed train UUIDs
#define SERVICE_UUID        "e20a39f4-73f5-4bc4-a12f-17d1ad07a961"
#define CHARACTERISTIC_UUID "08590f7e-db05-467e-8757-72f6faeb13d4"

// ← PASTE PETER'S MAC HERE (from nRF Connect scan)
// Format: "AA:BB:CC:DD:EE:FF"
#define TRAIN_MAC_ADDRESS   "XX:XX:XX:XX:XX:XX"

// GPIO pins
#define IR_ENTRY_PIN    16    // IR Sensor 1 — inner train entering zone
#define IR_EXIT_PIN     17    // IR Sensor 2 — inner train clearing zone
#define STATUS_LED_PIN   2    // Built-in LED

// LionChief speed values
#define SPEED_STOP      0x00
#define SPEED_SLOW      0x03
#define SPEED_MEDIUM    0x07
#define SPEED_FAST      0x0F

// Safety delay after zone clears (ms)
#define RESUME_DELAY_MS 2500

// BLE reconnect interval (ms)
#define RECONNECT_INTERVAL_MS 5000

// ── BLE COMMAND BYTES ─────────────────────────────────────────────────────────
uint8_t CMD_STOP[]   = {0x00, 0x45, 0x00};
uint8_t CMD_SLOW[]   = {0x00, 0x45, 0x03};
uint8_t CMD_MEDIUM[] = {0x00, 0x45, 0x07};
uint8_t CMD_FORWARD[]= {0x00, 0x46, 0x01};
uint8_t CMD_REVERSE[]= {0x00, 0x46, 0x02};
uint8_t CMD_BELL_ON[]= {0x00, 0x47, 0x01};
uint8_t CMD_BELL_OFF[]={0x00, 0x47, 0x00};
uint8_t CMD_HORN_ON[]= {0x00, 0x48, 0x01};
uint8_t CMD_HORN_OFF[]={0x00, 0x48, 0x00};

// ── STATE ─────────────────────────────────────────────────────────────────────
BLEClient*              pClient        = nullptr;
BLERemoteCharacteristic* pCharacteristic= nullptr;
bool                    bleConnected   = false;
bool                    zoneActive     = false;
volatile bool           entryTriggered = false;
volatile bool           exitCleared    = false;
unsigned long           lastReconnect  = 0;

// ── ISR: IR SENSOR CALLBACKS (interrupt-driven — fast response) ───────────────
void IRAM_ATTR onEntryBeamBroken() {
  // IR beam is normally HIGH, goes LOW when train breaks beam
  if (digitalRead(IR_ENTRY_PIN) == LOW) {
    entryTriggered = true;
  }
}

void IRAM_ATTR onExitBeamCleared() {
  // Beam goes back HIGH when train has fully passed
  if (digitalRead(IR_EXIT_PIN) == HIGH) {
    exitCleared = true;
  }
}

// ── BLE: DISCONNECT CALLBACK ──────────────────────────────────────────────────
class ClientCallbacks : public BLEClientCallbacks {
  void onConnect(BLEClient* client) {
    bleConnected = true;
    Serial.println("[BLE] ✅ Connected to train!");
    digitalWrite(STATUS_LED_PIN, HIGH);
  }
  void onDisconnect(BLEClient* client) {
    bleConnected    = false;
    pCharacteristic = nullptr;
    Serial.println("[BLE] ⚠️  Disconnected — will retry...");
    digitalWrite(STATUS_LED_PIN, LOW);
  }
};

// ── BLE: CONNECT TO TRAIN ─────────────────────────────────────────────────────
bool connectToTrain() {
  Serial.printf("[BLE] Connecting to %s ...\n", TRAIN_MAC_ADDRESS);

  if (pClient == nullptr) {
    pClient = BLEDevice::createClient();
    pClient->setClientCallbacks(new ClientCallbacks());
  }

  BLEAddress addr(TRAIN_MAC_ADDRESS);
  if (!pClient->connect(addr)) {
    Serial.println("[BLE] ❌ Connection failed.");
    return false;
  }

  // Get the service
  BLERemoteService* pService = pClient->getService(BLEUUID(SERVICE_UUID));
  if (pService == nullptr) {
    Serial.println("[BLE] ❌ Service not found on train.");
    pClient->disconnect();
    return false;
  }

  // Get the characteristic
  pCharacteristic = pService->getCharacteristic(BLEUUID(CHARACTERISTIC_UUID));
  if (pCharacteristic == nullptr) {
    Serial.println("[BLE] ❌ Characteristic not found.");
    pClient->disconnect();
    return false;
  }

  Serial.println("[BLE] ✅ Ready to send commands.");
  return true;
}

// ── BLE: SEND COMMAND ─────────────────────────────────────────────────────────
bool sendCommand(uint8_t* cmd, size_t len) {
  if (!bleConnected || pCharacteristic == nullptr) {
    Serial.println("[BLE] Cannot send — not connected.");
    return false;
  }
  pCharacteristic->writeValue(cmd, len, false);  // false = no response needed
  Serial.printf("[BLE] Sent: [0x%02X, 0x%02X, 0x%02X]\n", cmd[0], cmd[1], cmd[2]);
  return true;
}

// ── CONVENIENCE WRAPPERS ──────────────────────────────────────────────────────
void stopTrain() {
  Serial.println("[TRAIN] 🛑 STOP");
  sendCommand(CMD_STOP, 3);
  zoneActive = true;
  // Blink LED rapidly when stopped
  for (int i = 0; i < 6; i++) {
    digitalWrite(STATUS_LED_PIN, !digitalRead(STATUS_LED_PIN));
    delay(150);
  }
  digitalWrite(STATUS_LED_PIN, bleConnected ? HIGH : LOW);
}

void resumeTrain(uint8_t speed = SPEED_MEDIUM) {
  uint8_t cmd[] = {0x00, 0x45, speed};
  Serial.printf("[TRAIN] ✅ RESUME at speed 0x%02X\n", speed);
  sendCommand(cmd, 3);
  zoneActive = false;
}

// ── SETUP ─────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println();
  Serial.println("============================================");
  Serial.println("  Harry Locomotive — Collision Prevention");
  Serial.println("============================================");

  // ── GPIO setup ──────────────────────────────────────────────────────────────
  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, LOW);

  // IR sensors — internal pull-up, beam HIGH = clear, LOW = broken
  pinMode(IR_ENTRY_PIN, INPUT_PULLUP);
  pinMode(IR_EXIT_PIN,  INPUT_PULLUP);

  // Attach interrupts — CHANGE triggers on both rising and falling edges
  attachInterrupt(digitalPinToInterrupt(IR_ENTRY_PIN), onEntryBeamBroken, CHANGE);
  attachInterrupt(digitalPinToInterrupt(IR_EXIT_PIN),  onExitBeamCleared, CHANGE);

  Serial.printf("[GPIO] IR Entry sensor on GPIO%d\n", IR_ENTRY_PIN);
  Serial.printf("[GPIO] IR Exit  sensor on GPIO%d\n", IR_EXIT_PIN);

  // ── BLE setup ────────────────────────────────────────────────────────────────
  BLEDevice::init("TrainCollisionGuard");
  Serial.println("[BLE] Initialized. Connecting to train...");

  // Attempt initial connection
  if (connectToTrain()) {
    Serial.println("[SYSTEM] ✅ Ready — monitoring for collisions.");
  } else {
    Serial.println("[SYSTEM] ⚠️  Not connected yet — will retry in loop.");
  }
}

// ── LOOP ──────────────────────────────────────────────────────────────────────
void loop() {

  // ── Auto-reconnect if BLE dropped ────────────────────────────────────────
  if (!bleConnected) {
    unsigned long now = millis();
    if (now - lastReconnect > RECONNECT_INTERVAL_MS) {
      lastReconnect = now;
      Serial.println("[BLE] Attempting reconnect...");
      connectToTrain();
    }
    delay(100);
    return;
  }

  // ── Entry beam triggered → stop outer train ───────────────────────────────
  if (entryTriggered) {
    entryTriggered = false;
    exitCleared    = false;

    if (!zoneActive) {
      Serial.println("[SENSOR] ⚠️  Entry beam broken — inner train in zone!");
      stopTrain();
    }
  }

  // ── Exit beam cleared → resume outer train ───────────────────────────────
  if (exitCleared && zoneActive) {
    exitCleared = false;
    Serial.println("[SENSOR] ✅ Exit beam restored — zone is clear!");
    Serial.printf("[SYSTEM] Waiting %dms safety delay...\n", RESUME_DELAY_MS);
    delay(RESUME_DELAY_MS);
    resumeTrain(SPEED_MEDIUM);
  }

  // ── Heartbeat blink every 2s (shows system is alive) ─────────────────────
  static unsigned long lastBlink = 0;
  if (!zoneActive && millis() - lastBlink > 2000) {
    lastBlink = millis();
    digitalWrite(STATUS_LED_PIN, HIGH);
    delay(50);
    digitalWrite(STATUS_LED_PIN, LOW);
  }

  delay(10);  // Small yield
}
