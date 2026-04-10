/*
 * train_controller.ino
 * ====================
 * Harry Locomotive Project — ESP32 BLE + IR Sensor Collision Prevention
 *
 * WHAT IT DOES:
 *   1. Connects to LionChief train via BLE (direct MAC + name fallback)
 *   2. Monitors IR beam sensors on GPIO16 (entry) and GPIO17 (exit)
 *   3. When inner train breaks entry beam → sends STOP to outer train
 *   4. When inner train clears exit beam  → waits 2.5s → sends RESUME
 *   5. LED on GPIO2 shows system status
 *
 * CONNECTION:
 *   Tries direct MAC address first (TARGET_MAC below).
 *   Falls back to scanning by name prefix "LC0" if MAC fails.
 *
 * CONFIRMED UUIDS (Peter's train):
 *   Service UUID   : e20a39f4-73f5-4bc4-a12f-17d1ad07a961
 *   Characteristic : 08590f7e-db05-467e-8757-72f6faeb13d4
 *
 * WIRING:
 *   GPIO16 → E18-D80NK Sensor 1 — Black wire (entry at Point A)
 *   GPIO17 → E18-D80NK Sensor 2 — Black wire (exit at Point B)
 *   VIN    → Both sensors — Brown wire (5V power)
 *   GND    → Both sensors — Blue wire (ground)
 *   GPIO2  → Built-in LED (status indicator)
 *
 * LED STATUS CODES:
 *   Slow blink (2s)  → Connected, monitoring
 *   Solid ON         → Connected
 *   Off              → Not connected
 *   Fast blink (6x)  → Train stopped (zone active)
 *
 * LIBRARIES (install via PlatformIO or Arduino Library Manager):
 *   ESP32 BLE Arduino by Neil Kolban
 *
 * BOARD SETTINGS (Arduino IDE):
 *   Tools → Board: ESP32 Dev Module
 *   Tools → Upload Speed: 115200
 *   Tools → CPU Frequency: 240MHz
 *   Tools → Partition Scheme: Default
 */

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEClient.h>
#include <BLERemoteCharacteristic.h>

// ── CONFIGURATION ─────────────────────────────────────────────────
// Peter's confirmed train MAC address
#define TARGET_MAC          "CC:01:78:D0:F0:99"

// Fallback: scan for device whose name starts with this prefix
#define TRAIN_NAME_PREFIX   "LC0"

// LionChief BLE UUIDs — confirmed via nRF Connect
#define SERVICE_UUID        "e20a39f4-73f5-4bc4-a12f-17d1ad07a961"
#define CHARACTERISTIC_UUID "08590f7e-db05-467e-8757-72f6faeb13d4"

// GPIO pin assignments
#define IR_ENTRY_PIN    16   // Sensor at Point A — inner train entering zone
#define IR_EXIT_PIN     17   // Sensor at Point B — inner train leaving zone
#define STATUS_LED      2    // Built-in LED

// Train speed for resuming after collision prevention (0x00=stop, 0x07=medium)
#define RESUME_SPEED    0x07

// Safety delay after zone clears before resuming (milliseconds)
#define RESUME_DELAY_MS 2500

// BLE reconnect interval (milliseconds)
#define RECONNECT_MS    5000

// ── BLE COMMANDS ──────────────────────────────────────────────────
uint8_t CMD_STOP[]    = {0x00, 0x45, 0x00};
uint8_t CMD_RESUME[]  = {0x00, 0x45, RESUME_SPEED};
uint8_t CMD_HORN_ON[] = {0x00, 0x48, 0x01};
uint8_t CMD_BELL_ON[] = {0x00, 0x47, 0x01};
uint8_t CMD_BELL_OFF[]= {0x00, 0x47, 0x00};

// ── STATE ─────────────────────────────────────────────────────────
BLEClient*               pClient    = nullptr;
BLERemoteCharacteristic* pChar      = nullptr;
bool                     bleConnected   = false;
bool                     zoneActive     = false;
volatile bool            entryTriggered = false;
volatile bool            exitTriggered  = false;
unsigned long            lastReconnect  = 0;
String                   foundMAC       = "";

// ── ISR: ENTRY SENSOR ─────────────────────────────────────────────
// E18-D80NK is NPN — output is LOW when beam is broken by a train
void IRAM_ATTR onEntryChange() {
  if (digitalRead(IR_ENTRY_PIN) == LOW) {
    // Beam broken = train entering zone
    entryTriggered = true;
  }
}

// ── ISR: EXIT SENSOR ──────────────────────────────────────────────
void IRAM_ATTR onExitChange() {
  if (digitalRead(IR_EXIT_PIN) == HIGH) {
    // Beam restored = train has fully exited zone
    exitTriggered = true;
  }
}

// ── BLE: CLIENT CALLBACKS ─────────────────────────────────────────
class TrainClientCallbacks : public BLEClientCallbacks {
  void onConnect(BLEClient* client) {
    bleConnected = true;
    digitalWrite(STATUS_LED, HIGH);
    Serial.println("[BLE] ✅ Connected to train!");
  }
  void onDisconnect(BLEClient* client) {
    bleConnected = false;
    pChar        = nullptr;
    digitalWrite(STATUS_LED, LOW);
    Serial.println("[BLE] ⚠️  Disconnected — will retry...");
  }
};

// ── BLE: SEND COMMAND ─────────────────────────────────────────────
bool sendCommand(uint8_t* cmd, size_t len, const char* label) {
  if (!bleConnected || pChar == nullptr) {
    Serial.printf("[BLE] Cannot send %s — not connected\n", label);
    return false;
  }
  pChar->writeValue(cmd, len, false);
  Serial.printf("[BLE] ▶ %s  [0x%02X, 0x%02X, 0x%02X]\n",
                label, cmd[0], cmd[1], cmd[2]);
  return true;
}

// ── BLE: CONNECT BY MAC ───────────────────────────────────────────
bool connectToTrain(const char* mac) {
  Serial.printf("[BLE] Connecting to %s ...\n", mac);

  if (pClient == nullptr) {
    pClient = BLEDevice::createClient();
    pClient->setClientCallbacks(new TrainClientCallbacks());
  }

  if (!pClient->connect(BLEAddress(mac))) {
    Serial.println("[BLE] ❌ Connection failed.");
    return false;
  }

  BLERemoteService* pService = pClient->getService(BLEUUID(SERVICE_UUID));
  if (pService == nullptr) {
    Serial.println("[BLE] ❌ LionChief service not found.");
    pClient->disconnect();
    return false;
  }

  pChar = pService->getCharacteristic(BLEUUID(CHARACTERISTIC_UUID));
  if (pChar == nullptr) {
    Serial.println("[BLE] ❌ Write characteristic not found.");
    pClient->disconnect();
    return false;
  }

  return true;  // onConnect callback sets bleConnected = true
}

// ── BLE: SCAN CALLBACK (name-prefix fallback) ─────────────────────
class ScanCallback : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice device) {
    String name = device.getName().c_str();
    String addr = device.getAddress().toString().c_str();
    if (name.length() > 0) {
      Serial.println("  Scan: " + name + " [" + addr + "]");
    }
    if (name.startsWith(TRAIN_NAME_PREFIX)) {
      Serial.println("  >>> Found by name: " + name);
      foundMAC = addr;
      BLEDevice::getScan()->stop();
    }
  }
};

// ── TRAIN STOP ────────────────────────────────────────────────────
void stopTrain() {
  Serial.println("[TRAIN] 🛑 STOP — inner train in zone!");
  sendCommand(CMD_STOP, 3, "STOP");
  zoneActive = true;

  // Fast blink LED to indicate stopped state
  for (int i = 0; i < 6; i++) {
    digitalWrite(STATUS_LED, !digitalRead(STATUS_LED));
    delay(120);
  }
  digitalWrite(STATUS_LED, bleConnected ? HIGH : LOW);
}

// ── TRAIN RESUME ──────────────────────────────────────────────────
void resumeTrain() {
  Serial.println("[TRAIN] ✅ RESUME — zone cleared!");
  sendCommand(CMD_RESUME, 3, "RESUME");
  zoneActive = false;
}

// ── SETUP ─────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(600);

  Serial.println("\n================================================");
  Serial.println("  Harry Locomotive — Collision Prevention");
  Serial.printf( "  Target MAC: %s\n", TARGET_MAC);
  Serial.println("================================================\n");

  // ── GPIO setup ──────────────────────────────────────────────────
  pinMode(STATUS_LED, OUTPUT);
  digitalWrite(STATUS_LED, LOW);

  // IR sensors: NPN type, beam HIGH=clear, LOW=broken
  // INPUT_PULLUP keeps pin HIGH when nothing connected
  pinMode(IR_ENTRY_PIN, INPUT_PULLUP);
  pinMode(IR_EXIT_PIN,  INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(IR_ENTRY_PIN), onEntryChange, CHANGE);
  attachInterrupt(digitalPinToInterrupt(IR_EXIT_PIN),  onExitChange,  CHANGE);

  Serial.printf("[GPIO] Entry sensor: GPIO%d\n", IR_ENTRY_PIN);
  Serial.printf("[GPIO] Exit sensor : GPIO%d\n", IR_EXIT_PIN);
  Serial.printf("[GPIO] LED         : GPIO%d\n", STATUS_LED);

  // ── BLE setup ───────────────────────────────────────────────────
  BLEDevice::init("TrainCollisionGuard");

  // Step 1: Direct MAC connection
  Serial.println("\n[BLE] Step 1: Connecting by MAC address...");
  bool ok = connectToTrain(TARGET_MAC);

  // Step 2: Name-scan fallback if MAC fails
  if (!ok) {
    Serial.println("[BLE] Step 2: MAC failed — scanning by name...");
    BLEScan* pScan = BLEDevice::getScan();
    pScan->setAdvertisedDeviceCallbacks(new ScanCallback());
    pScan->setActiveScan(true);
    pScan->start(15, false);

    if (foundMAC.length() > 0) {
      ok = connectToTrain(foundMAC.c_str());
    }
  }

  if (ok) {
    Serial.println("\n[SYSTEM] ✅ Ready — monitoring for collisions.\n");
  } else {
    Serial.println("\n[SYSTEM] ⚠️  Not connected — will retry in loop.\n");
  }
}

// ── LOOP ──────────────────────────────────────────────────────────
void loop() {

  // ── Auto-reconnect if BLE dropped ───────────────────────────────
  if (!bleConnected) {
    unsigned long now = millis();
    if (now - lastReconnect > RECONNECT_MS) {
      lastReconnect = now;
      Serial.println("[BLE] Attempting reconnect...");

      bool ok = connectToTrain(TARGET_MAC);
      if (!ok && foundMAC.length() > 0) {
        ok = connectToTrain(foundMAC.c_str());
      }
      if (!ok) {
        // Scan for train again
        BLEScan* pScan = BLEDevice::getScan();
        pScan->setAdvertisedDeviceCallbacks(new ScanCallback());
        pScan->setActiveScan(true);
        pScan->start(8, false);
        if (foundMAC.length() > 0) {
          connectToTrain(foundMAC.c_str());
        }
      }
    }
    delay(100);
    return;
  }

  // ── Entry beam broken → stop outer train ────────────────────────
  if (entryTriggered) {
    entryTriggered = false;
    exitTriggered  = false;   // reset exit flag too

    if (!zoneActive) {
      Serial.println("[SENSOR] Entry beam broken — inner train entering zone");
      stopTrain();
    }
  }

  // ── Exit beam cleared → resume outer train ──────────────────────
  if (exitTriggered && zoneActive) {
    exitTriggered = false;
    Serial.println("[SENSOR] Exit beam cleared — inner train left zone");
    Serial.printf("[SYSTEM] Waiting %dms safety delay...\n", RESUME_DELAY_MS);
    delay(RESUME_DELAY_MS);
    resumeTrain();
  }

  // ── Heartbeat LED — slow blink every 2s shows system is alive ───
  static unsigned long lastBlink = 0;
  if (!zoneActive && (millis() - lastBlink > 2000)) {
    lastBlink = millis();
    digitalWrite(STATUS_LED, HIGH);
    delay(40);
    digitalWrite(STATUS_LED, LOW);
  }

  delay(10);
}
