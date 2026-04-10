/*
 * ══════════════════════════════════════════════════════════════════
 *  train_controller/src/main.cpp
 *  Harry Locomotive Project — Collision Prevention System v3
 *
 *  SYSTEM OVERVIEW:
 *    3 IR sensors + 1 relay + BLE → full collision prevention
 *    No laptop needed — ESP32 runs standalone
 *
 *  TWO TRAINS:
 *    Outer train → LionChief BLE (CC:01:78:D0:F0:99)
 *                  Controlled via Bluetooth commands
 *    Inner train → Track power only (no BLE)
 *                  Controlled by relay cutting track power
 *
 *  THREE IR SENSORS:
 *    Sensor A (GPIO16) → Inner loop track ONLY
 *                        Detects inner train ENTERING shared section
 *    Sensor B (GPIO15) → Outer loop track ONLY
 *                        Detects outer train ENTERING shared section
 *    Sensor C (GPIO17) → Exit of shared section
 *                        Detects ANY train LEAVING shared section
 *
 *  RELAY (GPIO18):
 *    Controls inner train track power
 *    HIGH → NC closed → power flows  → inner train runs
 *    LOW  → NC open   → power cut    → inner train stops
 *    (Standard Arduino relay module — active LOW)
 *
 *  COLLISION LOGIC:
 *    Case 1 — Inner train enters first (Sensor A fires):
 *      Zone locked → BLE STOP outer train → inner exits Sensor C
 *      → 2.5s delay → BLE RESUME outer → Zone unlocked
 *
 *    Case 2 — Outer train enters first (Sensor B fires):
 *      Zone locked → Relay cuts inner power → outer exits Sensor C
 *      → 2.5s delay → Relay restores inner power → Zone unlocked
 *
 *  SERIAL MONITOR COMMANDS (for manual testing):
 *    s = STOP outer         f = FORWARD outer
 *    r = REVERSE outer      e = EMERGENCY STOP
 *    1-7 = Set speed        + = Speed UP
 *    - = Speed DOWN         h = Horn toggle
 *    b = Bell toggle        l = Lights toggle
 *    n = Sound ON           m = Sound OFF
 *    a = Announce/Speech    v = Volume cycle
 *    x = Relay ON (inner)   z = Relay OFF (inner)
 *    i = Status             ? = Show menu
 *
 *  CONFIRMED TRAIN BLE DETAILS:
 *    Device Name    : LC015556-99F0
 *    MAC Address    : CC:01:78:D0:F0:99
 *    Service UUID   : e20a39f4-73f5-4bc4-a12f-17d1ad07a961
 *    Characteristic : 08590f7e-db05-467e-8757-72f6faeb13d4
 *
 *  WIRING:
 *    GPIO16 → Sensor A black wire  (inner loop entry)
 *    GPIO15 → Sensor B black wire  (outer loop entry)
 *    GPIO17 → Sensor C black wire  (shared section exit)
 *    GPIO18 → Relay module IN pin  (inner train power)
 *    VIN    → All sensors + relay VCC/brown wire (5V)
 *    GND    → All sensors + relay GND/blue wire
 *    GPIO2  → Built-in LED
 *
 *  LED STATUS:
 *    Slow blink 2s   → Running normally, zone clear
 *    Solid ON        → BLE connected
 *    OFF             → BLE disconnected
 *    4x fast blink   → Zone active (train stopped)
 * ══════════════════════════════════════════════════════════════════
 */

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEClient.h>
#include <BLERemoteCharacteristic.h>

// ══════════════════════════════════════════════════════════════════
//  CONFIGURATION
// ══════════════════════════════════════════════════════════════════

#define TARGET_MAC          "CC:01:78:D0:F0:99"   // Peter's confirmed MAC
#define TRAIN_NAME_PREFIX   "LC0"                  // fallback name scan
#define SERVICE_UUID        "e20a39f4-73f5-4bc4-a12f-17d1ad07a961"
#define CHARACTERISTIC_UUID "08590f7e-db05-467e-8757-72f6faeb13d4"

// GPIO pins
#define SENSOR_A_PIN    16   // Inner loop entry (inner train only)
#define SENSOR_B_PIN    15   // Outer loop entry (outer train only)
#define SENSOR_C_PIN    17   // Shared section exit (any train)
#define RELAY_PIN       18   // Inner train track power relay
#define STATUS_LED       2   // Built-in LED

// Relay states (active LOW relay module with NC terminal)
#define RELAY_INNER_ON  HIGH   // Power ON  → inner train runs
#define RELAY_INNER_OFF LOW    // Power OFF → inner train stops

// Timing
#define RESUME_DELAY_MS     2500   // Safety delay after zone clears
#define STOP_REPEAT_MS       500   // Repeat STOP signal every 500ms
#define RECONNECT_MS        5000   // BLE reconnect interval
#define SPEED_RAMP_DELAY_MS 3000   // Time at slow speed before ramping

// ══════════════════════════════════════════════════════════════════
//  ALL LIONCHIEF BLE COMMAND BYTES
// ══════════════════════════════════════════════════════════════════

// Speed (0x00 = stop, 0x01-0x07 = speed levels, 0x07 = medium)
uint8_t CMD_STOP[]      = {0x00, 0x45, 0x00};
uint8_t CMD_SPEED_1[]   = {0x00, 0x45, 0x01};
uint8_t CMD_SPEED_2[]   = {0x00, 0x45, 0x02};
uint8_t CMD_SPEED_3[]   = {0x00, 0x45, 0x03};
uint8_t CMD_SPEED_4[]   = {0x00, 0x45, 0x04};
uint8_t CMD_SPEED_5[]   = {0x00, 0x45, 0x05};
uint8_t CMD_SPEED_6[]   = {0x00, 0x45, 0x06};
uint8_t CMD_SPEED_7[]   = {0x00, 0x45, 0x07};   // medium-fast

// Direction
uint8_t CMD_FORWARD[]   = {0x00, 0x46, 0x01};
uint8_t CMD_REVERSE[]   = {0x00, 0x46, 0x02};

// Horn
uint8_t CMD_HORN_ON[]   = {0x00, 0x48, 0x01};
uint8_t CMD_HORN_OFF[]  = {0x00, 0x48, 0x00};

// Bell
uint8_t CMD_BELL_ON[]   = {0x00, 0x47, 0x01};
uint8_t CMD_BELL_OFF[]  = {0x00, 0x47, 0x00};

// Lights
uint8_t CMD_LIGHT_ON[]  = {0x00, 0x51, 0x01};
uint8_t CMD_LIGHT_OFF[] = {0x00, 0x51, 0x00};

// Sound / volume
uint8_t CMD_SOUND_ON[]  = {0x00, 0x4C, 0x07};   // volume max
uint8_t CMD_SOUND_OFF[] = {0x00, 0x4C, 0x00};   // volume zero
uint8_t CMD_VOL_1[]     = {0x00, 0x4C, 0x01};
uint8_t CMD_VOL_3[]     = {0x00, 0x4C, 0x03};
uint8_t CMD_VOL_5[]     = {0x00, 0x4C, 0x05};
uint8_t CMD_VOL_7[]     = {0x00, 0x4C, 0x07};

// Announce / conductor speech
uint8_t CMD_ANNOUNCE[]  = {0x00, 0x4D, 0x00, 0x00};   // random phrase

// Disconnect gracefully
uint8_t CMD_DISCONNECT[] = {0x00, 0x4B, 0x00, 0x00};

// ══════════════════════════════════════════════════════════════════
//  STATE VARIABLES
// ══════════════════════════════════════════════════════════════════

// BLE
BLEClient*               pClient      = nullptr;
BLERemoteCharacteristic* pChar        = nullptr;
bool                     bleConnected = false;
String                   foundMAC     = "";
unsigned long            lastReconnect = 0;

// Sensor ISR flags
volatile bool sensorA_fired = false;
volatile bool sensorB_fired = false;
volatile bool sensorC_fired = false;

// Zone state
bool zoneLocked       = false;
bool innerTrainInZone = false;
bool outerTrainInZone = false;
bool waitingToResume  = false;

// Timing
unsigned long resumeTimer  = 0;
unsigned long lastStopSent = 0;

// Manual control state
bool hornActive   = false;
bool bellActive   = false;
bool lightsActive = false;
int  currentSpeed = 0;
int  volumeLevel  = 7;

// ══════════════════════════════════════════════════════════════════
//  INTERRUPT SERVICE ROUTINES
//  E18-D80NK output = LOW when train detected
// ══════════════════════════════════════════════════════════════════

void IRAM_ATTR ISR_SensorA() {
  if (digitalRead(SENSOR_A_PIN) == LOW) sensorA_fired = true;
}

void IRAM_ATTR ISR_SensorB() {
  if (digitalRead(SENSOR_B_PIN) == LOW) sensorB_fired = true;
}

void IRAM_ATTR ISR_SensorC() {
  if (digitalRead(SENSOR_C_PIN) == LOW) sensorC_fired = true;
}

// ══════════════════════════════════════════════════════════════════
//  BLE CALLBACKS
// ══════════════════════════════════════════════════════════════════

class TrainCallbacks : public BLEClientCallbacks {
  void onConnect(BLEClient* c) {
    bleConnected = true;
    digitalWrite(STATUS_LED, HIGH);
    Serial.println("[BLE] ✅ Connected to outer train!");
  }
  void onDisconnect(BLEClient* c) {
    bleConnected = false;
    pChar        = nullptr;
    digitalWrite(STATUS_LED, LOW);
    Serial.println("[BLE] ⚠️  Disconnected — retrying...");
  }
};

class ScanCallback : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice device) {
    String name = device.getName().c_str();
    String addr = device.getAddress().toString().c_str();
    if (name.length() > 0) Serial.println("  Scanned: " + name + " [" + addr + "]");
    if (name.startsWith(TRAIN_NAME_PREFIX)) {
      Serial.println("  >>> Found: " + name);
      foundMAC = addr;
      BLEDevice::getScan()->stop();
    }
  }
};

// ══════════════════════════════════════════════════════════════════
//  BLE: CONNECT TO TRAIN
// ══════════════════════════════════════════════════════════════════

bool connectToTrain(const char* mac) {
  Serial.printf("[BLE] Connecting to %s...\n", mac);
  if (!pClient) {
    pClient = BLEDevice::createClient();
    pClient->setClientCallbacks(new TrainCallbacks());
  }
  if (!pClient->connect(BLEAddress(mac))) {
    Serial.println("[BLE] ❌ Failed");
    return false;
  }
  BLERemoteService* svc = pClient->getService(BLEUUID(SERVICE_UUID));
  if (!svc) { pClient->disconnect(); return false; }
  pChar = svc->getCharacteristic(BLEUUID(CHARACTERISTIC_UUID));
  if (!pChar) { pClient->disconnect(); return false; }
  return true;
}

// ══════════════════════════════════════════════════════════════════
//  BLE: SEND COMMAND
// ══════════════════════════════════════════════════════════════════

bool sendBLE(uint8_t* cmd, size_t len, const char* label) {
  if (!bleConnected || !pChar) {
    Serial.printf("[BLE] Cannot send %s — not connected\n", label);
    return false;
  }
  pChar->writeValue(cmd, len, false);
  Serial.printf("[BLE] ▶ %s\n", label);
  return true;
}

// ══════════════════════════════════════════════════════════════════
//  OUTER TRAIN CONTROL (BLE)
// ══════════════════════════════════════════════════════════════════

void stopOuterTrain() {
  currentSpeed = 0;
  sendBLE(CMD_STOP, 3, "STOP outer train");
  for (int i = 0; i < 4; i++) {
    digitalWrite(STATUS_LED, !digitalRead(STATUS_LED));
    delay(100);
  }
  digitalWrite(STATUS_LED, bleConnected ? HIGH : LOW);
}

void resumeOuterTrain() {
  // Start slow then ramp to medium — safer than jumping to full speed
  sendBLE(CMD_SPEED_2, 3, "RESUME outer — slow");
  currentSpeed = 2;
  delay(SPEED_RAMP_DELAY_MS);
  sendBLE(CMD_SPEED_7, 3, "RESUME outer — medium");
  currentSpeed = 7;
}

void setOuterSpeed(int level) {
  level = constrain(level, 0, 7);
  uint8_t cmd[] = {0x00, 0x45, (uint8_t)level};
  char lbl[20];
  snprintf(lbl, sizeof(lbl), "SPEED %d", level);
  sendBLE(cmd, 3, lbl);
  currentSpeed = level;
}

// ══════════════════════════════════════════════════════════════════
//  INNER TRAIN CONTROL (RELAY)
// ══════════════════════════════════════════════════════════════════

void stopInnerTrain() {
  digitalWrite(RELAY_PIN, RELAY_INNER_OFF);
  Serial.println("[RELAY] Inner train STOPPED — power cut");
}

void resumeInnerTrain() {
  digitalWrite(RELAY_PIN, RELAY_INNER_ON);
  Serial.println("[RELAY] Inner train RUNNING — power restored");
}

// ══════════════════════════════════════════════════════════════════
//  SERIAL MENU
// ══════════════════════════════════════════════════════════════════

void printMenu() {
  Serial.println("\n╔══════════════════════════════════════════════╗");
  Serial.println("║  LIONCHIEF MANUAL CONTROL (Serial Monitor)  ║");
  Serial.println("╠══════════════════════════════════════════════╣");
  Serial.println("║  OUTER TRAIN (BLE)                          ║");
  Serial.println("║  s=STOP   e=EMERGENCY   f=FORWARD           ║");
  Serial.println("║  r=REVERSE   +=SpeedUP   -=SpeedDOWN        ║");
  Serial.println("║  1-7=Set speed directly                     ║");
  Serial.println("╠══════════════════════════════════════════════╣");
  Serial.println("║  SOUNDS                                     ║");
  Serial.println("║  h=Horn  b=Bell  l=Lights  a=Announce       ║");
  Serial.println("║  n=SoundON  m=SoundOFF  v=VolumeNext        ║");
  Serial.println("╠══════════════════════════════════════════════╣");
  Serial.println("║  INNER TRAIN (RELAY)                        ║");
  Serial.println("║  x=PowerON   z=PowerOFF                     ║");
  Serial.println("╠══════════════════════════════════════════════╣");
  Serial.println("║  i=Status   ?=Menu                          ║");
  Serial.println("╚══════════════════════════════════════════════╝");
  Serial.println("Type command + Enter:");
}

void printStatus() {
  Serial.println("\n── STATUS ──────────────────────────────────────");
  Serial.printf("  BLE Connected : %s\n", bleConnected ? "YES ✅" : "NO ❌");
  Serial.printf("  Zone Locked   : %s\n", zoneLocked ? "YES ⚠️" : "NO");
  Serial.printf("  Inner in zone : %s\n", innerTrainInZone ? "YES" : "NO");
  Serial.printf("  Outer in zone : %s\n", outerTrainInZone ? "YES" : "NO");
  Serial.printf("  Outer Speed   : %d/7\n", currentSpeed);
  Serial.printf("  Horn          : %s\n", hornActive ? "ON" : "OFF");
  Serial.printf("  Bell          : %s\n", bellActive ? "ON" : "OFF");
  Serial.printf("  Lights        : %s\n", lightsActive ? "ON" : "OFF");
  Serial.printf("  Volume        : %d/7\n", volumeLevel);
  Serial.printf("  Inner relay   : %s\n",
    digitalRead(RELAY_PIN) == RELAY_INNER_ON ? "POWERED" : "CUT");
  Serial.println("────────────────────────────────────────────────\n");
}

void handleSerialCommand(char cmd) {
  switch (cmd) {
    // ── Outer train speed ────────────────────────────────────────
    case 's': stopOuterTrain();         break;
    case 'e': stopOuterTrain();         break;
    case 'f': sendBLE(CMD_FORWARD, 3, "FORWARD");  break;
    case 'r': sendBLE(CMD_REVERSE, 3, "REVERSE");  break;
    case '+': setOuterSpeed(min(7, currentSpeed + 1)); break;
    case '-': setOuterSpeed(max(0, currentSpeed - 1)); break;
    case '1': setOuterSpeed(1); break;
    case '2': setOuterSpeed(2); break;
    case '3': setOuterSpeed(3); break;
    case '4': setOuterSpeed(4); break;
    case '5': setOuterSpeed(5); break;
    case '6': setOuterSpeed(6); break;
    case '7': setOuterSpeed(7); break;

    // ── Horn ─────────────────────────────────────────────────────
    case 'h':
      hornActive = !hornActive;
      sendBLE(hornActive ? CMD_HORN_ON : CMD_HORN_OFF, 3,
              hornActive ? "HORN ON" : "HORN OFF");
      break;

    // ── Bell ─────────────────────────────────────────────────────
    case 'b':
      bellActive = !bellActive;
      sendBLE(bellActive ? CMD_BELL_ON : CMD_BELL_OFF, 3,
              bellActive ? "BELL ON" : "BELL OFF");
      break;

    // ── Lights ───────────────────────────────────────────────────
    case 'l':
      lightsActive = !lightsActive;
      sendBLE(lightsActive ? CMD_LIGHT_ON : CMD_LIGHT_OFF, 3,
              lightsActive ? "LIGHTS ON" : "LIGHTS OFF");
      break;

    // ── Announce / speech ─────────────────────────────────────────
    case 'a':
      pChar->writeValue(CMD_ANNOUNCE, 4, false);
      Serial.println("[BLE] ▶ ANNOUNCE");
      break;

    // ── Sound / Volume ────────────────────────────────────────────
    case 'n': sendBLE(CMD_SOUND_ON,  3, "SOUND ON (vol max)"); volumeLevel = 7; break;
    case 'm': sendBLE(CMD_SOUND_OFF, 3, "SOUND OFF");          volumeLevel = 0; break;
    case 'v':
      // Cycle through volume levels 1 → 3 → 5 → 7 → 1
      volumeLevel = (volumeLevel < 7) ? volumeLevel + 2 : 1;
      {
        uint8_t vcmd[] = {0x00, 0x4C, (uint8_t)volumeLevel};
        char vlbl[20];
        snprintf(vlbl, sizeof(vlbl), "VOLUME %d", volumeLevel);
        sendBLE(vcmd, 3, vlbl);
      }
      break;

    // ── Inner train relay (manual override) ───────────────────────
    case 'x': resumeInnerTrain(); break;
    case 'z': stopInnerTrain();   break;

    // ── Info ─────────────────────────────────────────────────────
    case '?': printMenu();   break;
    case 'i': printStatus(); break;

    default:
      Serial.println("Unknown command. Type ? for menu.");
      break;
  }
}

// ══════════════════════════════════════════════════════════════════
//  SETUP
// ══════════════════════════════════════════════════════════════════

void setup() {
  Serial.begin(115200);
  delay(600);

  Serial.println("\n╔══════════════════════════════════════════════╗");
  Serial.println("║  Harry Locomotive — Collision Prevention v3  ║");
  Serial.println("║  3 IR Sensors + Relay + BLE                  ║");
  Serial.printf( "║  MAC: %-40s║\n", TARGET_MAC);
  Serial.println("╚══════════════════════════════════════════════╝\n");

  // LED
  pinMode(STATUS_LED, OUTPUT);
  digitalWrite(STATUS_LED, LOW);

  // Relay — start with inner train powered ON
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, RELAY_INNER_ON);
  Serial.printf("[GPIO] Relay (inner power) GPIO%d — POWERED ON\n", RELAY_PIN);

  // IR Sensors
  pinMode(SENSOR_A_PIN, INPUT_PULLUP);
  pinMode(SENSOR_B_PIN, INPUT_PULLUP);
  pinMode(SENSOR_C_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(SENSOR_A_PIN), ISR_SensorA, FALLING);
  attachInterrupt(digitalPinToInterrupt(SENSOR_B_PIN), ISR_SensorB, FALLING);
  attachInterrupt(digitalPinToInterrupt(SENSOR_C_PIN), ISR_SensorC, FALLING);
  Serial.printf("[GPIO] Sensor A (inner entry) GPIO%d\n", SENSOR_A_PIN);
  Serial.printf("[GPIO] Sensor B (outer entry) GPIO%d\n", SENSOR_B_PIN);
  Serial.printf("[GPIO] Sensor C (exit)         GPIO%d\n", SENSOR_C_PIN);

  // BLE
  BLEDevice::init("TrainCollisionGuard");
  Serial.println("\n[BLE] Connecting by MAC...");
  bool ok = connectToTrain(TARGET_MAC);

  if (!ok) {
    Serial.println("[BLE] Scanning by name prefix...");
    BLEScan* pScan = BLEDevice::getScan();
    pScan->setAdvertisedDeviceCallbacks(new ScanCallback());
    pScan->setActiveScan(true);
    pScan->start(15, false);
    if (foundMAC.length() > 0) ok = connectToTrain(foundMAC.c_str());
  }

  Serial.println(ok
    ? "\n[SYSTEM] ✅ Ready — monitoring shared section\n"
    : "\n[SYSTEM] ⚠️  BLE not connected — will retry in loop\n");

  printMenu();
}

// ══════════════════════════════════════════════════════════════════
//  MAIN LOOP
// ══════════════════════════════════════════════════════════════════

void loop() {

  // ── BLE Auto-reconnect ──────────────────────────────────────────
  if (!bleConnected) {
    if (millis() - lastReconnect > RECONNECT_MS) {
      lastReconnect = millis();
      Serial.println("[BLE] Reconnecting...");
      bool ok = connectToTrain(TARGET_MAC);
      if (!ok && foundMAC.length() > 0) ok = connectToTrain(foundMAC.c_str());
      if (!ok) {
        BLEScan* pScan = BLEDevice::getScan();
        pScan->setAdvertisedDeviceCallbacks(new ScanCallback());
        pScan->setActiveScan(true);
        pScan->start(8, false);
        if (foundMAC.length() > 0) connectToTrain(foundMAC.c_str());
      }
    }
    delay(100);
    return;
  }

  // ── Serial manual control ───────────────────────────────────────
  if (Serial.available()) {
    char cmd = Serial.read();
    if (cmd != '\n' && cmd != '\r') {
      Serial.printf("\n> '%c'\n", cmd);
      handleSerialCommand(cmd);
    }
  }

  // ── SENSOR A: Inner train entering ─────────────────────────────
  if (sensorA_fired) {
    sensorA_fired = false;
    if (!zoneLocked) {
      Serial.println("\n[SENSOR A] ⚠️  Inner train entering shared section!");
      innerTrainInZone = true;
      zoneLocked       = true;
      waitingToResume  = false;
      stopOuterTrain();
      lastStopSent = millis();
      Serial.println("[ZONE] LOCKED — outer train stopped\n");
    } else {
      Serial.println("[SENSOR A] Zone already locked — ignored");
    }
  }

  // ── SENSOR B: Outer train entering ─────────────────────────────
  if (sensorB_fired) {
    sensorB_fired = false;
    if (!zoneLocked) {
      Serial.println("\n[SENSOR B] ⚠️  Outer train entering shared section!");
      outerTrainInZone = true;
      zoneLocked       = true;
      waitingToResume  = false;
      stopInnerTrain();
      Serial.println("[ZONE] LOCKED — inner train power cut\n");
    } else {
      Serial.println("[SENSOR B] Zone already locked — ignored");
    }
  }

  // ── SENSOR C: Train exiting ─────────────────────────────────────
  if (sensorC_fired) {
    sensorC_fired = false;
    if (zoneLocked && !waitingToResume) {
      Serial.println("[SENSOR C] ✅ Train exiting shared section");
      Serial.printf("[SYSTEM] Safety delay %dms...\n", RESUME_DELAY_MS);
      waitingToResume = true;
      resumeTimer     = millis();
    }
  }

  // ── Repeat STOP while inner is in zone ─────────────────────────
  // Prevents outer train from creeping in if first BLE message missed
  if (innerTrainInZone && zoneLocked) {
    if (millis() - lastStopSent > STOP_REPEAT_MS) {
      sendBLE(CMD_STOP, 3, "STOP repeated");
      lastStopSent = millis();
    }
  }

  // ── Safety delay expired → resume stopped train ─────────────────
  if (waitingToResume && (millis() - resumeTimer >= RESUME_DELAY_MS)) {
    waitingToResume = false;

    if (innerTrainInZone) {
      Serial.println("[RESUME] Inner cleared → resuming outer train");
      innerTrainInZone = false;
      resumeOuterTrain();
    } else if (outerTrainInZone) {
      Serial.println("[RESUME] Outer cleared → restoring inner train power");
      outerTrainInZone = false;
      resumeInnerTrain();
    }

    zoneLocked = false;
    Serial.println("[ZONE] UNLOCKED — both trains free\n");
  }

  // ── Heartbeat LED ───────────────────────────────────────────────
  static unsigned long lastBlink = 0;
  if (!zoneLocked && (millis() - lastBlink > 2000)) {
    lastBlink = millis();
    digitalWrite(STATUS_LED, HIGH);
    delay(40);
    digitalWrite(STATUS_LED, LOW);
  }

  delay(10);
}