/*
 * ═══════════════════════════════════════════════════════════════════════════
 *  train_controller/src/main.cpp
 *  Harry Locomotive Project — Autonomous Collision Prevention System
 *  Datix AI  |  Ahmed Ali  |  v5.1  |  May 2026
 *
 * ─── CHANGES IN v5.1 ───────────────────────────────────────────────────────
 *
 *  1. RESUME AT USER-SET SPEED (not hardcoded slow speed)
 *     Before any STOP is sent, the current speed is saved as userSetSpeed.
 *     When outer train resumes after collision prevention, it returns to
 *     exactly the speed the user had set — not a fixed slow speed.
 *     Example: user sets speed 5, train stops for inner loop, resumes at 5.
 *
 *  2. INNER TRAIN AUTO-LOOP EVERY 30 SECONDS
 *     Inner train no longer requires manual X press.
 *     Every AUTO_INNER_INTERVAL_MS (default 30s), if:
 *       - Zone is clear (STATE_IDLE)
 *       - Outer BLE train is NOT in shared section
 *       - Inner train is not already running
 *     → Inner train automatically gets power, runs one loop, parks.
 *     X key still works for immediate manual trigger.
 *     Interval is configurable at the top of this file.
 *
 *  3. KEEPALIVE BUG FIXED
 *     Keepalive now only sends if currentSpeed > 0.
 *     Previously sending speed 0 during keepalive was silently stopping
 *     the outer train every 20 seconds.
 *
 *  4. GPIO CORRECTED — Peter's confirmed wiring
 *     RELAY_POWER_PIN  = GPIO19 (Peter has inner train power here)
 *     RELAY_SWITCH_PIN = GPIO18 (track switch — not yet connected)
 *
 * ─── STATE MACHINE (6 states) ──────────────────────────────────────────────
 *
 *   IDLE ──[SensorA]──► LOCKED ──[SensorC]──► INNER_PARKING ──► DELAY ──►
 *   IDLE ──[SensorB]──► LOCKED ──[SensorC]──► DELAY ──► SWITCH_PULSE ──►
 *   Both paths → RAMP → IDLE
 *
 *   STATE_IDLE           Outer running, inner parked — monitoring sensors
 *   STATE_LOCKED         Zone occupied — collision prevention active
 *   STATE_INNER_PARKING  Inner exited — coasting to parking position
 *   STATE_DELAY          Safety buffer before resuming outer train
 *   STATE_SWITCH_PULSE   1s relay pulse resets track switch (outer exit only)
 *   STATE_RAMP           Outer train ramping back to user-set speed
 *
 * ─── COMPLETE PIN ASSIGNMENTS ──────────────────────────────────────────────
 *
 *   GPIO15 → Sensor B  black wire  Outer loop entry — only outer train passes
 *   GPIO16 → Sensor A  black wire  Inner loop entry — only inner train passes
 *   GPIO17 → Sensor C  black wire  Shared section exit — any train
 *   GPIO18 → Relay 2   IN pin      Track switch 1s pulse (not yet connected)
 *   GPIO19 → Relay 1   IN pin      Inner train track power (Peter confirmed)
 *   GPIO2  → Built-in LED          Status indicator — no external wiring
 *   VIN    → All 3 sensors VCC     5V power (brown wire on E18-D80NK)
 *   VIN    → Relay 1 VCC           5V power
 *   VIN    → Relay 2 VCC           5V power
 *   GND    → All 3 sensors GND     Common ground (blue wire on E18-D80NK)
 *   GND    → Relay 1 GND           Common ground
 *   GND    → Relay 2 GND           Common ground
 *
 * ─── RELAY WIRING DETAIL ───────────────────────────────────────────────────
 *
 *   Both relays = standard Arduino module = ACTIVE LOW
 *   HIGH = coil off = safe/default state
 *   LOW  = coil on  = relay activates
 *
 *   Relay 1 (GPIO19) — Inner train track power — uses NC terminal:
 *     HIGH → NC closed → power flows    → inner train runs
 *     LOW  → NC opens  → power cut      → inner train stops
 *
 *   Relay 2 (GPIO18) — Track switch reset — uses NO terminal:
 *     HIGH → NO open   → switch idle    → default state
 *     LOW  → NO closes → switch resets  → 1 second pulse only
 *     Switch has its own 18V AC supply — relay just closes the circuit.
 *
 * ─── E18-D80NK SENSOR WIRING ───────────────────────────────────────────────
 *
 *   Brown wire → VIN (5V)
 *   Blue wire  → GND
 *   Black wire → GPIO signal pin
 *   Output = HIGH when beam clear, LOW when beam broken (train detected)
 *
 * ─── BLE OUTER TRAIN ───────────────────────────────────────────────────────
 *   MAC            : CC:01:78:D0:F0:99
 *   Name           : LC015556-99F0
 *   Service UUID   : e20a39f4-73f5-4bc4-a12f-17d1ad07a961
 *   Characteristic : 08590f7e-db05-467e-8757-72f6faeb13d4
 *
 * ═══════════════════════════════════════════════════════════════════════════
 */

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEClient.h>
#include <BLERemoteCharacteristic.h>
#include <esp_task_wdt.h>

// ═══════════════════════════════════════════════════════════════════════════
//  CONFIGURATION — only change values here, never touch logic below
// ═══════════════════════════════════════════════════════════════════════════

// BLE — outer LionChief train
#define TARGET_MAC          "CC:01:78:D0:F0:99"
#define TRAIN_NAME_PREFIX   "LC0"
#define SERVICE_UUID        "e20a39f4-73f5-4bc4-a12f-17d1ad07a961"
#define CHARACTERISTIC_UUID "08590f7e-db05-467e-8757-72f6faeb13d4"

// GPIO — Peter's confirmed wiring (v5.1 corrected)
#define SENSOR_A_PIN     16   // Inner loop entry (inner train only)
#define SENSOR_B_PIN     15   // Outer loop entry (outer train only)
#define SENSOR_C_PIN     17   // Shared section exit (any train)
#define RELAY_POWER_PIN  19   // Relay 1: inner train track power ← GPIO19 confirmed
#define RELAY_SWITCH_PIN 18   // Relay 2: track switch 1s pulse   ← GPIO18 not yet connected
#define STATUS_LED        2   // Built-in LED

// Relay polarity — standard Arduino relay module is ACTIVE LOW
#define RELAY_ENERGIZE  LOW    // Coil ON  → NC opens / NO closes
#define RELAY_RELEASE   HIGH   // Coil OFF → NC closed / NO open (safe default)

// ── Timing — all in milliseconds ───────────────────────────────────────────

#define AUTO_INNER_INTERVAL_MS  30000  // ★ TUNE THIS: how often inner train
                                       //   auto-runs. 30000 = every 30 seconds.
                                       //   Change to 45000 for every 45 seconds.
                                       //   Inner only starts if zone is clear.

#define PARKING_DELAY_MS         2000  // ★ TUNE THIS: time inner train takes to
                                       //   reach parking spot AFTER Sensor C.
                                       //   Increase if it overshoots parking.
                                       //   Decrease if it stops too early.

#define RESUME_DELAY_MS          1500  // Safety buffer after zone clears
#define SPEED_RAMP_MS            1500  // Time at medium speed before user speed
#define STOP_REPEAT_MS            500  // Repeat BLE STOP every 500ms while locked
#define SWITCH_PULSE_MS          1000  // Track switch relay pulse duration
#define ZONE_TIMEOUT_MS         30000  // Force resume if Sensor C never fires
#define SENSOR_DEBOUNCE_MS        300  // Min ms between same-sensor triggers
#define BLE_RECONNECT_MS         5000  // BLE reconnect attempt interval
#define BLE_KEEPALIVE_MS        20000  // BLE keepalive ping interval
#define WATCHDOG_TIMEOUT_S          60 // Hardware WDT reset timeout

// Speed levels (0-7)
#define DEFAULT_OUTER_SPEED       7   // Speed outer train starts at on boot
#define RESUME_RAMP_SPEED         5   // Intermediate speed during ramp-up

// ═══════════════════════════════════════════════════════════════════════════
//  BLE COMMANDS — confirmed on Peter's LionChief locomotive via nRF Connect
// ═══════════════════════════════════════════════════════════════════════════

uint8_t CMD_STOP[]     = {0x00, 0x45, 0x00};
uint8_t CMD_SPEED_1[]  = {0x00, 0x45, 0x01};
uint8_t CMD_SPEED_2[]  = {0x00, 0x45, 0x02};
uint8_t CMD_SPEED_3[]  = {0x00, 0x45, 0x03};
uint8_t CMD_SPEED_4[]  = {0x00, 0x45, 0x04};
uint8_t CMD_SPEED_5[]  = {0x00, 0x45, 0x05};
uint8_t CMD_SPEED_6[]  = {0x00, 0x45, 0x06};
uint8_t CMD_SPEED_7[]  = {0x00, 0x45, 0x07};
uint8_t CMD_FORWARD[]  = {0x00, 0x46, 0x01};
uint8_t CMD_REVERSE[]  = {0x00, 0x46, 0x02};
uint8_t CMD_HORN_ON[]  = {0x00, 0x48, 0x01};
uint8_t CMD_HORN_OFF[] = {0x00, 0x48, 0x00};
uint8_t CMD_BELL_ON[]  = {0x00, 0x47, 0x01};
uint8_t CMD_BELL_OFF[] = {0x00, 0x47, 0x00};
uint8_t CMD_LIGHT_ON[] = {0x00, 0x51, 0x01};
uint8_t CMD_LIGHT_OFF[]= {0x00, 0x51, 0x00};
uint8_t CMD_SOUND_ON[] = {0x00, 0x4C, 0x07};
uint8_t CMD_SOUND_OFF[]= {0x00, 0x4C, 0x00};
uint8_t CMD_ANNOUNCE[] = {0x00, 0x4D, 0x00, 0x00};

// ═══════════════════════════════════════════════════════════════════════════
//  ERROR CODES
// ═══════════════════════════════════════════════════════════════════════════

enum ErrorCode {
  ERR_NONE = 0,
  ERR_BLE_CONNECT_FAILED,
  ERR_BLE_SERVICE_NOT_FOUND,
  ERR_BLE_CHAR_NOT_FOUND,
  ERR_BLE_WRITE_FAILED,
  ERR_BLE_NOT_CONNECTED,
  ERR_BLE_SCAN_NO_RESULT,
  ERR_SENSOR_STUCK_LOW,
  ERR_ZONE_TIMEOUT,
  ERR_RELAY_STUCK,
  ERR_WATCHDOG_RESET,
  ERR_INNER_BLOCKED,
};

const char* ERROR_MESSAGES[] = {
  "No error",
  "BLE connect() failed — train off or out of range",
  "BLE service UUID not found — wrong device?",
  "BLE characteristic not found — UUID mismatch",
  "BLE GATT write failed — connection dropped mid-send",
  "BLE command ignored — not connected",
  "BLE scan found no LionChief device",
  "IR sensor stuck LOW — check wiring/alignment",
  "Zone timeout — Sensor C never fired, force-resuming",
  "Relay GPIO did not respond to write",
  "System reset by hardware watchdog",
  "Inner train blocked — zone active, will try again when clear",
};

RTC_DATA_ATTR uint32_t errorCounts[12] = {0};
RTC_DATA_ATTR uint32_t totalRestarts   = 0;
RTC_DATA_ATTR uint32_t wdtResets       = 0;

void reportError(ErrorCode code, const char* context = nullptr) {
  if (code == ERR_NONE) return;
  errorCounts[code]++;
  Serial.println("\n╔══════════════════════════════════════════════╗");
  Serial.printf( "║ ❌ ERROR [%02d] x%-3lu — %s\n",
                 (int)code, (unsigned long)errorCounts[code],
                 ERROR_MESSAGES[code]);
  if (context) Serial.printf("║ Context : %s\n", context);
  Serial.printf( "║ Uptime  : %lums\n", millis());
  Serial.println("╚══════════════════════════════════════════════╝\n");
}

// ═══════════════════════════════════════════════════════════════════════════
//  SYSTEM STATE
// ═══════════════════════════════════════════════════════════════════════════

BLEClient*               pClient      = nullptr;
BLERemoteCharacteristic* pChar        = nullptr;
bool                     bleConnected = false;
String                   foundMAC     = "";

enum ZoneState {
  STATE_IDLE,
  STATE_LOCKED,
  STATE_INNER_PARKING,
  STATE_DELAY,
  STATE_SWITCH_PULSE,
  STATE_RAMP
};

ZoneState zoneState        = STATE_IDLE;
bool      innerTrainCaused = false;

// ── Speed tracking ─────────────────────────────────────────────────────────
// currentSpeed  = speed currently commanded to outer train
// userSetSpeed  = speed user chose before any stop — restored on resume
int currentSpeed = DEFAULT_OUTER_SPEED;
int userSetSpeed = DEFAULT_OUTER_SPEED;

// ── Inner train state ──────────────────────────────────────────────────────
bool          innerTrainActive    = false;
bool          innerWasCutForOuter = false;
unsigned long lastInnerRun        = 0;   // millis() of last inner train start
                                          // used for AUTO_INNER_INTERVAL_MS timer

// ── Non-blocking timers ────────────────────────────────────────────────────
unsigned long lockTimer     = 0;
unsigned long parkingTimer  = 0;
unsigned long resumeTimer   = 0;
unsigned long switchTimer   = 0;
unsigned long rampTimer     = 0;
unsigned long lastStopSent  = 0;
unsigned long lastReconnect = 0;
unsigned long lastKeepalive = 0;

// ── Sensor debounce timestamps ─────────────────────────────────────────────
unsigned long lastSensorA = 0;
unsigned long lastSensorB = 0;
unsigned long lastSensorC = 0;

// ── Sensor stuck detection ─────────────────────────────────────────────────
#define SENSOR_STUCK_CHECKS 50
uint8_t sensorALowCount = 0;
uint8_t sensorBLowCount = 0;
uint8_t sensorCLowCount = 0;

// ── ISR flags ─────────────────────────────────────────────────────────────
volatile bool sensorA_fired = false;
volatile bool sensorB_fired = false;
volatile bool sensorC_fired = false;

// ── Manual control state ───────────────────────────────────────────────────
bool hornActive   = false;
bool bellActive   = false;
bool lightsActive = false;
int  volumeLevel  = 7;

// ── Statistics ─────────────────────────────────────────────────────────────
uint32_t stopsSent      = 0;
uint32_t resumesSent    = 0;
uint32_t bleWriteErrors = 0;
uint32_t zoneTimeouts   = 0;
uint32_t bleReconnects  = 0;
uint32_t innerLoopsRun  = 0;

// ═══════════════════════════════════════════════════════════════════════════
//  ISR — FALLING edge = train just broke the beam
// ═══════════════════════════════════════════════════════════════════════════

void IRAM_ATTR ISR_SensorA() { if (digitalRead(SENSOR_A_PIN)==LOW) sensorA_fired=true; }
void IRAM_ATTR ISR_SensorB() { if (digitalRead(SENSOR_B_PIN)==LOW) sensorB_fired=true; }
void IRAM_ATTR ISR_SensorC() { if (digitalRead(SENSOR_C_PIN)==LOW) sensorC_fired=true; }

// ═══════════════════════════════════════════════════════════════════════════
//  BLE CALLBACKS
// ═══════════════════════════════════════════════════════════════════════════

class TrainClientCallbacks : public BLEClientCallbacks {
  void onConnect(BLEClient* client) {
    bleConnected = true;
    bleReconnects++;
    digitalWrite(STATUS_LED, HIGH);
    Serial.printf("[BLE] ✅ Connected to outer train (total connects: %lu)\n",
                  (unsigned long)bleReconnects);
  }
  void onDisconnect(BLEClient* client) {
    bleConnected = false;
    pChar        = nullptr;
    digitalWrite(STATUS_LED, LOW);
    Serial.println("[BLE] ⚠️  Disconnected — auto-reconnect in 5s");
    if (zoneState != STATE_IDLE)
      Serial.println("[BLE] ⚠️  Lost connection while zone LOCKED — reconnecting urgently");
  }
};

class ScanCallback : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice device) {
    String name = device.getName().c_str();
    String addr = device.getAddress().toString().c_str();
    if (name.length() > 0)
      Serial.printf("[BLE] Scan: %-28s [%s]\n", name.c_str(), addr.c_str());
    if (name.startsWith(TRAIN_NAME_PREFIX)) {
      Serial.printf("[BLE] ✅ LionChief found: %s [%s]\n", name.c_str(), addr.c_str());
      foundMAC = addr;
      BLEDevice::getScan()->stop();
    }
  }
};

// ═══════════════════════════════════════════════════════════════════════════
//  BLE: CONNECT
// ═══════════════════════════════════════════════════════════════════════════

bool connectToTrain(const char* mac) {
  Serial.printf("[BLE] Connecting to %s...\n", mac);
  if (!pClient) {
    pClient = BLEDevice::createClient();
    pClient->setClientCallbacks(new TrainClientCallbacks());
  }
  if (!pClient->connect(BLEAddress(mac))) {
    reportError(ERR_BLE_CONNECT_FAILED, mac);
    return false;
  }
  BLERemoteService* pSvc = pClient->getService(BLEUUID(SERVICE_UUID));
  if (!pSvc) {
    reportError(ERR_BLE_SERVICE_NOT_FOUND, SERVICE_UUID);
    pClient->disconnect();
    return false;
  }
  pChar = pSvc->getCharacteristic(BLEUUID(CHARACTERISTIC_UUID));
  if (!pChar) {
    reportError(ERR_BLE_CHAR_NOT_FOUND, CHARACTERISTIC_UUID);
    pClient->disconnect();
    return false;
  }
  Serial.println("[BLE] ✅ Service + characteristic found — ready");
  return true;
}

// ═══════════════════════════════════════════════════════════════════════════
//  BLE: SEND COMMAND
// ═══════════════════════════════════════════════════════════════════════════

bool sendBLE(uint8_t* cmd, size_t len, const char* label) {
  if (!bleConnected || !pChar) {
    reportError(ERR_BLE_NOT_CONNECTED, label);
    return false;
  }
  try {
    pChar->writeValue(cmd, len, false);
    Serial.printf("[BLE] ▶ %-38s [%02X %02X %02X]\n",
                  label, cmd[0], cmd[1], (len > 2 ? cmd[2] : 0));
    return true;
  }
  catch (const std::exception& e) {
    bleWriteErrors++;
    reportError(ERR_BLE_WRITE_FAILED, e.what());
    bleConnected = false;
    return false;
  }
  catch (...) {
    bleWriteErrors++;
    reportError(ERR_BLE_WRITE_FAILED, "Unknown exception in GATT write");
    bleConnected = false;
    return false;
  }
}

// ── BLE speed helpers ──────────────────────────────────────────────────────

// Send STOP — saves current speed first so we can restore it on resume
bool bleSendStop() {
  stopsSent++;
  // Only save userSetSpeed if train was actually moving
  // (prevents saving 0 if stop is sent while already stopped)
  if (currentSpeed > 0) userSetSpeed = currentSpeed;
  currentSpeed = 0;
  return sendBLE(CMD_STOP, 3, "STOP outer train");
}

// Send speed command and update both tracking variables
// userSetSpeed only updated if speed > 0 — prevents resume-at-zero bug
bool bleSetSpeed(int speed, const char* label) {
  speed = constrain(speed, 0, 7);
  uint8_t cmd[] = {0x00, 0x45, (uint8_t)speed};
  currentSpeed = speed;
  if (speed > 0) userSetSpeed = speed;  // never save 0 as restore target
  return sendBLE(cmd, 3, label);
}

// Resume at intermediate ramp speed, capped at userSetSpeed
// Prevents train from briefly overshooting if user had set a low speed
bool bleResumeRamp() {
  resumesSent++;
  int rampSpeed = min(RESUME_RAMP_SPEED, userSetSpeed);
  if (rampSpeed == 0) rampSpeed = RESUME_RAMP_SPEED;  // fallback if userSetSpeed was 0
  currentSpeed = rampSpeed;
  uint8_t cmd[] = {0x00, 0x45, (uint8_t)rampSpeed};
  char label[40];
  snprintf(label, sizeof(label), "RESUME ramp speed %d", rampSpeed);
  return sendBLE(cmd, 3, label);
}

// Resume at the exact speed user had before the stop
bool bleResumeUserSpeed() {
  currentSpeed = userSetSpeed;
  uint8_t cmd[] = {0x00, 0x45, (uint8_t)userSetSpeed};
  char label[40];
  snprintf(label, sizeof(label), "RESUME user speed %d", userSetSpeed);
  return sendBLE(cmd, 3, label);
}

// ═══════════════════════════════════════════════════════════════════════════
//  RELAY CONTROL
// ═══════════════════════════════════════════════════════════════════════════

void innerTrainPowerCut() {
  digitalWrite(RELAY_POWER_PIN, RELAY_ENERGIZE);
  if (digitalRead(RELAY_POWER_PIN) != RELAY_ENERGIZE)
    reportError(ERR_RELAY_STUCK, "RELAY_POWER_PIN (GPIO19) did not energize");
  Serial.println("[RELAY1] 🛑 Inner train STOPPED — track power cut (GPIO19)");
}

void innerTrainPowerRestore() {
  digitalWrite(RELAY_POWER_PIN, RELAY_RELEASE);
  if (digitalRead(RELAY_POWER_PIN) != RELAY_RELEASE)
    reportError(ERR_RELAY_STUCK, "RELAY_POWER_PIN (GPIO19) did not release");
  Serial.println("[RELAY1] ✅ Inner train RUNNING — track power restored (GPIO19)");
}

void trackSwitchPulseStart() {
  digitalWrite(RELAY_SWITCH_PIN, RELAY_ENERGIZE);
  Serial.printf("[RELAY2] Track switch PULSING %dms (GPIO18)\n", SWITCH_PULSE_MS);
}

void trackSwitchPulseEnd() {
  digitalWrite(RELAY_SWITCH_PIN, RELAY_RELEASE);
  Serial.println("[RELAY2] Track switch pulse DONE (GPIO18)");
}

// ═══════════════════════════════════════════════════════════════════════════
//  INNER TRAIN AUTO-START
//  Called both from auto-timer and X manual command.
//  Returns true if inner train was started, false if blocked.
// ═══════════════════════════════════════════════════════════════════════════

bool startInnerTrain(bool manual) {
  if (zoneState != STATE_IDLE) {
    if (manual) {
      reportError(ERR_INNER_BLOCKED,
                  "Zone active — inner will auto-start when zone clears");
      Serial.println("[INNER] ❌ Zone busy — will auto-start at next opportunity");
    }
    return false;
  }
  if (innerTrainActive) {
    if (manual) Serial.println("[INNER] ⚠️  Already running its loop");
    return false;
  }

  innerLoopsRun++;
  innerTrainActive    = true;
  innerWasCutForOuter = false;
  lastInnerRun        = millis();

  innerTrainPowerRestore();

  Serial.printf("[INNER] 🚂 Inner train STARTED — loop #%lu (%s)\n",
                (unsigned long)innerLoopsRun,
                manual ? "manual X" : "auto-timer");
  Serial.printf("[INNER] Will park %dms after Sensor C fires\n",
                PARKING_DELAY_MS);
  return true;
}

// ═══════════════════════════════════════════════════════════════════════════
//  SENSOR HEALTH CHECK
// ═══════════════════════════════════════════════════════════════════════════

void checkSensorHealth() {
  sensorALowCount = (digitalRead(SENSOR_A_PIN)==LOW)
    ? (sensorALowCount < 255 ? sensorALowCount+1 : 255) : 0;
  sensorBLowCount = (digitalRead(SENSOR_B_PIN)==LOW)
    ? (sensorBLowCount < 255 ? sensorBLowCount+1 : 255) : 0;
  sensorCLowCount = (digitalRead(SENSOR_C_PIN)==LOW)
    ? (sensorCLowCount < 255 ? sensorCLowCount+1 : 255) : 0;

  if (sensorALowCount >= SENSOR_STUCK_CHECKS) {
    reportError(ERR_SENSOR_STUCK_LOW, "Sensor A GPIO16 stuck LOW");
    sensorALowCount = 0;
  }
  if (sensorBLowCount >= SENSOR_STUCK_CHECKS) {
    reportError(ERR_SENSOR_STUCK_LOW, "Sensor B GPIO15 stuck LOW");
    sensorBLowCount = 0;
  }
  if (sensorCLowCount >= SENSOR_STUCK_CHECKS) {
    reportError(ERR_SENSOR_STUCK_LOW, "Sensor C GPIO17 stuck LOW");
    sensorCLowCount = 0;
  }
}

// ═══════════════════════════════════════════════════════════════════════════
//  DIAGNOSTICS
// ═══════════════════════════════════════════════════════════════════════════

void printStatus() {
  const char* SN[] = {"IDLE","LOCKED","INNER_PARKING","DELAY","SWITCH_PULSE","RAMP"};
  unsigned long upSec = millis()/1000;
  unsigned long nextInner = 0;
  if (zoneState == STATE_IDLE && !innerTrainActive) {
    unsigned long elapsed = millis() - lastInnerRun;
    if (elapsed < AUTO_INNER_INTERVAL_MS)
      nextInner = (AUTO_INNER_INTERVAL_MS - elapsed) / 1000;
  }

  Serial.println("\n╔══════════════════════════════════════════════════════╗");
  Serial.println("║           SYSTEM DIAGNOSTICS v5.1                   ║");
  Serial.println("╠══════════════════════════════════════════════════════╣");
  Serial.printf( "║  Uptime         : %02luh %02lum %02lus\n", upSec/3600,(upSec%3600)/60,upSec%60);
  Serial.printf( "║  Restarts       : %-5lu  WDT resets: %-5lu\n",(unsigned long)totalRestarts,(unsigned long)wdtResets);
  Serial.println("╠══════════════════════════════════════════════════════╣");
  Serial.printf( "║  BLE            : %s\n", bleConnected?"Connected ✅":"Disconnected ❌");
  Serial.printf( "║  BLE reconnects : %-5lu  Write errors: %-5lu\n",(unsigned long)bleReconnects,(unsigned long)bleWriteErrors);
  Serial.println("╠══════════════════════════════════════════════════════╣");
  Serial.printf( "║  Zone state     : %s\n", SN[zoneState]);
  Serial.printf( "║  Outer speed    : %d/7 (user set: %d/7)\n", currentSpeed, userSetSpeed);
  Serial.printf( "║  Relay 1 GPIO19 : %s\n", digitalRead(RELAY_POWER_PIN)==RELAY_ENERGIZE?"CUT ⚠️":"OK — power flowing ✅");
  Serial.printf( "║  Relay 2 GPIO18 : %s\n", digitalRead(RELAY_SWITCH_PIN)==RELAY_ENERGIZE?"PULSING":"idle");
  Serial.println("╠══════════════════════════════════════════════════════╣");
  Serial.printf( "║  Inner active   : %s\n", innerTrainActive?"YES — running loop":"NO — parked");
  Serial.printf( "║  Inner loops run: %-5lu\n",(unsigned long)innerLoopsRun);
  if (nextInner > 0)
    Serial.printf("║  Next auto-start: ~%lus\n", nextInner);
  else
    Serial.println("║  Next auto-start: starting soon or already active");
  Serial.println("╠══════════════════════════════════════════════════════╣");
  Serial.printf( "║  Stops sent     : %-5lu  Resumes: %-5lu\n",(unsigned long)stopsSent,(unsigned long)resumesSent);
  Serial.printf( "║  Zone timeouts  : %-5lu\n",(unsigned long)zoneTimeouts);
  Serial.println("╠══════════════════════════════════════════════════════╣");
  Serial.printf( "║  Sensor A GPIO16: %s\n", digitalRead(SENSOR_A_PIN)==LOW?"LOW — blocked ⚠️":"HIGH — clear ✅");
  Serial.printf( "║  Sensor B GPIO15: %s\n", digitalRead(SENSOR_B_PIN)==LOW?"LOW — blocked ⚠️":"HIGH — clear ✅");
  Serial.printf( "║  Sensor C GPIO17: %s\n", digitalRead(SENSOR_C_PIN)==LOW?"LOW — blocked ⚠️":"HIGH — clear ✅");
  Serial.println("╠══════════════════════════════════════════════════════╣");
  bool anyErr = false;
  for (int i=1;i<=11;i++) {
    if (errorCounts[i]>0) {
      Serial.printf("║  ERR[%02d] x%-3lu %s\n",i,(unsigned long)errorCounts[i],ERROR_MESSAGES[i]);
      anyErr = true;
    }
  }
  if (!anyErr) Serial.println("║  No errors recorded ✅");
  Serial.println("╚══════════════════════════════════════════════════════╝\n");
}

void printMenu() {
  Serial.println("\n╔════════════════════════════════════════════════════╗");
  Serial.println("║   Harry Locomotive v5.1 — Serial Commands          ║");
  Serial.println("╠════════════════════════════════════════════════════╣");
  Serial.println("║  OUTER TRAIN (BLE)                                 ║");
  Serial.println("║  s/e = STOP     f = FORWARD    r = REVERSE         ║");
  Serial.println("║  1-7 = Set speed directly                          ║");
  Serial.println("║  + / - = Speed up / down one step                  ║");
  Serial.println("║  Resume always returns to speed you last set       ║");
  Serial.println("╠════════════════════════════════════════════════════╣");
  Serial.println("║  SOUNDS                                            ║");
  Serial.println("║  h=Horn  b=Bell  l=Lights  a=Announce              ║");
  Serial.println("║  n=SoundON  m=SoundOFF  v=VolumeNext               ║");
  Serial.println("╠════════════════════════════════════════════════════╣");
  Serial.println("║  INNER TRAIN (AUTO every 30s + manual)             ║");
  Serial.printf( "║  Auto-runs every %-5dms when zone is clear        ║\n",
                 AUTO_INNER_INTERVAL_MS);
  Serial.println("║  X = Trigger inner train immediately (manual)      ║");
  Serial.println("║  z = Emergency cut inner train power               ║");
  Serial.println("║  p = Manual track switch pulse (1s, GPIO18)        ║");
  Serial.println("╠════════════════════════════════════════════════════╣");
  Serial.println("║  i = Full diagnostics    ? = This menu             ║");
  Serial.println("╚════════════════════════════════════════════════════╝\n");
}

// ═══════════════════════════════════════════════════════════════════════════
//  SERIAL COMMAND HANDLER
// ═══════════════════════════════════════════════════════════════════════════

void handleSerial(char cmd) {
  switch (cmd) {

    // ── Outer train speed ──────────────────────────────────────────
    case 's': case 'e':
      bleSendStop();
      break;
    case 'f': sendBLE(CMD_FORWARD, 3, "FORWARD"); break;
    case 'r': sendBLE(CMD_REVERSE, 3, "REVERSE"); break;
    case '+': bleSetSpeed(currentSpeed + 1, "SPEED UP");   break;
    case '-': bleSetSpeed(currentSpeed - 1, "SPEED DOWN"); break;
    case '1': bleSetSpeed(1, "SPEED 1"); break;
    case '2': bleSetSpeed(2, "SPEED 2"); break;
    case '3': bleSetSpeed(3, "SPEED 3"); break;
    case '4': bleSetSpeed(4, "SPEED 4"); break;
    case '5': bleSetSpeed(5, "SPEED 5"); break;
    case '6': bleSetSpeed(6, "SPEED 6"); break;
    case '7': bleSetSpeed(7, "SPEED 7"); break;

    // ── Sounds ────────────────────────────────────────────────────
    case 'h':
      hornActive = !hornActive;
      sendBLE(hornActive ? CMD_HORN_ON : CMD_HORN_OFF, 3,
              hornActive ? "HORN ON" : "HORN OFF");
      break;
    case 'b':
      bellActive = !bellActive;
      sendBLE(bellActive ? CMD_BELL_ON : CMD_BELL_OFF, 3,
              bellActive ? "BELL ON" : "BELL OFF");
      break;
    case 'l':
      lightsActive = !lightsActive;
      sendBLE(lightsActive ? CMD_LIGHT_ON : CMD_LIGHT_OFF, 3,
              lightsActive ? "LIGHTS ON" : "LIGHTS OFF");
      break;
    case 'a':
      if (pChar) pChar->writeValue(CMD_ANNOUNCE, 4, false);
      Serial.println("[BLE] ▶ ANNOUNCE");
      break;
    case 'n': sendBLE(CMD_SOUND_ON,  3, "SOUND ON");  volumeLevel=7; break;
    case 'm': sendBLE(CMD_SOUND_OFF, 3, "SOUND OFF"); volumeLevel=0; break;
    case 'v': {
      volumeLevel = (volumeLevel < 7) ? volumeLevel + 2 : 1;
      uint8_t vc[] = {0x00, 0x4C, (uint8_t)volumeLevel};
      char vl[20]; snprintf(vl, sizeof(vl), "VOLUME %d", volumeLevel);
      sendBLE(vc, 3, vl);
      break;
    }

    // ── Inner train ───────────────────────────────────────────────
    case 'X': case 'x':
      startInnerTrain(true);   // manual trigger
      break;

    case 'z':
      innerTrainActive    = false;
      innerWasCutForOuter = false;
      innerTrainPowerCut();
      Serial.println("[INNER] ⚠️  Emergency STOP — inner power cut manually");
      break;

    case 'p':
      if (zoneState == STATE_IDLE) {
        trackSwitchPulseStart();
        switchTimer = millis();
        Serial.println("[RELAY2] Manual switch pulse started");
      } else {
        Serial.println("[RELAY2] Cannot pulse — zone is active");
      }
      break;

    case 'i': printStatus(); break;
    case '?': printMenu();   break;
    default:
      Serial.printf("[CMD] Unknown: '%c' — press ? for menu\n", cmd);
      break;
  }
}

// ═══════════════════════════════════════════════════════════════════════════
//  SETUP
//  ⚠️  RELAYS SET FIRST — before Serial.begin() to prevent boot float
// ═══════════════════════════════════════════════════════════════════════════

void setup() {
  // MUST be first — prevents relay boot float (GPIO floats LOW = relay fires)
  pinMode(RELAY_POWER_PIN,  OUTPUT);
  pinMode(RELAY_SWITCH_PIN, OUTPUT);
  digitalWrite(RELAY_POWER_PIN,  RELAY_RELEASE);
  digitalWrite(RELAY_SWITCH_PIN, RELAY_RELEASE);

  Serial.begin(115200);
  delay(400);
  totalRestarts++;

  esp_reset_reason_t reason = esp_reset_reason();
  if (reason == ESP_RST_TASK_WDT || reason == ESP_RST_WDT) {
    wdtResets++;
    reportError(ERR_WATCHDOG_RESET, "Loop hung — watchdog reset");
  }

  Serial.println("\n╔════════════════════════════════════════════════════╗");
  Serial.println("║  Harry Locomotive — Collision Prevention v5.1      ║");
  Serial.println("║  Datix AI  |  Ahmed Ali  |  May 2026               ║");
  Serial.println("╠════════════════════════════════════════════════════╣");
  Serial.printf( "║  Boot #%-5lu  WDT resets: %-5lu                    ║\n",
                 (unsigned long)totalRestarts,(unsigned long)wdtResets);
  Serial.printf( "║  Inner auto-interval : %-5dms (~%ds)              ║\n",
                 AUTO_INNER_INTERVAL_MS, AUTO_INNER_INTERVAL_MS/1000);
  Serial.printf( "║  Parking delay       : %-5dms                     ║\n",
                 PARKING_DELAY_MS);
  Serial.printf( "║  Resume ramp speed   : %-5d/7                     ║\n",
                 RESUME_RAMP_SPEED);
  Serial.println("╚════════════════════════════════════════════════════╝\n");

  // Confirm relay state at boot
  Serial.printf("[GPIO] Relay 1 GPIO19 (inner power) : %s\n",
    digitalRead(RELAY_POWER_PIN)==RELAY_RELEASE ? "RELEASED ✅" : "⚠️ NOT RELEASED");
  Serial.printf("[GPIO] Relay 2 GPIO18 (switch pulse): %s\n",
    digitalRead(RELAY_SWITCH_PIN)==RELAY_RELEASE ? "RELEASED ✅" : "⚠️ NOT RELEASED");

  esp_task_wdt_init(WATCHDOG_TIMEOUT_S, true);
  esp_task_wdt_add(NULL);
  Serial.printf("[WDT] Watchdog armed — %ds timeout\n", WATCHDOG_TIMEOUT_S);

  pinMode(STATUS_LED, OUTPUT);
  digitalWrite(STATUS_LED, LOW);

  // Sensors — INPUT_PULLUP: HIGH = clear, LOW = train detected
  pinMode(SENSOR_A_PIN, INPUT_PULLUP);
  pinMode(SENSOR_B_PIN, INPUT_PULLUP);
  pinMode(SENSOR_C_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(SENSOR_A_PIN), ISR_SensorA, FALLING);
  attachInterrupt(digitalPinToInterrupt(SENSOR_B_PIN), ISR_SensorB, FALLING);
  attachInterrupt(digitalPinToInterrupt(SENSOR_C_PIN), ISR_SensorC, FALLING);
  Serial.printf("[GPIO] Sensor A GPIO%d — inner loop entry\n", SENSOR_A_PIN);
  Serial.printf("[GPIO] Sensor B GPIO%d — outer loop entry\n", SENSOR_B_PIN);
  Serial.printf("[GPIO] Sensor C GPIO%d — shared section exit\n", SENSOR_C_PIN);

  // Boot sensor check
  delay(100);
  if (digitalRead(SENSOR_A_PIN)==LOW) reportError(ERR_SENSOR_STUCK_LOW,"Sensor A LOW at boot");
  if (digitalRead(SENSOR_B_PIN)==LOW) reportError(ERR_SENSOR_STUCK_LOW,"Sensor B LOW at boot");
  if (digitalRead(SENSOR_C_PIN)==LOW) reportError(ERR_SENSOR_STUCK_LOW,"Sensor C LOW at boot");

  // BLE connect
  BLEDevice::init("TrainController");
  Serial.println("\n[BLE] Connecting by MAC...");
  bool ok = connectToTrain(TARGET_MAC);

  if (!ok) {
    Serial.printf("[BLE] Scanning for '%s'...\n", TRAIN_NAME_PREFIX);
    BLEScan* pScan = BLEDevice::getScan();
    pScan->setAdvertisedDeviceCallbacks(new ScanCallback());
    pScan->setActiveScan(true);
    pScan->start(15, false);
    if (foundMAC.length() > 0)
      ok = connectToTrain(foundMAC.c_str());
    else
      reportError(ERR_BLE_SCAN_NO_RESULT, "Is outer train powered on?");
  }

  // Start inner auto-timer from now so first auto-run happens after interval
  lastInnerRun = millis();

  // Set outer train to default running speed
  if (ok) {
    bleSetSpeed(DEFAULT_OUTER_SPEED, "INITIAL SPEED");
  }

  Serial.println(ok
    ? "\n[SYSTEM] ✅ Ready\n"
      "         Outer train running at user speed\n"
      "         Inner train will auto-start every 30s\n"
      "         Press X for immediate inner train trigger\n"
    : "\n[SYSTEM] ⚠️  BLE not connected — will retry every 5s\n");

  printMenu();
}

// ═══════════════════════════════════════════════════════════════════════════
//  MAIN LOOP
// ═══════════════════════════════════════════════════════════════════════════

void loop() {
  unsigned long now = millis();
  esp_task_wdt_reset();

  // ── BLE: Auto-reconnect ────────────────────────────────────────────
  if (!bleConnected && (now - lastReconnect > BLE_RECONNECT_MS)) {
    lastReconnect = now;
    Serial.println("[BLE] Reconnecting...");
    bool ok = connectToTrain(TARGET_MAC);
    if (!ok && foundMAC.length() > 0) ok = connectToTrain(foundMAC.c_str());
    if (!ok) {
      BLEScan* pScan = BLEDevice::getScan();
      pScan->setAdvertisedDeviceCallbacks(new ScanCallback());
      pScan->setActiveScan(true);
      pScan->start(5, false);
      if (foundMAC.length() > 0) connectToTrain(foundMAC.c_str());
    }
    // Re-send STOP immediately if reconnected during a locked zone
    // Use sendBLE directly — bleSendStop would corrupt userSetSpeed/stopsSent
    if (bleConnected && zoneState != STATE_IDLE && innerTrainCaused) {
      Serial.println("[BLE] Reconnected during lock — re-sending STOP");
      sendBLE(CMD_STOP, 3, "STOP re-send after reconnect");
      lastStopSent = now;
    }
  }

  // ── BLE: Keepalive — only if speed > 0 (never accidentally stops train) ─
  if (bleConnected &&
      (zoneState == STATE_IDLE || zoneState == STATE_RAMP) &&
      (now - lastKeepalive > BLE_KEEPALIVE_MS)) {
    lastKeepalive = now;
    if (currentSpeed > 0) {
      uint8_t c[] = {0x00, 0x45, (uint8_t)currentSpeed};
      sendBLE(c, 3, "keepalive");
    }
  }

  // ── Serial commands ────────────────────────────────────────────────
  if (Serial.available()) {
    char cmd = Serial.read();
    if (cmd != '\n' && cmd != '\r') {
      Serial.printf("\n> '%c'\n", cmd);
      handleSerial(cmd);
    }
  }

  // ── Sensor health check (every ~50 cycles) ─────────────────────────
  static uint8_t healthCtr = 0;
  if (++healthCtr >= 50) { healthCtr = 0; checkSensorHealth(); }

  // ── Manual switch pulse auto-end ───────────────────────────────────
  if (zoneState == STATE_IDLE &&
      digitalRead(RELAY_SWITCH_PIN) == RELAY_ENERGIZE &&
      (now - switchTimer >= SWITCH_PULSE_MS)) {
    trackSwitchPulseEnd();
  }

  // ── Auto inner train timer ─────────────────────────────────────────
  // Every AUTO_INNER_INTERVAL_MS, if zone is clear, start inner train.
  // startInnerTrain() checks zone state internally — safe to call here.
  if (zoneState == STATE_IDLE &&
      !innerTrainActive &&
      (now - lastInnerRun >= AUTO_INNER_INTERVAL_MS)) {
    startInnerTrain(false);   // auto trigger (not manual)
  }

  // ════════════════════════════════════════════════════════════════════
  //  ZONE STATE MACHINE
  // ════════════════════════════════════════════════════════════════════

  switch (zoneState) {

    // ──────────────────────────────────────────────────────────────────
    //  STATE_IDLE
    // ──────────────────────────────────────────────────────────────────
    case STATE_IDLE: {

      // Sensor A — inner train entering shared section
      if (sensorA_fired) {
        sensorA_fired = false;
        if (now - lastSensorA > SENSOR_DEBOUNCE_MS) {
          lastSensorA = now;
          if (!innerTrainActive)
            Serial.println("[SENSOR A] ⚠️  Fired but inner not active — monitoring");
          Serial.println("\n[SENSOR A] ⚠️  Inner train entering shared section!");
          Serial.printf( "[ACTION]   BLE STOP — user speed %d saved, will restore on resume\n",
                         userSetSpeed);
          innerTrainCaused = true;
          zoneState        = STATE_LOCKED;
          lockTimer        = now;
          lastStopSent     = now;
          bleSendStop();   // saves userSetSpeed before setting currentSpeed=0
        }
      }

      // Sensor B — outer train entering shared section
      if (sensorB_fired) {
        sensorB_fired = false;
        if (now - lastSensorB > SENSOR_DEBOUNCE_MS) {
          lastSensorB = now;
          Serial.println("\n[SENSOR B] ⚠️  Outer train entering shared section!");
          innerTrainCaused = false;
          zoneState        = STATE_LOCKED;
          lockTimer        = now;
          if (innerTrainActive) {
            innerWasCutForOuter = true;
            innerTrainPowerCut();
            Serial.println("[ACTION]   Inner power cut — will restore after outer exits");
          } else {
            innerWasCutForOuter = false;
            Serial.println("[ACTION]   Zone locked — inner was parked, no relay action");
          }
        }
      }

      break;
    }

    // ──────────────────────────────────────────────────────────────────
    //  STATE_LOCKED
    // ──────────────────────────────────────────────────────────────────
    case STATE_LOCKED: {

      // Repeat STOP while inner is in zone — outer must not move
      if (innerTrainCaused && (now - lastStopSent > STOP_REPEAT_MS)) {
        lastStopSent = now;
        sendBLE(CMD_STOP, 3, "STOP repeated");
      }

      // Train exits shared section
      if (sensorC_fired) {
        sensorC_fired = false;
        if (now - lastSensorC > SENSOR_DEBOUNCE_MS) {
          lastSensorC = now;
          Serial.println("[SENSOR C] ✅ Train exiting shared section");
          if (innerTrainCaused) {
            Serial.printf("[INNER]    Coasting to parking — %dms\n", PARKING_DELAY_MS);
            zoneState    = STATE_INNER_PARKING;
            parkingTimer = now;
          } else {
            Serial.printf("[SYSTEM]   Outer exited — safety delay %dms\n", RESUME_DELAY_MS);
            zoneState   = STATE_DELAY;
            resumeTimer = now;
          }
        }
      }

      // Timeout — Sensor C never fired
      if (now - lockTimer > ZONE_TIMEOUT_MS) {
        zoneTimeouts++;
        reportError(ERR_ZONE_TIMEOUT,
                    innerTrainCaused
                    ? "Inner may be stuck in shared section"
                    : "Outer may be stuck in shared section");
        if (innerTrainCaused) {
          innerTrainActive = false;
          innerTrainPowerCut();
        }
        // Reset inner timer so auto-start doesn't fire immediately after recovery
        lastInnerRun     = now;
        innerWasCutForOuter = false;   // clear flag — timeout is a hard reset
        zoneState   = STATE_DELAY;
        resumeTimer = now;
      }

      break;
    }

    // ──────────────────────────────────────────────────────────────────
    //  STATE_INNER_PARKING
    //  Inner exited zone — count down to parking cut
    // ──────────────────────────────────────────────────────────────────
    case STATE_INNER_PARKING: {

      // Keep outer stopped while inner coasts to parking
      if (now - lastStopSent > STOP_REPEAT_MS) {
        lastStopSent = now;
        sendBLE(CMD_STOP, 3, "STOP during inner parking");
      }

      if (now - parkingTimer >= PARKING_DELAY_MS) {
        innerTrainActive = false;
        lastInnerRun     = now;   // reset auto-timer from now
        innerTrainPowerCut();
        Serial.println("[INNER]    ⛔ Stopped at parking position");
        Serial.printf( "[INNER]    Next auto-run in %ds\n", AUTO_INNER_INTERVAL_MS/1000);
        zoneState   = STATE_DELAY;
        resumeTimer = now;
      }

      break;
    }

    // ──────────────────────────────────────────────────────────────────
    //  STATE_DELAY
    //  Safety buffer. Then either switch pulse (outer) or resume (inner).
    // ──────────────────────────────────────────────────────────────────
    case STATE_DELAY: {

      if (now - lastStopSent > STOP_REPEAT_MS) {
        lastStopSent = now;
        sendBLE(CMD_STOP, 3, "STOP during delay");
      }

      if (now - resumeTimer >= RESUME_DELAY_MS) {
        if (!innerTrainCaused) {
          // Outer exited → pulse track switch first
          trackSwitchPulseStart();
          zoneState   = STATE_SWITCH_PULSE;
          switchTimer = now;
        } else {
          // Inner parked → resume outer at ramp speed first
          Serial.printf("[RESUME]   Inner parked → outer ramping to speed %d\n",
                        RESUME_RAMP_SPEED);
          bleResumeRamp();
          zoneState = STATE_RAMP;
          rampTimer = now;
        }
      }

      break;
    }

    // ──────────────────────────────────────────────────────────────────
    //  STATE_SWITCH_PULSE — only for outer train exit
    // ──────────────────────────────────────────────────────────────────
    case STATE_SWITCH_PULSE: {

      if (now - lastStopSent > STOP_REPEAT_MS) {
        lastStopSent = now;
        sendBLE(CMD_STOP, 3, "STOP during switch pulse");
      }

      if (now - switchTimer >= SWITCH_PULSE_MS) {
        trackSwitchPulseEnd();

        // Restore inner power if it was cut because outer entered zone
        if (innerWasCutForOuter && innerTrainActive) {
          innerTrainPowerRestore();
          innerWasCutForOuter = false;
          Serial.println("[INNER]    Power restored — continuing loop");
        } else {
          innerWasCutForOuter = false;
        }

        Serial.printf("[RESUME]   Outer train ramping to speed %d\n", RESUME_RAMP_SPEED);
        bleResumeRamp();
        zoneState = STATE_RAMP;
        rampTimer = now;
      }

      break;
    }

    // ──────────────────────────────────────────────────────────────────
    //  STATE_RAMP
    //  Short time at ramp speed → then restore exact user-set speed
    // ──────────────────────────────────────────────────────────────────
    case STATE_RAMP: {

      if (now - rampTimer >= SPEED_RAMP_MS) {
        // Restore to the exact speed the user had before the stop
        Serial.printf("[RESUME]   Restoring user speed %d/7\n", userSetSpeed);
        bleResumeUserSpeed();

        sensorA_fired    = false;
        sensorB_fired    = false;
        sensorC_fired    = false;
        innerTrainCaused = false;
        zoneState        = STATE_IDLE;

        Serial.println("[ZONE]     ✅ UNLOCKED — outer at user speed, inner parked\n");
      }

      break;
    }
  }

  // ── Heartbeat LED ──────────────────────────────────────────────────
  static unsigned long lastBlink   = 0;
  static unsigned long ledOnTime   = 0;
  static bool          ledBlinking = false;

  if (zoneState == STATE_IDLE && bleConnected) {
    if (!ledBlinking && (now - lastBlink > 2000)) {
      lastBlink = now; ledOnTime = now; ledBlinking = true;
      digitalWrite(STATUS_LED, HIGH);
    }
    if (ledBlinking && (now - ledOnTime > 40)) {
      ledBlinking = false;
      digitalWrite(STATUS_LED, LOW);
    }
  }

  delay(5);
}