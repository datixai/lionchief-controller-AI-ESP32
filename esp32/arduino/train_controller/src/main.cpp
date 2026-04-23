/*
 * ═══════════════════════════════════════════════════════════════════════════
 *  train_controller/src/main.cpp
 *  Harry Locomotive Project — Autonomous Collision Prevention System
 *  Datix AI  |  Ahmed Ali  |  v4.1  |  April 2026
 *
 * ─── ARCHITECTURE ──────────────────────────────────────────────────────────
 *
 *  This firmware implements a deterministic, non-blocking zone state machine
 *  that prevents collisions between two trains sharing a common track section.
 *
 *  HARDWARE:
 *    Two trains — one BLE-controlled (outer loop), one track-power (inner loop)
 *    Three E18-D80NK IR beam sensors for zone detection
 *    Two relay modules — one for inner train power, one for track switch reset
 *    ESP32 DevKit V1 (30-pin) as the sole controller
 *
 *  DETECTION PRINCIPLE:
 *    Sensors are placed on physically separate loops so each sensor can only
 *    be triggered by one specific train — no software identification needed.
 *    Sensor A is unreachable by the outer train. Sensor B is unreachable by
 *    the inner train. Sensor C detects any train exiting the shared section.
 *
 *  STATE MACHINE (5 states):
 *
 *    ┌─────────────────────────────────────────────────────────────────┐
 *    │                                                                 │
 *    │  IDLE ──[SensorA/B]──► LOCKED ──[SensorC/timeout]──► DELAY    │
 *    │   ▲                                                      │     │
 *    │   │                                              [delay done]  │
 *    │   │                                                      ▼     │
 *    │   └──────────────────── RAMP ◄── SWITCH_PULSE ◄──────────┘    │
 *    │                                                                 │
 *    └─────────────────────────────────────────────────────────────────┘
 *
 *    STATE_IDLE         Normal operation — monitoring all sensors
 *    STATE_LOCKED       Train in zone — outer stopped / inner power cut
 *    STATE_DELAY        Train exited — 2.5s safety buffer before resume
 *    STATE_SWITCH_PULSE 1-second relay pulse resets track switch direction
 *    STATE_RAMP         Resume outer train slowly before ramping to full speed
 *
 *  NON-BLOCKING GUARANTEE:
 *    Zero delay() calls anywhere in the main control loop.
 *    All timing is millis()-based so the BLE stack is never starved.
 *    The only delay() calls are in setup() before any BLE activity begins.
 *
 * ─── WIRING ────────────────────────────────────────────────────────────────
 *
 *    GPIO16 → Sensor A black wire  (inner loop entry — inner train only)
 *    GPIO15 → Sensor B black wire  (outer loop entry — outer train only)
 *    GPIO17 → Sensor C black wire  (shared section exit — any train)
 *    GPIO18 → Relay 1 IN           (inner train track power control)
 *    GPIO19 → Relay 2 IN           (track switch reset — 1s momentary pulse)
 *    VIN    → All sensors VCC + both relays VCC  (5V rail)
 *    GND    → All sensors GND + both relays GND  (common ground)
 *    GPIO2  → Built-in status LED  (no external component needed)
 *
 * ─── RELAY WIRING DETAIL ───────────────────────────────────────────────────
 *
 *    Standard Arduino relay module = ACTIVE LOW (coil energizes on LOW)
 *
 *    Relay 1 — Inner train track power (uses NC terminal):
 *      COM → inner loop track power wire #1
 *      NC  → inner loop track power wire #2
 *      Logic: HIGH = de-energized = NC closed = power flows = train runs
 *             LOW  = energized    = NC opens  = power cut   = train stops
 *
 *    Relay 2 — Track switch reset (uses NO terminal):
 *      COM → track switch contact #1
 *      NO  → track switch contact #2
 *      Logic: HIGH = de-energized = NO open   = switch untouched (default)
 *             LOW  = energized    = NO closes = switch reset (1s pulse only)
 *
 * ─── CONFIRMED TRAIN BLE DETAILS ───────────────────────────────────────────
 *    Device Name    : LC015556-99F0
 *    MAC Address    : CC:01:78:D0:F0:99
 *    Service UUID   : e20a39f4-73f5-4bc4-a12f-17d1ad07a961
 *    Characteristic : 08590f7e-db05-467e-8757-72f6faeb13d4
 *
 * ═══════════════════════════════════════════════════════════════════════════
 */

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEClient.h>
#include <BLERemoteCharacteristic.h>
#include <esp_task_wdt.h>   // ESP32 hardware watchdog

// ═══════════════════════════════════════════════════════════════════════════
//  CONFIGURATION — edit these values to tune system behaviour
// ═══════════════════════════════════════════════════════════════════════════

// BLE — outer train
#define TARGET_MAC           "CC:01:78:D0:F0:99"
#define TRAIN_NAME_PREFIX    "LC0"
#define SERVICE_UUID         "e20a39f4-73f5-4bc4-a12f-17d1ad07a961"
#define CHARACTERISTIC_UUID  "08590f7e-db05-467e-8757-72f6faeb13d4"

// GPIO assignments
#define SENSOR_A_PIN     16    // Inner loop entry sensor
#define SENSOR_B_PIN     15    // Outer loop entry sensor
#define SENSOR_C_PIN     17    // Shared section exit sensor
#define RELAY_POWER_PIN  18    // Relay 1: inner train track power
#define RELAY_SWITCH_PIN 19    // Relay 2: track switch reset pulse
#define STATUS_LED        2    // Built-in LED (no external wiring)

// Relay polarity — standard Arduino relay module is ACTIVE LOW
#define RELAY_ENERGIZE   LOW   // Activate relay coil
#define RELAY_RELEASE    HIGH  // Release relay coil (default/safe state)

// Timing parameters (milliseconds)
#define RESUME_DELAY_MS      2500   // Safety buffer after train exits zone
#define SPEED_RAMP_MS        3000   // Duration at slow speed before full speed
#define STOP_REPEAT_MS        500   // BLE STOP repeat interval while zone locked
#define SWITCH_PULSE_MS      1000   // Track switch relay pulse duration
#define ZONE_TIMEOUT_MS     30000   // Max lock duration — prevents permanent deadlock
#define SENSOR_DEBOUNCE_MS    300   // Min interval between same-sensor triggers
#define BLE_RECONNECT_MS     5000   // Interval between BLE reconnect attempts
#define BLE_KEEPALIVE_MS    20000   // BLE keepalive ping interval
#define WATCHDOG_TIMEOUT_S     60   // Hardware WDT timeout in seconds

// Resume speed levels
#define RESUME_SLOW_SPEED    2      // Speed sent on initial resume (0-7)
#define RESUME_FULL_SPEED    7      // Speed sent after ramp delay

// ═══════════════════════════════════════════════════════════════════════════
//  ALL LIONCHIEF BLE COMMAND BYTES
//  Confirmed via nRF Connect on Peter's actual locomotive
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
//  Centralised error enum so every failure has a unique, searchable label
// ═══════════════════════════════════════════════════════════════════════════

enum ErrorCode {
  ERR_NONE = 0,
  ERR_BLE_CONNECT_FAILED,       // pClient->connect() returned false
  ERR_BLE_SERVICE_NOT_FOUND,    // LionChief service UUID not found on device
  ERR_BLE_CHAR_NOT_FOUND,       // Write characteristic not found
  ERR_BLE_WRITE_FAILED,         // GATT write operation threw an exception
  ERR_BLE_NOT_CONNECTED,        // Command sent while disconnected
  ERR_BLE_SCAN_NO_RESULT,       // Name-prefix scan found no matching device
  ERR_SENSOR_STUCK_HIGH,        // Sensor reads LOW continuously (possible fault)
  ERR_ZONE_TIMEOUT,             // Zone locked for ZONE_TIMEOUT_MS with no Sensor C
  ERR_RELAY_STUCK,              // Relay pin could not be set (GPIO fault)
  ERR_WATCHDOG_RESET,           // System was reset by hardware watchdog
};

// Human-readable error messages matching the enum above
const char* ERROR_MESSAGES[] = {
  "No error",
  "BLE connect() failed — train may be off or out of range",
  "BLE service UUID not found — wrong device or train firmware issue",
  "BLE characteristic not found — UUID mismatch",
  "BLE GATT write failed — connection may have dropped mid-send",
  "BLE command ignored — not connected",
  "BLE scan found no LionChief device with prefix LC0",
  "IR sensor reads LOW continuously — check wiring or sensor alignment",
  "Zone timeout — Sensor C did not fire within 30s, force-resuming",
  "Relay GPIO write error",
  "System was reset by hardware watchdog — possible loop hang",
};

// Error statistics — persisted in RTC RAM so they survive soft resets
RTC_DATA_ATTR uint32_t errorCounts[11] = {0};   // index = ErrorCode value
RTC_DATA_ATTR uint32_t totalRestarts   = 0;
RTC_DATA_ATTR uint32_t wdtResets       = 0;

// ── Error reporter ─────────────────────────────────────────────────────────
void reportError(ErrorCode code, const char* context = nullptr) {
  if (code == ERR_NONE) return;

  errorCounts[code]++;

  Serial.println("\n╔══════════════════════════════════════════════╗");
  Serial.printf( "║ ❌ ERROR [%02d] — Count: %-22lu║\n",
                 (int)code, (unsigned long)errorCounts[code]);
  Serial.printf( "║ %s\n", ERROR_MESSAGES[code]);
  if (context) {
    Serial.printf("║ Context: %s\n", context);
  }
  Serial.printf( "║ Uptime: %lums\n", millis());
  Serial.println("╚══════════════════════════════════════════════╝\n");
}

// ═══════════════════════════════════════════════════════════════════════════
//  SYSTEM STATE
// ═══════════════════════════════════════════════════════════════════════════

// BLE handles
BLEClient*               pClient      = nullptr;
BLERemoteCharacteristic* pChar        = nullptr;
bool                     bleConnected = false;
String                   foundMAC     = "";

// ── Zone state machine ─────────────────────────────────────────────────────
enum ZoneState {
  STATE_IDLE,           // Both trains running normally
  STATE_LOCKED,         // Zone occupied — one train stopped
  STATE_DELAY,          // Zone cleared — safety buffer counting down
  STATE_SWITCH_PULSE,   // Pulsing track switch relay
  STATE_RAMP            // Outer train ramping from slow to full speed
};

ZoneState zoneState       = STATE_IDLE;
bool      innerTrainCaused = false;   // Which train triggered the lock?

// ── Non-blocking timers ────────────────────────────────────────────────────
unsigned long lockTimer     = 0;
unsigned long resumeTimer   = 0;
unsigned long switchTimer   = 0;
unsigned long rampTimer     = 0;
unsigned long lastStopSent  = 0;
unsigned long lastReconnect = 0;
unsigned long lastKeepalive = 0;

// ── Sensor debounce timestamps ─────────────────────────────────────────────
unsigned long lastSensorA   = 0;
unsigned long lastSensorB   = 0;
unsigned long lastSensorC   = 0;

// ── Sensor stuck detection ─────────────────────────────────────────────────
// If a sensor reads LOW for more than this many consecutive checks = fault
#define SENSOR_STUCK_CHECKS  50
uint8_t sensorALowCount = 0;
uint8_t sensorBLowCount = 0;
uint8_t sensorCLowCount = 0;

// ── ISR flags (set in interrupt, cleared in main loop) ─────────────────────
volatile bool sensorA_fired = false;
volatile bool sensorB_fired = false;
volatile bool sensorC_fired = false;

// ── Manual control tracking ────────────────────────────────────────────────
bool hornActive    = false;
bool bellActive    = false;
bool lightsActive  = false;
int  currentSpeed  = 0;
int  volumeLevel   = 7;

// ── Statistics ─────────────────────────────────────────────────────────────
uint32_t stopsSent         = 0;
uint32_t resumesSent       = 0;
uint32_t bleWriteErrors    = 0;
uint32_t zoneTimeouts      = 0;
uint32_t bleReconnects     = 0;

// ═══════════════════════════════════════════════════════════════════════════
//  INTERRUPT SERVICE ROUTINES
//  IRAM_ATTR places these in RAM for guaranteed fast execution.
//  E18-D80NK: output is HIGH when beam clear, LOW when object detected.
//  We trigger on FALLING edge (HIGH → LOW = train just arrived).
// ═══════════════════════════════════════════════════════════════════════════

void IRAM_ATTR ISR_SensorA() {
  if (digitalRead(SENSOR_A_PIN) == LOW) sensorA_fired = true;
}
void IRAM_ATTR ISR_SensorB() {
  if (digitalRead(SENSOR_B_PIN) == LOW) sensorB_fired = true;
}
void IRAM_ATTR ISR_SensorC() {
  if (digitalRead(SENSOR_C_PIN) == LOW) sensorC_fired = true;
}

// ═══════════════════════════════════════════════════════════════════════════
//  BLE CLIENT CALLBACKS
// ═══════════════════════════════════════════════════════════════════════════

class TrainClientCallbacks : public BLEClientCallbacks {
  void onConnect(BLEClient* client) {
    bleConnected  = true;
    bleReconnects++;
    digitalWrite(STATUS_LED, HIGH);
    Serial.printf("[BLE] ✅ Connected to outer train  (total reconnects: %lu)\n",
                  (unsigned long)bleReconnects);
  }
  void onDisconnect(BLEClient* client) {
    bleConnected = false;
    pChar        = nullptr;
    digitalWrite(STATUS_LED, LOW);
    Serial.println("[BLE] ⚠️  Disconnected from train — auto-reconnect in 5s");

    // If we disconnect during a locked zone, the outer train may start moving.
    // Log this as a significant event but do not change zone state —
    // the STOP will be re-sent as soon as BLE reconnects.
    if (zoneState != STATE_IDLE) {
      Serial.println("[BLE] ⚠️  Disconnected while zone is LOCKED — "
                     "reconnecting urgently");
    }
  }
};

// ── BLE scan callback — used for name-prefix fallback ─────────────────────
class ScanCallback : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice device) {
    String name = device.getName().c_str();
    String addr = device.getAddress().toString().c_str();

    // Log every found device for debugging
    if (name.length() > 0) {
      Serial.printf("[BLE] Scan found: %-30s [%s]\n",
                    name.c_str(), addr.c_str());
    }

    // Match by LionChief name prefix
    if (name.startsWith(TRAIN_NAME_PREFIX)) {
      Serial.printf("[BLE] ✅ LionChief found by name: %s [%s]\n",
                    name.c_str(), addr.c_str());
      foundMAC = addr;
      BLEDevice::getScan()->stop();
    }
  }
};

// ═══════════════════════════════════════════════════════════════════════════
//  BLE: CONNECT TO TRAIN
//  Returns true on full success (connected + service + characteristic found)
//  Every failure path logs a specific error code.
// ═══════════════════════════════════════════════════════════════════════════

bool connectToTrain(const char* mac) {
  Serial.printf("[BLE] Connecting to %s...\n", mac);

  // Reuse existing client or create a new one
  if (!pClient) {
    pClient = BLEDevice::createClient();
    pClient->setClientCallbacks(new TrainClientCallbacks());
  }

  // Attempt TCP-level BLE connection
  if (!pClient->connect(BLEAddress(mac))) {
    reportError(ERR_BLE_CONNECT_FAILED, mac);
    return false;
  }

  // Locate the LionChief GATT service
  BLERemoteService* pService = pClient->getService(BLEUUID(SERVICE_UUID));
  if (!pService) {
    reportError(ERR_BLE_SERVICE_NOT_FOUND,
                "Expected: e20a39f4-73f5-4bc4-a12f-17d1ad07a961");
    pClient->disconnect();
    return false;
  }

  // Locate the write characteristic
  pChar = pService->getCharacteristic(BLEUUID(CHARACTERISTIC_UUID));
  if (!pChar) {
    reportError(ERR_BLE_CHAR_NOT_FOUND,
                "Expected: 08590f7e-db05-467e-8757-72f6faeb13d4");
    pClient->disconnect();
    return false;
  }

  // onConnect callback will set bleConnected = true
  Serial.println("[BLE] ✅ Service and characteristic found — all commands ready");
  return true;
}

// ═══════════════════════════════════════════════════════════════════════════
//  BLE: SEND COMMAND
//  Wraps every write in a try-catch and reports the exact failure.
//  Returns true on success, false on any failure.
// ═══════════════════════════════════════════════════════════════════════════

bool sendBLE(uint8_t* cmd, size_t len, const char* label) {
  if (!bleConnected || !pChar) {
    reportError(ERR_BLE_NOT_CONNECTED, label);
    return false;
  }

  try {
    pChar->writeValue(cmd, len, false);
    Serial.printf("[BLE] ▶ %-35s [0x%02X 0x%02X 0x%02X]\n",
                  label, cmd[0], cmd[1], (len > 2 ? cmd[2] : 0));
    return true;
  }
  catch (const std::exception& e) {
    bleWriteErrors++;
    reportError(ERR_BLE_WRITE_FAILED, e.what());
    bleConnected = false;   // treat as disconnected — will trigger reconnect
    return false;
  }
  catch (...) {
    bleWriteErrors++;
    reportError(ERR_BLE_WRITE_FAILED, "Unknown exception during GATT write");
    bleConnected = false;
    return false;
  }
}

// ── Convenience wrappers for common commands ──────────────────────────────
bool bleSendStop()       { stopsSent++;   return sendBLE(CMD_STOP,    3, "STOP outer train"); }
bool bleResumeSlowly()   { resumesSent++; return sendBLE(CMD_SPEED_2, 3, "RESUME outer — slow"); }
bool bleResumeFull()     {               return sendBLE(CMD_SPEED_7, 3, "RESUME outer — full speed"); }

// ═══════════════════════════════════════════════════════════════════════════
//  RELAY CONTROL
//  Both functions log their action and confirm the GPIO state after writing.
// ═══════════════════════════════════════════════════════════════════════════

void innerTrainPowerCut() {
  digitalWrite(RELAY_POWER_PIN, RELAY_ENERGIZE);
  // Verify the pin was actually set (detects GPIO faults)
  if (digitalRead(RELAY_POWER_PIN) != RELAY_ENERGIZE) {
    reportError(ERR_RELAY_STUCK, "RELAY_POWER_PIN did not respond to write");
  }
  Serial.println("[RELAY1] 🛑 Inner train STOPPED — track power cut");
}

void innerTrainPowerRestore() {
  digitalWrite(RELAY_POWER_PIN, RELAY_RELEASE);
  if (digitalRead(RELAY_POWER_PIN) != RELAY_RELEASE) {
    reportError(ERR_RELAY_STUCK, "RELAY_POWER_PIN did not release");
  }
  Serial.println("[RELAY1] ✅ Inner train RUNNING — track power restored");
}

void trackSwitchPulseStart() {
  digitalWrite(RELAY_SWITCH_PIN, RELAY_ENERGIZE);
  Serial.printf("[RELAY2] Track switch PULSING — %dms pulse started\n",
                SWITCH_PULSE_MS);
}

void trackSwitchPulseEnd() {
  digitalWrite(RELAY_SWITCH_PIN, RELAY_RELEASE);
  Serial.println("[RELAY2] Track switch pulse COMPLETE — switch position reset");
}

// ═══════════════════════════════════════════════════════════════════════════
//  SENSOR HEALTH CHECK
//  Detects sensors stuck LOW (continuous beam block = sensor fault or misalignment)
// ═══════════════════════════════════════════════════════════════════════════

void checkSensorHealth() {
  // Increment stuck counter if pin reads LOW; reset if HIGH
  sensorALowCount = (digitalRead(SENSOR_A_PIN) == LOW)
                    ? (sensorALowCount < 255 ? sensorALowCount + 1 : 255) : 0;
  sensorBLowCount = (digitalRead(SENSOR_B_PIN) == LOW)
                    ? (sensorBLowCount < 255 ? sensorBLowCount + 1 : 255) : 0;
  sensorCLowCount = (digitalRead(SENSOR_C_PIN) == LOW)
                    ? (sensorCLowCount < 255 ? sensorCLowCount + 1 : 255) : 0;

  if (sensorALowCount >= SENSOR_STUCK_CHECKS) {
    reportError(ERR_SENSOR_STUCK_HIGH, "Sensor A (GPIO16) reads LOW continuously");
    sensorALowCount = 0;   // reset so we don't spam the error every loop
  }
  if (sensorBLowCount >= SENSOR_STUCK_CHECKS) {
    reportError(ERR_SENSOR_STUCK_HIGH, "Sensor B (GPIO15) reads LOW continuously");
    sensorBLowCount = 0;
  }
  if (sensorCLowCount >= SENSOR_STUCK_CHECKS) {
    reportError(ERR_SENSOR_STUCK_HIGH, "Sensor C (GPIO17) reads LOW continuously");
    sensorCLowCount = 0;
  }
}

// ═══════════════════════════════════════════════════════════════════════════
//  DIAGNOSTICS — full system status dump
// ═══════════════════════════════════════════════════════════════════════════

void printStatus() {
  const char* STATE_NAMES[] = {
    "IDLE", "LOCKED", "DELAY", "SWITCH_PULSE", "RAMP"
  };

  unsigned long uptime = millis();
  unsigned long upSec  = uptime / 1000;
  unsigned long upMin  = upSec / 60;
  unsigned long upHr   = upMin / 60;

  Serial.println("\n╔══════════════════════════════════════════════════╗");
  Serial.println("║          SYSTEM DIAGNOSTICS                      ║");
  Serial.println("╠══════════════════════════════════════════════════╣");
  Serial.printf( "║  Uptime        : %02luh %02lum %02lus                      ║\n",
                 upHr, upMin % 60, upSec % 60);
  Serial.printf( "║  Total restarts: %-5lu  WDT resets: %-5lu         ║\n",
                 (unsigned long)totalRestarts, (unsigned long)wdtResets);
  Serial.println("╠══════════════════════════════════════════════════╣");
  Serial.printf( "║  BLE           : %s                  ║\n",
                 bleConnected ? "Connected ✅      " : "Disconnected ❌   ");
  Serial.printf( "║  BLE reconnects: %-5lu  Write errors: %-5lu       ║\n",
                 (unsigned long)bleReconnects, (unsigned long)bleWriteErrors);
  Serial.printf( "║  Zone state    : %-15s                 ║\n",
                 STATE_NAMES[zoneState]);
  Serial.printf( "║  Locked by     : %-20s            ║\n",
                 innerTrainCaused ? "Inner train" : "Outer train (or idle)");
  Serial.println("╠══════════════════════════════════════════════════╣");
  Serial.printf( "║  Outer speed   : %d/7                              ║\n",
                 currentSpeed);
  Serial.printf( "║  Inner power   : %-20s            ║\n",
                 digitalRead(RELAY_POWER_PIN) == RELAY_ENERGIZE
                 ? "CUT ⚠️" : "OK ✅");
  Serial.printf( "║  Switch relay  : %-20s            ║\n",
                 digitalRead(RELAY_SWITCH_PIN) == RELAY_ENERGIZE
                 ? "PULSING" : "idle");
  Serial.println("╠══════════════════════════════════════════════════╣");
  Serial.printf( "║  Stops sent    : %-5lu  Resumes sent : %-5lu       ║\n",
                 (unsigned long)stopsSent, (unsigned long)resumesSent);
  Serial.printf( "║  Zone timeouts : %-5lu                             ║\n",
                 (unsigned long)zoneTimeouts);
  Serial.println("╠══════════════════════════════════════════════════╣");
  Serial.println("║  Sensor state  :                                 ║");
  Serial.printf( "║    A (GPIO16)  : %s               ║\n",
                 digitalRead(SENSOR_A_PIN) == LOW ? "LOW  — beam blocked ⚠️ " : "HIGH — beam clear  ✅");
  Serial.printf( "║    B (GPIO15)  : %s               ║\n",
                 digitalRead(SENSOR_B_PIN) == LOW ? "LOW  — beam blocked ⚠️ " : "HIGH — beam clear  ✅");
  Serial.printf( "║    C (GPIO17)  : %s               ║\n",
                 digitalRead(SENSOR_C_PIN) == LOW ? "LOW  — beam blocked ⚠️ " : "HIGH — beam clear  ✅");
  Serial.println("╠══════════════════════════════════════════════════╣");
  Serial.println("║  Error counts  :                                 ║");
  for (int i = 1; i <= 9; i++) {
    if (errorCounts[i] > 0) {
      Serial.printf("║    [%02d] %-30s : %-4lu  ║\n",
                    i, ERROR_MESSAGES[i], (unsigned long)errorCounts[i]);
    }
  }
  if (zoneState == STATE_LOCKED) {
    unsigned long elapsed   = (millis() - lockTimer) / 1000;
    unsigned long remaining = (ZONE_TIMEOUT_MS / 1000) - elapsed;
    Serial.printf("║  Lock timeout  : %lus elapsed, %lus remaining        ║\n",
                  elapsed, remaining);
  }
  Serial.println("╚══════════════════════════════════════════════════╝\n");
}

void printMenu() {
  Serial.println("\n╔══════════════════════════════════════════════╗");
  Serial.println("║  Harry Locomotive — Serial Monitor Commands  ║");
  Serial.println("╠══════════════════════════════════════════════╣");
  Serial.println("║  OUTER TRAIN (BLE)                           ║");
  Serial.println("║  s/e = STOP/EMERGENCY    f = FORWARD         ║");
  Serial.println("║  r = REVERSE  +/- = Speed  1-7 = Set speed   ║");
  Serial.println("╠══════════════════════════════════════════════╣");
  Serial.println("║  SOUNDS                                      ║");
  Serial.println("║  h=Horn  b=Bell  l=Lights  a=Announce        ║");
  Serial.println("║  n=SoundON  m=SoundOFF  v=VolumeNext         ║");
  Serial.println("╠══════════════════════════════════════════════╣");
  Serial.println("║  INNER TRAIN (RELAY MANUAL)                  ║");
  Serial.println("║  x = Power ON    z = Power OFF               ║");
  Serial.println("║  p = Track switch pulse (1 second)           ║");
  Serial.println("╠══════════════════════════════════════════════╣");
  Serial.println("║  i = Full diagnostics    ? = This menu       ║");
  Serial.println("╚══════════════════════════════════════════════╝\n");
}

// ── Serial command handler ─────────────────────────────────────────────────
void handleSerial(char cmd) {
  switch (cmd) {
    case 's':
    case 'e':
      currentSpeed = 0;
      bleSendStop();
      break;
    case 'f': sendBLE(CMD_FORWARD,  3, "FORWARD");  break;
    case 'r': sendBLE(CMD_REVERSE,  3, "REVERSE");  break;
    case '+': {
      if (currentSpeed < 7) currentSpeed++;
      uint8_t c[] = {0x00, 0x45, (uint8_t)currentSpeed};
      char l[20]; snprintf(l, sizeof(l), "SPEED %d", currentSpeed);
      sendBLE(c, 3, l);
      break;
    }
    case '-': {
      if (currentSpeed > 0) currentSpeed--;
      uint8_t c[] = {0x00, 0x45, (uint8_t)currentSpeed};
      char l[20]; snprintf(l, sizeof(l), "SPEED %d", currentSpeed);
      sendBLE(c, 3, l);
      break;
    }
    case '1': currentSpeed=1; sendBLE(CMD_SPEED_1,3,"SPEED 1"); break;
    case '2': currentSpeed=2; sendBLE(CMD_SPEED_2,3,"SPEED 2"); break;
    case '3': currentSpeed=3; sendBLE(CMD_SPEED_3,3,"SPEED 3"); break;
    case '4': currentSpeed=4; sendBLE(CMD_SPEED_4,3,"SPEED 4"); break;
    case '5': currentSpeed=5; sendBLE(CMD_SPEED_5,3,"SPEED 5"); break;
    case '6': currentSpeed=6; sendBLE(CMD_SPEED_6,3,"SPEED 6"); break;
    case '7': currentSpeed=7; sendBLE(CMD_SPEED_7,3,"SPEED 7"); break;
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
      if (pChar) { pChar->writeValue(CMD_ANNOUNCE, 4, false); }
      Serial.println("[BLE] ▶ ANNOUNCE");
      break;
    case 'n': sendBLE(CMD_SOUND_ON,  3, "SOUND ON (max vol)"); volumeLevel=7; break;
    case 'm': sendBLE(CMD_SOUND_OFF, 3, "SOUND OFF");          volumeLevel=0; break;
    case 'v': {
      volumeLevel = (volumeLevel < 7) ? volumeLevel + 2 : 1;
      uint8_t vc[] = {0x00, 0x4C, (uint8_t)volumeLevel};
      char vl[20]; snprintf(vl, sizeof(vl), "VOLUME %d", volumeLevel);
      sendBLE(vc, 3, vl);
      break;
    }
    case 'x': innerTrainPowerRestore(); break;
    case 'z': innerTrainPowerCut();     break;
    case 'p': trackSwitchPulseStart(); switchTimer = millis(); break;
    case 'i': printStatus(); break;
    case '?': printMenu();   break;
    default:
      Serial.printf("[CMD] Unknown command '%c' — press ? for menu\n", cmd);
      break;
  }
}

// ═══════════════════════════════════════════════════════════════════════════
//  SETUP
// ═══════════════════════════════════════════════════════════════════════════

void setup() {
  Serial.begin(115200);
  delay(500);

  totalRestarts++;

  // Check if we were reset by the hardware watchdog
  esp_reset_reason_t resetReason = esp_reset_reason();
  if (resetReason == ESP_RST_TASK_WDT || resetReason == ESP_RST_WDT) {
    wdtResets++;
    reportError(ERR_WATCHDOG_RESET, "System was reset by hardware watchdog");
  }

  Serial.println("\n╔══════════════════════════════════════════════════╗");
  Serial.println("║  Harry Locomotive — Collision Prevention v4.1   ║");
  Serial.println("║  Datix AI  |  Ahmed Ali  |  April 2026          ║");
  Serial.println("╠══════════════════════════════════════════════════╣");
  Serial.printf( "║  Boot #%-5lu  WDT resets: %-5lu                  ║\n",
                 (unsigned long)totalRestarts, (unsigned long)wdtResets);
  Serial.printf( "║  Target MAC: %-36s║\n", TARGET_MAC);
  Serial.println("╚══════════════════════════════════════════════════╝\n");

  // ── Hardware watchdog ────────────────────────────────────────────
  // Will reset the ESP32 if loop() hangs for WATCHDOG_TIMEOUT_S seconds.
  // Must call esp_task_wdt_reset() regularly in loop().
  esp_task_wdt_init(WATCHDOG_TIMEOUT_S, true);
  esp_task_wdt_add(NULL);
  Serial.printf("[WDT] Hardware watchdog armed — %ds timeout\n",
                WATCHDOG_TIMEOUT_S);

  // ── GPIO: Status LED ─────────────────────────────────────────────
  pinMode(STATUS_LED, OUTPUT);
  digitalWrite(STATUS_LED, LOW);

  // ── GPIO: Relay 1 — inner train power ────────────────────────────
  // Default RELEASE = NC closed = inner train has track power on boot.
  // The relay should NOT be energized on startup.
  pinMode(RELAY_POWER_PIN, OUTPUT);
  digitalWrite(RELAY_POWER_PIN, RELAY_RELEASE);
  Serial.printf("[GPIO] Relay 1 (inner power)  GPIO%-2d — RELEASED (train has power)\n",
                RELAY_POWER_PIN);

  // ── GPIO: Relay 2 — track switch ─────────────────────────────────
  // Default RELEASE = NO open = switch contacts open = switch untouched.
  pinMode(RELAY_SWITCH_PIN, OUTPUT);
  digitalWrite(RELAY_SWITCH_PIN, RELAY_RELEASE);
  Serial.printf("[GPIO] Relay 2 (track switch) GPIO%-2d — RELEASED (switch idle)\n",
                RELAY_SWITCH_PIN);

  // ── GPIO: IR sensors ─────────────────────────────────────────────
  // INPUT_PULLUP: pin reads HIGH when beam is clear, LOW when beam broken.
  // FALLING interrupt fires the moment a train enters each sensor's beam.
  pinMode(SENSOR_A_PIN, INPUT_PULLUP);
  pinMode(SENSOR_B_PIN, INPUT_PULLUP);
  pinMode(SENSOR_C_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(SENSOR_A_PIN), ISR_SensorA, FALLING);
  attachInterrupt(digitalPinToInterrupt(SENSOR_B_PIN), ISR_SensorB, FALLING);
  attachInterrupt(digitalPinToInterrupt(SENSOR_C_PIN), ISR_SensorC, FALLING);
  Serial.printf("[GPIO] Sensor A (inner entry) GPIO%d\n", SENSOR_A_PIN);
  Serial.printf("[GPIO] Sensor B (outer entry) GPIO%d\n", SENSOR_B_PIN);
  Serial.printf("[GPIO] Sensor C (exit)         GPIO%d\n", SENSOR_C_PIN);

  // Initial sensor health check — warn if any sensor reads LOW on boot
  delay(100);
  if (digitalRead(SENSOR_A_PIN) == LOW)
    reportError(ERR_SENSOR_STUCK_HIGH, "Sensor A reads LOW at boot — check wiring");
  if (digitalRead(SENSOR_B_PIN) == LOW)
    reportError(ERR_SENSOR_STUCK_HIGH, "Sensor B reads LOW at boot — check wiring");
  if (digitalRead(SENSOR_C_PIN) == LOW)
    reportError(ERR_SENSOR_STUCK_HIGH, "Sensor C reads LOW at boot — check wiring");

  // ── BLE: Connect to outer train ──────────────────────────────────
  BLEDevice::init("TrainController");

  Serial.println("\n[BLE] Step 1: Connecting by direct MAC address...");
  bool ok = connectToTrain(TARGET_MAC);

  if (!ok) {
    Serial.printf("[BLE] Step 2: MAC failed — scanning for prefix '%s'...\n",
                  TRAIN_NAME_PREFIX);
    BLEScan* pScan = BLEDevice::getScan();
    pScan->setAdvertisedDeviceCallbacks(new ScanCallback());
    pScan->setActiveScan(true);
    pScan->start(15, false);

    if (foundMAC.length() > 0) {
      ok = connectToTrain(foundMAC.c_str());
    } else {
      reportError(ERR_BLE_SCAN_NO_RESULT,
                  "Is the train powered on and within BLE range?");
    }
  }

  if (ok) {
    Serial.println("\n[SYSTEM] ✅ Ready — monitoring shared section\n");
  } else {
    Serial.println("\n[SYSTEM] ⚠️  BLE not connected — "
                   "collision detection active but STOP cannot be sent\n"
                   "          Will reconnect automatically every 5 seconds\n");
  }

  printMenu();
}

// ═══════════════════════════════════════════════════════════════════════════
//  MAIN LOOP
//  Must complete each iteration quickly — never call blocking functions here.
//  delay(5) at the bottom gives the BLE stack 5ms of CPU time per cycle.
// ═══════════════════════════════════════════════════════════════════════════

void loop() {
  unsigned long now = millis();

  // Feed the hardware watchdog — proves the loop is not hung
  esp_task_wdt_reset();

  // ── BLE: Auto-reconnect ──────────────────────────────────────────
  if (!bleConnected && (now - lastReconnect > BLE_RECONNECT_MS)) {
    lastReconnect = now;
    Serial.println("[BLE] Attempting reconnect...");

    // Priority: if zone is locked and reconnect fails, keep trying faster
    bool ok = connectToTrain(TARGET_MAC);
    if (!ok && foundMAC.length() > 0) {
      ok = connectToTrain(foundMAC.c_str());
    }
    if (!ok) {
      // Last resort: full BLE scan again
      BLEScan* pScan = BLEDevice::getScan();
      pScan->setAdvertisedDeviceCallbacks(new ScanCallback());
      pScan->setActiveScan(true);
      pScan->start(5, false);
      if (foundMAC.length() > 0) connectToTrain(foundMAC.c_str());
    }

    // If we reconnected while zone is locked, immediately re-send STOP
    if (bleConnected && zoneState != STATE_IDLE && innerTrainCaused) {
      Serial.println("[BLE] Reconnected during locked zone — re-sending STOP");
      bleSendStop();
      lastStopSent = now;
    }
  }

  // ── BLE: Keepalive ping ──────────────────────────────────────────
  // Prevents the LionChief from auto-disconnecting during long idle periods.
  // Only send during IDLE or RAMP so we don't interfere with STOP commands.
  if (bleConnected &&
      (zoneState == STATE_IDLE || zoneState == STATE_RAMP) &&
      (now - lastKeepalive > BLE_KEEPALIVE_MS)) {
    lastKeepalive = now;
    uint8_t c[] = {0x00, 0x45, (uint8_t)currentSpeed};
    sendBLE(c, 3, "keepalive ping");
  }

  // ── Serial: Manual commands ──────────────────────────────────────
  if (Serial.available()) {
    char cmd = Serial.read();
    if (cmd != '\n' && cmd != '\r') {
      Serial.printf("\n> '%c'\n", cmd);
      handleSerial(cmd);
    }
  }

  // ── Sensor health check (runs every ~50 loop cycles) ─────────────
  static uint8_t healthCheckCounter = 0;
  if (++healthCheckCounter >= 50) {
    healthCheckCounter = 0;
    checkSensorHealth();
  }

  // ── Manual track switch pulse — auto-end after SWITCH_PULSE_MS ───
  // Handles the 'p' manual command pulse timeout
  if (zoneState == STATE_IDLE &&
      digitalRead(RELAY_SWITCH_PIN) == RELAY_ENERGIZE &&
      (now - switchTimer >= SWITCH_PULSE_MS)) {
    trackSwitchPulseEnd();
  }

  // ════════════════════════════════════════════════════════════════
  //  ZONE STATE MACHINE — fully non-blocking, millis()-based
  // ════════════════════════════════════════════════════════════════

  switch (zoneState) {

    // ──────────────────────────────────────────────────────────────
    //  STATE_IDLE
    //  Both trains running normally. Monitor all sensors for entry.
    // ──────────────────────────────────────────────────────────────
    case STATE_IDLE: {

      if (sensorA_fired) {
        sensorA_fired = false;
        if (now - lastSensorA > SENSOR_DEBOUNCE_MS) {
          lastSensorA = now;
          Serial.println("\n[SENSOR A] ⚠️  Inner train entering shared section!");
          Serial.println("[ACTION]   BLE STOP sent to outer train");

          innerTrainCaused = true;
          zoneState        = STATE_LOCKED;
          lockTimer        = now;
          lastStopSent     = now;
          bleSendStop();
          currentSpeed = 0;
        }
      }

      if (sensorB_fired) {
        sensorB_fired = false;
        if (now - lastSensorB > SENSOR_DEBOUNCE_MS) {
          lastSensorB = now;
          Serial.println("\n[SENSOR B] ⚠️  Outer train entering shared section!");
          Serial.println("[ACTION]   Relay cutting inner train track power");

          innerTrainCaused = false;
          zoneState        = STATE_LOCKED;
          lockTimer        = now;
          innerTrainPowerCut();
        }
      }

      break;
    }

    // ──────────────────────────────────────────────────────────────
    //  STATE_LOCKED
    //  Zone occupied. Repeat STOP if needed. Wait for Sensor C exit.
    //  Timeout after ZONE_TIMEOUT_MS to prevent permanent deadlock.
    // ──────────────────────────────────────────────────────────────
    case STATE_LOCKED: {

      // Keep outer train stopped while inner train occupies the zone
      if (innerTrainCaused && (now - lastStopSent > STOP_REPEAT_MS)) {
        lastStopSent = now;
        bleSendStop();
      }

      // Train exit detected via Sensor C
      if (sensorC_fired) {
        sensorC_fired = false;
        if (now - lastSensorC > SENSOR_DEBOUNCE_MS) {
          lastSensorC = now;
          Serial.println("[SENSOR C] ✅ Train exiting shared section");
          Serial.printf( "[SYSTEM]   Safety delay %dms starting...\n",
                         RESUME_DELAY_MS);
          zoneState   = STATE_DELAY;
          resumeTimer = now;
        }
      }

      // Zone timeout — Sensor C did not fire within allowed time
      if (now - lockTimer > ZONE_TIMEOUT_MS) {
        zoneTimeouts++;
        reportError(ERR_ZONE_TIMEOUT,
                    innerTrainCaused
                    ? "Inner train may be stopped/derailed in zone"
                    : "Outer train may be stopped/derailed in zone");
        Serial.println("[TIMEOUT]  Force-resuming — safety delay starting");
        zoneState   = STATE_DELAY;
        resumeTimer = now;
      }

      break;
    }

    // ──────────────────────────────────────────────────────────────
    //  STATE_DELAY
    //  Zone cleared but trains need time to fully separate before
    //  the stopped train resumes. Continue sending STOP during delay.
    // ──────────────────────────────────────────────────────────────
    case STATE_DELAY: {

      if (innerTrainCaused && (now - lastStopSent > STOP_REPEAT_MS)) {
        lastStopSent = now;
        bleSendStop();
      }

      if (now - resumeTimer >= RESUME_DELAY_MS) {
        Serial.println("[SYSTEM]   Safety delay complete");
        Serial.println("[RELAY2]   Pulsing track switch to reset position...");
        trackSwitchPulseStart();
        zoneState   = STATE_SWITCH_PULSE;
        switchTimer = now;
      }

      break;
    }

    // ──────────────────────────────────────────────────────────────
    //  STATE_SWITCH_PULSE
    //  1-second relay pulse resets track switch to correct position.
    //  Continue holding outer train stopped during the pulse.
    // ──────────────────────────────────────────────────────────────
    case STATE_SWITCH_PULSE: {

      if (innerTrainCaused && (now - lastStopSent > STOP_REPEAT_MS)) {
        lastStopSent = now;
        bleSendStop();
      }

      if (now - switchTimer >= SWITCH_PULSE_MS) {
        trackSwitchPulseEnd();

        if (innerTrainCaused) {
          // Inner train was the problem → resume outer at slow speed
          Serial.println("[RESUME]   Inner cleared → outer train resuming slowly");
          bleResumeSlowly();
          currentSpeed = RESUME_SLOW_SPEED;
        } else {
          // Outer train was the problem → restore inner train track power
          Serial.println("[RESUME]   Outer cleared → inner train power restored");
          innerTrainPowerRestore();
        }

        zoneState = STATE_RAMP;
        rampTimer = now;
      }

      break;
    }

    // ──────────────────────────────────────────────────────────────
    //  STATE_RAMP
    //  Give outer train SPEED_RAMP_MS at slow speed before full speed.
    //  This prevents it from rushing immediately into the shared section.
    // ──────────────────────────────────────────────────────────────
    case STATE_RAMP: {

      if (now - rampTimer >= SPEED_RAMP_MS) {
        if (innerTrainCaused) {
          bleResumeFull();
          currentSpeed = RESUME_FULL_SPEED;
        }
        // (If outer caused the lock, inner train already restored in SWITCH_PULSE)

        // Clear any ISR flags that fired during the resume sequence
        sensorA_fired    = false;
        sensorB_fired    = false;
        sensorC_fired    = false;
        innerTrainCaused = false;
        zoneState        = STATE_IDLE;

        Serial.println("[ZONE]     ✅ UNLOCKED — both trains free\n");
      }

      break;
    }
  }

  // ── Heartbeat LED ────────────────────────────────────────────────
  // Slow 40ms blink every 2 seconds = system alive and in IDLE state.
  // LED stays solid when BLE is connected, off when disconnected.
  static unsigned long lastBlink   = 0;
  static unsigned long ledOnTime   = 0;
  static bool          ledBlinking = false;

  if (zoneState == STATE_IDLE && bleConnected) {
    if (!ledBlinking && (now - lastBlink > 2000)) {
      lastBlink   = now;
      ledOnTime   = now;
      ledBlinking = true;
      digitalWrite(STATUS_LED, HIGH);
    }
    if (ledBlinking && (now - ledOnTime > 40)) {
      ledBlinking = false;
      digitalWrite(STATUS_LED, LOW);
    }
  }

  // Small cooperative yield — gives BLE stack CPU time each loop cycle.
  // 5ms is safe: state machine timers are all in the hundreds-to-thousands
  // of ms range, so 5ms granularity has no practical impact on precision.
  delay(5);
}