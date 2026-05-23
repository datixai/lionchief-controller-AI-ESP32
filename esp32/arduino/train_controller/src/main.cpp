/*
 * ═══════════════════════════════════════════════════════════════════════════
 *  train_controller/src/main.cpp
 *  Harry Locomotive Project — Autonomous Collision Prevention System
 *  Datix AI  |  Ahmed Ali  |  v5.0  |  May 2026
 *
 * ─── WHAT CHANGED IN v5 ────────────────────────────────────────────────────
 *
 *  1. INNER TRAIN — ONE LOOP THEN AUTO STOP
 *     Inner train is OFF by default. Press X to run one loop.
 *     After Sensor C fires (inner exits shared section), a configurable
 *     PARKING_DELAY_MS counts down, then relay cuts power automatically.
 *     Inner train stops at parking position. Does not run again until X pressed.
 *
 *  2. INNER TRAIN BLOCKED IF OUTER IS IN SHARED SECTION
 *     Pressing X while zone is active prints a warning and does nothing.
 *     Inner train only starts when zone is completely clear (STATE_IDLE).
 *
 *  3. TRACK SWITCH PULSE — ONLY FOR OUTER TRAIN EXIT
 *     Relay 2 (GPIO19) now ONLY pulses when the OUTER train (Sensor B)
 *     triggered the lock and has exited. Inner train exit does NOT pulse
 *     the track switch (Peter confirmed this is correct).
 *
 *  4. RELAY BOOT FIX
 *     Both relay pins are set HIGH (released) as the very first lines
 *     in setup(), before Serial.begin() even runs. This prevents the
 *     brief GPIO float that was energizing relays at power-on.
 *
 *  5. INNER TRAIN POWER RESTORED AFTER OUTER EXIT
 *     If the outer train entered the zone while the inner train was
 *     running (cut its power), inner train power is automatically
 *     restored after the outer train exits and switch pulse completes.
 *
 * ─── STATE MACHINE (6 states) ──────────────────────────────────────────────
 *
 *   ┌──────────────────────────────────────────────────────────────────────┐
 *   │                                                                      │
 *   │  IDLE ──[SensorA]──► LOCKED(inner) ──[SensorC]──► INNER_PARKING    │
 *   │    │                                                      │          │
 *   │    │                                             [parking delay]     │
 *   │    │                                                      ▼          │
 *   │    │                                                   DELAY         │
 *   │    │                                                      │          │
 *   │    └──[SensorB]──► LOCKED(outer) ──[SensorC]──► DELAY   │          │
 *   │                                                      │    │          │
 *   │                                             [outer only] │          │
 *   │                                                      ▼    ▼          │
 *   │              IDLE ◄── RAMP ◄── SWITCH_PULSE ◄──────────────        │
 *   │                                                                      │
 *   └──────────────────────────────────────────────────────────────────────┘
 *
 *   STATE_IDLE           Outer running, inner parked — monitoring sensors
 *   STATE_LOCKED         Zone occupied — outer stopped OR inner power cut
 *   STATE_INNER_PARKING  Inner exited zone — counting down to parking stop
 *   STATE_DELAY          Safety buffer before resuming outer train
 *   STATE_SWITCH_PULSE   1s relay pulse resets track switch (outer exit only)
 *   STATE_RAMP           Outer train ramping slow → full speed
 *
 * ─── WIRING (unchanged from v4) ────────────────────────────────────────────
 *
 *   GPIO16 → Sensor A  (inner loop entry — inner train only)
 *   GPIO15 → Sensor B  (outer loop entry — outer train only)
 *   GPIO17 → Sensor C  (shared section exit — any train)
 *   GPIO18 → Relay 1   (inner train track power — NC terminal)
 *   GPIO19 → Relay 2   (track switch 1s pulse — NO terminal)
 *   VIN    → All sensor VCC + both relay VCC
 *   GND    → All sensor GND + both relay GND
 *   GPIO2  → Built-in status LED
 *
 * ─── RELAY WIRING ──────────────────────────────────────────────────────────
 *
 *   Relay module = ACTIVE LOW (coil energizes when IN pin = LOW)
 *
 *   Relay 1 — inner train power (NC terminal):
 *     HIGH = released = NC closed = power flows = train runs
 *     LOW  = energized = NC open  = power cut   = train stops
 *
 *   Relay 2 — track switch (NO terminal, own 18V AC supply):
 *     HIGH = released = NO open   = switch untouched (default)
 *     LOW  = energized = NO closes = switch resets (1s pulse only)
 *
 * ─── CONFIRMED BLE DETAILS ─────────────────────────────────────────────────
 *   MAC  : CC:01:78:D0:F0:99    Name: LC015556-99F0
 *   SVC  : e20a39f4-73f5-4bc4-a12f-17d1ad07a961
 *   CHAR : 08590f7e-db05-467e-8757-72f6faeb13d4
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
//  CONFIGURATION
//  Adjust these values to tune system behaviour without changing logic.
// ═══════════════════════════════════════════════════════════════════════════

// BLE — outer LionChief train
#define TARGET_MAC          "CC:01:78:D0:F0:99"
#define TRAIN_NAME_PREFIX   "LC0"
#define SERVICE_UUID        "e20a39f4-73f5-4bc4-a12f-17d1ad07a961"
#define CHARACTERISTIC_UUID "08590f7e-db05-467e-8757-72f6faeb13d4"

// GPIO — confirmed wiring from v4, unchanged
#define SENSOR_A_PIN     16   // Inner loop entry (inner train only)
#define SENSOR_B_PIN     15   // Outer loop entry (outer train only)
#define SENSOR_C_PIN     17   // Shared section exit (any train)
#define RELAY_POWER_PIN  18   // Relay 1: inner train track power
#define RELAY_SWITCH_PIN 19   // Relay 2: track switch 1s pulse
#define STATUS_LED        2   // Built-in LED

// Relay polarity — standard Arduino relay module is ACTIVE LOW
#define RELAY_ENERGIZE  LOW    // Coil ON  → NC opens / NO closes
#define RELAY_RELEASE   HIGH   // Coil OFF → NC closed / NO open (safe default)

// ── Timing — all in milliseconds ───────────────────────────────────────────
#define PARKING_DELAY_MS    2000   // ★ TUNE THIS: time inner train takes to
                                   //   reach parking spot AFTER Sensor C fires.
                                   //   Increase if train overshoots parking.
                                   //   Decrease if train stops too early.

#define RESUME_DELAY_MS     2000   // Safety buffer after zone clears
#define SPEED_RAMP_MS       3000   // Time at slow speed before full speed
#define STOP_REPEAT_MS       500   // Repeat BLE STOP every 500ms while locked
#define SWITCH_PULSE_MS     1000   // Track switch relay pulse duration
#define ZONE_TIMEOUT_MS    30000   // Max lock time — force resume if Sensor C fails
#define SENSOR_DEBOUNCE_MS   300   // Min ms between same-sensor triggers
#define BLE_RECONNECT_MS    5000   // BLE reconnect attempt interval
#define BLE_KEEPALIVE_MS   20000   // BLE keepalive ping interval
#define WATCHDOG_TIMEOUT_S    60   // Hardware WDT — resets ESP32 if loop hangs

// Resume speed levels (0-7)
#define RESUME_SLOW_SPEED   2      // Initial resume speed after stop
#define RESUME_FULL_SPEED   7      // Full speed after ramp delay

// ═══════════════════════════════════════════════════════════════════════════
//  BLE COMMANDS — all confirmed on Peter's actual LionChief locomotive
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
//  ERROR CODES — every failure has a unique searchable label
// ═══════════════════════════════════════════════════════════════════════════

enum ErrorCode {
  ERR_NONE = 0,
  ERR_BLE_CONNECT_FAILED,      // connect() returned false
  ERR_BLE_SERVICE_NOT_FOUND,   // LionChief service UUID not on device
  ERR_BLE_CHAR_NOT_FOUND,      // Write characteristic not found
  ERR_BLE_WRITE_FAILED,        // GATT write threw exception
  ERR_BLE_NOT_CONNECTED,       // Command sent while disconnected
  ERR_BLE_SCAN_NO_RESULT,      // Name scan found nothing
  ERR_SENSOR_STUCK_LOW,        // Sensor reads LOW continuously
  ERR_ZONE_TIMEOUT,            // Zone locked 30s with no Sensor C
  ERR_RELAY_STUCK,             // Relay GPIO did not respond
  ERR_WATCHDOG_RESET,          // Hardware WDT triggered a reset
  ERR_INNER_BLOCKED,           // X pressed but zone was active
};

const char* ERROR_MESSAGES[] = {
  "No error",
  "BLE connect() failed — train off or out of range",
  "BLE service UUID not found — wrong device?",
  "BLE characteristic not found — UUID mismatch",
  "BLE GATT write failed — connection dropped mid-send",
  "BLE command ignored — not connected",
  "BLE scan found no LionChief device with prefix LC0",
  "IR sensor reads LOW continuously — check wiring/alignment",
  "Zone timeout — Sensor C did not fire within 30s, force-resuming",
  "Relay GPIO did not respond to write",
  "System reset by hardware watchdog — possible loop hang",
  "Inner train start blocked — zone is active, wait for clear",
};

// Persisted across soft resets via RTC RAM
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

// BLE handles
BLEClient*               pClient      = nullptr;
BLERemoteCharacteristic* pChar        = nullptr;
bool                     bleConnected = false;
String                   foundMAC     = "";

// ── Zone state machine ─────────────────────────────────────────────────────
enum ZoneState {
  STATE_IDLE,            // Outer running, inner parked — all clear
  STATE_LOCKED,          // Zone occupied — collision prevention active
  STATE_INNER_PARKING,   // Inner exited zone — coasting to parking spot
  STATE_DELAY,           // Safety buffer before resuming outer train
  STATE_SWITCH_PULSE,    // Pulsing track switch relay (outer exit only)
  STATE_RAMP             // Outer train ramping slow → full speed
};

ZoneState zoneState        = STATE_IDLE;
bool      innerTrainCaused = false;   // true = inner triggered lock
                                      // false = outer triggered lock

// ── Inner train one-loop tracking ─────────────────────────────────────────
bool innerTrainActive     = false;   // true = inner train is running its loop
bool innerWasCutForOuter  = false;   // true = inner was cut because outer entered
                                     //        restore power after outer exits

// ── Non-blocking timers ────────────────────────────────────────────────────
unsigned long lockTimer     = 0;
unsigned long parkingTimer  = 0;   // when Sensor C fired for inner train exit
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
int  currentSpeed = 0;
int  volumeLevel  = 7;

// ── Statistics ─────────────────────────────────────────────────────────────
uint32_t stopsSent      = 0;
uint32_t resumesSent    = 0;
uint32_t bleWriteErrors = 0;
uint32_t zoneTimeouts   = 0;
uint32_t bleReconnects  = 0;
uint32_t innerLoopsRun  = 0;

// ═══════════════════════════════════════════════════════════════════════════
//  INTERRUPT SERVICE ROUTINES
//  IRAM_ATTR = stored in RAM for guaranteed fast execution.
//  E18-D80NK outputs HIGH when beam clear, LOW when beam blocked.
//  FALLING edge = train just broke the beam.
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
//  BLE CALLBACKS
// ═══════════════════════════════════════════════════════════════════════════

class TrainClientCallbacks : public BLEClientCallbacks {
  void onConnect(BLEClient* client) {
    bleConnected = true;
    bleReconnects++;
    digitalWrite(STATUS_LED, HIGH);
    Serial.printf("[BLE] ✅ Connected to outer train (reconnect #%lu)\n",
                  (unsigned long)bleReconnects);
  }
  void onDisconnect(BLEClient* client) {
    bleConnected = false;
    pChar        = nullptr;
    digitalWrite(STATUS_LED, LOW);
    Serial.println("[BLE] ⚠️  Disconnected — auto-reconnect in 5s");
    if (zoneState != STATE_IDLE)
      Serial.println("[BLE] ⚠️  Disconnected while zone LOCKED — reconnecting urgently");
  }
};

class ScanCallback : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice device) {
    String name = device.getName().c_str();
    String addr = device.getAddress().toString().c_str();
    if (name.length() > 0)
      Serial.printf("[BLE] Scan: %-28s [%s]\n", name.c_str(), addr.c_str());
    if (name.startsWith(TRAIN_NAME_PREFIX)) {
      Serial.printf("[BLE] ✅ Found LionChief: %s [%s]\n",
                    name.c_str(), addr.c_str());
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
//  Every write is wrapped in try-catch — connection drop mid-send is caught.
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

// Convenience wrappers
bool bleSendStop()     { stopsSent++;   return sendBLE(CMD_STOP,    3, "STOP outer train"); }
bool bleResumeSlow()   { resumesSent++; return sendBLE(CMD_SPEED_2, 3, "RESUME outer — slow"); }
bool bleResumeFull()   {               return sendBLE(CMD_SPEED_7, 3, "RESUME outer — full"); }

// ═══════════════════════════════════════════════════════════════════════════
//  RELAY CONTROL
//  GPIO state is read back after every write to detect faults.
// ═══════════════════════════════════════════════════════════════════════════

void innerTrainPowerCut() {
  digitalWrite(RELAY_POWER_PIN, RELAY_ENERGIZE);
  if (digitalRead(RELAY_POWER_PIN) != RELAY_ENERGIZE)
    reportError(ERR_RELAY_STUCK, "RELAY_POWER_PIN did not set to ENERGIZE");
  Serial.println("[RELAY1] 🛑 Inner train STOPPED — track power cut");
}

void innerTrainPowerRestore() {
  digitalWrite(RELAY_POWER_PIN, RELAY_RELEASE);
  if (digitalRead(RELAY_POWER_PIN) != RELAY_RELEASE)
    reportError(ERR_RELAY_STUCK, "RELAY_POWER_PIN did not release");
  Serial.println("[RELAY1] ✅ Inner train RUNNING — track power restored");
}

// Relay 2 — only called when OUTER train exits (innerTrainCaused == false)
void trackSwitchPulseStart() {
  digitalWrite(RELAY_SWITCH_PIN, RELAY_ENERGIZE);
  Serial.printf("[RELAY2] Track switch PULSING — %dms\n", SWITCH_PULSE_MS);
}

void trackSwitchPulseEnd() {
  digitalWrite(RELAY_SWITCH_PIN, RELAY_RELEASE);
  Serial.println("[RELAY2] Track switch pulse DONE — switch position reset");
}

// ═══════════════════════════════════════════════════════════════════════════
//  SENSOR HEALTH CHECK
//  Runs every 50 loop cycles. Reports if any sensor is stuck LOW.
// ═══════════════════════════════════════════════════════════════════════════

void checkSensorHealth() {
  sensorALowCount = (digitalRead(SENSOR_A_PIN) == LOW)
    ? (sensorALowCount < 255 ? sensorALowCount + 1 : 255) : 0;
  sensorBLowCount = (digitalRead(SENSOR_B_PIN) == LOW)
    ? (sensorBLowCount < 255 ? sensorBLowCount + 1 : 255) : 0;
  sensorCLowCount = (digitalRead(SENSOR_C_PIN) == LOW)
    ? (sensorCLowCount < 255 ? sensorCLowCount + 1 : 255) : 0;

  if (sensorALowCount >= SENSOR_STUCK_CHECKS) {
    reportError(ERR_SENSOR_STUCK_LOW, "Sensor A (GPIO16) stuck LOW — check alignment");
    sensorALowCount = 0;
  }
  if (sensorBLowCount >= SENSOR_STUCK_CHECKS) {
    reportError(ERR_SENSOR_STUCK_LOW, "Sensor B (GPIO15) stuck LOW — check alignment");
    sensorBLowCount = 0;
  }
  if (sensorCLowCount >= SENSOR_STUCK_CHECKS) {
    reportError(ERR_SENSOR_STUCK_LOW, "Sensor C (GPIO17) stuck LOW — check alignment");
    sensorCLowCount = 0;
  }
}

// ═══════════════════════════════════════════════════════════════════════════
//  DIAGNOSTICS
// ═══════════════════════════════════════════════════════════════════════════

void printStatus() {
  const char* SNAMES[] = {
    "IDLE","LOCKED","INNER_PARKING","DELAY","SWITCH_PULSE","RAMP"
  };
  unsigned long upSec = millis() / 1000;

  Serial.println("\n╔════════════════════════════════════════════════════╗");
  Serial.println("║             SYSTEM DIAGNOSTICS v5                 ║");
  Serial.println("╠════════════════════════════════════════════════════╣");
  Serial.printf( "║  Uptime        : %02luh %02lum %02lus\n",
                 upSec/3600, (upSec%3600)/60, upSec%60);
  Serial.printf( "║  Restarts      : %-5lu   WDT resets: %-5lu\n",
                 (unsigned long)totalRestarts, (unsigned long)wdtResets);
  Serial.println("╠════════════════════════════════════════════════════╣");
  Serial.printf( "║  BLE           : %s\n",
                 bleConnected ? "Connected ✅" : "Disconnected ❌");
  Serial.printf( "║  BLE reconnects: %-5lu   Write errors: %-5lu\n",
                 (unsigned long)bleReconnects, (unsigned long)bleWriteErrors);
  Serial.println("╠════════════════════════════════════════════════════╣");
  Serial.printf( "║  Zone state    : %s\n", SNAMES[zoneState]);
  Serial.printf( "║  Locked by     : %s\n",
                 innerTrainCaused ? "Inner train" : "Outer train / idle");
  Serial.printf( "║  Inner active  : %s\n",
                 innerTrainActive ? "YES — running loop" : "NO — parked");
  Serial.printf( "║  Inner cut for outer: %s\n",
                 innerWasCutForOuter ? "YES — will restore after outer exits" : "NO");
  Serial.println("╠════════════════════════════════════════════════════╣");
  Serial.printf( "║  Outer speed   : %d/7\n", currentSpeed);
  Serial.printf( "║  Relay 1 power : %s\n",
                 digitalRead(RELAY_POWER_PIN) == RELAY_ENERGIZE
                 ? "CUT ⚠️" : "OK — power flowing ✅");
  Serial.printf( "║  Relay 2 switch: %s\n",
                 digitalRead(RELAY_SWITCH_PIN) == RELAY_ENERGIZE
                 ? "PULSING" : "idle");
  Serial.println("╠════════════════════════════════════════════════════╣");
  Serial.printf( "║  Stops sent    : %-5lu   Resumes: %-5lu\n",
                 (unsigned long)stopsSent, (unsigned long)resumesSent);
  Serial.printf( "║  Inner loops   : %-5lu   Timeouts: %-5lu\n",
                 (unsigned long)innerLoopsRun, (unsigned long)zoneTimeouts);
  Serial.println("╠════════════════════════════════════════════════════╣");
  Serial.printf( "║  Sensor A GPIO16: %s\n",
                 digitalRead(SENSOR_A_PIN)==LOW ? "LOW — blocked ⚠️" : "HIGH — clear ✅");
  Serial.printf( "║  Sensor B GPIO15: %s\n",
                 digitalRead(SENSOR_B_PIN)==LOW ? "LOW — blocked ⚠️" : "HIGH — clear ✅");
  Serial.printf( "║  Sensor C GPIO17: %s\n",
                 digitalRead(SENSOR_C_PIN)==LOW ? "LOW — blocked ⚠️" : "HIGH — clear ✅");
  Serial.println("╠════════════════════════════════════════════════════╣");
  bool anyErr = false;
  for (int i = 1; i <= 11; i++) {
    if (errorCounts[i] > 0) {
      Serial.printf("║  ERR[%02d] x%-3lu %s\n",
                    i, (unsigned long)errorCounts[i], ERROR_MESSAGES[i]);
      anyErr = true;
    }
  }
  if (!anyErr) Serial.println("║  No errors recorded ✅");
  Serial.println("╚════════════════════════════════════════════════════╝\n");
}

void printMenu() {
  Serial.println("\n╔══════════════════════════════════════════════════╗");
  Serial.println("║   Harry Locomotive v5 — Serial Commands          ║");
  Serial.println("╠══════════════════════════════════════════════════╣");
  Serial.println("║  OUTER TRAIN (BLE)                               ║");
  Serial.println("║  s/e=STOP   f=FORWARD   r=REVERSE               ║");
  Serial.println("║  1-7=Speed  +=SpeedUP   -=SpeedDOWN             ║");
  Serial.println("╠══════════════════════════════════════════════════╣");
  Serial.println("║  SOUNDS                                          ║");
  Serial.println("║  h=Horn  b=Bell  l=Lights  a=Announce           ║");
  Serial.println("║  n=SoundON  m=SoundOFF  v=VolumeNext            ║");
  Serial.println("╠══════════════════════════════════════════════════╣");
  Serial.println("║  INNER TRAIN (ONE-LOOP MODE)                     ║");
  Serial.println("║  X = Start inner train — runs ONE loop then stops║");
  Serial.println("║      Blocked if outer train is in shared section ║");
  Serial.println("║  z = Force cut inner power (emergency)           ║");
  Serial.println("║  p = Manual track switch pulse (1 second)        ║");
  Serial.println("╠══════════════════════════════════════════════════╣");
  Serial.println("║  i = Full diagnostics    ? = This menu           ║");
  Serial.println("╚══════════════════════════════════════════════════╝\n");
}

// ═══════════════════════════════════════════════════════════════════════════
//  SERIAL COMMAND HANDLER
// ═══════════════════════════════════════════════════════════════════════════

void handleSerial(char cmd) {
  switch (cmd) {

    // ── Outer train ────────────────────────────────────────────────
    case 's': case 'e':
      currentSpeed = 0;
      bleSendStop();
      break;
    case 'f': sendBLE(CMD_FORWARD, 3, "FORWARD"); break;
    case 'r': sendBLE(CMD_REVERSE, 3, "REVERSE"); break;
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

    // ── Sounds ─────────────────────────────────────────────────────
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

    // ── Inner train ────────────────────────────────────────────────
    case 'X': case 'x':
      // Only allowed when zone is completely clear
      if (zoneState != STATE_IDLE) {
        reportError(ERR_INNER_BLOCKED,
                    "Zone is active — wait for outer train to clear shared section");
        Serial.println("[INNER] ❌ Cannot start — wait for zone to clear then press X again");
      } else if (innerTrainActive) {
        Serial.println("[INNER] ⚠️  Inner train is already running its loop");
      } else {
        innerTrainActive    = false;  // will be set true below
        innerWasCutForOuter = false;
        innerLoopsRun++;
        innerTrainActive = true;
        innerTrainPowerRestore();     // give inner train track power
        Serial.printf("[INNER] 🚂 Inner train STARTED — loop #%lu\n",
                      (unsigned long)innerLoopsRun);
        Serial.printf("[INNER] Will auto-stop %dms after Sensor C fires\n",
                      PARKING_DELAY_MS);
      }
      break;

    case 'z':
      // Emergency cut inner power regardless of state
      innerTrainActive    = false;
      innerWasCutForOuter = false;
      innerTrainPowerCut();
      Serial.println("[INNER] ⚠️  Emergency STOP — inner train power cut");
      break;

    case 'p':
      // Manual track switch pulse — only in IDLE
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
//  ⚠️  RELAY PINS SET HIGH FIRST — before Serial.begin() or anything else.
//  This prevents the brief GPIO float during boot from energizing the relays.
// ═══════════════════════════════════════════════════════════════════════════

void setup() {

  // ── RELAY BOOT FIX — must be absolute first lines ─────────────────
  // GPIO floats LOW briefly during ESP32 boot before setup() runs.
  // Setting HIGH here as early as possible prevents relays from firing.
  pinMode(RELAY_POWER_PIN,  OUTPUT);
  pinMode(RELAY_SWITCH_PIN, OUTPUT);
  digitalWrite(RELAY_POWER_PIN,  RELAY_RELEASE);   // HIGH = NC closed = inner has power
  digitalWrite(RELAY_SWITCH_PIN, RELAY_RELEASE);   // HIGH = NO open   = switch idle
  // ─────────────────────────────────────────────────────────────────

  Serial.begin(115200);
  delay(400);
  totalRestarts++;

  // Check for hardware WDT reset
  esp_reset_reason_t reason = esp_reset_reason();
  if (reason == ESP_RST_TASK_WDT || reason == ESP_RST_WDT) {
    wdtResets++;
    reportError(ERR_WATCHDOG_RESET, "Loop hung — system was reset by watchdog");
  }

  Serial.println("\n╔══════════════════════════════════════════════════╗");
  Serial.println("║  Harry Locomotive — Collision Prevention v5      ║");
  Serial.println("║  Datix AI  |  Ahmed Ali  |  May 2026             ║");
  Serial.println("╠══════════════════════════════════════════════════╣");
  Serial.printf( "║  Boot #%-5lu  WDT resets: %-5lu                  ║\n",
                 (unsigned long)totalRestarts, (unsigned long)wdtResets);
  Serial.printf( "║  Parking delay : %-4dms (tune in config)         ║\n",
                 PARKING_DELAY_MS);
  Serial.println("╚══════════════════════════════════════════════════╝\n");

  // Confirm relay pins are released (safety log)
  Serial.printf("[GPIO] Relay 1 power  GPIO%-2d — %s\n",
                RELAY_POWER_PIN,
                digitalRead(RELAY_POWER_PIN) == RELAY_RELEASE
                ? "RELEASED ✅ (inner has power)" : "⚠️ NOT RELEASED");
  Serial.printf("[GPIO] Relay 2 switch GPIO%-2d — %s\n",
                RELAY_SWITCH_PIN,
                digitalRead(RELAY_SWITCH_PIN) == RELAY_RELEASE
                ? "RELEASED ✅ (switch idle)" : "⚠️ NOT RELEASED");

  // Hardware watchdog — resets ESP32 if loop() hangs
  esp_task_wdt_init(WATCHDOG_TIMEOUT_S, true);
  esp_task_wdt_add(NULL);
  Serial.printf("[WDT] Watchdog armed — %ds timeout\n", WATCHDOG_TIMEOUT_S);

  // Status LED
  pinMode(STATUS_LED, OUTPUT);
  digitalWrite(STATUS_LED, LOW);

  // IR sensors — INPUT_PULLUP: HIGH = beam clear, LOW = beam broken
  pinMode(SENSOR_A_PIN, INPUT_PULLUP);
  pinMode(SENSOR_B_PIN, INPUT_PULLUP);
  pinMode(SENSOR_C_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(SENSOR_A_PIN), ISR_SensorA, FALLING);
  attachInterrupt(digitalPinToInterrupt(SENSOR_B_PIN), ISR_SensorB, FALLING);
  attachInterrupt(digitalPinToInterrupt(SENSOR_C_PIN), ISR_SensorC, FALLING);
  Serial.printf("[GPIO] Sensor A GPIO%d (inner entry)\n", SENSOR_A_PIN);
  Serial.printf("[GPIO] Sensor B GPIO%d (outer entry)\n", SENSOR_B_PIN);
  Serial.printf("[GPIO] Sensor C GPIO%d (exit)\n",        SENSOR_C_PIN);

  // Boot-time sensor check — warn if any sensor already reads LOW
  delay(100);
  if (digitalRead(SENSOR_A_PIN) == LOW)
    reportError(ERR_SENSOR_STUCK_LOW, "Sensor A LOW at boot — check wiring");
  if (digitalRead(SENSOR_B_PIN) == LOW)
    reportError(ERR_SENSOR_STUCK_LOW, "Sensor B LOW at boot — check wiring");
  if (digitalRead(SENSOR_C_PIN) == LOW)
    reportError(ERR_SENSOR_STUCK_LOW, "Sensor C LOW at boot — check wiring");

  // BLE connect to outer train
  BLEDevice::init("TrainController");
  Serial.println("\n[BLE] Connecting by MAC...");
  bool ok = connectToTrain(TARGET_MAC);

  if (!ok) {
    Serial.printf("[BLE] MAC failed — scanning for '%s'...\n", TRAIN_NAME_PREFIX);
    BLEScan* pScan = BLEDevice::getScan();
    pScan->setAdvertisedDeviceCallbacks(new ScanCallback());
    pScan->setActiveScan(true);
    pScan->start(15, false);
    if (foundMAC.length() > 0) {
      ok = connectToTrain(foundMAC.c_str());
    } else {
      reportError(ERR_BLE_SCAN_NO_RESULT, "Is outer train powered on?");
    }
  }

  Serial.println(ok
    ? "\n[SYSTEM] ✅ Ready\n"
      "         Outer train running — inner train parked\n"
      "         Press X to run inner train one loop\n"
    : "\n[SYSTEM] ⚠️  BLE not connected — will retry every 5s\n");

  printMenu();
}

// ═══════════════════════════════════════════════════════════════════════════
//  MAIN LOOP — fully non-blocking, millis()-based timing throughout
// ═══════════════════════════════════════════════════════════════════════════

void loop() {
  unsigned long now = millis();

  // Feed hardware watchdog every cycle
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
    // Re-send STOP immediately if we reconnected during a locked zone
    if (bleConnected && zoneState != STATE_IDLE && innerTrainCaused) {
      Serial.println("[BLE] Reconnected during lock — re-sending STOP");
      bleSendStop();
      lastStopSent = now;
    }
  }

  // ── BLE: Keepalive ping ────────────────────────────────────────────
  // Only during IDLE or RAMP to avoid interfering with STOP commands
  if (bleConnected &&
      (zoneState == STATE_IDLE || zoneState == STATE_RAMP) &&
      (now - lastKeepalive > BLE_KEEPALIVE_MS)) {
    lastKeepalive = now;
    uint8_t c[] = {0x00, 0x45, (uint8_t)currentSpeed};
    sendBLE(c, 3, "keepalive");
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

  // ════════════════════════════════════════════════════════════════════
  //  ZONE STATE MACHINE
  // ════════════════════════════════════════════════════════════════════

  switch (zoneState) {

    // ──────────────────────────────────────────────────────────────────
    //  STATE_IDLE
    //  Outer train running freely. Inner train parked (off by default).
    //  Waiting for either sensor to detect a train approaching.
    // ──────────────────────────────────────────────────────────────────
    case STATE_IDLE: {

      // Sensor A — inner train entering shared section
      if (sensorA_fired) {
        sensorA_fired = false;
        if (now - lastSensorA > SENSOR_DEBOUNCE_MS) {
          lastSensorA = now;

          if (!innerTrainActive) {
            // Inner train was not supposed to be running — log but do not lock
            // (could be a false trigger or train was manually powered)
            Serial.println("[SENSOR A] ⚠️  Triggered but inner train not active — monitoring");
          }

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

      // Sensor B — outer train entering shared section
      if (sensorB_fired) {
        sensorB_fired = false;
        if (now - lastSensorB > SENSOR_DEBOUNCE_MS) {
          lastSensorB = now;
          Serial.println("\n[SENSOR B] ⚠️  Outer train entering shared section!");

          innerTrainCaused = false;
          zoneState        = STATE_LOCKED;
          lockTimer        = now;

          // If inner train is running, cut its power to prevent collision
          if (innerTrainActive) {
            innerWasCutForOuter = true;
            innerTrainPowerCut();
            Serial.println("[ACTION]   Inner train power cut (will restore after outer exits)");
          } else {
            innerWasCutForOuter = false;
            Serial.println("[ACTION]   Zone locked — inner train was parked, no relay action");
          }
        }
      }

      break;
    }

    // ──────────────────────────────────────────────────────────────────
    //  STATE_LOCKED
    //  A train is in the shared section.
    //  If inner caused it → keep BLE STOP on outer + wait for Sensor C
    //  If outer caused it → inner power already cut (if was running)
    //  Timeout after ZONE_TIMEOUT_MS to prevent permanent deadlock.
    // ──────────────────────────────────────────────────────────────────
    case STATE_LOCKED: {

      // Keep outer train stopped while inner is in zone
      if (innerTrainCaused && (now - lastStopSent > STOP_REPEAT_MS)) {
        lastStopSent = now;
        bleSendStop();
      }

      // Sensor C — train exiting shared section
      if (sensorC_fired) {
        sensorC_fired = false;
        if (now - lastSensorC > SENSOR_DEBOUNCE_MS) {
          lastSensorC = now;
          Serial.println("[SENSOR C] ✅ Train exiting shared section");

          if (innerTrainCaused) {
            // Inner train has exited — let it coast to parking spot
            Serial.printf("[INNER]    Coasting to parking — %dms until power cut\n",
                          PARKING_DELAY_MS);
            zoneState   = STATE_INNER_PARKING;
            parkingTimer = now;
          } else {
            // Outer train has exited — safety delay then switch pulse
            Serial.printf("[SYSTEM]   Outer exited — safety delay %dms\n",
                          RESUME_DELAY_MS);
            zoneState   = STATE_DELAY;
            resumeTimer = now;
          }
        }
      }

      // Zone timeout — Sensor C never fired
      if (now - lockTimer > ZONE_TIMEOUT_MS) {
        zoneTimeouts++;
        reportError(ERR_ZONE_TIMEOUT,
                    innerTrainCaused
                    ? "Inner train may be stuck/derailed in shared section"
                    : "Outer train may be stuck/derailed in shared section");
        // Force-cut inner and go to delay to resume outer
        if (innerTrainCaused) {
          innerTrainActive = false;
          innerTrainPowerCut();
        }
        zoneState   = STATE_DELAY;
        resumeTimer = now;
      }

      break;
    }

    // ──────────────────────────────────────────────────────────────────
    //  STATE_INNER_PARKING
    //  Inner train has exited the shared section via Sensor C.
    //  It is coasting toward its parking position.
    //  Outer train stays stopped (STOP repeated) during parking.
    //  After PARKING_DELAY_MS → cut relay → inner stops at parking spot.
    //  Then begin safety delay before resuming outer train.
    // ──────────────────────────────────────────────────────────────────
    case STATE_INNER_PARKING: {

      // Keep outer train stopped during parking coast
      if (now - lastStopSent > STOP_REPEAT_MS) {
        lastStopSent = now;
        bleSendStop();
      }

      if (now - parkingTimer >= PARKING_DELAY_MS) {
        // Cut inner train power — stops at parking position
        innerTrainActive = false;
        innerTrainPowerCut();
        Serial.println("[INNER]    ⛔ Inner train stopped at parking position");
        Serial.println("[INNER]    Loop complete — press X to run again");

        // Begin safety delay before resuming outer train
        Serial.printf("[SYSTEM]   Safety delay %dms before outer resumes\n",
                      RESUME_DELAY_MS);
        zoneState   = STATE_DELAY;
        resumeTimer = now;
      }

      break;
    }

    // ──────────────────────────────────────────────────────────────────
    //  STATE_DELAY
    //  Safety buffer — trains need time to fully clear before resuming.
    //  For inner-train exits: outer resumes after delay (no switch pulse).
    //  For outer-train exits: switch pulse fires after delay.
    //  Keep sending STOP during delay.
    // ──────────────────────────────────────────────────────────────────
    case STATE_DELAY: {

      if (now - lastStopSent > STOP_REPEAT_MS) {
        lastStopSent = now;
        bleSendStop();
      }

      if (now - resumeTimer >= RESUME_DELAY_MS) {
        if (!innerTrainCaused) {
          // Outer train exited — pulse track switch to reset direction
          Serial.println("[RELAY2]   Pulsing track switch — outer loop reset");
          trackSwitchPulseStart();
          zoneState   = STATE_SWITCH_PULSE;
          switchTimer = now;
        } else {
          // Inner train exited (already parked) — resume outer directly
          Serial.println("[RESUME]   Inner parked → outer train resuming slowly");
          bleResumeSlow();
          currentSpeed = RESUME_SLOW_SPEED;
          zoneState    = STATE_RAMP;
          rampTimer    = now;
        }
      }

      break;
    }

    // ──────────────────────────────────────────────────────────────────
    //  STATE_SWITCH_PULSE
    //  1-second relay pulse resets track switch to outer loop direction.
    //  Only reached when OUTER train was the one that exited.
    //  Inner train power is restored here if it was cut for the outer.
    // ──────────────────────────────────────────────────────────────────
    case STATE_SWITCH_PULSE: {

      // Keep outer stopped during switch pulse
      if (now - lastStopSent > STOP_REPEAT_MS) {
        lastStopSent = now;
        bleSendStop();
      }

      if (now - switchTimer >= SWITCH_PULSE_MS) {
        trackSwitchPulseEnd();

        // Restore inner train power if it was cut because outer entered zone
        if (innerWasCutForOuter && innerTrainActive) {
          innerTrainPowerRestore();
          innerWasCutForOuter = false;
          Serial.println("[INNER]    Power restored — inner train continuing loop");
        } else {
          innerWasCutForOuter = false;
        }

        // Resume outer train at slow speed
        Serial.println("[RESUME]   Outer train resuming slowly after switch pulse");
        bleResumeSlow();
        currentSpeed = RESUME_SLOW_SPEED;
        zoneState    = STATE_RAMP;
        rampTimer    = now;
      }

      break;
    }

    // ──────────────────────────────────────────────────────────────────
    //  STATE_RAMP
    //  Outer train at slow speed — ramp up to full speed after delay.
    //  Prevents outer train rushing immediately back into shared section.
    // ──────────────────────────────────────────────────────────────────
    case STATE_RAMP: {

      if (now - rampTimer >= SPEED_RAMP_MS) {
        bleResumeFull();
        currentSpeed = RESUME_FULL_SPEED;

        // Clear all ISR flags accumulated during resume sequence
        sensorA_fired    = false;
        sensorB_fired    = false;
        sensorC_fired    = false;
        innerTrainCaused = false;
        zoneState        = STATE_IDLE;

        Serial.println("[ZONE]     ✅ UNLOCKED — outer running, inner parked\n");
      }

      break;
    }
  }

  // ── Heartbeat LED ──────────────────────────────────────────────────
  // 40ms blink every 2s = system alive in IDLE. Solid = BLE connected.
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

  delay(5);   // Yield 5ms to BLE stack each cycle — safe, timers are 100ms+ range
}