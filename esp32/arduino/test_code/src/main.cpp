// ══════════════════════════════════════════════════════════════════
//  main.cpp  —  LionChief BLE Controller (test_code)
//  Harry Locomotive Project
//
//  WHAT IT DOES:
//    Connects to Peter's LionChief train via BLE and lets you control
//    all train functions from the Serial Monitor keyboard.
//
//  CONNECTION MODE:
//    The code tries TWO methods in order:
//      1. Direct MAC connection (TARGET_MAC below) — fast, reliable
//      2. Name-prefix scan fallback (TRAIN_NAME_PREFIX) — if MAC fails
//
//  HOW TO FLASH:
//    cd esp32/arduino/test_code
//    pio run --target upload
//    (Hold BOOT button when "Connecting..." appears)
//    pio device monitor
//
//  SERIAL COMMANDS (type in monitor + press Enter):
//    s = STOP           f = FORWARD       r = REVERSE
//    e = EMERGENCY STOP + = Speed UP      - = Speed DOWN
//    1-7 = Set speed    h = Horn toggle   b = Bell toggle
//    n = Sound ON       m = Sound OFF     a = Announce
//    i = Status         ? = Show menu
// ══════════════════════════════════════════════════════════════════

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>

// ── CONFIGURATION ─────────────────────────────────────────────────
// Peter's confirmed train (from Android nRF Connect scan)
#define TARGET_MAC          "CC:01:78:D0:F0:99"
#define TRAIN_NAME_PREFIX   "LC0"        // fallback: match any device starting with LC0

// LionChief BLE UUIDs — confirmed via nRF Connect
#define SERVICE_UUID        "e20a39f4-73f5-4bc4-a12f-17d1ad07a961"
#define CHARACTERISTIC_UUID "08590f7e-db05-467e-8757-72f6faeb13d4"

// ── BLE COMMAND BYTES ─────────────────────────────────────────────
uint8_t CMD_STOP[]      = {0x00, 0x45, 0x00};
uint8_t CMD_SPEED_1[]   = {0x00, 0x45, 0x01};
uint8_t CMD_SPEED_2[]   = {0x00, 0x45, 0x02};
uint8_t CMD_SPEED_3[]   = {0x00, 0x45, 0x03};
uint8_t CMD_SPEED_4[]   = {0x00, 0x45, 0x04};
uint8_t CMD_SPEED_5[]   = {0x00, 0x45, 0x05};
uint8_t CMD_SPEED_6[]   = {0x00, 0x45, 0x06};
uint8_t CMD_SPEED_7[]   = {0x00, 0x45, 0x07};
uint8_t CMD_FORWARD[]   = {0x00, 0x46, 0x01};
uint8_t CMD_REVERSE[]   = {0x00, 0x46, 0x02};
uint8_t CMD_HORN_ON[]   = {0x00, 0x48, 0x01};
uint8_t CMD_HORN_OFF[]  = {0x00, 0x48, 0x00};
uint8_t CMD_BELL_ON[]   = {0x00, 0x47, 0x01};
uint8_t CMD_BELL_OFF[]  = {0x00, 0x47, 0x00};
uint8_t CMD_SOUND_ON[]  = {0x00, 0x4C, 0x07};  // max volume
uint8_t CMD_SOUND_OFF[] = {0x00, 0x4C, 0x00};
uint8_t CMD_ANNOUNCE[]  = {0x00, 0x4D, 0x00, 0x00};

// ── GLOBALS ───────────────────────────────────────────────────────
BLEClient*               pClient   = nullptr;
BLERemoteCharacteristic* pChar     = nullptr;
bool     connected     = false;
bool     hornActive    = false;
bool     bellActive    = false;
int      currentSpeed  = 0;
String   foundMAC      = "";    // filled by scan fallback

// ── SEND COMMAND ──────────────────────────────────────────────────
void sendCmd(uint8_t* cmd, size_t len, const char* label) {
  if (!connected || !pChar) {
    Serial.println("[BLE] Not connected — command ignored");
    return;
  }
  pChar->writeValue(cmd, len, false);
  Serial.printf("[SENT] %s  [0x%02X, 0x%02X, 0x%02X]\n",
                label, cmd[0], cmd[1], cmd[2]);
}

// ── COMMAND WRAPPERS ──────────────────────────────────────────────
void trainStop()      { currentSpeed=0; sendCmd(CMD_STOP,     3, "STOP"); }
void trainSpeed1()    { currentSpeed=1; sendCmd(CMD_SPEED_1,  3, "SPEED 1"); }
void trainSpeed2()    { currentSpeed=2; sendCmd(CMD_SPEED_2,  3, "SPEED 2"); }
void trainSpeed3()    { currentSpeed=3; sendCmd(CMD_SPEED_3,  3, "SPEED 3"); }
void trainSpeed4()    { currentSpeed=4; sendCmd(CMD_SPEED_4,  3, "SPEED 4"); }
void trainSpeed5()    { currentSpeed=5; sendCmd(CMD_SPEED_5,  3, "SPEED 5"); }
void trainSpeed6()    { currentSpeed=6; sendCmd(CMD_SPEED_6,  3, "SPEED 6"); }
void trainSpeed7()    { currentSpeed=7; sendCmd(CMD_SPEED_7,  3, "SPEED 7 (MAX)"); }
void trainForward()   { sendCmd(CMD_FORWARD,  3, "FORWARD"); }
void trainReverse()   { sendCmd(CMD_REVERSE,  3, "REVERSE"); }
void trainAnnounce()  { pChar->writeValue(CMD_ANNOUNCE, 4, false); Serial.println("[SENT] ANNOUNCE"); }

void trainSpeedUp() {
  if (currentSpeed < 7) {
    currentSpeed++;
    uint8_t cmd[] = {0x00, 0x45, (uint8_t)currentSpeed};
    char lbl[20]; snprintf(lbl, sizeof(lbl), "SPEED UP → %d", currentSpeed);
    sendCmd(cmd, 3, lbl);
  } else {
    Serial.println("Already at MAX speed (7)");
  }
}

void trainSpeedDown() {
  if (currentSpeed > 0) {
    currentSpeed--;
    uint8_t cmd[] = {0x00, 0x45, (uint8_t)currentSpeed};
    char lbl[20]; snprintf(lbl, sizeof(lbl), "SPEED DOWN → %d", currentSpeed);
    sendCmd(cmd, 3, lbl);
  } else {
    Serial.println("Already stopped");
  }
}

void trainHornToggle() {
  if (hornActive) { sendCmd(CMD_HORN_OFF, 3, "HORN OFF"); hornActive = false; }
  else            { sendCmd(CMD_HORN_ON,  3, "HORN ON");  hornActive = true;  }
}

void trainBellToggle() {
  if (bellActive) { sendCmd(CMD_BELL_OFF, 3, "BELL OFF"); bellActive = false; }
  else            { sendCmd(CMD_BELL_ON,  3, "BELL ON");  bellActive = true;  }
}

// ── PRINT MENU ────────────────────────────────────────────────────
void printMenu() {
  Serial.println("\n╔══════════════════════════════════════════╗");
  Serial.println("║     LIONCHIEF TRAIN CONTROLLER           ║");
  Serial.println("╠══════════════════════════════════════════╣");
  Serial.println("║  s = STOP        e = EMERGENCY STOP      ║");
  Serial.println("║  f = FORWARD     r = REVERSE             ║");
  Serial.println("║  + = Speed UP    - = Speed DOWN          ║");
  Serial.println("║  1-7 = Set exact speed                   ║");
  Serial.println("╠══════════════════════════════════════════╣");
  Serial.println("║  h = Horn        b = Bell                ║");
  Serial.println("║  n = Sound ON    m = Sound OFF           ║");
  Serial.println("║  a = Announce    i = Status  ? = Menu    ║");
  Serial.println("╚══════════════════════════════════════════╝");
  Serial.println("Type a key and press Enter:");
}

void printStatus() {
  Serial.println("\n── TRAIN STATUS ────────────────────────────");
  Serial.printf("  Connected : %s\n", connected ? "YES ✅" : "NO ❌");
  Serial.printf("  MAC       : %s\n", String(TARGET_MAC).c_str());
  Serial.printf("  Speed     : %d/7\n", currentSpeed);
  Serial.printf("  Horn      : %s\n", hornActive ? "ON" : "OFF");
  Serial.printf("  Bell      : %s\n", bellActive ? "ON" : "OFF");
  Serial.println("────────────────────────────────────────────\n");
}

// ── HANDLE SERIAL COMMAND ─────────────────────────────────────────
void handleCommand(char cmd) {
  switch (cmd) {
    case 's': trainStop();        break;
    case 'e': trainStop();        break;  // same as stop
    case 'f': trainForward();     break;
    case 'r': trainReverse();     break;
    case '+': trainSpeedUp();     break;
    case '-': trainSpeedDown();   break;
    case '1': trainSpeed1();      break;
    case '2': trainSpeed2();      break;
    case '3': trainSpeed3();      break;
    case '4': trainSpeed4();      break;
    case '5': trainSpeed5();      break;
    case '6': trainSpeed6();      break;
    case '7': trainSpeed7();      break;
    case 'h': trainHornToggle();  break;
    case 'b': trainBellToggle();  break;
    case 'n': sendCmd(CMD_SOUND_ON,  3, "SOUND ON");  break;
    case 'm': sendCmd(CMD_SOUND_OFF, 3, "SOUND OFF"); break;
    case 'a': trainAnnounce();    break;
    case '?': printMenu();        break;
    case 'i': printStatus();      break;
    default:  Serial.println("Unknown command. Press ? for menu."); break;
  }
}

// ── BLE: CONNECT BY MAC ADDRESS ───────────────────────────────────
bool connectByMAC(const char* mac) {
  Serial.printf("[BLE] Connecting to MAC: %s\n", mac);

  if (pClient == nullptr) {
    pClient = BLEDevice::createClient();
  }

  BLEAddress bleAddr(mac);
  if (!pClient->connect(bleAddr)) {
    Serial.println("[BLE] Connection FAILED");
    return false;
  }
  Serial.println("[BLE] TCP connected — getting service...");

  BLERemoteService* pService = pClient->getService(BLEUUID(SERVICE_UUID));
  if (!pService) {
    Serial.println("[BLE] Service UUID not found on device!");
    pClient->disconnect();
    return false;
  }

  pChar = pService->getCharacteristic(BLEUUID(CHARACTERISTIC_UUID));
  if (!pChar) {
    Serial.println("[BLE] Characteristic not found!");
    pClient->disconnect();
    return false;
  }

  Serial.println("[BLE] ✅ Ready — all commands available!");
  return true;
}

// ── BLE: SCAN CALLBACK (name-prefix fallback) ─────────────────────
class ScanCallback : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice device) {
    String name = device.getName().c_str();
    String addr = device.getAddress().toString().c_str();

    if (name.length() > 0) {
      Serial.println("  Scanned: " + name + "  [" + addr + "]");
    }

    // Match name prefix "LC0"
    if (name.startsWith(TRAIN_NAME_PREFIX)) {
      Serial.println("  >>> LionChief found by name: " + name);
      foundMAC = addr;
      BLEDevice::getScan()->stop();
    }
  }
};

// ── SETUP ─────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(800);

  Serial.println("\n╔══════════════════════════════════════════╗");
  Serial.println("║  LionChief Controller — Starting Up      ║");
  Serial.printf( "║  Target MAC: %-28s║\n", TARGET_MAC);
  Serial.println("╚══════════════════════════════════════════╝\n");

  BLEDevice::init("ESP32-LionChief");

  // ── Step 1: Try direct MAC connection ──────────────────────────
  Serial.println("[BLE] Step 1: Trying direct MAC connection...");
  connected = connectByMAC(TARGET_MAC);

  // ── Step 2: If MAC failed, scan by name prefix as fallback ──────
  if (!connected) {
    Serial.println("[BLE] Step 2: MAC failed — scanning by name prefix...");
    BLEScan* pScan = BLEDevice::getScan();
    pScan->setAdvertisedDeviceCallbacks(new ScanCallback());
    pScan->setActiveScan(true);
    pScan->start(15, false);  // scan 15 seconds

    if (foundMAC.length() > 0) {
      connected = connectByMAC(foundMAC.c_str());
    } else {
      Serial.println("[BLE] Train not found in scan either.");
      Serial.println("[BLE] Make sure train is powered on, then reset ESP32.");
    }
  }

  if (connected) {
    printMenu();
  } else {
    Serial.println("\n[STATUS] Not connected. Commands will retry on next connection.");
  }
}

// ── LOOP ──────────────────────────────────────────────────────────
void loop() {
  // Read and execute serial commands
  if (Serial.available()) {
    char cmd = Serial.read();
    if (cmd != '\n' && cmd != '\r') {
      Serial.printf("\n> '%c'\n", cmd);
      handleCommand(cmd);
    }
  }

  // Auto-reconnect if connection dropped
  if (connected && pClient && !pClient->isConnected()) {
    Serial.println("\n[BLE] Connection lost! Attempting reconnect...");
    connected = false;
    foundMAC  = "";

    // Try MAC first, then scan
    connected = connectByMAC(TARGET_MAC);

    if (!connected) {
      BLEScan* pScan = BLEDevice::getScan();
      pScan->start(10, false);
      if (foundMAC.length() > 0) {
        connected = connectByMAC(foundMAC.c_str());
      }
    }

    if (connected) {
      Serial.println("[BLE] Reconnected ✅");
      printMenu();
    }
  }

  delay(50);
}
