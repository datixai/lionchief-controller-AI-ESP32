// in terminal run pio run --target upload
//once you see connectig press boot button on ESP 32 and keep press until write commands seccussfull
//Once you see successfull run this command pio device monitor
#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>

// ── BLE UUIDs ────────────────────────────────────────────────
#define SERVICE_UUID        "e20a39f4-73f5-4bc4-a12f-17d1ad07a961"
#define CHARACTERISTIC_UUID "08590f7e-db05-467e-8757-72f6faeb13d4"
#define TRAIN_NAME          "LC015556-99F0"
#define TARGET_MAC "CC:01:78:D0:F0:99"

// ── ALL LIONCHIEF COMMANDS ───────────────────────────────────
// Speed commands (0=stop, 1-7=speed levels)
uint8_t CMD_STOP[]         = {0x00, 0x45, 0x00};
uint8_t CMD_SPEED_1[]      = {0x00, 0x45, 0x01};
uint8_t CMD_SPEED_2[]      = {0x00, 0x45, 0x02};
uint8_t CMD_SPEED_3[]      = {0x00, 0x45, 0x03};
uint8_t CMD_SPEED_4[]      = {0x00, 0x45, 0x04};
uint8_t CMD_SPEED_5[]      = {0x00, 0x45, 0x05};
uint8_t CMD_SPEED_6[]      = {0x00, 0x45, 0x06};
uint8_t CMD_SPEED_7[]      = {0x00, 0x45, 0x07};  // max speed

// Direction commands
uint8_t CMD_FORWARD[]      = {0x00, 0x45, 0x07};  // forward medium
uint8_t CMD_REVERSE[]      = {0x00, 0x46, 0x07};  // reverse medium

// Horn commands
uint8_t CMD_HORN_ON[]      = {0x00, 0x48, 0x01};
uint8_t CMD_HORN_OFF[]     = {0x00, 0x48, 0x00};

// Bell commands
uint8_t CMD_BELL_ON[]      = {0x00, 0x47, 0x01};
uint8_t CMD_BELL_OFF[]     = {0x00, 0x47, 0x00};

// Sound commands
uint8_t CMD_SOUND_ON[]     = {0x00, 0x4C, 0x01};
uint8_t CMD_SOUND_OFF[]    = {0x00, 0x4C, 0x00};

// Announce / speech
uint8_t CMD_ANNOUNCE[]     = {0x00, 0x4D, 0x01};

// Emergency stop (cuts power immediately)
uint8_t CMD_EMERGENCY[]    = {0x00, 0x45, 0x00};

// ── GLOBALS ──────────────────────────────────────────────────
BLEClient*               pClient = nullptr;
BLERemoteCharacteristic* pChar   = nullptr;
bool     connected      = false;
bool     hornActive     = false;
bool     bellActive     = false;
int      currentSpeed   = 0;
String   trainAddress   = "";

// ── SEND COMMAND ─────────────────────────────────────────────
void sendCmd(uint8_t* cmd, size_t len, String label) {
  if (!connected || !pChar) {
    Serial.println("Not connected to train!");
    return;
  }
  pChar->writeValue(cmd, len, false);
  Serial.println("SENT: " + label);
}

// ── ALL COMMAND FUNCTIONS ────────────────────────────────────
void trainStop()      { currentSpeed=0; sendCmd(CMD_STOP,      3, "STOP"); }
void trainSpeed1()    { currentSpeed=1; sendCmd(CMD_SPEED_1,   3, "SPEED 1 (slowest)"); }
void trainSpeed2()    { currentSpeed=2; sendCmd(CMD_SPEED_2,   3, "SPEED 2"); }
void trainSpeed3()    { currentSpeed=3; sendCmd(CMD_SPEED_3,   3, "SPEED 3"); }
void trainSpeed4()    { currentSpeed=4; sendCmd(CMD_SPEED_4,   3, "SPEED 4 (medium)"); }
void trainSpeed5()    { currentSpeed=5; sendCmd(CMD_SPEED_5,   3, "SPEED 5"); }
void trainSpeed6()    { currentSpeed=6; sendCmd(CMD_SPEED_6,   3, "SPEED 6"); }
void trainSpeed7()    { currentSpeed=7; sendCmd(CMD_SPEED_7,   3, "SPEED 7 (max)"); }
void trainForward()   {                 sendCmd(CMD_FORWARD,   3, "FORWARD"); }
void trainReverse()   {                 sendCmd(CMD_REVERSE,   3, "REVERSE"); }
void trainEmergency() { currentSpeed=0; sendCmd(CMD_EMERGENCY, 3, "EMERGENCY STOP"); }
void trainAnnounce()  {                 sendCmd(CMD_ANNOUNCE,  3, "ANNOUNCE/SPEECH"); }

void trainHornOn()  { hornActive=true;  sendCmd(CMD_HORN_ON,  3, "HORN ON"); }
void trainHornOff() { hornActive=false; sendCmd(CMD_HORN_OFF, 3, "HORN OFF"); }

void trainBellOn()  { bellActive=true;  sendCmd(CMD_BELL_ON,  3, "BELL ON"); }
void trainBellOff() { bellActive=false; sendCmd(CMD_BELL_OFF, 3, "BELL OFF"); }

void trainSoundOn()  { sendCmd(CMD_SOUND_ON,  3, "SOUND ON"); }
void trainSoundOff() { sendCmd(CMD_SOUND_OFF, 3, "SOUND OFF"); }

void trainSpeedUp() {
  if (currentSpeed < 7) {
    currentSpeed++;
    uint8_t cmd[] = {0x00, 0x45, (uint8_t)currentSpeed};
    sendCmd(cmd, 3, "SPEED UP → " + String(currentSpeed));
  } else {
    Serial.println("Already at MAX speed (7)");
  }
}

void trainSpeedDown() {
  if (currentSpeed > 0) {
    currentSpeed--;
    uint8_t cmd[] = {0x00, 0x45, (uint8_t)currentSpeed};
    sendCmd(cmd, 3, "SPEED DOWN → " + String(currentSpeed));
  } else {
    Serial.println("Already stopped (speed 0)");
  }
}

// ── PRINT MENU ───────────────────────────────────────────────
void printMenu() {
  Serial.println("\n╔════════════════════════════════════════╗");
  Serial.println("║      LIONCHIEF TRAIN CONTROLLER        ║");
  Serial.println("╠════════════════════════════════════════╣");
  Serial.println("║  SPEED CONTROL                         ║");
  Serial.println("║  s = STOP          f = FORWARD         ║");
  Serial.println("║  r = REVERSE       e = EMERGENCY STOP  ║");
  Serial.println("║  + = Speed UP      - = Speed DOWN      ║");
  Serial.println("║  1-7 = Set speed directly              ║");
  Serial.println("╠════════════════════════════════════════╣");
  Serial.println("║  SOUNDS                                ║");
  Serial.println("║  h = Horn ON/OFF   b = Bell ON/OFF     ║");
  Serial.println("║  n = Sound ON      m = Sound OFF       ║");
  Serial.println("║  a = Announce/Speech                   ║");
  Serial.println("╠════════════════════════════════════════╣");
  Serial.println("║  INFO                                  ║");
  Serial.println("║  ? = Show this menu                    ║");
  Serial.println("║  i = Train status                      ║");
  Serial.println("╚════════════════════════════════════════╝");
  Serial.println("Type command and press Enter:");
}

void printStatus() {
  Serial.println("\n── TRAIN STATUS ──────────────────────────");
  Serial.println("Connected : " + String(connected ? "YES ✅" : "NO ❌"));
  Serial.println("Speed     : " + String(currentSpeed) + "/7");
  Serial.println("Horn      : " + String(hornActive ? "ON" : "OFF"));
  Serial.println("Bell      : " + String(bellActive ? "ON" : "OFF"));
  Serial.println("──────────────────────────────────────────\n");
}

// ── HANDLE SERIAL INPUT ──────────────────────────────────────
void handleCommand(char cmd) {
  switch (cmd) {
    // Speed
    case 's': trainStop();      break;
    case 'e': trainEmergency(); break;
    case 'f': trainForward();   break;
    case 'r': trainReverse();   break;
    case '+': trainSpeedUp();   break;
    case '-': trainSpeedDown(); break;
    case '1': trainSpeed1();    break;
    case '2': trainSpeed2();    break;
    case '3': trainSpeed3();    break;
    case '4': trainSpeed4();    break;
    case '5': trainSpeed5();    break;
    case '6': trainSpeed6();    break;
    case '7': trainSpeed7();    break;

    // Sounds
    case 'h': hornActive ? trainHornOff() : trainHornOn(); break;
    case 'b': bellActive ? trainBellOff() : trainBellOn(); break;
    case 'n': trainSoundOn();   break;
    case 'm': trainSoundOff();  break;
    case 'a': trainAnnounce();  break;

    // Info
    case '?': printMenu();   break;
    case 'i': printStatus(); break;

    default:
      Serial.println("Unknown command. Type ? for menu.");
      break;
  }
}

// ── CONNECT TO TRAIN ─────────────────────────────────────────
bool connectToTrain(String address) {
  Serial.println("Connecting to: " + address);
  pClient = BLEDevice::createClient();

  if (!pClient->connect(BLEAddress(address.c_str()))) {
    Serial.println("Connection FAILED!");
    return false;
  }
  Serial.println("Connected!");

  BLERemoteService* pService =
    pClient->getService(BLEUUID(SERVICE_UUID));
  if (!pService) {
    Serial.println("Service not found!");
    pClient->disconnect();
    return false;
  }

  pChar = pService->getCharacteristic(BLEUUID(CHARACTERISTIC_UUID));
  if (!pChar) {
    Serial.println("Characteristic not found!");
    pClient->disconnect();
    return false;
  }

  Serial.println("Train ready!");
  return true;
}

// ── SCAN CALLBACK ────────────────────────────────────────────
class ScanCallback : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice device) {
    String name = device.getName().c_str();
    if (name.length() > 0) {
      Serial.println("Found: " + name + " | " +
                     device.getAddress().toString().c_str());
    }
    if (name.indexOf(TRAIN_NAME) >= 0) {
      Serial.println(">>> TRAIN FOUND: " + name);
      trainAddress = device.getAddress().toString().c_str();
      BLEDevice::getScan()->stop();
    }
  }
};

// ── SETUP ────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("LionChief Controller Starting...");
  BLEDevice::init("ESP32-LionChief");

  Serial.println("Scanning for train (15 seconds)...");
  BLEScan* pScan = BLEDevice::getScan();
  pScan->setAdvertisedDeviceCallbacks(new ScanCallback());
  pScan->setActiveScan(true);
  pScan->start(15, false);

  if (trainAddress.length() > 0) {
    connected = connectToTrain(trainAddress);
  } else {
    Serial.println("Train not found! Power it on and reset ESP32.");
  }

  if (connected) {
    printMenu();
  }
}

// ── LOOP ─────────────────────────────────────────────────────
void loop() {
  // Handle Serial commands
  if (Serial.available()) {
    char cmd = Serial.read();
    if (cmd != '\n' && cmd != '\r') {
      Serial.println("\n> CMD: " + String(cmd));
      handleCommand(cmd);
    }
  }

  // Auto reconnect if disconnected
  if (connected && !pClient->isConnected()) {
    Serial.println("Connection lost! Rescanning...");
    connected    = false;
    trainAddress = "";
    BLEScan* pScan = BLEDevice::getScan();
    pScan->start(10, false);
    if (trainAddress.length() > 0) {
      connected = connectToTrain(trainAddress);
      if (connected) printMenu();
    }
  }

  delay(50);
}