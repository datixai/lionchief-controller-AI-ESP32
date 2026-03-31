# 🚂 LionChief Collision Prevention System

> Automated collision prevention for a Lionel LionChief O-scale model train layout.
> Two trains share a 3-foot section of track. This system detects when a train
> enters the shared section and automatically stops the other train via Bluetooth (BLE),
> then resumes it when the track clears.

**Developed by:** Ahmed (datixai) | **Client:** Peter (Harry Locomotive Project)

---

## 📋 Table of Contents

- [How It Works](#how-it-works)
- [Project Structure](#project-structure)
- [Three Detection Systems](#three-detection-systems)
- [Quick Start](#quick-start)
- [BLE Train Details](#ble-train-details)
- [Dataset and AI Model Training](#dataset-and-ai-model-training)
- [Hardware Shopping List](#hardware-shopping-list)
- [Wiring Reference](#wiring-reference)
- [BLE Command Reference](#ble-command-reference)
- [Running Tests](#running-tests)
- [Configuration Reference](#configuration-reference)
- [Development Roadmap](#development-roadmap)
- [Reference Repositories](#reference-repositories)

---

## How It Works

```
Peter's Layout (15ft x 20ft O-Scale):

  ┌─────────────────────────────────────────┐
  │           OUTER LOOP                    │
  │   ◆═══════════════════════◆             │
  │   ║  SHARED SECTION        ║             │
  │   ║  (3 feet — one track)  ║             │
  │   ◆═══════════════════════◆             │
  │         ↘                               │
  │           INNER LOOP (private oval)     │
  └─────────────────────────────────────────┘

Point A = Entry junction (train enters shared section)
Point B = Exit  junction (train exits shared section)

Collision Logic:
  Train enters shared section at A
  → OTHER train stops before Point A
  → Waits until first train exits from B
  → Resumes after 2.5 second safety delay
```

**Three independent detection systems:**

| System | Method | Hardware | Status |
|--------|--------|---------|--------|
| System 1 | IR beam sensors + ESP32 | ESP32 + E18-D80NK sensors | ✅ Ready to deploy |
| System 2 | OpenCV zone detection | Laptop + USB webcam | ✅ Working now |
| System 3 | YOLOv11n AI detection | Raspberry Pi 5 + USB camera | ⏳ Needs dataset |

---

## 📁 Project Structure

```
lionchief-controller-AI-ESP32/
│
├── esp32/                              ← ESP32 microcontroller code
│   ├── arduino/
│   │   ├── test_code/                  ← ACTIVE: flash this to ESP32
│   │   │   ├── src/
│   │   │   │   └── main.cpp            ← BLE scanner + all train commands
│   │   │   └── platformio.ini          ← PlatformIO config (COM3, esp32dev)
│   │   └── train_controller/
│   │       └── train_controller.ino    ← Arduino IDE alternative
│   └── esphome/
│       ├── train_controller.yaml       ← Flash via ESPHome (WiFi OTA updates)
│       └── secrets.yaml.example        ← Copy to secrets.yaml, add WiFi creds
│
├── laptop_cv/                          ← Camera detection on laptop (no Pi needed)
│   ├── main.py                         ← ENTRY POINT — run this
│   ├── config.py                       ← Camera, BLE, and detection settings
│   ├── calibrate_zone.py               ← Draw shared zone with mouse, saves JSON
│   ├── zone_detector.py                ← OpenCV background subtraction
│   ├── ble_controller.py               ← BLE connection direct from laptop
│   ├── shared_zone.json                ← Zone pixel coords (auto-created)
│   └── requirements.txt
│
├── raspberry_pi/                       ← Simulation + future Pi deployment
│   ├── mock_train.py                   ← Full visual simulation (Pygame)
│   ├── main.py                         ← Pi entry point (future use)
│   ├── config.py                       ← Pi settings
│   ├── calibrate_zone.py               ← Pi camera zone calibration
│   ├── scan_train.py                   ← Find train MAC via BLE scan
│   ├── requirements.txt
│   ├── .env.example
│   ├── lionchief/                      ← BLE train control package
│   │   ├── controller.py               ← Async BLE controller (bleak)
│   │   └── commands.py                 ← All LionChief BLE command bytes
│   ├── detection/                      ← AI detection package
│   │   ├── detector.py                 ← YOLO / TFLite / Mock detectors
│   │   └── zone_manager.py             ← Shared zone overlap logic
│   └── tests/
│       ├── test_commands.py            ← Unit tests for BLE commands
│       └── test_zone_manager.py        ← Unit tests for zone logic
│
├── models/                             ← AI model files (gitignored — large files)
│   └── README.md                       ← Instructions for placing model files
│
├── .gitignore
└── README.md
```

---

## Three Detection Systems

### System 1 — ESP32 + IR Sensors (Hardware)

Most reliable. ESP32 physically detects train breaking an infrared beam.

```
E18-D80NK Sensor at Point A
  → Train breaks beam → ESP32 reads LOW on GPIO16
  → ESP32 sends BLE STOP to other train
  → Response time: less than 2ms

E18-D80NK Sensor at Point B
  → Train clears beam → ESP32 reads HIGH on GPIO17
  → ESP32 waits 2.5 seconds safety delay
  → ESP32 sends BLE RESUME
```

**Flash ESP32 using PlatformIO:**
```bash
cd esp32/arduino/test_code
# PlatformIO extension must be installed in VS Code
pio run --target upload
# Hold BOOT button on ESP32 when "Connecting..." appears
# Then release when upload percentage starts
pio device monitor
```

**Train commands available via serial monitor (type key + Enter):**

| Key | Action |
|-----|--------|
| `s` | Stop |
| `e` | Emergency stop |
| `f` | Forward |
| `r` | Reverse |
| `+` / `-` | Speed up / down |
| `1` – `7` | Set exact speed level |
| `h` | Horn toggle ON/OFF |
| `b` | Bell toggle ON/OFF |
| `a` | Announce / speech |
| `i` | Show train status |
| `?` | Show full menu |

---

### System 2 — Laptop CV with OpenCV

No ESP32. No Raspberry Pi. Just laptop and any USB webcam.

```bash
cd laptop_cv
pip install -r requirements.txt

# Step 1: Calibrate zone (run once, or after remounting camera)
python calibrate_zone.py
# Camera opens → click and drag to draw the shared zone box
# Press S → saved automatically to shared_zone.json

# Step 2: Run detection
python main.py
```

**How it works:**
```
USB Camera feeds frames to OpenCV
Background subtractor learns empty layout (first ~30 seconds)
Any moving object detected in shared_zone.json pixel box = train
Laptop sends BLE STOP command directly to train via bleak library
No ESP32 involved — laptop IS the controller
Zone clears → 2.5 second delay → BLE RESUME sent
```

**shared_zone.json** is created by calibrate_zone.py and read by all other files:
```json
{
  "x1": 144,
  "y1": 127,
  "x2": 979,
  "y2": 218,
  "saved_at": "2026-03-29 15:28:47",
  "note": "Shared track section — both trains use this zone"
}
```
Re-run calibrate_zone.py any time the camera moves — all files update automatically.

**Keyboard controls in camera window:**

| Key | Action |
|-----|--------|
| `Q` / `ESC` | Quit |
| `P` | Pause / resume detection |
| `S` | Manual STOP train |
| `R` | Manual RESUME train |
| `H` | Horn |
| `B` | Bell toggle |
| `+` / `-` | Speed up / down |

---

### System 3 — Raspberry Pi 5 + YOLOv11n AI

Most intelligent. Identifies which specific train is in the shared zone.
Requires dataset from Peter's real layout footage. See the Dataset section below.

```bash
# Run full visual simulation right now (no hardware needed)
cd raspberry_pi
pip install pygame bleak
python mock_train.py
```

**Mock simulation controls:**

| Key / Button | Action |
|-------------|--------|
| `SPACE` | Emergency STOP outer train |
| `R` | Resume outer train |
| `T` | Toggle inner train ON/OFF |
| `+` / `-` | Speed up / down |
| `Q` / `ESC` | Quit |
| On-screen buttons | Click with mouse |

---

## Quick Start

### No Hardware — Run Simulation Now

```bash
git clone https://github.com/datixai/lionchief-controller-AI-ESP32.git
cd lionchief-controller-AI-ESP32
pip install pygame bleak
cd raspberry_pi
python mock_train.py
```

### Laptop + USB Webcam

```bash
cd laptop_cv
pip install -r requirements.txt
python calibrate_zone.py   # draw zone once
python main.py             # run detection
```

### ESP32 Only (BLE Control via Keyboard)

```bash
cd esp32/arduino/test_code
# Install PlatformIO IDE extension in VS Code
pio run --target upload    # hold BOOT button when Connecting...
pio device monitor         # type ? for menu
```

---

## 🔑 BLE Train Details

| Setting | Value |
|---------|-------|
| Device Name | `LC015556-99F0` (Peter's confirmed train) |
| Service UUID | `e20a39f4-73f5-4bc4-a12f-17d1ad07a961` ✅ |
| Characteristic UUID | `08590f7e-db05-467e-8757-72f6faeb13d4` ✅ |
| Write Permission | ✅ confirmed via nRF Connect |
| Notify Permission | ✅ confirmed via nRF Connect |
| MAC Address | Scan with nRF Connect on Android |

**How to find MAC address (Android only):**
```
1. Install nRF Connect — Google Play (free)
2. Power on the train
3. Tap Scan
4. Find LC015556 in the list
5. MAC shows next to the device name: XX:XX:XX:XX:XX:XX
6. Set TRAIN_MAC = "XX:XX:XX:XX:XX:XX" in config.py
```

> Important: nRF Connect on iOS does NOT show MAC address due to Apple
> privacy policy. Must use Android phone.

**Auto-discovery without MAC:**
Leave `TRAIN_MAC = ""` in config.py. Code scans for any device with `LC0`
in its name and connects automatically.

---

## 🤖 Dataset and AI Model Training

No public dataset exists for LionChief O-scale trains. YouTube videos are
unusable — wrong camera angles, different layouts, single trains only.
The only suitable dataset is footage from Peter's real layout.

### What to Annotate

```
Two classes only:
  Class 0: outer_train  ← the blue train on the outer loop
  Class 1: inner_train  ← the orange/yellow train on the inner loop

Do NOT annotate:
  Tracks        (static background — never annotate)
  Shared zone   (defined as pixel box in shared_zone.json)
  Buildings     (static background)
  Scenery       (static background)
```

### Step 1 — Collect Footage From Peter's Real Layout

Peter records 15-20 minutes with phone mounted above the layout:
```
Requirements for good footage:
  Camera angle: top-down or slightly angled from above
  Both trains must be running at the same time
  Both trains must be visible in the frame
  Focus especially on trains passing through shared section
  Good lighting — no harsh shadows

Delivery: Google Drive or WeTransfer
File format: any video format (MP4 preferred)
```

### Step 2 — Extract Frames

```bash
# Extract 2 frames per second
ffmpeg -i peters_layout_video.mp4 -vf fps=2 frames/frame_%04d.jpg

# 15 min video at 2fps = 1800 frames
# Select the best 500-600 frames (delete blurry/duplicate ones)
```

**How many frames are needed:**

| Amount | Detection Quality | Labeling Time |
|--------|-----------------|--------------|
| 100 | Poor — misses trains often | 30 min |
| 300 | Basic — sometimes fails | 1.5 hrs |
| **500–600** | **Good — recommended** | **2–3 hrs** |
| 1000+ | Excellent | 5+ hrs |

500 frames is enough because:
- Only 2 classes (simple task)
- Controlled environment (same layout, same lighting)
- Fixed camera angle
- Trains are distinct objects

### Step 3 — Label on Roboflow

```
1. Go to https://roboflow.com → sign up free
2. New Project → Object Detection → 2 classes: outer_train, inner_train
3. Upload the 500-600 frames
4. For each image:
   → Draw bounding box around blue train  → label "outer_train"
   → Draw bounding box around orange train → label "inner_train"
5. Apply augmentations: flip horizontal, brightness ±20%, rotation ±5°
6. Generate dataset version
7. Export as YOLOv11 PyTorch format
8. Copy the download code snippet shown
```

### Step 4 — Train on Google Colab (Free GPU)

```python
# Open Google Colab: colab.research.google.com
# Select Runtime → Change runtime type → T4 GPU

!pip install ultralytics roboflow

# Download dataset from Roboflow (paste your code snippet here)
from roboflow import Roboflow
rf = Roboflow(api_key="YOUR_KEY")
dataset = rf.workspace("YOUR_WS").project("YOUR_PROJECT").version(1).download("yolov11")

from ultralytics import YOLO

# YOLOv11n — fastest on Raspberry Pi 5 (22+ FPS)
model = YOLO('yolo11n.pt')

model.train(
    data=f'{dataset.location}/data.yaml',
    epochs=100,
    imgsz=640,
    batch=16,
    device=0,
)

# Check accuracy — target mAP@50 > 0.85
# Results saved to: runs/detect/train/

# Export for Raspberry Pi 5 (NCNN format)
best = YOLO('runs/detect/train/weights/best.pt')
best.export(format='ncnn', imgsz=320)
# Downloads folder: best_ncnn_model/
```

Training time on Colab free T4 GPU: 30-45 minutes.

### Step 5 — Deploy on Raspberry Pi 5

```bash
# Copy model to project
cp -r best_ncnn_model/ models/

# Edit raspberry_pi/config.py:
MODEL_TYPE = "yolo_ncnn"
TRAIN_MAC_ADDRESS = "XX:XX:XX:XX:XX:XX"

# Run
cd raspberry_pi
python main.py
```

### Why YOLOv11n Over YOLOv8n

| Feature | YOLOv8n | YOLOv11n |
|---------|---------|---------|
| Released | 2023 | October 2024 |
| Parameters | 3.2M | 2.6M (22% smaller) |
| Speed on Pi 5 | ~15 FPS | ~22 FPS |
| Accuracy mAP | 37.3 | 39.5 |
| Training code | Same command | Same command |

YOLOv11n trains with the exact same code as v8n — just change `yolov8n.pt` to `yolo11n.pt`.

---

## 🛒 Hardware Shopping List

### ESP32 System (Peter buys in USA)

| Item | Search on Amazon | Qty | Approx Price |
|------|-----------------|-----|-------------|
| ESP32 Board | "DOIT ESP32 DevKit V1 30 pin" | 1 | $8–12 |
| IR Sensor | "E18-D80NK infrared sensor" | 2 | $5–8 each |
| Jumper Wires F-F | "Female to Female Dupont wires 20cm" | 1 pack | $6–8 |
| Micro USB Data Cable | "Micro USB data cable Android" | 1 | $5–8 |
| Male Header Pins | "2.54mm male pin header strip 40 pin" | 2 strips | $5–7 |
| Soldering Iron | "Soldering iron kit 60W temperature control" | 1 | $15–25 |
| Solder Wire | "60/40 rosin core solder wire 0.8mm" | 1 roll | $8–12 |

### Raspberry Pi System (future AI deployment)

| Item | Search on Amazon | Qty | Approx Price |
|------|-----------------|-----|-------------|
| Raspberry Pi 5 | "Raspberry Pi 5 8GB RAM" | 1 | $80–90 |
| MicroSD Card | "SanDisk 64GB microSD Class 10 UHS-1" | 1 | $10–15 |
| Pi 5 Power Supply | "Raspberry Pi 5 USB-C 5V 5A 27W" | 1 | $12–15 |
| USB Camera | "Logitech C920 HD Pro Webcam 1080p" | 1 | $62–75 |
| MicroSD Card Reader | "USB microSD card reader" | 1 | $5–8 |
| Micro HDMI Cable | "Micro HDMI to HDMI cable 6ft" | 1 | $8–12 |
| Pi 5 Case with Fan | "Raspberry Pi 5 case active cooler" | 1 | $10–15 |
| Flexible Camera Mount | "Flexible gooseneck camera mount" | 1 | $10–15 |

> Cheapest Pi 5 in USA: Micro Center stores sell Pi 5 8GB for $89 vs $129 on Amazon.

---

## 📐 Wiring Reference

**ESP32 to IR Sensor connections:**

| ESP32 Pin | IR Sensor Wire | Purpose |
|-----------|----------------|---------|
| VIN (5V) | Brown | Power |
| GND | Blue | Ground |
| GPIO16 | Black (Sensor 1) | Entry Point A signal |
| GPIO17 | Black (Sensor 2) | Exit Point B signal |
| GPIO2 | — | Built-in status LED |

**Track mounting (top view):**
```
  [Sensor 1]──────────────────────[Sensor 2]
       ↑                                ↑
    Point A                          Point B
  (entry junction)               (exit junction)
  ←─────────── 3 foot shared section ──────────→
```

Both sensors face across the track so the train body breaks the beam when passing.

---

## 📦 BLE Command Reference

All commands write to Characteristic UUID `08590f7e-db05-467e-8757-72f6faeb13d4`:

| Command | Bytes | Description |
|---------|-------|-------------|
| Stop | `[0x00, 0x45, 0x00]` | Immediate stop |
| Speed 1 | `[0x00, 0x45, 0x01]` | Slowest |
| Speed 7 | `[0x00, 0x45, 0x07]` | Fastest |
| Forward | `[0x00, 0x46, 0x01]` | Direction forward |
| Reverse | `[0x00, 0x46, 0x02]` | Direction reverse |
| Bell ON | `[0x00, 0x47, 0x01]` | Ring bell |
| Bell OFF | `[0x00, 0x47, 0x00]` | Stop bell |
| Horn ON | `[0x00, 0x48, 0x01]` | Blow horn |
| Horn OFF | `[0x00, 0x48, 0x00]` | Stop horn |
| Lights ON | `[0x00, 0x51, 0x01]` | Headlights on |
| Lights OFF | `[0x00, 0x51, 0x00]` | Headlights off |
| Speak | `[0x00, 0x4D, 0x00, 0x00]` | Random conductor phrase |
| Volume | `[0x00, 0x4C, 0x00–0x07]` | Master volume |

> Protocol reverse-engineered by [Property404](https://github.com/Property404/lionchief-controller)

---

## 🧪 Running Tests

```bash
cd raspberry_pi
pip install pytest
pytest tests/ -v
```

Expected output:
```
tests/test_commands.py::TestCommandBytes::test_stop_command    PASSED
tests/test_commands.py::TestCommandBytes::test_resume_command  PASSED
tests/test_commands.py::TestUUIDs::test_service_uuid           PASSED
tests/test_zone_manager.py::TestZone::test_train_inside_zone   PASSED
...
28 passed in 0.66s
```

---

## ⚙️ Configuration Reference

### laptop_cv/config.py

| Setting | Default | Description |
|---------|---------|-------------|
| `TRAIN_MAC` | `""` | Train MAC — empty = auto-find by name |
| `TRAIN_NAME` | `"LC0"` | Scan for device with this name prefix |
| `CAMERA_INDEX` | `0` | 0=built-in webcam, 1=first USB camera |
| `ZONE_FILE` | `"shared_zone.json"` | Zone coordinates file |
| `MIN_DETECTION_AREA` | `800` | Min pixel area to count as train |
| `DETECTION_FRAMES_THRESHOLD` | `3` | Consecutive frames before triggering |
| `RESUME_DELAY_SECONDS` | `2.5` | Safety delay before resuming |
| `SHOW_VIDEO` | `True` | Show live camera window |

### raspberry_pi/config.py

| Setting | Default | Description |
|---------|---------|-------------|
| `TRAIN_MAC_ADDRESS` | `""` | Train MAC address |
| `MODEL_TYPE` | `"mock"` | `yolo` / `yolo_ncnn` / `tflite` / `mock` |
| `RESUME_SPEED` | `7` | Speed 0–31 after zone clears |
| `RESUME_DELAY` | `2.5` | Seconds before resuming |
| `CAMERA_INDEX` | `0` | Camera device index |
| `DETECTION_CONFIDENCE` | `0.50` | Minimum YOLO confidence |
| `SHOW_DISPLAY` | `True` | Show live camera window |

---

## 🗓 Development Roadmap

### Completed ✅

- [x] BLE command library — all commands verified on real train
- [x] Async BLE controller with auto-reconnect
- [x] ESP32 ESPHome YAML — ready to flash
- [x] ESP32 Arduino C++ — full train control via serial menu
- [x] Pygame mock simulation — 28/28 unit tests passing
- [x] OpenCV laptop CV detection — working
- [x] Zone calibration tool — auto-saves to shared_zone.json
- [x] BLE scanning by device name — no MAC hardcoding needed
- [x] Peter's train confirmed: LC015556-99F0, UUIDs verified ✅
- [x] GitHub repo live with complete code

### Pending ⏳

- [ ]  Scans train MAC on Android nRF Connect → update config.py
- [ ]  mounts IR sensors at Point A and Point B on track
- [ ] Flash ESP32 → test BLE STOP/RESUME on real train
- [ ] Mount USB webcam above layout → run calibrate_zone.py
- [ ] Test laptop_cv real-time detection with real trains running
- [ ]  records 15-20 minutes of layout footage (both trains)
- [ ] Extract frames → label on Roboflow → train YOLOv11n on Colab
- [ ] Export NCNN model → deploy on Raspberry Pi 5
- [ ] Full integration test — all three systems on real layout

---

## 🔗 Reference Repositories

| Repo | Purpose |
|------|---------|
| [Property404/lionchief-controller](https://github.com/Property404/lionchief-controller) | Original BLE protocol decoding |
| [idaband/lionchief-controller-raspberrypi](https://github.com/idaband/lionchief-controller-raspberrypi) | Pi-native BLE + extended commands |
| [chrcraven/LionchiefInteractiveDisplay](https://github.com/chrcraven/LionchiefInteractiveDisplay) | Web interface + mock mode |
| [hbldh/bleak](https://github.com/hbldh/bleak) | Python BLE library used throughout |
| [nkolban/ESP32_BLE_Arduino](https://github.com/nkolban/ESP32_BLE_Arduino) | ESP32 Arduino BLE library |
| [automaticdai/rpi-object-detection](https://github.com/automaticdai/rpi-object-detection) | YOLO on Raspberry Pi reference |
| [ultralytics/ultralytics](https://github.com/ultralytics/ultralytics) | YOLOv8 / YOLO11 framework |
| [roboflow/notebooks](https://github.com/roboflow/notebooks) | Training notebooks for Google Colab |

---

## 👨‍💻 Developer Notes

```
Train does not verify checksums:
  [0x00, 0x45, 0x00] is sufficient to stop it

ESP32 BLE limit:
  Maximum 3 simultaneous BLE connections
  Do not add more ble_client blocks in ESPHome YAML

iOS nRF Connect limitation:
  Apple privacy policy blocks MAC address display on iOS
  Use Android phone to get train MAC address

Zone detection logic:
  OpenCV background subtraction detects ANY movement in the zone
  No need to identify which train — if anything moves in zone, stop the other
  This works perfectly for collision prevention without YOLO

Development order recommendation:
  1. Test mock_train.py first (zero hardware needed)
  2. Test BLE via ESP32 serial menu
  3. Test laptop_cv with USB webcam
  4. Add IR sensors for hardware detection
  5. Add YOLO model after Peter records layout footage
```
