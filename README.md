# 🚂 Harry Locomotive — Collision Prevention System

Automated collision prevention for Lionel LionChief O-scale model train layout.
Two trains share a 3-foot section of track. This system detects when the inner
loop train enters the shared section and automatically stops the outer loop train
via Bluetooth (BLE), then resumes it when the track clears.

---

## 📁 Project Structure

```
harry_locomotive/
├── esp32/
│   ├── esphome/
│   │   ├── train_controller.yaml      ← Flash to ESP32 via ESPHome
│   │   └── secrets.yaml.example       ← Copy to secrets.yaml, fill in WiFi
│   └── arduino/
│       └── train_controller/
│           └── train_controller.ino   ← Alternative: raw Arduino C++
│
├── raspberry_pi/
│   ├── main.py                        ← ENTRY POINT — run this
│   ├── config.py                      ← All settings in one place
│   ├── calibrate_zone.py              ← Draw detection zone with mouse
│   ├── scan_train.py                  ← Find train's BLE MAC address
│   ├── mock_train.py                  ← Test everything without hardware
│   ├── requirements.txt
│   ├── .env.example
│   │
│   ├── lionchief/                     ← BLE train control package
│   │   ├── __init__.py
│   │   ├── controller.py              ← Async BLE controller (bleak)
│   │   └── commands.py                ← All LionChief command bytes
│   │
│   ├── detection/                     ← AI detection package
│   │   ├── __init__.py
│   │   ├── detector.py                ← YOLO / TFLite / Mock detectors
│   │   └── zone_manager.py            ← Shared zone overlap logic
│   │
│   └── tests/
│       ├── test_commands.py           ← Unit tests for BLE commands
│       └── test_zone_manager.py       ← Unit tests for zone logic
│
├── models/                            ← Place trained models here (gitignored)
│   └── README.md
│
├── docs/
│   └── wiring_diagram.md              ← Pin connections + track layout
│
└── .gitignore
```

---

## ⚡ Quick Start

### Option A — Test Without Any Hardware (Start Here)

```bash
# 1. Clone repo
git clone https://github.com/YOUR_USERNAME/harry-locomotive.git
cd harry-locomotive/raspberry_pi

# 2. Create virtual environment
python -m venv venv
source venv/bin/activate        # Windows: venv\Scripts\activate

# 3. Install minimal deps
pip install bleak

# 4. Run full simulation — keyboard controlled
python mock_train.py
```

**Controls in mock mode:**
- `T` — inner train ENTERS zone
- `C` — inner train CLEARS zone
- `S` — emergency stop
- `R` — resume
- `Q` — quit

---

### Option B — Raspberry Pi + Camera (AI Approach)

```bash
# 1. Install all dependencies
pip install -r requirements.txt

# 2. Find Peter's train MAC address
python scan_train.py
# → copy the MAC address shown

# 3. Edit config.py
#    Set TRAIN_MAC_ADDRESS = "AA:BB:CC:DD:EE:FF"
#    Set MODEL_TYPE = "mock" first to test without model

# 4. Calibrate the shared zone
python calibrate_zone.py
# → click and drag over the shared track section in the camera window
# → press S to save

# 5. Run the system (mock mode first)
python main.py --mock

# 6. When ready with real model, switch model type
#    Set MODEL_TYPE = "yolo_ncnn" in config.py
#    Copy models/best_ncnn_model/ from your Colab training
python main.py
```

---

### Option C — ESP32 + IR Sensors (Hardware Approach)

#### Using ESPHome (Recommended)

```bash
# 1. Install ESPHome
pip install esphome

# 2. Copy and fill in secrets
cp esp32/esphome/secrets.yaml.example esp32/esphome/secrets.yaml
# Edit secrets.yaml with Peter's WiFi credentials

# 3. Edit train_controller.yaml
#    Replace XX:XX:XX:XX:XX:XX with Peter's train MAC address

# 4. Flash to ESP32 (connect via USB)
cd esp32/esphome
esphome run train_controller.yaml
```

#### Using Arduino IDE (Alternative)

1. Install Arduino IDE
2. Install `ESP32 BLE Arduino` library by Neil Kolban
3. Open `esp32/arduino/train_controller/train_controller.ino`
4. Set your train MAC in `#define TRAIN_MAC_ADDRESS`
5. Select **Tools → Board → ESP32 Dev Module**
6. Upload

---

## 🔑 Confirmed Train BLE Details

Peter's train has been identified:

| Setting | Value |
|---------|-------|
| Service UUID | `e20a39f4-73f5-4bc4-a12f-17d1ad07a961` ✅ |
| Characteristic UUID | `08590f7e-db05-467e-8757-72f6faeb13d4` ✅ |
| MAC Address | **Still needed** — Peter uses nRF Connect app |

**How Peter finds the MAC:**
1. Download **nRF Connect** (free, Google Play / App Store)
2. Power on the train
3. Tap Scan
4. Find "LionChief" in the list
5. Copy the MAC address (format: `AA:BB:CC:DD:EE:FF`)

Or run:
```bash
python raspberry_pi/scan_train.py
```

---

## 🤖 Training the AI Model

Since hardware is in the US and you're in Pakistan, train the model remotely:

### Step 1 — Collect Training Data (No Train Needed)

```bash
# Install yt-dlp
pip install yt-dlp

# Download Lionel LionChief running video from YouTube
yt-dlp "https://www.youtube.com/results?search_query=lionel+lionchief+O+scale+running" -o train_video.mp4

# Extract frames (2 per second)
ffmpeg -i train_video.mp4 -vf fps=2 frames/frame_%04d.jpg
```

### Step 2 — Label Images

1. Go to [roboflow.com](https://roboflow.com) → Create free account
2. New Project → Object Detection → Upload your frames
3. Draw bounding boxes around the train → label as `train`
4. Apply augmentations (flip, brightness) to multiply dataset
5. Export as **YOLOv8 PyTorch** format → copy the snippet

### Step 3 — Train on Google Colab (Free GPU)

Open: [YOLOv8 Training Notebook](https://colab.research.google.com/github/roboflow-ai/notebooks/blob/main/notebooks/train-yolov8-object-detection-on-custom-dataset.ipynb)

```python
# Paste your Roboflow snippet, then:
from ultralytics import YOLO

model = YOLO('yolov8n.pt')          # nano = fastest on Pi
model.train(
    data='dataset/data.yaml',
    epochs=100,
    imgsz=640,
    batch=16,
)
# Download runs/detect/train/weights/best.pt
```

### Step 4 — Convert for Raspberry Pi

```python
# Run in Colab before downloading
model = YOLO('best.pt')
model.export(format='ncnn', imgsz=320)   # creates best_ncnn_model/
```

### Step 5 — Deploy

1. Copy `best_ncnn_model/` folder to `harry-locomotive/models/`
2. Set `MODEL_TYPE = "yolo_ncnn"` in `config.py`
3. Run `python main.py`

---

## 📐 Wiring

See **[docs/wiring_diagram.md](docs/wiring_diagram.md)** for full ASCII wiring diagrams.

**Quick reference:**

| ESP32 Pin | Connects To |
|-----------|-------------|
| GPIO16 | IR Sensor 1 OUT (entry beam) |
| GPIO17 | IR Sensor 2 OUT (exit beam) |
| 3.3V | IR Sensor VCC |
| GND | IR Sensor GND |
| GPIO2 | Status LED (built-in) |
| GPIO0 | Emergency stop button (boot btn) |

---

## 🧪 Running Tests

```bash
cd raspberry_pi
pip install pytest
pytest tests/ -v
```

Expected output:
```
tests/test_commands.py::TestCommandBytes::test_stop_command PASSED
tests/test_commands.py::TestUUIDs::test_service_uuid_matches_peter_train PASSED
tests/test_zone_manager.py::TestZone::test_detection_fully_inside_zone PASSED
...
```

---

## ⚙️ Configuration Reference

All settings are in `raspberry_pi/config.py`:

| Setting | Default | Description |
|---------|---------|-------------|
| `TRAIN_MAC_ADDRESS` | `""` | Peter's train MAC (required for real use) |
| `MODEL_TYPE` | `"mock"` | `yolo` / `yolo_ncnn` / `tflite` / `mock` |
| `RESUME_SPEED` | `7` | Speed (0–31) after zone clears |
| `RESUME_DELAY` | `2.5` | Seconds to wait before resuming |
| `SHARED_ZONE` | `(150,150,490,330)` | Pixel coords — use calibrate_zone.py |
| `CAMERA_INDEX` | `0` | USB camera device index |
| `DETECTION_CONFIDENCE` | `0.50` | Minimum detection confidence |
| `SHOW_DISPLAY` | `True` | Show live camera window |

---

## 📦 LionChief BLE Command Reference

All commands send to Characteristic UUID `08590f7e-db05-467e-8757-72f6faeb13d4`:

| Command | Bytes | Description |
|---------|-------|-------------|
| Stop | `[0x00, 0x45, 0x00]` | Immediate stop |
| Speed (0–31) | `[0x00, 0x45, 0x00–0x1F]` | Set speed |
| Forward | `[0x00, 0x46, 0x01]` | Set direction |
| Reverse | `[0x00, 0x46, 0x02]` | Set direction |
| Bell ON | `[0x00, 0x47, 0x01]` | Ring bell |
| Bell OFF | `[0x00, 0x47, 0x00]` | |
| Horn ON | `[0x00, 0x48, 0x01]` | Blow horn |
| Horn OFF | `[0x00, 0x48, 0x00]` | |
| Lights ON | `[0x00, 0x51, 0x01]` | Headlights |
| Lights OFF | `[0x00, 0x51, 0x00]` | |
| Speak | `[0x00, 0x4D, 0x00, 0x00]` | Random phrase |
| Volume | `[0x00, 0x4C, 0x00–0x07]` | Master volume |

*Protocol reverse-engineered by Property404 — github.com/Property404/lionchief-controller*

---

## 🔗 Key Reference Repositories

| Repo | Purpose |
|------|---------|
| [Property404/lionchief-controller](https://github.com/Property404/lionchief-controller) | Original BLE protocol decoding |
| [idaband/lionchief-controller-raspberrypi](https://github.com/idaband/lionchief-controller-raspberrypi) | Pi-native BLE + extended commands |
| [chrcraven/LionchiefInteractiveDisplay](https://github.com/chrcraven/LionchiefInteractiveDisplay) | Full web interface + mock mode |
| [hbldh/bleak](https://github.com/hbldh/bleak) | Modern Python BLE library |
| [nkolban/ESP32_BLE_Arduino](https://github.com/nkolban/ESP32_BLE_Arduino) | ESP32 Arduino BLE library |
| [automaticdai/rpi-object-detection](https://github.com/automaticdai/rpi-object-detection) | YOLO on Raspberry Pi |
| [ultralytics/ultralytics](https://github.com/ultralytics/ultralytics) | YOLOv8/YOLO11 framework |
| [roboflow/notebooks](https://github.com/roboflow/notebooks) | Training notebooks (Colab) |

---

## 🗓 Development Roadmap

- [x] BLE command library (all commands verified)
- [x] Async controller with auto-reconnect
- [x] YOLO / TFLite / Mock detector
- [x] Zone detection logic
- [x] ESP32 ESPHome YAML
- [x] ESP32 Arduino C++
- [x] Mock simulation
- [x] Unit tests
- [ ] Peter sends MAC address → update `config.py`
- [ ] Collect YouTube frames → label on Roboflow
- [ ] Train YOLOv8n on Colab → download best.pt
- [ ] Export to NCNN → deploy on Pi
- [ ] Peter mounts IR sensors → flash ESP32
- [ ] Full integration test on real layout

---

## 👨‍💻 Developer Notes

- The train **does not verify the checksum** — so `[0x00, 0x45, 0x00]` is all you need to stop it
- BLE max 3 connections per ESP32 — don't add more `ble_client` blocks
- Use `MODEL_TYPE = "mock"` for all development until hardware arrives
- The `mock_train.py` script runs the full collision logic without any hardware
- Run `pytest tests/` before committing to verify nothing broke
