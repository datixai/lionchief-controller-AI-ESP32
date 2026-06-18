# Project 3 — Single Track Safe Distance System
**Harry Locomotive Project | Datix AI | June 2026**

Camera-based safe distance control for two trains running on the same outer loop track.

---

## How It Works

Two trains run on Track 3 (outer loop) in the same direction.  
The overhead ceiling camera watches both trains simultaneously.  
Train B (rear, BLE) automatically slows down or stops if it gets too close to Train A (front).

### Detection Method
Each train has a **colored sticker** placed on its roof:
- **Train A** (front, runs freely) → one bright color, e.g. **red**
- **Train B** (rear, BLE controlled) → different color, e.g. **yellow**

The camera detects each color blob every frame and measures the pixel distance between them. The speed controller maps this distance to a BLE speed command for Train B.

### Distance Zones
| Zone | Gap | Action |
|------|-----|--------|
| DANGER  | < 80px  | STOP Train B immediately |
| WARNING | < 150px | Slow to minimum speed |
| CAUTION | < 230px | Reduce speed |
| SAFE    | < 320px | Run at user-set speed |
| FAR     | > 450px | Speed up to catch up |

All distances are tunable in `config.py` or via `calibrate.py`.

---

## File Structure

```
single_track/
├── config.py           All settings — MAC, colors, distances, timing
├── calibrate.py        Color picker + distance calibration tool
├── train_detector.py   HSV color blob detection for both trains
├── speed_controller.py Gap → speed mapping with smoothing
├── ble_controller.py   Async BLE control for Train B
├── main.py             Main loop — detection + control + display
├── requirements.txt    Python packages to install
├── README.md           This file
└── SafeDistance_Guide.pdf  Client guide (send to Peter)
```

`calibration.json` is created by `calibrate.py` and read automatically.

---

## Setup (First Time)

### 1. Activate virtual environment
```
cd C:\Users\peter\Documents\Linchief_train
.venv\Scripts\activate
```

### 2. Install packages
```
pip install -r single_track\requirements.txt
```

### 3. Put stickers on trains
- Train A (front): bright RED sticker on roof
- Train B (rear):  bright YELLOW sticker on roof
- Stickers should be at least 2×2cm
- Must be clearly visible from ceiling camera

---

## Step 1 — Calibrate

```
cd single_track
python calibrate.py
```

**In the window:**
1. Press **TAB** to select Train A
2. **Click** on Train A's sticker in the live image → color is sampled
3. Press **TAB** to switch to Train B
4. **Click** on Train B's sticker → color sampled
5. Use **+** / **-** to adjust sensitivity until blobs are clean
6. Position trains at various gaps and press **D / W / C / S / F** to set distance thresholds
7. Press **A** to save → creates `calibration.json`

---

## Step 2 — Run

```
python main.py
```

Camera window opens. Both trains shown with colored circles.  
A distance line shows the current gap and zone.  
Train B speed adjusts automatically.

---

## Keyboard Commands

| Key | Action |
|-----|--------|
| `1-7` | Set Train B cruising speed |
| `+` / `-` | Speed up / down |
| `S` | Manual STOP Train B |
| `R` | Manual RESUME Train B |
| `P` | Pause / resume auto control |
| `H` | Horn toggle |
| `B` | Bell toggle |
| `L` | Lights toggle |
| `Q` / `ESC` | Quit |

---

## Tuning

Edit `config.py` to adjust behaviour:

| Setting | Default | Effect |
|---------|---------|--------|
| `DISTANCE_DANGER` | 80 | Stop threshold (px) |
| `DISTANCE_WARNING` | 150 | Slow threshold (px) |
| `DISTANCE_SAFE` | 320 | Comfortable gap (px) |
| `MIN_BLOB_AREA` | 150 | Min sticker area — increase if false detections |
| `ALPHA_SLOW_DOWN` | 0.7 | Braking response (higher = faster) |
| `ALPHA_SPEED_UP` | 0.25 | Acceleration response (lower = smoother) |
| `MIN_COMMAND_INTERVAL_MS` | 300 | BLE command rate limit |
| `MISSING_TIMEOUT_S` | 2.0 | Seconds before "train not seen" safety |

---

## Changing the BLE Train

Update only `TRAIN_B_MAC` in `config.py`. Everything else stays the same.

---

## Troubleshooting

| Problem | Fix |
|---------|-----|
| Train not detected | Re-run calibrate.py — re-sample color with current lighting |
| False detections | Increase `MIN_BLOB_AREA` or reduce `+` sensitivity in calibrate.py |
| Jerky speed | Reduce `ALPHA_SLOW_DOWN` for smoother braking |
| BLE not connecting | Turn phone Bluetooth OFF — check MAC in config.py |
| Both blobs same color | Use more distinct sticker colors |
