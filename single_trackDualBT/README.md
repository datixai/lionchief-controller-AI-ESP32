# single_trackBT — Dual BLE Safe Distance
**Harry Locomotive Project 3 BT | Datix AI | June 2026**

Same as `single_track` but **both trains are Bluetooth controlled**.

## Key Difference — Cooperative Control
Because both trains are BLE, the system adjusts **both** speeds:

| Zone | Train A (front) | Train B (rear) |
|------|----------------|----------------|
| DANGER | STOP | STOP |
| WARNING | Speed up +1 | Slow to min |
| CAUTION | Hold user speed | Slow to caution |
| SAFE | Hold user speed | Hold user speed |
| FAR | Slow down -1 | Speed up to catch |

## Update MACs
Open `config.py` and set both MACs before running:
```python
TRAIN_A_MAC = "XX:XX:XX:XX:XX:XX"   # front train
TRAIN_B_MAC = "XX:XX:XX:XX:XX:XX"   # rear train
```

## Run
```
.venv\Scripts\activate
cd single_trackBT
pip install -r requirements.txt
python calibrate.py   # first time only
python main.py
```

## Keyboard
| Key | Action |
|-----|--------|
| `1-7` | Set both trains' speed |
| `E` | Emergency stop BOTH trains |
| `S` | Stop both |
| `R` | Resume both |
| `P` | Pause/resume auto control |
| `Q` | Quit |

## Files
- `config.py` — Two MACs, all settings
- `calibrate.py` — Same as single_track
- `train_detector.py` — Same as single_track
- `speed_controller.py` — Cooperative dual control
- `ble_controller.py` — Manages both BLE trains
- `main.py` — Main loop
