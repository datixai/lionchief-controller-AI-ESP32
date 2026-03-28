# models/

Place your trained model files here.

## Expected files

| File | Used when | How to get it |
|------|-----------|---------------|
| `best.pt` | `MODEL_TYPE=yolo` (PC/dev) | Download from Google Colab after training |
| `best_ncnn_model/` | `MODEL_TYPE=yolo_ncnn` (Raspberry Pi) | Export from best.pt — see README |
| `model.tflite` | `MODEL_TYPE=tflite` (Pi Zero) | Convert from best.pt — see README |
| `labels.txt` | TFLite only | One label per line, e.g. `train` |

## Conversion commands

```bash
# PyTorch → NCNN (run on PC, copy folder to Pi)
from ultralytics import YOLO
model = YOLO("models/best.pt")
model.export(format="ncnn", imgsz=320)   # saves best_ncnn_model/

# PyTorch → TFLite
model.export(format="tflite", imgsz=320)  # saves best.tflite
```

## .gitignore

All model files are excluded from Git (too large for GitHub).
Use Google Drive or Roboflow to share models with team.
