# ══════════════════════════════════════════════════════════════════
#  calibrate.py  —  Color & Distance Calibration Tool
#  Harry Locomotive Project 3  |  Datix AI  |  June 2026
#
#  STEP 1:  Place colored stickers on both train roofs.
#           Use bright solid colors that differ from the track.
#           Good choices:  bright RED, YELLOW, GREEN, BLUE, PINK
#
#  STEP 2:  Run this tool:
#             python calibrate.py
#
#  STEP 3:  Click on Train A sticker color in the camera view.
#           Click on Train B sticker color in the camera view.
#
#  STEP 4:  Use slider to adjust color sensitivity until the
#           blob masks look clean (solid white over each sticker
#           only, no stray noise elsewhere).
#
#  STEP 5:  Position trains at various distances and press keys
#           D, W, C, S, F to set each distance zone threshold.
#
#  STEP 6:  Press A to save all settings → calibration.json
#
#  KEYBOARD:
#    TAB    Switch active train (A ↔ B)
#    CLICK  Sample color at cursor position
#    +/-    Adjust color range sensitivity
#    D      Set DANGER  distance (trains dangerously close now)
#    W      Set WARNING distance (trains too close, slow down)
#    C      Set CAUTION distance (start reducing speed)
#    S      Set SAFE    distance (comfortable following gap)
#    F      Set FAR     distance (gap too large, catch up)
#    M      Toggle mask preview
#    A      Save all and quit
#    Q/ESC  Quit without saving
# ══════════════════════════════════════════════════════════════════

import cv2
import numpy as np
import json
import os
import sys

import config

CAL_FILE = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), config.CALIBRATION_FILE)

# ── Calibration state ─────────────────────────────────────────────
active_train = "A"      # currently editing A or B

# Per-train HSV calibration
train_cal = {
    "A": {
        "hsv_lower1":  list(config.TRAIN_A_HSV_LOWER1),
        "hsv_upper1":  list(config.TRAIN_A_HSV_UPPER1),
        "hsv_lower2":  list(config.TRAIN_A_HSV_LOWER2),
        "hsv_upper2":  list(config.TRAIN_A_HSV_UPPER2),
        "dual_range":  config.TRAIN_A_USES_DUAL,
        "color_name":  "red",
        "sample_h":    -1,  # sampled hue, -1 = not set
    },
    "B": {
        "hsv_lower1":  list(config.TRAIN_B_HSV_LOWER),
        "hsv_upper1":  list(config.TRAIN_B_HSV_UPPER),
        "hsv_lower2":  [0, 0, 0],
        "hsv_upper2":  [0, 0, 0],
        "dual_range":  False,
        "color_name":  "yellow",
        "sample_h":    -1,
    },
}

# Distance thresholds (pixels)
dist_cal = {
    "danger_px":  config.DISTANCE_DANGER,
    "warning_px": config.DISTANCE_WARNING,
    "caution_px": config.DISTANCE_CAUTION,
    "safe_px":    config.DISTANCE_SAFE,
    "far_px":     config.DISTANCE_FAR,
}

sensitivity   = 20    # ± H range around sampled hue
show_mask     = True
cursor_pos    = (0, 0)
last_distance = None  # most recent measured distance in pixels

# Train display colors (BGR)
TRAIN_COLORS = {
    "A": (0,   165, 255),  # orange
    "B": (0,   220, 220),  # yellow-ish
}


# ── Helpers ───────────────────────────────────────────────────────

def build_mask(hsv, cal):
    lo1 = np.array(cal["hsv_lower1"])
    hi1 = np.array(cal["hsv_upper1"])
    mask = cv2.inRange(hsv, lo1, hi1)
    if cal["dual_range"]:
        lo2  = np.array(cal["hsv_lower2"])
        hi2  = np.array(cal["hsv_upper2"])
        mask = cv2.bitwise_or(mask, cv2.inRange(hsv, lo2, hi2))
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    mask = cv2.dilate(mask, kernel, iterations=1)
    return mask


def find_centroid(mask):
    contours, _ = cv2.findContours(
        mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    largest = max(contours, key=cv2.contourArea)
    if cv2.contourArea(largest) < config.MIN_BLOB_AREA:
        return None
    M = cv2.moments(largest)
    if M["m00"] == 0:
        return None
    return (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))


def sample_color_at(frame, x, y, radius=5):
    """Sample HSV at (x,y) from a small patch and return mean H, S, V."""
    h, w = frame.shape[:2]
    x1 = max(0, x - radius)
    x2 = min(w, x + radius + 1)
    y1 = max(0, y - radius)
    y2 = min(h, y + radius + 1)
    patch     = frame[y1:y2, x1:x2]
    hsv_patch = cv2.cvtColor(patch, cv2.COLOR_BGR2HSV)
    mean_h = float(np.mean(hsv_patch[:, :, 0]))
    mean_s = float(np.mean(hsv_patch[:, :, 1]))
    mean_v = float(np.mean(hsv_patch[:, :, 2]))
    return mean_h, mean_s, mean_v


def set_color_range(train_key, h, s, v, sens):
    """Set HSV range for a train based on sampled pixel."""
    cal  = train_cal[train_key]
    sens_s = max(10, 255 - int(s) + 20)   # be generous on S/V

    # Detect red (hue wraps around 0/180 boundary)
    is_red = h < 15 or h > 165

    if is_red:
        cal["dual_range"] = True
        cal["hsv_lower1"] = [0,           max(50, int(s)-50), max(50, int(v)-60)]
        cal["hsv_upper1"] = [min(10, int(h)+sens), 255, 255]
        cal["hsv_lower2"] = [max(165, int(h)-sens), max(50, int(s)-50), max(50, int(v)-60)]
        cal["hsv_upper2"] = [180,         255, 255]
        cal["color_name"] = "red"
    else:
        cal["dual_range"] = False
        lo_h = max(0,   int(h) - sens)
        hi_h = min(179, int(h) + sens)
        lo_s = max(40,  int(s) - 60)
        lo_v = max(40,  int(v) - 60)
        cal["hsv_lower1"] = [lo_h, lo_s, lo_v]
        cal["hsv_upper1"] = [hi_h, 255,  255]
        cal["color_name"] = f"hue{int(h)}"

    cal["sample_h"] = h
    print(f"  [Train {train_key}] Sampled H={h:.0f} S={s:.0f} V={v:.0f} "
          f"{'(dual/red)' if cal['dual_range'] else ''}")
    print(f"  Range: {cal['hsv_lower1']} → {cal['hsv_upper1']}")


def save_calibration():
    data = {
        "train_a": {
            k: v for k, v in train_cal["A"].items()
        },
        "train_b": {
            k: v for k, v in train_cal["B"].items()
        },
        "distances": dist_cal,
    }
    with open(CAL_FILE, "w") as f:
        json.dump(data, f, indent=2)
    print(f"\n  ✅ Calibration saved → {CAL_FILE}")


# ── Mouse callback ────────────────────────────────────────────────
_last_frame_bgr = None

def mouse_callback(event, x, y, flags, param):
    global cursor_pos, _last_frame_bgr
    cursor_pos = (x, y)
    if event == cv2.EVENT_LBUTTONDOWN and _last_frame_bgr is not None:
        h, s, v = sample_color_at(_last_frame_bgr, x, y)
        set_color_range(active_train, h, s, v, sensitivity)


# ── Main ──────────────────────────────────────────────────────────

def main():
    global active_train, sensitivity, show_mask
    global cursor_pos, last_distance, _last_frame_bgr

    print("╔══════════════════════════════════════════════════════╗")
    print("║  LionChief — Color & Distance Calibration           ║")
    print("╠══════════════════════════════════════════════════════╣")
    print("║  CLICK  = Sample color for active train             ║")
    print("║  TAB    = Switch Train A ↔ B                        ║")
    print("║  + / -  = Adjust color sensitivity                  ║")
    print("║  D      = Set DANGER  distance (trains very close)  ║")
    print("║  W      = Set WARNING distance                      ║")
    print("║  C      = Set CAUTION distance                      ║")
    print("║  S      = Set SAFE    distance (ideal following)    ║")
    print("║  F      = Set FAR     distance (catch up needed)    ║")
    print("║  M      = Toggle mask preview                       ║")
    print("║  A      = SAVE ALL and quit                         ║")
    print("║  Q/ESC  = Quit without saving                       ║")
    print("╚══════════════════════════════════════════════════════╝\n")

    cap = cv2.VideoCapture(config.CAMERA_INDEX, cv2.CAP_DSHOW)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  config.CAMERA_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, config.CAMERA_HEIGHT)
    cap.set(cv2.CAP_PROP_FPS,          config.CAMERA_FPS)

    if not cap.isOpened():
        print(f"❌ Cannot open camera {config.CAMERA_INDEX}")
        sys.exit(1)

    WIN = "Calibration — Single Track Safe Distance"
    cv2.namedWindow(WIN)
    cv2.setMouseCallback(WIN, mouse_callback)

    while True:
        ret, frame = cap.read()
        if not ret:
            continue

        _last_frame_bgr = frame.copy()
        h_frame, w_frame = frame.shape[:2]

        blurred = cv2.GaussianBlur(frame, (5, 5), 0)
        hsv     = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        # Build masks for both trains
        mask_a = build_mask(hsv, train_cal["A"])
        mask_b = build_mask(hsv, train_cal["B"])

        # Find centroids
        pos_a = find_centroid(mask_a)
        pos_b = find_centroid(mask_b)

        # Calculate distance if both visible
        if pos_a and pos_b:
            dx = pos_a[0] - pos_b[0]
            dy = pos_a[1] - pos_b[1]
            last_distance = float(np.sqrt(dx * dx + dy * dy))
        else:
            last_distance = None

        # ── Build display ──────────────────────────────────────
        display = frame.copy()

        # Mask overlay
        if show_mask:
            # Colour the masks on the display
            mask_a_3ch = cv2.cvtColor(mask_a, cv2.COLOR_GRAY2BGR)
            mask_b_3ch = cv2.cvtColor(mask_b, cv2.COLOR_GRAY2BGR)
            a_overlay = np.zeros_like(display)
            b_overlay = np.zeros_like(display)
            a_overlay[mask_a > 0] = (0,  165, 255)  # orange = Train A
            b_overlay[mask_b > 0] = (0,  220, 220)  # cyan   = Train B
            display = cv2.addWeighted(display, 0.7, a_overlay, 0.5, 0)
            display = cv2.addWeighted(display, 1.0, b_overlay, 0.5, 0)

        # Draw train centroids and labels
        if pos_a:
            cv2.circle(display, pos_a, 10, (0, 165, 255), 2)
            cv2.putText(display, "A", (pos_a[0]+12, pos_a[1]-8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 165, 255), 2)
        if pos_b:
            cv2.circle(display, pos_b, 10, (0, 220, 50), 2)
            cv2.putText(display, "B", (pos_b[0]+12, pos_b[1]-8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 220, 50), 2)

        # Draw distance line
        if pos_a and pos_b:
            cv2.line(display, pos_a, pos_b, (255, 255, 0), 1)
            mid_x = (pos_a[0] + pos_b[0]) // 2
            mid_y = (pos_a[1] + pos_b[1]) // 2
            cv2.putText(display,
                        f"{last_distance:.0f}px",
                        (mid_x + 6, mid_y - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)

        # Cursor crosshair
        cx, cy = cursor_pos
        cv2.line(display, (cx - 12, cy), (cx + 12, cy), (220, 220, 220), 1)
        cv2.line(display, (cx, cy - 12), (cx, cy + 12), (220, 220, 220), 1)

        # ── Status panel (top) ────────────────────────────────
        cv2.rectangle(display, (0, 0), (w_frame, 56), (20, 20, 20), -1)

        active_col = (0, 165, 255) if active_train == "A" else (0, 220, 50)
        cv2.putText(display,
                    f"ACTIVE: Train {active_train} | "
                    f"Sensitivity: ±{sensitivity} | "
                    f"Mask: {'ON' if show_mask else 'OFF'} | "
                    f"TAB=Switch  +/-=Sensitivity  M=Mask  A=Save  Q=Quit",
                    (8, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (220, 220, 220), 1)

        dist_str = f"{last_distance:.0f}px" if last_distance else "---"
        cv2.putText(display,
                    f"Gap: {dist_str} | "
                    f"D={dist_cal['danger_px']}  "
                    f"W={dist_cal['warning_px']}  "
                    f"C={dist_cal['caution_px']}  "
                    f"S={dist_cal['safe_px']}  "
                    f"F={dist_cal['far_px']}  "
                    f"  [CLICK train sticker to sample color]",
                    (8, 42), cv2.FONT_HERSHEY_SIMPLEX, 0.45, active_col, 1)

        # ── Calibration status (right panel) ──────────────────
        cal_a = train_cal["A"]
        cal_b = train_cal["B"]
        r_x   = w_frame - 240
        cv2.rectangle(display, (r_x, 60), (w_frame, 180), (30, 30, 30), -1)
        cv2.putText(display, "Train A (front — no BLE)",
                    (r_x + 6, 78), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0, 165, 255), 1)
        a_sampled = "✅ sampled" if cal_a["sample_h"] >= 0 else "❌ not set — click sticker"
        cv2.putText(display, f"  {cal_a['color_name']} — {a_sampled}",
                    (r_x + 6, 96), cv2.FONT_HERSHEY_SIMPLEX, 0.36, (180, 180, 180), 1)
        cv2.putText(display, f"  Blob: {'found' if pos_a else 'NOT FOUND'}",
                    (r_x + 6, 112), cv2.FONT_HERSHEY_SIMPLEX, 0.36,
                    (0, 200, 0) if pos_a else (0, 0, 220), 1)

        cv2.putText(display, "Train B (rear — BLE controlled)",
                    (r_x + 6, 135), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0, 220, 50), 1)
        b_sampled = "✅ sampled" if cal_b["sample_h"] >= 0 else "❌ not set — click sticker"
        cv2.putText(display, f"  {cal_b['color_name']} — {b_sampled}",
                    (r_x + 6, 153), cv2.FONT_HERSHEY_SIMPLEX, 0.36, (180, 180, 180), 1)
        cv2.putText(display, f"  Blob: {'found' if pos_b else 'NOT FOUND'}",
                    (r_x + 6, 169), cv2.FONT_HERSHEY_SIMPLEX, 0.36,
                    (0, 200, 0) if pos_b else (0, 0, 220), 1)

        cv2.imshow(WIN, display)
        key = cv2.waitKey(20) & 0xFF

        # ── Key handling ───────────────────────────────────────
        if key in (ord('q'), ord('Q'), 27):
            print("  Quit — no changes saved.")
            break

        elif key == 9:   # TAB — switch active train
            active_train = "B" if active_train == "A" else "A"
            print(f"  Active train: {active_train}")

        elif key in (ord('+'), ord('=')):
            sensitivity = min(40, sensitivity + 2)
            # Re-apply range if already sampled
            cal = train_cal[active_train]
            if cal["sample_h"] >= 0:
                set_color_range(active_train, cal["sample_h"], 150, 150, sensitivity)
            print(f"  Sensitivity: ±{sensitivity}")

        elif key == ord('-'):
            sensitivity = max(4, sensitivity - 2)
            cal = train_cal[active_train]
            if cal["sample_h"] >= 0:
                set_color_range(active_train, cal["sample_h"], 150, 150, sensitivity)
            print(f"  Sensitivity: ±{sensitivity}")

        elif key in (ord('m'), ord('M')):
            show_mask = not show_mask

        elif key in (ord('d'), ord('D')):
            if last_distance:
                dist_cal["danger_px"] = int(last_distance)
                print(f"  DANGER  distance set: {int(last_distance)}px")
            else:
                print("  Cannot set — both trains must be visible")

        elif key in (ord('w'), ord('W')):
            if last_distance:
                dist_cal["warning_px"] = int(last_distance)
                print(f"  WARNING distance set: {int(last_distance)}px")
            else:
                print("  Cannot set — both trains must be visible")

        elif key in (ord('c'), ord('C')):
            if last_distance:
                dist_cal["caution_px"] = int(last_distance)
                print(f"  CAUTION distance set: {int(last_distance)}px")
            else:
                print("  Cannot set — both trains must be visible")

        elif key in (ord('s'), ord('S')):
            if last_distance:
                dist_cal["safe_px"] = int(last_distance)
                print(f"  SAFE    distance set: {int(last_distance)}px")
            else:
                print("  Cannot set — both trains must be visible")

        elif key in (ord('f'), ord('F')):
            if last_distance:
                dist_cal["far_px"] = int(last_distance)
                print(f"  FAR     distance set: {int(last_distance)}px")
            else:
                print("  Cannot set — both trains must be visible")

        elif key in (ord('a'), ord('A')):
            save_calibration()
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
