# 📐 Wiring Diagram — Harry Locomotive Project

## ESP32 DevKit Pin Connections

```
                    ┌───────────────────────────────┐
                    │         ESP32 DevKit v1        │
                    │                                │
          USB ──────┤ USB                     3V3 ├──── VCC (IR Sensors)
                    │                         GND ├──── GND (IR Sensors)
                    │                             │
  IR Sensor 1 OUT ──┤ GPIO16  (INPUT_PULLUP)      │
  IR Sensor 2 OUT ──┤ GPIO17  (INPUT_PULLUP)      │
                    │                             │
  Status LED (+) ───┤ GPIO2   (OUTPUT) ← built-in │
                    │                             │
  Boot Button ──────┤ GPIO0   (INPUT_PULLUP)      │
                    │       (emergency stop)      │
                    └───────────────────────────────┘
```

---

## IR Beam Sensor Wiring (E18-D80NK or similar)

Each sensor has 3 wires:

```
  IR Sensor 1 (Entry of shared section)
  ┌─────────────────────────────────────┐
  │ VCC (Brown/Red)  ──────────── 3.3V  │  on ESP32
  │ GND (Black/Blue) ──────────── GND   │  on ESP32
  │ OUT (Yellow/White) ────────── GPIO16 │  on ESP32
  └─────────────────────────────────────┘

  IR Sensor 2 (Exit of shared section)
  ┌─────────────────────────────────────┐
  │ VCC (Brown/Red)  ──────────── 3.3V  │  on ESP32
  │ GND (Black/Blue) ──────────── GND   │  on ESP32
  │ OUT (Yellow/White) ────────── GPIO17 │  on ESP32
  └─────────────────────────────────────┘
```

> ⚠️ **Note:** Some IR sensors need 5V to work reliably. If sensors don't trigger,
> move VCC to the 5V pin on ESP32 instead of 3.3V. The OUT signal is still safe
> at 3.3V logic level for the ESP32 GPIO.

---

## Track Layout — Sensor Placement

```
OUTER LOOP ─────────────────────────────────────────────────
                                         │
                    ┌────────────────────┼────────────────┐
                    │    SHARED SECTION  │  (3 feet)      │
                    │                   │                  │
  IR SENSOR 1 ──→  │ ●                  │              ● ←── IR SENSOR 2
  (Entry beam)     │ │  ← Train →       │    ← Train → │   (Exit beam)
                   │ │                  │              │   │
                    └─┼──────────────────┼──────────────┘
                      │                  │
INNER LOOP ───────────┴──────────────────┴───────────────────

        │← ─ ─ ─ ─ ─ ─ ─ 3 feet  ─ ─ ─ ─ ─ ─ ─ →│
```

**Sensor 1 (Entry):** Place just BEFORE the shared section begins, on the inner loop side.
**Sensor 2 (Exit):** Place just AFTER the shared section ends, on the inner loop side.

---

## IR Beam Sensor Physical Mounting

```
  Top view of track cross-section:

  ┌──────────────────┐
  │   TRAIN TRACK    │
  └──────────────────┘
       ↑        ↑
  Emitter    Receiver
  (IR LED)   (Photodetector)

  Mount both facing each other across the track.
  Height: ~2–3cm above rail level.
  Gap: adjust sensitivity screw so LED is barely lit in clear state.
```

---

## Optional: Status LED Wiring

```
  GPIO2 ──── [220Ω resistor] ──── LED (+) ──── LED (-) ──── GND

  LED behavior:
  • Solid ON   = BLE connected, system running
  • Rapid blink = train stopped (zone active)
  • OFF         = not connected to train
  • Slow blink  = system alive but train not yet paired
```

---

## Raspberry Pi GPIO (Not Used for BLE)

The Raspberry Pi connects to the train via **Bluetooth (BLE) only**.
No physical wiring to the train is needed.

**USB Camera connection:**
```
  USB Camera ──── USB-A port on Raspberry Pi
  (any standard USB webcam)
```

**Optional: Pi Camera Module 3 (better quality):**
```
  Pi Camera ──── CSI ribbon cable connector on Raspberry Pi
  (change CAMERA_INDEX = 0 stays the same, Pi Camera shows as /dev/video0)
```

---

## Full System Block Diagram

```
┌─────────────────────────────────────────────────────────────┐
│                       PETER'S LAYOUT                         │
│                                                              │
│   IR Sensor 1 ──┐                                           │
│   IR Sensor 2 ──┤──→ ESP32 ──BLE──→ Outer Loop Train (STOP)│
│                 │     (Hardware Safety Layer)                │
│                 │                                            │
│   USB Camera ───┘──→ Raspberry Pi ──BLE──→ Outer Loop Train │
│                       (AI Camera Layer)                      │
│                                                              │
│   ← Both systems target the SAME train via BLE →            │
└─────────────────────────────────────────────────────────────┘

  If EITHER system detects the inner train → outer train stops.
  Double safety. Hardware = fast. AI = smart.
```

---

## Parts List for ESP32 Setup

| Component | Quantity | Notes |
|-----------|----------|-------|
| ESP32 DevKit (38-pin) | 1 | Any ESP32 DevKit v1 |
| E18-D80NK IR Beam Sensor | 2 | Adjustable range 3–80cm |
| Jumper wires (F-M) | 6 | 3 per sensor |
| Breadboard (mini) | 1 | For prototyping |
| USB Micro cable | 1 | Power + flashing |
| 5V USB adapter | 1 | Power supply |

**Total approx: $25–35 USD**

---

## Parts List for Raspberry Pi Setup

| Component | Quantity | Notes |
|-----------|----------|-------|
| Raspberry Pi 5 (4GB) | 1 | Pi 4 also works |
| USB Webcam OR Pi Camera Module 3 | 1 | USB is easiest |
| MicroSD Card 32GB | 1 | Class 10 / A1 |
| Pi 5V USB-C Power Supply | 1 | Official recommended |

**Total approx: $100–160 USD**
