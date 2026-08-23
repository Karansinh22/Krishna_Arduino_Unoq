<p align="center">
  <img src="https://img.shields.io/badge/Arduino-UNO%20Q-00878F?style=for-the-badge&logo=arduino&logoColor=white" alt="Arduino UNO Q">
  <img src="https://img.shields.io/badge/AI_Model-YOLOv8_OBB-FF6F00?style=for-the-badge&logo=pytorch&logoColor=white" alt="YOLOv8">
  <img src="https://img.shields.io/badge/LiDAR-RPLIDAR_A1-6366F1?style=for-the-badge" alt="RPLIDAR A1">
  <img src="https://img.shields.io/badge/Python-3.10+-3776AB?style=for-the-badge&logo=python&logoColor=white" alt="Python">
  <img src="https://img.shields.io/badge/License-MIT-22C55E?style=for-the-badge" alt="MIT License">
</p>

<h1 align="center">🌾 Autonomous Agrobot</h1>
<h3 align="center">AI-Powered Precision Weed Control System for Arduino UNO Q</h3>

<p align="center">
  <b>Arduino Physical AI Challenge India 2026</b> · Robu.in × Arduino<br>
  Team <b>Krishna</b> · Registration ID: <code>APC-2026-GJ-59026</code><br>
  Track: <b>Industrial AI Sustainability</b><br>
  Birla Vishvakarma Mahavidyalaya Engineering College, Anand
</p>

---

## 📋 Table of Contents

- [Problem Statement](#-problem-statement)
- [How It Works](#-how-it-works)
- [Why Arduino UNO Q?](#-why-arduino-uno-q)
- [System Architecture](#-system-architecture)
- [Hardware BOM](#-hardware-bill-of-materials)
- [Pin Configuration](#-pin-configuration)
- [AI / ML Model](#-ai--ml-model-details)
- [Code Structure](#-code-structure)
- [Installation & Usage](#-installation--usage)
- [Web Dashboard](#-web-dashboard)
- [Testing & Results](#-testing--results)
- [Team](#-team)

---

## 🎯 Problem Statement

Manual and blanket-spray weeding in farm fields is **labour-intensive** and leads to **excess herbicide use**, raising costs and causing environmental harm. Smallholder farmers in particular face rising labour costs and lack access to affordable, selective weed-control technology.

This project addresses the need for a **low-cost, fully autonomous robot** that can distinguish weeds from crops and treat only the weeds — cutting chemical usage and manual effort in precision agriculture.

---

## ⚙️ How It Works

The Autonomous Agrobot is a precision-agriculture-based autonomous weeding robot designed to **identify and selectively target weeds**. The complete operational workflow follows a **detect → stop → spray → resume** cycle:

```
┌──────────────────────────────────────────────────────────────────┐
│                    SYSTEM INITIALIZATION                        │
│  LiDAR ✓  Cameras ✓  Motors ✓  Servos → 90° Home  Pump OFF    │
└──────────────────────┬───────────────────────────────────────────┘
                       ▼
         ┌─────────────────────────┐
         │  AUTONOMOUS NAVIGATION  │ ◄── LiDAR sector distances
         │  Follow crop row path   │     (Front / Left / Right)
         └────────────┬────────────┘
                      ▼
         ┌─────────────────────────┐
         │  VISUAL WEED DETECTION  │ ◄── YOLOv8 OBB inference
         │  Camera frames → ONNX   │     on dual webcam feed
         └────────────┬────────────┘
                      ▼ (weed detected)
         ┌─────────────────────────┐
         │  STOP DRIVE MOTORS      │
         │  Lock camera on target  │
         └────────────┬────────────┘
                      ▼
         ┌─────────────────────────┐
         │  POSITION SPRAY NOZZLE  │ ◄── Servo 2: 90° → 45°
         │  Articulate robotic arm │
         └────────────┬────────────┘
                      ▼
         ┌─────────────────────────┐
         │  ACTUATE SPRAY PUMP     │ ◄── Relay ON for 3.0 sec
         │  Apply herbicide        │     (12V diaphragm pump)
         └────────────┬────────────┘
                      ▼
         ┌─────────────────────────┐
         │  RESET & RESUME         │
         │  Nozzle → 90° Home      │
         │  Resume navigation path │
         │  Cooldown: 4 sec        │
         └─────────────────────────┘
```

---

## 🧠 Why Arduino UNO Q?

The **Arduino UNO Q** serves as the **central computing and control platform** for the entire robot, integrating vision processing, sensor interfacing, navigation control, and actuator operation into a single system.

| Feature | Benefit |
|---------|---------|
| Dual-processor (MPU + MCU) | Linux runs Python/AI; MCU handles real-time GPIO |
| RouterBridge IPC | Zero-latency RPC calls between processors |
| USB host support | Connects LiDAR + dual webcams simultaneously |
| Single-board consolidation | Reduces wiring, power draw, and inter-controller delays |

Instead of relying on a separate single-board computer and microcontroller, the Arduino UNO Q consolidates these functions — reducing hardware complexity, power consumption, and communication delays.

---

## 🏗️ System Architecture

```
                              ┌──────────────────────────┐
                              │     Arduino UNO Q        │
                              │      (ABX00087)          │
                              ├──────────┬───────────────┤
                              │   MPU    │     MCU       │
                              │ (Linux)  │  (Zephyr)     │
                              │          │               │
                              │ Python   │ RouterBridge  │
                              │ Flask    │ RPC Handlers  │
                              │ ONNX RT  │               │
                              └────┬─────┴───────┬───────┘
                                   │             │
              ┌────────────────────┤             ├────────────────────┐
              │                    │             │                    │
        ┌─────▼─────┐     ┌───────▼──────┐  ┌───▼────────┐   ┌──────▼──────┐
        │  RPLiDAR   │     │ Webcam × 2   │  │ DC Motors  │   │ Servo × 4   │
        │  A1        │     │ Logitech     │  │ (4-wheel   │   │ (Spray Arm  │
        │            │     │ Brio 100     │  │  diff.     │   │  Nozzle)    │
        │ /dev/      │     │ /dev/video2  │  │  drive)    │   │ D6,D9,     │
        │ ttyUSB0    │     │ /dev/video4  │  │ D2-D5      │   │ D10,D11    │
        └────────────┘     └──────────────┘  └────────────┘   └─────────────┘
                                                  │
                                    ┌─────────────┤
                              ┌─────▼─────┐  ┌────▼───────┐
                              │ Relay × 2  │  │ Laser × 2  │
                              │ (Pump)     │  │ (Targeting)│
                              │ D7, D8     │  │ GPIO 12,13 │
                              └────────────┘  └────────────┘
```

---

## 🛒 Hardware Bill of Materials

| # | Component | Qty | Purpose |
|---|-----------|-----|---------|
| 1 | Arduino UNO Q (ABX00087) | 1 | Central controller (MPU + MCU) |
| 2 | RPLiDAR A1 | 1 | 360° mapping & autonomous navigation |
| 3 | Logitech Brio 100 Webcam | 2 | Front crop view + spray arm view |
| 4 | DC Geared Motors | 4 | Differential drive wheels |
| 5 | Motor Driver Module (H-Bridge) | 2 | Dual DC motor control |
| 6 | Metal Gear Servo Motors | 4 | 4-axis robotic spray arm |
| 7 | 12V Diaphragm Water Pump | 2 | Herbicide spray actuation |
| 8 | Spray Nozzle, Tubing & Reservoir | 1 set | Fluid delivery system |
| 9 | Relay Module (2-channel) | 1 | Pump switching circuit |
| 10 | LED/LCD Display Module | 1 | On-board status feedback |
| 11 | Li-Po Battery Pack + PDB | 1 | Power distribution |
| 12 | USB Hub | 1 | Multi-peripheral connectivity |
| 13 | Buck Converter | 1 | Voltage regulation |
| 14 | 3D-Printed PLA Chassis | 1 set | Structural frame |
| 15 | Wheels with Rubber Tyres | 4 | Traction & mobility |

---

## 📌 Pin Configuration

### Motor Driver (4-Wheel Differential Drive)
| Signal | Pin | Description |
|--------|-----|-------------|
| `L_DIR` | D2 | Left motor direction |
| `L_PWM` | D3 | Left motor speed (PWM) |
| `R_DIR` | D4 | Right motor direction |
| `R_PWM` | D5 | Right motor speed (PWM) |

### Robotic Spray Arm Servos (4-Axis)
| Servo | Pin | Function |
|-------|-----|----------|
| Servo 1 | D6 | Arm base pan |
| Servo 2 | D9 | Nozzle pitch / targeting |
| Servo 3 | D10 | Arm elevation |
| Servo 4 | D11 | Deflector gate |

### Actuators
| Device | Pin | Description |
|--------|-----|-------------|
| Relay 1 (Pump) | D7 | Diaphragm spray pump |
| Relay 2 (Valve) | D8 | Secondary pump / solenoid |
| Laser 1 | GPIO 12 | Visual targeting beam |
| Laser 2 | GPIO 13 | Alignment marker |

---

## 🤖 AI / ML Model Details

| Parameter | Value |
|-----------|-------|
| **Model** | Ultralytics YOLOv8m-OBB (Oriented Bounding Box) |
| **Training Platform** | Google Colab |
| **Accuracy** | 93% |
| **Dataset** | 3,167 annotated images |
| **Export Format** | ONNX (640 × 640 fixed input) |
| **Inference Runtime** | ONNX Runtime (CPU) |
| **Confidence Threshold** | 40% |
| **Classes** | `crop`, `weed` |

The OBB detection allows the system to represent plants according to their **actual orientation** instead of restricting detections to conventional horizontal bounding boxes. Detections below the confidence threshold are filtered out, and IoU-based NMS (0.45) removes duplicate boxes.

**Limitations:** Performance can be affected by lighting conditions, shadows, occlusion, motion blur, camera angle, and variations in plant appearance not represented in the training dataset.

---

## 📁 Code Structure

```
FINAL_UNO_Q/
├── app.yaml                  # Arduino UNO Q application manifest
├── README.md                 # This documentation
├── .gitignore                # Git ignore rules
│
├── sketch/                   # MCU firmware (Arduino C++)
│   ├── sketch.ino            # Motor, servo, relay & laser RPC handlers
│   └── sketch.yaml           # Build profile (arduino:zephyr platform)
│
└── python/                   # MPU application (Linux/Python)
    ├── main.py               # Flask server, LiDAR, camera & AI pipeline
    ├── requirements.txt      # Python dependencies
    ├── model/
    │   └── best.onnx         # YOLOv8m-OBB trained model weights
    └── web/
        └── index.html        # Real-time control dashboard UI
```

### Main Functions

| Function | File | Description |
|----------|------|-------------|
| `setup()` | `sketch.ino` | Initialises LiDAR, cameras, motor drivers, servos, and registers RPC endpoints via RouterBridge |
| `loop()` | `sketch.ino` | Main execution loop — RPC commands are processed asynchronously via RouterBridge |
| `autonomous_loop()` | `main.py` | Reads LiDAR sector data, runs obstacle-avoidance navigation, and coordinates drive commands |
| `detect_plants()` | `main.py` | Runs the YOLOv8 OBB crop/weed classification pipeline on live camera frames |
| `plant_relay_action()` | `main.py` | Positions the robotic arm over the target and actuates the pump for the spray sequence |

---

## 🚀 Installation & Usage

### Prerequisites
- **Arduino UNO Q** hardware board
- **Arduino App Lab** installed on host computer / board
- Python 3.10+ on the UNO Q Linux MPU
- GStreamer installed for webcam video capture
- RPLiDAR A1 connected via USB (`/dev/ttyUSB0`)
- Dual Logitech Brio 100 webcams connected at `/dev/video2` and `/dev/video4`

---

### 1. Launching via Arduino App Lab
1. Open **Arduino App Lab**.
2. Load / import the `FINAL_UNO_Q` project folder.
3. **Arduino App Lab** automatically parses `app.yaml`, compiles & flashes the `sketch/sketch.ino` firmware to the MCU, installs Python requirements, and executes `python/main.py` on the MPU.
4. Click **Run App** in Arduino App Lab.

---

### 2. Accessing the Web Dashboard
Open any web browser on the same local network and navigate to:
```text
http://<UNO-Q-IP>:8080
```
> **Tip:** Find your board's IP address by running `hostname -I` in the terminal or viewing it directly in **Arduino App Lab**.

---

## 🖥️ Web Dashboard

The real-time control dashboard provides:

| Feature | Description |
|---------|-------------|
| **Dual Camera Feed** | Live stream with YOLOv8 OBB bounding-box overlays |
| **LiDAR Polar Radar** | 360° scan visualization with sweep animation |
| **Sector Distances** | Front / Left / Right minimum obstacle distances (mm) |
| **Mode Switching** | Toggle between Manual Override and Autonomous Navigation |
| **WASD Drive Controls** | Keyboard-driven motor control with speed adjustment |
| **4-Axis Servo Sliders** | Individual servo angle control + preset buttons |
| **Pump & Laser Toggles** | Manual ON/OFF for all actuators |
| **Wheel Trim Calibration** | Left/Right PWM compensation sliders |
| **Activity Telemetry Log** | Timestamped system event terminal |
| **Emergency All-Off** | Master safety kill switch |

**Keyboard Shortcuts:**
| Key | Action |
|-----|--------|
| `W` / `A` / `S` / `D` | Forward / Left / Backward / Right (latching) |
| `Space` | Emergency stop motors |
| `Z` / `X` | Increase / decrease speed by 10% |

---

## 🧪 Testing & Results

The complete system was tested component-by-component as well as in integrated operation:

- ✅ **Drive System** — Stable differential steering across varying terrain
- ✅ **LiDAR Mapping** — Accurate 360° environmental scan and obstacle detection
- ✅ **Camera Pipeline** — Consistent real-time video with no significant lag
- ✅ **AI Detection** — 93% accuracy distinguishing crops from weeds
- ✅ **Spray Mechanism** — Precise servo arm positioning and timed pump actuation
- ✅ **Web Dashboard** — Real-time monitoring and manual override capability
- ✅ **Integrated Cycle** — Detect → Stop → Spray → Resume loop operates reliably

---

## 👥 Team

| Name | Email | Role |
|------|-------|------|
| Prince Prasad | 24me111@bvmengineering.ac.in | Mechanical Design |
| Mohit Kapadiya | 24me100@bvmengineering.ac.in | Mechanical Design |
| Tirth Prajapati | 24ec424@bvmengineering.ac.in | Electronics & Circuits |
| Desai Karansinh | 23it402@bvmengineering.ac.in | Software & AI |

---

## 📄 License

This project is licensed under the [MIT License](LICENSE).

---

<p align="center">
  <b>Arduino Physical AI Challenge India 2026</b> · Robu.in × Arduino<br>
  Built with ❤️ by Team Krishna
</p>
