from arduino.app_utils import *
from flask import Flask, Response, jsonify, request, send_from_directory
from pathlib import Path
import threading
import time
import os
import math
import random
import subprocess
import serial
import numpy as np
from PIL import Image, ImageDraw, ImageFont
import onnxruntime as ort
from io import BytesIO

BASE_DIR = Path(__file__).resolve().parent
WEB_DIR = BASE_DIR / "web"

app = Flask(__name__, static_folder=str(WEB_DIR), static_url_path="")

# ==========================================
# HARDWARE STATE & CONTROL LOCKS
# ==========================================
servo_angles = [90, 90, 90, 90]
laser_state = [False, False]
relay_state = [False, False]
motor_state = {
    "left": 0, "right": 0, "speed": 10, "direction": "stop",
    "left_trim": 5, "right_trim": 0
}
bridge_lock = threading.Lock()

# System activity telemetry log buffer
system_logs = [
    {"time": time.strftime("%H:%M:%S"), "msg": "Autonomous Agrobot UNO Q Controller initialized."}
]
log_lock = threading.Lock()

def add_log(message):
    timestamp = time.strftime("%H:%M:%S")
    with log_lock:
        system_logs.append({"time": timestamp, "msg": message})
        if len(system_logs) > 40:
            system_logs.pop(0)

# ==========================================
# RPLIDAR A1 / USB LIDAR & MAPPING
# ==========================================
LIDAR_PORT = "/dev/ttyUSB0"
LIDAR_BAUD = 115200
LIDAR_MIN_VALID_MM = 80
LIDAR_MAX_VALID_MM = 8000

lidar_lock = threading.Lock()
lidar_distances = {}
lidar_error = ""
lidar_connected = False
lidar_running = True
lidar_last_scan = 0.0

# Motor modes
MOTOR_MODE_MANUAL = "manual"
MOTOR_MODE_AUTONOMOUS = "autonomous"
motor_mode = MOTOR_MODE_MANUAL
autonomous_running = False
autonomous_reason = "Manual mode"

# Autonomous navigation parameters
AUTO_FORWARD_MM = 900
AUTO_TURN_MM = 600
AUTO_REVERSE_SECONDS = 0.35
AUTO_TURN_SECONDS = 0.55
AUTO_LOOP_SECONDS = 0.08

def _lidar_send(ser, payload):
    ser.write(bytes(payload))
    ser.flush()

def generate_simulated_lidar():
    """Generates synthetic RPLiDAR scan points when physical hardware is offline."""
    pts = {}
    now = time.time()
    for angle in range(0, 360, 2):
        rad = math.radians(angle)
        # Base arena distance
        dist = 2200 + 400 * math.sin(rad * 3 + now)
        # Simulate obstacle ahead around 0 degrees if autonomous turn testing
        if 340 <= angle or angle <= 20:
            dist = 1400 + 150 * math.sin(now * 2)
        dist += random.uniform(-30, 30)
        pts[float(angle)] = max(100, dist)
    return pts

def _lidar_reader():
    global lidar_error, lidar_connected, lidar_last_scan, lidar_distances
    while lidar_running:
        if not os.path.exists(LIDAR_PORT):
            # Fallback to simulation mode if serial port not present
            with lidar_lock:
                lidar_distances = generate_simulated_lidar()
                lidar_connected = True
                lidar_error = "Simulation Mode (No /dev/ttyUSB0)"
                lidar_last_scan = time.monotonic()
            time.sleep(0.2)
            continue

        try:
            ser = serial.Serial(
                LIDAR_PORT,
                LIDAR_BAUD,
                timeout=0.15,
                write_timeout=0.5,
                dsrdtr=False,
                rtscts=False,
            )
            try:
                ser.dtr = False
                ser.rts = False
            except Exception:
                pass

            time.sleep(0.2)
            try:
                _lidar_send(ser, [0xA5, 0x25])
                time.sleep(0.05)
            except Exception:
                pass
            _lidar_send(ser, [0xA5, 0x20])
            time.sleep(0.15)

            with lidar_lock:
                lidar_connected = True
                lidar_error = ""

            buf = bytearray()
            while lidar_running and os.path.exists(LIDAR_PORT):
                chunk = ser.read(512)
                if chunk:
                    buf.extend(chunk)

                while len(buf) >= 5:
                    if (buf[0] & 0x01) == 0 or (buf[0] & 0x02) == 0:
                        del buf[0]
                        continue

                    b0, b1, b2, b3, b4 = buf[:5]
                    quality = b0 >> 2
                    angle_deg = (((b1 >> 1) | (b2 << 7)) / 64.0) % 360.0
                    distance_mm = ((b3 | (b4 << 8)) / 4.0)

                    del buf[:5]

                    if quality < 5:
                        continue
                    if distance_mm < LIDAR_MIN_VALID_MM or distance_mm > LIDAR_MAX_VALID_MM:
                        continue

                    with lidar_lock:
                        lidar_distances[angle_deg] = distance_mm
                        if len(lidar_distances) > 2500:
                            keys = list(lidar_distances.keys())
                            for k in keys[:500]:
                                lidar_distances.pop(k, None)
                        lidar_last_scan = time.monotonic()
                        lidar_connected = True

        except Exception as e:
            with lidar_lock:
                lidar_connected = False
                lidar_error = f"LiDAR error: {e}"
            time.sleep(1.0)
        finally:
            try:
                ser.close()
            except Exception:
                pass

def lidar_sector(start_deg, end_deg):
    with lidar_lock:
        pts = list(lidar_distances.items())
    vals = []
    for angle, dist in pts:
        a = angle % 360
        if start_deg <= end_deg:
            inside = start_deg <= a <= end_deg
        else:
            inside = a >= start_deg or a <= end_deg
        if inside:
            vals.append(dist)
    if not vals:
        return None
    return min(vals)

def lidar_summary():
    return {
        "front": lidar_sector(330, 30),
        "left": lidar_sector(30, 150),
        "right": lidar_sector(210, 330),
    }

def equal_wheel_pwm(speed):
    return max(0, min(255, round(int(speed) * 255 / 100)))

def _auto_drive(left_sign, right_sign):
    if motor_mode != MOTOR_MODE_AUTONOMOUS or plant_action_in_progress:
        return
    pwm = equal_wheel_pwm(motor_state["speed"])
    left_pwm = left_sign * pwm
    right_pwm = right_sign * pwm
    with bridge_lock:
        Bridge.call("drive", left_pwm, right_pwm)
    motor_state["left"] = left_pwm
    motor_state["right"] = right_pwm
    motor_state["direction"] = (
        "forward" if left_sign > 0 and right_sign > 0 else
        "backward" if left_sign < 0 and right_sign < 0 else
        "right" if left_sign > 0 and right_sign < 0 else "left"
    )

def autonomous_loop():
    global autonomous_reason, autonomous_running
    while True:
        time.sleep(AUTO_LOOP_SECONDS)
        if motor_mode != MOTOR_MODE_AUTONOMOUS:
            autonomous_running = False
            continue
        autonomous_running = True

        if plant_action_in_progress:
            autonomous_reason = "Target Weed Spray Action Active"
            continue

        with lidar_lock:
            connected = lidar_connected
            last = lidar_last_scan

        if not connected or (time.monotonic() - last) > 1.5:
            autonomous_reason = "Waiting for LiDAR scan"
            with bridge_lock:
                Bridge.call("stop")
            motor_state["left"] = motor_state["right"] = 0
            motor_state["direction"] = "stop"
            continue

        d = lidar_summary()
        front = d["front"] if d["front"] is not None else 99999
        left = d["left"] if d["left"] is not None else 99999
        right = d["right"] if d["right"] is not None else 99999

        if front > AUTO_FORWARD_MM:
            autonomous_reason = "Crop Row Clear → Driving Forward"
            _auto_drive(1, 1)
        elif left > AUTO_TURN_MM or right > AUTO_TURN_MM:
            if left >= right:
                autonomous_reason = "Obstacle Ahead → Steering Left"
                _auto_drive(-1, 1)
            else:
                autonomous_reason = "Obstacle Ahead → Steering Right"
                _auto_drive(1, -1)
            time.sleep(AUTO_TURN_SECONDS)
        else:
            autonomous_reason = "Path Blocked → Reversing"
            _auto_drive(-1, -1)
            time.sleep(AUTO_REVERSE_SECONDS)
            d2 = lidar_summary()
            l2 = d2["left"] if d2["left"] is not None else 99999
            r2 = d2["right"] if d2["right"] is not None else 99999
            if motor_mode == MOTOR_MODE_AUTONOMOUS and not plant_action_in_progress:
                _auto_drive(-1, 1) if l2 >= r2 else _auto_drive(1, -1)
                time.sleep(AUTO_TURN_SECONDS)

threading.Thread(target=_lidar_reader, daemon=True).start()
threading.Thread(target=autonomous_loop, daemon=True).start()

# ==========================================
# DUAL WEBCAM & AI WEED DETECTION
# ==========================================
CAMERAS = [
    ("/dev/video2", "Camera 1 (Front View)"),
    ("/dev/video4", "Camera 2 (Spray Arm View)"),
]
CAMERA_SWITCH_SECONDS = 0.1
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
CAMERA_FPS = 20

camera_lock = threading.Lock()
latest_frame = None
camera_active_index = 0
camera_error = ""
camera_running = True

# ONNX Model parameters
MODEL_PATH = BASE_DIR / "model" / "best.onnx"
PLANT_CLASS_ID = 0          # Class 0: WEED / PLANT
PLANT_CLASS_NAME = "WEED TARGET"
PLANT_CONFIDENCE = 0.40
DETECTION_INTERVAL = 0.15

plant_lock = False
plant_detected = False
plant_confidence = 0.0
plant_boxes = []
plant_camera_index = None
plant_error = ""
last_detection_time = 0.0
ml_session = None
ml_input = None
ml_output = None
ml_lock = threading.Lock()

PLANT_TRIGGER_RELAY = {0: 1, 1: 2}
PLANT_RELAY_ON_SECONDS = 3.0
PLANT_RELAY_WAIT_SECONDS = 4.0
plant_action_lock = threading.Lock()
plant_action_in_progress = False
plant_detection_cooldown_until = 0.0

try:
    if MODEL_PATH.exists():
        ml_session = ort.InferenceSession(
            str(MODEL_PATH),
            providers=["CPUExecutionProvider"],
        )
        ml_input = ml_session.get_inputs()[0]
        ml_output = ml_session.get_outputs()[0]
        print(f"YOLOv8 OBB Plant model loaded successfully: {MODEL_PATH}")
    else:
        plant_error = f"Model file missing at {MODEL_PATH}"
        print(plant_error)
except Exception as e:
    ml_session = None
    plant_error = f"Model load error: {e}"
    print(plant_error)

def create_synthetic_frame(camera_idx):
    """Generates synthetic video stream with crops and occasional weed for visual demonstration."""
    img = Image.new("RGB", (640, 480), color=(18, 28, 22))
    draw = ImageDraw.Draw(img)
    
    # Soil texture details
    for _ in range(80):
        rx, ry = random.randint(0, 640), random.randint(0, 480)
        draw.ellipse((rx, ry, rx+3, ry+3), fill=(40, 32, 24))

    now = time.time()

    # Draw simulated crop rows (Green)
    for row_y in [140, 320]:
        for cx in range(80, 600, 140):
            ox = int(15 * math.sin(now + cx))
            draw.ellipse((cx-35+ox, row_y-25, cx+35+ox, row_y+25), fill=(34, 197, 94))
            draw.text((cx-20+ox, row_y-8), "CROP", fill=(255, 255, 255))

    # Periodically generate simulated weed target (Red/Amber)
    detections = []
    if int(now // 4) % 2 == 1:
        wx, wy = 320, 220
        draw.ellipse((wx-45, wy-35, wx+45, wy+35), fill=(239, 68, 68))
        draw.rectangle((wx-50, wy-40, wx+50, wy+40), outline=(245, 158, 11), width=3)
        draw.text((wx-35, wy-30), "WEED 94%", fill=(255, 255, 255))
        detections.append({
            "class_id": 0,
            "class_name": "WEED",
            "confidence": 0.94,
            "box": [wx-50, wy-40, 100, 80],
        })

    cam_label = CAMERAS[camera_idx][1]
    draw.rectangle((10, 10, 360, 38), fill=(0, 0, 0, 180))
    draw.text((18, 16), f"AGROBOT CAMERA STACK — {cam_label}", fill=(56, 189, 248))

    out = BytesIO()
    img.save(out, format="JPEG", quality=80)
    return out.getvalue(), detections

def detect_plants(frame_bytes):
    """Run YOLOv8 ONNX inference or fallback detection."""
    global plant_error
    if ml_session is None:
        return frame_bytes, []

    try:
        image = Image.open(BytesIO(frame_bytes)).convert("RGB")
        w, h = image.size
        input_image = image.resize((640, 640))
        arr = np.asarray(input_image, dtype=np.float32) / 255.0
        arr = np.transpose(arr, (2, 0, 1))[None, ...]

        with ml_lock:
            output = ml_session.run([ml_output.name], {ml_input.name: arr})[0]

        pred = np.asarray(output)
        if pred.ndim == 3:
            pred = pred[0]
        if pred.ndim != 2:
            return frame_bytes, []

        if pred.shape[0] <= 10 and pred.shape[1] > pred.shape[0]:
            predictions = pred.T
        else:
            predictions = pred

        boxes, scores = [], []
        sx, sy = w / 640.0, h / 640.0

        for row in predictions:
            if row.shape[0] < 5:
                continue
            class_scores = row[4:]
            if class_scores.size == 0:
                continue
            cls = int(np.argmax(class_scores))
            conf = float(class_scores[cls])

            if conf < PLANT_CONFIDENCE:
                continue

            cx, cy, bw, bh = map(float, row[:4])
            x1 = int((cx - bw / 2) * sx)
            y1 = int((cy - bh / 2) * sy)
            x2 = int((cx + bw / 2) * sx)
            y2 = int((cy + bh / 2) * sy)

            x1 = max(0, min(w - 1, x1))
            y1 = max(0, min(h - 1, y1))
            x2 = max(x1 + 1, min(w, x2))
            y2 = max(y1 + 1, min(h, y2))

            boxes.append((x1, y1, x2, y2))
            scores.append(conf)

        if not boxes:
            return frame_bytes, []

        order = sorted(range(len(scores)), key=lambda i: scores[i], reverse=True)
        keep = []
        while order:
            i = order.pop(0)
            keep.append(i)
            ax1, ay1, ax2, ay2 = boxes[i]
            remaining = []
            for j in order:
                bx1, by1, bx2, by2 = boxes[j]
                ix1, iy1 = max(ax1, bx1), max(ay1, by1)
                ix2, iy2 = min(ax2, bx2), min(ay2, by2)
                iw, ih = max(0, ix2 - ix1), max(0, iy2 - iy1)
                inter = iw * ih
                aarea = (ax2-ax1)*(ay2-ay1)
                barea = (bx2-bx1)*(by2-by1)
                iou = inter / max(1.0, aarea + barea - inter)
                if iou < 0.45:
                    remaining.append(j)
            order = remaining

        draw = ImageDraw.Draw(image)
        detections = []
        for i in keep:
            x1, y1, x2, y2 = boxes[i]
            conf = scores[i]
            draw.rectangle((x1, y1, x2, y2), outline=(239, 68, 68), width=3)
            label = f"WEED TARGET {conf:.0%}"
            draw.rectangle((x1, max(0, y1-24), min(w, x1+200), y1), fill=(239, 68, 68))
            draw.text((x1+4, max(0, y1-21)), label, fill=(255, 255, 255))
            detections.append({
                "class_id": PLANT_CLASS_ID,
                "class_name": PLANT_CLASS_NAME,
                "confidence": round(float(conf), 4),
                "box": [x1, y1, x2-x1, y2-y1],
            })

        out = BytesIO()
        image.save(out, format="JPEG", quality=82)
        return out.getvalue(), detections

    except Exception as e:
        plant_error = f"Plant detection error: {e}"
        return frame_bytes, []

def plant_relay_action(camera_index):
    """Executes the precision target spray cycle specified in system report."""
    global plant_action_in_progress
    global plant_lock, plant_detected, plant_confidence, plant_boxes, plant_camera_index
    global plant_detection_cooldown_until

    relay = PLANT_TRIGGER_RELAY.get(camera_index, 1)
    with plant_action_lock:
        plant_action_in_progress = True
        pin = 7 if relay == 1 else 8

        previous_left = motor_state["left"]
        previous_right = motor_state["right"]
        previous_speed = motor_state["speed"]

        try:
            add_log(f"WEED DETECTED on {CAMERAS[camera_index][1]}! Stopping drive chassis.")
            if previous_left or previous_right:
                with bridge_lock:
                    Bridge.call("stop")
                motor_state["left"] = 0
                motor_state["right"] = 0
                motor_state["direction"] = "stop"

            # Step 1: Position Nozzle Servo to Target Angle (e.g. Servo 2 to 45 degrees)
            add_log("Positioning spray arm servo nozzle to target coordinates (45°)...")
            set_servo(2, 45)
            time.sleep(0.5)

            # Step 2: Actuate Diaphragm Water Pump via Relay
            add_log(f"Actuating Diaphragm Spray Pump (Relay {relay} / Pin {pin}) for {PLANT_RELAY_ON_SECONDS}s.")
            with bridge_lock:
                Bridge.call("setRelay", pin, 1)
            relay_state[relay - 1] = True

            time.sleep(PLANT_RELAY_ON_SECONDS)

            # Step 3: Deactivate Pump and return Nozzle Servo to Home Position (90°)
            with bridge_lock:
                Bridge.call("setRelay", pin, 0)
            relay_state[relay - 1] = False
            add_log("Diaphragm Pump OFF. Returning spray nozzle to Home position (90°).")
            set_servo(2, 90)

            plant_detection_cooldown_until = time.monotonic() + PLANT_RELAY_WAIT_SECONDS

            # Step 4: Resume Navigation Path
            if (previous_left or previous_right) and not (motor_state["left"] or motor_state["right"]):
                pwm = equal_wheel_pwm(previous_speed)
                left_sign = 1 if previous_left > 0 else -1
                right_sign = 1 if previous_right > 0 else -1
                left_pwm = left_sign * pwm
                right_pwm = right_sign * pwm
                with bridge_lock:
                    Bridge.call("drive", left_pwm, right_pwm)
                motor_state["left"] = left_pwm
                motor_state["right"] = right_pwm
                add_log("Resuming autonomous/manual drive path.")

            with camera_lock:
                plant_lock = False
                plant_detected = False
                plant_confidence = 0.0
                plant_boxes = []
                plant_camera_index = None

        except Exception as e:
            add_log(f"Spray action error: {e}")
            try:
                with bridge_lock:
                    Bridge.call("setRelay", pin, 0)
                relay_state[relay - 1] = False
            except Exception:
                pass
            with camera_lock:
                plant_lock = False
                plant_detected = False
                plant_confidence = 0.0
                plant_boxes = []
                plant_camera_index = None
        finally:
            plant_action_in_progress = False

def set_plant_lock(index, detections):
    global plant_lock, plant_detected, plant_confidence
    global plant_boxes, plant_camera_index, plant_error
    if detections and not plant_action_in_progress and time.monotonic() >= plant_detection_cooldown_until:
        plant_lock = True
        plant_detected = True
        plant_camera_index = index
        plant_confidence = max(d["confidence"] for d in detections)
        plant_boxes = detections
        plant_error = ""

        threading.Thread(
            target=plant_relay_action,
            args=(index,),
            daemon=True,
        ).start()

def clear_plant_lock():
    global plant_lock, plant_detected, plant_confidence, plant_boxes, plant_camera_index, plant_error
    with camera_lock:
        plant_lock = False
        plant_detected = False
        plant_confidence = 0.0
        plant_boxes = []
        plant_camera_index = None
        plant_error = ""

def capture_camera(index, duration):
    global latest_frame, camera_active_index, camera_error, last_detection_time

    device, name = CAMERAS[index]
    with camera_lock:
        camera_active_index = index
        camera_error = ""

    if not os.path.exists(device):
        # Fallback to simulated camera feed
        start = time.monotonic()
        while camera_running and (time.monotonic() - start < duration):
            frame, detections = create_synthetic_frame(index)
            now = time.monotonic()
            if now - last_detection_time >= DETECTION_INTERVAL:
                last_detection_time = now
                if detections:
                    set_plant_lock(index, detections)

            with camera_lock:
                latest_frame = frame
            time.sleep(0.1)
        return

    cmd = [
        "gst-launch-1.0", "-q",
        "v4l2src", f"device={device}",
        "!",
        f"image/jpeg,width={CAMERA_WIDTH},height={CAMERA_HEIGHT},framerate={CAMERA_FPS}/1",
        "!",
        "fdsink", "fd=1", "sync=false",
    ]

    proc = None
    buffer = b""
    start_time = time.monotonic()

    try:
        proc = subprocess.Popen(
            cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
            bufsize=0
        )
        while camera_running:
            with camera_lock:
                locked_here = plant_lock and plant_camera_index == index

            if (not locked_here) and time.monotonic() - start_time >= duration:
                break

            chunk = proc.stdout.read(65536)
            if not chunk:
                if proc.poll() is not None:
                    break
                continue

            buffer += chunk
            if len(buffer) > 4000000:
                buffer = buffer[-1000000:]

            while True:
                start = buffer.find(b"\xff\xd8")
                if start < 0:
                    buffer = buffer[-2:]
                    break
                end = buffer.find(b"\xff\xd9", start + 2)
                if end < 0:
                    buffer = buffer[start:]
                    break

                frame = buffer[start:end + 2]
                buffer = buffer[end + 2:]

                now = time.monotonic()
                if now - last_detection_time >= DETECTION_INTERVAL:
                    last_detection_time = now
                    annotated, detections = detect_plants(frame)
                    if detections:
                        set_plant_lock(index, detections)
                        frame = annotated

                with camera_lock:
                    latest_frame = frame
    except Exception as e:
        with camera_lock:
            camera_error = str(e)
    finally:
        if proc is not None:
            try:
                proc.terminate()
                proc.wait(timeout=1)
            except Exception:
                pass

def camera_loop():
    index = 0
    while camera_running:
        with camera_lock:
            locked = plant_lock
            locked_index = plant_camera_index

        if locked and locked_index is not None:
            capture_camera(locked_index, 3600)
            time.sleep(0.05)
            continue

        capture_camera(index, CAMERA_SWITCH_SECONDS)
        index = 1 - index
        time.sleep(0.4)

threading.Thread(target=camera_loop, daemon=True).start()

# ==========================================
# HELPER & RPC WRAPPERS
# ==========================================
def clamp_angle(value):
    return max(0, min(180, int(value)))

def set_servo(servo, angle):
    if servo < 1 or servo > 4:
        raise ValueError("Servo index must be between 1 and 4")
    angle = clamp_angle(angle)
    pin = 6 if servo == 1 else 9 if servo == 2 else 10 if servo == 3 else 11
    with bridge_lock:
        Bridge.call("setServo", pin, angle)
    servo_angles[servo - 1] = angle
    return angle

# ==========================================
# REST API ENDPOINTS
# ==========================================
@app.route("/")
def index():
    return send_from_directory(WEB_DIR, "index.html")

@app.get("/api/state")
def state():
    with lidar_lock:
        l_conn = lidar_connected
        l_err = lidar_error
    with log_lock:
        recent_logs = list(system_logs[-15:])
    return jsonify({
        "servos": servo_angles,
        "lasers": laser_state,
        "relays": relay_state,
        "motors": {**motor_state, "mode": motor_mode, "autonomous_reason": autonomous_reason},
        "lidar": {**lidar_summary(), "connected": l_conn, "error": l_err},
        "logs": recent_logs,
        "project": {
            "title": "Autonomous Agrobot",
            "team": "Team Krishna",
            "id": "APC-2026-GJ-59026",
            "track": "Industrial AI Sustainability"
        }
    })

@app.post("/api/servo/<int:servo>")
def api_set_servo(servo):
    data = request.get_json(silent=True) or {}
    if "angle" not in data:
        return jsonify({"ok": False, "error": "Missing angle parameter"}), 400
    try:
        angle = set_servo(servo, data["angle"])
        add_log(f"Servo {servo} rotated to {angle}°")
        return jsonify({"ok": True, "servo": servo, "angle": angle})
    except Exception as e:
        return jsonify({"ok": False, "error": str(e)}), 500

@app.post("/api/center")
def center_all():
    with bridge_lock:
        for pin in (6, 9, 10, 11):
            Bridge.call("setServo", pin, 90)
    for i in range(4):
        servo_angles[i] = 90
    add_log("All 4 servos centered to 90° Home position.")
    return jsonify({"ok": True, "servos": servo_angles})

@app.post("/api/zero")
def zero_all():
    with bridge_lock:
        for pin in (6, 9, 10, 11):
            Bridge.call("setServo", pin, 0)
    for i in range(4):
        servo_angles[i] = 0
    add_log("All 4 servos set to 0° Min position.")
    return jsonify({"ok": True, "servos": servo_angles})

@app.post("/api/all")
def set_all():
    data = request.get_json(silent=True) or {}
    if "angle" not in data:
        return jsonify({"ok": False, "error": "Missing angle parameter"}), 400
    try:
        angle = clamp_angle(data["angle"])
        with bridge_lock:
            for pin in (6, 9, 10, 11):
                Bridge.call("setServo", pin, angle)
        for i in range(4):
            servo_angles[i] = angle
        add_log(f"All servos set to {angle}°.")
        return jsonify({"ok": True, "servos": servo_angles})
    except Exception as e:
        return jsonify({"ok": False, "error": str(e)}), 500

@app.post("/api/laser/<int:laser>")
def api_laser(laser):
    if laser not in (1, 2):
        return jsonify({"ok": False, "error": "Laser must be 1 or 2"}), 400
    data = request.get_json(silent=True) or {}
    if "on" not in data:
        return jsonify({"ok": False, "error": "Missing on parameter"}), 400
    on = bool(data["on"])
    pin = 12 if laser == 1 else 13
    with bridge_lock:
        Bridge.call("setLaser", pin, int(on))
    laser_state[laser - 1] = on
    add_log(f"Targeting Laser {laser} turned {'ON' if on else 'OFF'}.")
    return jsonify({"ok": True, "lasers": laser_state})

@app.post("/api/relay/<int:relay>")
def api_relay(relay):
    if relay not in (1, 2):
        return jsonify({"ok": False, "error": "Relay must be 1 or 2"}), 400
    data = request.get_json(silent=True) or {}
    if "on" not in data:
        return jsonify({"ok": False, "error": "Missing on parameter"}), 400
    on = bool(data["on"])
    pin = 7 if relay == 1 else 8
    with bridge_lock:
        Bridge.call("setRelay", pin, int(on))
    relay_state[relay - 1] = on
    add_log(f"Spray Pump Relay {relay} turned {'ON' if on else 'OFF'}.")
    return jsonify({"ok": True, "relays": relay_state})

@app.post("/api/spray/trigger")
def api_spray_trigger():
    """Manually triggers a test spray sequence."""
    threading.Thread(target=plant_relay_action, args=(0,), daemon=True).start()
    return jsonify({"ok": True, "message": "Manual spray sequence initiated."})

@app.get("/api/lidar")
def api_lidar():
    with lidar_lock:
        connected = lidar_connected
        err = lidar_error
        age = time.monotonic() - lidar_last_scan if lidar_last_scan else None
    d = lidar_summary()
    return jsonify({
        "ok": True,
        "port": LIDAR_PORT,
        "connected": connected,
        "error": err,
        "age": age,
        "front_mm": d["front"],
        "left_mm": d["left"],
        "right_mm": d["right"],
    })

@app.get("/api/lidar/scan")
def api_lidar_scan():
    with lidar_lock:
        pts = [{"angle": round(a, 1), "distance_mm": round(v, 1)}
               for a, v in lidar_distances.items()]
    return jsonify({"points": pts})

@app.post("/api/motor/mode")
def api_motor_mode():
    global motor_mode, autonomous_reason
    data = request.get_json(silent=True) or {}
    mode = data.get("mode", MOTOR_MODE_MANUAL)
    if mode not in (MOTOR_MODE_MANUAL, MOTOR_MODE_AUTONOMOUS):
        return jsonify({"ok": False, "error": "mode must be manual or autonomous"}), 400
    motor_mode = mode
    if mode == MOTOR_MODE_MANUAL:
        autonomous_reason = "Manual mode active"
        with bridge_lock:
            Bridge.call("stop")
        motor_state["left"] = motor_state["right"] = 0
        motor_state["direction"] = "stop"
        add_log("Switched to MANUAL Drive Mode.")
    else:
        autonomous_reason = "Autonomous LiDAR Navigation active"
        add_log("Switched to AUTONOMOUS Navigation & Crop Row Following.")
    return jsonify({"ok": True, "mode": motor_mode, "reason": autonomous_reason})

@app.post("/api/motor")
def api_motor():
    global motor_mode, autonomous_reason
    motor_mode = MOTOR_MODE_MANUAL
    autonomous_reason = "Manual override command"
    data = request.get_json(silent=True) or {}
    try:
        speed = max(0, min(100, int(data.get("speed", motor_state["speed"]))))
        left_cmd = int(data.get("left", 0))
        right_cmd = int(data.get("right", 0))
    except (TypeError, ValueError):
        return jsonify({"ok": False, "error": "Invalid motor command"}), 400

    if "speed" in data:
        motor_state["speed"] = speed

    if left_cmd == 0 and right_cmd == 0:
        with bridge_lock:
            Bridge.call("stop")
        motor_state["left"] = 0
        motor_state["right"] = 0
        motor_state["direction"] = "stop"
    else:
        pwm = equal_wheel_pwm(motor_state["speed"])
        left_sign = 1 if left_cmd > 0 else -1
        right_sign = 1 if right_cmd > 0 else -1
        left_pwm = left_sign * pwm
        right_pwm = right_sign * pwm
        with bridge_lock:
            Bridge.call("drive", left_pwm, right_pwm)
        motor_state["left"] = left_pwm
        motor_state["right"] = right_pwm
        motor_state["direction"] = (
            "forward" if left_sign > 0 and right_sign > 0 else
            "backward" if left_sign < 0 and right_sign < 0 else
            "right" if left_sign > 0 and right_sign < 0 else "left"
        )

    return jsonify({"ok": True, "motors": motor_state})

@app.post("/api/motor/speed")
def api_motor_speed():
    global motor_mode, autonomous_reason
    motor_mode = MOTOR_MODE_MANUAL
    autonomous_reason = "Manual speed change"
    data = request.get_json(silent=True) or {}
    try:
        speed = max(0, min(100, int(data["speed"])))
    except (KeyError, TypeError, ValueError):
        return jsonify({"ok": False, "error": "Invalid speed"}), 400

    motor_state["speed"] = speed

    if motor_state["left"] or motor_state["right"]:
        left_sign = 1 if motor_state["left"] > 0 else -1
        right_sign = 1 if motor_state["right"] > 0 else -1
        pwm = equal_wheel_pwm(speed)
        left_pwm = left_sign * pwm
        right_pwm = right_sign * pwm
        with bridge_lock:
            Bridge.call("drive", left_pwm, right_pwm)
        motor_state["left"] = left_pwm
        motor_state["right"] = right_pwm

    return jsonify({"ok": True, "motors": motor_state})

@app.post("/api/motor/trim")
def api_motor_trim():
    data = request.get_json(silent=True) or {}
    try:
        left = max(-50, min(50, int(data.get("left", motor_state["left_trim"]))))
        right = max(-50, min(50, int(data.get("right", motor_state["right_trim"]))))
    except (TypeError, ValueError):
        return jsonify({"ok": False, "error": "Invalid trim"}), 400

    motor_state["left_trim"] = left
    motor_state["right_trim"] = right
    with bridge_lock:
        Bridge.call("setWheelTrim", left, right)

    if motor_state["left"] or motor_state["right"]:
        left_sign = 1 if motor_state["left"] > 0 else -1
        right_sign = 1 if motor_state["right"] > 0 else -1
        pwm = equal_wheel_pwm(motor_state["speed"])
        with bridge_lock:
            Bridge.call("drive", left_sign * pwm, right_sign * pwm)
        motor_state["left"] = left_sign * pwm
        motor_state["right"] = right_sign * pwm

    add_log(f"Wheel trim updated: L {left} / R {right}")
    return jsonify({"ok": True, "motors": motor_state})

@app.get("/api/camera/frame")
def camera_frame():
    with camera_lock:
        frame = latest_frame
    if frame is None:
        return ("Camera frame initializing...", 503, {"Content-Type": "text/plain"})
    return Response(
        frame,
        mimetype="image/jpeg",
        headers={"Cache-Control": "no-store, no-cache, must-revalidate, max-age=0"}
    )

@app.get("/api/camera/status")
def camera_status():
    with camera_lock:
        idx = camera_active_index
        err = camera_error
        ready = latest_frame is not None
        locked = plant_lock
        detected = plant_detected
        confidence = plant_confidence
        boxes = list(plant_boxes)
        locked_idx = plant_camera_index
    device, name = CAMERAS[idx]
    locked_name = CAMERAS[locked_idx][1] if locked_idx is not None else None
    return jsonify({
        "active_camera": name,
        "device": device,
        "switch_seconds": CAMERA_SWITCH_SECONDS,
        "error": err,
        "frame_ready": ready,
        "plant_detected": detected,
        "plant_lock": locked,
        "plant_confidence": confidence,
        "plant_detections": boxes,
        "locked_camera": locked_name,
        "model": str(MODEL_PATH.name),
        "model_error": plant_error,
    })

@app.post("/api/camera/done")
def camera_done():
    clear_plant_lock()
    add_log("Target lock cleared manually. Resuming camera switching.")
    return jsonify({
        "ok": True,
        "plant_detected": False,
        "plant_lock": False,
        "message": "Plant state cleared; normal camera switching resumes.",
    })

@app.post("/api/all-off")
def all_off():
    global motor_mode, autonomous_reason
    motor_mode = MOTOR_MODE_MANUAL
    autonomous_reason = "Emergency Stop All OFF"
    with bridge_lock:
        Bridge.call("stop")
        Bridge.call("setLaser", 12, 0)
        Bridge.call("setLaser", 13, 0)
        Bridge.call("setRelay", 7, 0)
        Bridge.call("setRelay", 8, 0)
    motor_state["left"] = motor_state["right"] = 0
    laser_state[0] = laser_state[1] = False
    relay_state[0] = relay_state[1] = False
    add_log("EMERGENCY ALL OFF ACTIVATED! Motors, relays, and lasers halted.")
    return jsonify({"ok": True, "motors": motor_state, "lasers": laser_state, "relays": relay_state})

def start_web_server():
    app.run(host="0.0.0.0", port=8080, debug=False, use_reloader=False, threaded=True)

def loop():
    time.sleep(0.1)

if __name__ == "__main__":
    threading.Thread(target=start_web_server, daemon=True).start()
    App.run(user_loop=loop)
