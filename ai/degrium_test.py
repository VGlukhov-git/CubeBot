import degirum as dg
import cv2
import numpy as np
import time
import sys
import gpiod
import threading
import socket

from flask import Flask, Response
from pathlib import Path

sys.path.append(str(Path(__file__).resolve().parent.parent))

# source venv/bin/activate

# -------------------- MODELS --------------------
DEPTH_MODEL_NAME = "scdepthv3--320x256_quant_hailort_multidevice_1"
DET_MODEL_NAME = "yolov8n_relu6_hand--640x640_quant_hailort_multidevice_1"

zoo = dg.connect("@local", zoo_url="./models/")
print("Available models:", zoo.list_models())

depth_model = zoo.load_model(DEPTH_MODEL_NAME)
det_model = zoo.load_model(DET_MODEL_NAME)

# -------------------- CAMERA --------------------
cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1600)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1200)
cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

app = Flask(__name__)

DET_W = 640
DET_H = 640

cx_g = DET_W / 2
cy_g = DET_H / 2
# -------------------- STEPPER THREAD --------------------
motor_stop_event = threading.Event()
delta = 15

HOST = "127.0.0.1"   # change to your server IP
PORT = 3001          # change to your server port

def send_cmd(sock, cmd: str):
    try:
        print(cmd)
        sock.sendall((cmd + "\n").encode("utf-8"))
    except OSError as e:
        print(f"Socket send error: {e}")

def stepper_worker():
    sock = None
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect((HOST, PORT))
        
        while not motor_stop_event.is_set():
            if cx_g > DET_W / 2 + delta:
                send_cmd(sock, f"RIGHT")
            if cx_g < DET_W / 2 - delta:
                send_cmd(sock, f"LEFT")
            if cy_g > DET_H / 2 + delta:
                send_cmd(sock, f"DOWN")
            if cy_g < DET_H / 2 - delta:
                send_cmd(sock, f"UP")
            da = 0.03
            time.sleep(da)

    except OSError as e:
        print(f"Socket connection error: {e}")

    finally:
        if sock:
            try:
                sock.close()
            except OSError:
                pass

# -------------------- IMAGE HELPERS --------------------
def preprocess_frame(frame):
    frame = cv2.rotate(frame, cv2.ROTATE_180)
    h, w = frame.shape[:2]
    frame = frame[:, : w // 2]
    return frame

def depth_postprocessor(model_output):
    raw = model_output[0]
    q_data = raw["data"]
    scale = raw["quantization"]["scale"][0]
    zero = raw["quantization"]["zero"][0]
    dequantized = (q_data.astype(np.float32) - zero) * scale
    depth_map = dequantized.reshape((256, 320))
    return depth_map

def draw_detections(frame, depth_norm, detections, depth_map):
    h, w = depth_norm.shape[:2]
    global cx_g, cy_g

    cx_g = DET_W / 2
    cy_g = DET_H / 2
    for det in detections:
        x1, y1, x2, y2 = det["box"]
        label = det["label"]
        score = det["score"]

        cx = int((x1 + x2) / 2)
        cy = int((y1 + y2) / 2)

        cx = np.clip(cx, 0, w - 1)
        cy = np.clip(cy, 0, h - 1)

        cx_g, cy_g = cx, cy
        depth_value = 0
        if ((cx < 600) and (cy < 600)):
            depth_value = depth_map[cx, cy]

        cv2.rectangle(frame, (x1, y1), (x2, y2), (0,255,0), 2)
        cv2.putText(frame, f"{label} {score:.2f}",
                    (x1, max(20, y1-5)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1)

        cv2.circle(frame, (cx, cy), 4, (255,255,255), -1)

        cv2.putText(frame, f"{depth_value:.2f}",
                    (cx - 20, cy),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    (255,255,255), 2, cv2.LINE_AA)

    return frame, (cx_g, cy_g)

# -------------------- DETECTION --------------------

def run_detection(frame):
    h, w = frame.shape[:2]
    resized = cv2.resize(frame, (DET_W, DET_H))
    results = det_model(resized).results

    detections = []
    for det in results:
        bbox = np.array(det["bbox"], dtype=float)
        x1, y1, x2, y2 = bbox

        x1 = int(x1 / DET_W * w)
        y1 = int(y1 / DET_W * h)
        x2 = int(x2 / DET_W * w)
        y2 = int(y2 / DET_W * h)

        detections.append({
            "box": (x1, y1, x2, y2),
            "label": det.get("label", "obj"),
            "score": det.get("score", 0)
        })

    return detections

# -------------------- STREAM --------------------
def generate_frames():
    prev_time = time.time()

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        frame = preprocess_frame(frame)

        detections = run_detection(frame)

        depth_raw = depth_model(frame).results
        depth = depth_postprocessor(depth_raw)

        frame_h, frame_w = frame.shape[:2]
        resized_depth = cv2.resize(depth, (frame_w, frame_h),
                                interpolation=cv2.INTER_NEAREST)

        depth_norm = cv2.normalize(resized_depth, None, 0, 255, cv2.NORM_MINMAX)
        depth_norm = depth_norm.astype(np.uint8)
        depth_color = cv2.applyColorMap(depth_norm, cv2.COLORMAP_JET)

        h, w = depth_norm.shape[:2]
        cx = w // 2
        cy = h // 2
        line_width = 2

        horizontal_strip = depth_norm[max(0, cy - line_width // 2):min(h, cy - line_width // 2 + line_width), :]
        vertical_strip = depth_norm[:, max(0, cx - line_width // 2):min(w, cx - line_width // 2 + line_width)]

        horizontal_line = horizontal_strip.mean(axis=0).astype(np.uint8)   # shape (w,)
        vertical_line = vertical_strip.mean(axis=1).astype(np.uint8)       # shape (h,)

        cross_lines = {
            "horizontal": horizontal_line,
            "vertical": vertical_line
        }

        depth_color_with_cross = depth_color.copy()
        cv2.line(depth_color_with_cross, (0, cy), (w - 1, cy), (255, 255, 255), 2)
        cv2.line(depth_color_with_cross, (cx, 0), (cx, h - 1), (255, 255, 255), 2)

        sample_step = 2   # draw text every 40 pixels

        for x in range(0, w, sample_step):
            d = float(depth_norm[cy, x])   # real depth value
            cv2.circle(depth_color_with_cross, (x, cy + int(d)), 1, (255, 255, 255), -1)

        for y in range(0, h, sample_step):
            d = float(depth_norm[y, cx])   # real depth value
            cv2.circle(depth_color_with_cross, (cx + int((d-6)/6*6), y), 1, (255, 255, 255), -1)

        center_depth = float(resized_depth[cy, cx])
        cv2.circle(depth_color_with_cross, (cx, cy), 4, (0, 0, 0), -1)
        cv2.putText(depth_color_with_cross,
                    f"C:{center_depth:.1f}",
                    (cx + 8, cy - 8),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (255, 255, 255),
                    2,
                    cv2.LINE_AA)

        frame_with_det, arr = draw_detections(
            depth_color_with_cross, depth_norm, detections, resized_depth
        )

        curr_time = time.time()
        fps = 1.0 / (curr_time - prev_time)
        prev_time = curr_time

        cv2.putText(frame_with_det, f"FPS: {fps:.1f}",
                    (20, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8, (255, 255, 255), 2)

        combined = np.hstack((frame, frame_with_det))

        ret, buffer = cv2.imencode(".jpg", combined,
                                [cv2.IMWRITE_JPEG_QUALITY, 95])

        yield (
            b"--frame\r\n"
            b"Content-Type: image/jpeg\r\n\r\n" + buffer.tobytes() + b"\r\n"
        )

# -------------------- ROUTES --------------------
@app.route("/stream")
def stream():
    return Response(generate_frames(),
                    mimetype="multipart/x-mixed-replace; boundary=frame")

@app.route("/")
def index():
    return '<html><body style="background:#000;"><h1>Hailo-8L Deep Stereo</h1><img src="/stream"></body></html>'


if __name__ == "__main__":
    stepper_thread = threading.Thread(target=stepper_worker, daemon=True)
    stepper_thread.start()
    try:
        app.run(host="0.0.0.0", port=5000, threaded=True)
    finally:
        motor_stop_event.set()