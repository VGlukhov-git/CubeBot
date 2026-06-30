import cv2
import numpy as np
import atexit
import json
import threading
import queue
import time
from flask import Flask, Response

from hailo_platform import (
    HEF, VDevice, ConfigureParams, HailoStreamInterface,
    InputVStreamParams, OutputVStreamParams, InferVStreams, FormatType,
)
# source venv/bin/activate

# ==========================
# CONFIG & MODELS
# ==========================
DEPTH_HEF_PATH = "/home/vladimir/projects/robot_arm/camera/models/scdepthv3--320x256_quant_hailort_multidevice_1.hef"
DET_HEF_PATH   = "/home/vladimir/projects/robot_arm/camera/models/yolov11n.hef"
DET_HEF_PATH   = "/home/vladimir/projects/robot_arm/camera/models/hand/yolov8n_relu6_hand--640x640_quant_hailort_multidevice_1.hef"

PORT = 5000
JPEG_QUALITY = 95
CONF_THRESH = 0.25

# Stereo params
FOCAL_LENGTH_PX = 1400.0   
BASELINE_M = 0.06          
MAX_DISPARITY = 300        
CAMERA_PARAM = "stereocamera_test_calib_param.json"

app = Flask(__name__)

# Global variables for thread communication
frame_queue = queue.Queue(maxsize=2)
latest_processed_frame = None

# ============================================================
# INITIALIZE HAILO
# ============================================================
device = VDevice()

def depth_postprocessor(model_output,):
    raw = model_output[0]  # Получаем первый элемент списка
    q_data = raw["data"]
    scale = raw["quantization"]["scale"][0]
    zero = raw["quantization"]["zero"][0]
    dequantized = (q_data.astype(np.float32) - zero) * scale
    depth_map = dequantized.reshape((256, 320))  # высота, ширина
    return depth_map

def setup_model(hef_path):
    hef = HEF(hef_path)
    config = ConfigureParams.create_from_hef(hef, interface=HailoStreamInterface.PCIe)
    network_group = device.configure(hef, config)[0]
    input_params = InputVStreamParams.make_from_network_group(network_group, quantized=True, format_type=FormatType.UINT8)
    output_params = OutputVStreamParams.make_from_network_group(network_group, quantized=False, format_type=FormatType.FLOAT32)
    pipeline = InferVStreams(network_group, input_params, output_params)
    return hef, network_group, pipeline

print("Initializing Hailo Models...")
depth_hef, depth_ng, depth_pipeline = setup_model(DEPTH_HEF_PATH)
det_hef, det_ng, det_pipeline = setup_model(DET_HEF_PATH)

depth_pipeline.__enter__()
det_pipeline.__enter__()

# --- EXTRACT INPUT SHAPES ---
depth_input_info = depth_hef.get_input_vstream_infos()[0]
depth_output_info = depth_hef.get_output_vstream_infos()[0]
DEPTH_H, DEPTH_W, _ = depth_input_info.shape  # Restored these variables

det_input_info = det_hef.get_input_vstream_infos()[0]
DET_H, DET_W, _ = det_input_info.shape        # Restored these variables

depth_params = depth_ng.create_params()
det_params = det_ng.create_params()

# Load Stereo Maps
with open(CAMERA_PARAM) as fp:
    cp = json.load(fp)
Kl, Dl, Kr, Dr, R, T = np.array(cp["Kl"]), np.array(cp["Dl"]), np.array(cp["Kr"]), np.array(cp["Dr"]), np.array(cp["R"]), np.array(cp["T"])
imSize = np.array(cp["imSize"]) // 2
Kl *= 0.5; Kr *= 0.5
R1, R2, P1, P2, Q, _, _ = cv2.stereoRectify(Kl, Dl, Kr, Dr, imSize, R, T)
xmap1, ymap1 = cv2.initUndistortRectifyMap(Kl, Dl, R1, P1, imSize, cv2.CV_16SC2)
xmap2, ymap2 = cv2.initUndistortRectifyMap(Kr, Dr, R2, P2, imSize, cv2.CV_16SC2)

COCO_LABELS = [
    "person","bicycle","car","motorcycle","airplane","bus","train","truck","boat",
    "traffic light","fire hydrant","stop sign","parking meter","bench","bird","cat",
    "dog","horse","sheep","cow","elephant","bear","zebra","giraffe","backpack","umbrella",
    "handbag","tie","suitcase","frisbee","skis","snowboard","sports ball","kite",
    "baseball bat","baseball glove","skateboard","surfboard","tennis racket","bottle",
    "wine glass","cup","fork","knife","spoon","bowl","banana","apple","sandwich",
    "orange","broccoli","carrot","hot dog","pizza","donut","cake","chair","couch",
    "potted plant","bed","dining table","toilet","tv","laptop","mouse","remote",
    "keyboard","cell phone","microwave","oven","toaster","sink","refrigerator","book",
    "clock","vase","scissors","teddy bear","hair drier","toothbrush"
]

# ============================================================
# HELPERS
# ============================================================
def parse_dets(res, ow, oh):
    key = list(res.keys())[0]
    class_arrays = res[key][0]
    parsed = []
    for cls_id, arr in enumerate(class_arrays):
        for det in arr:
            y1, x1, y2, x2, score = det
            if score < CONF_THRESH: continue
            ix1, ix2 = int(x1 * ow), int(x2 * ow)
            iy1, iy2 = int(y1 * oh), int(y2 * oh)
            parsed.append({
                "cls": cls_id, "score": score,
                "box": (ix1, iy1, ix2, iy2),
                "center": ((ix1 + ix2) // 2, (iy1 + iy2) // 2)
            })
    return parsed

# ============================================================
# THREAD 1: CAMERA CAPTURE
# ============================================================
def video_capture_thread():
    cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1600)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1200)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    while True:
        ret, frame = cap.read()
        if not ret: continue
        if frame_queue.full():
            try: frame_queue.get_nowait()
            except queue.Empty: pass
        frame_queue.put(frame)

# ============================================================
# THREAD 2: INFERENCE & POST-PROCESS
# ============================================================
def inference_worker():
    global latest_processed_frame
    fps_avg = 0
    
    while True:
        frame = frame_queue.get()
        t1 = cv2.getTickCount()

        # 1. Pre-process
        frame = cv2.rotate(frame, cv2.ROTATE_180)
        h, w = frame.shape[:2]
        left_f = cv2.remap(frame[:, :w // 2], xmap1, ymap1, cv2.INTER_LINEAR)
        right_f = cv2.remap(frame[:, w // 2:], xmap2, ymap2, cv2.INTER_LINEAR)
        oh, ow = left_f.shape[:2]

        # 2. Depth Inference
        d_in = cv2.resize(cv2.cvtColor(left_f, cv2.COLOR_BGR2RGB), (DEPTH_W, DEPTH_H))
        with depth_ng.activate(depth_params):
            d_out = depth_pipeline.infer({depth_input_info.name: np.expand_dims(d_in, 0)})
        raw_depth = d_out[depth_output_info.name][0]
        depth = raw_depth #depth_postprocessor(raw_depth)
        frame_h, frame_w = left_f.shape[:2]
        resized_depth = cv2.resize(depth, (frame_w, frame_h), interpolation=cv2.INTER_NEAREST)
        
        depth_norm = cv2.normalize(resized_depth, None, 0, 255, cv2.NORM_MINMAX)
        depth_norm = depth_norm.astype(np.uint8)
        depth_cm = cv2.applyColorMap(depth_norm, cv2.COLORMAP_JET)
        
        det_l_in = np.expand_dims(cv2.resize(left_f, (DET_W, DET_H)), 0)
        det_r_in = np.expand_dims(cv2.resize(right_f, (DET_W, DET_H)), 0)
        
        with det_ng.activate(det_params):
            res_l = det_pipeline.infer({det_input_info.name: det_l_in})
            res_r = det_pipeline.infer({det_input_info.name: det_r_in}) #TESTING

        l_dets = parse_dets(res_l, ow, oh)
        r_dets = parse_dets(res_r, ow, oh)

        for ld in l_dets:
            best_rd = None
            min_dx = 1e9
            for rd in r_dets:
                if rd["cls"] == ld["cls"]:
                    dx = ld["center"][0] - rd["center"][0]
                    # Check horizontal disparity and vertical alignment
                    if 1 < dx < MAX_DISPARITY and abs(ld["center"][1] - rd["center"][1]) < 20:
                        if dx < min_dx:
                            min_dx, best_rd = dx, rd
            
            x1, y1, x2, y2 = ld["box"]
            cv2.rectangle(depth_cm, (x1, y1), (x2, y2), (0, 255, 0), 2)
            
            lbl = COCO_LABELS[ld["cls"]] if ld["cls"] < len(COCO_LABELS) else "Object"
            if best_rd:
                dist = (FOCAL_LENGTH_PX * BASELINE_M) / float(min_dx)
                txt = f"{lbl}: {dist:.2f}m"
            else:
                txt = f"{lbl}"
            cv2.putText(depth_cm, txt, (x1, y1 - 10), 1, 1.2, (255, 255, 255), 2)

        # Timing and Global Update
        dt = (cv2.getTickCount() - t1) / cv2.getTickFrequency()
        fps_avg = (1.0/dt) if fps_avg == 0 else (fps_avg * 0.9 + (1.0/dt) * 0.1)
        cv2.putText(depth_cm, f"FPS: {fps_avg:.1f}", (10, 35), 1, 1.5, (0, 255, 255), 2)
        combined = np.hstack((left_f, depth_cm))
        _, buf = cv2.imencode(".jpg", combined, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
        latest_processed_frame = buf.tobytes()

# ============================================================
# FLASK & RUN
# ============================================================
@app.route("/stream")
def video():
    def stream():
        while True:
            if latest_processed_frame:
                time.sleep(0.003)
                yield (b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + latest_processed_frame + b"\r\n")
    return Response(stream(), mimetype="multipart/x-mixed-replace; boundary=frame")

@app.route("/")
def index():
    return '<html><body style="background:#000;"><h1>Hailo-8L Deep Stereo</h1><img src="/stream"></body></html>'

if __name__ == "__main__":
    threading.Thread(target=video_capture_thread, daemon=True).start()
    threading.Thread(target=inference_worker, daemon=True).start()
    app.run(host="0.0.0.0", port=PORT, threaded=True, use_reloader=False)