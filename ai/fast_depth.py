import cv2
import numpy as np
import atexit
import threading
import time

from flask import Flask, Response
from picamera2 import Picamera2
from libcamera import controls

from hailo_platform import (
    HEF, VDevice, ConfigureParams, HailoStreamInterface,
    InputVStreamParams, OutputVStreamParams, InferVStreams, FormatType,
)

# ==========================
# CONFIG & MODELS
# ==========================
DEPTH_HEF_PATH = "/home/vladimir/projects/CubeBot/ai/models/scdepthv3--320x256_quant_hailort_multidevice_1.hef"
DET_HEF_PATH = "/home/vladimir/projects/CubeBot/ai/models/yolov11n.hef"

PORT = 5000
JPEG_QUALITY = 90
CONF_THRESH = 0.25
NMS_IOU = 0.25

CAMERA_WIDTH = 1536
CAMERA_HEIGHT = 864

app = Flask(__name__)

latest_camera_frame = None
latest_depth_frame = None
frame_lock = threading.Lock()

# ==========================
# CAMERA
# ==========================
camera = Picamera2()

camera.configure(
    camera.create_video_configuration(
        main={
            "size": (CAMERA_WIDTH, CAMERA_HEIGHT),
            "format": "RGB888",
        }
    )
)

COCO_LABELS = [
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck",
    "boat", "traffic light", "fire hydrant", "stop sign", "parking meter", "bench",
    "bird", "cat", "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra",
    "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee",
    "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove",
    "skateboard", "surfboard", "tennis racket", "bottle", "wine glass", "cup",
    "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange",
    "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch",
    "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse",
    "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink",
    "refrigerator", "book", "clock", "vase", "scissors", "teddy bear",
    "hair drier", "toothbrush"
]

print("Available camera modes:")
for mode in camera.sensor_modes:
    print(mode)

camera.start()

camera.set_controls({
    "AfMode": controls.AfModeEnum.Continuous,
})

time.sleep(1)

# ==========================
# HAILO INIT
# ==========================
device = VDevice()


def setup_model(hef_path):
    hef = HEF(hef_path)

    config = ConfigureParams.create_from_hef(
        hef,
        interface=HailoStreamInterface.PCIe,
    )

    network_group = device.configure(hef, config)[0]

    input_params = InputVStreamParams.make_from_network_group(
        network_group,
        quantized=True,
        format_type=FormatType.UINT8,
    )

    output_params = OutputVStreamParams.make_from_network_group(
        network_group,
        quantized=False,
        format_type=FormatType.FLOAT32,
    )

    pipeline = InferVStreams(
        network_group,
        input_params,
        output_params,
    )

    return hef, network_group, pipeline


print("Initializing Hailo models...")

depth_hef, depth_ng, depth_pipeline = setup_model(DEPTH_HEF_PATH)
det_hef, det_ng, det_pipeline = setup_model(DET_HEF_PATH)

depth_pipeline.__enter__()
det_pipeline.__enter__()

depth_input_info = depth_hef.get_input_vstream_infos()[0]
depth_output_info = depth_hef.get_output_vstream_infos()[0]
DEPTH_H, DEPTH_W, _ = depth_input_info.shape

det_input_info = det_hef.get_input_vstream_infos()[0]
DET_H, DET_W, _ = det_input_info.shape

print("Depth input:", DEPTH_W, DEPTH_H)
print("Detection input:", DET_W, DET_H)

depth_params = depth_ng.create_params()
det_params = det_ng.create_params()

det_out_infos = det_hef.get_output_vstream_infos()

for info in det_out_infos:
    print(f"{info.name} {info.shape}")

CONF_OUT_NAME = None
BBOX_OUT_NAME = None

for info in det_out_infos:
    if len(info.shape) == 3 and info.shape[-1] == 64:
        BBOX_OUT_NAME = info.name
    elif len(info.shape) == 3:
        CONF_OUT_NAME = info.name


def cleanup():
    try:
        camera.stop()
    except Exception:
        pass

    try:
        depth_pipeline.__exit__(None, None, None)
    except Exception:
        pass

    try:
        det_pipeline.__exit__(None, None, None)
    except Exception:
        pass

    try:
        device.release()
    except Exception:
        pass


atexit.register(cleanup)


# ==========================
# HELPERS
# ==========================
def nms_xyxy(boxes, scores, iou_thres):
    if boxes.size == 0:
        return np.array([], dtype=np.int64)

    x1, y1, x2, y2 = boxes.T
    areas = np.maximum(0.0, x2 - x1) * np.maximum(0.0, y2 - y1)

    order = scores.argsort()[::-1]
    keep = []

    while order.size > 0:
        i = order[0]
        keep.append(i)

        if order.size == 1:
            break

        xx1 = np.maximum(x1[i], x1[order[1:]])
        yy1 = np.maximum(y1[i], y1[order[1:]])
        xx2 = np.minimum(x2[i], x2[order[1:]])
        yy2 = np.minimum(y2[i], y2[order[1:]])

        w = np.maximum(0.0, xx2 - xx1)
        h = np.maximum(0.0, yy2 - yy1)

        inter = w * h
        iou = inter / (areas[i] + areas[order[1:]] - inter + 1e-9)

        inds = np.where(iou <= iou_thres)[0]
        order = order[inds + 1]

    return np.array(keep, dtype=np.int64)


def make_yolov8_grids(det_w, det_h, strides=(8, 16, 32)):
    centers = []
    s_all = []

    for s in strides:
        gw = det_w // s
        gh = det_h // s

        xs = (np.arange(gw, dtype=np.float32) + 0.5) * s
        ys = (np.arange(gh, dtype=np.float32) + 0.5) * s

        xv, yv = np.meshgrid(xs, ys)

        c = np.stack(
            [xv.reshape(-1), yv.reshape(-1)],
            axis=1,
        )

        centers.append(c)
        s_all.append(np.full((c.shape[0],), s, dtype=np.float32))

    return np.concatenate(centers, axis=0), np.concatenate(s_all, axis=0)


GRID_CENTERS, GRID_STRIDES = make_yolov8_grids(
    DET_W,
    DET_H,
    strides=(8, 16, 32),
)


def softmax_last(x):
    x = x - np.max(x, axis=-1, keepdims=True)
    e = np.exp(x)

    return e / (np.sum(e, axis=-1, keepdims=True) + 1e-9)

def parse_dets(res, ow, oh):
    parsed = []

    for name, value in res.items():
        if not isinstance(value, list):
            continue

        # Case: [ [class0_dets, class1_dets, ... class79_dets] ]
        if len(value) == 1 and isinstance(value[0], list):
            classes_list = value[0]
        else:
            classes_list = value

        for cls_id, class_dets in enumerate(classes_list):
            if class_dets is None:
                continue

            try:
                arr = np.asarray(class_dets, dtype=np.float32)
            except ValueError:
                continue

            if arr.size == 0:
                continue

            arr = np.squeeze(arr)

            if arr.ndim == 1:
                if arr.shape[0] < 5:
                    continue
                arr = arr.reshape(1, -1)

            if arr.ndim != 2 or arr.shape[1] < 5:
                continue

            for det in arr:
                y1, x1, y2, x2, score = det[:5]

                if score < CONF_THRESH:
                    continue

                # Hailo postprocess usually gives normalized yxyx
                if x2 <= 1.5 and y2 <= 1.5:
                    x1 *= ow
                    x2 *= ow
                    y1 *= oh
                    y2 *= oh

                ix1 = int(max(0, min(ow - 1, x1)))
                iy1 = int(max(0, min(oh - 1, y1)))
                ix2 = int(max(0, min(ow - 1, x2)))
                iy2 = int(max(0, min(oh - 1, y2)))

                if ix2 <= ix1 or iy2 <= iy1:
                    continue

                parsed.append({
                    "cls": int(cls_id),
                    "score": float(score),
                    "box": (ix1, iy1, ix2, iy2),
                    "center": ((ix1 + ix2) // 2, (iy1 + iy2) // 2),
                })

        return parsed

    return parsed

# ==========================
# INFERENCE THREAD
# ==========================
def inference_worker():
    global latest_camera_frame
    global latest_depth_frame

    fps_avg = 0

    while True:
        frame_rgb = camera.capture_array()

        if frame_rgb is None:
            print("Camera frame is None")
            continue

        frame_rgb = cv2.rotate(frame_rgb, cv2.ROTATE_180)
        frame = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)

        with frame_lock:
            latest_camera_frame = frame.copy()

        t1 = cv2.getTickCount()

        oh, ow = frame.shape[:2]

        # -------- DEPTH --------
        depth_input = cv2.resize(
            cv2.cvtColor(frame, cv2.COLOR_BGR2RGB),
            (DEPTH_W, DEPTH_H),
        )

        depth_input = np.expand_dims(depth_input, 0)

        with depth_ng.activate(depth_params):
            depth_out = depth_pipeline.infer({
                depth_input_info.name: depth_input,
            })

        raw_depth = depth_out[depth_output_info.name][0]

        resized_depth = cv2.resize(
            raw_depth,
            (ow, oh),
            interpolation=cv2.INTER_NEAREST,
        )

        depth_norm = cv2.normalize(
            resized_depth,
            None,
            0,
            255,
            cv2.NORM_MINMAX,
        )

        depth_norm = depth_norm.astype(np.uint8)

        depth_colormap = cv2.applyColorMap(
            depth_norm,
            cv2.COLORMAP_JET,
        )

        # -------- DETECTION --------
        det_input = cv2.resize(frame, (DET_W, DET_H))
        det_input = np.expand_dims(det_input, 0)

        with det_ng.activate(det_params):
            det_out = det_pipeline.infer({
                det_input_info.name: det_input,
            })

        dets = parse_dets(det_out, ow, oh)

        # -------- DRAW PREDICTIONS ON DEPTH MAP --------
        for det in dets:
            x1, y1, x2, y2 = det["box"]
            cx, cy = det["center"]
            score = det["score"]

            depth_value = resized_depth[cy, cx]

            cv2.rectangle(
                depth_colormap,
                (x1, y1),
                (x2, y2),
                (0, 255, 0),
                2,
            )

            label = COCO_LABELS[det["cls"]] if det["cls"] < len(COCO_LABELS) else str(det["cls"])
            txt = f"{label} {score:.2f} depth:{depth_value:.2f}"

            cv2.putText(
                depth_colormap,
                txt,
                (x1, max(20, y1 - 10)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 255),
                2,
            )

            cv2.circle(
                depth_colormap,
                (cx, cy),
                5,
                (255, 255, 255),
                -1,
            )

        # -------- FPS --------
        dt = (cv2.getTickCount() - t1) / cv2.getTickFrequency()

        if dt > 0:
            fps = 1.0 / dt
            fps_avg = fps if fps_avg == 0 else fps_avg * 0.9 + fps * 0.1

        cv2.putText(
            depth_colormap,
            f"FPS: {fps_avg:.1f}",
            (10, 35),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.0,
            (0, 255, 255),
            2,
        )

        success, buffer = cv2.imencode(
            ".jpg",
            depth_colormap,
            [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY],
        )

        if not success:
            print("Depth JPEG encode failed")
            continue

        with frame_lock:
            latest_depth_frame = buffer.tobytes()


# ==========================
# STREAM GENERATORS
# ==========================
def generate_camera_frames():
    while True:
        with frame_lock:
            if latest_camera_frame is None:
                frame = None
            else:
                frame = latest_camera_frame.copy()

        if frame is None:
            time.sleep(0.01)
            continue

        success, buffer = cv2.imencode(
            ".jpg",
            frame,
            [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY],
        )

        if not success:
            continue

        yield (
            b"--frame\r\n"
            b"Content-Type: image/jpeg\r\n\r\n"
            + buffer.tobytes()
            + b"\r\n"
        )


def generate_depth_frames():
    while True:
        with frame_lock:
            frame_bytes = latest_depth_frame

        if frame_bytes is None:
            time.sleep(0.01)
            continue

        yield (
            b"--frame\r\n"
            b"Content-Type: image/jpeg\r\n\r\n"
            + frame_bytes
            + b"\r\n"
        )


# ==========================
# FLASK ROUTES
# ==========================
@app.route("/")
def index():
    return """
    <html>
        <body style="background:#000; color:white;">
            <h1>CubeBot Camera Streams</h1>

            <h2>Raw Camera</h2>
            <img src="/video" width="768">

            <h2>Depth Map + Predictions</h2>
            <img src="/depth" width="768">
        </body>
    </html>
    """


@app.route("/video")
def video():
    return Response(
        generate_camera_frames(),
        mimetype="multipart/x-mixed-replace; boundary=frame",
    )


@app.route("/depth")
def depth():
    return Response(
        generate_depth_frames(),
        mimetype="multipart/x-mixed-replace; boundary=frame",
    )


# ==========================
# RUN
# ==========================
if __name__ == "__main__":
    threading.Thread(
        target=inference_worker,
        daemon=True,
    ).start()

    app.run(
        host="0.0.0.0",
        port=PORT,
        threaded=True,
        use_reloader=False,
    )