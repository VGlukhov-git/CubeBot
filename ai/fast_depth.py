import cv2
import numpy as np
import atexit
import threading
import time

from flask import Flask, Response
from picamera2 import Picamera2
from libcamera import controls

import socket
import struct

from hailo_platform import (
    HEF,
    VDevice,
    ConfigureParams,
    HailoStreamInterface,
    InputVStreamParams,
    OutputVStreamParams,
    InferVStreams,
    FormatType,
)

# ==========================================================
# CONFIG
# ==========================================================

DEPTH_HEF_PATH = "/home/vladimir/projects/CubeBot/ai/models/scdepthv3--320x256_quant_hailort_multidevice_1.hef"
SEG_HEF_PATH = "/home/vladimir/projects/CubeBot/ai/models/segmentation/yolov8m_seg.hef"
DET_HEF_PATH = "/home/vladimir/projects/CubeBot/ai/models/yolov11n.hef"

# Default: instance segmentation.
# Change to "detection" to use DET_HEF_PATH instead.
VISION_MODE = "detection"  # "segmentation" | "detection"

PORT = 5000
JPEG_QUALITY = 90
CONF_THRESH = 0.25
NMS_IOU = 0.45
MASK_THRESH = 0.50
MASK_ALPHA = 0.45
MAX_DETECTIONS = 50

# Depth grid shown on the right-side depth image.
# GRID_COLS = N, GRID_ROWS = M.
DEPTH_GRID_ENABLED = True
DEPTH_GRID_COLS = 5
DEPTH_GRID_ROWS = 5
DEPTH_GRID_STAT = "center"  # "median" | "mean" | "center"
DEPTH_GRID_DECIMALS = 3

CAMERA_WIDTH = 1920
CAMERA_HEIGHT = 1080

YOLO_INPUT_RGB = True

app = Flask(__name__)
latest_combined_frame = None
frame_lock = threading.Lock()

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
    "hair drier", "toothbrush",
]


# ==========================================================
# CAMERA
# ==========================================================

camera = Picamera2()
camera.configure(
    camera.create_video_configuration(
        main={
            "size": (CAMERA_WIDTH, CAMERA_HEIGHT),
            "format": "RGB888",
        }
    )
)

camera.start()
camera.set_controls({"AfMode": controls.AfModeEnum.Continuous})
time.sleep(1)


# ==========================================================
# HAILO INIT
# ==========================================================

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


print("Initializing depth model...")
depth_hef, depth_ng, depth_pipeline = setup_model(DEPTH_HEF_PATH)
depth_pipeline.__enter__()

depth_input_info = depth_hef.get_input_vstream_infos()[0]
depth_output_info = depth_hef.get_output_vstream_infos()[0]
DEPTH_H, DEPTH_W, _ = depth_input_info.shape
depth_params = depth_ng.create_params()

print("Depth input:", DEPTH_W, DEPTH_H)


# Initialize ONLY the selected vision model.
vision_hef = None
vision_ng = None
vision_pipeline = None
vision_input_info = None
vision_params = None
VISION_H = None
VISION_W = None

if VISION_MODE == "segmentation":
    print("Initializing YOLOv8m instance segmentation model...")
    vision_hef, vision_ng, vision_pipeline = setup_model(SEG_HEF_PATH)
elif VISION_MODE == "detection":
    print("Initializing object detection model...")
    vision_hef, vision_ng, vision_pipeline = setup_model(DET_HEF_PATH)
else:
    raise ValueError('VISION_MODE must be "segmentation" or "detection"')

vision_pipeline.__enter__()
vision_input_info = vision_hef.get_input_vstream_infos()[0]
VISION_H, VISION_W, _ = vision_input_info.shape
vision_params = vision_ng.create_params()

print("Vision mode:", VISION_MODE)
print("Vision input:", VISION_W, VISION_H)
print("Vision outputs:")
for info in vision_hef.get_output_vstream_infos():
    print(" ", info.name, info.shape)


# ==========================================================
# CLEANUP
# ==========================================================

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
        vision_pipeline.__exit__(None, None, None)
    except Exception:
        pass

    try:
        device.release()
    except Exception:
        pass


atexit.register(cleanup)


# ==========================================================
# COMMON HELPERS
# ==========================================================

def sigmoid(x):
    x = np.clip(x, -30.0, 30.0)
    return 1.0 / (1.0 + np.exp(-x))


def softmax_last(x):
    x = x - np.max(x, axis=-1, keepdims=True)
    e = np.exp(x)
    return e / (np.sum(e, axis=-1, keepdims=True) + 1e-9)


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


def class_aware_nms(boxes, scores, classes, iou_thres):
    keep_all = []

    for cls_id in np.unique(classes):
        idx = np.where(classes == cls_id)[0]
        keep_local = nms_xyxy(boxes[idx], scores[idx], iou_thres)
        keep_all.extend(idx[keep_local].tolist())

    if not keep_all:
        return np.array([], dtype=np.int64)

    keep_all = np.asarray(keep_all, dtype=np.int64)
    order = np.argsort(scores[keep_all])[::-1]
    return keep_all[order]


def prepare_yolo_input(frame_bgr, width, height):
    resized = cv2.resize(frame_bgr, (width, height), interpolation=cv2.INTER_LINEAR)

    if YOLO_INPUT_RGB:
        resized = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)

    return np.expand_dims(resized, axis=0)


def class_color(cls_id):
    # Deterministic color from class id.
    hue = int((cls_id * 37) % 180)
    hsv = np.uint8([[[hue, 220, 255]]])
    bgr = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)[0, 0]
    return tuple(int(v) for v in bgr)


def label_for_class(cls_id):
    return COCO_LABELS[cls_id] if 0 <= cls_id < len(COCO_LABELS) else str(cls_id)


# ==========================================================
# OBJECT DETECTION PARSER
# ==========================================================

def parse_dets(res, ow, oh):
    """
    Parser for a Hailo NMS-style detection output such as the one used by
    your previous yolov11n HEF.
    """
    parsed = []

    for _, value in res.items():
        if not isinstance(value, list):
            continue

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

                # Normalized Hailo NMS coordinates.
                if x2 <= 1.5 and y2 <= 1.5:
                    x1 *= ow
                    x2 *= ow
                    y1 *= oh
                    y2 *= oh

                ix1 = int(np.clip(x1, 0, ow - 1))
                iy1 = int(np.clip(y1, 0, oh - 1))
                ix2 = int(np.clip(x2, 0, ow - 1))
                iy2 = int(np.clip(y2, 0, oh - 1))

                if ix2 <= ix1 or iy2 <= iy1:
                    continue

                parsed.append({
                    "cls": int(cls_id),
                    "score": float(score),
                    "box": (ix1, iy1, ix2, iy2),
                    "center": ((ix1 + ix2) // 2, (iy1 + iy2) // 2),
                    "mask": None,
                })

    return parsed


# ==========================================================
# YOLOv8 INSTANCE SEGMENTATION POSTPROCESS
# ==========================================================

def _strip_batch(arr):
    arr = np.asarray(arr, dtype=np.float32)
    if arr.ndim == 4 and arr.shape[0] == 1:
        arr = arr[0]
    return arr


def identify_yolov8_seg_outputs(res):
    """
    Hailo YOLOv8-seg normally gives 10 NHWC outputs:
      3 x bbox distributions (64 channels)
      3 x class scores       (80 channels for COCO)
      3 x mask coefficients  (32 channels)
      1 x mask prototypes    (32 channels, largest spatial map)

    We identify them by shape instead of relying on Hailo tensor names.
    """
    tensors = []

    for name, value in res.items():
        arr = _strip_batch(value)
        if arr.ndim != 3:
            continue

        h, w, c = arr.shape
        tensors.append({
            "name": name,
            "array": arr,
            "h": h,
            "w": w,
            "c": c,
        })

    if not tensors:
        raise RuntimeError("No 3D NHWC tensors found in YOLOv8-seg output")

    bbox_tensors = [t for t in tensors if t["c"] == 64]
    score_tensors = [t for t in tensors if t["c"] == len(COCO_LABELS)]
    mask32_tensors = [t for t in tensors if t["c"] == 32]

    if len(bbox_tensors) != 3:
        raise RuntimeError(
            f"Expected 3 bbox tensors with 64 channels, got {len(bbox_tensors)}. "
            f"Outputs: {[(t['name'], t['array'].shape) for t in tensors]}"
        )

    if len(score_tensors) != 3:
        raise RuntimeError(
            f"Expected 3 score tensors with {len(COCO_LABELS)} channels, got {len(score_tensors)}. "
            "If this is a custom model, update COCO_LABELS / class count."
        )

    if len(mask32_tensors) != 4:
        raise RuntimeError(
            f"Expected 4 tensors with 32 channels (3 coeff + 1 proto), got {len(mask32_tensors)}"
        )

    # Prototype tensor is the 32-channel tensor with the largest spatial map.
    proto_tensor = max(mask32_tensors, key=lambda t: t["h"] * t["w"])
    coeff_tensors = [t for t in mask32_tensors if t is not proto_tensor]

    bbox_by_size = {(t["h"], t["w"]): t for t in bbox_tensors}
    score_by_size = {(t["h"], t["w"]): t for t in score_tensors}
    coeff_by_size = {(t["h"], t["w"]): t for t in coeff_tensors}

    scales = []
    for size, bbox in bbox_by_size.items():
        if size not in score_by_size or size not in coeff_by_size:
            raise RuntimeError(f"Could not match YOLOv8-seg tensors for feature map {size}")

        scales.append({
            "h": size[0],
            "w": size[1],
            "bbox": bbox["array"],
            "scores": score_by_size[size]["array"],
            "coeff": coeff_by_size[size]["array"],
        })

    # e.g. 20x20, 40x40, 80x80
    scales.sort(key=lambda s: s["h"] * s["w"])

    return scales, proto_tensor["array"]


def decode_yolov8_seg(res, output_w, output_h):
    scales, proto = identify_yolov8_seg_outputs(res)

    all_boxes = []
    all_scores = []
    all_classes = []
    all_coeffs = []

    reg_max = 16  # 64 bbox channels / 4 sides
    reg_range = np.arange(reg_max, dtype=np.float32)

    for scale in scales:
        fh = scale["h"]
        fw = scale["w"]

        bbox_raw = scale["bbox"].reshape(-1, 4, reg_max)
        scores_raw = scale["scores"].reshape(-1, len(COCO_LABELS))
        coeff_raw = scale["coeff"].reshape(-1, 32)

        # YOLOv8 DFL bbox decoding.
        distances = np.sum(softmax_last(bbox_raw) * reg_range, axis=-1)

        stride_x = VISION_W / float(fw)
        stride_y = VISION_H / float(fh)

        xs = (np.arange(fw, dtype=np.float32) + 0.5) * stride_x
        ys = (np.arange(fh, dtype=np.float32) + 0.5) * stride_y
        grid_x, grid_y = np.meshgrid(xs, ys)

        cx = grid_x.reshape(-1)
        cy = grid_y.reshape(-1)

        x1 = cx - distances[:, 0] * stride_x
        y1 = cy - distances[:, 1] * stride_y
        x2 = cx + distances[:, 2] * stride_x
        y2 = cy + distances[:, 3] * stride_y

        boxes = np.stack([x1, y1, x2, y2], axis=1)

        # Hailo Model Zoo YOLOv8 score heads are normally already activated.
        classes = np.argmax(scores_raw, axis=1)
        scores = scores_raw[np.arange(scores_raw.shape[0]), classes]

        valid = scores >= CONF_THRESH

        if not np.any(valid):
            continue

        all_boxes.append(boxes[valid])
        all_scores.append(scores[valid])
        all_classes.append(classes[valid])
        all_coeffs.append(coeff_raw[valid])

    if not all_boxes:
        return []

    boxes = np.concatenate(all_boxes, axis=0).astype(np.float32)
    scores = np.concatenate(all_scores, axis=0).astype(np.float32)
    classes = np.concatenate(all_classes, axis=0).astype(np.int32)
    coeffs = np.concatenate(all_coeffs, axis=0).astype(np.float32)

    # Clip boxes in model-input coordinates before NMS.
    boxes[:, [0, 2]] = np.clip(boxes[:, [0, 2]], 0, VISION_W - 1)
    boxes[:, [1, 3]] = np.clip(boxes[:, [1, 3]], 0, VISION_H - 1)

    keep = class_aware_nms(boxes, scores, classes, NMS_IOU)
    keep = keep[:MAX_DETECTIONS]

    boxes = boxes[keep]
    scores = scores[keep]
    classes = classes[keep]
    coeffs = coeffs[keep]

    # Mask prototypes: [proto_h, proto_w, 32]
    proto_h, proto_w, proto_c = proto.shape
    if proto_c != 32:
        raise RuntimeError(f"Unexpected prototype tensor shape: {proto.shape}")

    proto_flat = proto.reshape(-1, proto_c)
    mask_logits = proto_flat @ coeffs.T
    masks_small = sigmoid(mask_logits).reshape(proto_h, proto_w, -1)

    sx = output_w / float(VISION_W)
    sy = output_h / float(VISION_H)

    detections = []

    for i in range(len(keep)):
        mx1, my1, mx2, my2 = boxes[i]

        x1 = int(np.clip(mx1 * sx, 0, output_w - 1))
        y1 = int(np.clip(my1 * sy, 0, output_h - 1))
        x2 = int(np.clip(mx2 * sx, 0, output_w - 1))
        y2 = int(np.clip(my2 * sy, 0, output_h - 1))

        if x2 <= x1 or y2 <= y1:
            continue

        # Resize prototype mask directly to camera dimensions because the model
        # input above is a simple resize (not letterboxed).
        mask_full = cv2.resize(
            masks_small[:, :, i],
            (output_w, output_h),
            interpolation=cv2.INTER_LINEAR,
        )

        mask_binary = mask_full >= MASK_THRESH

        # Restrict mask to its detection box.
        crop_mask = np.zeros_like(mask_binary, dtype=bool)
        crop_mask[y1:y2 + 1, x1:x2 + 1] = True
        mask_binary &= crop_mask

        cx = (x1 + x2) // 2
        cy = (y1 + y2) // 2

        detections.append({
            "cls": int(classes[i]),
            "score": float(scores[i]),
            "box": (x1, y1, x2, y2),
            "center": (cx, cy),
            "mask": mask_binary,
        })

    return detections


# ==========================================================
# DRAWING
# ==========================================================

def draw_mask(image, mask, color, alpha=MASK_ALPHA):
    if mask is None or not np.any(mask):
        return

    overlay = image.copy()
    overlay[mask] = color
    cv2.addWeighted(overlay, alpha, image, 1.0 - alpha, 0, dst=image)


def extract_depth_grid(depth_map, cols, rows, statistic="median"):
    """
    Split depth_map into rows x cols cells and return one depth value per cell.

    The returned values are in the native units produced by the depth model.
    For monocular relative-depth models these are NOT meters unless the model
    has been calibrated separately.
    """
    h, w = depth_map.shape[:2]
    values = np.full((rows, cols), np.nan, dtype=np.float32)

    x_edges = np.linspace(0, w, cols + 1, dtype=np.int32)
    y_edges = np.linspace(0, h, rows + 1, dtype=np.int32)

    for row in range(rows):
        y1, y2 = y_edges[row], y_edges[row + 1]

        for col in range(cols):
            x1, x2 = x_edges[col], x_edges[col + 1]
            cell = depth_map[y1:y2, x1:x2]

            if cell.size == 0:
                continue

            # Ignore NaN/Inf values if the model produces any.
            valid = cell[np.isfinite(cell)]
            if valid.size == 0:
                continue

            if statistic == "mean":
                value = np.mean(valid)
            elif statistic == "center":
                cx = min(w - 1, (x1 + x2) // 2)
                cy = min(h - 1, (y1 + y2) // 2)
                value = depth_map[cy, cx]
            else:
                value = np.median(valid)

            values[row, col] = float(value)

    return values, x_edges, y_edges


def draw_depth_grid(depth_image, depth_map):
    grid_values, x_edges, y_edges = extract_depth_grid(
        depth_map,
        DEPTH_GRID_COLS,
        DEPTH_GRID_ROWS,
        DEPTH_GRID_STAT,
    )

    h, w = depth_image.shape[:2]

    # Grid lines
    for x in x_edges[1:-1]:
        cv2.line(depth_image, (int(x), 0), (int(x), h - 1), (255, 255, 255), 1)

    for y in y_edges[1:-1]:
        cv2.line(depth_image, (0, int(y)), (w - 1, int(y)), (255, 255, 255), 1)

    # Values at cell centers
    for row in range(DEPTH_GRID_ROWS):
        for col in range(DEPTH_GRID_COLS):
            value = grid_values[row, col]
            if not np.isfinite(value):
                text = "--"
            else:
                text = f"{value:.{DEPTH_GRID_DECIMALS}f}"

            cx = int((x_edges[col] + x_edges[col + 1]) // 2)
            cy = int((y_edges[row] + y_edges[row + 1]) // 2)

            # Black outline + white text keeps values visible over the colormap.
            (tw, th), _ = cv2.getTextSize(
                text,
                cv2.FONT_HERSHEY_SIMPLEX,
                0.50,
                1,
            )
            tx = max(2, cx - tw // 2)
            ty = max(th + 2, cy + th // 2)

            cv2.putText(
                depth_image,
                text,
                (tx, ty),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.50,
                (0, 0, 0),
                3,
                cv2.LINE_AA,
            )
            cv2.putText(
                depth_image,
                text,
                (tx, ty),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.50,
                (255, 255, 255),
                1,
                cv2.LINE_AA,
            )

    return grid_values


def draw_vision_results(camera_frame, depth_frame, detections, resized_depth):
    oh, ow = camera_frame.shape[:2]

    for det in detections:
        cls_id = det["cls"]
        score = det["score"]
        x1, y1, x2, y2 = det["box"]
        cx, cy = det["center"]
        mask = det.get("mask")

        cx = int(np.clip(cx, 0, ow - 1))
        cy = int(np.clip(cy, 0, oh - 1))
        depth_value = float(resized_depth[cy, cx])

        color = class_color(cls_id)

        if mask is not None:
            draw_mask(camera_frame, mask, color)
            draw_mask(depth_frame, mask, color)

        cv2.rectangle(camera_frame, (x1, y1), (x2, y2), color, 2)
        cv2.rectangle(depth_frame, (x1, y1), (x2, y2), color, 2)

        label = label_for_class(cls_id)
        txt = f"{label} {score:.2f} depth:{depth_value:.2f}"

        cv2.putText(
            camera_frame,
            txt,
            (x1, max(22, y1 - 8)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.58,
            color,
            2,
        )

        cv2.putText(
            depth_frame,
            txt,
            (x1, max(22, y1 - 8)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.58,
            (255, 255, 255),
            2,
        )

        cv2.circle(camera_frame, (cx, cy), 4, color, -1)
        cv2.circle(depth_frame, (cx, cy), 4, (255, 255, 255), -1)


# ==========================================================
# INFERENCE WORKER
# ==========================================================

UNITY_IP = "192.168.1.6"   # PC running Unity
UNITY_PORT = 5005

depth_socket = socket.socket(
    socket.AF_INET,
    socket.SOCK_DGRAM,
)

depth_frame_id = 0

def send_depth_grid_udp(depth_grid):
    global depth_frame_id

    if depth_grid is None:
        return

    # Make sure data is contiguous float32
    grid = np.ascontiguousarray(
        depth_grid,
        dtype=np.float32,
    )

    rows, cols = grid.shape

    # Header:
    # uint16 rows
    # uint16 cols
    # uint32 frame ID
    header = struct.pack(
        "<HHI",
        rows,
        cols,
        depth_frame_id,
    )

    packet = header + grid.tobytes()

    try:
        depth_socket.sendto(
            packet,
            (UNITY_IP, UNITY_PORT),
        )

        depth_frame_id += 1

    except OSError as e:
        print("Depth UDP send error:", e)

def inference_worker():
    global latest_combined_frame

    fps_avg = 0.0

    while True:
        frame_rgb = camera.capture_array()

        if frame_rgb is None:
            print("Camera frame is None")
            continue

        frame = cv2.rotate(frame_rgb, cv2.ROTATE_180)

#        frame = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
        oh, ow = frame.shape[:2]

        t1 = cv2.getTickCount()

        # --------------------------------------------------
        # DEPTH
        # --------------------------------------------------
        depth_input = cv2.resize(
            cv2.cvtColor(frame, cv2.COLOR_BGR2RGB),
            (DEPTH_W, DEPTH_H),
            interpolation=cv2.INTER_LINEAR,
        )
        depth_input = np.expand_dims(depth_input, axis=0)

        with depth_ng.activate(depth_params):
            depth_out = depth_pipeline.infer({
                depth_input_info.name: depth_input,
            })

        raw_depth = depth_out[depth_output_info.name][0]

        # Some depth HEFs return HxW, others HxWx1.
        raw_depth = np.squeeze(raw_depth)

        resized_depth = cv2.resize(
            raw_depth,
            (ow, oh),
            interpolation=cv2.INTER_NEAREST
        )


        depth_norm = cv2.normalize(
                    resized_depth,
                    None,
                    0,
                    255,
                    cv2.NORM_MINMAX
                )
        depth_norm = depth_norm.astype(np.uint8)

        depth_colormap = cv2.applyColorMap(depth_norm, cv2.COLORMAP_JET)


        # --------------------------------------------------
        # VISION: SEGMENTATION OR DETECTION
        # --------------------------------------------------
        vision_input = prepare_yolo_input(frame, VISION_W, VISION_H)

        with vision_ng.activate(vision_params):
            vision_out = vision_pipeline.infer({
                vision_input_info.name: vision_input,
            })

        if VISION_MODE == "segmentation":
            detections = decode_yolov8_seg(vision_out, ow, oh)
        else:
            detections = parse_dets(vision_out, ow, oh)

        camera_visual = frame.copy()
        draw_vision_results(camera_visual, depth_colormap, detections, resized_depth)

        # --------------------------------------------------
        # N x M DEPTH GRID
        # --------------------------------------------------
        if DEPTH_GRID_ENABLED:
            depth_grid_values, x_edges, y_edges = extract_depth_grid(
                resized_depth,
                DEPTH_GRID_COLS,
                DEPTH_GRID_ROWS,
                DEPTH_GRID_STAT,
            )
            send_depth_grid_udp(
                depth_grid_values
            )
            draw_depth_grid(depth_colormap, resized_depth)
        else:
            depth_grid_values = None

        # --------------------------------------------------
        # FPS / LABELS
        # --------------------------------------------------
        dt = (cv2.getTickCount() - t1) / cv2.getTickFrequency()

        if dt > 0:
            fps = 1.0 / dt
            fps_avg = fps if fps_avg == 0 else fps_avg * 0.9 + fps * 0.1

        mode_text = "INSTANCE SEGMENTATION" if VISION_MODE == "segmentation" else "OBJECT DETECTION"

        cv2.putText(
            camera_visual,
            f"{mode_text} | FPS: {fps_avg:.1f}",
            (10, 35),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.9,
            (0, 255, 255),
            2,
        )

        cv2.putText(
            depth_colormap,
            f"DEPTH | FPS: {fps_avg:.1f}",
            (10, 35),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.9,
            (0, 255, 255),
            2,
        )

        # --------------------------------------------------
        # SIDE-BY-SIDE
        # --------------------------------------------------
        combined = np.hstack((camera_visual, depth_colormap))

        success, buffer = cv2.imencode(
            ".jpg",
            combined,
            [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY],
        )

        if not success:
            print("Combined JPEG encode failed")
            continue

        with frame_lock:
            latest_combined_frame = buffer.tobytes()


# ==========================================================
# MJPEG STREAM
# ==========================================================

def generate_frames():
    while True:
        with frame_lock:
            frame_bytes = latest_combined_frame

        if frame_bytes is None:
            time.sleep(0.01)
            continue

        yield (
            b"--frame\r\n"
            b"Content-Type: image/jpeg\r\n\r\n"
            + frame_bytes
            + b"\r\n"
        )

        time.sleep(0.01)


# ==========================================================
# FLASK
# ==========================================================

@app.route("/")
def index():
    return f"""
    <html>
        <head>
            <title>CubeBot Vision</title>
        </head>
        <body style="margin:0;background:#000;color:white;text-align:center;font-family:Arial;">
            <h2>CubeBot - {VISION_MODE}</h2>
            <img src="/stream" style="width:98vw;height:auto;">
        </body>
    </html>
    """


@app.route("/stream")
def stream():
    return Response(
        generate_frames(),
        mimetype="multipart/x-mixed-replace; boundary=frame",
    )


# ==========================================================
# RUN
# ==========================================================

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
