"""Picamera2 -> optional Hailo inference -> MJPEG stream.

Notes:
* Detection decoder supports Hailo NMS-by-class output, not raw YOLO heads.
* Segmentation decoder supports the 10 raw YOLOv8m-seg HEF heads (DFL + prototypes).
* Multiple enabled HEFs run sequentially: one active network at a time.
* Resize-only YOLO preprocessing (not letterbox) matches coordinate mapping.
"""
from __future__ import annotations

import logging
import threading
import time
from contextlib import ExitStack
from pathlib import Path

import cv2
import numpy as np
from flask import Flask, Response, jsonify, request
from libcamera import controls
from picamera2 import Picamera2

# ---------------------- configuration ----------------------
DEPTH_ENABLED = False
SEGMENTATION_ENABLED = False
DETECTION_ENABLED = False

DEPTH_HEF_PATH = '/home/vladimir/projects/CubeBot/ai/models/scdepthv3--320x256_quant_hailort_multidevice_1.hef'
SEG_HEF_PATH = '/home/vladimir/projects/CubeBot/ai/models/segmentation/yolov8m_seg.hef'
DET_HEF_PATH = '/home/vladimir/projects/CubeBot/ai/models/yolov11n.hef'

CAMERA_SIZE = (1920, 1080)
JPEG_QUALITY = 80
DETECTION_THRESHOLD = 0.40
SEGMENTATION_THRESHOLD = 0.25
SEGMENTATION_MASK_THRESHOLD = 0.50
SEGMENTATION_NMS_IOU = 0.45
MAX_SEGMENTATION_INSTANCES = 30
DEPTH_MODE = 'depth'  # 'depth', 'overlay', or 'side_by_side'
DEPTH_OPACITY = 0.45  # used only for 'overlay'
SEGMENTATION_OPACITY = 0.40
MAX_STREAM_FPS = 30

logging.basicConfig(level=logging.INFO, format='%(asctime)s %(levelname)s %(message)s')
LOG = logging.getLogger('camera-ai')
app = Flask(__name__)


class HailoModel:
    """Own one HailoRT network and its inference context."""

    def __init__(self, path: str, device):
        from hailo_platform import (
            HEF, ConfigureParams, HailoStreamInterface,
            InputVStreamParams, OutputVStreamParams, InferVStreams, FormatType,
        )

        if not Path(path).is_file():
            raise FileNotFoundError(path)
        self.hef = HEF(path)
        self.input_infos = self.hef.get_input_vstream_infos()
        self.output_infos = self.hef.get_output_vstream_infos()
        if len(self.input_infos) != 1:
            raise ValueError(f'{path}: expected 1 input, got {len(self.input_infos)}')
        in_shape = self.input_infos[0].shape  # H, W, C
        if len(in_shape) != 3 or in_shape[-1] != 3:
            raise ValueError(f'{path}: expected NHWC RGB input; got {in_shape}')
        self.height, self.width = int(in_shape[0]), int(in_shape[1])
        self.input_name = self.input_infos[0].name
        self.name = Path(path).stem
        self.configure_params = ConfigureParams.create_from_hef(
            self.hef, interface=HailoStreamInterface.PCIe
        )
        self.network_group = device.configure(self.hef, self.configure_params)[0]
        self.activation_params = self.network_group.create_params()
        self.input_params = InputVStreamParams.make(
            self.network_group, format_type=FormatType.UINT8
        )
        self.output_params = OutputVStreamParams.make(
            self.network_group, format_type=FormatType.FLOAT32
        )
        self.stack = ExitStack()
        self.pipeline = None
        LOG.info('%s input=%s outputs=%s', self.name, in_shape,
                 [(x.name, str(x.shape), str(x.format.order)) for x in self.output_infos])

    def __enter__(self):
        from hailo_platform import InferVStreams
        try:
            self.pipeline = self.stack.enter_context(
                InferVStreams(self.network_group, self.input_params, self.output_params)
            )
        except Exception:
            self.stack.close()
            raise
        return self

    def __exit__(self, *args):
        return self.stack.__exit__(*args)

    def infer(self, frame_bgr: np.ndarray):
        # Resize directly to model dimensions; no letterbox, so mapping is linear.
        resized = cv2.resize(frame_bgr, (self.width, self.height))
        rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
        batch = np.ascontiguousarray(rgb[None, ...], dtype=np.uint8)
        # A Hailo-8L cannot keep multiple manually managed network groups
        # activated at once. Activate only around the blocking inference call.
        with self.network_group.activate(self.activation_params):
            return self.pipeline.infer({self.input_name: batch})


def first_output(outputs):
    if len(outputs) != 1:
        raise ValueError(f'Expected a single decoded output, got {list(outputs)}')
    return next(iter(outputs.values()))


def depth_overlay(frame: np.ndarray, outputs: dict):
    """Render SCDepthV3 like the working reference implementation.

    Processing order is important:
      raw Hailo depth -> resize with NEAREST -> min/max normalize -> JET.
    Native model values are relative depth, not calibrated meters.
    """
    depth = np.asarray(first_output(outputs), dtype=np.float32).squeeze()
    if depth.ndim != 2:
        raise ValueError(f'Depth output must be a 2D map, got {depth.shape}')

    h, w = frame.shape[:2]
    depth = cv2.resize(depth, (w, h), interpolation=cv2.INTER_NEAREST)

    finite = np.isfinite(depth)
    if not finite.any():
        LOG.warning('Depth model returned no finite values')
        return frame
    if not finite.all():
        lo = float(depth[finite].min())
        hi = float(depth[finite].max())
        depth = np.nan_to_num(depth, nan=lo, neginf=lo, posinf=hi)

    # Match the supplied working depth pipeline: no percentile clipping.
    depth_norm = cv2.normalize(depth, None, 0, 255, cv2.NORM_MINMAX)
    depth_norm = depth_norm.astype(np.uint8)
    depth_colormap = cv2.applyColorMap(depth_norm, cv2.COLORMAP_JET)

    if DEPTH_MODE == 'depth':
        return depth_colormap
    if DEPTH_MODE == 'overlay':
        return cv2.addWeighted(frame, 1.0 - DEPTH_OPACITY,
                               depth_colormap, DEPTH_OPACITY, 0)
    if DEPTH_MODE == 'side_by_side':
        return np.hstack((frame, depth_colormap))
    raise ValueError(f'Unknown DEPTH_MODE: {DEPTH_MODE!r}')


def decode_hailo_nms(outputs: dict):
    """Yield Hailo NMS-by-class detections (class, ymin, xmin, ymax, xmax, score).

    HailoRT may return a ragged list with one entry per class, potentially
    wrapped in a batch list, or a dense [classes, boxes, 5] tensor.
    Each class is converted independently because class counts vary.
    """
    raw = first_output(outputs)

    if isinstance(raw, (list, tuple)):
        classes = raw
        # Some HailoRT versions include an outer batch dimension (batch=1).
        if len(classes) == 1 and isinstance(classes[0], (list, tuple)):
            classes = classes[0]
    else:
        array = np.asarray(raw)
        if array.ndim == 4 and array.shape[0] == 1:
            array = array[0]
        if array.ndim != 3 or array.shape[-1] < 5:
            raise ValueError(f'Unsupported detection output shape {array.shape}. '
                             'Expected Hailo NMS-by-class output.')
        classes = array

    for class_id, class_boxes in enumerate(classes):
        if class_boxes is None:
            continue
        # Convert one class only; converting the full ragged class list fails.
        boxes = np.asarray(class_boxes, dtype=np.float32)
        if boxes.size == 0:
            continue
        boxes = np.squeeze(boxes)
        if boxes.ndim == 1:
            if boxes.shape[0] < 5:
                raise ValueError(f'Class {class_id}: invalid detection {boxes.shape}')
            boxes = boxes.reshape(1, -1)
        if boxes.ndim != 2 or boxes.shape[1] < 5:
            raise ValueError(f'Class {class_id}: unexpected boxes shape {boxes.shape}')

        for ymin, xmin, ymax, xmax, score in boxes[:, :5]:
            if not np.all(np.isfinite((ymin, xmin, ymax, xmax, score))):
                continue
            if score >= DETECTION_THRESHOLD:
                yield (class_id, float(ymin), float(xmin), float(ymax),
                       float(xmax), float(score))


def detection_overlay(frame: np.ndarray, outputs: dict):
    h, w = frame.shape[:2]
    for class_id, y1, x1, y2, x2, score in decode_hailo_nms(outputs):
        p1 = (int(np.clip(x1, 0, 1) * w), int(np.clip(y1, 0, 1) * h))
        p2 = (int(np.clip(x2, 0, 1) * w), int(np.clip(y2, 0, 1) * h))
        cv2.rectangle(frame, p1, p2, (0, 255, 0), 2)
        cv2.putText(frame, f'class {class_id}: {score:.2f}',
                    (p1[0], max(20, p1[1] - 8)), cv2.FONT_HERSHEY_SIMPLEX,
                    0.65, (0, 255, 0), 2)
    return frame


def _sigmoid(x: np.ndarray) -> np.ndarray:
    return 1.0 / (1.0 + np.exp(-np.clip(x, -30.0, 30.0)))


def _softmax(x: np.ndarray) -> np.ndarray:
    x = x - x.max(axis=-1, keepdims=True)
    e = np.exp(x)
    return e / e.sum(axis=-1, keepdims=True)


def _nms_xyxy(boxes: np.ndarray, scores: np.ndarray, iou_threshold: float) -> np.ndarray:
    order = np.argsort(scores)[::-1]
    keep = []
    areas = np.maximum(0, boxes[:, 2] - boxes[:, 0]) * np.maximum(0, boxes[:, 3] - boxes[:, 1])
    while order.size:
        i = int(order[0])
        keep.append(i)
        if order.size == 1:
            break
        rest = order[1:]
        lt = np.maximum(boxes[i, :2], boxes[rest, :2])
        rb = np.minimum(boxes[i, 2:], boxes[rest, 2:])
        wh = np.maximum(0, rb - lt)
        intersection = wh[:, 0] * wh[:, 1]
        iou = intersection / (areas[i] + areas[rest] - intersection + 1e-9)
        order = rest[iou <= iou_threshold]
    return np.asarray(keep, dtype=np.int64)


def _class_aware_nms(boxes: np.ndarray, scores: np.ndarray,
                     classes: np.ndarray) -> np.ndarray:
    selections = []
    for cls in np.unique(classes):
        ids = np.flatnonzero(classes == cls)
        selections.extend(ids[_nms_xyxy(boxes[ids], scores[ids], SEGMENTATION_NMS_IOU)].tolist())
    selections = np.asarray(selections, dtype=np.int64)
    return selections[np.argsort(scores[selections])[::-1]][:MAX_SEGMENTATION_INSTANCES]


def _parse_seg_heads(outputs: dict):
    """10 raw NHWC heads: 3 bbox(64), 3 class(80), 3 coeff(32), 1 proto(32).

    Hailo's FCR is a tensor stream format; output arrays returned by HailoRT
    with FLOAT32 VStreams are expected to be NHWC as in the supplied reference.
    """
    groups = {64: [], 80: [], 32: []}
    for name, value in outputs.items():
        array = np.asarray(value, dtype=np.float32)
        if array.ndim == 4 and array.shape[0] == 1:
            array = array[0]
        if array.ndim != 3 or array.shape[-1] not in groups:
            raise ValueError(f'Unexpected segmentation tensor {name}: {array.shape}')
        groups[array.shape[-1]].append((name, array))

    if tuple(len(groups[c]) for c in (64, 80, 32)) != (3, 3, 4):
        raise ValueError('Expected YOLOv8-seg heads 3x64, 3x80, 4x32; got '
                         + str({c: [(n, a.shape) for n, a in v] for c, v in groups.items()}))
    proto_name, proto = max(groups[32], key=lambda x: x[1].shape[0] * x[1].shape[1])
    coeffs = [(n, a) for n, a in groups[32] if n != proto_name]

    by_size = {}
    for kind, collection in (('bbox', groups[64]), ('scores', groups[80]), ('coeff', coeffs)):
        for name, tensor in collection:
            shape = tensor.shape[:2]
            if kind in by_size.setdefault(shape, {}):
                raise ValueError(f'Duplicate {kind} tensor for scale {shape}')
            by_size[shape][kind] = tensor
    if len(by_size) != 3 or any(set(d) != {'bbox', 'scores', 'coeff'} for d in by_size.values()):
        raise ValueError(f'Unmatched YOLOv8-seg scales: {[(s, list(d)) for s, d in by_size.items()]}')
    return [by_size[s] for s in sorted(by_size)], proto


def decode_yolov8_seg(outputs: dict, image_w: int, image_h: int,
                      model_w: int = 640, model_h: int = 640):
    """Decode this Hailo YOLOv8m-seg HEF into (class, confidence, box, mask).

    Assumes class scores are already activated, matching the working reference.
    Boxes are in resized model-input space; camera preprocessing uses resize,
    not letterbox. Results are raw instance masks, not semantic class maps.
    """
    heads, proto = _parse_seg_heads(outputs)
    all_boxes, all_scores, all_classes, all_coeffs = [], [], [], []
    dfl_bins = np.arange(16, dtype=np.float32)

    for group in heads:
        fh, fw = group['bbox'].shape[:2]
        distances = (_softmax(group['bbox'].reshape(-1, 4, 16)) * dfl_bins).sum(axis=-1)
        class_scores = group['scores'].reshape(-1, 80)
        coeffs = group['coeff'].reshape(-1, 32)
        classes = np.argmax(class_scores, axis=-1)
        scores = class_scores[np.arange(len(classes)), classes]
        valid = np.isfinite(scores) & (scores >= SEGMENTATION_THRESHOLD)
        if not np.any(valid):
            continue

        stride_x, stride_y = model_w / fw, model_h / fh
        xx, yy = np.meshgrid((np.arange(fw) + 0.5) * stride_x,
                             (np.arange(fh) + 0.5) * stride_y)
        cx, cy = xx.reshape(-1), yy.reshape(-1)
        boxes = np.column_stack((cx - distances[:, 0] * stride_x,
                                 cy - distances[:, 1] * stride_y,
                                 cx + distances[:, 2] * stride_x,
                                 cy + distances[:, 3] * stride_y))
        all_boxes.append(boxes[valid])
        all_scores.append(scores[valid])
        all_classes.append(classes[valid])
        all_coeffs.append(coeffs[valid])

    if not all_boxes:
        return []
    boxes = np.concatenate(all_boxes).astype(np.float32)
    scores = np.concatenate(all_scores).astype(np.float32)
    classes = np.concatenate(all_classes).astype(np.int32)
    coeffs = np.concatenate(all_coeffs).astype(np.float32)
    boxes[:, [0, 2]] = np.clip(boxes[:, [0, 2]], 0, model_w - 1)
    boxes[:, [1, 3]] = np.clip(boxes[:, [1, 3]], 0, model_h - 1)
    selected = _class_aware_nms(boxes, scores, classes)
    if selected.size == 0:
        return []
    boxes, scores, classes, coeffs = (a[selected] for a in (boxes, scores, classes, coeffs))

    ph, pw, pc = proto.shape
    if pc != 32:
        raise ValueError(f'Expected 32 mask prototypes, got {proto.shape}')
    masks = _sigmoid(proto.reshape(-1, 32) @ coeffs.T).reshape(ph, pw, -1)
    xscale, yscale = image_w / model_w, image_h / model_h
    results = []
    for i, (box, score, cls) in enumerate(zip(boxes, scores, classes)):
        x1 = int(np.clip(box[0] * xscale, 0, image_w - 1))
        y1 = int(np.clip(box[1] * yscale, 0, image_h - 1))
        x2 = int(np.clip(box[2] * xscale, 0, image_w - 1))
        y2 = int(np.clip(box[3] * yscale, 0, image_h - 1))
        if x2 <= x1 or y2 <= y1:
            continue
        # Resize probability mask, then crop it to its detected bounding box.
        mask = cv2.resize(masks[:, :, i], (image_w, image_h),
                          interpolation=cv2.INTER_LINEAR) >= SEGMENTATION_MASK_THRESHOLD
        cropped_mask = np.zeros((image_h, image_w), dtype=bool)
        cropped_mask[y1:y2 + 1, x1:x2 + 1] = mask[y1:y2 + 1, x1:x2 + 1]
        results.append((int(cls), float(score), (x1, y1, x2, y2), cropped_mask))
    return results


def segmentation_overlay(frame: np.ndarray, outputs: dict):
    """Draw YOLOv8 instance masks and boxes on BGR camera image."""
    h, w = frame.shape[:2]
    instances = decode_yolov8_seg(outputs, w, h)
    for cls, score, (x1, y1, x2, y2), mask in instances:
        hue = int(cls * 37 % 180)
        color = tuple(int(x) for x in cv2.cvtColor(
            np.uint8([[[hue, 220, 255]]]), cv2.COLOR_HSV2BGR)[0, 0])
        if np.any(mask):
            frame[mask] = (frame[mask].astype(np.float32) * (1 - SEGMENTATION_OPACITY)
                           + np.asarray(color, dtype=np.float32) * SEGMENTATION_OPACITY).astype(np.uint8)
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
        cv2.putText(frame, f'class {cls}: {score:.2f}',
                    (x1, max(20, y1 - 8)), cv2.FONT_HERSHEY_SIMPLEX,
                    0.65, color, 2)
    return frame


class CameraStream:
    def __init__(self):
        self.condition = threading.Condition()
        self.jpeg = None
        self.sequence = 0
        self.stopped = threading.Event()
        self.thread = None
        self.camera = None
        self.error = None
        self.mode_lock = threading.Lock()
        self.modes = {
            'depth': DEPTH_ENABLED,
            'segmentation': SEGMENTATION_ENABLED,
            'detection': DETECTION_ENABLED,
        }
        self.mode_errors = {}

    def status(self):
        with self.mode_lock:
            return {'modes': self.modes.copy(), 'errors': self.mode_errors.copy(),
                    'running': self.thread is not None and self.thread.is_alive(),
                    'stream_error': str(self.error) if self.error else None}

    def set_mode(self, name: str, enabled: bool):
        if name not in self.modes:
            raise KeyError(name)
        with self.mode_lock:
            self.modes[name] = enabled
            self.mode_errors.pop(name, None)
        return self.status()

    def disable_failed_mode(self, name: str, exc: Exception):
        with self.mode_lock:
            self.modes[name] = False
            self.mode_errors[name] = str(exc)

    def start(self):
        self.thread = threading.Thread(target=self._run, name='capture-inference', daemon=True)
        self.thread.start()

    def stop(self):
        self.stopped.set()
        with self.condition:
            self.condition.notify_all()
        if self.thread is not None:
            self.thread.join(timeout=5)

    def _run(self):
        try:
            self._capture_loop()
        except Exception as exc:
            self.error = exc
            LOG.exception('Capture/inference thread stopped')
            with self.condition:
                self.condition.notify_all()

    def _capture_loop(self):
        model_specs = {
            'depth': (DEPTH_HEF_PATH, depth_overlay),
            'segmentation': (SEG_HEF_PATH, segmentation_overlay),
            'detection': (DET_HEF_PATH, detection_overlay),
        }
        # Models are lazily loaded on their first activation. A shared VDevice
        # stays alive until shutdown; individual Hailo networks activate only
        # inside model.infer(), so multiple modes may be used sequentially.
        with ExitStack() as resources:
            device = None
            loaded_models = {}

            self.camera = Picamera2()
            self.camera.configure(self.camera.create_video_configuration(
                main={'size': CAMERA_SIZE, 'format': 'RGB888'}
            ))
            self.camera.start()
            try:
                self.camera.set_controls({'AfMode': controls.AfModeEnum.Continuous})
                # A fixed pause isn't necessary; streaming begins immediately.
                while not self.stopped.is_set():
                    start = time.monotonic()
                    frame = self.camera.capture_array()
                    frame = cv2.rotate(frame, cv2.ROTATE_180)
                    # Picamera2 RGB888 capture_array is BGR ordered for OpenCV.
                    # Do not swap channels here; JPEG encoding expects BGR.
                    source_frame = frame.copy()

                    # Always infer against the same unmodified camera frame.
                    # Rendering depth last would erase segmentation/detection.
                    results = {}
                    with self.mode_lock:
                        active_modes = self.modes.copy()
                    for name, active in active_modes.items():
                        if not active:
                            continue
                        try:
                            if name not in loaded_models:
                                if device is None:
                                    from hailo_platform import VDevice
                                    device = resources.enter_context(VDevice())
                                loaded_models[name] = resources.enter_context(
                                    HailoModel(model_specs[name][0], device)
                                )
                                LOG.info('Loaded %s model', name)
                            results[name] = loaded_models[name].infer(source_frame)
                        except Exception as exc:
                            LOG.exception('Disabling %s after model error', name)
                            self.disable_failed_mode(name, exc)

                    # Construct the background first. Segmentation and detection
                    # are then painted over it, never underneath it.
                    frame = (depth_overlay(source_frame.copy(), results['depth'])
                             if 'depth' in results else source_frame.copy())

                    # When depth is side-by-side, both panels have the original
                    # dimensions. Render the same instance masks/boxes on each.
                    panels = ([frame[:, :source_frame.shape[1]],
                               frame[:, source_frame.shape[1]:]]
                              if 'depth' in results and DEPTH_MODE == 'side_by_side'
                              else [frame])
                    for panel in panels:
                        if 'segmentation' in results:
                            segmentation_overlay(panel, results['segmentation'])
                        if 'detection' in results:
                            detection_overlay(panel, results['detection'])

                    success, buffer = cv2.imencode(
                        '.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY]
                    )
                    if success:
                        with self.condition:
                            self.jpeg = buffer.tobytes()
                            self.sequence += 1
                            self.condition.notify_all()
                    remaining = 1 / MAX_STREAM_FPS - (time.monotonic() - start)
                    if remaining > 0:
                        self.stopped.wait(remaining)
            finally:
                self.camera.stop()
                self.camera.close()
                self.camera = None

    def frames(self):
        last_seen = -1
        while not self.stopped.is_set():
            with self.condition:
                self.condition.wait_for(
                    lambda: self.sequence != last_seen or self.error is not None
                    or self.stopped.is_set(), timeout=5
                )
                if self.error is not None or self.stopped.is_set():
                    break
                if self.jpeg is None or self.sequence == last_seen:
                    continue
                last_seen, jpeg = self.sequence, self.jpeg
            yield (b'--frame\r\nContent-Type: image/jpeg\r\n'
                   + b'Content-Length: ' + str(len(jpeg)).encode() + b'\r\n\r\n'
                   + jpeg + b'\r\n')


stream = CameraStream()


@app.route('/')
def index():
    return r'''<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width,initial-scale=1" />
  <title>CubeBot Camera</title>
  <style>
    body { margin:0; background:#101318; color:#f1f5f9;
           font-family:system-ui,-apple-system,Segoe UI,sans-serif; }
    header { display:flex; flex-wrap:wrap; align-items:center; gap:16px;
             padding:16px 20px; background:#1b2029; }
    h1 { font-size:19px; font-weight:600; margin:0 12px 0 0; }
    .controls { display:flex; flex-wrap:wrap; gap:10px; }
    .mode { border:1px solid #586174; border-radius:8px; padding:10px 18px;
            cursor:pointer; background:#303846; color:#fff; font-size:14px;
            font-weight:600; transition:background .12s; }
    .mode[aria-pressed="true"] { background:#147d55; border-color:#1ec78b; }
    .mode:disabled { opacity:.55; cursor:wait; }
    .mode:focus-visible { outline:3px solid #9bd6ff; outline-offset:2px; }
    #status { font-size:13px; color:#c4cad4; padding:10px 20px 0; min-height:20px; }
    main { display:flex; justify-content:center; padding:12px; }
    #video { display:block; max-width:100%; height:auto; object-fit:contain; }
  </style>
</head>
<body>
  <header>
    <h1>CubeBot Camera</h1>
    <div class="controls">
      <button class="mode" type="button" data-mode="depth" aria-pressed="false">Depth: OFF</button>
      <button class="mode" type="button" data-mode="segmentation" aria-pressed="false">Segmentation: OFF</button>
      <button class="mode" type="button" data-mode="detection" aria-pressed="false">Detection: OFF</button>
    </div>
  </header>
  <div id="status" role="status" aria-live="polite"></div>
  <main><img id="video" src="/video" alt="Live camera stream" /></main>
  <script>
    const buttons = [...document.querySelectorAll('.mode')];
    const statusText = document.getElementById('status');
    const labels = {depth:'Depth', segmentation:'Segmentation', detection:'Detection'};

    function updateUI(data) {
      for (const button of buttons) {
        const name = button.dataset.mode;
        const active = !!data.modes[name];
        button.setAttribute('aria-pressed', String(active));
        button.textContent = `${labels[name]}: ${active ? 'ON' : 'OFF'}`;
      }
      const errors = Object.entries(data.errors || {}).map(([name, error]) => `${labels[name]}: ${error}`);
      statusText.textContent = data.stream_error || errors.join(' | ') || '';
    }

    async function refresh() {
      try {
        const response = await fetch('/api/modes', {cache:'no-store'});
        if (!response.ok) throw new Error(`HTTP ${response.status}`);
        updateUI(await response.json());
      } catch (err) {
        statusText.textContent = `Failed to read modes: ${err.message}`;
      }
    }

    for (const button of buttons) {
      button.addEventListener('click', async () => {
        const name = button.dataset.mode;
        const enabled = button.getAttribute('aria-pressed') !== 'true';
        button.disabled = true;
        try {
          const response = await fetch(`/api/modes/${name}`, {
            method:'PUT', headers:{'Content-Type':'application/json'},
            body:JSON.stringify({enabled})
          });
          const data = await response.json();
          if (!response.ok) throw new Error(data.error || `HTTP ${response.status}`);
          updateUI(data);
        } catch (err) {
          statusText.textContent = `Could not update ${labels[name]}: ${err.message}`;
        } finally {
          button.disabled = false;
        }
      });
    }
    refresh();
    // Reconcile toggles with the worker (e.g. failed model loading).
    setInterval(() => { if (!buttons.some(b => b.disabled)) refresh(); }, 2000);
  </script>
</body>
</html>'''


@app.route('/api/modes', methods=['GET'])
def get_modes():
    return jsonify(stream.status())


@app.route('/api/modes/<name>', methods=['PUT'])
def put_mode(name):
    if name not in stream.modes:
        return jsonify({'error': 'Unknown mode'}), 404
    data = request.get_json(silent=True)
    if not isinstance(data, dict) or type(data.get('enabled')) is not bool:
        return jsonify({'error': 'Expected JSON: {"enabled": true|false}'}), 400
    return jsonify(stream.set_mode(name, data['enabled']))


@app.route('/video')
def video():
    return Response(stream.frames(), mimetype='multipart/x-mixed-replace; boundary=frame',
                    headers={'Cache-Control': 'no-store, no-cache, must-revalidate',
                             'X-Accel-Buffering': 'no'})


if __name__ == '__main__':
    try:
        stream.start()
        app.run(host='0.0.0.0', port=5000, threaded=True, use_reloader=False)
    finally:
        stream.stop()
