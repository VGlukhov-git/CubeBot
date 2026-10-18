import argparse
import json
import threading
import time
from contextlib import contextmanager
from pathlib import Path

import cv2
import mediapipe as mp
import numpy as np
import onnxruntime as ort
from flask import Flask, Response, jsonify
from mediapipe.tasks import python
from mediapipe.tasks.python import vision

PROJECT_DIR = Path(__file__).resolve().parent
MODEL_PATH = PROJECT_DIR / "models/hand_landmarker.task"

from control.controller import RobotController
from devices.wt901 import Wt901



controller = RobotController()
controller.set_initial_position()

target_x = 1280/2
target_y = 720/2

target_pitch = 0
target_yaw = 0

delta_box = 10
    
class GestureClassifier:
    def __init__(self, model_path, classes_path):
        mapping = json.loads(Path(classes_path).read_text(encoding="utf-8"))
        if not isinstance(mapping, dict) or not mapping:
            raise ValueError("Class mapping must be a nonempty object")
        if set(mapping) != {str(i) for i in range(len(mapping))}:
            raise ValueError("Class mapping must have consecutive indices starting at 0")
        if not all(isinstance(name, str) and name for name in mapping.values()):
            raise ValueError("Class names must be nonempty strings")
        self.labels = [Path(mapping[str(i)]).stem for i in range(len(mapping))]
        self.session = ort.InferenceSession(str(model_path), providers=["CPUExecutionProvider"])
        inputs = self.session.get_inputs()
        outputs = self.session.get_outputs()
        if len(inputs) != 1 or inputs[0].type != "tensor(float)" or inputs[0].shape[1:] != [21, 3]:
            raise ValueError("Expected one float32 model input shaped [batch, 21, 3]")
        if len(outputs) != 1 or outputs[0].shape[1:] != [len(self.labels)]:
            raise ValueError("Model output does not match the class mapping")
        self.input_name = inputs[0].name

    def predict(self, keypoints):
        data = np.asarray(keypoints, dtype=np.float32)
        if data.shape != (21, 3) or not np.isfinite(data).all():
            raise ValueError("Expected 21 finite [x, y, z] keypoints")
        scores = self.session.run(None, {self.input_name: data[None, ...]})[0][0]
        index = int(np.argmax(scores))
        return self.labels[index], float(scores[index])


def left_hand_keypoints(results):
    """Return one left hand as 21 [x, y, z] triples, or None if absent."""
    candidates = [
        (categories[0].score, landmarks)
        for landmarks, categories in zip(results.hand_landmarks, results.handedness)
        if categories and categories[0].category_name == "Left"
    ]
    if not candidates:
        return None
    # Keep one hand per frame, even if multiple detections are labelled Left.
    _, landmarks = max(candidates, key=lambda candidate: candidate[0])
    return [[float(point.x), float(point.y), float(point.z)] for point in landmarks]



@contextmanager
def open_camera(args):
    if args.source == "picamera2":
        from picamera2 import Picamera2

        camera = Picamera2(args.camera)
        try:
            # Picamera2 RGB888 arrays are BGR, matching OpenCV.
            camera.configure(camera.create_video_configuration(
                main={"size": (args.width, args.height), "format": "RGB888"},
                controls={"FrameRate": args.fps},
            ))
            camera.start()
            yield camera.capture_array
        finally:
            camera.close()
    else:
        camera = cv2.VideoCapture(args.camera, cv2.CAP_V4L2)
        try:
            if not camera.isOpened():
                raise RuntimeError(f"Cannot open USB camera /dev/video{args.camera}")
            camera.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
            camera.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
            camera.set(cv2.CAP_PROP_FPS, args.fps)
            camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)

            def read():
                success, frame = camera.read()
                if not success:
                    raise RuntimeError("Cannot read USB camera frame")
                return frame

            yield read
        finally:
            camera.release()

last_label = None
last_score = None
last_x = None
last_y = None

def annotate(frame, hands, classifier, timestamp, fps):
    global last_label, last_score, last_x, last_y, controller, target_pitch, target_yaw, target_x, target_y
    delta_box = 10
    # Preserve the exact mirrored, unnormalized training coordinates.
    frame = cv2.flip(frame, 1)
    image = mp.Image(image_format=mp.ImageFormat.SRGB,
                     data=cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
    keypoints = left_hand_keypoints(hands.detect_for_video(image, timestamp))
    status = "Show your left hand"
    if keypoints is None:
        last_label = None
        last_score = None
        last_x = None
        last_y = None
    if keypoints is not None:
        label, score = classifier.predict(keypoints)
        status = f"{label} | score: {score:.3f}"
        last_score = score
        height, width = frame.shape[:2]
        points = [(int(x * width), int(y * height)) for x, y, _ in keypoints]

        

        # cx, cy = last_x, last_y
        if label == 'gesture_4' and score > 0.9:
            last_label = label
            cx = points[8][0]
            cy = points[8][1]
            print(f"Center: x={cx}, y={cy}", target_y)
            delta_y = abs(cy - target_y) / (35*2)
            delta_x = abs(cx - target_x) / (35*2)
            
            if (cy > target_y - delta_box):
                target_pitch += delta_y
            if (cy < target_y + delta_box):
                target_pitch -= delta_y
            if (cx > target_x - delta_box):
                target_yaw -= delta_x
            if (cx < target_x + delta_box):
                target_yaw += delta_x

            controller.body.set_rotation(
                    roll=0,
                    pitch=target_pitch,
                    yaw=target_yaw,
                )
        if label == 'gesture_2' and label != last_label and score > 0.9:
            print(controller.isRunning)
            if controller.isRunning is not True:
                last_label = label
                controller.set_bottom_position()
        if label == 'gesture_3' and label != last_label and score > 0.9:
            if controller.isRunning is not True:
                last_label = label
                controller.move_to_initial_position()
        for connection in vision.HandLandmarksConnections.HAND_CONNECTIONS:
            cv2.line(frame, points[connection.start], points[connection.end],
                     (0, 255, 0), 2, cv2.LINE_AA)
        for index, point in enumerate(points):
            cv2.circle(frame, point, 3, (0, 0, 255), -1, cv2.LINE_AA)
            cv2.putText(frame, str(index), (point[0] + 4, point[1] - 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0, 255, 255), 1, cv2.LINE_AA)
    for text, y in ((status, 30), (f"FPS: {fps:.1f}", 55)):
        cv2.putText(frame, text, (10, y), cv2.FONT_HERSHEY_SIMPLEX,
                    0.6, (0, 255, 255), 2, cv2.LINE_AA)
    return frame


class VideoStream:
    """One producer; clients receive the latest JPEG without queuing old frames."""

    def __init__(self, args):
        self.args = args
        self.condition = threading.Condition()
        self.stop_event = threading.Event()
        self.jpeg = None
        self.sequence = 0
        self.error = None
        self.thread = threading.Thread(target=self.run, daemon=True)

    def run(self):
        try:
            args = self.args
            classifier = GestureClassifier(args.model, args.classes)
            options = vision.HandLandmarkerOptions(
                base_options=python.BaseOptions(model_asset_path=str(args.landmarker)),
                running_mode=vision.RunningMode.VIDEO,
                num_hands=2,
                min_hand_detection_confidence=0.5,
                min_hand_presence_confidence=0.5,
                min_tracking_confidence=0.5,
            )
            with open_camera(args) as read, vision.HandLandmarker.create_from_options(options) as hands:
                previous_timestamp = -1
                previous_time = time.monotonic()
                while not self.stop_event.is_set():
                    started = time.monotonic()
                    frame = read()
                    now = time.monotonic()
                    fps = 1 / max(now - previous_time, 0.0001)
                    previous_time = now
                    timestamp = max(time.monotonic_ns() // 1_000_000, previous_timestamp + 1)
                    previous_timestamp = timestamp
                    frame = cv2.rotate(frame, cv2.ROTATE_180)
                    frame = annotate(frame, hands, classifier, timestamp, fps)
                    success, encoded = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
                    if not success:
                        raise RuntimeError("JPEG encoding failed")
                    with self.condition:
                        self.jpeg = encoded.tobytes()
                        self.sequence += 1
                        self.condition.notify_all()
                    self.stop_event.wait(max(0, 1 / args.fps - (time.monotonic() - started)))
        except Exception as error:
            with self.condition:
                self.error = str(error)
            print(f"Video stream failed: {error}", flush=True)
        finally:
            self.stop_event.set()
            with self.condition:
                self.condition.notify_all()

    def frames(self):
        sequence = -1
        while True:
            with self.condition:
                self.condition.wait_for(
                    lambda: self.stop_event.is_set() or
                    (self.jpeg is not None and self.sequence != sequence)
                )
                if self.stop_event.is_set():
                    return
                jpeg, sequence = self.jpeg, self.sequence
            yield (b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: " +
                   str(len(jpeg)).encode() + b"\r\n\r\n" + jpeg + b"\r\n")

    def close(self):
        self.stop_event.set()
        with self.condition:
            self.condition.notify_all()
        self.thread.join(timeout=5)


def create_app(stream):
    app = Flask(__name__)

    @app.get("/")
    def index():
        return """<!doctype html><html><head><title>Pi Gesture Demo</title>
        <meta name="viewport" content="width=device-width, initial-scale=1">
        <style>body{background:#171717;color:white;font-family:sans-serif;text-align:center}
        img{max-width:100%;height:auto}</style></head><body>
        <h1>Left-hand gesture demo</h1><p id="status">Show your left hand to the camera.</p>
        <img src="/video_feed" alt="Live keypoints and gesture predictions">
        <script>setInterval(async()=>{try{const r=await fetch('/health');const s=await r.json();
        document.getElementById('status').textContent=s.error ||
        (s.ready?'Show your left hand to the camera.':'Starting camera...');
        }catch(e){document.getElementById('status').textContent='Server disconnected';}},2000);
        </script></body></html>"""

    @app.get("/video_feed")
    def video_feed():
        if stream.stop_event.is_set():
            return jsonify(error=stream.error or "Stream stopped"), 503
        return Response(stream.frames(), mimetype="multipart/x-mixed-replace; boundary=frame",
                        headers={"Cache-Control": "no-store"})

    @app.get("/health")
    def health():
        with stream.condition:
            stopped = stream.stop_event.is_set()
            return jsonify(ready=stream.jpeg is not None and not stopped,
                           error=stream.error or ("Stream stopped" if stopped else None)), 503 if stopped else 200

    return app


def positive_int(value):
    number = int(value)
    if number < 1:
        raise argparse.ArgumentTypeError("must be positive")
    return number


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", choices=("picamera2", "usb"), default="picamera2")
    parser.add_argument("--camera", type=int, default=0)
    parser.add_argument("--model", type=Path, default=PROJECT_DIR / "models/gesture_model.onnx")
    parser.add_argument("--classes", type=Path)
    parser.add_argument("--landmarker", type=Path, default=MODEL_PATH)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=positive_int, default=5000)
    parser.add_argument("--width", type=positive_int, default=1080)
    parser.add_argument("--height", type=positive_int, default=720)
    parser.add_argument("--fps", type=positive_int, default=24)
    args = parser.parse_args()
    if args.port > 65535 or args.camera < 0:
        parser.error("Port must be at most 65535 and camera index must be nonnegative")
    args.classes = args.classes or args.model.with_suffix(".classes.json")
    for path in (args.model, args.classes, args.landmarker):
        if not path.is_file():
            parser.error(f"File not found: {path}")
    stream = VideoStream(args)
    stream.thread.start()
    try:
        create_app(stream).run(host=args.host, port=args.port, threaded=True, use_reloader=False)
    finally:
        stream.close()


controller_thread = threading.Thread(
    target=controller.run,
    daemon=True,
)

# controller_runner_thread = threading.Thread(
#     target=robot_runner,
#     daemon=True,
# )

# controller_thread.start()
# controller_runner_thread.start()



if __name__ == "__main__":
    main()
