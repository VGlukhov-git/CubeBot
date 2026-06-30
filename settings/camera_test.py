from flask import Flask, Response
from picamera2 import Picamera2
import cv2
import time
from libcamera import controls
app = Flask(__name__)

# DEPTH_HEF_PATH = "/home/vladimir/projects/CubeBot/ai/models/scdepthv3--320x256_quant_hailort_multidevice_1.hef"
# DET_HEF_PATH = "/home/vladimir/projects/CubeBot/ai/models/best_ball_v8n.hef"

camera = Picamera2()
camera.configure(
    camera.create_video_configuration(
        main={"size": (1536, 864), "format": "RGB888"}
    )
)

for mode in camera.sensor_modes:
    print(mode)

camera.start()
camera.set_controls({
    "AfMode": controls.AfModeEnum.Continuous
})
time.sleep(1)


def generate_frames():
    while True:
        frame = camera.capture_array()

        frame = cv2.rotate(frame, cv2.ROTATE_180)
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

        success, buffer = cv2.imencode(".jpg", frame)
        if not success:
            continue

        jpg = buffer.tobytes()

        yield (
            b"--frame\r\n"
            b"Content-Type: image/jpeg\r\n\r\n" + jpg + b"\r\n"
        )


@app.route("/")
def index():
    return """
    <html>
      <body>
        <h1>Raspberry Pi Camera MJPEG Stream</h1>
        <img src="/video">
      </body>
    </html>
    """


@app.route("/video")
def video():
    return Response(
        generate_frames(),
        mimetype="multipart/x-mixed-replace; boundary=frame"
    )


if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000)