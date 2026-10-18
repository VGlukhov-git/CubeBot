
import asyncio
import json
import math
import os
from pathlib import Path

from aiohttp import web
import socketio
from devices.servo_uart import ServoUart

# Frontend build directory
DIST_DIR = Path(__file__).resolve().parent / "dist"

sio = socketio.AsyncServer(async_mode="aiohttp", cors_allowed_origins="*")
app = web.Application()
sio.attach(app)

servo_device = ServoUart()

ROBOT_ID = "cubebot_local"
SETTINGS_FILE = Path(__file__).resolve().parent / "settings.json"

LEG_NAMES = ("front-left", "front-right", "rear-left", "rear-right")
JOINT_NAMES = ("hip", "upperLeg", "knee")

async def serve_frontend(request):
    requested_path = request.match_info.get("path", "")
    file_path = (DIST_DIR / requested_path).resolve()

    # Security: prevent access outside dist
    if not file_path.is_relative_to(DIST_DIR.resolve()):
        raise web.HTTPForbidden()

    # Serve existing files
    if file_path.is_file():
        return web.FileResponse(file_path)

    # Do not return HTML for API or JSON requests
    if (
        requested_path.startswith(("api/", "socket.io/"))
        or Path(requested_path).suffix
    ):
        raise web.HTTPNotFound()

    # SPA fallback for frontend routes
    index_file = DIST_DIR / "index.html"

    if index_file.is_file():
        return web.FileResponse(index_file)

    raise web.HTTPNotFound()


app.router.add_get("/", serve_frontend)
app.router.add_get("/{path:.*}", serve_frontend)

def load_settings():
    with SETTINGS_FILE.open("r", encoding="utf-8") as file:
        return json.load(file)


def save_settings(settings):
    # Atomic replacement avoids leaving a partially written JSON file.
    temp_file = SETTINGS_FILE.with_suffix(".json.tmp")

    try:
        with temp_file.open("w", encoding="utf-8") as file:
            json.dump(settings, file, indent=2)
            file.write("\n")

        os.replace(temp_file, SETTINGS_FILE)
    finally:
        temp_file.unlink(missing_ok=True)


def validate_mapper(mapper):
    if not isinstance(mapper, dict) or set(mapper) != set(LEG_NAMES):
        return False

    channels = []

    for leg in LEG_NAMES:
        joints = mapper.get(leg)

        if not isinstance(joints, dict) or set(joints) != set(JOINT_NAMES):
            return False

        for joint in JOINT_NAMES:
            channel = joints[joint]

            if type(channel) is not int or not 0 <= channel < 12:
                return False

            channels.append(channel)

    # Each physical servo must have a unique channel.
    return len(set(channels)) == 12


@sio.event
async def connect(sid, environ, auth=None):
    print(f"Client connected: {sid}", flush=True)


@sio.event
async def disconnect(sid, reason):
    print(f"Client disconnected: {sid} ({reason})", flush=True)


@sio.event
async def message(sid, data):
    if not isinstance(data, dict) or data.get("robotId") != ROBOT_ID:
        return {"ok": False, "error": "Invalid robot ID or message"}

    message_type = data.get("type")

    if message_type == "settings":
        action = data.get("value")

        if action == "load":
            try:
                settings = await asyncio.to_thread(load_settings)

                await sio.emit(
                    "message",
                    {
                        "robotId": ROBOT_ID,
                        "type": "setting_loaded",
                        "value": settings,
                    },
                    to=sid,
                )

                return {"ok": True}

            except (OSError, ValueError) as error:
                print(f"Settings load error: {error}", flush=True)
                return {"ok": False, "error": str(error)}

        if action == "save":
            mapper = data.get("mapper")

            if not validate_mapper(mapper):
                return {"ok": False, "error": "Invalid mapper"}

            settings = {"mapper": mapper}

            try:
                await asyncio.to_thread(save_settings, settings)

                await sio.emit(
                    "message",
                    {
                        "robotId": ROBOT_ID,
                        "type": "setting_saved",
                        "value": True,
                    },
                    to=sid,
                )

                return {"ok": True}

            except OSError as error:
                print(f"Settings save error: {error}", flush=True)
                return {"ok": False, "error": str(error)}

        return {"ok": False, "error": "Unknown settings action"}

    if (
        type(data.get("joint")) is not int
        or not 0 <= data["joint"] < 12
        or type(data.get("value")) not in (int, float)
        or not math.isfinite(data["value"])
        or not 0 <= data["value"] <= 180
    ):
        print(f"Rejected invalid joint message from {sid}", flush=True)
        return {"ok": False, "error": "Invalid joint message"}

    print(json.dumps(data), flush=True)
    servo_device.send_message(data["joint"], data["value"])

    return {"ok": True}


if __name__ == "__main__":
    web.run_app(
        app,
        host="127.0.0.1",
        port=int(os.environ.get("PORT", "5057")),
    )
