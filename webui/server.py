"""aiohttp web server: static UI, MJPEG streams, REST settings API, WebSocket control."""
import asyncio
import json
import logging
import os
import time
import uuid

import cv2
import numpy as np
from aiohttp import web, WSMsgType

from . import settings as S
from .procman import SubscriberProcess
from .recorder import list_runs

log = logging.getLogger(__name__)

STATIC = os.path.join(os.path.dirname(os.path.abspath(__file__)), "static")
STREAM_FPS = 15
TELEMETRY_HZ = 10
STREAMS = ("raw", "lane", "light")


def _placeholder(text):
    img = np.full((360, 640, 3), 40, np.uint8)
    cv2.putText(img, text, (40, 190), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (200, 200, 200), 2, cv2.LINE_AA)
    return cv2.imencode(".jpg", img)[1].tobytes()


class App:
    def __init__(self, car, settings, bus, log_dir):
        self.car, self.settings, self.bus = car, settings, bus
        self.log_dir = log_dir
        self.subscriber = SubscriberProcess(log_dir)
        self.clients = {}            # cid -> WebSocketResponse
        self.jpeg = {}               # stream name -> (frame_seq, bytes)
        self.pwm_cfg = S.load_pwm_config()
        self.placeholders = {
            "raw": _placeholder("No camera frames"),
            "lane": _placeholder("Vision preview is off"),
            "light": _placeholder("Traffic light detection is off"),
        }

    # ------------------------------------------------------------------ setup
    def build(self):
        app = web.Application(middlewares=[self._no_cache])
        app.add_routes([
            web.get("/", self.index),
            web.get("/ws", self.ws),
            web.get("/stream/{name}", self.stream),
            web.get("/api/state", self.get_state),
            web.post("/api/settings", self.post_settings),
            web.post("/api/preset", self.post_preset),
            web.get("/api/pwm", self.get_pwm),
            web.post("/api/pwm", self.post_pwm),
            web.post("/api/sample", self.post_sample),
            web.get("/api/logs", self.get_logs),
            web.delete("/api/logs/{name}", self.delete_log),
            web.get("/logs/{file}", self.download_log),
            web.get("/api/subscriber", self.get_subscriber),
            web.post("/api/subscriber", self.post_subscriber),
            web.static("/static", STATIC),
        ])
        app.on_startup.append(self._on_startup)
        app.on_shutdown.append(self._on_shutdown)
        return app

    @web.middleware
    async def _no_cache(self, request, handler):
        resp = await handler(request)
        if request.path == "/" or request.path.startswith("/static"):
            resp.headers["Cache-Control"] = "no-store"
        return resp

    async def _on_startup(self, app):
        self._telemetry_task = asyncio.create_task(self._telemetry_loop())

    async def _on_shutdown(self, app):
        self._telemetry_task.cancel()
        for ws in list(self.clients.values()):
            await ws.close()

    # ------------------------------------------------------------------ pages / streams
    async def index(self, request):
        return web.FileResponse(os.path.join(STATIC, "index.html"))

    async def _jpeg(self, name):
        seq = self.car.frame_seq
        cached = self.jpeg.get(name)
        if cached and cached[0] == seq:
            return cached[1]
        frame = self.car.frames.get(name)
        if frame is None:
            data = self.placeholders[name]
        else:
            loop = asyncio.get_running_loop()
            data = await loop.run_in_executor(
                None, lambda: cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 70])[1].tobytes())
        self.jpeg[name] = (seq, data)
        return data

    async def stream(self, request):
        name = request.match_info["name"]
        if name not in STREAMS:
            raise web.HTTPNotFound()
        resp = web.StreamResponse(headers={
            "Content-Type": "multipart/x-mixed-replace; boundary=frame",
            "Cache-Control": "no-store",
        })
        await resp.prepare(request)
        last_seq, last_sent = -1, 0.0
        try:
            while True:
                seq = self.car.frame_seq
                # Re-send at least once a second so placeholders/stalls still refresh
                if seq != last_seq or time.time() - last_sent > 1.0:
                    data = await self._jpeg(name)
                    await resp.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                                     + str(len(data)).encode() + b"\r\n\r\n" + data + b"\r\n")
                    last_seq, last_sent = seq, time.time()
                await asyncio.sleep(1.0 / STREAM_FPS)
        except (ConnectionResetError, asyncio.CancelledError):
            pass
        return resp

    # ------------------------------------------------------------------ REST
    def _state(self):
        return {
            "settings": self.settings.snapshot(),
            "presets": {k: {"label": v["label"], "script": v["script"]} for k, v in S.PRESETS.items()},
        }

    async def get_state(self, request):
        return web.json_response(self._state())

    async def _broadcast_settings(self):
        msg = json.dumps({"type": "settings", "settings": self.settings.snapshot()})
        for ws in list(self.clients.values()):
            try:
                await ws.send_str(msg)
            except Exception:
                pass

    async def post_settings(self, request):
        try:
            cfg = self.settings.update(await request.json())
        except (ValueError, TypeError) as e:
            raise web.HTTPBadRequest(text=str(e))
        self.car.settings_changed(cfg)
        await self._broadcast_settings()
        return web.json_response(cfg)

    async def post_preset(self, request):
        name = (await request.json()).get("name")
        if name not in S.PRESETS:
            raise web.HTTPBadRequest(text="unknown preset")
        cfg = self.settings.apply_preset(name)
        self.car.settings_changed(cfg)
        await self._broadcast_settings()
        return web.json_response(cfg)

    async def get_pwm(self, request):
        return web.json_response({"config": self.pwm_cfg, "saved": S.load_pwm_config(),
                                  "defaults": S.PWM_DEFAULTS, "path": S.PWM_CONFIG_PATH})

    async def post_pwm(self, request):
        body = await request.json()
        try:
            cfg = S.validate_pwm_config(body["config"])
        except (KeyError, ValueError, TypeError) as e:
            raise web.HTTPBadRequest(text=str(e))
        self.pwm_cfg = cfg
        live = self.bus.publish_config(cfg)
        if body.get("save"):
            try:
                S.save_pwm_config(cfg)
            except OSError as e:
                raise web.HTTPInternalServerError(text=f"could not save: {e}")
        return web.json_response({"config": cfg, "live": live, "saved": S.load_pwm_config()})

    async def post_sample(self, request):
        b = await request.json()
        res = self.car.sample_hsv(b.get("x", 0), b.get("y", 0), b.get("size", 20))
        if res is None:
            raise web.HTTPServiceUnavailable(text="no camera frame")
        return web.json_response(res)

    async def get_logs(self, request):
        return web.json_response({"runs": list_runs(self.log_dir), "dir": self.log_dir,
                                  "recording": self.car.recorder.status()})

    def _log_file(self, fn):
        if "/" in fn or "\\" in fn or fn.startswith(".") or not fn.startswith("run_"):
            raise web.HTTPNotFound()
        return os.path.join(self.log_dir, fn)

    async def download_log(self, request):
        path = self._log_file(request.match_info["file"])
        if not os.path.isfile(path):
            raise web.HTTPNotFound()
        return web.FileResponse(path, headers={
            "Content-Disposition": f'attachment; filename="{os.path.basename(path)}"'})

    async def delete_log(self, request):
        name = request.match_info["name"]
        rec = self.car.recorder
        if rec.active and rec.name == name:
            raise web.HTTPConflict(text="run is still recording")
        for ext in (".csv", ".mp4"):
            path = self._log_file(name + ext)
            if os.path.isfile(path):
                os.remove(path)
        return web.json_response({"ok": True})

    async def get_subscriber(self, request):
        return web.json_response({"pid": self.subscriber.pid(), "log": self.subscriber.tail(),
                                  "bus": self.bus.status()})

    async def post_subscriber(self, request):
        action = (await request.json()).get("action")
        loop = asyncio.get_running_loop()
        try:
            if action == "start":
                await loop.run_in_executor(None, self.subscriber.start)
            elif action == "stop":
                self.car.stop()
                await loop.run_in_executor(None, self.subscriber.stop)
            else:
                raise web.HTTPBadRequest(text="unknown action")
        except OSError as e:
            raise web.HTTPInternalServerError(text=str(e))
        return web.json_response({"pid": self.subscriber.pid()})

    # ------------------------------------------------------------------ WebSocket
    async def ws(self, request):
        ws = web.WebSocketResponse(heartbeat=5.0)
        await ws.prepare(request)
        cid = uuid.uuid4().hex[:8]
        self.clients[cid] = ws
        await ws.send_str(json.dumps({"type": "hello", "id": cid}))
        log.info("client %s connected (%s)", cid, request.remote)
        try:
            async for msg in ws:
                if msg.type != WSMsgType.TEXT:
                    continue
                try:
                    data = json.loads(msg.data)
                    err = self._handle(cid, data)
                except Exception as e:
                    err = f"bad message: {e}"
                if err:
                    await ws.send_str(json.dumps({"type": "error", "msg": err}))
        finally:
            self.clients.pop(cid, None)
            self.car.client_gone(cid)
            log.info("client %s disconnected", cid)
        return ws

    def _handle(self, cid, d):
        car, t = self.car, d.get("t")
        if t == "drive":
            car.drive(cid, d.get("steer", 0.0), d.get("throttle", 0.0), d.get("raw", False))
        elif t == "engage":
            return car.engage_manual(cid)
        elif t == "release":
            car.release(cid)
        elif t == "auto":
            return car.start_auto()
        elif t == "stop":
            car.stop()
        elif t == "estop":
            car.trigger_estop(d.get("source", "UI"))
        elif t == "reset_estop":
            car.reset_estop()
        elif t == "vision":
            car.set_vision(cid, bool(d.get("on")))
        elif t == "record":
            car.set_recording(bool(d.get("on")))
        else:
            return f"unknown message type {t!r}"
        return None

    async def _telemetry_loop(self):
        while True:
            await asyncio.sleep(1.0 / TELEMETRY_HZ)
            for cid, ws in list(self.clients.items()):
                try:
                    status = self.car.status(cid)
                    status["type"] = "telemetry"
                    await ws.send_str(json.dumps(status))
                except Exception:
                    pass
