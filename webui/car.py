"""Vehicle controller: the single high-level program that publishes to car/cmd.

Modes
  idle    nothing is published (pwm_subscriber falls back to stationary after 0.5 s)
  manual  commands from the one browser client that holds control, at 30 Hz
  auto    commands from the autonomous pipeline, one per camera frame
E-stop latches: it forces idle, sends a burst of explicit stop commands and
blocks every mode until it is reset.
"""
import logging
import threading
import time

import cv2
import numpy as np

from .camera import open_camera, W, H
from .recorder import Recorder

log = logging.getLogger(__name__)

CONTROL_HZ = 30
MANUAL_TIMEOUT_S = 0.3     # browser must keep sending drive messages (deadman)
SUBSCRIBER_TIMEOUT_S = 0.5  # same as pwm_subscriber.timeout_s
ZERO = {"steer": 0.0, "throttle": 0.0, "reverse": False, "rev_power": 0.0}


class Car:
    def __init__(self, settings, bus, camera_spec, log_dir):
        from .pipeline import AutoPipeline   # imports src/ (needs camera stubs installed first)
        self.settings = settings
        self.bus = bus
        self.camera_spec = camera_spec
        self.recorder = Recorder(log_dir)
        self.pipeline = AutoPipeline()

        self.lock = threading.RLock()
        self.mode = "idle"
        self.estop = False
        self.owner = None             # client id holding manual control
        self.manual = {"steer": 0.0, "throttle": 0.0, "raw": False, "ts": 0.0}
        self.stop_burst_until = 0.0
        self.vision_clients = set()   # clients that want the vision pipeline running
        self.message = ""             # last notable event, shown in the UI
        self.message_id = 0

        self.cfg = settings.snapshot()
        self.pipeline_dirty = True
        self.camera_dirty = False
        self.camera_restart = False
        self.record_request = None    # True/False when a start/stop is pending

        self.camera = None
        self.camera_error = None
        self.fps = 0.0
        self.frame_seq = 0
        self.frame_ts = 0.0
        self.frames = {}              # stream name -> ndarray (BGR)
        self.last_raw = None
        self.vision = None            # latest pipeline telemetry
        self.running = False

    # ------------------------------------------------------------------ lifecycle
    def start(self):
        self.running = True
        threading.Thread(target=self._vision_loop, name="vision", daemon=True).start()
        threading.Thread(target=self._control_loop, name="control", daemon=True).start()

    def shutdown(self):
        with self.lock:
            self.mode = "idle"
            for _ in range(5):
                self.bus.publish_cmd(**ZERO)
        self.running = False
        time.sleep(0.2)
        self.recorder.stop()
        if self.camera is not None:
            self.camera.close()

    def settings_changed(self, cfg):
        with self.lock:
            old = self.cfg
            self.cfg = cfg
            self.pipeline_dirty = True
            if cfg["camera"] != old["camera"]:
                if cfg["camera"]["rotate180"] != old["camera"]["rotate180"] or \
                        cfg["camera"]["fps"] != old["camera"]["fps"]:
                    self.camera_restart = True
                else:
                    self.camera_dirty = True

    def _note(self, msg):
        self.message = msg
        self.message_id += 1

    # ------------------------------------------------------------------ mode control
    def _burst_stop(self, seconds):
        self.stop_burst_until = time.time() + seconds

    def engage_manual(self, cid):
        with self.lock:
            if self.estop:
                return "E-stop is active. Reset it first."
            if self.mode == "auto":
                return "Autopilot is running. Stop it first."
            if self.owner not in (None, cid):
                self._note("Manual control taken over by another browser")
            self.mode, self.owner = "manual", cid
            self.manual.update(steer=0.0, throttle=0.0, raw=False, ts=time.time())
            return None

    def release(self, cid=None):
        with self.lock:
            if self.mode == "manual" and (cid is None or self.owner == cid):
                self.mode, self.owner = "idle", None
                self._burst_stop(0.5)

    def drive(self, cid, steer, throttle, raw=False):
        with self.lock:
            if self.mode != "manual" or self.owner != cid:
                return False
            self.manual.update(steer=max(-1.0, min(1.0, float(steer))),
                               throttle=max(-1.0, min(1.0, float(throttle))),
                               raw=bool(raw), ts=time.time())
            return True

    def start_auto(self):
        with self.lock:
            if self.estop:
                return "E-stop is active. Reset it first."
            if self.camera is None:
                return "Camera is not running."
            self.mode, self.owner = "auto", None
            self._note("Autopilot started")
            return None

    def stop(self):
        with self.lock:
            if self.mode != "idle":
                self._note(f"{self.mode.capitalize()} stopped")
            self.mode, self.owner = "idle", None
            self._burst_stop(0.5)

    def trigger_estop(self, source="UI"):
        with self.lock:
            self.estop = True
            self.mode, self.owner = "idle", None
            self._burst_stop(1.5)
            self._note(f"E-STOP ({source})")
            log.warning("E-STOP triggered from %s", source)

    def reset_estop(self):
        with self.lock:
            self.estop = False
            self._note("E-stop reset")

    def client_gone(self, cid):
        with self.lock:
            self.vision_clients.discard(cid)
            if self.mode == "manual" and self.owner == cid:
                self.mode, self.owner = "idle", None
                self._burst_stop(0.5)
                self._note("Controlling browser disconnected; car stopped")

    def set_vision(self, cid, on):
        with self.lock:
            (self.vision_clients.add if on else self.vision_clients.discard)(cid)

    def set_recording(self, on):
        self.record_request = bool(on)

    # ------------------------------------------------------------------ helpers
    def effective_command(self):
        """What pwm_subscriber is currently acting on (used by the simulator)."""
        cmd = self.bus.last_sent
        if time.time() - cmd["ts"] > SUBSCRIBER_TIMEOUT_S:
            return ZERO
        return cmd

    def _manual_to_cmd(self):
        m, mc = self.manual, self.cfg["manual"]
        if m["raw"]:   # calibration tests: values are already in car/cmd convention
            steer, t = m["steer"], m["throttle"]
            return {"steer": steer, "throttle": max(0.0, t), "reverse": t < 0, "rev_power": max(0.0, -t)}
        steer = m["steer"] * mc["steer_limit"] * (-1.0 if mc["invert_steer"] else 1.0)
        t = m["throttle"]
        if t >= 0:
            return {"steer": steer, "throttle": t * mc["max_throttle"], "reverse": False, "rev_power": 0.0}
        return {"steer": steer, "throttle": 0.0, "reverse": True, "rev_power": -t * mc["max_reverse"]}

    def sample_hsv(self, x, y, size=20):
        frame = self.last_raw
        if frame is None:
            return None
        x = int(max(0, min(W - 1, x)))
        y = int(max(0, min(H - 1, y)))
        r = max(1, int(size) // 2)
        patch = frame[max(0, y - r):y + r, max(0, x - r):x + r]
        # Same conversion the src/ estimators use on Picamera2 frames
        hsv = cv2.mean(cv2.cvtColor(patch, cv2.COLOR_BGR2HSV))[:3]
        bgr = cv2.mean(patch)[:3]
        return {"hsv": [int(round(v)) for v in hsv],
                "color": "#%02x%02x%02x" % (int(bgr[2]), int(bgr[1]), int(bgr[0]))}

    # ------------------------------------------------------------------ threads
    def _control_loop(self):
        period = 1.0 / CONTROL_HZ
        while self.running:
            t0 = time.time()
            with self.lock:
                if self.mode == "manual" and not self.estop:
                    if t0 - self.manual["ts"] < MANUAL_TIMEOUT_S:
                        self.bus.publish_cmd(**self._manual_to_cmd())
                    else:
                        self.bus.publish_cmd(**ZERO)   # deadman: browser stopped sending
                elif self.mode == "auto" and t0 - self.frame_ts > 1.0:
                    self.mode = "idle"
                    self._burst_stop(0.5)
                    self._note("Autopilot stopped: no camera frames")
                if self.mode == "idle" and t0 < self.stop_burst_until:
                    self.bus.publish_cmd(**ZERO)
            time.sleep(max(0.0, period - (time.time() - t0)))

    def _open_camera(self):
        try:
            self.camera = open_camera(self.camera_spec, self.cfg["camera"], self.effective_command)
            self.camera_error = None
            log.info("Camera opened: %s", self.camera.name)
        except Exception as e:
            self.camera = None
            self.camera_error = str(e)
            log.error("Camera error: %s", e)

    def _vision_loop(self):
        fps_t, fps_n = time.time(), 0
        while self.running:
            if self.camera_restart and self.camera is not None:
                self.camera_restart = False
                self.camera.close()
                self.camera = None
            if self.camera is None:
                self._open_camera()
                if self.camera is None:
                    time.sleep(2.0)
                    continue
            if self.camera_dirty:
                self.camera_dirty = False
                try:
                    self.camera.apply_controls(self.cfg["camera"])
                except Exception as e:
                    log.error("Camera controls: %s", e)
            try:
                frame = self.camera.read()
            except Exception as e:
                log.error("Camera read failed: %s", e)
                self.camera_error = str(e)
                self.camera.close()
                self.camera = None
                continue

            if self.pipeline_dirty:
                self.pipeline_dirty = False
                self.pipeline.configure(self.cfg["pipeline"])

            with self.lock:
                run_vision = self.mode == "auto" or bool(self.vision_clients)
            telemetry, frames = None, {"raw": frame}
            if run_vision:
                try:
                    cmd, telemetry, out = self.pipeline.process(frame)
                    frames.update({k: v for k, v in out.items() if v is not None})
                    with self.lock:
                        if self.mode == "auto" and not self.estop:
                            self.bus.publish_cmd(steer=cmd["steer"], throttle=cmd["throttle"])
                except Exception as e:
                    log.exception("Pipeline error")
                    with self.lock:
                        if self.mode == "auto":
                            self.mode = "idle"
                            self._burst_stop(0.5)
                        self._note(f"Pipeline error: {e}")

            self.last_raw = frame
            self.frames = frames
            self.vision = telemetry
            self.frame_seq += 1
            self.frame_ts = time.time()
            self._handle_recording(frame, telemetry)

            fps_n += 1
            if time.time() - fps_t >= 1.0:
                self.fps = fps_n / (time.time() - fps_t)
                fps_t, fps_n = time.time(), 0

    def _handle_recording(self, frame, telemetry):
        req, self.record_request = self.record_request, None
        if req is True and not self.recorder.active:
            try:
                self.recorder.start(self.cfg["camera"]["fps"], (frame.shape[1], frame.shape[0]))
                self._note(f"Recording {self.recorder.name}")
            except Exception as e:
                self._note(f"Recording failed: {e}")
        elif req is False and self.recorder.active:
            self.recorder.stop()
            self._note("Recording saved")
        if self.recorder.active:
            cmd = self.effective_command()
            self.recorder.write(frame, self.mode, cmd, telemetry)

    # ------------------------------------------------------------------ status
    def status(self, cid=None):
        with self.lock:
            return {
                "mode": self.mode,
                "estop": self.estop,
                "owner": self.owner,
                "you_control": cid is not None and self.owner == cid,
                "message": self.message,
                "message_id": self.message_id,
                "vision_running": self.mode == "auto" or bool(self.vision_clients),
                "manual_input": {"steer": self.manual["steer"], "throttle": self.manual["throttle"]},
                "camera": {"name": getattr(self.camera, "name", None), "error": self.camera_error,
                           "fps": round(self.fps, 1)},
                "vision": self.vision,
                "recording": self.recorder.status(),
                "bus": self.bus.status(),
            }
