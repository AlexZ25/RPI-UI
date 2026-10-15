"""Camera sources. All return 640x360 frames in the same channel order the
existing code gets from Picamera2 "RGB888" (which is BGR in memory)."""
import logging
import time

import cv2

from . import compat

log = logging.getLogger(__name__)

W, H = 640, 360


class PiCamera:
    name = "picamera2"

    def __init__(self, cam_settings):
        from libcamera import Transform
        from picamera2 import Picamera2
        self.picam2 = Picamera2()
        config = self.picam2.create_video_configuration(
            main={"size": (W, H), "format": "RGB888"},
            transform=Transform(rotation=180) if cam_settings["rotate180"] else Transform(),
            controls={"FrameRate": cam_settings["fps"]},
        )
        self.picam2.configure(config)
        self.apply_controls(cam_settings)
        self.picam2.start()
        time.sleep(0.2)

    def apply_controls(self, cam_settings):
        if cam_settings["auto_exposure"]:
            self.picam2.set_controls({"AeEnable": True})
        else:
            self.picam2.set_controls({
                "AeEnable": False,
                "ExposureTime": int(cam_settings["exposure_us"]),
                "AnalogueGain": float(cam_settings["gain"]),
            })

    def read(self):
        return self.picam2.capture_array()

    def close(self):
        try:
            self.picam2.stop()
            self.picam2.close()
        except Exception:
            pass


class CvCamera:
    """USB webcam (index) or video file (looped). Useful for offline testing."""

    def __init__(self, source, cam_settings):
        self.source = source
        self.name = f"opencv:{source}"
        self.is_file = not str(source).isdigit()
        self.cap = cv2.VideoCapture(source if self.is_file else int(source))
        if not self.cap.isOpened():
            raise RuntimeError(f"cannot open camera/video {source}")
        self.rotate = cam_settings["rotate180"]
        self.period = 1.0 / max(1, cam_settings["fps"])
        self.last = 0.0

    def apply_controls(self, cam_settings):
        self.rotate = cam_settings["rotate180"]

    def read(self):
        if self.is_file:   # pace file playback to the configured fps
            wait = self.period - (time.monotonic() - self.last)
            if wait > 0:
                time.sleep(wait)
            self.last = time.monotonic()
        ok, frame = self.cap.read()
        if not ok and self.is_file:
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            ok, frame = self.cap.read()
        if not ok:
            raise RuntimeError("camera read failed")
        frame = cv2.resize(frame, (W, H))
        return cv2.rotate(frame, cv2.ROTATE_180) if self.rotate and not self.is_file else frame

    def close(self):
        self.cap.release()


class SimCamera:
    name = "simulator"

    def __init__(self, cam_settings, command_source):
        from .sim import SimWorld
        self.world = SimWorld()
        self.command_source = command_source
        self.period = 1.0 / max(1, cam_settings["fps"])
        self.last = time.monotonic()

    def apply_controls(self, cam_settings):
        self.period = 1.0 / max(1, cam_settings["fps"])

    def read(self):
        wait = self.period - (time.monotonic() - self.last)
        if wait > 0:
            time.sleep(wait)
        self.last = time.monotonic()
        self.world.step(self.command_source())
        return self.world.render()

    def close(self):
        pass


def open_camera(spec, cam_settings, command_source):
    """spec: auto | picamera | sim | <webcam index> | <video file path>"""
    if spec in ("auto", "picamera"):
        if compat.HAVE_PICAMERA2:
            return PiCamera(cam_settings)
        if spec == "picamera":
            raise RuntimeError("picamera2 is not available")
        log.warning("picamera2 not available; using the simulator camera")
        return SimCamera(cam_settings, command_source)
    if spec == "sim":
        return SimCamera(cam_settings, command_source)
    return CvCamera(spec, cam_settings)
