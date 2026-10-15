"""Autonomous pipeline built from the existing src/ modules.

Per frame it does what script/lane_dist_light_ctrl.py (and its siblings) do:
  lane estimate -> Stanley steering -> constant throttle
  -> scale throttle by nearest ArUco distance -> stop on red/yellow light.
Each stage can be switched on/off and tuned live from the UI.
"""
import contextlib
import io
import math
import time

import cv2
import numpy as np

from src.aruco_dist import ArucoDistanceEstimator
from src.controllers import StanleyController
from src.single_lane import SingleLaneEstimator
from src.traffic_light import TrafficLightDetector
from src.two_lane_poly import TwoLaneEstimator

W, H = 640, 360


def _arr(v):
    return np.array(v, dtype=np.int32)


class AutoPipeline:
    def __init__(self):
        self.params = None
        self.lane_type = None
        self.lane = None
        self.controller = StanleyController()
        self.aruco = None
        self.light = None

    def configure(self, p):
        """Apply a pipeline settings dict. Detector state is kept when possible."""
        if self.lane is None or p["lane"] != self.lane_type:
            self.lane = SingleLaneEstimator(W, H) if p["lane"] == "single" else TwoLaneEstimator(W, H)
            self.lane_type = p["lane"]
        if p["lane"] == "single":
            s = p["single"]
            self.lane.lower_red1, self.lane.upper_red1 = _arr(s["red1_lo"]), _arr(s["red1_hi"])
            self.lane.lower_red2, self.lane.upper_red2 = _arr(s["red2_lo"]), _arr(s["red2_hi"])
        else:
            s = p["two"]
            self.lane.lower_yellow, self.lane.upper_yellow = _arr(s["yellow_lo"]), _arr(s["yellow_hi"])
            self.lane.lower_white, self.lane.upper_white = _arr(s["white_lo"]), _arr(s["white_hi"])
        self.lane.area_min = s["area_min"]
        self.lane.px_to_m = s["px_to_m"]

        st = p["stanley"]
        self.controller.k_e = st["k_e"]
        self.controller.k_v = st["k_v"]
        self.controller.wheelbase = st["wheelbase"]
        self.controller.ff_gain = st["ff_gain"]
        self.controller.max_steer_rad = math.radians(st["max_steer_deg"])

        dist, light = p["distance"], p["light"]
        marker_id = int(light["marker_id"])
        keep = int(dist["keep_missed_frames"])
        if self.aruco is None or self.aruco.keep_missed_frames != keep:
            self.aruco = ArucoDistanceEstimator(keep_missed_frames=keep)
        self.aruco.reserved_ids = [marker_id]   # the traffic light marker is not an obstacle

        if self.light is None or self.light.marker_id != marker_id:
            self.light = TrafficLightDetector(marker_id=marker_id, enlarge_scale=2.0)
        self.light.hold_seconds = float(light["hold_seconds"])
        lh = p["light_hsv"]
        self.light.LOWER_RED, self.light.UPPER_RED = _arr(lh["red_lo"]), _arr(lh["red_hi"])
        self.light.LOWER_YELLOW, self.light.UPPER_YELLOW = _arr(lh["yellow_lo"]), _arr(lh["yellow_hi"])
        self.light.LOWER_GREEN, self.light.UPPER_GREEN = _arr(lh["green_lo"]), _arr(lh["green_hi"])
        self.params = p

    def process(self, frame):
        """Returns (command, telemetry, debug_frames)."""
        p = self.params
        t0 = time.perf_counter()
        target = float(p["throttle"])
        notes = io.StringIO()

        # The src/ estimators print diagnostics; capture them for the UI instead of spamming stdout.
        with contextlib.redirect_stdout(notes):
            est = self.lane.lane_estimate(frame)

            lane_ok = est is not None and est.len > 0 and est.len >= p["min_lane_points"]
            if lane_ok:
                steer = self.controller.compute_steering(est)
                throttle = target
                reason = ""
            else:
                steer, throttle = 0.0, 0.0
                reason = "no lane"

            min_dist, markers = None, {}
            if p["distance"]["enabled"]:
                min_dist, markers = self.aruco.estimate_distances(frame)
                if min_dist is not None:
                    d = p["distance"]
                    scale = max(0.0, min(1.0, (min_dist - d["stop_cm"]) / max(1e-6, d["slow_range_cm"])))
                    throttle *= scale
                    if lane_ok and scale <= 0.0:
                        reason = "obstacle too close"
                    elif lane_ok and scale < 1.0:
                        reason = "slowing for obstacle"

            light_state, focus = None, None
            if p["light"]["enabled"]:
                light_state, focus = self.light.process(frame)
                if light_state == "RED" or (light_state == "YEL" and p["light"]["stop_on_yellow"]):
                    throttle = 0.0
                    if lane_ok:
                        reason = f"{'red' if light_state == 'RED' else 'yellow'} light"

        annotated = frame.copy()
        for mid, out in markers.items():
            if out.corners is None:
                continue
            cv2.aruco.drawDetectedMarkers(annotated, [out.corners], np.array([[mid]], dtype=np.int32))
            x, y = out.corners[0][0]
            cv2.putText(annotated, f"{out.dist_m:.0f} cm" + ("" if out.detected else " (held)"),
                        (int(x), max(12, int(y) - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (0, 255, 0) if out.detected else (0, 255, 255), 2, cv2.LINE_AA)

        note_lines = [l for l in notes.getvalue().splitlines() if l.strip()]
        telemetry = {
            "lane_ok": bool(lane_ok),
            "offset_m": float(est.offset_m) if est is not None else None,
            "yaw_deg": float(est.yaw_deg) if est is not None else None,
            "curvature": float(est.curvature) if est is not None else None,
            "lane_points": int(est.len) if est is not None else 0,
            "min_dist_cm": None if min_dist is None else float(min_dist),
            "markers": [{"id": int(m), "dist_cm": float(o.dist_m), "detected": bool(o.detected)}
                        for m, o in markers.items()],
            "light": light_state,
            "steer": float(steer),
            "throttle": float(throttle),
            "target": target,
            "reason": reason,
            "note": note_lines[-1] if note_lines else "",
            "proc_ms": (time.perf_counter() - t0) * 1000.0,
        }
        frames = {
            "raw": annotated,
            "lane": est.debug_frame if est is not None else None,
            "light": focus,
        }
        return {"steer": steer, "throttle": throttle}, telemetry, frames
