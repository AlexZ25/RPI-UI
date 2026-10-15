"""Simulated camera + car for developing/testing the UI without the Pi.

Renders a top-down (bird's-eye) track, warps it into the camera view with the
inverse of the estimators' perspective transform, and adds an ArUco "lead car"
and an ArUco-tagged traffic light. The car follows the commands the UI
publishes, using the same steering sign convention as pwm_subscriber.
Frames are BGR, the same channel order Picamera2 "RGB888" returns.
"""
import math
import time

import cv2
import numpy as np

W, H = 640, 360


def _hsv_to_bgr(h, s, v):
    return tuple(int(c) for c in cv2.cvtColor(np.uint8([[[h, s, v]]]), cv2.COLOR_HSV2BGR)[0, 0])


FLOOR = _hsv_to_bgr(105, 25, 70)
WALL = _hsv_to_bgr(110, 20, 150)
RED_LANE = _hsv_to_bgr(4, 200, 190)        # matches src/single_lane.py red range
YELLOW_LANE = _hsv_to_bgr(30, 189, 137)    # = src/two_lane_poly.py yellow center
WHITE_LANE = _hsv_to_bgr(34, 56, 136)      # = src/two_lane_poly.py white center
LIGHT_BGR = {"GRN": _hsv_to_bgr(60, 220, 230), "YEL": _hsv_to_bgr(25, 220, 230), "RED": _hsv_to_bgr(170, 230, 230)}
LIGHT_CYCLE = [("GRN", 8.0), ("YEL", 2.0), ("RED", 5.0)]

# Same transform as SingleLaneEstimator / TwoLaneEstimator
SRC = np.float32([[0, H], [W, H], [W, H * 0.45], [0, H * 0.45]])
DST = np.float32([[W * 0.35, H - 1], [W * 0.65, H - 1], [W, H * 0.15], [0, H * 0.15]])
M_INV = cv2.getPerspectiveTransform(DST, SRC)

ARUCO_A, ARUCO_B = 2368.8255, -9.4504   # src/aruco_dist.py distance model (cm)


def _marker(marker_id, size):
    d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    img = cv2.aruco.generateImageMarker(d, marker_id, size) if hasattr(cv2.aruco, "generateImageMarker") \
        else cv2.aruco.drawMarker(d, marker_id, size)
    border = max(3, size // 5)
    img = cv2.copyMakeBorder(img, border, border, border, border, cv2.BORDER_CONSTANT, value=255)
    return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)


class SimWorld:
    def __init__(self):
        self.e = 0.0          # lane center lateral position relative to car (px, + = right)
        self.theta = 0.0      # car heading relative to lane (rad, + = pointing right)
        self.s = 0.0          # distance travelled along track (px)
        self.lead_cm = 90.0   # distance to the ArUco lead car (cm)
        self.t0 = time.monotonic()
        self.last = self.t0
        self._marker_cache = {}

    def road_curvature(self):
        return 0.0022 * math.sin(self.s / 700.0)

    def step(self, cmd):
        now = time.monotonic()
        dt = min(0.1, now - self.last)
        self.last = now

        # Same thresholds/semantics as pwm_subscriber's throttle mapping
        if cmd["reverse"] and cmd["rev_power"] > 0.05:
            v = -120.0 * cmd["rev_power"]
        elif not cmd["reverse"] and cmd["throttle"] > 0.05:
            v = 300.0 * cmd["throttle"]
        else:
            v = 0.0
        # pwm_subscriber: steer +1 -> duty above center -> left (joy_test convention); right turn = negative steer
        car_curv = -cmd["steer"] / 220.0
        k = self.road_curvature()
        self.theta += v * (car_curv - k) * dt
        self.theta = max(-1.2, min(1.2, self.theta))
        self.e -= v * math.sin(self.theta) * dt
        self.s += abs(v) * dt
        if abs(self.e) > 500:
            self.e, self.theta = 0.0, 0.0

        # Lead car drives at ~18 cm/s; 1 px ~= 0.2 cm
        car_cm_s = v * 0.2
        self.lead_cm += (18.0 - car_cm_s) * dt
        if self.lead_cm < 8.0:
            self.lead_cm = 8.0
        if self.lead_cm > 140.0:
            self.lead_cm = 60.0

    def light_state(self):
        period = sum(d for _, d in LIGHT_CYCLE)
        t = (time.monotonic() - self.t0) % period
        for state, d in LIGHT_CYCLE:
            if t < d:
                return state
            t -= d
        return "GRN"

    def _get_marker(self, marker_id, size):
        key = (marker_id, size)
        if key not in self._marker_cache:
            if len(self._marker_cache) > 200:
                self._marker_cache.clear()
            self._marker_cache[key] = _marker(marker_id, size)
        return self._marker_cache[key]

    def _paste(self, frame, img, cx, bottom):
        h, w = img.shape[:2]
        x0, y0 = int(cx - w / 2), int(bottom - h)
        x1, y1 = x0 + w, y0 + h
        fx0, fy0, fx1, fy1 = max(0, x0), max(0, y0), min(W, x1), min(H, y1)
        if fx1 > fx0 and fy1 > fy0:
            frame[fy0:fy1, fx0:fx1] = img[fy0 - y0:fy1 - y0, fx0 - x0:fx1 - x0]

    def render(self):
        bev = np.full((H, W, 3), FLOOR, np.uint8)
        k = self.road_curvature()
        ys = np.arange(0, H, 4, dtype=np.float64)
        d = (H - 1) - ys
        xc = W / 2 + self.e - self.theta * d + 0.5 * k * d * d
        for offset, color, thick in ((0, RED_LANE, 26), (-110, YELLOW_LANE, 12), (110, WHITE_LANE, 12)):
            pts = np.stack([xc + offset, ys], axis=1).astype(np.int32)
            cv2.polylines(bev, [pts], False, color, thick, cv2.LINE_AA)

        frame = cv2.warpPerspective(bev, M_INV, (W, H), flags=cv2.INTER_LINEAR,
                                    borderMode=cv2.BORDER_CONSTANT, borderValue=FLOOR)
        horizon = int(H * 0.45)
        frame[:horizon] = WALL

        # Lead car: ArUco id 1, pixel height chosen to match the distance model
        size = int(max(10, min(160, ARUCO_A / (self.lead_cm - ARUCO_B))))
        cx = W / 2 + self.e * 0.3 - self.theta * 200
        self._paste(frame, self._get_marker(1, size), cx, horizon + size // 3)

        # Traffic light: ArUco id 5 with the lamp drawn in the ROI just above it
        light = self._get_marker(5, 40)
        lx, lbottom = 560, horizon - 10
        self._paste(frame, light, lx, lbottom)
        border = max(3, 40 // 5)
        top = lbottom - light.shape[0] + border
        left, right = int(lx - 20), int(lx + 20)
        cv2.rectangle(frame, (left, top - 100), (right, top - 2), (30, 30, 30), -1)
        cv2.circle(frame, ((left + right) // 2, top - 50), 13, LIGHT_BGR[self.light_state()], -1)

        noise = np.random.randint(-6, 7, frame.shape, dtype=np.int16)
        return np.clip(frame.astype(np.int16) + noise, 0, 255).astype(np.uint8)
