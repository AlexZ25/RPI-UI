#!/usr/bin/env python3
"""
Lane-following autopilot that uses a bird's-eye inverse perspective mapping
to estimate lane geometry, then feeds a Stanley-style controller with
curvature feed-forward and a fixed-speed throttle schedule. Commands are
published to the MQTT topic consumed by rpi_pwm_sub.py.
"""
import argparse
import json
import math
import os
import time
from dataclasses import dataclass
from typing import Optional, Tuple

import cv2
import numpy as np
import paho.mqtt.client as mqtt
from libcamera import Transform
from picamera2 import Picamera2

# ---------------------------------------------------------------------------
# Camera + bird's-eye transform configuration (mirrors lane_overlay_birdeye)
# ---------------------------------------------------------------------------
W, H, FPS = 640, 360, 30
TRANSFORM = Transform(rotation=180)

SRC_QUAD = np.float32([
    [W * 0.05, H * 0.98],
    [W * 0.95, H * 0.98],
    [W * 0.90, H * 0.45],
    [W * 0.10, H * 0.45],
])
DST_QUAD = np.float32([
    [W * 0.35, H - 1],
    [W * 0.65, H - 1],
    [W * 0.90, 0.0],
    [W * 0.10, 0.0],
])
BIRD_SIZE = (W, H)

HSV_LOW = np.array([0, 0, 90])
HSV_HIGH = np.array([180, 60, 200])

LOWER_RED1 = np.array([0, 120, 180])
UPPER_RED1 = np.array([10, 255, 255])
LOWER_RED2 = np.array([100, 120, 180])
UPPER_RED2 = np.array([180, 255, 255])

AREA_MIN = 200
DEFAULT_CONFIG = {
    "hsv_low": [10, 60, 60],
    "hsv_high": [110, 255, 255],
    "area_min": 200,
    "gauss_kernel": [5, 5],
    "k_e": 1.0,
    "k_v": 1.0,
    "ff_gain": 1.0,
    "throttle_cmd": 0.4, # >= 0.4
    "steer_smooth_alpha": 0.4,
}

HSV_LOW = np.array(DEFAULT_CONFIG["hsv_low"])
HSV_HIGH = np.array(DEFAULT_CONFIG["hsv_high"])
AREA_MIN = DEFAULT_CONFIG["area_min"]
GAUSS_KERNEL = tuple(DEFAULT_CONFIG["gauss_kernel"])

# ---------------------------------------------------------------------------
# Vehicle & controller parameters
# ---------------------------------------------------------------------------
PX_TO_M = 0.002    # 2 mm/pixel – tune for your setup
WHEELBASE_M = 0.13

K_E = DEFAULT_CONFIG["k_e"]
K_V = DEFAULT_CONFIG["k_v"]
FF_GAIN = DEFAULT_CONFIG["ff_gain"]
MAX_STEER_RAD = math.radians(30)

CONTROL_SPEED_MPS = 0.2  # virtual speed used in controller math
THROTTLE_CMD = DEFAULT_CONFIG["throttle_cmd"]      # constant throttle command
STEER_SMOOTH_ALPHA = DEFAULT_CONFIG["steer_smooth_alpha"]

# ---------------------------------------------------------------------------
# MQTT interface to rpi_pwm_sub.py
# ---------------------------------------------------------------------------
MQTT_BROKER = "localhost"
MQTT_PORT = 1883
MQTT_TOPIC = "car/cmd"
MQTT_CLIENT_ID = "lane-stanley-birdeye"

DEBUG_VIEW = True

if DEBUG_VIEW:
    os.environ.setdefault("QT_QPA_PLATFORM", "wayland")


def load_config(path: Optional[str]) -> dict:
    cfg = DEFAULT_CONFIG.copy()
    if not path:
        return cfg
    try:
        with open(path, "r", encoding="utf-8") as f:
            user_cfg = json.load(f)
        for key, value in user_cfg.items():
            if key in cfg:
                cfg[key] = value
    except FileNotFoundError:
        print(f"[lane-stanley] Config file '{path}' not found. Using defaults.")
    except json.JSONDecodeError as exc:
        print(f"[lane-stanley] Failed to parse config '{path}': {exc}. Using defaults.")
    return cfg


def apply_config(cfg: dict) -> None:
    global HSV_LOW, HSV_HIGH, AREA_MIN, GAUSS_KERNEL
    global K_E, K_V, FF_GAIN, THROTTLE_CMD, STEER_SMOOTH_ALPHA

    HSV_LOW = np.array(cfg["hsv_low"], dtype=np.int32)
    HSV_HIGH = np.array(cfg["hsv_high"], dtype=np.int32)
    AREA_MIN = int(cfg["area_min"])
    GAUSS_KERNEL = tuple(int(x) for x in cfg["gauss_kernel"])

    K_E = float(cfg["k_e"])
    K_V = float(cfg["k_v"])
    FF_GAIN = float(cfg["ff_gain"])
    THROTTLE_CMD = float(cfg["throttle_cmd"])
    STEER_SMOOTH_ALPHA = float(cfg["steer_smooth_alpha"])


def parse_args():
    parser = argparse.ArgumentParser(description="Bird's-eye Stanley lane follower")
    parser.add_argument(
        "--config",
        type=str,
        help="Path to JSON config file overriding controller and vision parameters.",
    )
    return parser.parse_args()


@dataclass
class LaneEstimate:
    offset_m: float
    yaw_deg: float
    curvature: float
    centerline_bev: np.ndarray          # (N, 2) centerline in BEV
    left_bev: Optional[np.ndarray]      # (N_l, 2) left lane boundary in BEV
    right_bev: Optional[np.ndarray]     # (N_r, 2) right lane boundary in BEV
    frame_original: np.ndarray
    frame_birdeye: np.ndarray


def compute_lane_estimate(frame_rgb: np.ndarray, M: np.ndarray, M_inv: np.ndarray) -> Optional[LaneEstimate]:
    if frame_rgb is None or frame_rgb.size == 0:
        return None

    # Bird's-eye transform
    bird = cv2.warpPerspective(frame_rgb, M, BIRD_SIZE, flags=cv2.INTER_LINEAR)
    bird_blur = cv2.GaussianBlur(bird, GAUSS_KERNEL, 0)
    hsv_bird = cv2.cvtColor(bird_blur, cv2.COLOR_RGB2HSV)

    # Detect red lane markings (left/right lane lines)
    mask_red1 = cv2.inRange(hsv_bird, LOWER_RED1, UPPER_RED1)
    mask_red2 = cv2.inRange(hsv_bird, LOWER_RED2, UPPER_RED2)
    mask = cv2.bitwise_or(mask_red1, mask_red2)

    lane_mask = np.zeros_like(mask)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None

    # Keep contours above area threshold
    big_contours = [c for c in contours if cv2.contourArea(c) > AREA_MIN]
    if not big_contours:
        return None

    # Helper: convert a contour to a polyline (x,y) by taking mean x per row
    def contour_to_polyline(contour: np.ndarray) -> Optional[np.ndarray]:
        cmask = np.zeros_like(mask)
        cv2.drawContours(cmask, [contour], -1, 255, thickness=cv2.FILLED)
        pts = []
        rows, cols = cmask.shape
        for y in range(rows):
            xs = np.flatnonzero(cmask[y])
            if xs.size:
                cx = int(xs.mean())
                pts.append((cx, y))
        if len(pts) < 2:
            return None
        return np.array(pts, dtype=np.float32)

    left_bev = None
    right_bev = None
    centerline = None

    # Case 1: at least two lane boundaries found → use two-lane geometry
    if len(big_contours) >= 2:
        # Take two largest contours
        big_contours.sort(key=cv2.contourArea, reverse=True)
        c1, c2 = big_contours[0], big_contours[1]

        poly1 = contour_to_polyline(c1)
        poly2 = contour_to_polyline(c2)

        # If one of them is degenerate, we will fall back later
        if poly1 is not None and poly2 is not None:
            # Assign left/right based on mean x
            if poly1[:, 0].mean() < poly2[:, 0].mean():
                left_bev = poly1
                right_bev = poly2
            else:
                left_bev = poly2
                right_bev = poly1

            # Build dictionaries keyed by row (y) for averaging
            left_dict = {}
            for x, y in left_bev:
                y_int = int(y)
                left_dict[y_int] = float(x)

            right_dict = {}
            for x, y in right_bev:
                y_int = int(y)
                right_dict[y_int] = float(x)

            y_common = sorted(set(left_dict.keys()) & set(right_dict.keys()))
            if len(y_common) >= 2:
                center_pts = []
                for y in y_common:
                    xL = left_dict[y]
                    xR = right_dict[y]
                    xC = 0.5 * (xL + xR)
                    center_pts.append((xC, y))
                centerline = np.array(center_pts, dtype=np.float32)

    # Case 2: fallback – use single large blob and infer "virtual" centerline
    if centerline is None:
        # Use the single largest contour as before
        largest = max(big_contours, key=cv2.contourArea)
        cv2.drawContours(lane_mask, [largest], -1, 255, thickness=cv2.FILLED)

        pts = []
        for row_idx, row in enumerate(lane_mask):
            cols = np.flatnonzero(row)
            if cols.size:
                cx = int((int(cols[0]) + int(cols[-1])) / 2)
                pts.append((cx, row_idx))
        if len(pts) < 2:
            return None

        centerline = np.array(pts, dtype=np.float32)

        # No explicit left/right here
        left_bev = None
        right_bev = None
    else:
        # If we had two lanes, also draw them in lane_mask (for completeness)
        cv2.drawContours(lane_mask, [big_contours[0]], -1, 255, thickness=cv2.FILLED)
        if len(big_contours) > 1:
            cv2.drawContours(lane_mask, [big_contours[1]], -1, 255, thickness=cv2.FILLED)

    # ----------------------------------------------------------------------
    # Fit polynomial to centerline and compute offset, yaw, curvature
    # ----------------------------------------------------------------------
    y_vals = centerline[:, 1]
    x_vals = centerline[:, 0]
    deg = 2 if centerline.shape[0] >= 3 else 1

    try:
        coeffs = np.polyfit(y_vals, x_vals, deg)
    except np.linalg.LinAlgError:
        return None

    poly = np.poly1d(coeffs)
    y_eval = float(np.max(y_vals))
    x_eval = float(poly(y_eval))
    offset_px = x_eval - (BIRD_SIZE[0] / 2.0)

    slope = float(np.polyder(poly)(y_eval))
    yaw_deg = math.degrees(math.atan(slope))

    curvature = 0.0
    if deg >= 2:
        second_der = float(np.polyder(poly, 2)(y_eval))
        denom = (1.0 + slope ** 2) ** 1.5
        curvature = second_der / denom if denom != 0 else 0.0

    # ----------------------------------------------------------------------
    # Visualization in BEV and original frame
    # ----------------------------------------------------------------------
    vis_bird = bird.copy()
    centerline_int = centerline.astype(np.int32)
    cv2.polylines(vis_bird, [centerline_int], False, (255, 255, 255), 2)  # center: white

    if left_bev is not None:
        cv2.polylines(vis_bird, [left_bev.astype(np.int32)], False, (0, 255, 0), 2)   # left: green
    if right_bev is not None:
        cv2.polylines(vis_bird, [right_bev.astype(np.int32)], False, (255, 0, 0), 2)  # right: blue

    # Reproject centerline to original view
    pts_full = centerline.reshape(-1, 1, 2)
    reproj = cv2.perspectiveTransform(pts_full, M_inv).reshape(-1, 2)
    vis_orig = frame_rgb.copy()
    cv2.polylines(vis_orig, [np.round(reproj).astype(np.int32)], False, (0, 0, 255), 2)

    # Convert to metric units
    offset_m = offset_px * PX_TO_M
    curvature_m = curvature / PX_TO_M if PX_TO_M > 0 else curvature

    return LaneEstimate(
        offset_m=offset_m,
        yaw_deg=yaw_deg,
        curvature=curvature_m,
        centerline_bev=centerline,
        left_bev=left_bev,
        right_bev=right_bev,
        frame_original=vis_orig,
        frame_birdeye=vis_bird,
    )

def stanley_control(estimate: LaneEstimate) -> Tuple[float, float]:
    heading_error_rad = math.radians(estimate.yaw_deg)
    cross_track_m = estimate.offset_m
    curvature_m = estimate.curvature

    stanley_term = math.atan(
        (K_E * cross_track_m) / (K_V + CONTROL_SPEED_MPS)
    )

    delta_ff = math.atan(WHEELBASE_M * curvature_m) * FF_GAIN
    steer_rad = heading_error_rad + stanley_term + delta_ff
    steer_norm = max(-1.0, min(1.0, steer_rad / MAX_STEER_RAD))

    throttle = max(0.0, THROTTLE_CMD)
    return steer_norm, throttle


def publish_command(client: mqtt.Client, steer: float, throttle: float):
    payload = {
        "steer": float(steer),
        "throttle": float(throttle),
        "reverse": False,
        "rev_power": 0.0,
        "ts": time.time(),
    }
    client.publish(MQTT_TOPIC, json.dumps(payload), qos=0)


def main():
    args = parse_args()
    cfg = load_config(args.config)
    apply_config(cfg)

    picam2 = Picamera2()
    config = picam2.create_video_configuration(
        main={"size": (W, H), "format": "RGB888"},
        transform=TRANSFORM,
        controls={"FrameRate": FPS},
    )
    picam2.configure(config)
    picam2.start()
    time.sleep(0.2)

    if DEBUG_VIEW:
        cv2.namedWindow("stanley-birdeye", cv2.WINDOW_AUTOSIZE)

    M = cv2.getPerspectiveTransform(SRC_QUAD, DST_QUAD)
    M_inv = cv2.getPerspectiveTransform(DST_QUAD, SRC_QUAD)

    client = mqtt.Client(client_id=MQTT_CLIENT_ID, clean_session=True)
    client.connect(MQTT_BROKER, MQTT_PORT, 60)
    client.loop_start()

    last_steer = 0.0
    t_prev = time.time()

    try:
        while True:
            frame_rgb = picam2.capture_array()
            estimate = compute_lane_estimate(frame_rgb, M, M_inv)

            if estimate is None:
                steer_cmd = 0.0
                throttle_cmd = 0.0
                display = frame_rgb if frame_rgb is not None else np.zeros((H, W, 3), dtype=np.uint8)
            else:
                steer_raw, throttle_raw = stanley_control(estimate)

                steer_cmd = STEER_SMOOTH_ALPHA * steer_raw + (1 - STEER_SMOOTH_ALPHA) * last_steer
                throttle_cmd = throttle_raw
                last_steer = steer_cmd

                bird_resized = cv2.resize(estimate.frame_birdeye, (W, H))
                orig_scale = 0.6
                orig_w = int(W * orig_scale)
                orig_h = int(H * orig_scale)
                orig_resized = cv2.resize(estimate.frame_original, (orig_w, orig_h))

                orig_x = 10
                orig_y = 10
                gap = 10
                bird_y = orig_y + orig_h + gap
                total_h = bird_y + H
                display = np.zeros((total_h, W, 3), dtype=np.uint8)
                display[bird_y:bird_y + H, :, :] = bird_resized
                display[orig_y:orig_y + orig_h, orig_x:orig_x + orig_w] = orig_resized
                cv2.rectangle(display, (orig_x - 2, orig_y - 2), (orig_x + orig_w + 2, orig_y + orig_h + 2), (0, 0, 0), 2)
                cv2.putText(display, "ORIGINAL", (orig_x, orig_y + 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2, cv2.LINE_AA)
                cv2.putText(display, "BIRD-EYE", (10, bird_y + 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2, cv2.LINE_AA)

                metrics_x = orig_x + orig_w + 20
                metrics_y = orig_y + 30
                cv2.putText(
                    display,
                    f"offset {estimate.offset_m:+.3f} m",
                    (metrics_x, metrics_y),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (255, 255, 0),
                    2,
                    cv2.LINE_AA,
                )
                cv2.putText(
                    display,
                    f"yaw {estimate.yaw_deg:+.1f}°",
                    (metrics_x, metrics_y + 25),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (255, 255, 0),
                    2,
                    cv2.LINE_AA,
                )
                if abs(estimate.curvature) > 1e-6:
                    radius_text = f"radius {1.0 / estimate.curvature:+.1f} m"
                else:
                    radius_text = "radius straight"
                cv2.putText(
                    display,
                    radius_text,
                    (metrics_x, metrics_y + 50),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (255, 255, 0),
                    2,
                    cv2.LINE_AA,
                )
                cv2.putText(
                    display,
                    f"steer {steer_cmd:+.2f}",
                    (metrics_x, metrics_y + 75),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 255, 255),
                    2,
                    cv2.LINE_AA,
                )
                cv2.putText(
                    display,
                    f"throttle {throttle_cmd:.2f}",
                    (metrics_x, metrics_y + 100),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 255, 255),
                    2,
                    cv2.LINE_AA,
                )

            publish_command(client, steer_cmd, throttle_cmd)

            if DEBUG_VIEW and display is not None:
                cv2.imshow("stanley-birdeye", display)
                if (cv2.waitKey(1) & 0xFF) == ord("q"):
                    break

            t_now = time.time()
            dt = t_now - t_prev
            t_prev = t_now
            time.sleep(max(0.0, (1.0 / FPS) - dt))
    except KeyboardInterrupt:
        pass
    finally:
        publish_command(client, 0.0, 0.0)
        client.loop_stop()
        picam2.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
