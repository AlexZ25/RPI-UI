#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import time
from dataclasses import dataclass
from typing import Optional

import cv2
import numpy as np
from libcamera import Transform
from picamera2 import Picamera2


@dataclass
class LaneEstimate:
    offset_m: float
    yaw_deg: float
    curvature: float
    len: int
    centerline_bev: np.ndarray
    debug_frame: np.ndarray
    center_poly_coeffs: np.ndarray
    white_poly_coeffs: np.ndarray
    yellow_poly_coeffs: np.ndarray


class TwoLaneEstimator:
    def __init__(self, width: int = 640, height: int = 360):
        self.bird_size = (width, height)

        self.src_pts = np.float32([
            [width * 0.0, height * 1.0],
            [width * 1.0, height * 1.0],
            [width * 1.0, height * 0.45],
            [width * 0.0, height * 0.45],
        ])

        self.dst_pts = np.float32([
            [width * 0.35, height - 1],
            [width * 0.65, height - 1],
            [width * 1.0, height * 0.15],
            [width * 0.0, height * 0.15],
        ])

        self.M = cv2.getPerspectiveTransform(self.src_pts, self.dst_pts)
        self.M_inv = cv2.getPerspectiveTransform(self.dst_pts, self.src_pts)

        self.gauss_kernel = (5, 5)

        # HSV thresholds
        # self.yellow = np.array([25, 206, 105])
        # self.yellow = np.array([25, 172, 95])
        self.yellow = np.array([30, 189, 137])
        self.lower_yellow = self.yellow - np.array([15, 50, 50])
        self.upper_yellow = self.yellow + np.array([15, 50, 50])

        # self.white = np.array([17, 86, 77])
        # self.white = np.array([17, 51, 85])
        self.white = np.array([34, 56, 136])
        self.lower_white = self.white - np.array([10, 20, 40])
        self.upper_white = self.white + np.array([10, 20, 40])

        self.area_min = 20

        # Pixel-to-meter scale in BEV.
        # You should calibrate this for your track.
        self.px_to_m = 0.0025

    def _empty_estimate(self, debug_frame: np.ndarray) -> LaneEstimate:
        return LaneEstimate(
            offset_m=0.0,
            yaw_deg=0.0,
            curvature=0.0,
            len=0,
            centerline_bev=np.array([]),
            debug_frame=debug_frame,
            center_poly_coeffs=np.array([]),
            white_poly_coeffs=np.array([]),
            yellow_poly_coeffs=np.array([]),
        )

    def _extract_lane_points(self, mask: np.ndarray) -> Optional[np.ndarray]:
        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE,
        )

        if not contours:
            return None

        largest = max(contours, key=cv2.contourArea)

        if cv2.contourArea(largest) <= self.area_min:
            return None

        lane_mask = np.zeros_like(mask)
        cv2.drawContours(lane_mask, [largest], -1, 255, thickness=cv2.FILLED)

        pts = []

        for row_idx, row in enumerate(lane_mask):
            cols = np.flatnonzero(row)

            if cols.size:
                cx = 0.5 * (float(cols[0]) + float(cols[-1]))
                pts.append((cx, float(row_idx)))

        if len(pts) < 5:
            return None

        return np.array(pts, dtype=np.float32)

    def _fit_parametric_lane(self, pts: np.ndarray, deg: int = 2):
        """
        Fit lane as x = f(y).
        """
        y = pts[:, 1]
        x = pts[:, 0]

        deg = min(deg, len(pts) - 1)

        coeffs = np.polyfit(y, x, deg)
        poly = np.poly1d(coeffs)

        return coeffs, poly

    def _curvature_from_poly(self, poly: np.poly1d, y_eval: float) -> float:
        """
        Curvature for x = f(y), in pixel coordinates.
        """
        dx_dy = float(np.polyder(poly, 1)(y_eval))
        d2x_dy2 = float(np.polyder(poly, 2)(y_eval))

        denom = (1.0 + dx_dy ** 2) ** 1.5

        if denom < 1e-6:
            return 0.0

        return d2x_dy2 / denom

    def lane_estimate(self, frame_rgb: np.ndarray) -> Optional[LaneEstimate]:
        if frame_rgb is None or frame_rgb.size == 0:
            print("Empty frame received.")
            return None

        bird = cv2.warpPerspective(
            frame_rgb,
            self.M,
            self.bird_size,
            flags=cv2.INTER_LINEAR,
        )

        # Remove unstable far region.
        bird[:int(self.bird_size[1] * 0.15), :] = 0

        bird_blur = cv2.GaussianBlur(bird, self.gauss_kernel, 5)

        # Picamera2 RGB888 gives RGB image.
        hsv_bird = cv2.cvtColor(bird_blur, cv2.COLOR_BGR2HSV)

        mask_white = cv2.inRange(
            hsv_bird,
            self.lower_white,
            self.upper_white,
        )

        mask_yellow = cv2.inRange(
            hsv_bird,
            self.lower_yellow,
            self.upper_yellow,
        )

        mask_white_color = cv2.cvtColor(mask_white, cv2.COLOR_GRAY2RGB)
        mask_yellow_color = cv2.cvtColor(mask_yellow, cv2.COLOR_GRAY2RGB)

        # Keep your original triple layout for clarity
        masks = np.hstack((
            mask_yellow_color,
            mask_white_color
        ))
        masks = cv2.resize(
            masks,
            (bird.shape[1], bird.shape[0]//2),
        )

        # Top: original + BEV + fitted result

        debug_frame = np.vstack((bird, masks))

        early_return = self._empty_estimate(debug_frame)

        white_pts = self._extract_lane_points(mask_white)
        yellow_pts = self._extract_lane_points(mask_yellow)

        if white_pts is None:
            print("No valid white lane found.")
            return early_return

        if yellow_pts is None:
            print("No valid yellow lane found.")
            return early_return

        try:
            white_coeffs, white_poly = self._fit_parametric_lane(white_pts, deg=2)
            yellow_coeffs, yellow_poly = self._fit_parametric_lane(yellow_pts, deg=2)
        except np.linalg.LinAlgError:
            print("Lane polynomial fitting failed.")
            return early_return

        y_min = int(max(np.min(white_pts[:, 1]), np.min(yellow_pts[:, 1])))
        y_max = int(min(np.max(white_pts[:, 1]), np.max(yellow_pts[:, 1])))

        if y_max <= y_min:
            print("No overlapping lane range.")
            return early_return

        y_sample = np.linspace(y_min, y_max, 80)

        x_white = white_poly(y_sample)
        x_yellow = yellow_poly(y_sample)

        x_center = 0.5 * (x_white + x_yellow)

        centerline = np.stack(
            [x_center, y_sample],
            axis=1,
        ).astype(np.float32)

        try:
            center_coeffs = np.polyfit(y_sample, x_center, 2)
            center_poly = np.poly1d(center_coeffs)
        except np.linalg.LinAlgError:
            print("Centerline polynomial fitting failed.")
            return early_return

        # Evaluate near the car, at the bottom of the BEV.
        y_eval = float(y_max)
        x_eval = float(center_poly(y_eval))

        offset_px = x_eval - self.bird_size[0] / 2.0
        offset_m = offset_px * self.px_to_m

        slope = float(np.polyder(center_poly, 1)(y_eval))
        yaw_deg = math.degrees(math.atan(slope))

        curvature_px = self._curvature_from_poly(center_poly, y_eval)

        # Since x and y are both scaled by px_to_m, curvature scales as 1 / meter.
        curvature_m = curvature_px / self.px_to_m if self.px_to_m > 0 else curvature_px

        debug = bird.copy()

        white_curve = np.stack(
            [x_white, y_sample],
            axis=1,
        ).astype(np.int32)

        yellow_curve = np.stack(
            [x_yellow, y_sample],
            axis=1,
        ).astype(np.int32)

        center_curve = centerline.astype(np.int32)

        cv2.polylines(debug, [white_curve], False, (255, 255, 255), 2)
        cv2.polylines(debug, [yellow_curve], False, (255, 255, 0), 2)
        cv2.polylines(debug, [center_curve], False, (0, 255, 0), 2)

        cv2.circle(
            debug,
            (int(x_eval), int(y_eval)),
            5,
            (255, 0, 0),
            -1,
        )

        cv2.putText(
            debug,
            f"Offset: {offset_m:.2f} m | Yaw: {yaw_deg:.1f} deg | Curv: {curvature_m:.4f} 1/m",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (0, 255, 0),
            2,
            cv2.LINE_AA,
        )

        mask_white_color = cv2.cvtColor(mask_white, cv2.COLOR_GRAY2RGB)
        mask_yellow_color = cv2.cvtColor(mask_yellow, cv2.COLOR_GRAY2RGB)

        mask_debug = np.hstack((mask_yellow_color, mask_white_color))

        # Resize mask debug to same width as debug before stacking.
        mask_debug = cv2.resize(
            mask_debug,
            (debug.shape[1], debug.shape[0]//2),
        )

        debug_frame = np.vstack((debug, mask_debug))

        return LaneEstimate(
            offset_m=offset_m,
            yaw_deg=yaw_deg,
            curvature=curvature_m,
            len=len(centerline),
            centerline_bev=centerline,
            debug_frame=debug_frame,
            center_poly_coeffs=center_coeffs,
            white_poly_coeffs=white_coeffs,
            yellow_poly_coeffs=yellow_coeffs,
        )


if __name__ == "__main__":
    estimator = TwoLaneEstimator(width=640, height=360)

    picam2 = Picamera2()

    config = picam2.create_still_configuration(
        main={
            "format": "RGB888",
            "size": (640, 360),
        },
        transform=Transform(rotation=180),
    )

    picam2.configure(config)

    picam2.set_controls({
        "ExposureTime": 10000,
        "AnalogueGain": 3.0,
    })

    picam2.start()
    time.sleep(0.5)

    print("Press 'q' to quit.")

    try:
        while True:
            frame = picam2.capture_array()

            lane_estimate = estimator.lane_estimate(frame)

            if lane_estimate is not None:
                cv2.imshow("Lane Estimation", lane_estimate.debug_frame)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

    finally:
        cv2.destroyAllWindows()
        picam2.stop()