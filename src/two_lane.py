import argparse
import json
import math
import os
import time
from dataclasses import dataclass
from typing import Optional, Tuple

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

class TwoLaneEstimator:
    def __init__(self, width = 640, height = 360):
        # Bird's-eye view parameters
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
        self.gauss_kernel = (5, 5)
        
        self.yellow = np.array([30, 206, 135])
        self.lower_yellow = self.yellow - np.array([10, 20, 20])
        self.upper_yellow = self.yellow + np.array([10, 20, 20])
        self.white = np.array([35, 60, 123])
        self.lower_white = self.white - np.array([10, 20, 20])
        self.upper_white = self.white + np.array([10, 20, 20])
        # self.lower_yellow = np.array([0, 80, 180])
        # self.upper_yellow = np.array([180, 160, 260])
        # self.lower_white = np.array([20, 20, 180])
        # self.upper_white = np.array([40, 40, 220 ])
        self.area_min = 100
        self.px_to_m = 0.002 

        self.M = cv2.getPerspectiveTransform(self.src_pts, self.dst_pts)
        self.M_inv = cv2.getPerspectiveTransform(self.dst_pts, self.src_pts)

    def lane_estimate(self, frame_rgb: np.ndarray) -> Optional[LaneEstimate]:
        if frame_rgb is None or frame_rgb.size == 0:
            print("Empty frame received.")
            return None

        bird = cv2.warpPerspective(frame_rgb, self.M, self.bird_size, flags=cv2.INTER_LINEAR)
        bird[:int(self.bird_size[1]*0.15), :] = 0  # Mask out top area

        debug_frame = np.hstack((frame_rgb, bird, np.zeros_like(bird)))

        bird_blur = cv2.GaussianBlur(bird, self.gauss_kernel, 5)
        hsv_bird = cv2.cvtColor(bird_blur, cv2.COLOR_BGR2HSV)

        mask_white = cv2.inRange(hsv_bird, self.lower_white, self.upper_white)
        mask_yellow = cv2.inRange(hsv_bird, self.lower_yellow, self.upper_yellow)

        mask_white_color = cv2.cvtColor(mask_white, cv2.COLOR_GRAY2RGB)
        mask_yellow_color = cv2.cvtColor(mask_yellow, cv2.COLOR_GRAY2RGB)
        masks = np.hstack((mask_yellow_color, mask_white_color, mask_yellow_color))
        debug_frame = np.vstack((debug_frame, masks))

        early_return = LaneEstimate(
            offset_m=0.0,
            yaw_deg=0.0,
            curvature=0.0,
            len=0.0,
            centerline_bev=np.array([]),
            debug_frame=debug_frame,
        )

        # white line
        white_lane_mask = np.zeros_like(mask_white)
        white_contours, _ = cv2.findContours(mask_white, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not white_contours:
            print("No white contours found.")
            return early_return
        largest_white = max(white_contours, key=cv2.contourArea)
        if cv2.contourArea(largest_white) <= self.area_min:
            print("No sufficiently large white contour found.")
            return early_return

        cv2.drawContours(white_lane_mask, [largest_white], -1, 255, thickness=cv2.FILLED)

	    # yellow line
        yellow_lane_mask = np.zeros_like(mask_yellow)
        yellow_contours, _ = cv2.findContours(mask_yellow, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not yellow_contours:
                print("No yellow contours found.")
                return early_return
        largest_yellow = max(yellow_contours, key=cv2.contourArea)
        if cv2.contourArea(largest_yellow) <= self.area_min:
                print("No sufficiently large yellow contour found.")
                return early_return
        cv2.drawContours(yellow_lane_mask, [largest_yellow], -1, 255, thickness=cv2.FILLED)

        # white
        pts_w = []
        for row_idx, row in enumerate(white_lane_mask):
            cols = np.flatnonzero(row)
            if cols.size:
                cx = int((int(cols[0]) + int(cols[-1])) / 2)
                pts_w.append((cx, row_idx))
        if len(pts_w) < 2:
            print("Not enough lane points found.")
            return early_return

        centerline_w = np.array(pts_w, dtype=np.float32)

        centerline_w_len = centerline_w.shape[0]

        y_vals = centerline_w[:, 1]
        x_vals = centerline_w[:, 0]
        deg = 2 if centerline_w.shape[0] >= 3 else 1
        try:
            coeffs = np.polyfit(y_vals, x_vals, deg)
        except np.linalg.LinAlgError:
            print("Polynomial fit failed.")
            return early_return
        poly = np.poly1d(coeffs)
        y_eval = float(np.max(y_vals))
        x_eval = float(poly(y_eval))
        offset_px = x_eval - (self.bird_size[0] / 2.0)
        slope = float(np.polyder(poly)(y_eval))
        yaw_deg = math.degrees(math.atan(slope))

        curvature = 0.0
        if deg >= 2:
            second_der = float(np.polyder(poly, 2)(y_eval))
            denom = (1.0 + slope ** 2) ** 1.5
            curvature = second_der / denom if denom != 0 else 0.0

        centerline_int = centerline_w.astype(np.int32)
        cv2.polylines(bird, [centerline_int], False, (255, 255, 255), 2)


        # yellow
        pts_y = []
        for row_idx, row in enumerate(yellow_lane_mask):
            cols = np.flatnonzero(row)
            if cols.size:
                cx = int((int(cols[0]) + int(cols[-1])) / 2)
                pts_y.append((cx, row_idx))
        if len(pts_y) < 2:
            print("Not enough lane points found.")
            return early_return

        centerline_y = np.array(pts_y, dtype=np.float32)

        centerline_y_len = centerline_y.shape[0]

        y_vals_y = centerline_y[:, 1]
        x_vals_y = centerline_y[:, 0]
        deg = 2 if centerline_y.shape[0] >= 3 else 1
        try:
            coeffs = np.polyfit(y_vals_y, x_vals_y, deg)
        except np.linalg.LinAlgError:
            print("Polynomial fit failed.")
            return early_return
        poly = np.poly1d(coeffs)
        y_eval_y = float(np.max(y_vals_y))
        x_eval_y = float(poly(y_eval_y))
        offset_px = x_eval_y - (self.bird_size[0] / 2.0)
        slope = float(np.polyder(poly)(y_eval_y))
        yaw_deg = math.degrees(math.atan(slope))

        curvature_y = 0.0
        if deg >= 2:
            second_der = float(np.polyder(poly, 2)(y_eval_y))
            denom_y = (1.0 + slope ** 2) ** 1.5
            curvature_y = second_der / denom_y if denom_y != 0 else 0.0

        centerline_int_y = centerline_y.astype(np.int32)
        cv2.polylines(bird, [centerline_int_y], False, (0, 128, 0), 2)

        # other
        offset_m = offset_px * self.px_to_m
        curvature_m = curvature / self.px_to_m if self.px_to_m > 0 else curvature

        cv2.putText(
            bird,
            f"Offset: {offset_m:.2f} m, Yaw: {yaw_deg:.1f} deg, Curv: {curvature_m:.4f} 1/m",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 0),
            2,
            cv2.LINE_AA,
        )
        # debug_frame = bird
        mask_color = cv2.cvtColor(mask_white, cv2.COLOR_GRAY2RGB)
        debug_frame = np.vstack((bird, mask_color))

        return LaneEstimate(
            offset_m=offset_m,
            yaw_deg=yaw_deg,
            curvature=curvature_m,
            len=centerline_w_len,
            centerline_bev=centerline_w,
            debug_frame=debug_frame,
        )
    
if __name__ == "__main__":
    estimator = TwoLaneEstimator()
    # Further code to utilize the estimator can be added here
    picam2 = Picamera2()
    config = picam2.create_still_configuration(
        main={"format": "RGB888", "size": (640, 360)}, 
        transform=Transform(rotation=180))
    picam2.configure(config)
    # picam2.set_controls({
    #     # Exposure time in microseconds (smaller → less blur, but darker)
    #     "ExposureTime": 50000,       # 6000–10000 is a good starting range
    #     "AnalogueGain": 1.0,        # boost brightness a bit
    # })
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


            cv2.imshow("Lane Estimation", lane_estimate.debug_frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        cv2.destroyAllWindows()
        picam2.stop()
