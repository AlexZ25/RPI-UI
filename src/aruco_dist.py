# import argparse
# import json
# import math
# import os
# import time
# from dataclasses import dataclass
# from typing import Optional, Tuple

# import cv2
# import numpy as np
# from libcamera import Transform
# from picamera2 import Picamera2



# class ArucoDistanceEstimator:
#     def __init__(self):
#         dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
#         parameters = cv2.aruco.DetectorParameters()
#         # TUNE PARAMETERS FOR BETTER STABILITY
#         parameters.adaptiveThreshWinSizeMin = 3
#         parameters.adaptiveThreshWinSizeMax = 23
#         parameters.adaptiveThreshWinSizeStep = 10
#         parameters.adaptiveThreshConstant = 7

#         parameters.minMarkerPerimeterRate = 0.02
#         parameters.maxMarkerPerimeterRate = 4.0
#         parameters.polygonalApproxAccuracyRate = 0.03

#         parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
#         parameters.cornerRefinementWinSize = 5
#         parameters.cornerRefinementMaxIterations = 30
#         parameters.cornerRefinementMinAccuracy = 0.01
#         self.detector = cv2.aruco.ArucoDetector(dictionary, parameters)

#     def estimate_distance(self, frame_rgb: np.ndarray) -> Tuple[float, Optional[np.ndarray]]:
#         frame_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
#         gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)

#         corners, ids, rejected = self.detector.detectMarkers(gray)

#         if ids is not None and len(ids) > 0:
#             marker_corners = corners[0]
#             marker_height_left = cv2.norm(marker_corners[0][0] - marker_corners[0][3])
#             marker_height_right = cv2.norm(marker_corners[0][1] - marker_corners[0][2])
#             marker_height = (marker_height_left + marker_height_right) / 2.0
#             # Apply polynomial fit to estimate distance from marker length
#             # Coefficients obtained from dist_fit.ipynb
#             a = 2368.8255
#             b = -9.4504
#             dist_m = a / marker_height + b
#             return dist_m, marker_corners
#         return -1.0, None  # Indicate no marker detected
    

# if __name__ == "__main__":
#     picam2 = Picamera2()
#     config = picam2.create_still_configuration(
#         main={"format": "RGB888", "size": (640, 360)}, 
#         transform=Transform(rotation=180))
#     picam2.configure(config)
#     picam2.set_controls({
#         # Exposure time in microseconds (smaller → less blur, but darker)
#         "ExposureTime": 50000,       # 6000–10000 is a good starting range
#         "AnalogueGain": 1.0,        # boost brightness a bit
#     })
#     picam2.start()
#     time.sleep(0.5)

#     estimator = ArucoDistanceEstimator()

#     print("Press 'q' to quit.")

#     try:
#         while True:
#             frame = picam2.capture_array()

#             dist_m, marker_corners = estimator.estimate_distance(frame)


#             if marker_corners is not None:
#                 cv2.aruco.drawDetectedMarkers(frame, [marker_corners])
#                 corner = marker_corners[0][0]  # top-left corner
#                 x, y = int(corner[0]), int(corner[1])
#                 cv2.circle(frame, (marker_corners[0][0][0].astype(int), marker_corners[0][0][1].astype(int)), 5, (0, 0, 255), -1)
#                 cv2.circle(frame, (marker_corners[0][2][0].astype(int), marker_corners[0][2][1].astype(int)), 5, (0, 255, 0), -1)
#                 cv2.putText(
#                     frame,
#                     f"Dist: {dist_m:.2f} cm",
#                     (x, y - 10),
#                     cv2.FONT_HERSHEY_SIMPLEX,
#                     0.7,
#                     (0, 255, 0),
#                     2,
#                 )
#                 cv2.putText(
#                     frame,
#                     f"Height1(px): {cv2.norm(marker_corners[0][0] - marker_corners[0][3]):.1f}",
#                     (x, y - 40),
#                     cv2.FONT_HERSHEY_SIMPLEX,
#                     0.7,
#                     (0, 255, 0),
#                     2,
#                 )
#                 cv2.putText(
#                     frame,
#                     f"Height2(px): {cv2.norm(marker_corners[0][1] - marker_corners[0][2]):.1f}",
#                     (x, y - 70),
#                     cv2.FONT_HERSHEY_SIMPLEX,
#                     0.7,
#                     (0, 255, 0),
#                     2,
#                 )


#             cv2.imshow("Aruco Distance Estimation", frame)

#             if cv2.waitKey(1) & 0xFF == ord('q'):
#                 break

#     finally:
#         picam2.stop()
#         cv2.destroyAllWindows()


import time
from dataclasses import dataclass
from typing import Dict, Optional, Tuple

import cv2
import numpy as np
from libcamera import Transform
from picamera2 import Picamera2


@dataclass
class MarkerState:
    last_dist_m: float
    last_corners: Optional[np.ndarray]  # (1,4,2)
    miss_count: int


@dataclass
class MarkerOutput:
    marker_id: int
    dist_m: float
    detected: bool          # True if detected this frame, False if stale (kept)
    miss_count: int         # consecutive missed frames
    corners: Optional[np.ndarray]


class ArucoDistanceEstimator:
    def __init__(self, keep_missed_frames: int = 5):
        self.keep_missed_frames = int(keep_missed_frames)
        self.state: Dict[int, MarkerState] = {}

        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        parameters = cv2.aruco.DetectorParameters()

        # TUNE PARAMETERS FOR BETTER STABILITY
        parameters.adaptiveThreshWinSizeMin = 3
        parameters.adaptiveThreshWinSizeMax = 23
        parameters.adaptiveThreshWinSizeStep = 10
        parameters.adaptiveThreshConstant = 7

        parameters.minMarkerPerimeterRate = 0.02
        parameters.maxMarkerPerimeterRate = 4.0
        parameters.polygonalApproxAccuracyRate = 0.03

        parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        parameters.cornerRefinementWinSize = 5
        parameters.cornerRefinementMaxIterations = 30
        parameters.cornerRefinementMinAccuracy = 0.01

        self.detector = cv2.aruco.ArucoDetector(dictionary, parameters)

        self.reserved_ids = [5]

        # Your calibrated model
        self.a = 2368.8255
        self.b = -9.4504

    def estimate_distances(self, frame_rgb: np.ndarray):
        """
        Multi-marker detect.
        Keep last distance for up to `keep_missed_frames` if marker disappears.
        NO estimation/interpolation beyond holding the last value.
        """
        # Detect in this frame
        frame_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
        gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)

        corners_list, ids, rejected = self.detector.detectMarkers(gray)

        detected_this_frame: Dict[int, Tuple[float, np.ndarray]] = {}
        if ids is not None and len(ids) > 0:
            ids_flat = ids.flatten().tolist()
            for i, marker_id in enumerate(ids_flat):
                if marker_id in self.reserved_ids:
                    continue
                marker_corners = corners_list[i]  # (1,4,2)

                h_left = cv2.norm(marker_corners[0][0] - marker_corners[0][3])
                h_right = cv2.norm(marker_corners[0][1] - marker_corners[0][2])
                marker_height = (h_left + h_right) / 2.0
                if marker_height < 1.0:
                    continue

                dist_m = self.a / marker_height + self.b
                detected_this_frame[marker_id] = (float(dist_m), marker_corners)

        # Update miss counts for all tracked markers first
        for marker_id in list(self.state.keys()):
            # will reset to 0 if detected below
            self.state[marker_id].miss_count += 1

        # Apply detections (reset miss_count)
        for marker_id, (dist_m, corners) in detected_this_frame.items():
            self.state[marker_id] = MarkerState(
                last_dist_m=dist_m,
                last_corners=corners,
                miss_count=0
            )

        # Drop markers that are missing too long
        for marker_id in list(self.state.keys()):
            if self.state[marker_id].miss_count > self.keep_missed_frames:
                self.state.pop(marker_id, None)

        # Build outputs: include detected and recently-missed markers
        outputs: Dict[int, MarkerOutput] = {}
        for marker_id, st in self.state.items():
            outputs[marker_id] = MarkerOutput(
                marker_id=marker_id,
                dist_m=st.last_dist_m,
                detected=(st.miss_count == 0),
                miss_count=st.miss_count,
                corners=st.last_corners,
            )
        
        # Distance to closest marker
        closest_marker_dist = min((st.last_dist_m for st in self.state.values()), default=None)

        return closest_marker_dist, outputs


if __name__ == "__main__":
    W, H = 640, 360

    picam2 = Picamera2()
    config = picam2.create_still_configuration(
        main={"format": "RGB888", "size": (W, H)},
        transform=Transform(rotation=180),
    )
    picam2.configure(config)
    picam2.set_controls({
        "ExposureTime": 50000,
        "AnalogueGain": 1.0,
    })
    picam2.start()
    time.sleep(0.5)

    estimator = ArucoDistanceEstimator(keep_missed_frames=5)

    print("Press 'q' to quit.")

    try:
        while True:
            frame = picam2.capture_array()
            _, outputs = estimator.estimate_distances(frame)

            # Draw + label tracked markers
            for mid, out in outputs.items():
                if out.corners is not None and out.detected:
                    cv2.aruco.drawDetectedMarkers(frame, [out.corners], np.array([[mid]], dtype=np.int32))

                if out.corners is not None:
                    corner = out.corners[0][0]
                    x, y = int(corner[0]), int(corner[1])
                else:
                    x, y = 10, 30  # fallback

                # NOTE: Your model units might be cm; adjust here if needed.
                dist_cm = out.dist_m * 100.0  # if out.dist_m is meters
                status = "DET" if out.detected else f"MISS {out.miss_count}/5"

                cv2.putText(
                    frame,
                    f"ID {mid}: {dist_cm:.1f} cm ({status})",
                    (x, y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 255, 0) if out.detected else (0, 255, 255),
                    2,
                )

            cv2.imshow("Aruco Distance Estimation (hold last value up to 5 misses)", frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        picam2.stop()
        cv2.destroyAllWindows()
