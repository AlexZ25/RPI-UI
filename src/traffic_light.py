import cv2
import numpy as np
import time


class TrafficLightDetector:
    """
    One-call interface:
        state, roi_focus_rgb = detector.process(frame_rgb)

    - frame_rgb: RGB image (H,W,3) uint8
    - state: "RED" | "YEL" | "GRN" | "UNK"
    - roi_focus_rgb: RGB image same size as input, showing an enlarged ROI (scaled-up) if available,
                     otherwise zeros (or last ROI if you choose to keep it).
    - Hold-last: if detection is missing for <= hold_seconds, returns last state.
    """

    def __init__(
        self,
        marker_id=5,
        hold_seconds=0.5,
        enlarge_scale=2.0,
        min_area=1,
    ):
        # ---- ArUco ----
        self.marker_id = marker_id
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params) \
            if hasattr(cv2.aruco, "ArucoDetector") else None

        # NOTE: keep conversions "as-is" relative to your last code:
        # - ROI is cropped from frame_rgb (RGB)
        # - HSV uses cv2.COLOR_BGR2HSV (as you had)
        # So we do NOT "fix" that here.

        # ---- Color thresholds (kept as-is from your last version) ----
        self.LOWER_RED = np.array([160, 200, 100])
        self.UPPER_RED = np.array([180, 255, 255])

        self.LOWER_YELLOW = np.array([10, 100, 100])
        self.UPPER_YELLOW = np.array([35, 255, 255])

        self.LOWER_GREEN = np.array([40, 80, 120])
        self.UPPER_GREEN = np.array([85, 255, 255])

        self.MIN_AREA = min_area
        self.kernel = np.ones((3, 3), np.uint8)

        # ---- Output/behavior ----
        self.enlarge_scale = float(enlarge_scale)
        self.hold_seconds = float(hold_seconds)

        # ---- State memory ----
        self._last_state = "UNK"
        self._last_detect_t = 0.0
        self._last_roi_focus = None  # RGB same size as input (cached)

    # -------------------------
    # Public API
    # -------------------------
    def process(self, frame_rgb):
        """
        Main one-call function:
          Input: frame_rgb
          Output: (state, roi_focus_rgb)
        """
        now = time.monotonic()
        H, W = frame_rgb.shape[:2]

        roi_rgb, roi_coords = self._find_roi(frame_rgb)

        # Always prepare a same-size focus frame
        roi_focus_rgb = np.zeros_like(frame_rgb)

        if roi_rgb is not None and roi_coords is not None:
            # Enlarge ROI into a same-size focus frame
            roi_focus_rgb = self._make_focus_frame(frame_rgb, roi_rgb, roi_coords)

            # Detect traffic light color inside ROI
            state = self._detect_state_from_roi(roi_rgb)

            if state != "UNK":
                self._last_state = state
                self._last_detect_t = now
                self._last_roi_focus = roi_focus_rgb
                cv2.putText(roi_focus_rgb, f"STATE: {state}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 0), 2, cv2.LINE_AA)
                return state, roi_focus_rgb

        # If we get here, detection failed this frame (no ROI or no color)
        state = self.get_state_with_hold(now)

        # For focus frame: prefer current roi_focus if we had ROI, else last cached focus (optional)
        if (roi_rgb is None or roi_coords is None) and self._last_roi_focus is not None:
            roi_focus_rgb = self._last_roi_focus

        cv2.putText(roi_focus_rgb, f"PREV STATE: {state}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 0), 2, cv2.LINE_AA)

        return state, roi_focus_rgb

    def get_state_with_hold(self, now=None):
        """
        Return last known state if current detection is missing for <= hold_seconds.
        Otherwise return "UNK".
        """
        if now is None:
            now = time.monotonic()

        if self._last_state != "UNK" and (now - self._last_detect_t) <= self.hold_seconds:
            return self._last_state
        return "UNK"

    # -------------------------
    # Internal: ROI finding (kept conversions as-is)
    # -------------------------
    def _find_roi(self, frame_rgb):
        """
        Same ROI logic you had. Returns (roi_rgb, (l, b, r, t)) or (None, None).
        """
        H, W = frame_rgb.shape[:2]
        gray = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2GRAY)

        if self.detector is not None:
            corners, ids, _ = self.detector.detectMarkers(gray)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

        if ids is None:
            return None, None

        ids_flat = ids.flatten()
        if self.marker_id not in ids_flat:
            return None, None

        index = int(np.where(ids_flat == self.marker_id)[0][0])
        marker_corners = corners[index][0]  # (4,2)

        x_min = int(np.min(marker_corners[:, 0]))
        y_min = int(np.min(marker_corners[:, 1]))
        x_max = int(np.max(marker_corners[:, 0]))
        y_max = int(np.max(marker_corners[:, 1]))

        marker_h = max(1, y_max - y_min)
        marker_w = max(1, x_max - x_min)

        # Your ROI logic
        # roi_top = y_max + int(0.2 * marker_h)
        # roi_bottom = y_min - int(0.2 * marker_h)
        # roi_left = x_min - int(0.5 * marker_w)
        # roi_right = x_min
        roi_top = y_min 
        roi_bottom = y_min - int(2.5 * marker_h)
        roi_left = x_min
        roi_right = x_max

        # Clip
        roi_left = max(0, roi_left)
        roi_right = min(W, roi_right)
        roi_bottom = max(0, roi_bottom)
        roi_top = min(H, roi_top)

        if roi_right <= roi_left or roi_top <= roi_bottom:
            return None, None

        roi_rgb = frame_rgb[roi_bottom:roi_top, roi_left:roi_right]
        return roi_rgb, (roi_left, roi_bottom, roi_right, roi_top)

    # -------------------------
    # Internal: mask cleanup
    # -------------------------
    def _morph(self, m):
        cv2.morphologyEx(m, cv2.MORPH_OPEN, self.kernel, dst=m, iterations=1)
        cv2.morphologyEx(m, cv2.MORPH_DILATE, self.kernel, dst=m, iterations=1)
        return m

    def _count_lit_pixels(self, mask):
        # number of "on" pixels; robust and cheap
        return int(cv2.countNonZero(mask))

    # -------------------------
    # Internal: detect traffic light state (keep conversions as-is)
    # -------------------------
    def _detect_state_from_roi(self, roi_rgb):
        """
        Returns: "RED" | "YEL" | "GRN" | "UNK"
        """
        # KEEP AS-IS: you used BGR2HSV even though roi_rgb comes from RGB frame
        hsv = cv2.cvtColor(roi_rgb, cv2.COLOR_BGR2HSV)

        mask_red = cv2.inRange(hsv, self.LOWER_RED, self.UPPER_RED)
        mask_yellow = cv2.inRange(hsv, self.LOWER_YELLOW, self.UPPER_YELLOW)
        mask_green = cv2.inRange(hsv, self.LOWER_GREEN, self.UPPER_GREEN)

        mask_red = self._morph(mask_red)
        mask_yellow = self._morph(mask_yellow)
        mask_green = self._morph(mask_green)

        # Decide state by largest mask support (simple + stable)
        red_score = self._count_lit_pixels(mask_red)
        yel_score = self._count_lit_pixels(mask_yellow)
        grn_score = self._count_lit_pixels(mask_green)

        # Optional: require a minimum score to avoid noise
        # You can tune this; start small because ROI is small.
        MIN_SCORE = 10
        best = max((red_score, "RED"), (yel_score, "YEL"), (grn_score, "GRN"), key=lambda x: x[0])

        if best[0] < MIN_SCORE:
            return "UNK"
        return best[1]

    # -------------------------
    # Internal: build "focus" frame (same size)
    # -------------------------
    def _make_focus_frame(self, frame_rgb, roi_rgb, roi_coords):
        """
        Returns an RGB frame same size as input, where we place a scaled-up ROI.
        We center the enlarged ROI in the output frame.
        """
        H, W = frame_rgb.shape[:2]
        l, b, r, t = roi_coords

        # scale ROI
        roi_h, roi_w = roi_rgb.shape[:2]
        if roi_h <= 0 or roi_w <= 0:
            return np.zeros_like(frame_rgb)

        scaled_w = int(round(roi_w * self.enlarge_scale))
        scaled_h = int(round(roi_h * self.enlarge_scale))

        # Prevent overly large
        scaled_w = max(1, min(W, scaled_w))
        scaled_h = max(1, min(H, scaled_h))

        roi_scaled = cv2.resize(roi_rgb, (scaled_w, scaled_h), interpolation=cv2.INTER_LINEAR)

        # place centered
        out = np.zeros_like(frame_rgb)
        x0 = (W - scaled_w) // 2
        y0 = (H - scaled_h) // 2
        out[y0:y0 + scaled_h, x0:x0 + scaled_w] = roi_scaled
        return out



from picamera2 import Picamera2
from libcamera import Transform
import cv2
import time

def main():
    picam2 = Picamera2()
    preview_config = picam2.create_preview_configuration(
        main={"format": "RGB888", "size": (640, 360)},
        transform=Transform(rotation=180),
    )
    picam2.configure(preview_config)
    picam2.set_controls({"ExposureTime": 10000, "AnalogueGain": 3.0})
    picam2.start()
    time.sleep(0.5)

    detector = TrafficLightDetector(marker_id=5, hold_seconds=0.5, enlarge_scale=2.5)

    try:
        while True:
            frame_rgb = picam2.capture_array()
            state, focus_rgb = detector.process(frame_rgb)

            # Display side-by-side (both RGB arrays; cv2.imshow expects BGR, but you asked not to touch conversions)
            # So we show them as-is (same as your prior approach).
            concatenated = np.hstack((frame_rgb, focus_rgb))

            cv2.putText(concatenated, f"STATE: {state}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 0), 2, cv2.LINE_AA)

            cv2.imshow("frame | focus", concatenated)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    finally:
        picam2.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
