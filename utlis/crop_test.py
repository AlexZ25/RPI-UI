from picamera2 import Picamera2
import cv2
import numpy as np
import time
from libcamera import Transform

TRANSFORM = Transform(rotation=180)
# =========================
# User parameters
# =========================
TARGET_ID = 5

# Choose ONE of these if needed:
# cv2.aruco.DICT_4X4_50 / 100 / 250 / 1000
ARUCO_DICT_ID = cv2.aruco.DICT_4X4_50

# ROI is marker bbox extended to the LEFT by (marker_width * EXTEND_SCALE)
EXTEND_SCALE = 2.5

# Padding around marker bbox and ROI (pixels)
PAD_PX = 6

# Minimum contour area for LED blobs
MIN_AREA = 10

# =========================
# HSV color thresholds (OpenCV HSV: H in [0,180])
# =========================
# LOWER_RED1 = np.array([0, 120, 150])
# UPPER_RED1 = np.array([10, 255, 255])
# LOWER_RED2 = np.array([170, 120, 150])   # typical red wrap-around range
# UPPER_RED2 = np.array([180, 255, 255])

LOWER_RED = np.array([160, 200, 200])
UPPER_RED = np.array([180, 255, 255])

LOWER_YELLOW = np.array([10, 100, 150])
UPPER_YELLOW = np.array([35, 255, 255])

LOWER_GREEN = np.array([40, 80, 120])
UPPER_GREEN = np.array([85, 255, 255])

# =========================
# Camera setup
# =========================
picam2 = Picamera2()
preview_config = picam2.create_preview_configuration(
    main={"format": "RGB888", "size": (640, 360)},
    transform=TRANSFORM,
)

picam2.configure(preview_config)
picam2.set_controls({
    "ExposureTime": 10000,
    "AnalogueGain": 3.0,
})
picam2.start()

# =========================
# ArUco setup
# =========================
ARUCO_DICT = cv2.aruco.getPredefinedDictionary(ARUCO_DICT_ID)

if hasattr(cv2.aruco, "ArucoDetector"):
    ARUCO_PARAMS = cv2.aruco.DetectorParameters()
    ARUCO_DETECTOR = cv2.aruco.ArucoDetector(ARUCO_DICT, ARUCO_PARAMS)
else:
    ARUCO_PARAMS = cv2.aruco.DetectorParameters_create()
    ARUCO_DETECTOR = None


def detect_color(mask, hsv_roi, vis_bgr, label, draw_bgr, offset_xy=(0, 0)):
    """
    Find blobs in mask, compute mean HSV inside enclosing circle, and draw on vis_bgr.
    mask/hsv_roi correspond to ROI coordinates.
    offset_xy is (x_offset, y_offset) to map ROI coords -> full image coords.
    """
    ox, oy = offset_xy
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < MIN_AREA:
            continue

        (x, y), radius = cv2.minEnclosingCircle(cnt)
        cx, cy = int(x), int(y)
        radius = int(max(radius, 2))

        # Mean HSV within the circle
        mask_circle = np.zeros(mask.shape, dtype=np.uint8)
        cv2.circle(mask_circle, (cx, cy), radius, 255, -1)
        hsv_pixels = hsv_roi[mask_circle == 255]
        mean_hsv = hsv_pixels.mean(axis=0) if len(hsv_pixels) > 0 else [0, 0, 0]
        H, S, V = mean_hsv

        # Draw on full visualization (offset)
        center_full = (cx + ox, cy + oy)

        cv2.circle(vis_bgr, center_full, radius + 3, draw_bgr, 2)
        cv2.putText(vis_bgr, label,
                    (center_full[0] - 20, center_full[1] - radius - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, draw_bgr, 2, cv2.LINE_AA)

        hsv_text = f"H:{H:.1f} S:{S:.1f} V:{V:.1f}"
        cv2.putText(vis_bgr, hsv_text,
                    (center_full[0] - 40, center_full[1] + radius + 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, draw_bgr, 2, cv2.LINE_AA)


def find_marker_and_left_extension_roi(frame_rgb, target_id=5, pad_px=0, extend_scale=2.0):
    """
    Detect ArUco markers and, for target_id, build an ROI that is:
      - vertically aligned to marker bbox (with pad)
      - horizontally: [min_x - extend_scale*marker_width, min_x] (with pad)
    Returns:
      roi_rgb (view/copy),
      (x0, y0, x1, y1) in full-image coords,
      marker_bbox (mx0, my0, mx1, my1) or None if not found
    """
    H, W = frame_rgb.shape[:2]
    gray = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2GRAY)

    if ARUCO_DETECTOR is not None:
        corners, ids, _ = ARUCO_DETECTOR.detectMarkers(gray)
    else:
        corners, ids, _ = cv2.aruco.detectMarkers(gray, ARUCO_DICT, parameters=ARUCO_PARAMS)

    if ids is None or len(ids) == 0:
        return None, None, None

    ids = ids.flatten()
    idxs = np.where(ids == target_id)[0]
    if len(idxs) == 0:
        return None, None, None

    i = int(idxs[0])
    pts = corners[i].reshape(-1, 2)  # (4,2)

    min_x = float(np.min(pts[:, 0]))
    max_x = float(np.max(pts[:, 0]))
    min_y = float(np.min(pts[:, 1]))
    max_y = float(np.max(pts[:, 1]))

    # Marker axis-aligned bbox
    mx0 = int(max(0, np.floor(min_x) - pad_px))
    mx1 = int(min(W, np.ceil(max_x) + pad_px))
    my0 = int(max(0, np.floor(min_y) - pad_px))
    my1 = int(min(H, np.ceil(max_y) + pad_px))
    marker_bbox = (mx0, my0, mx1, my1)

    marker_w = max(1, mx1 - mx0)
    # Build ROI extending left from marker's LEFT edge (mx0)
    roi_w = int(np.round(marker_w * extend_scale))

    x1 = mx0  # right edge of ROI = marker left edge
    x0 = max(0, x1 - roi_w - pad_px)

    y0 = my0
    y1 = my1

    # Ensure ROI valid
    if x1 <= x0 or y1 <= y0:
        return None, None, marker_bbox

    roi_rgb = frame_rgb[y0:y1, x0:x1]
    return roi_rgb, (x0, y0, x1, y1), marker_bbox


# Morphology kernel
kernel = np.ones((3, 3), np.uint8)

try:
    while True:
        frame_rgb = picam2.capture_array()  # RGB888

        # For display/overlay in OpenCV colors, use BGR
        # vis_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)

        roi_rgb, roi_rect, marker_bbox = find_marker_and_left_extension_roi(
            frame_rgb,
            target_id=TARGET_ID,
            pad_px=PAD_PX,
            extend_scale=EXTEND_SCALE
        )

        if marker_bbox is not None:
            mx0, my0, mx1, my1 = marker_bbox
            cv2.rectangle(frame_rgb, (mx0, my0), (mx1, my1), (0, 255, 0), 2)
            cv2.putText(frame_rgb, f"Aruco {TARGET_ID}",
                        (mx0, max(0, my0 - 8)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2, cv2.LINE_AA)

        if roi_rgb is None or roi_rect is None:
            cv2.putText(frame_rgb, f"Aruco {TARGET_ID} not found / ROI invalid",
                        (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2, cv2.LINE_AA)
            cv2.imshow("ROI-left-of-Aruco + LED", frame_rgb)
            if (cv2.waitKey(1) & 0xFF) == ord('q'):
                break
            continue

        x0, y0, x1, y1 = roi_rect
        cv2.rectangle(frame_rgb, (x0, y0), (x1, y1), (255, 0, 0), 2)
        cv2.putText(frame_rgb, "ROI (left extension)",
                    (x0, max(0, y0 - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2, cv2.LINE_AA)

        # Run HSV detection ONLY inside ROI
        hsv = cv2.cvtColor(roi_rgb, cv2.COLOR_BGR2HSV)

        # mask_red1 = cv2.inRange(hsv, LOWER_RED1, UPPER_RED1)
        # mask_red2 = cv2.inRange(hsv, LOWER_RED2, UPPER_RED2)
        # mask_red = cv2.bitwise_or(mask_red1, mask_red2)
        mask_red = cv2.inRange(hsv, LOWER_RED, UPPER_RED)

        mask_yellow = cv2.inRange(hsv, LOWER_YELLOW, UPPER_YELLOW)
        mask_green = cv2.inRange(hsv, LOWER_GREEN, UPPER_GREEN)

        for m in (mask_red, mask_yellow, mask_green):
            cv2.morphologyEx(m, cv2.MORPH_OPEN, kernel, dst=m, iterations=1)
            cv2.morphologyEx(m, cv2.MORPH_DILATE, kernel, dst=m, iterations=1)

        # Draw detections on full image using offsets
        detect_color(mask_red, hsv, frame_rgb, "RED", (0, 0, 255), offset_xy=(x0, y0))
        detect_color(mask_yellow, hsv, frame_rgb, "YEL", (0, 255, 255), offset_xy=(x0, y0))
        detect_color(mask_green, hsv, frame_rgb, "GRN", (0, 255, 0), offset_xy=(x0, y0))

        cv2.imshow("ROI-left-of-Aruco + LED", frame_rgb)

        if (cv2.waitKey(1) & 0xFF) == ord('q'):
            break

finally:
    picam2.stop()
    cv2.destroyAllWindows()
