#!/usr/bin/env python3
import argparse
import csv
import json
import os
import time

import cv2
import numpy as np
import paho.mqtt.client as mqtt
from libcamera import Transform
from picamera2 import Picamera2

from src.two_lane_poly import TwoLaneEstimator, LaneEstimate
from src.controllers import StanleyController
from src.aruco_dist import ArucoDistanceEstimator


# ---------------------------------------------------------------------------
# MQTT interface to rpi_pwm_sub.py
# ---------------------------------------------------------------------------
MQTT_BROKER = "localhost"
MQTT_PORT = 1883
MQTT_TOPIC = "car/cmd"
MQTT_CLIENT_ID = "lane-stanley-birdeye"

W = 640
H = 360
FPS = 30


def parse_args():
    parser = argparse.ArgumentParser(description="Bird's-eye Stanley lane follower")
    parser.add_argument(
        "--throttle",
        type=float,
        default=0.0,
        help="Throttle command, e.g. 0.18",
    )
    parser.add_argument(
        "--debug",
        action="store_true",
        help="Show debug window",
    )
    parser.add_argument(
        "--log-dir",
        type=str,
        default="logs",
        help="Directory to save video and control logs",
    )
    return parser.parse_args()


def publish_command(client: mqtt.Client, steer: float, throttle: float):
    payload = {
        "steer": float(steer),
        "throttle": float(throttle),
        "reverse": False,
        "rev_power": 0.0,
        "ts": time.time(),
    }
    client.publish(MQTT_TOPIC, json.dumps(payload), qos=0)


def safe_float(x):
    if x is None:
        return ""
    try:
        return float(x)
    except Exception:
        return ""


def main():
    args = parse_args()

    os.makedirs(args.log_dir, exist_ok=True)
    run_name = time.strftime("run_%Y%m%d_%H%M%S")

    video_path = os.path.join(args.log_dir, f"{run_name}.mp4")
    csv_path = os.path.join(args.log_dir, f"{run_name}.csv")

    # -----------------------------------------------------------------------
    # Camera setup
    # -----------------------------------------------------------------------
    picam2 = Picamera2()
    config = picam2.create_video_configuration(
        main={"size": (W, H), "format": "RGB888"},
        transform=Transform(rotation=180),
        controls={"FrameRate": FPS},
    )
    picam2.configure(config)
    picam2.set_controls({
        "ExposureTime": 10000,
        "AnalogueGain": 1.5,
    })
    picam2.start()
    time.sleep(0.2)

    # -----------------------------------------------------------------------
    # Video writer
    # Note: Picamera2 returns RGB, but OpenCV VideoWriter expects BGR.
    # -----------------------------------------------------------------------
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    video_writer = cv2.VideoWriter(video_path, fourcc, FPS, (W, H))

    if not video_writer.isOpened():
        raise RuntimeError(f"Could not open video writer: {video_path}")

    # -----------------------------------------------------------------------
    # CSV logger
    # -----------------------------------------------------------------------
    log_f = open(csv_path, "w", newline="", buffering=1)
    log_writer = csv.DictWriter(
        log_f,
        fieldnames=[
            "ts_wall",
            "ts_mono",
            "frame_id",
            "loop_dt",
            "steer_cmd",
            "throttle_cmd",
            "throttle_target",
            "lane_ok",
            "min_dist",
            "marker_ids",
            "stop_reason",
            "video_path",
        ],
    )
    log_writer.writeheader()

    # -----------------------------------------------------------------------
    # MQTT setup
    # -----------------------------------------------------------------------
    client = mqtt.Client(client_id=MQTT_CLIENT_ID, clean_session=True)
    client.connect(MQTT_BROKER, MQTT_PORT, 60)
    client.loop_start()

    estimator = TwoLaneEstimator()
    aruco_dist = ArucoDistanceEstimator()
    controller = StanleyController()
    throttle_target = float(args.throttle)

    frame_id = 0

    print(f"Recording video to: {video_path}")
    print(f"Logging control to: {csv_path}")

    try:
        while True:
            loop_start = time.monotonic()
            ts_wall = time.time()

            frame_rgb = picam2.capture_array()

            steer_cmd = 0.0
            throttle_cmd = 0.0
            stop_reason = ""
            lane_ok = False
            min_dist = None
            marker_info = None
            marker_ids = []

            # ---------------------------------------------------------------
            # Lane estimation and control
            # ---------------------------------------------------------------
            lane_estimate = estimator.lane_estimate(frame_rgb)
            print(f"Lane estimate: {lane_estimate}")

            if lane_estimate is None or lane_estimate.len == 0:
                print("No lane estimate available; stopping the car.")
                steer_cmd = 0.0
                throttle_cmd = 0.0
                stop_reason = "no_lane"
            else:
                lane_ok = True
                steer_cmd = controller.compute_steering(lane_estimate)
                throttle_cmd = throttle_target

                # -----------------------------------------------------------
                # ArUco distance-based throttle reduction
                # -----------------------------------------------------------
                min_dist, marker_info = aruco_dist.estimate_distances(frame_rgb)

                if marker_info is not None:
                    marker_ids = list(marker_info.keys())

                if min_dist is not None:
                    scale = max(0.0, min(1.0, (min_dist - 15) / 30))
                    throttle_cmd = throttle_target * scale

                    if throttle_cmd <= 1e-6:
                        stop_reason = "aruco_too_close"

            # ---------------------------------------------------------------
            # Always publish command, including stop commands
            # ---------------------------------------------------------------
            publish_command(client, steer_cmd, throttle_cmd)

            # ---------------------------------------------------------------
            # Write video frame
            # ---------------------------------------------------------------
            # frame_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
            video_writer.write(frame_rgb)

            # ---------------------------------------------------------------
            # Logging
            # ---------------------------------------------------------------
            loop_dt = time.monotonic() - loop_start

            log_writer.writerow({
                "ts_wall": ts_wall,
                "ts_mono": loop_start,
                "frame_id": frame_id,
                "loop_dt": loop_dt,
                "steer_cmd": steer_cmd,
                "throttle_cmd": throttle_cmd,
                "throttle_target": throttle_target,
                "lane_ok": lane_ok,
                "min_dist": safe_float(min_dist),
                "marker_ids": json.dumps(marker_ids),
                "stop_reason": stop_reason,
                "video_path": video_path,
            })

            # Periodically force log to disk
            if frame_id % FPS == 0:
                log_f.flush()
                os.fsync(log_f.fileno())

            # ---------------------------------------------------------------
            # Debug display
            # ---------------------------------------------------------------
            if args.debug:
                if lane_estimate is not None and hasattr(lane_estimate, "debug_frame"):
                    debug_frame = lane_estimate.debug_frame
                else:
                    debug_frame = frame_rgb.copy()

                cv2.imshow("Lane Following Debug", debug_frame)
                key = cv2.waitKey(1) & 0xFF
                if key == ord("q"):
                    break

            print(
                f"Frame {frame_id:06d} | "
                f"Steer: {steer_cmd:.3f}, "
                f"Throttle: {throttle_cmd:.3f}, "
                f"Lane: {lane_ok}, "
                f"MinDist: {min_dist}, "
                f"Reason: {stop_reason}"
            )

            frame_id += 1

    except KeyboardInterrupt:
        pass

    finally:
        print("Stopping car and closing logs...")

        publish_command(client, 0.0, 0.0)

        try:
            log_f.flush()
            os.fsync(log_f.fileno())
            log_f.close()
        except Exception:
            pass

        try:
            video_writer.release()
        except Exception:
            pass

        client.loop_stop()
        picam2.stop()
        cv2.destroyAllWindows()

        print(f"Saved video: {video_path}")
        print(f"Saved control log: {csv_path}")


if __name__ == "__main__":
    main()