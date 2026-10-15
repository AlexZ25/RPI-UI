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
from src.single_lane import SingleLaneEstimator, LaneEstimate
from src.controllers import StanleyController
from src.aruco_dist import ArucoDistanceEstimator
from src.traffic_light import TrafficLightDetector

# ---------------------------------------------------------------------------
# MQTT interface to rpi_pwm_sub.py
# ---------------------------------------------------------------------------
MQTT_BROKER = "localhost"
MQTT_PORT = 1883
MQTT_TOPIC = "car/cmd"
MQTT_CLIENT_ID = "lane-stanley-birdeye"

W = 640
H = 360

def parse_args():
    parser = argparse.ArgumentParser(description="Bird's-eye Stanley lane follower")
    parser.add_argument(
        "--throttle",
        type=float,
        help="Throttle command (e.g., 0.18)",
    )
    parser.add_argument(
        "--debug",
        action="store_true",
        help="Show debug window",
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

def main():

    args = parse_args()

    picam2 = Picamera2()
    config = picam2.create_video_configuration(
        main={"size": (W, H), "format": "RGB888"},
        transform=Transform(rotation=180),
        controls={"FrameRate": 30},
    )
    picam2.configure(config)
    picam2.set_controls({
        # Exposure time in microseconds (smaller → less blur, but darker)
        "ExposureTime": 5000,       # 6000–10000 is a good starting range
        "AnalogueGain": 1.0,        # boost brightness a bit
    })
    picam2.start()
    time.sleep(0.2)

    client = mqtt.Client(client_id=MQTT_CLIENT_ID, clean_session=True)
    client.connect(MQTT_BROKER, MQTT_PORT, 60)
    client.loop_start()

    t_prev = time.time()
    estimator = SingleLaneEstimator()
    aruco_dist = ArucoDistanceEstimator() 
    controller = StanleyController()
    traffic_light_detector = TrafficLightDetector()
    throttle_target = args.throttle  # constant throttle

    try:
        while True:
            frame_rgb = picam2.capture_array()
            lane_estimate = estimator.lane_estimate(frame_rgb)
            if lane_estimate is None:
                print("No lane estimate available; stopping the car.")
                throttle_cmd = 0.0
                steer_cmd = 0.0
                continue
            else:
                if lane_estimate.len < 20:
                    print("Insufficient centerline points; stopping the car.")
                    throttle_cmd = 0.0
                    steer_cmd = 0.0
                    continue
                steer_cmd = controller.compute_steering(lane_estimate)
                throttle_cmd = throttle_target

            min_dist, marker_info = aruco_dist.estimate_distances(frame_rgb)
            
            if min_dist is not None:
                throttle_cmd = throttle_target * max(0.0, min(1.0, (min_dist-20) / 50))

            state, roi_focus_rgb = traffic_light_detector.process(frame_rgb)
            if state == "RED" or state == "YEL":
                throttle_cmd = 0.0 
            

            publish_command(client, steer_cmd, throttle_cmd)

            if args.debug:
                # append frames together
                debug_frame = lane_estimate.debug_frame
                combined = np.vstack((roi_focus_rgb, debug_frame))
                # resize combined to fit in window
                combined = cv2.resize(combined, (int(W/2), H), interpolation=cv2.INTER_AREA)
                #draw detected Aruco info on frame-rgb
                if marker_info is not None:
                    for id in marker_info:
                        marker_output = marker_info[id]
                        corners = marker_output.corners
                        cv2.aruco.drawDetectedMarkers(frame_rgb, [corners], np.array([id]))
                combined = np.hstack((frame_rgb, combined))
                cv2.imshow("Lane Following Debug (L: Original, R: Bird's-eye)", combined)
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break
            print(f"Steer command: {steer_cmd:.3f}, Throttle command: {throttle_cmd:.3f}")


    except KeyboardInterrupt:
        pass
    finally:
        publish_command(client, 0.0, 0.0)
        client.loop_stop()
        picam2.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()