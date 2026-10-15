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

W, H, FPS = 640, 360, 30
TRANSFORM = Transform(rotation=180)

def main():
    picam2 = Picamera2()
    config = picam2.create_video_configuration(
        main={"size": (W, H), "format": "RGB888"},
        transform=TRANSFORM,
        controls={"FrameRate": FPS},
    )
    picam2.configure(config)
    picam2.set_controls({
        "ExposureTime": 10000,
        "AnalogueGain": 1.5,
    })
    picam2.start()
    time.sleep(0.2)

    try:
        while True:
            frame_rgb = picam2.capture_array()
            frame_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)

            # calcuate the average hsv value of center area of the image
            h, w, _ = frame_rgb.shape
            center_x, center_y = w // 2, h // 2
            center_area = frame_rgb[center_y - 10 : center_y + 10, center_x - 10 : center_x + 10]
            avg_hsv = cv2.mean(cv2.cvtColor(center_area, cv2.COLOR_BGR2HSV))[:3]
            avg_hsv = [int(x) for x in avg_hsv]
            display_text = f"HSV: {avg_hsv}"
            cv2.putText(
                frame_rgb,
                display_text,
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                (255, 255, 255),
                2,
            )
            cv2.rectangle(
                frame_rgb,
                (center_x - 10, center_y - 10),
                (center_x + 10, center_y + 10),
                (0, 255, 0),
                2,
            )
            cv2.imshow("frame", frame_rgb)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()