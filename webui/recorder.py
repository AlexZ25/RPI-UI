"""Records camera video + per-frame command/telemetry CSV (like script/two_lane_log.py)."""
import csv
import json
import os
import time

import cv2

FIELDS = [
    "ts_wall", "frame_id", "mode", "steer_cmd", "throttle_cmd", "reverse", "rev_power",
    "lane_ok", "offset_m", "yaw_deg", "curvature", "min_dist", "marker_ids", "light", "stop_reason",
]


class Recorder:
    def __init__(self, log_dir):
        self.log_dir = log_dir
        self.writer = None
        self.csv_f = None
        self.csv = None
        self.name = None
        self.frame_id = 0
        self.started = 0.0

    @property
    def active(self):
        return self.writer is not None

    def start(self, fps, size):
        os.makedirs(self.log_dir, exist_ok=True)
        self.name = time.strftime("run_%Y%m%d_%H%M%S")
        video = os.path.join(self.log_dir, self.name + ".mp4")
        self.writer = cv2.VideoWriter(video, cv2.VideoWriter_fourcc(*"mp4v"), fps, size)
        if not self.writer.isOpened():
            self.writer = None
            raise RuntimeError(f"could not open video writer {video}")
        self.csv_f = open(os.path.join(self.log_dir, self.name + ".csv"), "w", newline="", buffering=1)
        self.csv = csv.DictWriter(self.csv_f, fieldnames=FIELDS)
        self.csv.writeheader()
        self.frame_id = 0
        self.started = time.time()

    def write(self, frame, mode, cmd, tel):
        self.writer.write(frame)
        tel = tel or {}
        self.csv.writerow({
            "ts_wall": time.time(), "frame_id": self.frame_id, "mode": mode,
            "steer_cmd": cmd["steer"], "throttle_cmd": cmd["throttle"],
            "reverse": cmd["reverse"], "rev_power": cmd["rev_power"],
            "lane_ok": tel.get("lane_ok", ""), "offset_m": tel.get("offset_m", ""),
            "yaw_deg": tel.get("yaw_deg", ""), "curvature": tel.get("curvature", ""),
            "min_dist": "" if tel.get("min_dist_cm") is None else tel["min_dist_cm"],
            "marker_ids": json.dumps([m["id"] for m in tel.get("markers", [])]),
            "light": tel.get("light") or "", "stop_reason": tel.get("reason", ""),
        })
        self.frame_id += 1

    def stop(self):
        if self.writer is not None:
            self.writer.release()
            self.writer = None
        if self.csv_f is not None:
            self.csv_f.close()
            self.csv_f = None

    def status(self):
        return {"active": self.active, "name": self.name if self.active else None,
                "frames": self.frame_id if self.active else 0,
                "seconds": time.time() - self.started if self.active else 0}


def list_runs(log_dir):
    runs = {}
    if not os.path.isdir(log_dir):
        return []
    for fn in os.listdir(log_dir):
        base, ext = os.path.splitext(fn)
        if ext not in (".csv", ".mp4") or not base.startswith("run_"):
            continue
        st = os.stat(os.path.join(log_dir, fn))
        r = runs.setdefault(base, {"name": base, "files": {}, "size": 0, "mtime": 0})
        r["files"][ext[1:]] = fn
        r["size"] += st.st_size
        r["mtime"] = max(r["mtime"], st.st_mtime)
    return sorted(runs.values(), key=lambda r: r["mtime"], reverse=True)
