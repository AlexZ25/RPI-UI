"""Car localization from ArUco markers placed around the grid, seen by each car's own camera.

Runs on the central computer next to hub.py. For every online car whose camera is on, it
fetches the car's camera image (http://<car>/frame/camera.jpg), finds the fixed markers
listed in grid.toml, works out where the camera is from each one (solvePnP) and sends the
car's position and heading to the hub's map. Nothing runs or changes on the cars.

Setup (once):
    python3 localizer.py --make-markers markers/ --ids 0-11   # printable PNGs
  - Print them all the same size (the black square = marker_size in grid.toml), keep the
    white border. Mount each one upright around the grid, printed side facing in, at about
    the height of the car cameras.
  - Measure where each one is and which way it faces, and write it in grid.toml
    (copy grid.example.toml).
  - Better accuracy: calibrate the car camera once (all cars use the same camera model):
        python3 localizer.py --calibrate http://deepracer-1.local:8080 --board 9x6 --square 0.025
    then pass --intrinsics camera.json. Without it the camera is modelled from --hfov.

Run:
    python3 localizer.py --grid grid.toml                       # hub at http://localhost:9000
    python3 localizer.py --grid grid.toml --intrinsics camera.json --hz 10

Needs: numpy, opencv-python (4.7+).
"""
from __future__ import annotations

import argparse
import base64
import json
import math
import os
import time
import tomllib
import urllib.error
import urllib.request
from concurrent.futures import ThreadPoolExecutor

import cv2
import numpy as np

DICTIONARY = cv2.aruco.DICT_4X4_50
MAX_RANGE_M = 6.0              # ignore marker detections farther than this (too noisy)
UPRIGHT_COS = math.cos(math.radians(35))   # camera roll/pitch vs a vertical marker beyond this = wrong pose
MAX_REPROJ_PX = 3.0            # a pose that does not reproduce the marker's corners this well is wrong
CARS_EVERY_S = 2.0             # how often the car list is re-read from the hub


# ---------------------------------------------------------------------- grid + camera
def load_grid(path):
    with open(path, "rb") as f:
        g = tomllib.load(f)
    size = float(g.get("marker_size", 0.15))
    height = float(g.get("marker_height", 0.15))
    markers = {}
    for m in g.get("markers", []):
        mid = int(m["id"])
        if mid in markers:
            raise SystemExit(f"{path}: marker {mid} is listed twice")
        markers[mid] = {"id": mid, "x": float(m["x"]), "y": float(m["y"]),
                        "facing": float(m.get("facing", 0.0)), "size": float(m.get("size", size)),
                        "height": float(m.get("height", height))}
    if not markers:
        raise SystemExit(f"{path}: no [[markers]]")
    return {"width": float(g["width"]), "height": float(g["height"]),
            "camera_forward": float(g.get("camera_forward", 0.0)), "markers": markers}


class CameraModel:
    """Intrinsics for a given image size: from a calibration file, else a pinhole from the FOV."""

    def __init__(self, hfov_deg=66.0, path=None):
        self.hfov, self.calib, self._cache = hfov_deg, None, {}
        if path:
            with open(path) as f:
                self.calib = json.load(f)

    def get(self, w, h):
        if (w, h) not in self._cache:
            if self.calib:
                sx, sy = w / self.calib["width"], h / self.calib["height"]
                K = np.array(self.calib["K"], np.float64) * [[sx], [sy], [1]]
                dist = np.array(self.calib["dist"], np.float64)
            else:
                f = (w / 2) / math.tan(math.radians(self.hfov) / 2)
                K, dist = np.array([[f, 0, w / 2], [0, f, h / 2], [0, 0, 1]], np.float64), np.zeros(5)
            self._cache[(w, h)] = (K, dist)
        return self._cache[(w, h)]


# ---------------------------------------------------------------------- localization
class Localizer:
    def __init__(self, grid, camera):
        self.grid, self.camera = grid, camera
        params = cv2.aruco.DetectorParameters()
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.dictionary = cv2.aruco.getPredefinedDictionary(DICTIONARY)
        self.params = params

    @staticmethod
    def _solve(obj, img, K, dist):
        """A square marker has two candidate poses (hard to tell apart when it is small or seen
        head-on: then the position is off sideways by up to ~15% of the distance). Keep the one
        with the camera in front of the marker and upright (cameras are level, markers vertical)
        that fits the corners best. None if neither is plausible."""
        img = img.astype(np.float64)
        candidates = []
        for flag in (cv2.SOLVEPNP_IPPE_SQUARE, cv2.SOLVEPNP_ITERATIVE):   # IPPE can fail when seen head-on
            n, rvecs, tvecs, _ = cv2.solvePnPGeneric(obj, img, K, dist, flags=flag)
            candidates += list(zip(rvecs[:n], tvecs[:n]))
        best = None
        for rvec, tvec in candidates:
            R, _ = cv2.Rodrigues(rvec)
            cam = -R.T @ tvec.reshape(3)
            down = -R[1, 1]                # image "down" vs the marker's down: 1 = perfectly upright
            if cam[2] <= 0 or down < UPRIGHT_COS:
                continue
            proj, _ = cv2.projectPoints(obj, rvec, tvec, K, dist)
            err = float(np.sqrt(np.mean(np.sum((proj.reshape(-1, 2) - img) ** 2, axis=1))))
            if err > MAX_REPROJ_PX:
                continue
            if best is None or err < best[0]:
                best = (err, cam, R[2])
        return best and best[1:]

    def locate(self, frame):
        """(pose or None, ids seen). pose = {"x", "y", "heading", "markers"} in map metres / radians."""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if frame.ndim == 3 else frame
        # a detector per call: locate() runs on several threads at once
        corners, ids, _ = cv2.aruco.ArucoDetector(self.dictionary, self.params).detectMarkers(gray)
        seen = [] if ids is None else [int(i) for i in ids.flatten()]
        K, dist = self.camera.get(gray.shape[1], gray.shape[0])
        singles, world, image = [], [], []        # one pose per marker; all corners, for a joint solve
        for mid, c in zip(seen, corners):
            m = self.grid["markers"].get(mid)
            if m is None:
                continue
            c = c.reshape(4, 2)
            s = m["size"] / 2              # marker frame: x right, y up, z out of the printed side
            obj = np.float32([[-s, s, 0], [s, s, 0], [s, -s, 0], [-s, -s, 0]])
            # map frame in 3D: x right, y down (as on the map), z into the floor
            f = math.radians(m["facing"])
            n, r = np.array([math.cos(f), math.sin(f), 0]), np.array([math.sin(f), -math.cos(f), 0])
            centre, up = np.array([m["x"], m["y"], -m["height"]]), np.array([0, 0, -1.0])
            world += [centre + a * r + b * up for a, b, _ in obj]
            image += list(c)
            best = self._solve(obj, c, K, dist)
            if best is None:
                continue
            cam, fwd = best                # camera position / optical axis in the marker frame
            dist_m = float(np.linalg.norm(cam))
            if dist_m > MAX_RANGE_M:
                continue
            # marker frame -> map: z_m = facing direction n, x_m = r (the viewer's right when
            # looking at the marker); the vertical axis y_m is dropped
            pos = np.array([m["x"], m["y"]]) + cam[2] * n[:2] + cam[0] * r[:2]
            d = fwd[2] * n[:2] + fwd[0] * r[:2]
            singles.append((pos, math.atan2(d[1], d[0]), 1.0 / max(dist_m, 0.2) ** 2))   # near = accurate
        pose = self._joint(np.array(world), np.array(image), K, dist) if len(world) >= 8 else None
        if pose is None and singles:       # one marker, or the joint solve failed: weighted mean
            W = sum(w for _, _, w in singles)
            pos = sum(w * p for p, _, w in singles) / W
            heading = math.atan2(sum(w * math.sin(h) for _, h, w in singles),
                                 sum(w * math.cos(h) for _, h, w in singles))
            pose = (pos[0], pos[1], heading)
        if pose is None:
            return None, seen
        x, y, heading = pose
        off = self.grid["camera_forward"]   # camera -> car centre
        x, y = x - off * math.cos(heading), y - off * math.sin(heading)
        return {"x": round(float(x), 3), "y": round(float(y), 3), "heading": round(float(heading), 3),
                "markers": max(len(world) // 4, len(singles))}, seen

    @staticmethod
    def _joint(world, image, K, dist):
        """Camera pose from the corners of several markers at once: much better than one small
        marker, whose angle is uncertain. (x, y, heading) in the map, or None."""
        ok, rvec, tvec = cv2.solvePnP(world, image, K, dist, flags=cv2.SOLVEPNP_SQPNP)
        if not ok:
            return None
        rvec, tvec = cv2.solvePnPRefineLM(world, image, K, dist, rvec, tvec)
        R, _ = cv2.Rodrigues(rvec)
        proj, _ = cv2.projectPoints(world, rvec, tvec, K, dist)
        err = float(np.sqrt(np.mean(np.sum((proj.reshape(-1, 2) - image) ** 2, axis=1))))
        if err > MAX_REPROJ_PX or R[1, 2] < UPRIGHT_COS:   # image "down" must be the floor's down
            return None
        cam, fwd = -R.T @ tvec.reshape(3), R[2]
        return cam[0], cam[1], math.atan2(fwd[1], fwd[0])


# ---------------------------------------------------------------------- hub / cars
class Hub:
    def __init__(self, url, password):
        self.url, self.password = url.rstrip("/"), password

    def _req(self, path, body=None, timeout=1.0):
        req = urllib.request.Request(self.url + path, data=body, method="POST" if body else "GET",
                                     headers={"Content-Type": "application/json"} if body else {})
        if self.password:
            req.add_header("Authorization", "Basic " + base64.b64encode(f"localizer:{self.password}".encode()).decode())
        with urllib.request.urlopen(req, timeout=timeout) as r:
            return json.loads(r.read())

    def cars(self):
        return self._req("/api/cars")["cars"]

    def send(self, grid, poses):
        layout = {"width": grid["width"], "height": grid["height"],
                  "markers": [{k: m[k] for k in ("id", "x", "y", "facing")} for m in grid["markers"].values()]}
        self._req("/api/positions", json.dumps({"grid": layout, "cars": poses}).encode())


def fetch_frame(car_url, timeout=1.0):
    with urllib.request.urlopen(f"{car_url}/frame/camera.jpg", timeout=timeout) as r:
        data = np.frombuffer(r.read(), np.uint8)
    frame = cv2.imdecode(data, cv2.IMREAD_COLOR)
    if frame is None:
        raise ValueError("not a JPEG")
    return frame


# ---------------------------------------------------------------------- setup helpers
def parse_ids(text):
    ids = []
    for part in text.split(","):
        a, _, b = part.partition("-")
        ids += list(range(int(a), int(b or a) + 1))
    return ids


def make_markers(out_dir, ids):
    os.makedirs(out_dir, exist_ok=True)
    d = cv2.aruco.getPredefinedDictionary(DICTIONARY)
    for mid in ids:
        page = np.full((800, 760), 255, np.uint8)
        page[80:680, 80:680] = cv2.aruco.generateImageMarker(d, mid, 600)   # white border = quiet zone
        cv2.putText(page, f"#{mid}   (measure the black square; this side faces the grid)", (40, 750),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, 0, 2, cv2.LINE_AA)
        cv2.imwrite(os.path.join(out_dir, f"marker-{mid:02d}.png"), page)
    print(f"wrote {len(ids)} markers to {out_dir}/ (print them the same size; keep the white border)")


def calibrate(car_url, board, square, out, views=15):
    """Chessboard calibration of a car camera from its dashboard's camera image."""
    cols, rows = (int(v) for v in board.lower().split("x"))   # inner corners
    obj = np.zeros((rows * cols, 3), np.float32)
    obj[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * square
    obj_pts, img_pts, last, size = [], [], None, None
    print(f"hold a {board} chessboard (inner corners) in front of the car camera; move and tilt it between views")
    while len(img_pts) < views:
        gray = cv2.cvtColor(fetch_frame(car_url, 3.0), cv2.COLOR_BGR2GRAY)
        size = gray.shape[::-1]
        found, corners = cv2.findChessboardCorners(gray, (cols, rows))
        if found:
            centre = corners.reshape(-1, 2).mean(axis=0)
            if last is None or np.linalg.norm(centre - last) > 30:     # only keep views that moved
                corners = cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1),
                                           (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.01))
                obj_pts.append(obj)
                img_pts.append(corners)
                last = centre
                print(f"view {len(img_pts)}/{views}")
        time.sleep(0.3)
    err, K, dist, _, _ = cv2.calibrateCamera(obj_pts, img_pts, size, None, None)
    with open(out, "w") as f:
        json.dump({"width": size[0], "height": size[1], "K": K.tolist(), "dist": dist.ravel().tolist()}, f, indent=2)
    print(f"wrote {out} (reprojection error {err:.2f} px; below 1 is good)")


# ---------------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser(description="Car-camera ArUco localization for the Trackside hub")
    ap.add_argument("--hub", default="http://localhost:9000")
    ap.add_argument("--grid", default="grid.toml", help="marker layout (see grid.example.toml)")
    ap.add_argument("--intrinsics", help="camera.json from --calibrate (default: pinhole from --hfov)")
    ap.add_argument("--hfov", type=float, default=66.0,
                    help="horizontal field of view of the car camera in degrees (Pi Camera Module 3: 66; wide: 102)")
    ap.add_argument("--hz", type=float, default=10.0, help="position updates per second")
    ap.add_argument("--make-markers", metavar="DIR", help="write printable marker PNGs and exit")
    ap.add_argument("--ids", default="0-11", help="marker ids for --make-markers, e.g. 0-11 or 0,2,5-9")
    ap.add_argument("--calibrate", metavar="CAR_URL", help="calibrate the car camera with a chessboard and exit")
    ap.add_argument("--board", default="9x6", help="chessboard inner corners for --calibrate")
    ap.add_argument("--square", type=float, default=0.025, help="chessboard square size in metres")
    ap.add_argument("--out", default="camera.json", help="output of --calibrate")
    args = ap.parse_args()

    if args.make_markers:
        return make_markers(args.make_markers, parse_ids(args.ids))
    if args.calibrate:
        url = args.calibrate if "://" in args.calibrate else "http://" + args.calibrate
        return calibrate(url.rstrip("/"), args.board, args.square, args.out)
    if not os.path.exists(args.grid):
        ap.error(f"{args.grid} not found: copy grid.example.toml and enter your marker layout")

    grid = load_grid(args.grid)
    loc = Localizer(grid, CameraModel(args.hfov, args.intrinsics))
    hub = Hub(args.hub, os.environ.get("HUB_PASSWORD", ""))
    pool = ThreadPoolExecutor(max_workers=16)
    period, cars, cars_t, last_print = 1.0 / args.hz, [], 0.0, 0.0
    print(f"{len(grid['markers'])} markers in {args.grid}; sending positions to {hub.url} (Ctrl+C to stop)")

    def one(car):
        try:
            pose, seen = loc.locate(fetch_frame(car["url"]))
        except (urllib.error.URLError, OSError, ValueError) as e:
            return car, None, f"no image: {getattr(e, 'reason', e)}"
        unknown = sorted(set(seen) - set(grid["markers"]))
        note = f"markers {sorted(seen) or 'none'}" + (f" (not in grid: {unknown})" if unknown else "")
        if pose:
            note += f" -> x {pose['x']:.2f} y {pose['y']:.2f} heading {math.degrees(pose['heading']):.0f} deg"
        return car, pose, note

    try:
        while True:
            t0 = time.monotonic()
            report = {}
            try:
                if t0 - cars_t > CARS_EVERY_S:
                    cars, cars_t = hub.cars(), t0
                live = [c for c in cars if c["online"] and (c["status"] or {}).get("camera_running")
                        and not (c["status"] or {}).get("simulated")]
                poses = {}
                for car, pose, note in pool.map(one, live):
                    report[car["name"]] = note
                    if pose:
                        poses[car["id"]] = pose
                for c in cars:
                    if c not in live:
                        report[c["name"]] = ("offline" if not c["online"] else
                                             "simulated (placed by the hub)" if (c["status"] or {}).get("simulated")
                                             else "camera off (Run turns it on)")
                hub.send(grid, poses)
            except (urllib.error.URLError, OSError, ValueError, KeyError) as e:
                report = {"hub": f"not reachable: {getattr(e, 'reason', e)}"}
                cars_t = 0.0
            if t0 - last_print > 2:
                for name, note in report.items():
                    print(f"  {name:<20} {note}")
                print(flush=True)
                last_print = t0
            time.sleep(max(0.0, period - (time.monotonic() - t0)))
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
