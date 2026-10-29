"""Trackside: one page for a fleet of DeepRacer-style cars (standard library only).

Runs on the central computer. Each car runs its own dashboard (port 8080) at its own IP.
Cars join by themselves (join.py on the car registers its address, and finds the hub
from the hub's UDP broadcast) or are added by address from the page. The hub polls
every car's /api/status, proxies their images, forwards Run / Stop / policy uploads,
keeps the car list (saved to cars.toml) and shows a map of where the cars are: from
localizer.py (each car's own camera + ArUco markers placed around the grid) or, for
simulated cars, placed on an oval by the distance they have driven.

    python3 hub.py                            # cars.toml, http://<this computer>:9000
    python3 hub.py --cars my.toml --port 9001
    python3 hub.py --no-beacon                # do not broadcast (cars then need --hub)
    HUB_PASSWORD=secret python3 hub.py        # require a password (any user name)

Endpoints (JSON unless noted):
    GET    /                                  the page
    GET    /api/cars                          cached status of every car
    GET    /api/info                          this hub's network addresses (for joining)
    POST   /api/register                      from join.py on a car: {host, name?, port?}; address = sender's IP
    POST   /api/cars                          add a car {name, url, color?}
    PUT    /api/cars/<id>                     edit a car (same fields)
    DELETE /api/cars/<id>                     remove a car from the hub (the car itself is untouched)
    POST   /api/probe                         {url}: is there a car dashboard there?
    POST   /api/cars/<id>/run|stop            one car
    POST   /api/all/run|stop                  every car, in parallel (run: online cars only)
    POST   /api/policy                        {text, cars: [ids]}: save + reload on those cars
    GET    /car/<id>/frame/<name>.jpg         camera | overlay | mask | policy (proxied JPEG)
    GET    /api/map                           grid size, marker layout + car positions
    POST   /api/positions                     from localizer.py: {grid: {width, height, markers: [{id, x, y, facing}]},
                                                                  cars: {car id: {x, y, heading, markers}}}
"""
from __future__ import annotations

import argparse
import base64
import hmac
import json
import math
import os
import re
import socket
import sys
import threading
import time
import tomllib
import urllib.error
import urllib.request
from concurrent.futures import ThreadPoolExecutor
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse

APP = "Trackside"
HERE = os.path.dirname(os.path.abspath(__file__))
FRAMES = ("camera", "overlay", "mask", "policy")
PALETTE = ("#d9463e", "#3b7dd8", "#2f9e5b", "#e0a21b", "#8e5bd0", "#e0672b", "#1fa3a3", "#c94c8e")
POLL_S = 0.3                 # status poll interval per car
TIMEOUT_S = 1.0              # per request to a car
OFFLINE_AFTER_S = 1.5        # no good status for this long = offline
RUN_WAIT_S = 4.0             # how long "run" waits for a car's camera to start
POSITION_STALE_S = 2.0       # localizer positions older than this are not shown
BEACON_PORT = 9999           # UDP: the hub announces itself here so join.py can find it
BEACON_EVERY_S = 2.0
MAX_BODY = 2 * 1024 * 1024

# Oval used to place simulated cars (metres): two straights joined by half circles.
SIM_TRACK = {"width": 6.0, "height": 3.4}
_SIM_STRAIGHT, _SIM_R = 3.2, 1.2
_SIM_LAP = 2 * _SIM_STRAIGHT + 2 * math.pi * _SIM_R


def _sim_place(distance, lateral=0.0):
    """(x, y, heading) on the oval after `distance` metres, clockwise, y pointing down."""
    cx, cy = SIM_TRACK["width"] / 2, SIM_TRACK["height"] / 2
    half, r = _SIM_STRAIGHT / 2, _SIM_R
    d = distance % _SIM_LAP
    if d < _SIM_STRAIGHT:                                     # top straight, left -> right
        x, y, h = cx - half + d, cy - r, 0.0
    elif d < _SIM_STRAIGHT + math.pi * r:                     # right half circle
        a = (d - _SIM_STRAIGHT) / r - math.pi / 2
        x, y, h = cx + half + r * math.cos(a), cy + r * math.sin(a), a + math.pi / 2
    elif d < 2 * _SIM_STRAIGHT + math.pi * r:                 # bottom straight, right -> left
        x, y, h = cx + half - (d - _SIM_STRAIGHT - math.pi * r), cy + r, math.pi
    else:                                                     # left half circle
        a = (d - 2 * _SIM_STRAIGHT - math.pi * r) / r + math.pi / 2
        x, y, h = cx - half + r * math.cos(a), cy + r * math.sin(a), a + math.pi / 2
    # lateral > 0 = car right of the line (the inside of this clockwise oval)
    nx, ny = -math.sin(h), math.cos(h)
    return x + nx * lateral, y + ny * lateral, h


# ---------------------------------------------------------------------- talking to a car
class CarError(Exception):
    pass


def _request(url, data=None, timeout=TIMEOUT_S):
    body = None if data is None else json.dumps(data).encode()
    req = urllib.request.Request(url, data=body, method="GET" if data is None else "POST",
                                 headers={"Content-Type": "application/json"} if body else {})
    try:
        with urllib.request.urlopen(req, timeout=timeout) as r:
            return r.read(), r.headers.get("Content-Type", "")
    except urllib.error.HTTPError as e:          # the car answered with an error (e.g. 409)
        try:
            msg = json.loads(e.read()).get("error") or str(e)
        except (ValueError, AttributeError):
            msg = str(e)
        raise CarError(msg) from None
    except (urllib.error.URLError, OSError, ValueError) as e:
        reason = getattr(e, "reason", e)
        if isinstance(reason, OSError) and reason.strerror:
            reason = reason.strerror.lower()        # "[Errno 61] Connection refused" -> "connection refused"
        raise CarError(f"unreachable: {reason}") from None


def _json(url, data=None, timeout=TIMEOUT_S):
    raw, _ = _request(url, data, timeout)
    try:
        return json.loads(raw)
    except ValueError:
        raise CarError("not a car dashboard (reply is not JSON)") from None


STATUS_PATH = "/api/status?log_since=999999999&events_since=999999999"   # skip log/event history


class Car:
    def __init__(self, cid, name, url, color="", host=""):
        self.id = cid
        self.lock = threading.Lock()
        self.status = None           # trimmed status from the car
        self.last_ok = 0.0
        self.error = "not polled yet"
        self.message = ""            # result of the last hub action, shown on the card
        self.active = True           # False once removed: its poll thread ends
        self.host = host             # the car's hostname, when it joined by itself (join.py)
        self.configure(name, url, color)

    def configure(self, name, url, color):
        self.name, self.url, self.color = name, url.rstrip("/"), color

    def config(self):
        return {"id": self.id, "name": self.name, "url": self.url, "color": self.color, "host": self.host}

    @staticmethod
    def _trim(s):
        """Small view of /api/status: only what the page needs."""
        veh = s.get("vehicle", {})
        cmd = veh.get("command", {})
        pol, per = s.get("policy", {}), s.get("perception", {})
        last = s.get("last_step") or {}
        return {
            "car": s.get("car"), "simulated": s.get("simulated"), "mode": s.get("mode"),
            "stop_reason": s.get("stop_reason"),
            "steering": cmd.get("steering"), "throttle": cmd.get("throttle"),
            "policy_loaded": pol.get("loaded"), "policy_version": pol.get("version"),
            "policy_error": pol.get("load_error"), "run_time": pol.get("run_time"),
            "steps_per_s": pol.get("steps_per_s"),
            "camera_running": per.get("running"), "fps": per.get("fps"),
            "perception_error": per.get("error"),
            "link": {"kind": veh.get("kind"), "ok": veh.get("ok"), "detail": veh.get("detail"),
                     "pwm_alive": veh.get("pwm_alive")},
            "debug": last.get("debug") if isinstance(last, dict) else None,
            "max_throttle": s.get("limits", {}).get("max_throttle"),
            "sim": s.get("sim"),
        }

    def poll(self):
        try:
            s = _json(self.url + STATUS_PATH)
            with self.lock:
                self.status, self.last_ok, self.error = self._trim(s), time.monotonic(), ""
        except CarError as e:
            with self.lock:
                self.error = str(e)

    def online(self):
        with self.lock:
            return time.monotonic() - self.last_ok < OFFLINE_AFTER_S

    def view(self):
        online = self.online()
        with self.lock:
            return dict(self.config(), online=online, status=self.status if online else None,
                        error="" if online else self.error, message=self.message)

    def note(self, msg):
        with self.lock:
            self.message = msg

    # ---- actions; each returns (ok, message)
    def stop(self):
        try:
            _json(f"{self.url}/api/mode", {"mode": "stop", "client": "hub"})
            return True, "stopped"
        except CarError as e:
            return False, f"stop failed: {e}"

    def run(self):
        try:
            s = _json(self.url + STATUS_PATH)
            if not s.get("perception", {}).get("running"):
                _json(f"{self.url}/api/camera", {"on": True}, timeout=5.0)
            deadline = time.monotonic() + RUN_WAIT_S
            while True:                   # the camera needs a moment before the first frame
                try:
                    _json(f"{self.url}/api/mode", {"mode": "policy", "client": "hub"})
                    return True, "running"
                except CarError as e:
                    if "camera" not in str(e) or time.monotonic() > deadline:
                        raise
                    time.sleep(0.25)
        except CarError as e:
            return False, f"run failed: {e}"

    def upload_policy(self, text):
        try:
            current = _json(f"{self.url}/api/policy/source")
            r = _json(f"{self.url}/api/policy/source",
                      {"text": text, "base_version": current.get("version"), "reload": True}, timeout=5.0)
            reload = r.get("reload")
            return True, "policy uploaded" + ("" if reload in (None, "reloading") else f" ({reload})")
        except CarError as e:
            return False, f"upload failed: {e}"


# ---------------------------------------------------------------------- the fleet
class Fleet:
    def __init__(self, path):
        self.path = path
        self.lock = threading.Lock()
        self.cars: dict[str, Car] = {}
        self.pool = ThreadPoolExecutor(max_workers=16)
        # map
        self.grid = None                  # {"width", "height", "markers"} from localizer.py
        self.positions = {}               # car id -> {"x", "y", "heading", "markers", "t"}
        for c in self._load():
            self._add(c)

    # ---- car list
    def _load(self):
        if not os.path.exists(self.path):
            return []
        with open(self.path, "rb") as f:
            cfg = tomllib.load(f)
        cars, seen = [], set()
        for i, c in enumerate(cfg.get("cars", [])):
            cid = str(c.get("id") or f"car{i + 1}")
            if cid in seen or "url" not in c:
                sys.exit(f"{self.path}: car #{i + 1}: needs a unique id and a url")
            seen.add(cid)
            cars.append(Car(cid, c.get("name") or cid, c["url"], c.get("color") or PALETTE[i % len(PALETTE)],
                            c.get("host") or ""))
        return cars

    def _save(self):
        def q(s):
            return json.dumps(str(s))     # a JSON string is a valid TOML basic string
        lines = [f"# {APP} car list. Edited by the hub's Cars dialog; you can also edit it by hand",
                 "# while the hub is stopped. host = set when the car joined by itself (join.py).", ""]
        for c in self.list():
            lines += ["[[cars]]", f"id = {q(c.id)}", f"name = {q(c.name)}", f"url = {q(c.url)}",
                      f"color = {q(c.color)}"]
            if c.host:
                lines.append(f"host = {q(c.host)}")
            lines.append("")
        tmp = self.path + ".tmp"
        with open(tmp, "w") as f:
            f.write("\n".join(lines))
        os.replace(tmp, self.path)

    def _add(self, car):
        with self.lock:
            self.cars[car.id] = car

        def loop():
            while car.active:
                t0 = time.monotonic()
                car.poll()
                time.sleep(max(0.05, POLL_S - (time.monotonic() - t0)))
        threading.Thread(target=loop, daemon=True, name=f"poll-{car.id}").start()

    def list(self):
        with self.lock:
            return list(self.cars.values())

    def get(self, cid):
        with self.lock:
            return self.cars.get(cid)

    def _validate(self, data, editing=None):
        name = str(data.get("name") or "").strip()
        url = str(data.get("url") or "").strip()
        if not url:
            raise ValueError("the car's address (URL) is required")
        if not re.match(r"^https?://", url):
            url = "http://" + url
        if not re.match(r"^https?://[^/\s]+(:\d+)?/?$", url):
            raise ValueError("address should look like http://car-name.local:8080")
        if urlparse(url).port is None:
            url = url.rstrip("/") + ":8080"
        color = str(data.get("color") or "").strip()
        if color and not re.match(r"^#[0-9a-fA-F]{6}$", color):
            raise ValueError("color must look like #3b7dd8")
        for c in self.list():
            if c is not editing and c.url == url.rstrip("/"):
                raise ValueError(f"{c.name} already uses this address")
        return name, url, color

    def add(self, data, host=""):
        name, url, color = self._validate(data)
        cars = self.list()
        name = name or urlparse(url).hostname
        base = re.sub(r"[^a-z0-9]+", "-", name.lower()).strip("-") or "car"
        cid, n = base, 2
        while self.get(cid):
            cid, n = f"{base}-{n}", n + 1
        used_colors = {c.color for c in cars}
        color = color or next((p for p in PALETTE if p not in used_colors), PALETTE[len(cars) % len(PALETTE)])
        car = Car(cid, name, url, color, host)
        self._add(car)
        self._save()
        return car

    def edit(self, cid, data):
        car = self.get(cid)
        if car is None:
            raise KeyError(cid)
        name, url, color = self._validate(data, editing=car)
        with car.lock:
            car.configure(name or car.name, url, color or car.color)
            if url.rstrip("/") != car.url:
                car.last_ok, car.status = 0.0, None
        self._save()
        return car

    def remove(self, cid):
        with self.lock:
            car = self.cars.pop(cid, None)
        if car is None:
            raise KeyError(cid)
        car.active = False
        with self.lock:
            self.positions.pop(cid, None)
        self._save()

    def register(self, host, ip, port=8080, name=""):
        """A car announcing itself (join.py). The same car (by hostname) keeps its entry, colour
        and id when its IP changes; returns (car, what happened)."""
        host = str(host or "").strip()[:64]
        if not host:
            raise ValueError("host (the car's hostname) is required")
        port = int(port or 8080)
        if not 0 < port < 65536:
            raise ValueError("bad port")
        url = f"http://{ip}:{port}"
        cars = self.list()
        car = next((c for c in cars if c.host == host), None) or \
            next((c for c in cars if c.url == url and not c.host), None)     # added by hand earlier
        if car is not None and car.url == url and car.host == host:
            return car, "known"
        for c in cars:                       # another entry still has this IP: it is stale (DHCP gave it away)
            if c is not car and c.url == url:
                self.remove(c.id)
        if car is None:
            return self.add({"name": name or host, "url": url}, host=host), "added"
        with car.lock:
            old = car.url
            car.host = host
            car.configure(car.name, url, car.color)
            if old != car.url:
                car.last_ok, car.status = 0.0, None
        self._save()
        return car, "updated" if old != url else "linked"

    def each(self, cars, fn):
        """Run fn(car) on all cars at the same time; {id: {"ok", "message"}}."""
        futures = {c.id: self.pool.submit(fn, c) for c in cars}
        out = {}
        for cid, f in futures.items():
            ok, msg = f.result()
            car = self.get(cid)
            if car:
                car.note(msg)
            out[cid] = {"ok": ok, "message": msg}
        return out

    # ---- map
    def set_positions(self, data):
        grid = data.get("grid") or {}
        now = time.monotonic()
        with self.lock:
            self.grid = {"width": float(grid["width"]), "height": float(grid["height"]),
                         "markers": [{"id": int(m["id"]), "x": float(m["x"]), "y": float(m["y"]),
                                      "facing": float(m.get("facing", 0.0))} for m in grid.get("markers", [])],
                         "t": now}
            for cid, p in (data.get("cars") or {}).items():
                self.positions[str(cid)] = {"x": float(p["x"]), "y": float(p["y"]),
                                            "heading": float(p.get("heading", 0.0)),
                                            "markers": int(p.get("markers", 0)), "t": now}

    def map_view(self):
        now = time.monotonic()
        with self.lock:
            grid = self.grid
            fresh = {c: p for c, p in self.positions.items() if now - p["t"] < POSITION_STALE_S}
        if grid and now - grid["t"] < 10:                 # localizer.py is running: car cameras + ArUco grid
            cars = []
            for c in self.list():
                p = fresh.get(c.id)
                if p:
                    cars.append({"id": c.id, "name": c.name, "color": c.color, "x": p["x"], "y": p["y"],
                                 "heading": p["heading"], "markers": p["markers"], "age": round(now - p["t"], 2)})
            return {"source": "aruco", "track": {"width": grid["width"], "height": grid["height"]},
                    "landmarks": grid["markers"], "cars": cars, "total": len(self.list())}
        cars = []
        for c in self.list():                              # simulated cars: place by distance driven
            v = c.view()
            sim = (v["status"] or {}).get("sim")
            if sim:
                x, y, h = _sim_place(sim.get("distance_m", 0.0), sim.get("lateral_error_m", 0.0))
                cars.append({"id": c.id, "name": c.name, "color": c.color, "x": x, "y": y, "heading": h, "age": 0})
        if cars:
            return {"source": "sim", "track": SIM_TRACK,
                    "oval": {"straight": _SIM_STRAIGHT, "radius": _SIM_R}, "cars": cars}
        return {"source": None, "track": None, "cars": []}


# ---------------------------------------------------------------------- network
def lan_addresses():
    """This computer's IPv4 addresses that cars can reach (best effort, no packets sent)."""
    found = []
    for probe in ("10.255.255.255", "192.168.255.255", "172.31.255.255"):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            s.connect((probe, 1))                         # UDP connect only picks the outgoing interface
            ip = s.getsockname()[0]
            if not ip.startswith("127.") and ip not in found:
                found.append(ip)
        except OSError:
            pass
        finally:
            s.close()
    return found


def beacon(port):
    """Broadcast "here is the hub" so join.py on the cars finds it without being told the address."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    msg = json.dumps({"app": "trackside", "port": port}).encode()
    while True:
        try:
            s.sendto(msg, ("255.255.255.255", BEACON_PORT))
        except OSError:                                   # no network right now; keep trying
            pass
        time.sleep(BEACON_EVERY_S)


# ---------------------------------------------------------------------- HTTP server
class Handler(BaseHTTPRequestHandler):
    fleet: Fleet = None
    password: str = ""
    beacon: bool = True
    protocol_version = "HTTP/1.1"
    server_version = "trackside"

    def log_message(self, fmt, *args):
        pass

    def _send(self, status, body, ctype):
        self.send_response(status)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def _json(self, obj, status=HTTPStatus.OK):
        self._send(status, json.dumps(obj).encode(), "application/json")

    def _error(self, status, msg):
        self._json({"ok": False, "error": msg}, status)

    def _authorized(self):
        if not self.password:
            return True
        h = self.headers.get("Authorization", "")
        if h.startswith("Basic "):
            try:
                _, _, pw = base64.b64decode(h[6:]).decode().partition(":")
                if hmac.compare_digest(pw, self.password):
                    return True
            except (ValueError, UnicodeDecodeError):
                pass
        self.send_response(HTTPStatus.UNAUTHORIZED)
        self.send_header("WWW-Authenticate", f'Basic realm="{APP}"')
        self.send_header("Content-Length", "0")
        self.end_headers()
        return False

    def _raw_body(self):
        length = int(self.headers.get("Content-Length") or 0)
        if length > MAX_BODY:
            raise ValueError("body too large")
        return self.rfile.read(length) if length else b""

    def _body(self):
        raw = self._raw_body()
        data = json.loads(raw) if raw else {}
        if not isinstance(data, dict):
            raise ValueError("expected a JSON object")
        return data

    def do_GET(self):
        if not self._authorized():
            return
        path = urlparse(self.path).path
        fleet = self.fleet
        try:
            if path in ("/", "/index.html"):
                with open(os.path.join(HERE, "index.html"), "rb") as f:
                    return self._send(HTTPStatus.OK, f.read(), "text/html; charset=utf-8")
            if path == "/api/cars":
                return self._json({"cars": [c.view() for c in fleet.list()]})
            if path == "/api/map":
                return self._json(fleet.map_view())
            if path == "/api/info":
                return self._json({"addresses": lan_addresses(), "port": self.server.server_address[1],
                                   "beacon": self.beacon})
            parts = path.strip("/").split("/")          # car/<id>/frame/<name>.jpg
            if len(parts) == 4 and parts[0] == "car" and parts[2] == "frame" and parts[3].endswith(".jpg"):
                car, name = fleet.get(parts[1]), parts[3][:-4]
                if car is None or name not in FRAMES:
                    return self._error(HTTPStatus.NOT_FOUND, "not found")
                try:
                    body, ctype = _request(f"{car.url}/frame/{name}.jpg")
                except CarError as e:
                    return self._error(HTTPStatus.BAD_GATEWAY, str(e))
                return self._send(HTTPStatus.OK, body, ctype or "image/jpeg")
            self._error(HTTPStatus.NOT_FOUND, "not found")
        except (BrokenPipeError, ConnectionResetError):
            pass

    def do_POST(self):
        if not self._authorized():
            return
        path = urlparse(self.path).path
        fleet = self.fleet
        parts = path.strip("/").split("/")
        try:
            data = self._body()
            if path == "/api/positions":
                fleet.set_positions(data)
                return self._json({"ok": True})
            if path == "/api/register":
                car, what = fleet.register(data.get("host"), self.client_address[0], data.get("port"), data.get("name"))
                if what != "known":
                    print(f"car {what}: {car.name} ({car.host}) at {car.url}", flush=True)
                return self._json({"ok": True, "id": car.id, "name": car.name, "url": car.url, "result": what})
            if path == "/api/cars":
                return self._json({"ok": True, "car": fleet.add(data).view()})
            if path == "/api/probe":
                url = str(data.get("url") or "").strip()
                if not re.match(r"^https?://", url):
                    url = "http://" + url
                if urlparse(url).port is None:
                    url = url.rstrip("/") + ":8080"
                try:
                    s = _json(url.rstrip("/") + STATUS_PATH, timeout=2.0)
                    return self._json({"ok": True, "url": url, "car": s.get("car"), "simulated": s.get("simulated")})
                except CarError as e:
                    return self._json({"ok": False, "url": url, "error": str(e)})
            if len(parts) == 3 and parts[:2] == ["api", "all"] and parts[2] in ("run", "stop"):
                # stop goes to every car (even ones that look offline); run only to online ones
                cars = fleet.list()
                if parts[2] == "run":
                    cars = [c for c in cars if c.online()]
                fn = Car.stop if parts[2] == "stop" else Car.run
                return self._json({"ok": True, "results": fleet.each(cars, fn)})
            if len(parts) == 4 and parts[:2] == ["api", "cars"] and parts[3] in ("run", "stop"):
                car = fleet.get(parts[2])
                if car is None:
                    return self._error(HTTPStatus.NOT_FOUND, "unknown car")
                res = fleet.each([car], Car.stop if parts[3] == "stop" else Car.run)[car.id]
                return self._json(res, HTTPStatus.OK if res["ok"] else HTTPStatus.BAD_GATEWAY)
            if path == "/api/policy":
                text, ids = data.get("text"), data.get("cars") or []
                if not isinstance(text, str) or not text.strip():
                    return self._error(HTTPStatus.BAD_REQUEST, "empty policy file")
                cars = [c for c in (fleet.get(i) for i in ids) if c]
                if not cars:
                    return self._error(HTTPStatus.BAD_REQUEST, "no cars selected")
                return self._json({"ok": True, "results": fleet.each(cars, lambda c: c.upload_policy(text))})
            self._error(HTTPStatus.NOT_FOUND, "not found")
        except (ValueError, TypeError, KeyError) as e:
            self._error(HTTPStatus.BAD_REQUEST, str(e))
        except (BrokenPipeError, ConnectionResetError):
            pass

    def do_PUT(self):
        if not self._authorized():
            return
        parts = urlparse(self.path).path.strip("/").split("/")
        if len(parts) != 3 or parts[:2] != ["api", "cars"]:
            return self._error(HTTPStatus.NOT_FOUND, "not found")
        try:
            return self._json({"ok": True, "car": self.fleet.edit(parts[2], self._body()).view()})
        except KeyError:
            return self._error(HTTPStatus.NOT_FOUND, "unknown car")
        except (ValueError, TypeError) as e:
            return self._error(HTTPStatus.BAD_REQUEST, str(e))

    def do_DELETE(self):
        if not self._authorized():
            return
        parts = urlparse(self.path).path.strip("/").split("/")
        if len(parts) != 3 or parts[:2] != ["api", "cars"]:
            return self._error(HTTPStatus.NOT_FOUND, "not found")
        try:
            self.fleet.remove(parts[2])
            return self._json({"ok": True})
        except KeyError:
            return self._error(HTTPStatus.NOT_FOUND, "unknown car")


def main():
    ap = argparse.ArgumentParser(description=f"{APP}: one page for a fleet of cars")
    ap.add_argument("--cars", default=os.path.join(HERE, "cars.toml"), help="car list (default: cars.toml; created if missing)")
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=9000)
    ap.add_argument("--no-beacon", action="store_true", help=f"do not broadcast the hub on UDP {BEACON_PORT}")
    args = ap.parse_args()

    fleet = Fleet(args.cars)
    Handler.fleet = fleet
    Handler.password = os.environ.get("HUB_PASSWORD", "")
    Handler.beacon = not args.no_beacon
    server = ThreadingHTTPServer((args.host, args.port), Handler)
    server.daemon_threads = True
    if Handler.beacon:
        threading.Thread(target=beacon, args=(args.port,), daemon=True, name="beacon").start()
    print(f"{APP}: {len(fleet.list())} cars from {args.cars}")
    for c in fleet.list():
        print(f"  {c.id:<12} {c.name:<20} {c.url}" + (f"  (joined as {c.host})" if c.host else ""))
    print(f"open http://localhost:{args.port}" + ("  (password required)" if Handler.password else ""))
    addrs = lan_addresses()
    if addrs:
        print("cars and other computers reach this hub at: " +
              ", ".join(f"http://{a}:{args.port}" for a in addrs))
    print("on each car: python3 join.py" + ("" if Handler.beacon else f" --hub http://{addrs[0] if addrs else '<this computer>'}:{args.port}"),
          flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nbye")


if __name__ == "__main__":
    main()
