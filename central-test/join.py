"""Run on a car: tell the Trackside hub (on the central computer) that this car is here.

The hub then lists the car at http://<this car's IP>:8080 (its dashboard). The car
re-announces itself every few seconds, so it shows up again after the hub restarts and
its entry follows it when its IP changes. Standard library only; nothing else on the car
changes.

    python3 join.py                                  # find the hub by its UDP broadcast
    python3 join.py --hub http://192.168.1.10:9000   # or say where it is
    python3 join.py --name "Team Red"                # name shown in the hub (first join only)
    HUB_PASSWORD=secret python3 join.py              # if the hub was started with a password

To start it at boot, see trackside-join.service.
"""
from __future__ import annotations

import argparse
import base64
import json
import os
import socket
import time
import urllib.error
import urllib.request

BEACON_PORT = 9999          # must match hub.py
EVERY_S = 5.0


def find_hub(timeout):
    """Wait for the hub's broadcast; returns its URL or None."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(("", BEACON_PORT))
    s.settimeout(timeout)
    try:
        while True:
            data, (ip, _) = s.recvfrom(1024)
            try:
                msg = json.loads(data)
            except ValueError:
                continue
            if msg.get("app") == "trackside":
                return f"http://{ip}:{int(msg['port'])}"
    except socket.timeout:
        return None
    finally:
        s.close()


def register(hub, host, name, port, password):
    body = json.dumps({"host": host, "name": name, "port": port}).encode()
    req = urllib.request.Request(f"{hub}/api/register", data=body, method="POST",
                                 headers={"Content-Type": "application/json"})
    if password:
        req.add_header("Authorization", "Basic " + base64.b64encode(f"{host}:{password}".encode()).decode())
    with urllib.request.urlopen(req, timeout=3.0) as r:
        return json.loads(r.read())


def main():
    ap = argparse.ArgumentParser(description="Announce this car to the Trackside hub")
    ap.add_argument("--hub", help="hub URL, e.g. http://192.168.1.10:9000 (default: find it on the network)")
    ap.add_argument("--name", default="", help="name in the hub when the car is first added (default: hostname)")
    ap.add_argument("--host", default=socket.gethostname().split(".")[0],
                    help="this car's unique name for the hub (default: hostname)")
    ap.add_argument("--port", type=int, default=8080, help="port of this car's dashboard")
    args = ap.parse_args()
    password = os.environ.get("HUB_PASSWORD", "")

    hub = args.hub.rstrip("/") if args.hub else None
    if hub and "://" not in hub:
        hub = "http://" + hub
    last = None
    while True:
        if hub is None:
            print(f"looking for the hub (UDP broadcast on port {BEACON_PORT})...", flush=True)
            found = find_hub(10.0)
            if found is None:
                continue
            hub = found
        try:
            r = register(hub, args.host, args.name, args.port, password)
            state = f"in the hub {hub} as {r['name']} ({r['url']})"
        except urllib.error.HTTPError as e:
            state = f"hub {hub} refused: {e.code} {'wrong or missing HUB_PASSWORD' if e.code == 401 else e.reason}"
        except (urllib.error.URLError, OSError, ValueError, KeyError) as e:
            state = f"hub {hub} not reachable: {getattr(e, 'reason', e)}"
            if not args.hub:
                hub = None               # it may have moved: listen for the broadcast again
        if state != last:
            print(state, flush=True)
            last = state
        time.sleep(EVERY_S)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
