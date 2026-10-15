"""Run the DeepRacer web UI:  python -m webui  (from the repository root)."""
import argparse
import logging
import os
import socket
import sys

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def parse_args():
    p = argparse.ArgumentParser(description="DeepRacer browser control panel")
    p.add_argument("--host", default="0.0.0.0", help="interface to listen on (default: all)")
    p.add_argument("--port", type=int, default=8080)
    p.add_argument("--camera", default="auto",
                   help="auto | picamera | sim | <webcam index> | <video file> (default: auto = Pi camera, else simulator)")
    p.add_argument("--mqtt-host", default="localhost")
    p.add_argument("--mqtt-port", type=int, default=1883)
    p.add_argument("--log-dir", default=os.path.join(REPO, "logs"), help="recordings + subscriber log")
    p.add_argument("--settings", default=None, help="settings file (default: ~/dr_rpi_webui.json)")
    return p.parse_args()


def main():
    args = parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    logging.getLogger("aiohttp.access").setLevel(logging.WARNING)
    if REPO not in sys.path:
        sys.path.insert(0, REPO)

    from . import compat
    compat.ensure_camera_modules()   # must run before src/ is imported

    from aiohttp import web
    from .bus import Bus
    from .car import Car
    from .server import App
    from .settings import Settings, SETTINGS_PATH

    os.makedirs(args.log_dir, exist_ok=True)
    settings = Settings(args.settings or SETTINGS_PATH)
    bus = Bus(args.mqtt_host, args.mqtt_port)
    bus.start()
    car = Car(settings, bus, args.camera, args.log_dir)
    car.start()

    app = App(car, settings, bus, args.log_dir).build()
    try:
        ip = socket.gethostbyname(socket.gethostname())
    except OSError:
        ip = "<pi-ip>"
    print(f"\n  DeepRacer UI:  http://{ip}:{args.port}   (or http://localhost:{args.port})\n", flush=True)
    try:
        web.run_app(app, host=args.host, port=args.port, print=None)
    finally:
        car.shutdown()
        bus.stop()


if __name__ == "__main__":
    main()
