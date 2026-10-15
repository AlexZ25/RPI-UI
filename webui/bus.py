"""MQTT link to script/pwm_subscriber.py."""
import json
import logging
import threading
import time

from .compat import make_mqtt_client

log = logging.getLogger(__name__)

CMD_TOPIC = "car/cmd"
CONFIG_TOPIC = "car/config"
STATUS_TOPIC = "car/status/pwm"
SOURCE_TAG = "webui"   # extra payload field; ignored by pwm_subscriber


class Bus:
    def __init__(self, host="localhost", port=1883):
        self.host, self.port = host, port
        self.connected = False
        self.error = None
        self.client = None
        self.lock = threading.Lock()
        self.last_sent = {"steer": 0.0, "throttle": 0.0, "reverse": False, "rev_power": 0.0, "ts": 0.0}
        self.pwm_status = None          # last heartbeat from pwm_subscriber
        self.pwm_status_rx = 0.0
        self.foreign_cmd_rx = 0.0       # last car/cmd message not sent by us
        self.foreign_cmd = None

    def start(self):
        try:
            self.client = make_mqtt_client("dr-webui")
        except ImportError:
            self.error = "paho-mqtt not installed"
            log.warning("paho-mqtt not installed; running without MQTT")
            return
        self.client.on_connect = self._on_connect
        self.client.on_disconnect = self._on_disconnect
        self.client.on_message = self._on_message
        self.client.reconnect_delay_set(min_delay=1, max_delay=5)
        try:
            self.client.connect_async(self.host, self.port, 30)
            self.client.loop_start()
        except Exception as e:
            self.error = str(e)
            log.warning("MQTT connect failed: %s", e)

    def stop(self):
        if self.client is not None:
            self.client.loop_stop()
            try:
                self.client.disconnect()
            except Exception:
                pass

    # paho 1.x and 2.x pass different callback arguments; only positions 0-1 are shared.
    def _on_connect(self, client, *args):
        self.connected = True
        self.error = None
        client.subscribe(STATUS_TOPIC, qos=0)
        client.subscribe(CMD_TOPIC, qos=0)
        log.info("MQTT connected to %s:%s", self.host, self.port)

    def _on_disconnect(self, client, *args):
        self.connected = False
        log.warning("MQTT disconnected")

    def _on_message(self, client, userdata, msg):
        try:
            data = json.loads(msg.payload.decode("utf-8"))
        except Exception:
            return
        now = time.time()
        if msg.topic == STATUS_TOPIC:
            self.pwm_status = data
            self.pwm_status_rx = now
        elif msg.topic == CMD_TOPIC and data.get("src") != SOURCE_TAG:
            self.foreign_cmd = data
            self.foreign_cmd_rx = now

    def publish_cmd(self, steer=0.0, throttle=0.0, reverse=False, rev_power=0.0):
        cmd = {
            "steer": max(-1.0, min(1.0, float(steer))),
            "throttle": max(0.0, min(1.0, float(throttle))),
            "reverse": bool(reverse),
            "rev_power": max(0.0, min(1.0, float(rev_power))),
            "ts": time.time(),
        }
        with self.lock:
            self.last_sent = cmd
        if self.client is not None and self.connected:
            self.client.publish(CMD_TOPIC, json.dumps(dict(cmd, src=SOURCE_TAG)), qos=0)
        return cmd

    def publish_config(self, cfg):
        if self.client is None or not self.connected:
            return False
        self.client.publish(CONFIG_TOPIC, json.dumps(cfg), qos=1)
        return True

    def status(self):
        now = time.time()
        pwm_age = now - self.pwm_status_rx if self.pwm_status_rx else None
        return {
            "connected": self.connected,
            "host": f"{self.host}:{self.port}",
            "error": self.error,
            "pwm_alive": pwm_age is not None and pwm_age < 1.0,
            "pwm_age": pwm_age,
            "pwm": self.pwm_status if pwm_age is not None and pwm_age < 1.0 else None,
            "foreign_publisher": self.foreign_cmd_rx > 0 and now - self.foreign_cmd_rx < 1.0,
            "last_sent": self.last_sent,
        }
