#!/usr/bin/env python3
import json, time, threading
import paho.mqtt.client as mqtt
from rpi_hardware_pwm import HardwarePWM
import os

BROKER = "localhost"
TOPIC  = "car/cmd"
CONFIG_TOPIC = "car/config"       # live PWM calibration updates (e.g. from the web UI)
STATUS_TOPIC = "car/status/pwm"   # heartbeat + current PWM outputs
FREQ_HZ = 150

# Channels (check wiring)
STEER_CH    = 0
THROTTLE_CH = 1

# Behavior
timeout_s = 0.5   # safety: idle if no packets

# Shared state
last_rx = 0.0
latest_cmd = {"steer": 0.0, "throttle": 0.0, "reverse": False, "rev_power": 0.0, "ts": 0.0}
config = None
CONFIG_KEYS = ('steer_center', 'steer_range', 'throttle_rest', 'throttle_start', 'throttle_max')

def load_config(path = '~/pwm_config.json'):
    # Placeholder for loading configuration if needed in future
    try :
        with open(os.path.expanduser(path), 'r') as f:
            config = json.load(f)
            return config
    except:
        # return default config
        return {'steer_center': 22.5,
                'steer_range':2.7,
                'throttle_rest':21,
                'throttle_start':0.5,
                'throttle_max':1,
                }


def map_steer_to_pwm(steer, config):
    s = max(-1.0, min(1.0, steer))
    return config['steer_center'] + s * config['steer_range']

def map_throttle_fwd(throttle, config):
    t = max(0.0, min(1.0, throttle))
    if t <= 0.05:
        return config['throttle_rest']
    return config['throttle_rest'] - config['throttle_start'] - (config['throttle_max'] - config['throttle_start']) * t

def map_throttle_rev(power, config):
    p = max(0.0, min(1.0, power))
    if p <= 0.05:
        return config['throttle_rest']
    return config['throttle_rest'] + config['throttle_start'] + (config['throttle_max'] - config['throttle_start']) * p

def on_connect(client, userdata, flags, rc):
    print("MQTT connected:", rc)
    client.subscribe(TOPIC, qos=0)
    client.subscribe(CONFIG_TOPIC, qos=0)

def apply_config_update(data):
    # Swap in a new dict so the control loop never sees a half-updated config
    global config
    new_config = dict(config)
    for key in CONFIG_KEYS:
        if key in data:
            value = float(data[key])
            if not (0.0 <= value <= 100.0):
                raise ValueError(f"{key}={value} out of range")
            new_config[key] = value
    config = new_config
    print(f"\n[config] {config}")

def on_message(client, userdata, msg):
    global last_rx, latest_cmd
    try:
        data = json.loads(msg.payload.decode("utf-8"))
        if msg.topic == CONFIG_TOPIC:
            apply_config_update(data)
            return
        # merge defaults so missing fields don't crash
        latest_cmd.update({
            "steer": float(data.get("steer", 0.0)),
            "throttle": float(data.get("throttle", 0.0)),
            "reverse": bool(data.get("reverse", False)),
            "rev_power": float(data.get("rev_power", 0.0)),
            "ts": float(data.get("ts", time.time()))
        })
        last_rx = time.time()
        # Print the raw signal we just received
        print(f"\n[recv] {latest_cmd}")
    except Exception as e:
        print("Bad payload:", e)

def control_loop(client, verbose=False):
    pwm_steer = HardwarePWM(pwm_channel=STEER_CH, hz=FREQ_HZ)
    pwm_throt = HardwarePWM(pwm_channel=THROTTLE_CH, hz=FREQ_HZ)
    pwm_steer.start(config['steer_center'])
    pwm_throt.start(config['throttle_rest'])

    try:
        last_print = 0.0
        last_status = 0.0
        while True:
            now = time.time()
            cfg = config
            # Safety fallback
            if (now - last_rx) > timeout_s:
                steer_pwm = cfg['steer_center']
                thr_pwm = cfg['throttle_rest']
                mode = "idle(timeout)"
            else:
                s = latest_cmd["steer"]
                steer_pwm = map_steer_to_pwm(s, cfg)

                if latest_cmd["reverse"]:
                    thr_pwm = map_throttle_rev(latest_cmd["rev_power"], cfg)
                    mode = "REV"
                else:
                    thr_pwm = map_throttle_fwd(latest_cmd["throttle"], cfg)
                    mode = "FWD"

            # Print PWM outputs at ~10 Hz
            if now - last_print >= 0.1 and verbose:
                print(f"[pwm] mode={mode}  steer_pwm={steer_pwm:.2f}  throttle_pwm={thr_pwm:.2f}")
                last_print = now

            pwm_steer.change_duty_cycle(steer_pwm)
            pwm_throt.change_duty_cycle(thr_pwm)

            # Heartbeat at ~10 Hz so monitors (web UI) can see the subscriber is alive
            if now - last_status >= 0.1:
                status = {"mode": mode, "steer_pwm": steer_pwm, "throttle_pwm": thr_pwm,
                          "cmd": latest_cmd, "cmd_age": now - last_rx, "config": cfg, "ts": now}
                client.publish(STATUS_TOPIC, json.dumps(status), qos=0)
                last_status = now

            time.sleep(0.02)
    except KeyboardInterrupt:
        pass
    finally:
        pwm_steer.stop()
        pwm_throt.stop()

def main():
    global config
    config = load_config()

    client = mqtt.Client(client_id="pi-pwm-sub", clean_session=True)
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(BROKER, 1883, 60)

    t = threading.Thread(target=client.loop_forever, daemon=True)
    t.start()
    control_loop(client, verbose=False)

if __name__ == "__main__":
    main()
