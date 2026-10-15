"""Persistent settings for the web UI and the shared PWM calibration file."""
import copy
import json
import logging
import os
import threading

log = logging.getLogger(__name__)

# Same file and defaults as script/pwm_subscriber.py and utlis/joy_test.py
PWM_CONFIG_PATH = os.path.expanduser("~/pwm_config.json")
PWM_DEFAULTS = {
    "steer_center": 22.5,
    "steer_range": 2.7,
    "throttle_rest": 21,
    "throttle_start": 0.5,
    "throttle_max": 1,
}
PWM_KEYS = tuple(PWM_DEFAULTS)

SETTINGS_PATH = os.path.expanduser("~/dr_rpi_webui.json")

# Presets reproduce the behaviour of the existing control scripts.
PRESETS = {
    "lane_dist_light": {
        "label": "Single lane + distance + traffic light",
        "script": "script/lane_dist_light_ctrl.py",
        "pipeline": {
            "lane": "single", "min_lane_points": 20,
            "distance": {"enabled": True, "stop_cm": 20, "slow_range_cm": 50},
            "light": {"enabled": True},
        },
        "camera": {"auto_exposure": False, "exposure_us": 5000, "gain": 1.0},
    },
    "lane_dist": {
        "label": "Single lane + distance",
        "script": "script/lane_dist_ctrl.py",
        "pipeline": {
            "lane": "single", "min_lane_points": 20,
            "distance": {"enabled": True, "stop_cm": 15, "slow_range_cm": 30},
            "light": {"enabled": False},
        },
        "camera": {"auto_exposure": False, "exposure_us": 10000, "gain": 3.0},
    },
    "two_lane_dist": {
        "label": "Two lane (yellow/white) + distance",
        "script": "script/two_lane_dist_ctrl.py",
        "pipeline": {
            "lane": "two", "min_lane_points": 1,
            "distance": {"enabled": True, "stop_cm": 15, "slow_range_cm": 30},
            "light": {"enabled": False},
        },
        "camera": {"auto_exposure": False, "exposure_us": 10000, "gain": 3.0},
    },
}

DEFAULTS = {
    "manual": {
        "max_throttle": 0.6,     # cap applied to forward throttle in manual mode
        "max_reverse": 0.5,      # cap applied to reverse power in manual mode
        "steer_limit": 1.0,      # scale applied to steering in manual mode
        # Joystick convention in utlis/joy_test.py: stick right -> duty below center,
        # i.e. a negative `steer` on car/cmd. So UI "right" is sent as negative steer.
        "invert_steer": True,
    },
    "camera": {
        "rotate180": True,
        "auto_exposure": False,
        "exposure_us": 5000,
        "gain": 1.0,
        "fps": 30,
    },
    "pipeline": {
        "preset": "lane_dist_light",
        "lane": "single",                # "single" (src/single_lane.py) | "two" (src/two_lane_poly.py)
        "throttle": 0.6,                 # --throttle of the scripts
        "min_lane_points": 20,           # stop if the centerline has fewer points
        "stanley": {"k_e": -3.0, "k_v": 1.2, "wheelbase": 0.13, "ff_gain": -2.0, "max_steer_deg": 30.0},
        "distance": {"enabled": True, "stop_cm": 20, "slow_range_cm": 50, "keep_missed_frames": 5},
        "light": {"enabled": True, "marker_id": 5, "hold_seconds": 0.5, "stop_on_yellow": True},
        "single": {
            "red1_lo": [0, 50, 50], "red1_hi": [20, 255, 255],
            "red2_lo": [150, 50, 50], "red2_hi": [180, 255, 255],
            "area_min": 100, "px_to_m": 0.002,
        },
        "two": {
            # = center -/+ tolerance as written in src/two_lane_poly.py
            "yellow_lo": [15, 139, 87], "yellow_hi": [45, 239, 187],
            "white_lo": [24, 36, 96], "white_hi": [44, 76, 176],
            "area_min": 20, "px_to_m": 0.0025,
        },
        "light_hsv": {
            "red_lo": [160, 200, 100], "red_hi": [180, 255, 255],
            "yellow_lo": [10, 100, 100], "yellow_hi": [35, 255, 255],
            "green_lo": [40, 80, 120], "green_hi": [85, 255, 255],
        },
    },
}


def deep_merge(base, update):
    """Return a copy of `base` with `update` merged in. Only known keys are accepted."""
    out = copy.deepcopy(base)
    for key, value in update.items():
        if key not in out:
            continue
        if isinstance(out[key], dict) and isinstance(value, dict):
            out[key] = deep_merge(out[key], value)
        else:
            out[key] = _coerce(out[key], value)
    return out


def _coerce(template, value):
    """Coerce `value` to the type of `template` (bool/int/float/str/list of numbers)."""
    if isinstance(template, bool):
        return bool(value)
    if isinstance(template, (int, float)) and not isinstance(template, bool):
        return float(value) if isinstance(template, float) or isinstance(value, float) else int(value)
    if isinstance(template, list):
        if not isinstance(value, list) or len(value) != len(template):
            raise ValueError(f"expected list of {len(template)} values")
        return [max(0, min(255, int(v))) for v in value]
    if isinstance(template, str):
        return str(value)
    return value


class Settings:
    def __init__(self, path=SETTINGS_PATH):
        self.path = path
        self.lock = threading.Lock()
        self.data = copy.deepcopy(DEFAULTS)
        try:
            with open(path) as f:
                self.data = deep_merge(DEFAULTS, json.load(f))
        except FileNotFoundError:
            pass
        except Exception as e:
            log.warning("Could not read %s (%s); using defaults", path, e)

    def snapshot(self):
        with self.lock:
            return copy.deepcopy(self.data)

    def update(self, partial):
        with self.lock:
            self.data = deep_merge(self.data, partial)
            self._save()
            return copy.deepcopy(self.data)

    def apply_preset(self, name):
        preset = PRESETS[name]
        partial = {"pipeline": dict(preset["pipeline"], preset=name), "camera": preset["camera"]}
        return self.update(partial)

    def _save(self):
        tmp = self.path + ".tmp"
        try:
            with open(tmp, "w") as f:
                json.dump(self.data, f, indent=2)
            os.replace(tmp, self.path)
        except Exception as e:
            log.warning("Could not save settings: %s", e)


def load_pwm_config():
    try:
        with open(PWM_CONFIG_PATH) as f:
            cfg = json.load(f)
        return {k: float(cfg.get(k, PWM_DEFAULTS[k])) for k in PWM_KEYS}
    except Exception:
        return dict(PWM_DEFAULTS)


def validate_pwm_config(cfg):
    out = {}
    for key in PWM_KEYS:
        value = float(cfg[key])
        if not 0.0 <= value <= 100.0:
            raise ValueError(f"{key} must be a duty cycle between 0 and 100")
        out[key] = round(value, 3)
    return out


def save_pwm_config(cfg):
    with open(PWM_CONFIG_PATH, "w") as f:
        json.dump(cfg, f)
