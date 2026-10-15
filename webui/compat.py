"""Hardware/library compatibility helpers.

The `src/` modules import picamera2/libcamera at module level. On a dev machine
(no Pi camera stack) we install lightweight stand-ins so the vision code can
still be imported and exercised with the simulated or webcam camera.
"""
import sys
import types

HAVE_PICAMERA2 = False


def ensure_camera_modules():
    """Import picamera2/libcamera, or register stubs if they are unavailable."""
    global HAVE_PICAMERA2
    try:
        import picamera2  # noqa: F401
        import libcamera  # noqa: F401
        HAVE_PICAMERA2 = True
        return True
    except Exception:
        pass

    class _Unavailable:
        def __init__(self, *args, **kwargs):
            raise RuntimeError("picamera2 is not available on this machine")

    class Transform:
        def __init__(self, *args, **kwargs):
            self.args, self.kwargs = args, kwargs

    if "picamera2" not in sys.modules:
        mod = types.ModuleType("picamera2")
        mod.Picamera2 = _Unavailable
        mod.__dr_stub__ = True
        sys.modules["picamera2"] = mod
    if "libcamera" not in sys.modules:
        mod = types.ModuleType("libcamera")
        mod.Transform = Transform
        mod.__dr_stub__ = True
        sys.modules["libcamera"] = mod
    HAVE_PICAMERA2 = False
    return False


def make_mqtt_client(client_id):
    """Create a paho client that works on both paho-mqtt 1.x and 2.x."""
    import paho.mqtt.client as mqtt
    if hasattr(mqtt, "CallbackAPIVersion"):
        return mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=client_id, clean_session=True)
    return mqtt.Client(client_id=client_id, clean_session=True)
