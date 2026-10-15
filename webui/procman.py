"""Start/stop script/pwm_subscriber.py from the UI.

The subscriber is started in its own session so it keeps running (and keeps
the car in its safe idle fallback) even if the web server stops. Whether it is
alive is judged from its MQTT heartbeat, so an instance started by hand or by
systemd is detected too; only an instance started here can be stopped here.
"""
import os
import signal
import subprocess
import sys
import time

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


class SubscriberProcess:
    def __init__(self, log_dir):
        self.log_dir = log_dir
        self.pid_path = os.path.join(log_dir, "pwm_subscriber.pid")
        self.log_path = os.path.join(log_dir, "pwm_subscriber.log")

    def pid(self):
        try:
            with open(self.pid_path) as f:
                pid = int(f.read().strip())
            try:
                if os.waitpid(pid, os.WNOHANG)[0] == pid:   # our child exited (zombie reaped)
                    return None
            except ChildProcessError:
                pass   # not our child (e.g. server restarted); fall through to a liveness check
            os.kill(pid, 0)
            return pid
        except (OSError, ValueError):
            return None

    def start(self):
        if self.pid():
            return self.pid()
        os.makedirs(self.log_dir, exist_ok=True)
        out = open(self.log_path, "ab")
        proc = subprocess.Popen(
            [sys.executable, "-u", os.path.join(REPO, "script", "pwm_subscriber.py")],
            cwd=REPO, stdout=out, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL,
            start_new_session=True,
        )
        out.close()
        with open(self.pid_path, "w") as f:
            f.write(str(proc.pid))
        return proc.pid

    def stop(self, timeout=3.0):
        pid = self.pid()
        if not pid:
            return False
        os.kill(pid, signal.SIGINT)   # KeyboardInterrupt -> PWM channels are stopped cleanly
        deadline = time.time() + timeout
        while time.time() < deadline:
            try:
                os.kill(pid, 0)
            except OSError:
                break
            try:
                os.waitpid(pid, os.WNOHANG)   # reap if it is our child
            except ChildProcessError:
                pass
            time.sleep(0.1)
        else:
            os.kill(pid, signal.SIGTERM)
        try:
            os.remove(self.pid_path)
        except OSError:
            pass
        return True

    def tail(self, lines=60):
        try:
            with open(self.log_path, "rb") as f:
                f.seek(0, os.SEEK_END)
                f.seek(max(0, f.tell() - 64 * 1024))
                text = f.read().decode("utf-8", "replace")
        except OSError:
            return ""
        # The subscriber prints a [recv] line per command; collapse those so useful lines stay visible.
        out = [l for l in text.splitlines() if l.strip() and not l.startswith("[recv]")]
        return "\n".join(out[-lines:])
