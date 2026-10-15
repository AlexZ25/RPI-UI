# DR-RPI

This repository provides a control interface for operating the **AWS DeepRacer lower chassis using a Raspberry Pi**, along with several lightweight control algorithms such as **lane following, distance keeping, and traffic light detection**.

Instead of ROS, the system uses **MQTT** as a lightweight internal communication framework and **native Raspberry Pi PWM control** to interface with the DeepRacer lower frame.

---

# Hardware Requirements

- Raspberry Pi (tested with Raspberry Pi 5)
- AWS DeepRacer lower chassis
- MicroSD card
- Logitech **F310** game controller (for manual testing)
- Optional sensors (e.g. **Pi Camera**, **BNO055 IMU**)

---

# First-Time Setup

## 1. Install Raspberry Pi OS

Install the latest **Raspberry Pi OS** on a microSD card using the official Raspberry Pi Imager:

https://www.raspberrypi.com/software/

---

## MQTT Setup

Since **ROS is relatively heavy and not always convenient on Raspberry Pi**, this project uses **MQTT** as a lightweight message-based communication framework.

Install the MQTT broker:

```bash
sudo apt update
sudo apt install -y mosquitto mosquitto-clients
sudo systemctl enable mosquitto
```
Edit the MQTT configuration:
```
sudo nano /etc/mosquitto/mosquitto.conf
```
Add the following lines:
```
listener 1883
allow_anonymous true
```
Start the MQTT service:
```
sudo systemctl start mosquitto
```

### PWM Setup
PWM pins are used to directly control the DeepRacer **driving motor** and **steering servo moter**.

Edit the firmware configuration:
```
sudo nano /boot/firmware/config.txt
```
Add:
```
dtoverlay=pwm-2chan,pin=12,func=4,pin2=13,func2=4
```
Reboot the system:
```
sudo reboot
```

### I2C Setup
Some sensors such as the BNO055 IMU communicate through the I2C interface.

Enable I2C:
```
sudo raspi-config
```
Navigate to Interface Options → I2C → Enable


## Python Virtual Env
Recent Raspberry Pi OS versions restrict installing packages directly into the system Python environment.
Therefore we use a Python virtual environment (`venv`).

Install venv support:
```
sudo apt update
sudo apt install python3-venv python3-full
```

Create a virtual environment:
```
python3 -m venv --system-site-packages ~/venv
```
Activate the environment:
```
source ~/venv/bin/activate
```
Upgrade pip:
```
pip install --upgrade pip
```
Install required packages:
```
pip install rpi_hardware_pwm
pip install paho-mqtt
pip install opencv-python
pip install adafruit-circuitpython-bno055
```

## Using the Vehicle

⚠ **Safety Warning**

The `pwm_subscriber` **must remain running whenever the physical power switch on the lower frame is ON**.

The `pwm_subscriber` automatically enters a **stationary fallback mode** if no control commands are received, ensuring the vehicle stops safely when the high-level control program stops.

**Proper Shutdown Procedure**

To safely power off the vehicle:

1. Stop the high-level control program (i.e., the software publishing commands to `pwm_subscriber`).  
   *This step may be skipped if you are physically holding the vehicle.*
2. Turn the **physical power switch on the lower frame to OFF**.
3. Stop the `pwm_subscriber`.

**Controller Requirements**

For stable operation:

- Exactly **one `pwm_subscriber`** must be running.
- Exactly **one high-level control program** should publish commands to it.

Running multiple controllers simultaneously will result in **conflicting commands and unstable vehicle behavior**.

**Debuging**

During debugging, you may run the high-level control program independently to test sensors and algorithm through visulization and verify that the control commands sent through MQTT are reasonable.

### Before Run
Activate the virtual environment and navigate to the repository root:
```
source venv/bin/activate
cd DR-RPI
```

### Manual Control
To test whether PWM control works properly, connect a Logitech F310 controller to the Raspberry Pi.

Run the following commands in two separate terminals.

Terminal 1:
```
python script/pwm_subscriber.py
```
Terminal 2:
```
python utlis/joy_test.py
```
You should now be able to manually control the vehicle using the game controller.

If the MQTT server is configured correctly, manual control can also be performed remotely over the network.

### Autonomous Lane Following

This mode enables:

- Lane following
- Distance keeping
- Traffic light detection

Run the following commands in two terminals.

Terminal 1:
```
python script/pwm_subscriber.py
```
Terminal 2:
```
python -m script.lane_dist_light_ctrl --throttle 0.6
```
The `--throttle` parameter controls the maximum forward speed.

## Web UI (browser control panel)

The `webui` package lets you drive, calibrate, tune and run the autopilot from a browser on your laptop or phone, so you don't have to edit code or run scripts each time. It runs on the Pi and replaces the "high-level control program". `pwm_subscriber` still does the actual PWM output.

Install the one extra dependency:
```
pip install aiohttp
```
Start it from the repository root:
```
python -m webui
```
Then open `http://<pi-ip>:8080` on any device on the same network. The UI needs no internet access.

What each tab does:

| Tab | What it does |
| --- | --- |
| **Drive** | Manual driving with the on-screen joystick, keyboard (`WASD`/arrows) or a gamepad (Logitech F310 in **X** mode: RT throttle, LT reverse, stick steers, B = stop). Includes max-throttle/reverse limits. |
| **Autopilot** | Runs lane following + distance keeping + traffic light stop in-process, using the `src/` modules. Presets match `lane_dist_light_ctrl`, `lane_dist_ctrl` and `two_lane_dist_ctrl`. Throttle, Stanley gains, distance thresholds etc. can be changed live. While the tab is open it shows a dry-run preview, with live bird's-eye/traffic-light views and telemetry. |
| **Tuning** | Camera exposure/gain, plus HSV thresholds for the lane and traffic light colors. Click the camera image to sample a color and set a range from it. |
| **Calibrate** | Edit `~/pwm_config.json` (steering center/range, throttle neutral/start/max) live, with hold-to-test buttons. **Save** writes the file used by `pwm_subscriber` and `joy_test.py`. |
| **Logs** | Record video + per-frame CSV of commands/telemetry in any mode, then download or delete runs. |
| **System** | Start/stop `pwm_subscriber`, view its log, and see MQTT status. |

Safety behavior:
- The red **STOP** button (or `Space`, or gamepad B) is a latched E-stop. It sends stop commands and blocks all driving until it is reset.
- Only one browser can drive at a time. If it stops sending input for 0.3 s, closes the tab, or disconnects, the car stops. `pwm_subscriber` also idles after 0.5 s on its own.
- The UI warns if `pwm_subscriber` is not running, or if another program is also publishing to `car/cmd`. Don't run the control scripts while the UI is driving.
- In the autopilot, the car stops when the lane is lost (fewer than "min lane points").

Steering sign: `pwm_subscriber` maps `steer = +1` to a duty above center, while `joy_test.py` maps stick-right to a duty below center. Because of that, the UI sends "right" as negative `steer` by default. If the car steers the wrong way, untick **Invert steering** on the Drive tab.

`pwm_subscriber` additionally listens on `car/config` for live calibration and publishes a heartbeat on `car/status/pwm`. Its existing behavior and topics are unchanged.

Options: `python -m webui --help`. Useful ones:
- `--camera sim` runs a simulated track and car, so you can try the whole UI on a laptop without the Pi: `pip install opencv-python numpy aiohttp paho-mqtt`.
- `--camera 0` (USB webcam) or `--camera file.mp4` (replay a recording).
- `--port`, `--mqtt-host`.

To start on boot, see `deploy/dr-webui.service` (and optionally `deploy/dr-pwm-subscriber.service`). UI settings are stored in `~/dr_rpi_webui.json`.

## Other Usefual Commands
### Sync Code to Raspberry Pi
Quickly synchronize the repository from your development machine to the Raspberry Pi:
```
rsync -avz --exclude='.git' DR-RPI/ usr@{IP}:DR-RPI/
```
Replace `{IP}` with the IP address of your Raspberry Pi.
### Remove SSH Key for One Device
If an SSH key mismatch occurs when reconnecting to a device:
```
ssh-keygen -R {IP}
```